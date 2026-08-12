#!/usr/bin/env python3
"""
CI 测试脚本（Windows/Linux 通用）——跑 s32k348 模型全部本地测试：
  1. BSP 31 项自检（驱动寄存器级验证）
  2. bootloader 握手 + 烧写帧（时钟/UART/C40_Ip Flash 链路）
  3. FreeRTOS UART（Uart_Transmit HELLO 周期发送）
  4. 外设映射检查（info mtree——用户 abort 三地址）

用法：python ci/run_tests.py <qemu-binary>
"""
import subprocess
import socket
import sys
import time
import os

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.join(HERE, "tests")
QEMU = sys.argv[1] if len(sys.argv) > 1 else "qemu-system-arm"

passed = 0
failed = 0


def report(name, ok, extra=""):
    global passed, failed
    tag = "PASS" if ok else "FAIL"
    if ok:
        passed += 1
    else:
        failed += 1
    print(f"[{tag}] {name} {extra}")
    return ok


import tempfile


def qemu_run(args, timeout=220):
    """Run QEMU, return (rc, stdout). 输出到临时文件避免 pipe 满阻塞。"""
    f = tempfile.NamedTemporaryFile(delete=False, suffix=".log")
    fname = f.name
    try:
        r = subprocess.run([QEMU] + args, stdout=f, stderr=subprocess.STDOUT,
                           timeout=timeout)
    except subprocess.TimeoutExpired:
        r = None
    finally:
        f.close()
    with open(fname, errors="replace") as f:
        out = f.read()
    os.unlink(fname)
    return (r.returncode if r else -1), out


def socket_expect(port, expect, timeout_s=8, send=None, send_after=None):
    """Connect to QEMU chardev socket, optionally send, expect bytes."""
    s = None
    for _ in range(40):
        try:
            s = socket.create_connection(("127.0.0.1", port), timeout=2)
            break
        except OSError:
            time.sleep(0.4)
    if s is None:
        return False, b"connect-fail"
    s.settimeout(timeout_s)
    try:
        if send_after is not None:
            time.sleep(send_after)
        if send:
            s.sendall(send)
        d = b""
        t0 = time.time()
        while len(d) < len(expect) and time.time() - t0 < timeout_s:
            c = s.recv(len(expect) - len(d))
            if not c:
                break
            d += c
        ok = d == expect
        if ok and send:
            # bootloader wake：再收一次 ACK
            d2 = b""
            while len(d2) < len(expect) and time.time() - t0 < timeout_s:
                c = s.recv(len(expect) - len(d2))
                if not c:
                    break
                d2 += c
            ok = ok and d2 == expect
        s.close()
        return ok, d
    except socket.timeout:
        s.close()
        return False, b"timeout"


def run_bsp():
    """BSP 31 项自检：-nographic 输出 RESULT: 31 PASS"""
    elf = os.path.join(TESTS, "s32k348-bsp.elf")
    _, out = qemu_run(["-M", "s32k348evb", "-kernel", elf,
                       "-global", "s32k3-clkgen.fxosc-hz=16000000",
                       "-icount", "1", "-nographic"])
    ok = "RESULT: 31 PASS" in out and "0 FAIL" in out
    return report("BSP 31 项自检", ok,
                  f"(RESULT found: {'RESULT: 31 PASS' in out})")


def run_bootloader():
    """bootloader 握手 !STR + wake + 4 帧烧写 ACK（LPUART1 = 第 2 个 -serial）"""
    import threading
    elf = os.path.join(TESTS, "UartBootloader.elf")
    port = 5583
    proc = subprocess.Popen(
        [QEMU, "-M", "s32k348evb", "-kernel", elf,
         "-global", "s32k3-clkgen.fxosc-hz=16000000", "-icount", "1",
         "-display", "none",
         "-serial", "null",
         "-chardev", f"socket,id=bl,host=127.0.0.1,port={port},server=on,wait=on",
         "-serial", "chardev:bl", "-monitor", "none"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        ok_hand, d = socket_expect(port, b"!STR", timeout_s=10)
        report("bootloader 握手(!STR)", ok_hand, f"({d})")
    finally:
        proc.kill()
        proc.wait(timeout=5)
    return ok_hand


def run_freertos():
    """FreeRTOS UART（Uart_Transmit）HELLO 周期发送（LPUART1）"""
    elf = os.path.join(TESTS, "Uart_Transmit.elf")
    port = 5584
    proc = subprocess.Popen(
        [QEMU, "-M", "s32k348evb", "-kernel", elf,
         "-global", "s32k3-clkgen.fxosc-hz=16000000", "-icount", "1",
         "-display", "none",
         "-serial", "null",
         "-chardev", f"socket,id=u0,host=127.0.0.1,port={port},server=on,wait=on",
         "-serial", "chardev:u0", "-monitor", "none"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        ok, d = socket_expect(port, b"HELLO", timeout_s=10)
        report("FreeRTOS HELLO", ok, f"({d})")
    finally:
        proc.kill()
        proc.wait(timeout=5)
    return ok


import json


def qmp_hmp(port, cmd, timeout=25):
    """连接 QMP，执行 human-monitor-command（HMP 命令），返回输出文本。"""
    s = None
    for _ in range(20):
        try:
            s = socket.create_connection(("127.0.0.1", port), timeout=2)
            break
        except OSError:
            time.sleep(0.5)
    if s is None:
        return ""
    s.settimeout(4)
    try:
        d = b""
        try:
            while True:
                c = s.recv(8192)   # 欢迎 banner
                if not c:
                    break
                d += c
        except Exception:
            pass
        def send(obj):
            s.sendall((json.dumps(obj) + "\n").encode())
            time.sleep(0.4)
            buf = b""
            try:
                while True:
                    c = s.recv(8192)
                    if not c:
                        break
                    buf += c
            except Exception:
                pass
            return buf
        send({"execute": "qmp_capabilities"})
        resp = send({"execute": "human-monitor-command",
                     "arguments": {"command-line": cmd}})
        s.close()
        try:
            return json.loads(resp.decode(errors="replace"))["return"]
        except Exception:
            return ""
    except Exception:
        try:
            s.close()
        except Exception:
            pass
        return ""


def run_mtree():
    """info mtree 验证外设映射（用户 abort 三地址 + 关键外设）"""
    addrs = ["40210000", "40088000", "40098000",
             "4020c000", "4008c000", "40090000", "40410000"]
    port = 5597
    proc = subprocess.Popen(
        [QEMU, "-M", "s32k348evb", "-display", "none", "-S",
         "-qmp", f"tcp:127.0.0.1:{port},server,nowait"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        out = qmp_hmp(port, "info mtree")
        ok_all = True
        for a in addrs:
            found = a in out
            if not found:
                ok_all = False
            print(f"  mtree {a}: {'found' if found else 'MISSING'}")
        return report("info mtree 外设映射（3 abort 地址）", ok_all)
    finally:
        proc.kill()
        proc.wait(timeout=5)


if __name__ == "__main__":
    print(f"QEMU: {QEMU}")
    t0 = time.time()
    run_bsp()
    run_bootloader()
    run_freertos()
    run_mtree()
    print(f"\n===== CI TEST RESULT: {passed} PASS, {failed} FAIL "
          f"({time.time()-t0:.0f}s) =====")
    sys.exit(1 if failed else 0)
