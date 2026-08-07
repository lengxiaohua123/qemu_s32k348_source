/*
 * NXP S32K3xx LPUART QEMU device model
 *
 * Functional LPUART model with Chardev backend, so any instance can be
 * exposed to the host as:
 *   -serial stdio / pty / tcp:... / chardev socket
 * On Linux a pty shows up as /dev/pts/N (usable by any terminal program);
 * a "usbserial"-like device is obtained by bridging with socat
 * (see README) or by passing a physical USB-UART adapter with
 *   -chardev serial,path=/dev/ttyUSB0
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/char/s32k3_lpuart.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/qdev-clock.h"
#include "chardev/char-fe.h"
#include "chardev/char-serial.h"
#include "qemu/log.h"
#include "qemu/module.h"

static void s32k3_lpuart_update_irq(S32K3LpuartState *s)
{
    bool level = false;

    if ((s->ctrl & CTRL_TIE)  && (s->stat & STAT_TDRE)) {
        level = true;
    }
    if ((s->ctrl & CTRL_TCIE) && (s->stat & STAT_TC)) {
        level = true;
    }
    if ((s->ctrl & CTRL_RIE)  && (s->stat & STAT_RDRF)) {
        level = true;
    }
    /* RXWATER：接收 FIFO 达到水位即请求中断（RIE） */
    if ((s->ctrl & CTRL_RIE) && (s->water & 0xFF) &&
        (s->rx_fifo_len >= (s->water & 0xFF))) {
        level = true;
    }
    qemu_set_irq(s->irq, level);
}

static void s32k3_lpuart_update_params(S32K3LpuartState *s)
{
    QEMUSerialSetParams ssp;
    uint32_t sbr = s->baud & BAUD_SBR_MASK;
    uint32_t osr = ((s->baud & BAUD_OSR_MASK) >> BAUD_OSR_SHIFT) + 1;
    uint64_t clk = clock_get_hz(s->module_clk);

    if (sbr == 0) {
        sbr = 1;
    }
    ssp.speed = clk / (osr * sbr);
    ssp.data_bits = (s->ctrl & CTRL_M) ? 9 : 8;
    ssp.parity = (s->ctrl & CTRL_PE) ? 'E' : 'N';
    ssp.stop_bits = 1;
    qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_SET_PARAMS, &ssp);
    /* 发送时序：每字符耗时 = (1 start + data + parity + stop) 位 / 波特率。
     * 用 ptimer_set_period 显式设纳秒周期，避免高频下 delta 优化出错。 */
    if (s->tx_timer && ssp.speed > 0) {
        uint64_t bits = 1 + (ssp.data_bits + (ssp.parity != 'N')) + ssp.stop_bits;
        int64_t period_ns = (int64_t)bits * 1000000000LL / ssp.speed;
        if (period_ns < 1) {
            period_ns = 1;
        }
        ptimer_transaction_begin(s->tx_timer);
        ptimer_set_period(s->tx_timer, period_ns);
        ptimer_set_count(s->tx_timer, 1);
        if (s->tx_busy) {
            ptimer_run(s->tx_timer, 1);
        }
        ptimer_transaction_commit(s->tx_timer);
    }
}

/* RX 输入泵：周期 tick 主动 accept_input，处理 -icount 下
 * chardev(socket) 输入积压延迟——否则固件等待下一帧时主循环
 * 不活跃，socket 数据迟迟不进模型，每帧耗时数十秒 */
static void s32k3_lpuart_rx_pump(void *opaque)
{
    S32K3LpuartState *s = opaque;

    qemu_chr_fe_accept_input(&s->chr);
    /* ptimer 回调已在事务上下文中，直接重调度 */
    ptimer_set_count(s->rx_pump_timer, 1);
    ptimer_run(s->rx_pump_timer, 1);
}

/* 发送完成：TDRE 按波特率时序恢复置位 */
static void s32k3_lpuart_tx_tick(void *opaque)
{
    S32K3LpuartState *s = opaque;

    s->stat |= STAT_TDRE | STAT_TC;
    s->tx_busy = false;
    s32k3_lpuart_update_irq(s);
}

static void s32k3_lpuart_reset(DeviceState *dev)
{
    S32K3LpuartState *s = S32K3_LPUART(dev);

    s->global = 0;
    s->pincfg = 0;
    s->baud   = 0x0F000004; /* OSR=16, SBR=4 -> reset default */
    s->stat   = STAT_TDRE | STAT_TC;
    s->ctrl   = 0;
    s->match  = 0;
    s->modir  = 0;
    s->fifo   = FIFO_TXEMPT | FIFO_RXEMPT | 0x00300000;
    s->water  = 0;
    s->rx_fifo_len = 0;
    s->tx_fifo_len = 0;
    s->tx_fifo_head = 0;
    s->tx_busy = false;
    /* 启动 RX 输入泵（10ms 虚拟周期） */
    ptimer_transaction_begin(s->rx_pump_timer);
    ptimer_set_period(s->rx_pump_timer, 10000000);
    ptimer_set_count(s->rx_pump_timer, 1);
    ptimer_run(s->rx_pump_timer, 1);
    ptimer_transaction_commit(s->rx_pump_timer);
    s->param  = (S32K3_LPUART_FIFO_DEPTH << PARAM_TXFIFO_SHIFT) |
                S32K3_LPUART_FIFO_DEPTH;
    s->verid  = VERID_FEATURE(1, 1);
    qemu_chr_fe_accept_input(&s->chr);
    s32k3_lpuart_update_params(s);
    s32k3_lpuart_update_irq(s);
}

static void s32k3_lpuart_rx_push(S32K3LpuartState *s, uint8_t c)
{
    if (!(s->ctrl & (CTRL_RE | (1 << 2)))) {
        return;
    }
    /* 地址匹配（BAUD[MAEN1]/[MAEN2] + MATCFG=00 地址匹配模式）：
     * 使能时只有匹配 MATCH[MA1]/[MA2] 的字符进入 FIFO */
    if (((s->baud & BAUD_MAEN1) || (s->baud & BAUD_MAEN2)) &&
        !((s->baud & BAUD_MATCFG_MASK) >> BAUD_MATCFG_SHIFT)) {
        bool match1 = (s->baud & BAUD_MAEN1) && (c == (s->match & 0xFF));
        bool match2 = (s->baud & BAUD_MAEN2) && (c == ((s->match >> 8) & 0xFF));
        if (!match1 && !match2) {
            return;   /* 非地址匹配字符丢弃 */
        }
    }
    if (s->rx_fifo_len >= S32K3_LPUART_FIFO_DEPTH) {
        s->stat |= STAT_OR;
        return;
    }
    s->rx_fifo[s->rx_fifo_len++] = c;
    s->stat |= STAT_RDRF;
    if (s->rx_fifo_len >= S32K3_LPUART_FIFO_DEPTH) {
        s->fifo &= ~FIFO_RXEMPT;
    }
    s32k3_lpuart_update_irq(s);
}

/* 回环（CTRL[LOOPS]=1）：发送字符回送到接收路径 */
static void s32k3_lpuart_loopback(S32K3LpuartState *s, uint8_t c)
{
    if (s->ctrl & CTRL_LOOPS) {
        s32k3_lpuart_rx_push(s, c);
    }
}

static int s32k3_lpuart_can_receive(void *opaque)
{
    S32K3LpuartState *s = opaque;

    if (!(s->ctrl & (CTRL_RE | (1 << 2)))) {
        return 0;
    }
    return S32K3_LPUART_FIFO_DEPTH - s->rx_fifo_len;
}

static void s32k3_lpuart_receive(void *opaque, const uint8_t *buf, int size)
{
    S32K3LpuartState *s = opaque;
    int i;

    for (i = 0; i < size; i++) {
        s32k3_lpuart_rx_push(s, buf[i]);
    }
}

static void s32k3_lpuart_event(void *opaque, QEMUChrEvent event)
{
    (void)opaque;
    (void)event;
}

static uint64_t s32k3_lpuart_read(void *opaque, hwaddr addr, unsigned size)
{
    S32K3LpuartState *s = opaque;
    uint32_t r = 0;

    switch (addr) {
    case LPUART_VERID:
        r = s->verid;
        break;
    case LPUART_PARAM:
        r = s->param;
        break;
    case LPUART_GLOBAL:
        r = s->global;
        break;
    case LPUART_PINCFG:
        r = s->pincfg;
        break;
    case LPUART_BAUD:
        r = s->baud;
        break;
    case LPUART_STAT:
        r = s->stat;
        break;
    case LPUART_CTRL:
        r = s->ctrl;
        break;
    case LPUART_DATA:
        if (s->rx_fifo_len > 0) {
            r = s->rx_fifo[0];
            memmove(s->rx_fifo, s->rx_fifo + 1, --s->rx_fifo_len);
            if (s->rx_fifo_len == 0) {
                s->stat &= ~STAT_RDRF;
                s->fifo |= FIFO_RXEMPT;
            }
            qemu_chr_fe_accept_input(&s->chr);
            s32k3_lpuart_update_irq(s);
        } else {
            r = 0;
        }
        break;
    case LPUART_MATCH:
        r = s->match;
        break;
    case LPUART_MODIR:
        r = s->modir;
        break;
    case LPUART_FIFO:
        r = s->fifo;
        break;
    case LPUART_WATER:
        r = s->water;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_lpuart: read of unimplemented reg 0x%02" HWADDR_PRIx "\n",
                      addr);
    }
    return r;
}

static void s32k3_lpuart_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned size)
{
    S32K3LpuartState *s = opaque;
    uint32_t v = value;

    switch (addr) {
    case LPUART_GLOBAL:
        s->global = v;
        if (v & GLOBAL_RST) {
            s32k3_lpuart_reset(DEVICE(s));
            s->global &= ~GLOBAL_RST;
        }
        break;
    case LPUART_PINCFG:
        s->pincfg = v & 0x3;
        break;
    case LPUART_BAUD:
        s->baud = v;
        s32k3_lpuart_update_params(s);
        break;
    case LPUART_STAT:
        /* W1C bits */
        s->stat &= ~(v & (STAT_IDLE | STAT_OR | STAT_NF | STAT_FE | STAT_PF));
        s32k3_lpuart_update_irq(s);
        break;
    case LPUART_CTRL:
        s->ctrl = v;
        s32k3_lpuart_update_params(s);
        s32k3_lpuart_update_irq(s);
        break;
    case LPUART_DATA:
        if (s->ctrl & (CTRL_TE | (1 << 3))) {
            uint8_t c = v & 0xff;
            /* 发送：数据即时写出（保持功能），TDRE 清位后由 ptimer
             * 按波特率时序重新置位（模拟发送一位所需时间）。 */
            if (qemu_chr_fe_backend_connected(&s->chr)) {
                qemu_chr_fe_write_all(&s->chr, &c, 1);
            }
            /* 回环模式（CTRL[LOOPS]=1）：发送字符回送接收 */
            s32k3_lpuart_loopback(s, c);
            s->stat &= ~(STAT_TDRE | STAT_TC);
            /* TDRE 经 1ns ptimer 快速恢复：避免固件 SyncSend 轮询 TDRE 在
             * -icount 下忙等超时（中断上下文回调内 SyncSend 尤其敏感）；
             * 1ns 周期同时保持 QEMU timer 活跃，驱动 chardev 输入处理 */
            ptimer_transaction_begin(s->tx_timer);
            ptimer_set_period(s->tx_timer, 1);
            ptimer_set_count(s->tx_timer, 1);
            ptimer_run(s->tx_timer, 1);
            ptimer_transaction_commit(s->tx_timer);
            s32k3_lpuart_update_irq(s);
        }
        break;
    case LPUART_MATCH:
        s->match = v & 0xffff;
        break;
    case LPUART_MODIR:
        s->modir = v;
        break;
    case LPUART_FIFO:
        /* W1C TXOF/RXUF, TXFLUSH/RXFLUSH self-clear */
        if (v & (1 << 15)) { /* TXFLUSH */
            s->fifo |= FIFO_TXEMPT;
            s->stat |= STAT_TDRE | STAT_TC;
        }
        if (v & (1 << 14)) { /* RXFLUSH */
            s->rx_fifo_len = 0;
            s->fifo |= FIFO_RXEMPT;
            s->stat &= ~STAT_RDRF;
        }
        s32k3_lpuart_update_irq(s);
        break;
    case LPUART_WATER:
        s->water = v & 0x00030003;
        break;
    case LPUART_VERID:
    case LPUART_PARAM:
        /* read-only */
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_lpuart: write of unimplemented reg 0x%02" HWADDR_PRIx
                      " = 0x%08" PRIx64 "\n", addr, value);
    }
}

static const MemoryRegionOps s32k3_lpuart_ops = {
    .read = s32k3_lpuart_read,
    .write = s32k3_lpuart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void s32k3_lpuart_init(Object *obj)
{
    S32K3LpuartState *s = S32K3_LPUART(obj);

    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);

    memory_region_init_io(&s->iomem, obj, &s32k3_lpuart_ops, s,
                          TYPE_S32K3_LPUART, S32K3_LPUART_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    s->tx_timer = ptimer_init(s32k3_lpuart_tx_tick, s,
                              PTIMER_POLICY_LEGACY);
    s->rx_pump_timer = ptimer_init(s32k3_lpuart_rx_pump, s,
                                   PTIMER_POLICY_LEGACY);
}

static void s32k3_lpuart_realize(DeviceState *dev, Error **errp)
{
    S32K3LpuartState *s = S32K3_LPUART(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_lpuart: module_clk must be connected");
        return;
    }

    qemu_chr_fe_set_handlers(&s->chr,
                             s32k3_lpuart_can_receive,
                             s32k3_lpuart_receive,
                             s32k3_lpuart_event,
                             NULL, s, NULL, true);

    s32k3_lpuart_reset(dev);
}

static const Property s32k3_lpuart_properties[] = {
    DEFINE_PROP_CHR("chardev", S32K3LpuartState, chr),
    DEFINE_PROP_BOOL("lpuart_type", S32K3LpuartState, lpuart_type, false),
};

static void s32k3_lpuart_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_lpuart_reset);
    dc->realize = s32k3_lpuart_realize;
    device_class_set_props(dc, s32k3_lpuart_properties);
    dc->desc = "NXP S32K3xx LPUART";
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
}

static const TypeInfo s32k3_lpuart_types[] = {
    {
        .name          = TYPE_S32K3_LPUART,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3LpuartState),
        .instance_init = s32k3_lpuart_init,
        .class_init    = s32k3_lpuart_class_init,
    },
};

DEFINE_TYPES(s32k3_lpuart_types)
