/*
 * QEMU S32K344EVB Board Implementation
 * Based on NXP S32K3xx Data Sheet Rev. 14 (April 2026)
 * Target: BCOM Motor Controller
 *
 * Functional peripherals (all visible/usable from the host):
 *   - 16x LPUART   (chardev backend: stdio/pty/tcp/usbserial adapter)
 *   - 6x  LPSPI    (QEMU SSI bus: attach ssi-loopback / spi-flash etc.)
 *   - 8x  FlexCAN  CAN 2.0B / CAN FD (QEMU CAN bus -> host SocketCAN /
 *                  TCP bridge to CAN tools on Windows)
 *   - SIUL2 GPIO   (512 qdev gpio lines, external-interrupt capable)
 *   - EMAC         (netdev backend: tap / socket / user)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/arm/s32k344evb.h"
/* s32k3_emios: 结构在 hw/timer/s32k3_emios.c 内（pwm-dump 经 QOM property 访问） */
#include "hw/misc/s32k3_clkgen.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/split-irq.h"
#include "hw/core/irq.h"
#include "hw/misc/unimp.h"
#include "hw/core/loader.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "target/arm/cpu-qom.h"
#include "target/arm/cpu.h"
#include "system/system.h"
#include "system/reset.h"
#include "system/address-spaces.h"
#include "net/can_emu.h"
#include "hw/usb/usb.h"
#include "hw/usb/hcd-ehci.h"

/* ------------------------------------------------------------------
 *  Debug macro
 * ------------------------------------------------------------------ */
#define DB_PRINT(fmt, ...)                                                  \
    do {                                                                    \
        if (S32K344EVB_DEBUG) {                                             \
            qemu_log("S32K344EVB: " fmt "\n", ##__VA_ARGS__);               \
        }                                                                   \
    } while (0)

static bool S32K344EVB_DEBUG = false;

/* CM7_1 复位后保持 halted（单核固件不释放从核） */
static void s32k344_halt_cm7_1(void *opaque)
{
    CPUState *cpu1 = qemu_get_cpu(1);

    if (cpu1) {
        cpu1->halted = true;
    }
}

static void s32k344evb_board_init(MachineState *machine);

/* ------------------------------------------------------------------
 *  Memory Region Helpers
 * ------------------------------------------------------------------ */
static void s32k348_create_ram_region(S32K344EVBMachineState *s,
                                      const char *name,
                                      hwaddr base, uint64_t size)
{
    MemoryRegion *ram = g_new(MemoryRegion, 1);
    memory_region_init_ram(ram, NULL, name, size, &error_fatal);
    memory_region_add_subregion(s->system_memory, base, ram);
    DB_PRINT("RAM region '%s' created: 0x%08" PRIx64 " - 0x%08" PRIx64,
             name, (uint64_t)base, (uint64_t)(base + size - 1));
}

/* ------------------------------------------------------------------
 *  LPUART x16 (functional)
 * ------------------------------------------------------------------ */
static void s32k348_uart_board_init(S32K344EVBMachineState *s)
{
    static const uint8_t lpuart_irq[S32K344_LPUART_INSTANCES] = {
        S32K344_IRQ_LPUART0,  S32K344_IRQ_LPUART1,  S32K344_IRQ_LPUART2,
        S32K344_IRQ_LPUART3,  S32K344_IRQ_LPUART4,  S32K344_IRQ_LPUART5,
        S32K344_IRQ_LPUART6,  S32K344_IRQ_LPUART7,  S32K344_IRQ_LPUART8,
        S32K344_IRQ_LPUART9,  S32K344_IRQ_LPUART10, S32K344_IRQ_LPUART11,
        S32K344_IRQ_LPUART12, S32K344_IRQ_LPUART13, S32K344_IRQ_LPUART14,
        S32K344_IRQ_LPUART15,
    };
    int i;

    for (i = 0; i < S32K344_LPUART_INSTANCES; i++) {
        DeviceState *dev = qdev_new(TYPE_S32K3_LPUART);
        SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
        Chardev *chr = serial_hd(i);
        Clock *uart_clk;
        hwaddr uart_base;

        s->uart[i] = S32K3_LPUART(dev);
        if (chr != NULL) {
            qdev_prop_set_chr(dev, "chardev", chr);
        }

        uart_clk = (i < 2 || i == 8) ? s->aips_plat_clk : s->aips_slow_clk;
        qdev_connect_clock_in(dev, "module_clk", uart_clk);
        qdev_prop_set_bit(dev, "lpuart_type", (i < 2) ? true : false);

        sysbus_realize(sbd, &error_fatal);

        if (i < 8) {
            uart_base = S32K344_LPUART0_BASE + S32K344_LPUART_OFFSET * i;
        } else {
            uart_base = S32K344_LPUART8_BASE + S32K344_LPUART_OFFSET * (i - 8);
        }
        sysbus_mmio_map(sbd, 0, uart_base);
        sysbus_connect_irq(sbd, 0,
                           qdev_get_gpio_in(DEVICE(&s->armv7m), lpuart_irq[i]));

        DB_PRINT("LPUART%d mapped @ 0x%08" PRIx64 " irq %d",
                 i, (uint64_t)uart_base, lpuart_irq[i]);
    }
}

/* ------------------------------------------------------------------
 *  LPSPI x6 (functional, SSI bus named "spi<N>")
 * ------------------------------------------------------------------ */
static void s32k348_spi_board_init(S32K344EVBMachineState *s)
{
    static const hwaddr spi_base[S32K344_LPSPI_INSTANCES] = {
        S32K344_LPSPI0_BASE, S32K344_LPSPI1_BASE, S32K344_LPSPI2_BASE,
        S32K344_LPSPI3_BASE, S32K344_LPSPI4_BASE, S32K344_LPSPI5_BASE,
    };
    static const uint8_t spi_irq[S32K344_LPSPI_INSTANCES] = {
        S32K344_IRQ_LPSPI0, S32K344_IRQ_LPSPI1, S32K344_IRQ_LPSPI2,
        S32K344_IRQ_LPSPI3, S32K344_IRQ_LPSPI4, S32K344_IRQ_LPSPI5,
    };
    int i;

    for (i = 0; i < S32K344_LPSPI_INSTANCES; i++) {
        DeviceState *dev = qdev_new(TYPE_S32K3_LPSPI);
        SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

        s->spi[i] = S32K3_LPSPI(dev);
        qdev_connect_clock_in(dev, "module_clk", s->aips_slow_clk);
        sysbus_realize(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, spi_base[i]);
        sysbus_connect_irq(sbd, 0,
                           qdev_get_gpio_in(DEVICE(&s->armv7m), spi_irq[i]));

        DB_PRINT("LPSPI%d mapped @ 0x%08" PRIx64 " irq %d",
                 i, (uint64_t)spi_base[i], spi_irq[i]);

        if (i == 1) {
            /* LPSPI1：挂 GD3000 (MC33937) 预驱动从设备（BLDC 电机控制） */
            BusState *spi_bus = qdev_get_child_bus(dev, "spi");
            ssi_create_peripheral((SSIBus *)spi_bus, "s32k3-gd3000");
            DB_PRINT("GD3000 attached to LPSPI1");
        }
    }
}

/* ------------------------------------------------------------------
 *  FlexCAN x8 (functional CAN/CAN-FD; attach to host CAN buses)
 *
 *  Host side: create can-bus objects on the QEMU command line; the board
 *  wires FlexCAN0->canbus0 ... FlexCAN7->canbus7 when they exist:
 *
 *    -object can-bus,id=canbus0 \
 *    -object can-host-socketcan,if=can0,canbus=canbus0   (Linux)
 *
 *  or TCP bridge (works on Windows too):
 *    -object can-bus,id=canbus0 \
 *    -chardev socket,id=cansock0,host=127.0.0.1,port=29536,server=on,wait=off \
 *    -object can-host-socket,id=canhost0,canbus=canbus0,chardev=cansock0
 * ------------------------------------------------------------------ */
static void s32k348_can_board_init(S32K344EVBMachineState *s)
{
    static const hwaddr can_base[S32K344_FLEXCAN_INSTANCES] = {
        S32K344_FLEXCAN0_BASE, S32K344_FLEXCAN1_BASE,
        S32K344_FLEXCAN2_BASE, S32K344_FLEXCAN3_BASE,
        S32K344_FLEXCAN4_BASE, S32K344_FLEXCAN5_BASE,
        S32K344_FLEXCAN6_BASE, S32K344_FLEXCAN7_BASE,
    };
    static const uint8_t can_irq[S32K344_FLEXCAN_INSTANCES] = {
        S32K344_IRQ_FLEXCAN0, S32K344_IRQ_FLEXCAN1,
        S32K344_IRQ_FLEXCAN2, S32K344_IRQ_FLEXCAN3,
        S32K344_IRQ_FLEXCAN4, S32K344_IRQ_FLEXCAN5,
        S32K344_IRQ_FLEXCAN6, S32K344_IRQ_FLEXCAN7,
    };
    int i;

    for (i = 0; i < S32K344_FLEXCAN_INSTANCES; i++) {
        DeviceState *dev = qdev_new(TYPE_S32K3_FLEXCAN);
        SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
        char busname[16];
        Object *canbus_obj;

        s->can[i] = S32K3_FLEXCAN(dev);
        qdev_connect_clock_in(dev, "module_clk", s->aips_slow_clk);

        /* attach to host canbus<N> if the user created one */
        snprintf(busname, sizeof(busname), "canbus%d", i);
        canbus_obj = object_resolve_path_component(
            object_get_objects_root(), busname);
        if (canbus_obj) {
            object_property_set_link(OBJECT(dev), "canbus",
                                     canbus_obj, &error_fatal);
        } else if (i == 0) {
            /* default: private loopback bus so CAN works standalone.
             * CanBusState is a plain Object, not a Device.
             * NB: object_property_set_link() resolves the target's
             * canonical path, so the bus must be attached to a parent
             * (object_new() alone leaves it parentless and the link
             * silently fails in QEMU >= 8). */
            Object *bus = object_new(TYPE_CAN_BUS);
            object_property_add_child(OBJECT(s), "loopback-canbus0",
                                      bus);
            object_unref(bus);
            object_property_set_link(OBJECT(dev), "canbus",
                                     bus, &error_fatal);
        }

        sysbus_realize(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, can_base[i]);
        sysbus_connect_irq(sbd, 0,
                           qdev_get_gpio_in(DEVICE(&s->armv7m), can_irq[i]));

        DB_PRINT("FlexCAN%d mapped @ 0x%08" PRIx64 " irq %d bus %s",
                 i, (uint64_t)can_base[i], can_irq[i],
                 canbus_obj ? busname : (i == 0 ? "(loopback)" : "(none)"));
    }
}

/* ------------------------------------------------------------------
 *  TEMPSENSE (temperature sensor calibration @ 0x4037C000)
 *  TCA0/1/2 出厂校准系数；模型用 25C 典型值（TCA0=25<<4 定点）。
 * ------------------------------------------------------------------ */
static uint64_t s32k348_tempsense_read(void *opaque, hwaddr offset,
                                       unsigned size)
{
    switch (offset) {
    case 0x08: return 25u << 4;   /* TCA0 */
    case 0x0C: return 0;          /* TCA1 */
    case 0x10: return 0;          /* TCA2 */
    default: return 0;
    }
}

static void s32k348_tempsense_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
    /* TCA 出厂校准只读 */
}

static const MemoryRegionOps s32k348_tempsense_ops = {
    .read = s32k348_tempsense_read,
    .write = s32k348_tempsense_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void s32k348_tempsense_init(S32K344EVBMachineState *s)
{
    MemoryRegion *ts = g_new0(MemoryRegion, 1);

    memory_region_init_io(ts, NULL, &s32k348_tempsense_ops, NULL,
                          "s32k348.tempsense", 0x1000);
    memory_region_add_subregion(s->system_memory, 0x4037C000, ts);
}

/* ------------------------------------------------------------------
 *  SIUL2 GPIO (functional)
 * ------------------------------------------------------------------ */
/* 外部信号注入：qom-set /machine inject-ext-irq <pin>（0-7 = EIRQ0-7） */
/* timer 到期拉低：保持高电平跨 IFER 滤波两拍，确认上升沿 */
static void s32k348_inject_ext_irq_fall(void *opaque)
{
    S32K344EVBMachineState *s = opaque;

    if (s->inject_last_irq) {
        qemu_set_irq(s->inject_last_irq, 0);
        s->inject_last_irq = NULL;
    }
}

static void s32k348_inject_ext_irq_set(Object *obj, Visitor *v,
                                       const char *name, void *opaque,
                                       Error **errp)
{
    S32K344EVBMachineState *s = S32K344EVB_MACHINE(obj);
    uint32_t pin = 0;

    if (!visit_type_uint32(v, name, &pin, errp)) {
        return;
    }
    if (pin >= 8 || !s->siul2) {
        error_setg(errp, "inject-ext-irq: pin must be 0-7");
        return;
    }
    /* 拉高并保持 20us（SIUL2 滤波需两拍同电平确认沿） */
    qemu_irq in = qdev_get_gpio_in_named(DEVICE(s->siul2), "gpio-in", pin);
    qemu_set_irq(in, 1);
    s->inject_last_irq = in;
    timer_mod_ns(s->inject_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20000);
}

static void s32k348_inject_ext_irq_get(Object *obj, Visitor *v,
                                       const char *name, void *opaque,
                                       Error **errp)
{
    uint32_t val = 0;

    visit_type_uint32(v, name, &val, errp);
}

/* FlexCAN 注入：qom-set /machine inject-can "0x5165:!WAKEAPP"
 * （id 后跟 ':' + 最多 8 字节数据）。走模型接收路径，等价总线收帧。 */
static void s32k348_inject_can_set(Object *obj, const char *value, Error **errp)
{
    S32K344EVBMachineState *s = S32K344EVB_MACHINE(obj);
    uint32_t id;
    const char *d = NULL;
    uint8_t data[8] = { 0 };
    int dlc = 0;

    id = (uint32_t)strtoul(value, (char **)&d, 0);
    if (d && *d == ':') {
        d++;
        dlc = strlen(d);
        if (dlc > 8) {
            dlc = 8;
        }
        memcpy(data, d, dlc);
    }
    if (s->can[0]) {
        s32k3_flexcan_inject(s->can[0], id, data, dlc);
    }
}

/* ADC 模拟输入注入：qom-set /machine inject-adc-full <ch>（ADC2 通道，
 * 固件 TempSense 用 ADC2 ch0）。拉高 adc-in[ch] → ain=65535 →
 * 转换结果 0xFFF（满量程），验证固件读到的 ADC 采集值。 */
static void s32k348_inject_adc_full_set(Object *obj, Visitor *v,
                                        const char *name, void *opaque,
                                        Error **errp)
{
    S32K344EVBMachineState *s = S32K344EVB_MACHINE(obj);
    uint32_t ch = 0;

    if (!visit_type_uint32(v, name, &ch, errp)) {
        return;
    }
    if (ch >= 32 || !s->adc[2]) {
        error_setg(errp, "inject-adc-full: channel must be 0-31");
        return;
    }
    qemu_set_irq(qdev_get_gpio_in_named(s->adc[2], "adc-in", ch), 1);
}

/* PWM 占空比统计打印：qom-set /machine pwm-dump 0 */
static void s32k348_pwm_dump_set(Object *obj, Visitor *v,
                                 const char *name, void *opaque,
                                 Error **errp)
{
    S32K344EVBMachineState *s = S32K344EVB_MACHINE(obj);
    uint32_t dummy = 0;
    int inst;

    visit_type_uint32(v, name, &dummy, errp);
    for (inst = 0; inst < 3; inst++) {
        if (s->emios[inst]) {
            fprintf(stderr, "--- eMIOS%d PWM ---\n", inst);
            object_property_set_uint(OBJECT(s->emios[inst]), "pwm-dump",
                                     0, errp);
        }
    }
}

/* 霍尔信号注入：qom-set /machine inject-emios-edge <ch>（eMIOS1 通道）
 * 先拉高、timer 到期拉低 → 下降沿，触发 SAIC 输入捕获（EDPOL=1 时）。 */
static void s32k348_inject_emios_edge_set(Object *obj, Visitor *v,
                                          const char *name, void *opaque,
                                          Error **errp)
{
    S32K344EVBMachineState *s = S32K344EVB_MACHINE(obj);
    uint32_t ch = 0;

    if (!visit_type_uint32(v, name, &ch, errp)) {
        return;
    }
    if (ch >= 24 || !s->emios[1]) {
        error_setg(errp, "inject-emios-edge: channel must be 0-23");
        return;
    }
    qemu_irq in = qdev_get_gpio_in_named(s->emios[1], "input", ch);
    qemu_set_irq(in, 1);
    s->inject_last_irq = in;
    timer_mod_ns(s->inject_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 10000);
}

static void s32k348_siul2_board_init(S32K344EVBMachineState *s)
{
    DeviceState *dev = qdev_new(TYPE_S32K3_SIUL2);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    s->siul2 = S32K3_SIUL2(dev);
    qdev_connect_clock_in(dev, "module_clk", s->aips_plat_clk);
    sysbus_realize(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, S32K344_SIUL2_BASE);
    sysbus_connect_irq(sbd, 0,
                       qdev_get_gpio_in(DEVICE(&s->armv7m),
                                        S32K344_IRQ_SIUL2_EIRQ0));
    for (int i = 1; i < 4; i++) {
        sysbus_connect_irq(sbd, i,
                           qdev_get_gpio_in(DEVICE(&s->armv7m),
                                            S32K344_IRQ_SIUL2_EIRQ0 + i));
    }
    DB_PRINT("SIUL2 mapped @ 0x%08x irq %d",
             S32K344_SIUL2_BASE, S32K344_IRQ_SIUL2_EIRQ0);

    /*
     * 外部信号注入接口：HMP/QMP `qom-set /machine inject-ext-irq <pin>`
     * 对 SIUL2 的 gpio-in 输入产生一个带滤波保持的上升沿，触发 EIRQ 外部中断
     * （固件须已配置 DIRER0/IREER0 对应 pin，如 BLDC 固件的 EIRQ7）。
     */
    s->inject_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                   s32k348_inject_ext_irq_fall, s);
    object_property_add(OBJECT(s), "inject-ext-irq", "uint32",
                        s32k348_inject_ext_irq_get,
                        s32k348_inject_ext_irq_set, NULL, NULL);
    object_property_add(OBJECT(s), "inject-emios-edge", "uint32",
                        s32k348_inject_ext_irq_get,
                        s32k348_inject_emios_edge_set, NULL, NULL);
    object_property_add(OBJECT(s), "pwm-dump", "uint32",
                        s32k348_inject_ext_irq_get,
                        s32k348_pwm_dump_set, NULL, NULL);
    object_property_add(OBJECT(s), "inject-adc-full", "uint32",
                        s32k348_inject_ext_irq_get,
                        s32k348_inject_adc_full_set, NULL, NULL);
    object_property_add_str(OBJECT(s), "inject-can",
                            NULL, s32k348_inject_can_set);

    s32k348_tempsense_init(s);
}

/* ------------------------------------------------------------------
 *  EMAC (functional, netdev backend)
 * ------------------------------------------------------------------ */
static void s32k348_emac_board_init(S32K344EVBMachineState *s)
{
    DeviceState *dev = qdev_new(TYPE_S32K3_EMAC);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    s->emac = S32K3_EMAC(dev);
    qdev_connect_clock_in(dev, "module_clk", s->aips_plat_clk);
    /* QEMU 11: qemu_check_nic_model() is gone; use qemu_configure_nic_device(). */
    qemu_configure_nic_device(dev, true, NULL);
    sysbus_realize(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, S32K344_EMAC_BASE);
    sysbus_connect_irq(sbd, 0,
                       qdev_get_gpio_in(DEVICE(&s->armv7m), S32K344_IRQ_EMAC));
    DB_PRINT("EMAC mapped @ 0x%08x irq %d",
             S32K344_EMAC_BASE, S32K344_IRQ_EMAC);
}

/* ------------------------------------------------------------------
 *  eDMA (native S32K3xx model, 32 channels)
 * ------------------------------------------------------------------ */
static void s32k348_dma_board_init(S32K344EVBMachineState *s)
{
    DeviceState *dev = qdev_new(TYPE_S32K3_EDMA);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    int i;

    s->dma = S32K3_EDMA(dev);
    qdev_connect_clock_in(dev, "module_clk", s->aips_slow_clk);
    object_property_set_link(OBJECT(s->dma), "memory",
                             OBJECT(s->system_memory), &error_fatal);
    sysbus_realize(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, S32K344_EDMA_BASE);
    sysbus_mmio_map(sbd, 1, S32K344_EDMA_TCD1_BASE);
    sysbus_mmio_map(sbd, 2, S32K344_EDMA_TCD2_BASE);

    /* channel interrupts 0-31 */
    for (i = 0; i < S32K3_EDMA_CHANNELS; i++) {
        sysbus_connect_irq(sbd, i,
                           qdev_get_gpio_in(DEVICE(&s->armv7m),
                                            S32K344_IRQ_EDMA_CH0 + i));
    }
    /* 手册：eDMA 完成/错误共用通道中断（IRQ 4-35），无独立 error 线。
     * 模型 err_irq（index 32）不接 NVIC。 */

    DB_PRINT("eDMA mapped @ 0x%08x (regs), 0x%08x (TCD1), 0x%08x (TCD2)",
             S32K344_EDMA_BASE, S32K344_EDMA_TCD1_BASE, S32K344_EDMA_TCD2_BASE);
}

/* ------------------------------------------------------------------
 *  Main Board Init
 * ------------------------------------------------------------------ */
static void s32k344evb_board_init(MachineState *machine)
{
    S32K344EVBMachineState *s = S32K344EVB_MACHINE(machine);

    DB_PRINT("Initializing S32K344EVB board...");

    /* 1. System Memory Bus */
    s->system_memory = get_system_memory();

    /* 2. TCM */
    s32k348_create_ram_region(s, "s32k348.itcm",
                              S32K344_ITCM_BASE, S32K344_ITCM_SIZE);
    s32k348_create_ram_region(s, "s32k348.dtcm",
                              S32K344_DTCM_BASE, S32K344_DTCM_SIZE);

    /* 3. SRAM */
    const hwaddr sram_bases[S32K344_SRAM_BLOCKS] = {
        S32K344_SRAM0_BASE, S32K344_SRAM1_BASE, S32K344_SRAM2_BASE,
    };
    for (int i = 0; i < S32K344_SRAM_BLOCKS; i++) {
        char name[32];
        snprintf(name, sizeof(name), "s32k348.sram%d", i);
        s32k348_create_ram_region(s, name,
                                  sram_bases[i], S32K344_SRAM_BLK_SIZE);
    }

    /*
     * 4. Code Flash
     *
     * Modeled as RAM-backed regions so that the CMSIS-DAP debug probe
     * (and -kernel loading) can program them.  Attach -pflash/-blockdev
     * for persistence across runs if needed.
     */
    const hwaddr flash_bases[S32K344_FLASH_BLOCKS] = {
        S32K344_FLASH0_BASE, S32K344_FLASH1_BASE,
        S32K344_FLASH2_BASE, S32K344_FLASH3_BASE,
    };
    for (int i = 0; i < S32K344_FLASH_BLOCKS; i++) {
        char name[32];
        snprintf(name, sizeof(name), "s32k348.flash%d", i);
        s32k348_create_ram_region(s, name,
                                  flash_bases[i], S32K344_FLASH_BLK_SIZE);
    }

    /* 5. Data Flash（RAM：可读写擦除，Flash 命令经 c40asf 路由） */
    s32k348_create_ram_region(s, "s32k348.dataflash",
                              S32K344_DATA_FLASH_BASE,
                              S32K344_DATA_FLASH_SIZE);

    /* 6. UTEST / TestNVM（RAM：可读写擦除） */
    s32k348_create_ram_region(s, "s32k348.utest",
                              S32K344_UTEST_BASE, S32K344_UTEST_SIZE);

    /* 7. Clock tree (created first: armv7m requires cpuclk in QEMU 11).
     *
     * 真实时钟树（RM 第 24-30 章）：
     *   FIRC(48MHz) ──┬──> PLL 参考(REFCLKSEL=1) ──> PLL_PHI0
     *   FXOSC(8MHz) ─┴──> PLL 参考(REFCLKSEL=0) ──┘
     *   MUX_0 选 FIRC_CLK 或 PLL_PHI0_CLK，经 DC_0..2 分频出
     *   CORE_CLK / AIPS_PLAT_CLK / AIPS_SLOW_CLK。
     */
    {
        DeviceState *fxosc, *pll, *cgm;

        /* FIRC 48 MHz：用无 MMIO 的常量时钟表示 */
        s->firc_clk = clock_new(OBJECT(s), "firc");
        clock_set_hz(s->firc_clk, 48000000);

        /* SIRC 32 KHz：SWT 等慢速时钟源 */
        s->sirc_clk = clock_new(OBJECT(s), "sirc");
        clock_set_hz(s->sirc_clk, 32000);

        /* FXOSC：数字控制器 + 可配频率（-global s32k3-clkgen.fxosc-hz=N，
         * 默认 8MHz）。CTRL[OSCON] 使能 -> STATUS[OSC_STAT]。 */
        fxosc = qdev_new(TYPE_S32K3_CLKGEN);
        qdev_prop_set_uint32(fxosc, "kind", CLKGEN_KIND_FXOSC);
        sysbus_realize(SYS_BUS_DEVICE(fxosc), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(fxosc), 0, S32K344_FXOSC_BASE);
        s->fxosc_clk = clock_new(OBJECT(s), "fxosc");
        clock_set_hz(s->fxosc_clk,
                     object_property_get_uint(OBJECT(fxosc), "fxosc-hz",
                                              &error_abort));

        /* SXOSC：慢速外部振荡器（32 KHz）@ 0x402CC000 */
        {
            DeviceState *sxosc = qdev_new(TYPE_S32K3_CLKGEN);
            qdev_prop_set_uint32(sxosc, "kind", CLKGEN_KIND_SXOSC);
            sysbus_realize(SYS_BUS_DEVICE(sxosc), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(sxosc), 0, 0x402CC000);
        }

        /* PLL (PLLDIG)：参考 = FXOSC(0) 或 FIRC_DIV2(1) */
        pll = qdev_new(TYPE_S32K3_CLKGEN);
        qdev_prop_set_uint32(pll, "kind", CLKGEN_KIND_PLL);
        qdev_connect_clock_in(pll, "fxosc-clk", s->fxosc_clk);
        qdev_connect_clock_in(pll, "firc-clk", s->firc_clk);
        sysbus_realize(SYS_BUS_DEVICE(pll), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(pll), 0, S32K344_PLL_BASE);

        /* MC_CGM：输出三条生成时钟 */
        cgm = qdev_new(TYPE_S32K3_CLKGEN);
        qdev_prop_set_uint32(cgm, "kind", CLKGEN_KIND_MC_CGM);
        qdev_connect_clock_in(cgm, "firc-clk", s->firc_clk);
        qdev_connect_clock_in(cgm, "pll-in-clk",
                              qdev_get_clock_out(pll, "pll-clk"));
        sysbus_realize(SYS_BUS_DEVICE(cgm), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(cgm), 0, S32K344_MCCGM_BASE);
        s->sysclk = qdev_get_clock_out(cgm, S32K3_CLKGEN_CLK_SYSCLK);
        s->aips_plat_clk = qdev_get_clock_out(cgm,
                                              S32K3_CLKGEN_CLK_AIPS_PLAT);
        s->aips_slow_clk = qdev_get_clock_out(cgm,
                                              S32K3_CLKGEN_CLK_AIPS_SLOW);
        s->clkgen = cgm;
    }

    /* 8. Cortex-M7 CPU */
    object_initialize_child(OBJECT(s), "armv7m", &s->armv7m, TYPE_ARMV7M);
    /* S32K3x reset with VTOR pointing at code flash (0x400000) */
    s->armv7m.init_svtor = S32K344_FLASH0_BASE;
    s->armv7m.init_nsvtor = S32K344_FLASH0_BASE;
    qdev_prop_set_uint32(DEVICE(&s->armv7m), "num-irq", 256);
    qdev_prop_set_string(DEVICE(&s->armv7m), "cpu-type",
                         ARM_CPU_TYPE_NAME("cortex-m7"));
    /* S32K3 的 Cortex-M7 有 16 个 MPU region（QEMU cortex-m7 默认 8）。
     * RTD 固件配置 15 个 region，缺省会漏掉 AIPS region 导致外设访问
     * 被 MPU 背景区拒绝（DACCVIOL）。 */
    qdev_prop_set_uint32(DEVICE(&s->armv7m), "mpu-ns-regions", 16);
    object_property_set_link(OBJECT(&s->armv7m), "memory",
                             OBJECT(s->system_memory), &error_abort);
    qdev_connect_clock_in(DEVICE(&s->armv7m), "cpuclk", s->sysclk);
    sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), &error_fatal);

    /* 8b. CM7_1（从核）：S32K344 双核。共享同一 system_memory（外设/
     * RAM），复位后保持（真实 S32K3 由 CM7_0 写 MSCM 释放；单核固件
     * 不释放——CM7_1 一直停在复位）。外设中断默认路由到 CM7_0。 */
    qemu_register_reset(s32k344_halt_cm7_1, NULL);
    object_initialize_child(OBJECT(s), "armv7m1", &s->armv7m_1, TYPE_ARMV7M);
    s->armv7m_1.init_svtor = S32K344_FLASH0_BASE;
    s->armv7m_1.init_nsvtor = S32K344_FLASH0_BASE;
    qdev_prop_set_uint32(DEVICE(&s->armv7m_1), "num-irq", 256);
    qdev_prop_set_string(DEVICE(&s->armv7m_1), "cpu-type",
                         ARM_CPU_TYPE_NAME("cortex-m7"));
    qdev_prop_set_uint32(DEVICE(&s->armv7m_1), "mpu-ns-regions", 16);
    /* CM7_1 不能把 system_memory 直接作 board_memory（CM7_0 已加为
     * 其 container 子区域，重复 add 会断言失败）。用 alias 共享： */
    s->sys_alias = g_new0(MemoryRegion, 1);
    memory_region_init_alias(s->sys_alias, OBJECT(s), "s32k344-sys-alias",
                             s->system_memory, 0, ~(hwaddr)0);
    object_property_set_link(OBJECT(&s->armv7m_1), "memory",
                             OBJECT(s->sys_alias), &error_abort);
    qdev_connect_clock_in(DEVICE(&s->armv7m_1), "cpuclk", s->sysclk);
    sysbus_realize(SYS_BUS_DEVICE(&s->armv7m_1), &error_fatal);

    /* 9. Functional peripherals */
    s32k348_uart_board_init(s);
    s32k348_spi_board_init(s);
    s32k348_can_board_init(s);
    s32k348_siul2_board_init(s);
    s32k348_emac_board_init(s);
    s32k348_dma_board_init(s);

    /*
     * 9.7 USB host controller (EHCI)
     *
     * 注：S32K348 手册无 USB 外设（S32K3xx 系列无 USB），此 EHCI 为
     * 调试探针（s32k3-dap）虚拟总线，非真实芯片外设。保留但标注。
     * 挂载在 AIPS0 空闲槽位（非手册地址）。
     */
    {
        DeviceState *ehci = qdev_new(TYPE_PLATFORM_EHCI);
        sysbus_realize(SYS_BUS_DEVICE(ehci), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(ehci), 0, 0x40064000);
        DB_PRINT("USB EHCI (debug probe, NOT a real S32K348 peripheral) "
                 "mapped @ 0x40064000");
    }

    /* 10. Timers / ADC / I2C (functional models) */
    {
        /* PIT x3: 4 channels each on AIPS_SLOW_CLK (60 MHz).
         * S32K348 有 PIT0/1/2 三实例；每实例 4 通道共用一条 NVIC 线
         * （PIT0=IRQ96, PIT1=IRQ97, PIT2=IRQ98）。 */
        static const hwaddr pit_base[3] = {
            S32K344_PIT0_BASE, S32K344_PIT1_BASE, S32K344_PIT2_BASE,
        };
        static const uint8_t pit_irq[3] = {
            S32K344_IRQ_PIT0, S32K344_IRQ_PIT1, S32K344_IRQ_PIT2,
        };
        DeviceState *pit;
        int i;

        pit = qdev_new("s32k3-pit");
        s->pit = pit;
        qdev_connect_clock_in(pit, "module_clk", s->aips_slow_clk);
        sysbus_realize(SYS_BUS_DEVICE(pit), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(pit), 0, pit_base[0]);
        /* PIT0 ch0 经 split-irq 分扇到 NVIC + BCTU（见下），ch1..ch3 直连 */
        for (i = 1; i < 4; i++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(pit), i,
                               qdev_get_gpio_in(DEVICE(&s->armv7m),
                                                pit_irq[0]));
        }

        /* PIT1/PIT2：4 通道全部直连各自 NVIC 线 */
        for (i = 1; i < 3; i++) {
            DeviceState *pit_extra = qdev_new("s32k3-pit");
            int ch;
            qdev_connect_clock_in(pit_extra, "module_clk", s->aips_slow_clk);
            sysbus_realize(SYS_BUS_DEVICE(pit_extra), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(pit_extra), 0, pit_base[i]);
            for (ch = 0; ch < 4; ch++) {
                sysbus_connect_irq(SYS_BUS_DEVICE(pit_extra), ch,
                                   qdev_get_gpio_in(DEVICE(&s->armv7m),
                                                    pit_irq[i]));
            }
        }

        /* eMIOS x3: 24 channels each on AIPS_PLAT_CLK (240 MHz) */
        static const hwaddr emios_base[3] = {
            S32K344_EMIOS0_BASE, S32K344_EMIOS1_BASE, S32K344_EMIOS2_BASE,
        };
        static const uint8_t emios_irq[3] = {
            S32K344_IRQ_EMIOS0, S32K344_IRQ_EMIOS1, S32K344_IRQ_EMIOS2,
        };
        for (i = 0; i < 3; i++) {
            DeviceState *em = qdev_new("s32k3-emios");
            s->emios[i] = em;
            qdev_connect_clock_in(em, "module_clk", s->aips_plat_clk);
            sysbus_realize(SYS_BUS_DEVICE(em), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(em), 0, emios_base[i]);
            /* 模型内将 24 通道 flag OR 成 6 条 NVIC 线（ch23/19/15/11/7/3），
             * 即 sysbus irq 24-29（前 24 条是 per-channel irq） */
            for (int g = 0; g < 6; g++) {
                sysbus_connect_irq(SYS_BUS_DEVICE(em), g + 24,
                                   qdev_get_gpio_in(DEVICE(&s->armv7m),
                                                    emios_irq[i] + g));
            }
        }

        /* ADC x3 on AIPS_PLAT_CLK */
        static const hwaddr adc_base[3] = {
            S32K344_ADC0_BASE, S32K344_ADC1_BASE, S32K344_ADC2_BASE,
        };
        static const uint8_t adc_irq[3] = {
            S32K344_IRQ_ADC0, S32K344_IRQ_ADC1, S32K344_IRQ_ADC2,
        };
        for (i = 0; i < 3; i++) {

            DeviceState *ad = qdev_new("s32k3-adc");


            s->adc[i] = ad;
            qdev_connect_clock_in(ad, "module_clk", s->aips_plat_clk);
            sysbus_realize(SYS_BUS_DEVICE(ad), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(ad), 0, adc_base[i]);
            sysbus_connect_irq(SYS_BUS_DEVICE(ad), 0,
                               qdev_get_gpio_in(DEVICE(&s->armv7m),
                                                adc_irq[i]));
        }

        /* LPI2C x2 on AIPS_SLOW_CLK; bus names "i2c0"/"i2c1" */
        static const hwaddr i2c_base[S32K344_LPI2C_INSTANCES] = {
            S32K344_LPI2C0_BASE, S32K344_LPI2C1_BASE,
        };
        static const uint8_t i2c_irq[S32K344_LPI2C_INSTANCES] = {
            S32K344_IRQ_LPI2C0, S32K344_IRQ_LPI2C1,
        };
        for (i = 0; i < S32K344_LPI2C_INSTANCES; i++) {
            DeviceState *ic = qdev_new("s32k3-lpi2c");
            s->i2c[i] = ic;
            qdev_connect_clock_in(ic, "module_clk", s->aips_slow_clk);
            sysbus_realize(SYS_BUS_DEVICE(ic), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(ic), 0, i2c_base[i]);
            sysbus_connect_irq(SYS_BUS_DEVICE(ic), 0,
                               qdev_get_gpio_in(DEVICE(&s->armv7m),
                                                i2c_irq[i]));
        }
    }

    /* 11. LCU x2 + BCTU with real trigger-chain wiring:
     *
     *   eMIOS0 ch0 flag -> LCU0 input0 -> LCU0 output0 -> BCTU trigger0
     *   PIT ch0 flag    --------------------------------> BCTU trigger1
     *   BCTU adc-trig[i] -------------------------------> ADC[i] hw-trig
     *
     * so firmware can build the classic motor-control chain
     * "PWM edge -> LCU logic -> BCTU -> ADC sample".
     */
    {
        DeviceState *lcu0, *lcu1, *bctu;

        /* LCU instances on AIPS_PLAT_CLK */
        lcu0 = qdev_new("s32k3-lcu");
        qdev_connect_clock_in(lcu0, "module_clk", s->aips_plat_clk);
        sysbus_realize(SYS_BUS_DEVICE(lcu0), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(lcu0), 0, S32K344_LCU0_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(lcu0), 0,
                           qdev_get_gpio_in(DEVICE(&s->armv7m),
                                            S32K344_IRQ_LCU0));
        s->lcu[0] = lcu0;

        lcu1 = qdev_new("s32k3-lcu");
        qdev_connect_clock_in(lcu1, "module_clk", s->aips_plat_clk);
        sysbus_realize(SYS_BUS_DEVICE(lcu1), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(lcu1), 0, S32K344_LCU1_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(lcu1), 0,
                           qdev_get_gpio_in(DEVICE(&s->armv7m),
                                            S32K344_IRQ_LCU1));
        s->lcu[1] = lcu1;

        /* BCTU on AIPS_PLAT_CLK */
        bctu = qdev_new("s32k3-bctu");
        qdev_connect_clock_in(bctu, "module_clk", s->aips_plat_clk);
        sysbus_realize(SYS_BUS_DEVICE(bctu), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(bctu), 0, S32K344_BCTU_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(bctu), 0,
                           qdev_get_gpio_in(DEVICE(&s->armv7m),
                                            S32K344_IRQ_BCTU));
        s->bctu = bctu;

        /* trigger chain wiring */
        qdev_connect_gpio_out_named(s->emios[0], "pwm", 0,
                                    qdev_get_gpio_in_named(lcu0, "lc-in", 0));
        qdev_connect_gpio_out_named(lcu0, "lc-out", 0,
                                    qdev_get_gpio_in_named(bctu, "trig-in", 0));
        /* PIT ch0 -> NVIC (out 0) + BCTU trig 1 (out 1) via split-irq:
         * qdev GPIO outs are single links, so fan out before connecting. */
        DeviceState *pit_split = qdev_new(TYPE_SPLIT_IRQ);
        qdev_prop_set_uint32(pit_split, "num-lines", 2);
        qdev_realize_and_unref(pit_split, NULL, &error_fatal);
        sysbus_connect_irq(SYS_BUS_DEVICE(s->pit), 0,
                           qdev_get_gpio_in(DEVICE(pit_split), 0));
        qdev_connect_gpio_out(DEVICE(pit_split), 0,
                              qdev_get_gpio_in(DEVICE(&s->armv7m),
                                               S32K344_IRQ_PIT0));
        qdev_connect_gpio_out(DEVICE(pit_split), 1,
                              qdev_get_gpio_in_named(bctu, "trig-in", 1));

        /* BCTU -> ADC hardware triggers; ADC conv-done -> BCTU result */
        for (int i = 0; i < 3; i++) {
            qdev_connect_gpio_out_named(bctu, "adc-trig", i,
                                        qdev_get_gpio_in_named(s->adc[i],
                                                               "hw-trig", 0));
            qdev_connect_gpio_out_named(s->adc[i], "conv-done", 0,
                                        qdev_get_gpio_in_named(bctu,
                                                               "adc-done", i));
        }

        DB_PRINT("LCU0/LCU1/BCTU realized, trigger chain wired");
    }

    /* 12. Mode/Reset control blocks + SWT watchdog.
     *
     * FXOSC / PLL / MC_CGM 已在第 7 节作为真实时钟树创建；
     * 这里只补 MC_ME 与 MC_RGM（模式/复位状态）。
     */
    {
        /* MC_ME（模式状态） */
        DeviceState *mcme = qdev_new(TYPE_S32K3_CLKGEN);
        qdev_prop_set_uint32(mcme, "kind", CLKGEN_KIND_MC_ME);
        sysbus_realize(SYS_BUS_DEVICE(mcme), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(mcme), 0, S32K344_MCME_BASE);

        /* MC_RGM（复位状态）；复位事件 -> 请求 MC_CGM 切安全时钟 */
        DeviceState *mcrgm = qdev_new(TYPE_S32K3_CLKGEN);
        qdev_prop_set_uint32(mcrgm, "kind", CLKGEN_KIND_MC_RGM);
        sysbus_realize(SYS_BUS_DEVICE(mcrgm), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(mcrgm), 0, S32K344_MCRGM_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(mcrgm), 0,
                           qdev_get_gpio_in_named(s->clkgen, "safe-sw", 0));

        /* PFLASH 控制器（PFC）：等待状态配置/锁寄存器 */
        {
            DeviceState *pfc = qdev_new("s32k3-pfc");
            qdev_connect_clock_in(pfc, "module_clk", s->aips_slow_clk);
            sysbus_realize(SYS_BUS_DEVICE(pfc), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(pfc), 0, 0x40268000);
        }

        /* c40asf Flash 命令接口（FLASH0 @ 0x402EC000，RM 21.7.1.1） */
        {
            DeviceState *fl = qdev_new("s32k3-flash");
            qdev_connect_clock_in(fl, "module_clk", s->aips_slow_clk);
            sysbus_realize(SYS_BUS_DEVICE(fl), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(fl), 0, 0x402EC000);
        }

        /* DMA Channel Multiplexer x2（RM 14 章） */
        {
            static const hwaddr dmamux_base[2] = {
                0x40280000, 0x40284000,
            };
            int di;
            for (di = 0; di < 2; di++) {
                DeviceState *dmx = qdev_new("s32k3-dmamux");
                qdev_connect_clock_in(dmx, "module_clk", s->aips_slow_clk);
                sysbus_realize(SYS_BUS_DEVICE(dmx), &error_fatal);
                sysbus_mmio_map(SYS_BUS_DEVICE(dmx), 0, dmamux_base[di]);
                /* DMAMUX0 输出 -> eDMA 请求输入（32 通道链路） */
                if (di == 0) {
                    for (int ci = 0; ci < 32; ci++) {
                        qdev_connect_gpio_out(dmx, ci,
                                              qdev_get_gpio_in(
                                                  DEVICE(s->dma), ci));
                    }
                }
            }
        }

        /* Messaging Unit x2（MU_0 用于 HSE_B，MU_1 @ AIPS2） */
        {
            static const hwaddr mu_base[2] = {
                0x4038C000, 0x404EC000,
            };
            int mi;
            for (mi = 0; mi < 2; mi++) {
                DeviceState *mu = qdev_new("s32k3-mu");
                qdev_connect_clock_in(mu, "module_clk", s->aips_slow_clk);
                sysbus_realize(SYS_BUS_DEVICE(mu), &error_fatal);
                sysbus_mmio_map(SYS_BUS_DEVICE(mu), 0, mu_base[mi]);
            }
        }

        /* Trigger MUX（RM 65 章，触发源选择） */
        {
            DeviceState *trg = qdev_new("s32k3-trgmux");
            qdev_connect_clock_in(trg, "module_clk", s->aips_slow_clk);
            sysbus_realize(SYS_BUS_DEVICE(trg), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(trg), 0, 0x40080000);
        }

        /* FCCU（RM 52 章）+ XRDC（访问控制）占位 */
        {
            DeviceState *fccu = qdev_new("s32k3-fccu");
            qdev_prop_set_uint32(fccu, "kind", 0);   /* FCCU */
            qdev_connect_clock_in(fccu, "module_clk", s->aips_slow_clk);
            sysbus_realize(SYS_BUS_DEVICE(fccu), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(fccu), 0, 0x40384000);

            DeviceState *xrdc = qdev_new("s32k3-fccu");
            qdev_prop_set_uint32(xrdc, "kind", 1);   /* XRDC */
            qdev_connect_clock_in(xrdc, "module_clk", s->aips_slow_clk);
            sysbus_realize(SYS_BUS_DEVICE(xrdc), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(xrdc), 0, 0x40278000);
        }

        /* STCU/MSCM/RTC/CRC 占位（s32k3-sysctl，kind 区分） */
        {
            static const struct {
                uint32_t kind;
                hwaddr base;
            } sysctl_blocks[] = {
                { 0, 0x403A0000 },   /* STCU */
                { 1, 0x40260000 },   /* MSCM */
                { 2, 0x40288000 },   /* RTC */
                { 3, 0x40380000 },   /* CRC */
            };
            int si;
            for (si = 0; si < (int)ARRAY_SIZE(sysctl_blocks); si++) {
                DeviceState *sc = qdev_new("s32k3-sysctl");
                qdev_prop_set_uint32(sc, "kind", sysctl_blocks[si].kind);
                qdev_connect_clock_in(sc, "module_clk", s->aips_slow_clk);
                sysbus_realize(SYS_BUS_DEVICE(sc), &error_fatal);
                sysbus_mmio_map(SYS_BUS_DEVICE(sc), 0, sysctl_blocks[si].base);
            }
        }

        /* SEMA42 信号量（RM 63：16 门 + GPR） */
        {
            DeviceState *sema = qdev_new("s32k3-sema42");
            sysbus_realize(SYS_BUS_DEVICE(sema), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(sema), 0, 0x40460000);
        }

        /* CONFIGURATION_GPR（RM：APP_CORE_ACC=5 允许应用核写） */
        {
            DeviceState *gpr = qdev_new("s32k3-sysctl");
            qdev_prop_set_uint32(gpr, "kind", 4);
            sysbus_realize(SYS_BUS_DEVICE(gpr), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(gpr), 0, 0x4039C000);
        }

        /* RAM 控制器（PRAMC）x3：SRAM0/1/2（手册 23 章，PRCR1） */
        {
            static const hwaddr pramc_base[3] = {
                0x40264000, 0x40464000, 0x40468000,
            };
            int pi;
            for (pi = 0; pi < 3; pi++) {
                DeviceState *pramc = qdev_new("s32k3-pfc");
                qdev_prop_set_uint32(pramc, "kind", 1);
                qdev_connect_clock_in(pramc, "module_clk", s->aips_slow_clk);
                sysbus_realize(SYS_BUS_DEVICE(pramc), &error_fatal);
                sysbus_mmio_map(SYS_BUS_DEVICE(pramc), 0, pramc_base[pi]);
            }
        }

        /* SWT0 on AIPS_SLOW_CLK; timeout asserts irq only by
         * default (reset-on-timeout=false) so examples don't boot-loop.
         * 手册 S32K348 仅 SWT0 一个实例。 */
        {
            DeviceState *wt = qdev_new("s32k3-swt");
            s->swt[0] = wt;
            /* 手册 Table 431：S32K348 SWT 时钟源为 SIRC 32KHz */
            qdev_connect_clock_in(wt, "module_clk", s->sirc_clk);
            sysbus_realize(SYS_BUS_DEVICE(wt), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(wt), 0, S32K344_SWT0_BASE);
            sysbus_connect_irq(SYS_BUS_DEVICE(wt), 0,
                               qdev_get_gpio_in(DEVICE(&s->armv7m),
                                                S32K344_IRQ_SWT0));
        }
    }

    /* 12b. unimp 占位：手册 S32K348 存在但未建模的外设。
     * 真实固件会读写这些地址，占位避免总线错误（读返回 0/写丢弃）。 */
    {
        static const struct {
            const char *name;
            hwaddr base;
            size_t size;
        } unimp[] = {
            /* AIPS0 */
            { "s32k348.erm1",      0x4000C000, 0x4000 },
            /* AIPS1 */
            { "s32k348.axbs",      0x40200000, 0x4000 },
            { "s32k348.xbic-sys",  0x40204000, 0x4000 },
            { "s32k348.xbic-per",  0x40208000, 0x4000 },
            { "s32k348.sda-ap",    0x40254000, 0x4000 },
            { "s32k348.erm0",      0x4025C000, 0x4000 },
            { "s32k348.stm0",      0x40274000, 0x4000 },
            { "s32k348.intm",      0x4027C000, 0x4000 },
            { "s32k348.siul-pdac1a",0x40294000, 0x4000 },
            { "s32k348.siul-pdac1b",0x40298000, 0x4000 },
            { "s32k348.siul-pdac1c",0x4029C000, 0x4000 },
            { "s32k348.siul-pdac3",0x402A8000, 0x4000 },
            { "s32k348.dcm",       0x402AC000, 0x4000 },
            { "s32k348.wkpu",      0x402B4000, 0x4000 },
            { "s32k348.cmu",       0x402BC000, 0x4000 },
            { "s32k348.tscc",      0x402C4000, 0x4000 },
            { "s32k348.sirc",      0x402C8000, 0x4000 },
            
            { "s32k348.firc",      0x402D0000, 0x4000 },
            { "s32k348.pll2",      0x402E4000, 0x4000 },
            { "s32k348.pmc",       0x402E8000, 0x4000 },
            { "s32k348.flexio",    0x40324000, 0x4000 },
            { "s32k348.sai0",      0x4036C000, 0x4000 },
            { "s32k348.jdc",       0x40394000, 0x4000 },
            
            { "s32k348.selftest-gpr",0x403B0000, 0x4000 },
            /* AIPS2 */
            { "s32k348.xbic-dma",  0x40404000, 0x4000 },
            { "s32k348.xbic-pram", 0x40408000, 0x4000 },
            { "s32k348.qspi",      0x404CC000, 0x4000 },
            { "s32k348.sai1",      0x404DC000, 0x4000 },
            { "s32k348.usdhc",     0x404E4000, 0x4000 },
            { "s32k348.eim0",      0x4050C000, 0x4000 },
            { "s32k348.eim1",      0x40510000, 0x4000 },
            { "s32k348.eim2",      0x40514000, 0x4000 },
        };
        size_t i;

        for (i = 0; i < ARRAY_SIZE(unimp); i++) {
            create_unimplemented_device(unimp[i].name,
                                        unimp[i].base, unimp[i].size);
        }
        DB_PRINT("unimp placeholder: %d peripherals", (int)ARRAY_SIZE(unimp));
    }

    /* 13. Load kernel / firmware */
    if (machine->kernel_filename) {
        DB_PRINT("Loading kernel: %s", machine->kernel_filename);
        armv7m_load_kernel(ARM_CPU(first_cpu),
                           machine->kernel_filename,
                           S32K344_FLASH0_BASE,
                           S32K344_FLASH_BLK_SIZE * S32K344_FLASH_BLOCKS);


        /* S32K3 固件带 IVT boot header（0x00400000: 0x5AA55AA5），
         * 向量表地址在 IVT+0x0C（如 0x00400800），真实启动由 bootROM
         * 解析 IVT。这里模拟：若检测到 IVT 则把 init_svtor 指向向量表。
         * 注意：ELF 段以 ROM blob 形式存在，reset 才拷入 RAM，须用
         * rom_ptr_for_as 读取。 */
        AddressSpace *cpu_as = cpu_get_address_space(first_cpu, ARMASIdx_NS);
        const void *ivt_rom = rom_ptr_for_as(cpu_as, S32K344_FLASH0_BASE, 8);
        uint32_t ivt_tag = 0;
        if (ivt_rom) {
            ivt_tag = ldl_le_p(ivt_rom);
        }
        if (ivt_tag == 0x5AA55AA5u) {
            const void *vt_rom = rom_ptr_for_as(cpu_as,
                                                S32K344_FLASH0_BASE + 0x0C,
                                                4);
            uint32_t vt = vt_rom ? ldl_le_p(vt_rom) : 0;
            if ((vt & 0xFFF00000u) != 0 && (vt & 0x7Fu) == 0) {
                DB_PRINT("S32K3 IVT detected, vector table @ 0x%08X", vt);
                object_property_set_uint(OBJECT(s->armv7m.cpu), "init-nsvtor",
                                         vt, &error_fatal);
            }
        } else {
            /* 无 IVT：检查 0x00400000 是否有有效向量表（MSP 落在 SRAM）。
             * 若没有（固件链接到 0x00500000 等由外部启动器跳转的 APP），
             * 从 ELF entry 引导：写跳板到 0x00400000（设 MSP + 跳 entry）。 */
            const void *msp_rom = rom_ptr_for_as(cpu_as, S32K344_FLASH0_BASE, 4);
            uint32_t msp0 = msp_rom ? ldl_le_p(msp_rom) : 0;
            if ((msp0 & 0xFFF00000u) != 0x20400000u &&
                (msp0 & 0xFFF00000u) != 0x20000000u) {
                FILE *ef = fopen(machine->kernel_filename, "rb");
                uint32_t entry = 0;
                if (ef) {
                    uint8_t hdr[0x20];
                    if (fread(hdr, 1, 0x20, ef) == 0x20) {
                        /* ELF32 e_entry @ 0x18（本机仅 32 位 LE ELF） */
                        entry = ldl_le_p(hdr + 0x18);
                    }
                    fclose(ef);
                }
                if (entry) {
                    /* 栈顶取 SRAM 上沿（0x2042F000 为常见固件栈区；
                     * 可被固件自身 _start 的 msr msp 覆盖）。
                     * 跳板同时把 SCB->VTOR 指向固件加载基址，使固件
                     * SystemInit 的 MPU 区域计算覆盖实际代码区。 */
                    uint32_t stack = 0x2042F000u;
                    uint32_t jp = S32K344_FLASH0_BASE + 0x08;
                    uint8_t jb[0x28];
                    stl_le_p(jb + 0x00, stack);          /* MSP */
                    stl_le_p(jb + 0x04, jp | 1);         /* Reset -> 跳板 */
                    jb[0x08] = 0x03; jb[0x09] = 0x48;    /* ldr r0,[pc,#12]; VTOR */
                    jb[0x0A] = 0x04; jb[0x0B] = 0x49;    /* ldr r1,[pc,#16]; 固件基址 */
                    jb[0x0C] = 0x01; jb[0x0D] = 0x60;    /* str r1,[r0] */
                    jb[0x0E] = 0x04; jb[0x0F] = 0x48;    /* ldr r0,[pc,#16]; stack */
                    jb[0x10] = 0x85; jb[0x11] = 0x46;    /* mov sp,r0 */
                    jb[0x12] = 0x04; jb[0x13] = 0x49;    /* ldr r1,[pc,#16]; entry */
                    jb[0x14] = 0x08; jb[0x15] = 0x47;    /* bx r1 */
                    jb[0x16] = 0x00; jb[0x17] = 0xBF;    /* nop */
                    stl_le_p(jb + 0x18, 0xE000ED80u);    /* SCB->VTOR */
                    stl_le_p(jb + 0x1C, S32K344_FLASH0_BASE + 0x100000u);
                    stl_le_p(jb + 0x20, stack);
                    stl_le_p(jb + 0x24, entry | 1);
                    rom_add_blob_fixed_as("s32k348-elf-jump", jb,
                                          sizeof(jb), S32K344_FLASH0_BASE,
                                          cpu_as);
                    DB_PRINT("No vector table, jump stub -> entry 0x%08X",
                             entry);
                }
            }
        }
    }

    DB_PRINT("S32K344EVB board init complete.");
}

/* ------------------------------------------------------------------
 *  Machine Class Init
 * ------------------------------------------------------------------ */
static void s32k344evb_machine_init(MachineClass *mc)
{
    mc->desc = "NXP S32K344EVB (BCOM Motor Controller)";
    mc->init = s32k344evb_board_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m7");
    mc->default_ram_size = 0;   /* RAM is fixed on-chip */
    mc->no_parallel = 1;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    /* QEMU 11: mc->no_sdcard is gone; auto_create_sdcard defaults to false */
}

DEFINE_MACHINE_EXTENDED("s32k344evb", MACHINE, S32K344EVBMachineState,
                        s32k344evb_machine_init, false,
                        arm_machine_interfaces)
