/*
 * NXP S32K3xx Trigger MUX (TRGMUX) QEMU device model
 *
 * TRGMUX @ 0x40080000 (RM 65.x). Each target peripheral has one 32-bit
 * register with SEL0-3 fields (each 8 bits) selecting which of up to 255
 * trigger inputs is routed to that peripheral's trigger outputs 0-3.
 * Input lines arrive on the "trig-in" gpio; when the selected source
 * asserts, the target output ("trig-out") pulses.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_S32K3_TRGMUX "s32k3-trgmux"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3TrgmuxState, S32K3_TRGMUX)

#define S32K3_TRGMUX_REGS     32    /* target registers */
#define S32K3_TRGMUX_INPUTS   255

#define SEL0_SHIFT 0
#define SEL1_SHIFT 8
#define SEL2_SHIFT 16
#define SEL3_SHIFT 24
#define SEL_MASK   0xFF

struct S32K3TrgmuxState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock *module_clk;

    uint32_t sel[S32K3_TRGMUX_REGS];
    qemu_irq trig_out[S32K3_TRGMUX_REGS * 4];   /* 每目标 4 输出 */
    qemu_irq trig_in[S32K3_TRGMUX_INPUTS];      /* 触发输入 */
};

/* 触发输入：路由到所有选中该源的输出 */
static void s32k3_trgmux_in_set(void *opaque, int line, int level)
{
    S32K3TrgmuxState *s = opaque;
    int r, o;

    if (line < 0 || line > S32K3_TRGMUX_INPUTS) {
        return;
    }
    for (r = 0; r < S32K3_TRGMUX_REGS; r++) {
        for (o = 0; o < 4; o++) {
            int sel = (s->sel[r] >> (8 * o)) & SEL_MASK;
            if (sel == line) {
                qemu_set_irq(s->trig_out[r * 4 + o], level);
            }
        }
    }
}

static void s32k3_trgmux_reset(DeviceState *dev)
{
    S32K3TrgmuxState *s = S32K3_TRGMUX(dev);

    memset(s->sel, 0, sizeof(s->sel));
}

static uint64_t s32k3_trgmux_read(void *opaque, hwaddr addr, unsigned size)
{
    if (size == 8) {
        uint64_t lo = s32k3_trgmux_read(opaque, addr, 4);
        uint64_t hi = s32k3_trgmux_read(opaque, addr + 4, 4);
        return lo | (hi << 32);
    }
    if (size == 2) {
        uint32_t full = s32k3_trgmux_read(opaque, addr & ~3u, 4);
        return (addr & 2) ? (full >> 16) : (full & 0xFFFF);
    }
    S32K3TrgmuxState *s = opaque;

    if ((addr & 3) == 0 && (addr / 4) < S32K3_TRGMUX_REGS) {
        return s->sel[addr / 4];
    }
    return 0;
}

static void s32k3_trgmux_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned size)
{
    if (size == 8) {
        s32k3_trgmux_write(opaque, addr, value & 0xFFFFFFFF, 4);
        s32k3_trgmux_write(opaque, addr + 4, value >> 32, 4);
        return;
    }
    if (size == 2) {
        uint32_t full = s32k3_trgmux_read(opaque, addr & ~3u, 4);
        uint32_t w = value & 0xFFFF;
        uint32_t merged = (addr & 2) ? ((full & 0xFFFF) | (w << 16))
                                     : ((full & 0xFFFF0000u) | w);
        s32k3_trgmux_write(opaque, addr & ~3u, merged, 4);
        return;
    }
    S32K3TrgmuxState *s = opaque;

    if ((addr & 3) == 0 && (addr / 4) < S32K3_TRGMUX_REGS) {
        s->sel[addr / 4] = value & 0xFFFFFFFF;
    }
}

static const MemoryRegionOps s32k3_trgmux_ops = {
    .read = s32k3_trgmux_read,
    .write = s32k3_trgmux_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static void s32k3_trgmux_init(Object *obj)
{
    S32K3TrgmuxState *s = S32K3_TRGMUX(obj);

    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);

    memory_region_init_io(&s->iomem, obj, &s32k3_trgmux_ops, s,
                          TYPE_S32K3_TRGMUX, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    qdev_init_gpio_out(DEVICE(s), s->trig_out, S32K3_TRGMUX_REGS * 4);
    qdev_init_gpio_in(DEVICE(s), s32k3_trgmux_in_set, S32K3_TRGMUX_INPUTS);
}

static void s32k3_trgmux_realize(DeviceState *dev, Error **errp)
{
    S32K3TrgmuxState *s = S32K3_TRGMUX(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_trgmux: module_clk must be connected");
        return;
    }
    s32k3_trgmux_reset(dev);
}

static void s32k3_trgmux_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_trgmux_reset);
    dc->realize = s32k3_trgmux_realize;
    dc->desc = "NXP S32K3xx Trigger MUX";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo s32k3_trgmux_types[] = {
    {
        .name          = TYPE_S32K3_TRGMUX,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3TrgmuxState),
        .instance_init = s32k3_trgmux_init,
        .class_init    = s32k3_trgmux_class_init,
    },
};

DEFINE_TYPES(s32k3_trgmux_types)
