/*
 * NXP S32K3xx SIUL2 QEMU device model
 *
 * Models the pad-multiplexing (MSCR) registers plus GPIO data path.
 * Each of the 512 pads is exposed as a qdev GPIO line so boards can wire
 * LEDs / buttons / external controllers, and the input path can be driven
 * from QEMU monitor / test code.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/gpio/s32k3_siul2.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"

#define EIRQ_MAX 32

static void s32k3_siul2_update_irq(S32K3Siul2State *s)
{
    uint32_t pending = s->disr0 & s->direr0;
    int g;

    /* EIRQ0-31 按 8 个一组驱动 4 条中断线（IRQ 53-56） */
    for (g = 0; g < 4; g++) {
        uint32_t group = (pending >> (8 * g)) & 0xFF;
        qemu_set_irq(s->irq[g], group != 0);
    }
    /* 兼容单 irq（旧接口） */
    qemu_set_irq(s->irq[0], pending != 0);
}

#define SIUL2_FILT_US 5   /* IFER 滤波采样周期（虚拟 us） */

/* IFER 滤波确认：输入变化后保持稳定一个采样周期才确认沿（去毛刺） */
static void s32k3_siul2_filt_confirm(void *opaque)
{
    S32K3Siul2State *s = opaque;
    int pin = s->filt_pin;
    int level;

    if (pin < 0) {
        return;
    }
    s->filt_pin = -1;
    level = s->gpio_in[pin];
    if (level != s->filt_level) {
        return;   /* 采样期内又变化：毛刺，丢弃 */
    }
    /* 保持稳定：确认边沿（rise 由 filt_level=1 表达） */
    if (level) {
        if (s->ireer0 & (1 << pin)) {
            s->disr0 |= 1 << pin;
            s32k3_siul2_update_irq(s);
        }
    } else {
        if (s->ifeer0 & (1 << pin)) {
            s->disr0 |= 1 << pin;
            s32k3_siul2_update_irq(s);
        }
    }
}

static void s32k3_siul2_gpio_set(void *opaque, int line, int level)
{
    S32K3Siul2State *s = opaque;
    uint32_t prev;
    bool rise, fall;

    if (line < 0 || line >= S32K3_NUM_GPIO) {
        return;
    }

    prev = s->gpio_in[line];
    s->gpio_in[line] = level & 1;

    if (line < EIRQ_MAX && (s->direr0 & (1 << line))) {
        rise = !prev && level;
        fall = prev && !level;
        /* IFER 滤波（去毛刺）：变化后启动采样，保持稳定 SIUL2_FILT_US
         * 才确认沿；采样期内再次变化则丢弃（毛刺）。 */
        if (s->ifer0 & (1 << line)) {
            s->filt_pin = line;
            s->filt_level = level & 1;
            timer_mod_ns(s->filt_timer,
                         qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                         SIUL2_FILT_US * 1000);
        } else {
            /* 无滤波：直接触发 */
            if ((rise && (s->ireer0 & (1 << line))) ||
                (fall && (s->ifeer0 & (1 << line)))) {
                s->disr0 |= 1 << line;
                s32k3_siul2_update_irq(s);
            }
        }
    }
}

static void s32k3_siul2_reset(DeviceState *dev)
{
    S32K3Siul2State *s = S32K3_SIUL2(dev);

    memset(s->shadow, 0, sizeof(s->shadow));
    memset(s->imcr, 0, sizeof(s->imcr));
    memset(s->mscr, 0, sizeof(s->mscr));
    s->disr0 = 0;
    s->direr0 = 0;
    s->dirsr0 = 0;
    s->ireer0 = 0;
    s->ifeer0 = 0;
    s->ifer0 = 0;
    memset(s->gpio_out, 0, sizeof(s->gpio_out));
    s->filt_pin = -1;
    s32k3_siul2_update_irq(s);
}

static void s32k3_siul2_drive_pin(S32K3Siul2State *s, int pin, uint8_t val)
{
    s->gpio_out[pin] = val & 1;
    if (s->mscr[pin] & MSCR_OBE) {
        qemu_set_irq(s->gpios[pin], val & 1);
    }
}

static uint64_t s32k3_siul2_read(void *opaque, hwaddr addr, unsigned size)
{
    S32K3Siul2State *s = opaque;
    uint32_t r = 0;
    int n;

    if (addr >= SIUL2_MSCR_BASE &&
        addr < SIUL2_MSCR_BASE + 4 * SIUL2_MSCR_COUNT) {
        n = (addr - SIUL2_MSCR_BASE) / 4;
        return s->mscr[n];
    }
    if (addr >= SIUL2_IMCR_BASE && addr < SIUL2_IMCR_BASE + 4 * SIUL2_IMCR_COUNT) {
        n = (addr - SIUL2_IMCR_BASE) / 4;
        return s->imcr[n];
    }
    if (addr >= SIUL2_GPDO_BASE && addr < SIUL2_GPDO_BASE + S32K3_NUM_GPIO) {
        n = addr - SIUL2_GPDO_BASE;
        return s->gpio_out[n];
    }
    if (addr >= SIUL2_GPDI_BASE && addr < SIUL2_GPDI_BASE + S32K3_NUM_GPIO) {
        n = addr - SIUL2_GPDI_BASE;
        return s->gpio_in[n];
    }
    if (addr >= SIUL2_PGPDO_BASE && addr < SIUL2_PGPDO_BASE + 16 * 4) {
        n = (addr - SIUL2_PGPDO_BASE) / 4;
        r = 0;
        for (int b = 0; b < 32; b++) {
            r |= (s->gpio_out[n * 32 + b] & 1) << b;
        }
        return r;
    }
    if (addr >= SIUL2_PGPDI_BASE && addr < SIUL2_PGPDI_BASE + 16 * 4) {
        n = (addr - SIUL2_PGPDI_BASE) / 4;
        r = 0;
        for (int b = 0; b < 32; b++) {
            r |= (s->gpio_in[n * 32 + b] & 1) << b;
        }
        return r;
    }

    switch (addr) {
    case SIUL2_MIDR1:
        r = 0x53484C30;   /* 'SHL0' style part tag */
        break;
    case SIUL2_MIDR2:
        r = 0x00000012;
        break;
    case SIUL2_DISR0:
        r = s->disr0;
        break;
    case SIUL2_DIRER0:
        r = s->direr0;
        break;
    case SIUL2_DIRSR0:
        r = s->dirsr0;
        break;
    case SIUL2_IREER0:
        r = s->ireer0;
        break;
    case SIUL2_IFEER0:
        r = s->ifeer0;
        break;
    case SIUL2_IFER0:
        r = s->ifer0;
        break;
    default:
        if (addr < 0x1000) {
            return s->shadow[addr / 4];
        }
    }
    return r;
}

static void s32k3_siul2_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned size)
{
    S32K3Siul2State *s = opaque;
    uint32_t v = value;
    int n;

    if (addr >= SIUL2_MSCR_BASE &&
        addr < SIUL2_MSCR_BASE + 4 * SIUL2_MSCR_COUNT) {
        n = (addr - SIUL2_MSCR_BASE) / 4;
        s->mscr[n] = v;
        if (v & MSCR_OBE) {
            qemu_set_irq(s->gpios[n], s->gpio_out[n] & 1);
        }
        return;
    }
    if (addr >= SIUL2_GPDO_BASE && addr < SIUL2_GPDO_BASE + S32K3_NUM_GPIO) {
        n = addr - SIUL2_GPDO_BASE;
        s32k3_siul2_drive_pin(s, n, v);
        return;
    }
    if (addr >= SIUL2_PGPDO_BASE && addr < SIUL2_PGPDO_BASE + 16 * 4) {
        n = (addr - SIUL2_PGPDO_BASE) / 4;
        for (int b = 0; b < 32; b++) {
            s32k3_siul2_drive_pin(s, n * 32 + b, (v >> b) & 1);
        }
        return;
    }

    switch (addr) {
    case SIUL2_DISR0:
        s->disr0 &= ~v;   /* W1C */
        s32k3_siul2_update_irq(s);
        break;
    case SIUL2_DIRER0:
        s->direr0 = v;
        s32k3_siul2_update_irq(s);
        break;
    case SIUL2_DIRSR0:
        s->dirsr0 = v;
        break;
    case SIUL2_IREER0:
        s->ireer0 = v;
        break;
    case SIUL2_IFEER0:
        s->ifeer0 = v;
        break;
    case SIUL2_IFER0:
        s->ifer0 = v;
        break;
    case SIUL2_MIDR1:
    case SIUL2_MIDR2:
        break; /* read-only */
    default:
        if (addr >= SIUL2_IMCR_BASE &&
            addr < SIUL2_IMCR_BASE + 4 * SIUL2_IMCR_COUNT) {
            n = (addr - SIUL2_IMCR_BASE) / 4;
            s->imcr[n] = v;
        } else if (addr < 0x1000) {
            s->shadow[addr / 4] = v;
        }
    }
}

static const MemoryRegionOps s32k3_siul2_ops = {
    .read = s32k3_siul2_read,
    .write = s32k3_siul2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void s32k3_siul2_init(Object *obj)
{
    S32K3Siul2State *s = S32K3_SIUL2(obj);

    s->filt_pin = -1;
    s->filt_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                 s32k3_siul2_filt_confirm, s);
    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);

    memory_region_init_io(&s->iomem, obj, &s32k3_siul2_ops, s,
                          TYPE_S32K3_SIUL2, S32K3_SIUL2_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    for (int i = 0; i < 4; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq[i]);
    }

    qdev_init_gpio_out_named(DEVICE(s), s->gpios, "gpio", S32K3_NUM_GPIO);
    qdev_init_gpio_in_named(DEVICE(s), s32k3_siul2_gpio_set, "gpio-in",
                            S32K3_NUM_GPIO);
}

static void s32k3_siul2_realize(DeviceState *dev, Error **errp)
{
    S32K3Siul2State *s = S32K3_SIUL2(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_siul2: module_clk must be connected");
        return;
    }
    s32k3_siul2_reset(dev);
}

static void s32k3_siul2_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_siul2_reset);
    dc->realize = s32k3_siul2_realize;
    dc->desc = "NXP S32K3xx SIUL2 (GPIO / pad mux)";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo s32k3_siul2_types[] = {
    {
        .name          = TYPE_S32K3_SIUL2,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3Siul2State),
        .instance_init = s32k3_siul2_init,
        .class_init    = s32k3_siul2_class_init,
    },
};

DEFINE_TYPES(s32k3_siul2_types)
