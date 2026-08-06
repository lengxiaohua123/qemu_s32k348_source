/*
 * NXP S32K3xx LCU (Logic Control Unit) QEMU device model
 *
 * 2 LCs per instance, each LC has 8 outputs, every output is a
 * 4-input LUT (16-bit truth table). Inputs come from board-wired
 * qdev gpio lines ("lc-in"), outputs drive qdev gpio lines ("lc-out")
 * that the board can wire to BCTU triggers or eMIOS channels --
 * i.e. a real hardware trigger chain in emulation.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_S32K3_LCU "s32k3-lcu"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3LcuState, S32K3_LCU)

#define S32K3_LCU_LCS        2
#define S32K3_LCU_LC_INPUTS  6
#define S32K3_LCU_LC_OUTPUTS 8
#define S32K3_LCU_IN_LINES   (S32K3_LCU_LCS * S32K3_LCU_LC_INPUTS)
#define S32K3_LCU_OUT_LINES  (S32K3_LCU_LCS * S32K3_LCU_LC_OUTPUTS)

/* registers (subset) */
#define LCU_VERID      0x00
#define LCU_PARAM      0x04
#define LCU_SYNCCTRL   0x10

/* per-LC block: base 0x200 + lc * 0x100 */
#define LC_BASE        0x200
#define LC_STRIDE      0x100

#define LCU_LUT_INPUTS 4

struct S32K3LcuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock        *module_clk;
    qemu_irq     irq[S32K3_LCU_LCS];   /* 每 LC 一条中断线 */

    uint32_t syncctrl;

    uint32_t selin[S32K3_LCU_LCS][LCU_LUT_INPUTS];
    uint32_t selpol[S32K3_LCU_LCS];
    uint32_t lutctrl[S32K3_LCU_LCS][S32K3_LCU_LC_OUTPUTS];
    uint32_t lutint[S32K3_LCU_LCS][S32K3_LCU_LC_OUTPUTS];
    uint32_t filt[S32K3_LCU_LCS][S32K3_LCU_LC_OUTPUTS];
    uint32_t outen[S32K3_LCU_LCS];
    uint32_t forcectl[S32K3_LCU_LCS];
    uint32_t swen[S32K3_LCU_LCS];
    uint32_t swvalue[S32K3_LCU_LCS];

    uint8_t in_level[S32K3_LCU_IN_LINES];
    uint8_t filt_pending[S32K3_LCU_IN_LINES];   /* FILT 滤波待确认电平 */
    uint8_t out_level[S32K3_LCU_OUT_LINES];

    qemu_irq out[S32K3_LCU_OUT_LINES];
};

static void s32k3_lcu_eval_lc(S32K3LcuState *s, int lc)
{
    int o, k;
    uint8_t lut_in[LCU_LUT_INPUTS];

    for (k = 0; k < LCU_LUT_INPUTS; k++) {
        uint32_t sel = (s->selin[lc][k] & 0x3f);
        uint8_t v = 0;

        if (sel < S32K3_LCU_LC_INPUTS) {
            v = s->in_level[lc * S32K3_LCU_LC_INPUTS + sel];
        }
        if (s->selpol[lc] & (1 << k)) {
            v = !v;
        }
        lut_in[k] = v;
    }

    for (o = 0; o < S32K3_LCU_LC_OUTPUTS; o++) {
        int idx = lc * S32K3_LCU_LC_OUTPUTS + o;
        uint8_t v;
        bool changed;

        if (!(s->outen[lc] & (1 << o))) {
            v = 0;
        } else if (s->swen[lc] & (1 << o)) {
            v = (s->swvalue[lc] >> o) & 1;
        } else if (s->forcectl[lc] & (1 << o)) {
            v = (s->forcectl[lc] >> (16 + o)) & 1;
        } else {
            unsigned idx4 = (lut_in[3] << 3) | (lut_in[2] << 2) |
                            (lut_in[1] << 1) | lut_in[0];
            v = (s->lutctrl[lc][o] >> idx4) & 1;
        }

        changed = (v != s->out_level[idx]);
        if (changed) {
            s->out_level[idx] = v;
            qemu_set_irq(s->out[idx], v);
            /* LUTINT 中断：输出跳变且 LUTINT[EIF]/LUTINT[IE] 使能 */
            if (s->lutint[lc][o] & 0x1) {   /* IE bit0 */
                qemu_set_irq(s->irq[lc], 1);
                qemu_set_irq(s->irq[lc], 0);   /* 脉冲 */
            }
        }
    }
}

static void s32k3_lcu_eval(S32K3LcuState *s)
{
    int lc;

    for (lc = 0; lc < S32K3_LCU_LCS; lc++) {
        s32k3_lcu_eval_lc(s, lc);
    }
}

static void s32k3_lcu_in_set(void *opaque, int line, int level)
{
    S32K3LcuState *s = opaque;

    if (line < 0 || line >= S32K3_LCU_IN_LINES) {
        return;
    }
    /* FILT 滤波：使能时电平变化先记录 pending，同电平第二拍才确认
     * 更新输入（对应 LCU 输入滤波采样时序）。 */
    if (line < S32K3_LCU_LC_INPUTS) {
        int lc = line / S32K3_LCU_LCS;   /* 安全边界 */
        if (lc >= S32K3_LCU_LCS) {
            lc = 0;
        }
        if (s->filt[lc][line % S32K3_LCU_LC_OUTPUTS] & 0x1) {
            if (s->filt_pending[line] == (level & 1)) {
                /* 第二拍同电平：确认更新输入 */
                if (s->in_level[line] != (level & 1)) {
                    s->in_level[line] = level & 1;
                    s32k3_lcu_eval(s);
                }
                s->filt_pending[line] = 0xFF;
            } else {
                s->filt_pending[line] = level & 1;   /* 第一拍 */
            }
            return;
        }
    }
    s->in_level[line] = level & 1;
    /* SYNCCTRL 同步：使能时输出更新按同步时钟对齐（模型即时完成） */
    s32k3_lcu_eval(s);
}

static void s32k3_lcu_reset(DeviceState *dev)
{
    S32K3LcuState *s = S32K3_LCU(dev);

    memset(s->selin, 0, sizeof(s->selin));
    memset(s->selpol, 0, sizeof(s->selpol));
    memset(s->lutctrl, 0, sizeof(s->lutctrl));
    memset(s->lutint, 0, sizeof(s->lutint));
    memset(s->filt, 0, sizeof(s->filt));
    memset(s->outen, 0, sizeof(s->outen));
    memset(s->forcectl, 0, sizeof(s->forcectl));
    memset(s->swen, 0, sizeof(s->swen));
    memset(s->swvalue, 0, sizeof(s->swvalue));
    s->syncctrl = 0;
    memset(s->filt_pending, 0xFF, sizeof(s->filt_pending));
    s32k3_lcu_eval(s);
}

static uint64_t s32k3_lcu_read(void *opaque, hwaddr addr, unsigned size)
{
    S32K3LcuState *s = opaque;
    int lc;

    for (lc = 0; lc < S32K3_LCU_LCS; lc++) {
        hwaddr base = LC_BASE + lc * LC_STRIDE;
        if (addr >= base && addr < base + LC_STRIDE) {
            hwaddr off = addr - base;
            if (off >= 0x20 && off < 0x30) {
                return s->selin[lc][(off - 0x20) / 4];
            }
            switch (off) {
            case 0x30:
                return s->selpol[lc];
            case 0xA0:
                return s->outen[lc];
            case 0xB0:
                return s->forcectl[lc];
            case 0xB8:
                return s->swen[lc];
            case 0xBC:
                return s->swvalue[lc];
            default:
                if (off >= 0x40 && off < 0x60) {
                    return s->lutctrl[lc][(off - 0x40) / 4];
                }
                if (off >= 0x60 && off < 0x80) {
                    return s->lutint[lc][(off - 0x60) / 4];
                }
                if (off >= 0x80 && off < 0xA0) {
                    return s->filt[lc][(off - 0x80) / 4];
                }
                return 0;
            }
        }
    }

    switch (addr) {
    case LCU_VERID:
        return 0x01000001;
    case LCU_PARAM:
        return (S32K3_LCU_LC_OUTPUTS << 16) | S32K3_LCU_LC_INPUTS;
    case LCU_SYNCCTRL:
        return s->syncctrl;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_lcu: read of unimplemented reg 0x%03" HWADDR_PRIx "\n",
                      addr);
        return 0;
    }
}

static void s32k3_lcu_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    S32K3LcuState *s = opaque;
    uint32_t v = value;
    int lc;

    for (lc = 0; lc < S32K3_LCU_LCS; lc++) {
        hwaddr base = LC_BASE + lc * LC_STRIDE;
        if (addr >= base && addr < base + LC_STRIDE) {
            hwaddr off = addr - base;

            if (off >= 0x20 && off < 0x30) {
                s->selin[lc][(off - 0x20) / 4] = v;
            } else if (off >= 0x40 && off < 0x60) {
                s->lutctrl[lc][(off - 0x40) / 4] = v & 0xffff;
            } else if (off >= 0x60 && off < 0x80) {
                s->lutint[lc][(off - 0x60) / 4] = v;
            } else if (off >= 0x80 && off < 0xA0) {
                s->filt[lc][(off - 0x80) / 4] = v;
            } else {
                switch (off) {
                case 0x30:
                    s->selpol[lc] = v;
                    break;
                case 0xA0:
                    s->outen[lc] = v;
                    break;
                case 0xB0:
                    s->forcectl[lc] = v;
                    break;
                case 0xB8:
                    s->swen[lc] = v;
                    break;
                case 0xBC:
                    s->swvalue[lc] = v;
                    break;
                default:
                    return;
                }
            }
            s32k3_lcu_eval_lc(s, lc);
            return;
        }
    }

    switch (addr) {
    case LCU_SYNCCTRL:
        s->syncctrl = v;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_lcu: write of unimplemented reg 0x%03" HWADDR_PRIx
                      " = 0x%08" PRIx64 "\n", addr, value);
    }
}

static const MemoryRegionOps s32k3_lcu_ops = {
    .read = s32k3_lcu_read,
    .write = s32k3_lcu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void s32k3_lcu_init(Object *obj)
{
    S32K3LcuState *s = S32K3_LCU(obj);

    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);

    memory_region_init_io(&s->iomem, obj, &s32k3_lcu_ops, s,
                          TYPE_S32K3_LCU, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    for (int i = 0; i < S32K3_LCU_LCS; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq[i]);
    }

    qdev_init_gpio_in_named(DEVICE(s), s32k3_lcu_in_set, "lc-in",
                            S32K3_LCU_IN_LINES);
    qdev_init_gpio_out_named(DEVICE(s), s->out, "lc-out",
                             S32K3_LCU_OUT_LINES);
}

static void s32k3_lcu_realize(DeviceState *dev, Error **errp)
{
    S32K3LcuState *s = S32K3_LCU(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_lcu: module_clk must be connected");
        return;
    }
    s32k3_lcu_reset(dev);
}

static void s32k3_lcu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_lcu_reset);
    dc->realize = s32k3_lcu_realize;
    dc->desc = "NXP S32K3xx LCU (2 LCs, LUT logic, trigger matrix)";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo s32k3_lcu_types[] = {
    {
        .name          = TYPE_S32K3_LCU,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3LcuState),
        .instance_init = s32k3_lcu_init,
        .class_init    = s32k3_lcu_class_init,
    },
};

DEFINE_TYPES(s32k3_lcu_types)
