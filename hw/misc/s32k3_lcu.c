/*
 * NXP S32K3xx LCU (Logic Control Unit) QEMU device model
 *
 * 3 LCs per instance, each LC has 4 outputs, every output is a
 * 4-input LUT (16-bit truth table). Register layout per RM 62.8.1:
 *   LCn block @ n*0x40: LUTCTRL0-3/FILT0-3/INTDMAEN/STS/OUTPOL/FFILT/FCTRL/SCTRL
 *   MUXSEL0-11 @ 0x200
 *   SWEN @ 0x284 / SWVALUE @ 0x288 / OUTEN @ 0x28C / LCIN @ 0x290 /
 *   SWOUT @ 0x294 / LCOUT @ 0x298 / FORCEOUT @ 0x29C / FORCESTS @ 0x2A0 /
 *   DBGEN @ 0x2A8 / CFG @ 0x280
 * Inputs come from board-wired qdev gpio lines ("lc-in"), outputs drive
 * qdev gpio lines ("lc-out") that the board can wire to BCTU triggers or
 * eMIOS channels -- i.e. a real hardware trigger chain in emulation.
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

#define S32K3_LCU_LCS        3   /* LC0-2（RM Table：3 个 LC） */
#define S32K3_LCU_LC_INPUTS  4   /* 每 LC 4 输入（MUXSEL 12 个 / 3 LC） */
#define S32K3_LCU_LC_OUTPUTS 4   /* 每 LC 4 输出（LUTCTRL0-3） */
#define S32K3_LCU_IN_LINES   (S32K3_LCU_LCS * S32K3_LCU_LC_INPUTS)
#define S32K3_LCU_OUT_LINES  (S32K3_LCU_LCS * S32K3_LCU_LC_OUTPUTS)

/* per-LC block */
#define LC_STRIDE      0x40

/* 实例级寄存器（RM 62.8.1） */
#define LCU_CFG        0x280
#define LCU_SWEN       0x284
#define LCU_SWVALUE    0x288
#define LCU_OUTEN      0x28C
#define LCU_LCIN       0x290   /* R */
#define LCU_SWOUT      0x294   /* R */
#define LCU_LCOUT      0x298   /* R */
#define LCU_FORCEOUT   0x29C   /* R */
#define LCU_FORCESTS   0x2A0
#define LCU_DBGEN      0x2A8
#define LCU_MUXSEL_BASE 0x200

struct S32K3LcuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock        *module_clk;
    qemu_irq     irq[S32K3_LCU_LCS];   /* 每 LC 一条中断线 */

    uint32_t cfg;
    uint32_t swen;      /* 软件覆盖使能（bit i = 输出 i） */
    uint32_t swvalue;   /* 软件覆盖值 */
    uint32_t outen;     /* 输出使能 */
    uint32_t forces;    /* FORCESTS */
    uint32_t dbgen;

    uint32_t muxsel[S32K3_LCU_LCS * S32K3_LCU_LC_INPUTS];   /* MUXSEL0-11 */
    uint32_t lutctrl[S32K3_LCU_LCS][S32K3_LCU_LC_OUTPUTS];
    uint32_t filt[S32K3_LCU_LCS][S32K3_LCU_LC_OUTPUTS];
    uint32_t intdmaen[S32K3_LCU_LCS];
    uint32_t sts[S32K3_LCU_LCS];        /* 状态（输出跳变置位，W1C） */
    uint32_t outpol[S32K3_LCU_LCS];
    uint32_t ffilt[S32K3_LCU_LCS];
    uint32_t fctrl[S32K3_LCU_LCS];
    uint32_t sctrl[S32K3_LCU_LCS];

    uint32_t regs[0x100];   /* 影子：未实现偏移读回写值/写存储 */

    uint8_t in_level[S32K3_LCU_IN_LINES];
    uint8_t filt_pending[S32K3_LCU_IN_LINES];   /* FILT 滤波待确认电平 */
    uint8_t out_level[S32K3_LCU_OUT_LINES];

    qemu_irq out[S32K3_LCU_OUT_LINES];
};

static void s32k3_lcu_eval_lc(S32K3LcuState *s, int lc)
{
    int o, k;
    uint8_t lut_in[S32K3_LCU_LC_INPUTS];

    for (k = 0; k < S32K3_LCU_LC_INPUTS; k++) {
        uint32_t sel = s->muxsel[lc * S32K3_LCU_LC_INPUTS + k] & 0x3f;
        uint8_t v = 0;

        if (sel < S32K3_LCU_IN_LINES) {
            v = s->in_level[sel];
        }
        if (s->outpol[lc] & (1 << k)) {
            v = !v;
        }
        lut_in[k] = v;
    }

    for (o = 0; o < S32K3_LCU_LC_OUTPUTS; o++) {
        int idx = lc * S32K3_LCU_LC_OUTPUTS + o;
        uint8_t v;
        bool changed;

        if (!(s->outen & (1 << idx))) {
            v = 0;
        } else if (s->swen & (1 << idx)) {
            v = (s->swvalue >> idx) & 1;
        } else if (s->fctrl[lc] & (1 << o)) {
            v = (s->fctrl[lc] >> (16 + o)) & 1;
        } else {
            unsigned idx4 = (lut_in[3] << 3) | (lut_in[2] << 2) |
                            (lut_in[1] << 1) | lut_in[0];
            v = (s->lutctrl[lc][o] >> idx4) & 1;
        }

        changed = (v != s->out_level[idx]);
        if (changed) {
            s->out_level[idx] = v;
            qemu_set_irq(s->out[idx], v);
            /* 输出跳变：置 STS 位；INTDMAEN[IE] 使能时触发中断 */
            s->sts[lc] |= 1u << o;
            if (s->intdmaen[lc] & (1u << o)) {
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
    /* FILT 滤波：使能时电平变化先记录 pending，同电平第二拍才确认 */
    {
        int lc = line / S32K3_LCU_LC_INPUTS;
        int k = line % S32K3_LCU_LC_INPUTS;

        if (lc < S32K3_LCU_LCS && (s->filt[lc][k] & 0x1)) {
            if (s->filt_pending[line] == (level & 1)) {
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
    s32k3_lcu_eval(s);
}

static void s32k3_lcu_reset(DeviceState *dev)
{
    S32K3LcuState *s = S32K3_LCU(dev);

    memset(s->muxsel, 0, sizeof(s->muxsel));
    memset(s->lutctrl, 0, sizeof(s->lutctrl));
    memset(s->filt, 0, sizeof(s->filt));
    memset(s->intdmaen, 0, sizeof(s->intdmaen));
    memset(s->sts, 0, sizeof(s->sts));
    memset(s->outpol, 0, sizeof(s->outpol));
    memset(s->ffilt, 0, sizeof(s->ffilt));
    memset(s->fctrl, 0, sizeof(s->fctrl));
    memset(s->sctrl, 0, sizeof(s->sctrl));
    s->cfg = 0;
    s->swen = 0;
    s->swvalue = 0;
    s->outen = 0;
    s->forces = 0;
    s->dbgen = 0;
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->filt_pending, 0xFF, sizeof(s->filt_pending));
    s32k3_lcu_eval(s);
}

static uint64_t s32k3_lcu_read(void *opaque, hwaddr addr, unsigned size)
{
    S32K3LcuState *s = opaque;
    int lc;
    uint64_t r = 0;

    if (size == 8) {
        uint64_t lo = s32k3_lcu_read(opaque, addr, 4);
        uint64_t hi = s32k3_lcu_read(opaque, addr + 4, 4);
        return lo | (hi << 32);
    }
    if (size == 2) {
        uint32_t full = s32k3_lcu_read(opaque, addr & ~3u, 4);
        return (addr & 2) ? (full >> 16) : (full & 0xFFFF);
    }
    if (size == 1) {
        uint32_t full = s32k3_lcu_read(opaque, addr & ~3u, 4);
        return (full >> (8 * (addr & 3))) & 0xFF;
    }

    /* LCn 块（RM：n*0x40） */
    for (lc = 0; lc < S32K3_LCU_LCS; lc++) {
        hwaddr base = lc * LC_STRIDE;
        if (addr >= base && addr < base + LC_STRIDE) {
            hwaddr off = addr - base;

            switch (off) {
            case 0x00: case 0x04: case 0x08: case 0x0C:
                return s->lutctrl[lc][off / 4];
            case 0x10: case 0x14: case 0x18: case 0x1C:
                return s->filt[lc][(off - 0x10) / 4];
            case 0x20: return s->intdmaen[lc];
            case 0x24: return s->sts[lc];
            case 0x28: return s->outpol[lc];
            case 0x2C: return s->ffilt[lc];
            case 0x30: return s->fctrl[lc];
            case 0x34: return s->sctrl[lc];
            default:   return s->regs[addr / 4];
            }
        }
    }

    /* MUXSEL0-11 @ 0x200 */
    if (addr >= LCU_MUXSEL_BASE &&
        addr < LCU_MUXSEL_BASE + S32K3_LCU_IN_LINES * 4) {
        return s->muxsel[(addr - LCU_MUXSEL_BASE) / 4];
    }

    switch (addr) {
    case LCU_CFG:      return s->cfg;
    case LCU_SWEN:     return s->swen;
    case LCU_SWVALUE:  return s->swvalue;
    case LCU_OUTEN:    return s->outen;
    case LCU_LCIN:
        for (int i = 0; i < S32K3_LCU_IN_LINES; i++) {
            r |= (s->in_level[i] & 1) << i;
        }
        return r;
    case LCU_SWOUT:
        return s->swen & s->swvalue;
    case LCU_LCOUT:
        for (int i = 0; i < S32K3_LCU_OUT_LINES; i++) {
            r |= (s->out_level[i] & 1) << i;
        }
        return r;
    case LCU_FORCEOUT:
        return s->fctrl[0] | (s->fctrl[1] << 4) | (s->fctrl[2] << 8);
    case LCU_FORCESTS: return s->forces;
    case LCU_DBGEN:    return s->dbgen;
    default:
        return s->regs[addr / 4];
    }
}

static void s32k3_lcu_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    S32K3LcuState *s = opaque;
    uint32_t v = value;
    int lc;

    if (size == 8) {
        s32k3_lcu_write(opaque, addr, value & 0xFFFFFFFF, 4);
        s32k3_lcu_write(opaque, addr + 4, value >> 32, 4);
        return;
    }
    if (size == 2) {
        /* 16 位写：读-改-写对应半字（固件 RTD 可能 16 位写 LCU 位域） */
        uint32_t full = s32k3_lcu_read(opaque, addr & ~3u, 4);
        uint32_t w = value & 0xFFFF;
        uint32_t merged = (addr & 2) ? ((full & 0xFFFF) | (w << 16))
                                     : ((full & 0xFFFF0000u) | w);
        s32k3_lcu_write(opaque, addr & ~3u, merged, 4);
        return;
    }

    if (size == 1) {
        uint32_t full = s32k3_lcu_read(opaque, addr & ~3u, 4);
        uint32_t sh = 8 * (addr & 3);
        uint32_t merged = (full & ~(0xFFu << sh)) | ((value & 0xFF) << sh);
        s32k3_lcu_write(opaque, addr & ~3u, merged, 4);
        return;
    }
    /* LCn 块 */
    for (lc = 0; lc < S32K3_LCU_LCS; lc++) {
        hwaddr base = lc * LC_STRIDE;
        if (addr >= base && addr < base + LC_STRIDE) {
            hwaddr off = addr - base;

            switch (off) {
            case 0x00: case 0x04: case 0x08: case 0x0C:
                s->lutctrl[lc][off / 4] = v & 0xffff;   /* 16 位真值表 */
                break;
            case 0x10: case 0x14: case 0x18: case 0x1C:
                s->filt[lc][(off - 0x10) / 4] = v;
                break;
            case 0x20: s->intdmaen[lc] = v; break;
            case 0x24: s->sts[lc] &= ~v; break;          /* W1C */
            case 0x28: s->outpol[lc] = v; break;
            case 0x2C: s->ffilt[lc] = v; break;
            case 0x30: s->fctrl[lc] = v; break;
            case 0x34: s->sctrl[lc] = v; break;
            default:
                s->regs[addr / 4] = v;
                return;
            }
            s32k3_lcu_eval_lc(s, lc);
            return;
        }
    }

    /* MUXSEL0-11 @ 0x200 */
    if (addr >= LCU_MUXSEL_BASE &&
        addr < LCU_MUXSEL_BASE + S32K3_LCU_IN_LINES * 4) {
        s->muxsel[(addr - LCU_MUXSEL_BASE) / 4] = v;
        s32k3_lcu_eval(s);
        return;
    }

    switch (addr) {
    case LCU_CFG:      s->cfg = v; break;
    case LCU_SWEN:     s->swen = v; s32k3_lcu_eval(s); break;
    case LCU_SWVALUE:  s->swvalue = v; s32k3_lcu_eval(s); break;
    case LCU_OUTEN:    s->outen = v; s32k3_lcu_eval(s); break;
    case LCU_FORCESTS: s->forces = v; break;
    case LCU_DBGEN:    s->dbgen = v; break;
    default:
        s->regs[addr / 4] = v;
    }
}

static const MemoryRegionOps s32k3_lcu_ops = {
    .read = s32k3_lcu_read,
    .write = s32k3_lcu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
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
    dc->desc = "NXP S32K3xx LCU (3 LCs, LUT logic, trigger matrix)";
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
