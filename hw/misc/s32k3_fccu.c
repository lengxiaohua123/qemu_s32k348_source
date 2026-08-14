/*
 * NXP S32K3xx Fault Collection and Control Unit (FCCU) QEMU device model
 *
 * FCCU @ 0x40384000 (RM 52.x). Implements the key status/config registers
 * with correct reset values so ASIL firmware's FCCU init does not misread
 * fault states. Non-critical fault status (NCF_S0) supports W1C clearing.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_S32K3_FCCU "s32k3-fccu"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3FccuState, S32K3_FCCU)

/* registers (RM 52.5) */
#define FCCU_CTRL       0x00
#define FCCU_CTRLK      0x04   /* Control Key（解锁写寄存器） */
#define FCCU_CFG        0x08   /* Configuration（手册 52.7.1.1） */
#define FCCU_NCF_CFG0   0x1C
#define FCCU_NCFS_CFG0  0x4C
#define FCCU_NCF_S0     0x80
#define FCCU_NCFK       0x90
#define FCCU_NCF_E0     0x94
#define FCCU_NCF_TOE0   0xA4
#define FCCU_NCF_TO     0xB4
#define FCCU_CFG_TO     0xB8
#define FCCU_EINOUT     0xBC
#define FCCU_STAT       0xC0
#define FCCU_IRQ_STAT   0xE0
#define FCCU_IRQ_EN     0xE4
#define FCCU_TRANS_LOCK 0xF0
#define FCCU_PERMNT_LOCK 0xF4
#define FCCU_IRQ_ALARM_EN 0xFC

struct S32K3FccuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock *module_clk;
    qemu_irq irq;

    uint32_t kind;          /* 0=FCCU, 1=XRDC */

    uint32_t ctrl;
    uint32_t cfg;
    uint32_t ncf_cfg0;
    uint32_t ncfs_cfg0;
    uint32_t ncf_s0;
    uint32_t ncfk;
    uint32_t ncf_e0;
    uint32_t ncf_toe0;
    uint32_t ncf_to;
    uint32_t cfg_to;
    uint32_t einout;
    uint32_t stat;
    uint32_t irq_stat;
    uint32_t irq_en;
    uint32_t trans_lock;
    uint32_t permnt_lock;
};

static void s32k3_fccu_update_irq(S32K3FccuState *s)
{
    /* NCF 状态且 IRQ 使能 -> 中断 */
    qemu_set_irq(s->irq, (s->ncf_s0 & s->ncf_e0 & s->irq_en) != 0);
}

static void s32k3_fccu_reset(DeviceState *dev)
{
    S32K3FccuState *s = S32K3_FCCU(dev);

    s->ctrl = 0;
    s->cfg = 0;
    s->ncf_cfg0 = 0x000000FF;
    s->ncfs_cfg0 = 0;
    s->ncf_s0 = 0;
    s->ncfk = 0;
    s->ncf_e0 = 0;
    s->ncf_toe0 = 0x000000FF;
    s->ncf_to = 0x0003A980;
    s->cfg_to = 0x00000005;
    s->einout = 0;
    s->stat = 0x00000010;   /* 复位：NORMAL 状态 */
    s->irq_stat = 0;
    s->irq_en = 0;
    s->trans_lock = 0;
    s->permnt_lock = 0;
    s32k3_fccu_update_irq(s);
}

static uint64_t s32k3_fccu_read(void *opaque, hwaddr addr, unsigned size)
{
    if (size == 8) {
        uint64_t lo = s32k3_fccu_read(opaque, addr, 4);
        uint64_t hi = s32k3_fccu_read(opaque, addr + 4, 4);
        return lo | (hi << 32);
    }
    if (size == 2) {
        uint32_t full = s32k3_fccu_read(opaque, addr & ~3u, 4);
        return (addr & 2) ? (full >> 16) : (full & 0xFFFF);
    }
    if (size == 1) {
        uint32_t full = s32k3_fccu_read(opaque, addr & ~3u, 4);
        return (full >> (8 * (addr & 3))) & 0xFF;
    }
    S32K3FccuState *s = opaque;

    if (s->kind == 1) {
        /* XRDC：访问控制寄存器占位（读 0、写接受） */
        return 0;
    }

    switch (addr) {
    case FCCU_CTRL:       return s->ctrl;
    case FCCU_CFG:        return s->cfg;
    case FCCU_NCF_CFG0:   return s->ncf_cfg0;
    case FCCU_NCFS_CFG0:  return s->ncfs_cfg0;
    case FCCU_NCF_S0:     return s->ncf_s0;
    case FCCU_NCFK:       return s->ncfk;
    case FCCU_NCF_E0:     return s->ncf_e0;
    case FCCU_NCF_TOE0:   return s->ncf_toe0;
    case FCCU_NCF_TO:     return s->ncf_to;
    case FCCU_CFG_TO:     return s->cfg_to;
    case FCCU_EINOUT:     return s->einout;
    case FCCU_STAT:       return s->stat;
    case FCCU_IRQ_STAT:   return s->irq_stat;
    case FCCU_IRQ_EN:     return s->irq_en;
    case FCCU_TRANS_LOCK: return s->trans_lock;
    case FCCU_PERMNT_LOCK: return s->permnt_lock;
    case FCCU_IRQ_ALARM_EN: return 0;
    default:
        return 0;
    }
}

static void s32k3_fccu_write(void *opaque, hwaddr addr,
                             uint64_t value, unsigned size)
{
    if (size == 8) {
        s32k3_fccu_write(opaque, addr, value & 0xFFFFFFFF, 4);
        s32k3_fccu_write(opaque, addr + 4, value >> 32, 4);
        return;
    }
    if (size == 2) {
        uint32_t full = s32k3_fccu_read(opaque, addr & ~3u, 4);
        uint32_t w = value & 0xFFFF;
        uint32_t merged = (addr & 2) ? ((full & 0xFFFF) | (w << 16))
                                     : ((full & 0xFFFF0000u) | w);
        s32k3_fccu_write(opaque, addr & ~3u, merged, 4);
        return;
    }
    if (size == 1) {
        uint32_t full = s32k3_fccu_read(opaque, addr & ~3u, 4);
        uint32_t sh = 8 * (addr & 3);
        uint32_t merged = (full & ~(0xFFu << sh)) | ((value & 0xFF) << sh);
        s32k3_fccu_write(opaque, addr & ~3u, merged, 4);
        return;
    }
    S32K3FccuState *s = opaque;
    uint32_t v = value;

    if (s->kind == 1) {
        /* XRDC：写接受（不校验访问） */
        return;
    }

    switch (addr) {
    case FCCU_CTRL:
        s->ctrl = v;
        break;
    case FCCU_CTRLK:
        /* Control Key（只写解锁寄存器）：无操作 */
        break;
    case FCCU_CFG:
        s->cfg = v;
        break;
    case FCCU_NCF_CFG0:   s->ncf_cfg0 = v; break;
    case FCCU_NCFS_CFG0:  s->ncfs_cfg0 = v; break;
    case FCCU_NCF_S0:
        /* W1C 清除（需先 NCFK 解锁，简化：直接清） */
        s->ncf_s0 &= ~v;
        s32k3_fccu_update_irq(s);
        break;
    case FCCU_NCFK:
        s->ncfk = v;
        break;
    case FCCU_NCF_E0:     s->ncf_e0 = v; break;
    case FCCU_NCF_TOE0:   s->ncf_toe0 = v; break;
    case FCCU_NCF_TO:     s->ncf_to = v; break;
    case FCCU_CFG_TO:     s->cfg_to = v; break;
    case FCCU_EINOUT:     s->einout = v; break;
    case FCCU_IRQ_STAT:   s->irq_stat &= ~v; break;   /* W1C */
    case FCCU_IRQ_EN:     s->irq_en = v; s32k3_fccu_update_irq(s); break;
    case FCCU_TRANS_LOCK: s->trans_lock = v; break;
    case FCCU_PERMNT_LOCK: s->permnt_lock = v; break;
    default:
        break;
    }
}

static const MemoryRegionOps s32k3_fccu_ops = {
    .read = s32k3_fccu_read,
    .write = s32k3_fccu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static void s32k3_fccu_init(Object *obj)
{
    S32K3FccuState *s = S32K3_FCCU(obj);

    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);

    memory_region_init_io(&s->iomem, obj, &s32k3_fccu_ops, s,
                          TYPE_S32K3_FCCU, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

static void s32k3_fccu_realize(DeviceState *dev, Error **errp)
{
    S32K3FccuState *s = S32K3_FCCU(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_fccu: module_clk must be connected");
        return;
    }
    s32k3_fccu_reset(dev);
}

static const Property s32k3_fccu_properties[] = {
    DEFINE_PROP_UINT32("kind", S32K3FccuState, kind, 0),  /* 0=FCCU 1=XRDC */
};

static void s32k3_fccu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_fccu_reset);
    device_class_set_props(dc, s32k3_fccu_properties);
    dc->realize = s32k3_fccu_realize;
    dc->desc = "NXP S32K3xx Fault Collection / XRDC";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo s32k3_fccu_types[] = {
    {
        .name          = TYPE_S32K3_FCCU,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3FccuState),
        .instance_init = s32k3_fccu_init,
        .class_init    = s32k3_fccu_class_init,
    },
};

DEFINE_TYPES(s32k3_fccu_types)
