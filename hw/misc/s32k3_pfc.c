/*
 * NXP S32K3xx Flash Memory Controller (PFLASH / PFC) QEMU device model
 *
 * Platform flash configuration registers (PFCRn), access protection
 * (PFAPR) and program/erase address + lock registers, per RM Rev.11
 * (PFLASH chapter 22, base 0x40268000).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_S32K3_PFC "s32k3-pfc"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3PfcState, S32K3_PFC)

/* registers (RM 22.5) */
#define PFC_PFCR0        0x00
#define PFC_PFCR1        0x04
#define PFC_PFCR2        0x08
#define PFC_PFCR3        0x0C
#define PFC_PFCR4        0x10
#define PFC_PFAPR        0x14
#define PFC_PEALR        0x300
#define PFC_PEAR         0x304
#define PFC_PEALRX       0x308
#define PFC_PEARX        0x30C
#define PFC_PFCBLK0_SPELOCK 0x340
#define PFC_PFCBLKU_SPELOCK 0x358
#define PFC_PFCBLK0_SSPELOCK 0x35C
#define PFC_PFCBLK0_SETSLOCK 0x380
#define PFC_PFCBLKU_SETSLOCK 0x398
#define PFC_PFCBLK0_SSETSLOCK 0x39C
#define PFC_PFCBLK0_LOCKMASTER_S0 0x3C0   /* 0x3C0-45C 只读 */
#define PFC_PFCBLKU_LOCKMASTER_S 0x480    /* 只读 0x00FF */

struct S32K3PfcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock *module_clk;

    uint32_t kind;          /* 0=PFC, 1=PRAMC */
    uint32_t prcr1;         /* PRAMC: platform RAM config 1 (reset 0x100) */

    uint32_t pfcr[5];
    uint32_t pfapr;
    uint32_t pealr;
    uint32_t pealrx;
    uint32_t spelock[5];     /* PFCBLKn_SPELOCK 0-3 + UTEST@4 */
    uint32_t sspelock[5];
    uint32_t setslock[5];
    uint32_t ssetslock[5];
};

static void s32k3_pfc_reset(DeviceState *dev)
{
    S32K3PfcState *s = S32K3_PFC(dev);
    int i;

    /* PFCR0-3 reset 0x3 (RWSC=3 wait states), PFCR4 reset 0x0 */
    s->pfcr[0] = s->pfcr[1] = s->pfcr[2] = s->pfcr[3] = 0x3;
    s->pfcr[4] = 0x0;
    s->pfapr = 0xFFFFFFFF;
    s->pealr = 0;
    s->pealrx = 0;
    s->prcr1 = 0x100;   /* PRAMC PRCR1 复位 */
    for (i = 0; i < 5; i++) {
        /* S32K3 PFC SPELOCK/SSPELOCK 复位为未锁（0）。
         * bootloader(C40_Ip) 出厂即烧写 codeflash，GetLock 期望
         * PFCBLK0_SSPELOCK 对应位为 0（返回 UNPROTECTED），
         * 复位全锁会导致解锁/重锁振荡，每帧耗时数十秒。 */
        s->spelock[i] = 0;
        s->sspelock[i] = 0;
        s->setslock[i] = 0;
        s->ssetslock[i] = 0;
    }
}

static uint64_t s32k3_pfc_read(void *opaque, hwaddr addr, unsigned size)
{
    S32K3PfcState *s = opaque;

    if (s->kind == 1) {
        /* PRAMC：仅 PRCR1 */
        return addr == 0 ? s->prcr1 : 0;
    }

    switch (addr) {
    case PFC_PFCR0: case PFC_PFCR1: case PFC_PFCR2:
    case PFC_PFCR3: case PFC_PFCR4:
        return s->pfcr[(addr - PFC_PFCR0) / 4];
    case PFC_PFAPR:
        return s->pfapr;
    case PFC_PEALR:
        return s->pealr;
    case PFC_PEAR:
        return s->pealr & 0x0FFFFFFF;   /* physical addr = logical masked */
    case PFC_PEALRX:
        return s->pealrx;
    case PFC_PEARX:
        return s->pealrx & 0x0FFFFFFF;
    default:
        if (addr >= PFC_PFCBLK0_SPELOCK && addr < PFC_PFCBLK0_SPELOCK + 0x14) {
            return s->spelock[(addr - PFC_PFCBLK0_SPELOCK) / 4];
        }
        if (addr == PFC_PFCBLKU_SPELOCK) {
            return s->spelock[4];
        }
        if (addr >= PFC_PFCBLK0_SSPELOCK &&
            addr < PFC_PFCBLK0_SSPELOCK + 0x10) {
            return s->sspelock[(addr - PFC_PFCBLK0_SSPELOCK) / 4];
        }
        if (addr >= PFC_PFCBLK0_SETSLOCK &&
            addr < PFC_PFCBLK0_SETSLOCK + 0x18) {
            return s->setslock[(addr - PFC_PFCBLK0_SETSLOCK) / 4];
        }
        if (addr == PFC_PFCBLKU_SETSLOCK) {
            return s->setslock[4];
        }
        if (addr >= PFC_PFCBLK0_SSETSLOCK &&
            addr < PFC_PFCBLK0_SSETSLOCK + 0x10) {
            return s->ssetslock[(addr - PFC_PFCBLK0_SSETSLOCK) / 4];
        }
        /* LOCKMASTER 只读：block 0-15 lock master sector = 全 1 */
        if (addr >= PFC_PFCBLK0_LOCKMASTER_S0 &&
            addr < PFC_PFCBLK0_LOCKMASTER_S0 + 0x9C) {
            return 0xFFFFFFFF;
        }
        if (addr == PFC_PFCBLKU_LOCKMASTER_S) {
            return 0x000000FF;   /* UTEST lock master sector */
        }
        /* 其余只读/保留寄存器：返回 0（避免总线错误） */
        return 0;
    }
}

static void s32k3_pfc_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    S32K3PfcState *s = opaque;
    uint32_t v = value;

    if (s->kind == 1) {
        /* PRAMC：仅 PRCR1 */
        if (addr == 0) {
            s->prcr1 = v;
        }
        return;
    }

    switch (addr) {
    case PFC_PFCR0: case PFC_PFCR1: case PFC_PFCR2:
    case PFC_PFCR3: case PFC_PFCR4:
        s->pfcr[(addr - PFC_PFCR0) / 4] = v;
        break;
    case PFC_PFAPR:
        s->pfapr = v;
        break;
    case PFC_PEALR:
        s->pealr = v;
        break;
    case PFC_PEALRX:
        s->pealrx = v;
        break;
    default:
        if (addr >= PFC_PFCBLK0_SPELOCK && addr < PFC_PFCBLK0_SPELOCK + 0x14) {
            s->spelock[(addr - PFC_PFCBLK0_SPELOCK) / 4] = v;
            break;
        }
        if (addr == PFC_PFCBLKU_SPELOCK) {
            s->spelock[4] = v;
            break;
        }
        if (addr >= PFC_PFCBLK0_SSPELOCK &&
            addr < PFC_PFCBLK0_SSPELOCK + 0x10) {
            s->sspelock[(addr - PFC_PFCBLK0_SSPELOCK) / 4] = v;
            break;
        }
        if (addr >= PFC_PFCBLK0_SETSLOCK &&
            addr < PFC_PFCBLK0_SETSLOCK + 0x18) {
            s->setslock[(addr - PFC_PFCBLK0_SETSLOCK) / 4] = v;
            break;
        }
        if (addr == PFC_PFCBLKU_SETSLOCK) {
            s->setslock[4] = v;
            break;
        }
        if (addr >= PFC_PFCBLK0_SSETSLOCK &&
            addr < PFC_PFCBLK0_SSETSLOCK + 0x10) {
            s->ssetslock[(addr - PFC_PFCBLK0_SSETSLOCK) / 4] = v;
            break;
        }
        /* 其余只读寄存器：写忽略 */
        break;
    }
}

static const MemoryRegionOps s32k3_pfc_ops = {
    .read = s32k3_pfc_read,
    .write = s32k3_pfc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void s32k3_pfc_init(Object *obj)
{
    S32K3PfcState *s = S32K3_PFC(obj);

    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);

    memory_region_init_io(&s->iomem, obj, &s32k3_pfc_ops, s,
                          TYPE_S32K3_PFC, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static void s32k3_pfc_realize(DeviceState *dev, Error **errp)
{
    S32K3PfcState *s = S32K3_PFC(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_pfc: module_clk must be connected");
        return;
    }
    s32k3_pfc_reset(dev);
}

static const Property s32k3_pfc_properties[] = {
    DEFINE_PROP_UINT32("kind", S32K3PfcState, kind, 0),   /* 0=PFC 1=PRAMC */
};

static void s32k3_pfc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_pfc_reset);
    device_class_set_props(dc, s32k3_pfc_properties);
    dc->realize = s32k3_pfc_realize;
    dc->desc = "NXP S32K3xx Flash/RAM Memory Controller (PFLASH/PRAMC)";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo s32k3_pfc_types[] = {
    {
        .name          = TYPE_S32K3_PFC,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3PfcState),
        .instance_init = s32k3_pfc_init,
        .class_init    = s32k3_pfc_class_init,
    },
};

DEFINE_TYPES(s32k3_pfc_types)
