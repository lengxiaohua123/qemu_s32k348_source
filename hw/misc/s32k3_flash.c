/*
 * NXP S32K3xx c40asf Flash command interface (FLASH0) QEMU device model
 *
 * FLASH0 base 0x402EC000 (RM 21.7.1.1). Implements the program/erase
 * command flow so real firmware flash programming actually writes the
 * flash array:
 *   - program: MCR[PGM]=1, write DATA0-31, set EHV=1 -> data written
 *     to PEADR in the flash memory array (system address space)
 *   - sector erase: MCR[ERS]=1 + ESS, set EHV=1 -> erase sector
 *   - MCRS[DONE] reflects completion (instant model)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-clock.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "system/address-spaces.h"
#include "system/memory.h"

#define TYPE_S32K3_FLASH "s32k3-flash"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3FlashState, S32K3_FLASH)

/* registers (RM 21.7.1) */
#define FL_MCR          0x00
#define  MCR_EHV        (1 << 31)
#define  MCR_PSO        (1 << 27)
#define  MCR_SSO        (1 << 26)
#define  MCR_ESO        (1 << 25)
#define  MCR_PGM        (1 << 20)
#define  MCR_ERS        (1 << 19)
#define  MCR_ESS        (1 << 5)
#define FL_MCRS         0x04
#define  MCRS_DONE      (1 << 1)
#define FL_MCRE         0x08
#define  MCRE_ERR       (1 << 0)
#define FL_ADR          0x10
#define FL_PEADR        0x14
#define FL_SPELOCK      0x50
#define FL_SSPELOCK     0x54
#define FL_XMCR         0xF0
#define FL_XPEADR       0xF4
#define FL_DATA_BASE    0x100
#define FL_DATA_COUNT   32

#define S32K348_FLASH0_ARRAY 0x00400000   /* flash memory array */

struct S32K3FlashState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock *module_clk;

    uint32_t mcr;
    uint32_t mcrs;
    uint32_t mcre;
    uint32_t adr;
    uint32_t peadr;
    uint32_t data[FL_DATA_COUNT];
    uint32_t xpeadr;
};

static void s32k3_flash_reset(DeviceState *dev)
{
    S32K3FlashState *s = S32K3_FLASH(dev);

    s->mcr = 0;
    s->mcrs = 0x0000C100 | MCRS_DONE;   /* reset: DONE=1, defaults */
    s->mcre = 0;
    s->adr = 0;
    s->peadr = 0;
    s->xpeadr = 0;
    memset(s->data, 0xFF, sizeof(s->data));
}

/* program: write DATA0-31 to flash array at PEADR */
static void s32k3_flash_program(S32K3FlashState *s)
{
    uint32_t addr = s->peadr;
    int i;

    for (i = 0; i < FL_DATA_COUNT; i++) {
        address_space_write(&address_space_memory,
                            S32K348_FLASH0_ARRAY + addr + 4 * i,
                            MEMTXATTRS_UNSPECIFIED,
                            &s->data[i], 4);
    }
    s->mcre &= ~MCRE_ERR;
}

/* sector erase: fill sector (32KB) with 0xFF */
static void s32k3_flash_sector_erase(S32K3FlashState *s)
{
    uint32_t base = S32K348_FLASH0_ARRAY + (s->peadr & ~0x7FFFu);
    uint8_t ff = 0xFF;
    uint32_t off;

    for (off = 0; off < 0x8000; off++) {
        address_space_write(&address_space_memory, base + off,
                            MEMTXATTRS_UNSPECIFIED, &ff, 1);
    }
    s->mcre &= ~MCRE_ERR;
}

static uint64_t s32k3_flash_read(void *opaque, hwaddr addr, unsigned size)
{
    S32K3FlashState *s = opaque;

    switch (addr) {
    case FL_MCR:
        return s->mcr;
    case FL_MCRS:
        return s->mcrs;
    case FL_MCRE:
        return s->mcre;
    case FL_ADR:
        return s->adr;
    case FL_PEADR:
        return s->peadr;
    case FL_SPELOCK:
        return 0xFFFFFFFF;
    case FL_SSPELOCK:
        return 0x0FFFFFFF;
    case FL_XMCR:
        return 0x00FFC000;
    case FL_XPEADR:
        return s->xpeadr;
    default:
        if (addr >= FL_DATA_BASE && addr < FL_DATA_BASE + 4 * FL_DATA_COUNT) {
            return s->data[(addr - FL_DATA_BASE) / 4];
        }
        return 0;
    }
}

static void s32k3_flash_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned size)
{
    S32K3FlashState *s = opaque;
    uint32_t v = value;

    switch (addr) {
    case FL_MCR:
        s->mcr = v;
        if (v & MCR_EHV) {
            /* 高压操作开始：执行命令，然后完成 */
            if (s->mcr & MCR_PGM) {
                s32k3_flash_program(s);
            } else if (s->mcr & MCR_ERS) {
                s32k3_flash_sector_erase(s);
            }
            /* DONE 清后立即置位（即时完成） */
            s->mcrs &= ~MCRS_DONE;
            s->mcrs |= MCRS_DONE;
            s->mcr &= ~MCR_EHV;   /* EHV 自清除 */
        }
        break;
    case FL_MCRE:
        s->mcre &= ~v;   /* W1C */
        break;
    case FL_ADR:
        s->adr = v;
        s->peadr = v & 0x0FFFFFFF;
        break;
    case FL_XPEADR:
        s->xpeadr = v;
        break;
    default:
        if (addr >= FL_DATA_BASE && addr < FL_DATA_BASE + 4 * FL_DATA_COUNT) {
            s->data[(addr - FL_DATA_BASE) / 4] = v;
            break;
        }
        /* 其余只读/保留：写忽略 */
        break;
    }
}

static const MemoryRegionOps s32k3_flash_ops = {
    .read = s32k3_flash_read,
    .write = s32k3_flash_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void s32k3_flash_init(Object *obj)
{
    S32K3FlashState *s = S32K3_FLASH(obj);

    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);

    memory_region_init_io(&s->iomem, obj, &s32k3_flash_ops, s,
                          TYPE_S32K3_FLASH, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static void s32k3_flash_realize(DeviceState *dev, Error **errp)
{
    S32K3FlashState *s = S32K3_FLASH(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_flash: module_clk must be connected");
        return;
    }
    s32k3_flash_reset(dev);
}

static void s32k3_flash_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_flash_reset);
    dc->realize = s32k3_flash_realize;
    dc->desc = "NXP S32K3xx c40asf Flash command interface (FLASH0)";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo s32k3_flash_types[] = {
    {
        .name          = TYPE_S32K3_FLASH,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3FlashState),
        .instance_init = s32k3_flash_init,
        .class_init    = s32k3_flash_class_init,
    },
};

DEFINE_TYPES(s32k3_flash_types)
