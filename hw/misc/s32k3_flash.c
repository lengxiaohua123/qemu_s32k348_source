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

extern uint32_t s32k3_pfc_pealr;
#include "hw/core/sysbus.h"
#include "hw/core/qdev-clock.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/timer.h"
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
/* S32K348.h 权威：MCRS.DONE=bit15（FLASH_MCRS_DONE_SHIFT=15）、
 * PEG=bit14=Program/Erase Good（成功标志，RM bit14）。348 与 344
 * 布局一致——DONE 只置 bit15（原错误地另置 bit1，348 无此含义）。 */
#define  MCRS_DONE      (1 << 15)
#define  MCRS_OK14      (1 << 14)  /* PEG: Program/Erase Good（C40_Ip 判成功） */
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

/* 三个 flash 分区（S32K348 memory map）：
 * codeflash 0x00400000 8MB（4x2MB，扇区 32KB）
 * dataflash 0x10000000 128KB（扇区 8KB）
 * utest(testnvm) 0x1B000000 8KB（单扇区 8KB） */
#define S32K348_DFLASH_BASE   0x10000000u
#define S32K348_DFLASH_SIZE   (128u * 1024u)
#define S32K348_UTEST_BASE    0x1B000000u
#define S32K348_UTEST_SIZE    (8u * 1024u)

static bool s32k3_flash_addr_valid(uint32_t addr)
{
    return (addr >= S32K348_FLASH0_ARRAY &&
            addr < S32K348_FLASH0_ARRAY + 4u * 2u * 1024u * 1024u) ||
           (addr >= S32K348_DFLASH_BASE &&
            addr < S32K348_DFLASH_BASE + S32K348_DFLASH_SIZE) ||
           (addr >= S32K348_UTEST_BASE &&
            addr < S32K348_UTEST_BASE + S32K348_UTEST_SIZE);
}

static uint32_t s32k3_flash_sector_mask(uint32_t addr)
{
    if (addr >= S32K348_DFLASH_BASE &&
        addr < S32K348_DFLASH_BASE + S32K348_DFLASH_SIZE) {
        return 0x1FFFu;   /* dataflash 扇区 8KB */
    }
    if (addr >= S32K348_UTEST_BASE &&
        addr < S32K348_UTEST_BASE + S32K348_UTEST_SIZE) {
        return 0x1FFFu;   /* utest 8KB 整区 */
    }
    return 0x7FFFu;       /* codeflash 扇区 32KB */
}

/* program: write DATA0-31 to flash array at PEADR（绝对地址，支持三分区） */
static void s32k3_flash_program(S32K3FlashState *s)
{
    /* c40asf 编程：DATA0-31(32B) 从 PEADR 对齐地址(&~0x1F)写入；
     * DATA 写位置 = (LogicalAddress & 0x1F)>>2（bootloader 每帧填
     * 2 word 到递增位置，编程覆盖对齐块，未填位置 0xFF 无影响） */
    uint32_t addr = (s->peadr ? s->peadr : s32k3_pfc_pealr) & ~0x1Fu;
    int i;

    if (!s32k3_flash_addr_valid(addr)) {
        s->mcre |= MCRE_ERR;
        return;
    }
    for (i = 0; i < FL_DATA_COUNT; i++) {
        address_space_write(&address_space_memory, addr + 4 * i,
                            MEMTXATTRS_UNSPECIFIED,
                            &s->data[i], 4);
    }
    s->mcre &= ~MCRE_ERR;
    /* 烧写进度日志（bootloader 全量验证用） */
    if (addr >= 0x500000 && addr < 0x510000) {
        static int bl_cnt;
        if ((++bl_cnt % 500) == 1) {
            fprintf(stderr, "[BL-FLASH] t=%lld prog @%06x cnt=%d\n",
                    (long long)qemu_clock_get_ns(QEMU_CLOCK_REALTIME) / 1000000,
                    (unsigned)(addr - 0x500000), bl_cnt);
        }
    }
}

/* sector erase: fill sector (code 32KB / data 8KB / utest 8KB) with 0xFF */
static void s32k3_flash_sector_erase(S32K3FlashState *s)
{
    uint32_t peadr = s->peadr ? s->peadr : s32k3_pfc_pealr;
    uint32_t mask = s32k3_flash_sector_mask(peadr);
    uint32_t base = peadr & ~mask;
    uint32_t size = mask + 1;
    uint8_t ff = 0xFF;
    uint32_t off;

    if (!s32k3_flash_addr_valid(peadr)) {
        s->mcre |= MCRE_ERR;
        return;
    }
    for (off = 0; off < size; off++) {
        address_space_write(&address_space_memory, base + off,
                            MEMTXATTRS_UNSPECIFIED, &ff, 1);
    }
    s->mcre &= ~MCRE_ERR;
}

static uint64_t s32k3_flash_read(void *opaque, hwaddr addr, unsigned size)
{
    if (size == 8) {
        uint64_t lo = s32k3_flash_read(opaque, addr, 4);
        uint64_t hi = s32k3_flash_read(opaque, addr + 4, 4);
        return lo | (hi << 32);
    }
    if (size == 2) {
        uint32_t full = s32k3_flash_read(opaque, addr & ~3u, 4);
        return (addr & 2) ? (full >> 16) : (full & 0xFFFF);
    }
    if (size == 1) {
        uint32_t full = s32k3_flash_read(opaque, addr & ~3u, 4);
        return (full >> (8 * (addr & 3))) & 0xFF;
    }
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
    case 0x300:
        return s->peadr;
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
    if (size == 8) {
        s32k3_flash_write(opaque, addr, value & 0xFFFFFFFF, 4);
        s32k3_flash_write(opaque, addr + 4, value >> 32, 4);
        return;
    }
    if (size == 2) {
        uint32_t full = s32k3_flash_read(opaque, addr & ~3u, 4);
        uint32_t w = value & 0xFFFF;
        uint32_t merged = (addr & 2) ? ((full & 0xFFFF) | (w << 16))
                                     : ((full & 0xFFFF0000u) | w);
        s32k3_flash_write(opaque, addr & ~3u, merged, 4);
        return;
    }
    if (size == 1) {
        uint32_t full = s32k3_flash_read(opaque, addr & ~3u, 4);
        uint32_t sh = 8 * (addr & 3);
        uint32_t merged = (full & ~(0xFFu << sh)) | ((value & 0xFF) << sh);
        s32k3_flash_write(opaque, addr & ~3u, merged, 4);
        return;
    }
    S32K3FlashState *s = opaque;
    uint32_t v = value;

    switch (addr) {
    case FL_MCR:
        s->mcr = v;
        /* EHV 触发：S32K348 = bit31，S32K344(C40_Ip) = bit0 */
        if ((v & MCR_EHV) || (v & 0x01)) {
            /* 命令位：S32K348 PGM=bit20/ERS=bit19，S32K344 PGM=bit8/ERS=bit4 */
            if (v & (MCR_PGM | 0x100)) {
                s32k3_flash_program(s);
            } else if (v & (MCR_ERS | 0x10)) {
                s32k3_flash_sector_erase(s);
            }
            /* DONE 清后立即置位（即时完成）。
             * S32K348.h：MCRS.DONE=bit15；C40_Ip 成功条件
             * (MCRS & 0x34000)==0x4000（PEG(bit14)=1 且 PEP/PES=0）。 */
            s->mcrs &= ~(MCRS_DONE | 0x34000);
            s->mcrs |= MCRS_DONE | MCRS_OK14;
            s->mcr &= ~MCR_EHV;   /* EHV 自清除（S32K348） */
        }
        break;
    case FL_MCRE:
        s->mcre &= ~v;   /* W1C */
        break;
    case FL_ADR:
        s->adr = v;
        /* S32K348 flash 分区达 0x1B000000（utest），需 29 位地址 */
        s->peadr = v & 0x1FFFFFFF;
        break;
    case FL_XPEADR:
        s->xpeadr = v;
        break;
    case 0x300:   /* S32K344 c40asf PEADR（C40_Ip WriteJobAddress 写地址） */
        s->peadr = v & 0x1FFFFFFF;
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
    .valid = { .min_access_size = 1, .max_access_size = 8 },
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
