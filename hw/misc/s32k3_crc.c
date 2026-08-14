/*
 * S32K3 CRC (hw/misc/s32k3_crc.c) — Cyclic Redundancy Check
 *
 * 按 S32K3xx RM Ch58 实现：DATA/GPOLY/CTRL 三寄存器，
 * 支持 16/32 位 CRC、可编程多项式、种子（WAS）、转置（TOT/TOTR）、
 * 读补码（FXOR）。模型用逐位 CRC 引擎按配置实时计算。
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/misc/s32k3_crc.h"

#define CRC_DATA   0x00u
#define CRC_GPOLY  0x04u
#define CRC_CTRL   0x08u

#define CTRL_TOT_MASK   (3u << 30)
#define CTRL_TOTR_MASK  (3u << 28)
#define CTRL_FXOR       (1u << 26)
#define CTRL_WAS        (1u << 25)
#define CTRL_TCRC       (1u << 24)

static uint64_t s32k3_crc_read(void *opaque, hwaddr offset, unsigned size)
{
    if (size == 8) {
        uint64_t lo = s32k3_crc_read(opaque, offset, 4);
        uint64_t hi = s32k3_crc_read(opaque, offset + 4, 4);
        return lo | (hi << 32);
    }
    if (size == 2) {
        uint32_t full = s32k3_crc_read(opaque, offset & ~3u, 4);
        return (offset & 2) ? (full >> 16) : (full & 0xFFFF);
    }
    if (size == 1) {
        uint32_t full = s32k3_crc_read(opaque, offset & ~3u, 4);
        return (full >> (8 * (offset & 3))) & 0xFF;
    }
    S32K3CRCState *s = opaque;
    uint32_t v = 0;

    switch (offset) {
    case CRC_DATA:
        if (s->ctrl & CTRL_WAS) {
            v = s->seed;
        } else {
            v = s->crc;
            if (s->ctrl & CTRL_FXOR) {
                v = ~v;
            }
        }
        break;
    case CRC_GPOLY:
        v = s->gpoly;
        break;
    case CRC_CTRL:
        v = s->ctrl;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad offset 0x%lx\n",
                      __func__, (unsigned long)offset);
        break;
    }
    return v;
}

/* 转置（S32K3: TOT=01 bit-in-byte 翻转；10 bit+byte 翻转；11 byte 翻转） */
static uint32_t crc_transpose(uint32_t v, uint32_t tot, bool is32)
{
    switch (tot) {
    case 1: /* bits in bytes reflected */
        v = ((v & 0x0F0F0F0Fu) << 4) | ((v & 0xF0F0F0F0u) >> 4);
        v = ((v & 0x33333333u) << 2) | ((v & 0xCCCCCCCCu) >> 2);
        v = ((v & 0x55555555u) << 1) | ((v & 0xAAAAAAAAu) >> 1);
        break;
    case 2: /* bits + bytes reflected */
        v = __builtin_bswap32(v);
        v = ((v & 0x0F0F0F0Fu) << 4) | ((v & 0xF0F0F0F0u) >> 4);
        v = ((v & 0x33333333u) << 2) | ((v & 0xCCCCCCCCu) >> 2);
        v = ((v & 0x55555555u) << 1) | ((v & 0xAAAAAAAAu) >> 1);
        break;
    case 3: /* bytes reflected */
        v = __builtin_bswap32(v);
        break;
    default:
        break;
    }
    if (!is32) {
        v &= 0xFFFFu;
    }
    return v;
}

static void s32k3_crc_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    S32K3CRCState *s = opaque;
    uint32_t v = (uint32_t)value;
    if (size == 8) {
        s32k3_crc_write(opaque, offset, value & 0xFFFFFFFF, 4);
        s32k3_crc_write(opaque, offset + 4, value >> 32, 4);
        return;
    }
    if (size == 2 || size == 1) {
        uint32_t full = s32k3_crc_read(opaque, offset & ~3u, 4);
        uint32_t sh = 8 * (offset & 3);
        uint32_t wmask = (size == 1) ? 0xFFu : 0xFFFFu;
        uint32_t merged = (full & ~(wmask << sh)) | ((value & wmask) << sh);
        s32k3_crc_write(opaque, offset & ~3u, merged, 4);
        return;
    }

    switch (offset) {
    case CRC_DATA:
        if (s->ctrl & CTRL_WAS) {
            /* 写种子 */
            s->seed = crc_transpose(v, (s->ctrl & CTRL_TOT_MASK) >> 30,
                                    s->ctrl & CTRL_TCRC) & 0xFFFFFFFFu;
            s->crc = s->seed;
        } else {
            /* 数据输入：逐位 CRC 计算 */
            uint32_t width = (s->ctrl & CTRL_TCRC) ? 32u : 16u;
            uint32_t poly = s->gpoly & ((width == 32) ? 0xFFFFFFFFu : 0xFFFFu);
            uint32_t crc = s->crc & ((width == 32) ? 0xFFFFFFFFu : 0xFFFFu);
            uint32_t din = crc_transpose(v, (s->ctrl & CTRL_TOT_MASK) >> 30,
                                         s->ctrl & CTRL_TCRC);
            uint32_t i;
            for (i = 0; i < 32; i++) {
                uint32_t msb = crc >> (width - 1);
                crc <<= 1;
                if (msb ^ ((din >> (31 - i)) & 1u)) {
                    crc ^= poly;
                }
                crc &= (width == 32) ? 0xFFFFFFFFu : 0xFFFFu;
            }
            s->crc = crc;
        }
        break;
    case CRC_GPOLY:
        s->gpoly = v;
        break;
    case CRC_CTRL:
        s->ctrl = v & (CTRL_TOT_MASK | CTRL_TOTR_MASK | CTRL_FXOR |
                       CTRL_WAS | CTRL_TCRC);
        if (s->ctrl & CTRL_WAS) {
            s->crc = s->seed;
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad offset 0x%lx\n",
                      __func__, (unsigned long)offset);
        break;
    }
}

static const MemoryRegionOps s32k3_crc_ops = {
    .read = s32k3_crc_read,
    .write = s32k3_crc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static void s32k3_crc_reset(DeviceState *dev)
{
    S32K3CRCState *s = S32K3_CRC(dev);
    s->ctrl = 0;
    s->gpoly = 0x00001021u;
    s->seed = 0xFFFFFFFFu;
    s->crc = 0xFFFFFFFFu;
}

static void s32k3_crc_realize(DeviceState *dev, Error **errp)
{
    S32K3CRCState *s = S32K3_CRC(dev);
    memory_region_init_io(&s->iomem, OBJECT(dev), &s32k3_crc_ops, s,
                          TYPE_S32K3_CRC, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void s32k3_crc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    device_class_set_legacy_reset(dc, s32k3_crc_reset);
    dc->realize = s32k3_crc_realize;
}

static const TypeInfo s32k3_crc_info = {
    .name = TYPE_S32K3_CRC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(S32K3CRCState),
    .class_init = s32k3_crc_class_init,
};

static void s32k3_crc_register_types(void)
{
    type_register_static(&s32k3_crc_info);
}
type_init(s32k3_crc_register_types)
