/*
 * NXP S32K3xx WKPU (Wakeup Unit) + PDAC (Pad Data Access Control)
 * QEMU device model — 寄存器存储实现。
 *
 * WKPU @ 0x402B4000：唤醒控制（MER/IFMR/IWER/IMR/IRER/IFER 等）——
 * QEMU 无低功耗/外部唤醒输入，实现读回写值（固件初始化不卡）。
 * PDAC @ 0x40294000/0x40298000/0x4029C000/0x402A8000：引脚数据访问
 * 控制——纯存储。
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-clock.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_S32K3_WKPU "s32k3-wkpu"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3WkpuState, S32K3_WKPU)

#define WKPU_SIZE 0x1000

struct S32K3WkpuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock        *module_clk;
    uint32_t     regs[WKPU_SIZE / 4];   /* 全寄存器存储 */
};

static uint64_t s32k3_wkpu_read(void *opaque, hwaddr addr, unsigned size)
{
    if (size == 8) {
        uint64_t lo = s32k3_wkpu_read(opaque, addr, 4);
        uint64_t hi = s32k3_wkpu_read(opaque, addr + 4, 4);
        return lo | (hi << 32);
    }
    if (size == 2) {
        uint32_t full = s32k3_wkpu_read(opaque, addr & ~3u, 4);
        return (addr & 2) ? (full >> 16) : (full & 0xFFFF);
    }
    if (size == 1) {
        uint32_t full = s32k3_wkpu_read(opaque, addr & ~3u, 4);
        return (full >> (8 * (addr & 3))) & 0xFF;
    }
    S32K3WkpuState *s = opaque;

    if (addr < WKPU_SIZE) {
        return s->regs[addr / 4];
    }
    return 0;
}

static void s32k3_wkpu_write(void *opaque, hwaddr addr,
                             uint64_t value, unsigned size)
{
    if (size == 8) {
        s32k3_wkpu_write(opaque, addr, value & 0xFFFFFFFF, 4);
        s32k3_wkpu_write(opaque, addr + 4, value >> 32, 4);
        return;
    }
    if (size == 2 || size == 1) {
        uint32_t full = s32k3_wkpu_read(opaque, addr & ~3u, 4);
        uint32_t sh = 8 * (addr & 3);
        uint32_t wmask = (size == 1) ? 0xFFu : 0xFFFFu;
        uint32_t merged = (full & ~(wmask << sh)) | ((value & wmask) << sh);
        s32k3_wkpu_write(opaque, addr & ~3u, merged, 4);
        return;
    }
    S32K3WkpuState *s = opaque;

    if (addr < WKPU_SIZE) {
        s->regs[addr / 4] = value;
    }
}

static const MemoryRegionOps s32k3_wkpu_ops = {
    .read = s32k3_wkpu_read,
    .write = s32k3_wkpu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static void s32k3_wkpu_reset(DeviceState *dev)
{
    S32K3WkpuState *s = S32K3_WKPU(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void s32k3_wkpu_init(Object *obj)
{
    S32K3WkpuState *s = S32K3_WKPU(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &s32k3_wkpu_ops, s,
                          TYPE_S32K3_WKPU, WKPU_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);
}

static void s32k3_wkpu_realize(DeviceState *dev, Error **errp)
{
    if (!S32K3_WKPU(dev)->module_clk) {
        error_setg(errp, "s32k3_wkpu: module_clk must be connected");
        return;
    }
    s32k3_wkpu_reset(dev);
}

static void s32k3_wkpu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_wkpu_reset);
    dc->realize = s32k3_wkpu_realize;
    dc->desc = "NXP S32K3xx WKPU/PDAC register storage";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo s32k3_wkpu_types[] = {
    {
        .name          = TYPE_S32K3_WKPU,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3WkpuState),
        .instance_init = s32k3_wkpu_init,
        .class_init    = s32k3_wkpu_class_init,
    },
};

DEFINE_TYPES(s32k3_wkpu_types)
