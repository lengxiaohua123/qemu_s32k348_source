/*
 * NXP S32K3xx DMA Channel Multiplexer (DMAMUX) QEMU device model
 *
 * DMAMUX0 @ 0x40280000, DMAMUX1 @ 0x40284000 (RM 14.x).
 * Each channel has an 8-bit CHCFG register:
 *   bit7 ENBL, bit6 TRIG, bit5:0 SOURCE (peripheral request to route).
 * The multiplexer selects which peripheral request drives each eDMA
 * channel; the actual trigger happens when the peripheral asserts its
 * request line (board-wired to the "periph-req" inputs).
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

#define TYPE_S32K3_DMAMUX "s32k3-dmamux"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3DmamuxState, S32K3_DMAMUX)

#define S32K3_DMAMUX_CHANNELS 32
#define DMAMUX_CHCFG(n)      (n)      /* 0x00..0x1F, byte access */

/* CHCFG bits */
#define CHCFG_ENBL     (1 << 7)
#define CHCFG_TRIG     (1 << 6)
#define CHCFG_SOURCE_MASK 0x3F

struct S32K3DmamuxState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock *module_clk;

    uint8_t chcfg[S32K3_DMAMUX_CHANNELS];
    qemu_irq dma_req[S32K3_DMAMUX_CHANNELS];   /* 输出：到 eDMA 的请求 */
    qemu_irq periph_req[128];                  /* 输入：外设请求线 */
};

/* 外设请求输入：路由到已使能的 DMA 通道 */
static void s32k3_dmamux_req_set(void *opaque, int line, int level)
{
    S32K3DmamuxState *s = opaque;
    int ch;

    if (line < 0) {
        return;
    }
    for (ch = 0; ch < S32K3_DMAMUX_CHANNELS; ch++) {
        if ((s->chcfg[ch] & CHCFG_ENBL) &&
            ((s->chcfg[ch] & CHCFG_SOURCE_MASK) == line)) {
            qemu_set_irq(s->dma_req[ch], level);
        }
    }
}

static void s32k3_dmamux_reset(DeviceState *dev)
{
    S32K3DmamuxState *s = S32K3_DMAMUX(dev);

    memset(s->chcfg, 0, sizeof(s->chcfg));
}

static uint64_t s32k3_dmamux_read(void *opaque, hwaddr addr, unsigned size)
{
    S32K3DmamuxState *s = opaque;

    /* CHCFG 为 8 位寄存器（0x0-0x1F）；32 位访问读低字节对应通道 */
    if (addr < S32K3_DMAMUX_CHANNELS) {
        return s->chcfg[addr];
    }
    return 0;
}

static void s32k3_dmamux_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned size)
{
    S32K3DmamuxState *s = opaque;

    /* 8 位写：单通道；32/16 位写：拆字节写连续通道（固件兼容——原 valid 只
     * 允许 8 位，32 位写会触发 Data Abort——S32K3 固件 Dma_Ip 可能 32 位访问） */
    if (addr >= S32K3_DMAMUX_CHANNELS) {
        return;
    }
    for (unsigned i = 0; i < size && (addr + i) < S32K3_DMAMUX_CHANNELS; i++) {
        s->chcfg[addr + i] = (value >> (8 * i)) & 0xFF;
    }
}

static const MemoryRegionOps s32k3_dmamux_ops = {
    .read = s32k3_dmamux_read,
    .write = s32k3_dmamux_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static void s32k3_dmamux_init(Object *obj)
{
    S32K3DmamuxState *s = S32K3_DMAMUX(obj);

    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);

    memory_region_init_io(&s->iomem, obj, &s32k3_dmamux_ops, s,
                          TYPE_S32K3_DMAMUX, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    qdev_init_gpio_out(DEVICE(s), s->dma_req, S32K3_DMAMUX_CHANNELS);
    qdev_init_gpio_in(DEVICE(s), s32k3_dmamux_req_set, 128);
}

static void s32k3_dmamux_realize(DeviceState *dev, Error **errp)
{
    S32K3DmamuxState *s = S32K3_DMAMUX(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_dmamux: module_clk must be connected");
        return;
    }
    s32k3_dmamux_reset(dev);
}

static void s32k3_dmamux_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_dmamux_reset);
    dc->realize = s32k3_dmamux_realize;
    dc->desc = "NXP S32K3xx DMA Channel Multiplexer";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo s32k3_dmamux_types[] = {
    {
        .name          = TYPE_S32K3_DMAMUX,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3DmamuxState),
        .instance_init = s32k3_dmamux_init,
        .class_init    = s32k3_dmamux_class_init,
    },
};

DEFINE_TYPES(s32k3_dmamux_types)
