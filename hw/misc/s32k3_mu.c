/*
 * NXP S32K3xx Messaging Unit (MU) QEMU device model
 *
 * MU_0 @ 0x4038C000 (MUB side, used by HSE_B), MU_1 @ 0x404EC000.
 * The MU provides transmit/receive registers used to pass HSE service
 * requests and responses. This model stores the registers and reports
 * the status bits firmware expects (ready/empty), so security firmware
 * that pokes HSE services via the MU does not hang or fault.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"

void s32k3_mu_set_peer(DeviceState *dev_a, DeviceState *dev_b);
#include "hw/core/qdev-clock.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_S32K3_MU "s32k3-mu"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3MuState, S32K3_MU)

/* MU registers（S32K3 RM Ch40：CR@0x08/SR@0x0C/TSR/RSR/TR@0x200/RR@0x280） */
#define MU_VER          0x000
#define MU_PAR          0x004
#define MU_CR           0x008
#define MU_SR           0x00C
#define MU_FCR          0x100
#define MU_FSR          0x104
#define MU_GIER         0x110
#define MU_GCR          0x114
#define MU_GSR          0x118
#define MU_TCR          0x120
#define MU_TSR          0x124
#define MU_RCR          0x128
#define MU_RSR          0x12C
#define MU_TR(n)        (0x200 + 0x10 * (n))  /* transmit 0-3 */
#define MU_RR(n)        (0x280 + 0x10 * (n))  /* receive 0-3 */
/* SR 位（bit0 MURS 复位状态、bit5 TEP、bit6 RFP） */
#define  SR_MURS        (1 << 0)
#define  SR_TEP         (1 << 5)
#define  SR_RFP         (1 << 6)
/* TSR/RSR 位（TE0-3 / RF0-3 低位） */
#define  TSR_TEn(n)     (1 << (n))
#define  RSR_RFn(n)     (1 << (n))

struct S32K3MuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock *module_clk;

    uint32_t cr;
    uint32_t sr;
    uint32_t gcr;
    uint32_t tsr;
    uint32_t rsr;
    uint32_t tr[4];
    uint32_t rr[4];

    qemu_irq irq;
    S32K3MuState *peer;   /* 对端 MU（双核互联：TR 写 -> peer RR + IRQ） */
};

static void s32k3_mu_reset(DeviceState *dev)
{
    S32K3MuState *s = S32K3_MU(dev);

    s->cr = 0;
    s->sr = SR_MURS | SR_TEP;   /* 复位状态 + TX 空 */
    s->gcr = 0;
    s->tsr = 0x0000000F;        /* TE0-3 = 1（手册 TSR 复位 0000_000Fh） */
    s->rsr = 0;
    memset(s->tr, 0, sizeof(s->tr));
    memset(s->rr, 0, sizeof(s->rr));
}

static uint64_t s32k3_mu_read(void *opaque, hwaddr addr, unsigned size)
{
    if (size == 8) {
        uint64_t lo = s32k3_mu_read(opaque, addr, 4);
        uint64_t hi = s32k3_mu_read(opaque, addr + 4, 4);
        return lo | (hi << 32);
    }
    if (size == 2) {
        uint32_t full = s32k3_mu_read(opaque, addr & ~3u, 4);
        return (addr & 2) ? (full >> 16) : (full & 0xFFFF);
    }
    if (size == 1) {
        uint32_t full = s32k3_mu_read(opaque, addr & ~3u, 4);
        return (full >> (8 * (addr & 3))) & 0xFF;
    }
    S32K3MuState *s = opaque;

    switch (addr) {
    case MU_VER:  return 0x0309000F;
    case MU_PAR:  return 0x03010404;
    case MU_CR:   return s->cr;
    case MU_SR:   return s->sr;
    case MU_TSR:  return s->tsr;
    case MU_RSR:  return s->rsr;
    case MU_GCR:  return s->gcr;
    case MU_TCR:  return 0;
    case MU_RCR:  return 0;
    case MU_FCR:  return 0;
    case MU_GIER: return 0;
    default:
        if (addr >= MU_TR(0) && addr < MU_TR(0) + 0x40) {
            return s->tr[(addr - MU_TR(0)) / 0x10];
        }
        if (addr >= MU_RR(0) && addr < MU_RR(0) + 0x40) {
            int i = (addr - MU_RR(0)) / 0x10;
            uint32_t v = s->rr[i];
            /* 读走数据：清 RSR RFn 并降 IRQ */
            s->rsr &= ~RSR_RFn(i);
            qemu_set_irq(s->irq, 0);
            return v;
        }
        return 0;
    }
}

static void s32k3_mu_write(void *opaque, hwaddr addr,
                           uint64_t value, unsigned size)
{
    if (size == 8) {
        s32k3_mu_write(opaque, addr, value & 0xFFFFFFFF, 4);
        s32k3_mu_write(opaque, addr + 4, value >> 32, 4);
        return;
    }
    if (size == 2) {
        uint32_t full = s32k3_mu_read(opaque, addr & ~3u, 4);
        uint32_t w = value & 0xFFFF;
        uint32_t merged = (addr & 2) ? ((full & 0xFFFF) | (w << 16))
                                     : ((full & 0xFFFF0000u) | w);
        s32k3_mu_write(opaque, addr & ~3u, merged, 4);
        return;
    }
    if (size == 1) {
        uint32_t full = s32k3_mu_read(opaque, addr & ~3u, 4);
        uint32_t sh = 8 * (addr & 3);
        uint32_t merged = (full & ~(0xFFu << sh)) | ((value & 0xFF) << sh);
        s32k3_mu_write(opaque, addr & ~3u, merged, 4);
        return;
    }
    S32K3MuState *s = opaque;
    uint32_t v = value;

    switch (addr) {
    case MU_CR:
        s->cr = v & 0x3F;
        break;
    case MU_SR:
        s->sr &= ~(v & (SR_TEP | SR_RFP));   /* TEP/RFP W1C */
        break;
    case MU_GCR:
        s->gcr = v;
        break;
    case MU_TCR:
    case MU_RCR:
    case MU_FCR:
    case MU_GIER:
        break;
    case MU_TR(0): case MU_TR(1): case MU_TR(2): case MU_TR(3):
    {
        int i = (addr - MU_TR(0)) / 0x10;
        /* 发送：清 TSR TEi，置对端 RSR RFi（对端收到） */
        s->tr[i] = v;
        s->tsr &= ~TSR_TEn(i);
        if (s->peer) {
            s->peer->rr[i] = v;
            s->peer->rsr |= RSR_RFn(i);
            s->peer->sr |= SR_RFP;
            qemu_set_irq(s->peer->irq, 1);
        }
        break;
    }
    case MU_RR(0): case MU_RR(1): case MU_RR(2): case MU_RR(3):
        s->rr[(addr - MU_RR(0)) / 0x10] = v;
        s->rsr &= ~RSR_RFn((addr - MU_RR(0)) / 0x10);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps s32k3_mu_ops = {
    .read = s32k3_mu_read,
    .write = s32k3_mu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static void s32k3_mu_init(Object *obj)
{
    S32K3MuState *s = S32K3_MU(obj);

    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);

    memory_region_init_io(&s->iomem, obj, &s32k3_mu_ops, s,
                          TYPE_S32K3_MU, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

void s32k3_mu_set_peer(DeviceState *dev_a, DeviceState *dev_b)
{
    S32K3MuState *a = S32K3_MU(dev_a);
    S32K3MuState *b = S32K3_MU(dev_b);

    a->peer = b;
    b->peer = a;
}

static void s32k3_mu_realize(DeviceState *dev, Error **errp)
{
    S32K3MuState *s = S32K3_MU(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_mu: module_clk must be connected");
        return;
    }
    s32k3_mu_reset(dev);
}

static void s32k3_mu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_mu_reset);
    dc->realize = s32k3_mu_realize;
    dc->desc = "NXP S32K3xx Messaging Unit (MU)";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo s32k3_mu_types[] = {
    {
        .name          = TYPE_S32K3_MU,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3MuState),
        .instance_init = s32k3_mu_init,
        .class_init    = s32k3_mu_class_init,
    },
};

DEFINE_TYPES(s32k3_mu_types)
