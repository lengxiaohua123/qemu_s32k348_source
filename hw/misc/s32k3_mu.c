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

/* MU registers (standard NXP MU) */
#define MU_CR          0x000
#define MU_SR          0x004
#define  SR_RFn(n)     (1 << (20 + n))   /* receive full */
#define  SR_TEn(n)     (1 << (24 + n))   /* transmit empty */
#define  SR_EP         (1 << 28)         /* exception pending */
#define MU_GCR         0x008
#define MU_TR(n)       (0x010 + 4 * (n))  /* transmit 0-3 */
#define MU_RR(n)       (0x020 + 4 * (n))  /* receive 0-3 */

struct S32K3MuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock *module_clk;

    uint32_t cr;
    uint32_t sr;
    uint32_t gcr;
    uint32_t tr[4];
    uint32_t rr[4];

    qemu_irq irq;
    S32K3MuState *peer;   /* 对端 MU（双核互联：TR 写 -> peer RR + IRQ） */
};

static void s32k3_mu_reset(DeviceState *dev)
{
    S32K3MuState *s = S32K3_MU(dev);

    s->cr = 0;
    s->sr = 0x0FF00000;   /* TE0-3 set (tx empty), RF clear */
    s->gcr = 0;
    memset(s->tr, 0, sizeof(s->tr));
    memset(s->rr, 0, sizeof(s->rr));
}

static uint64_t s32k3_mu_read(void *opaque, hwaddr addr, unsigned size)
{
    S32K3MuState *s = opaque;

    switch (addr) {
    case MU_CR:
        return s->cr;
    case MU_SR:
        return s->sr;
    case MU_GCR:
        return s->gcr;
    default:
        if (addr >= MU_TR(0) && addr < MU_TR(0) + 16) {
            return s->tr[(addr - MU_TR(0)) / 4];
        }
        if (addr >= MU_RR(0) && addr < MU_RR(0) + 16) {
            return s->rr[(addr - MU_RR(0)) / 4];
        }
        return 0;
    }
}

static void s32k3_mu_write(void *opaque, hwaddr addr,
                           uint64_t value, unsigned size)
{
    S32K3MuState *s = opaque;
    uint32_t v = value;

    switch (addr) {
    case MU_CR:
        s->cr = v;
        break;
    case MU_GCR:
        s->gcr = v;
        break;
    case MU_TR(0): case MU_TR(1): case MU_TR(2): case MU_TR(3):
    {
        int i = (addr - MU_TR(0)) / 4;
        /* 发送：清 TE，置对应 RF（对端收到） */
        s->tr[i] = v;
        s->sr &= ~SR_TEn(i);
        s->sr |= SR_RFn(i);
        /* 双核互联：数据送到对端 MU 的 RR 并触发其中断 */
        if (s->peer) {
            s->peer->rr[i] = v;
            s->peer->sr |= SR_RFn(i);
            qemu_set_irq(s->peer->irq, 1);
        }
        break;
    }
    case MU_RR(0): case MU_RR(1): case MU_RR(2): case MU_RR(3):
        /* 接收：清 RF（读走） */
        s->rr[(addr - MU_RR(0)) / 4] = v;
        s->sr &= ~SR_RFn((addr - MU_RR(0)) / 4);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps s32k3_mu_ops = {
    .read = s32k3_mu_read,
    .write = s32k3_mu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
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
