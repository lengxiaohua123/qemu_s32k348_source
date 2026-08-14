/*
 * S32K3 RTC (hw/misc/s32k3_rtc.c) — Real Time Clock
 *
 * 按 S32K3xx RM Ch69 实现：RTCSUPV/RTCC/RTCS/RTCCNT/APIVAL/RTCVAL。
 * RTCCNT 32 位秒计数（CNTEN 使能时按虚拟 1Hz 递增），
 * 匹配 RTCVAL 置 RTCS[TIF]（W1C）并触发中断（TIE=1）。
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/ptimer.h"
#include "hw/core/irq.h"
#include "hw/misc/s32k3_rtc.h"

#define RTC_RTCSUPV  0x00u
#define RTC_RTCC     0x04u
#define RTC_RTCS     0x08u
#define RTC_RTCCNT   0x0Cu
#define RTC_APIVAL   0x10u
#define RTC_RTCVAL   0x14u

#define RTCC_CNTEN   (1u << 31)
#define RTCC_TIE     (1u << 30)
#define RTCC_FREEZE  (1u << 29)
#define RTCC_APIE    (1u << 14)
#define RTCC_CLKSEL  (3u << 0)

#define RTCS_TIF     (1u << 29)
#define RTCS_APIF    (1u << 13)
#define RTCS_W1C     (RTCS_TIF | RTCS_APIF)

static void s32k3_rtc_update_irq(S32K3RTCState *s)
{
    bool irq = false;
    if (s->rtcc & RTCC_TIE) {
        irq |= (s->rtcs & RTCS_TIF) != 0;
    }
    if (s->rtcc & RTCC_APIE) {
        irq |= (s->rtcs & RTCS_APIF) != 0;
    }
    qemu_set_irq(s->irq, irq);
}

static void s32k3_rtc_tick(void *opaque)
{
    S32K3RTCState *s = opaque;

    if (!(s->rtcc & RTCC_CNTEN)) {
        return;
    }
    s->rtccnt++;

    if (s->rtccnt == s->rtcval) {
        s->rtcs |= RTCS_TIF;
        s32k3_rtc_update_irq(s);
    }
    if (s->rtccnt == s->apival) {
        s->rtcs |= RTCS_APIF;
        s32k3_rtc_update_irq(s);
    }
    ptimer_set_count(s->ptimer, 1);
    ptimer_run(s->ptimer, 1);
}

static uint64_t s32k3_rtc_read(void *opaque, hwaddr offset, unsigned size)
{
    if (size == 8) {
        uint64_t lo = s32k3_rtc_read(opaque, offset, 4);
        uint64_t hi = s32k3_rtc_read(opaque, offset + 4, 4);
        return lo | (hi << 32);
    }
    if (size == 2) {
        uint32_t full = s32k3_rtc_read(opaque, offset & ~3u, 4);
        return (offset & 2) ? (full >> 16) : (full & 0xFFFF);
    }
    if (size == 1) {
        uint32_t full = s32k3_rtc_read(opaque, offset & ~3u, 4);
        return (full >> (8 * (offset & 3))) & 0xFF;
    }
    S32K3RTCState *s = opaque;
    uint32_t v = 0;

    switch (offset) {
    case RTC_RTCSUPV:
        v = s->rtcsupv;
        break;
    case RTC_RTCC:
        v = s->rtcc;
        break;
    case RTC_RTCS:
        v = s->rtcs;
        break;
    case RTC_RTCCNT:
        v = s->rtccnt;
        break;
    case RTC_APIVAL:
        v = s->apival;
        break;
    case RTC_RTCVAL:
        v = s->rtcval;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad offset 0x%lx\n",
                      __func__, (unsigned long)offset);
        break;
    }
    return v;
}

static void s32k3_rtc_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    S32K3RTCState *s = opaque;
    uint32_t v = (uint32_t)value;
    if (size == 8) {
        s32k3_rtc_write(opaque, offset, value & 0xFFFFFFFF, 4);
        s32k3_rtc_write(opaque, offset + 4, value >> 32, 4);
        return;
    }
    if (size == 2 || size == 1) {
        uint32_t full = s32k3_rtc_read(opaque, offset & ~3u, 4);
        uint32_t sh = 8 * (offset & 3);
        uint32_t wmask = (size == 1) ? 0xFFu : 0xFFFFu;
        uint32_t merged = (full & ~(wmask << sh)) | ((value & wmask) << sh);
        s32k3_rtc_write(opaque, offset & ~3u, merged, 4);
        return;
    }

    switch (offset) {
    case RTC_RTCSUPV:
        s->rtcsupv = v & (1u << 31);
        break;
    case RTC_RTCC:
        s->rtcc = v & (RTCC_CNTEN | RTCC_TIE | RTCC_FREEZE |
                       RTCC_APIE | RTCC_CLKSEL);
        ptimer_transaction_begin(s->ptimer);
        if (s->rtcc & RTCC_CNTEN) {
            ptimer_set_count(s->ptimer, 1);
            ptimer_run(s->ptimer, 1);
        } else {
            ptimer_stop(s->ptimer);
        }
        ptimer_transaction_commit(s->ptimer);
        s32k3_rtc_update_irq(s);
        break;
    case RTC_RTCS:
        s->rtcs &= ~(v & RTCS_W1C);
        s32k3_rtc_update_irq(s);
        break;
    case RTC_APIVAL:
        s->apival = v;
        break;
    case RTC_RTCVAL:
        s->rtcval = v;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad offset 0x%lx\n",
                      __func__, (unsigned long)offset);
        break;
    }
}

static const MemoryRegionOps s32k3_rtc_ops = {
    .read = s32k3_rtc_read,
    .write = s32k3_rtc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static void s32k3_rtc_reset(DeviceState *dev)
{
    S32K3RTCState *s = S32K3_RTC(dev);
    s->rtcsupv = 0x80000000u;
    s->rtcc = 0;
    s->rtcs = 0;
    s->rtccnt = 0;
    s->apival = 0;
    s->rtcval = 0;
    ptimer_transaction_begin(s->ptimer);
    ptimer_stop(s->ptimer);
    ptimer_transaction_commit(s->ptimer);
}

static void s32k3_rtc_realize(DeviceState *dev, Error **errp)
{
    S32K3RTCState *s = S32K3_RTC(dev);
    memory_region_init_io(&s->iomem, OBJECT(dev), &s32k3_rtc_ops, s,
                          TYPE_S32K3_RTC, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->ptimer = ptimer_init(s32k3_rtc_tick, s, PTIMER_POLICY_LEGACY);
    ptimer_transaction_begin(s->ptimer);
    ptimer_set_freq(s->ptimer, 1); /* 虚拟 1 Hz（RTC 秒计数） */
    ptimer_transaction_commit(s->ptimer);
}

static void s32k3_rtc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    device_class_set_legacy_reset(dc, s32k3_rtc_reset);
    dc->realize = s32k3_rtc_realize;
}

static const TypeInfo s32k3_rtc_info = {
    .name = TYPE_S32K3_RTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(S32K3RTCState),
    .class_init = s32k3_rtc_class_init,
};

static void s32k3_rtc_register_types(void)
{
    type_register_static(&s32k3_rtc_info);
}
type_init(s32k3_rtc_register_types)
