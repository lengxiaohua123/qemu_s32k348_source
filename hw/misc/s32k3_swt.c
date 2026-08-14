/*
 * NXP S32K3xx SWT (Software Watchdog Timer) QEMU device model
 *
 * Models the SWT unlock/service sequence and an optional timeout that
 * asserts a reset request.  Default behavior: watchdog can be enabled/
 * serviced exactly like real hardware (fixed + keyed sequences), but
 * timeouts are reported only (FRZ/flag) so firmware that never services
 * the watchdog does not reboot-loop the machine -- matching what most
 * learning examples expect.
 *
 * Register layout per S32K3xx RM (SWT chapter).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/ptimer.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "system/runstate.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_S32K3_SWT "s32k3-swt"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3SwtState, S32K3_SWT)

/* registers */
#define SWT_CR      0x00
#define  CR_WEN     (1 << 0)
#define  CR_FRZ     (1 << 1)
#define  CR_STP     (1 << 2)
#define  CR_SLK     (1 << 4)
#define  CR_HLK     (1 << 5)
#define  CR_ITR     (1 << 6)    /* interrupt then reset request (RM bit6) */
#define  CR_WND     (1 << 7)    /* window mode */
#define  CR_RIA     (1 << 8)    /* reset on invalid access */
#define  CR_SMD_MASK (3 << 9)   /* service mode select [10:9] */
#define SWT_IR      0x04
#define  IR_TIF     (1 << 0)
#define SWT_TO      0x08
#define SWT_WN      0x0C
#define SWT_SR      0x10
#define SWT_CO      0x14
#define SWT_SK      0x18
#define SWT_RRR     0x1C

/* SR[WSC] service codes (RM 66.6.6): unlock + fixed service both go to SR */
#define SR_UNLOCK1  0x0000C520
#define SR_UNLOCK2  0x0000D928
/* service sequence */
#define SR_SERVICE1 0x0000A602
#define SR_SERVICE2 0x0000B480

struct S32K3SwtState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock        *module_clk;
    qemu_irq     irq;

    uint32_t cr;
    uint32_t ir;
    uint32_t to;
    uint32_t wn;
    uint32_t co;
    uint32_t sk;
    uint32_t rrr;

    uint8_t  unlock_state;   /* 0=locked, 1=got first key, 2=unlocked */
    uint8_t  service_state;  /* 0=idle, 1=got first service word */

    ptimer_state *timer;
    bool     reset_on_timeout; /* property, default false */
};

static void s32k3_swt_update_irq(S32K3SwtState *s)
{
    qemu_set_irq(s->irq, (s->ir & IR_TIF) && (s->cr & CR_ITR));
}

static void s32k3_swt_expire(void *opaque)
{
    S32K3SwtState *s = opaque;

    s->ir |= IR_TIF;
    if (s->cr & CR_ITR) {
        s32k3_swt_update_irq(s);
    } else if (s->reset_on_timeout) {
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
    /* no reload: window closed until serviced */
    ptimer_stop(s->timer);
}

static void s32k3_swt_load(S32K3SwtState *s)
{
    uint64_t hz = clock_get_hz(s->module_clk);
    uint32_t period = s->to ? s->to : 1;

    ptimer_transaction_begin(s->timer);
    ptimer_set_freq(s->timer, hz ? hz : 1);
    if (s->cr & CR_WEN) {
        ptimer_set_limit(s->timer, period, 1);
        ptimer_run(s->timer, 0);   /* one-shot */
    } else {
        ptimer_stop(s->timer);
    }
    ptimer_transaction_commit(s->timer);
}

static void s32k3_swt_reset(DeviceState *dev)
{
    S32K3SwtState *s = S32K3_SWT(dev);

    s->cr = 0xFF00010A;      /* RM reset value: MAP=all1, RIA=1, FRZ=1, WEN=0 */
    s->ir = 0;
    s->to = 0x00000320;      /* RM reset timeout (chip TO_RST=320h @ SIRC) */
    s->wn = 0;
    s->co = 0;
    s->sk = 0x100;   /* 初始 service key（keyed 模式） */
    s->rrr = 0xFF;           /* reset by POR */
    s->unlock_state = 0;
    s->service_state = 0;
    s32k3_swt_load(s);
    s32k3_swt_update_irq(s);
}

static bool s32k3_swt_locked(S32K3SwtState *s)
{
    return (s->cr & CR_SLK) && s->unlock_state != 2;
}

static uint64_t s32k3_swt_read(void *opaque, hwaddr addr, unsigned size)
{
    if (size == 8) {
        uint64_t lo = s32k3_swt_read(opaque, addr, 4);
        uint64_t hi = s32k3_swt_read(opaque, addr + 4, 4);
        return lo | (hi << 32);
    }
    if (size == 2) {
        uint32_t full = s32k3_swt_read(opaque, addr & ~3u, 4);
        return (addr & 2) ? (full >> 16) : (full & 0xFFFF);
    }
    S32K3SwtState *s = opaque;

    switch (addr) {
    case SWT_CR:
        return s->cr;
    case SWT_IR:
        return s->ir;
    case SWT_TO:
        return s->to;
    case SWT_WN:
        return s->wn;
    case SWT_SR:
        return 0;
    case SWT_CO:
        return ptimer_get_count(s->timer);
    case SWT_SK:
        /* keyed 模式下返回当前 service key；fixed 模式返回 0 */
        return (s->cr & CR_SMD_MASK) == (1u << 9) ? s->sk : 0;
    case SWT_RRR:
        return s->rrr;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_swt: read of unimplemented reg 0x%03" HWADDR_PRIx "\n",
                      addr);
        return 0;
    }
}

static void s32k3_swt_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    if (size == 8) {
        s32k3_swt_write(opaque, addr, value & 0xFFFFFFFF, 4);
        s32k3_swt_write(opaque, addr + 4, value >> 32, 4);
        return;
    }
    if (size == 2) {
        uint32_t full = s32k3_swt_read(opaque, addr & ~3u, 4);
        uint32_t w = value & 0xFFFF;
        uint32_t merged = (addr & 2) ? ((full & 0xFFFF) | (w << 16))
                                     : ((full & 0xFFFF0000u) | w);
        s32k3_swt_write(opaque, addr & ~3u, merged, 4);
        return;
    }
    S32K3SwtState *s = opaque;
    uint32_t v = value;

    switch (addr) {
    case SWT_CR:
        if (!s32k3_swt_locked(s)) {
            s->cr = v;
            s->unlock_state = 0;
            s32k3_swt_load(s);
            s32k3_swt_update_irq(s);
        }
        break;
    case SWT_IR:
        s->ir &= ~v;      /* W1C TIF */
        s32k3_swt_update_irq(s);
        break;
    case SWT_TO:
        if (!s32k3_swt_locked(s)) {
            s->to = v & 0xffffffff;
            s32k3_swt_load(s);
        }
        break;
    case SWT_WN:
        if (!s32k3_swt_locked(s)) {
            s->wn = v;
        }
        break;
    case SWT_SR:
        /* SR[WSC] handles BOTH the unlock sequence (C520 -> D928,
         * RM 66.6.6) and the fixed service sequence (A602 -> B480). */
        if (v == SR_UNLOCK1 && s->unlock_state == 0) {
            s->unlock_state = 1;
        } else if (v == SR_UNLOCK2 && s->unlock_state == 1) {
            s->unlock_state = 2;
            s->cr &= ~CR_SLK;        /* soft lock cleared */
        }
        /* keyed 服务模式（CR[SMD]=01b）：写入值须等于下一 key
         * (17*SK+3) mod 2^16；连续两次匹配视为服务成功 */
        if ((s->cr & CR_SMD_MASK) == (1u << 9)) {
            uint16_t next = (uint16_t)((17 * (uint32_t)s->sk + 3) & 0xFFFF);
            if (v == next) {
                s->service_state++;
                s->sk = next;
                if (s->service_state >= 2) {
                    s->service_state = 0;
                    s->ir &= ~IR_TIF;
                    s32k3_swt_load(s);
                    s32k3_swt_update_irq(s);
                }
            } else {
                s->service_state = 0;
            }
            break;
        }
        /* service sequence A602 -> B480 */
        if (v == SR_SERVICE1 && s->service_state == 0) {
            s->service_state = 1;
        } else if (v == SR_SERVICE2 && s->service_state == 1) {
            s->service_state = 0;
            /* window mode（CR[WND]=1）：仅当计数 < WN 时才允许服务 */
            if ((s->cr & CR_WND) &&
                (ptimer_get_count(s->timer) > s->wn)) {
                break;   /* 窗口外服务被忽略 */
            }
            s->ir &= ~IR_TIF;
            s32k3_swt_load(s);       /* reload timeout */
            s32k3_swt_update_irq(s);
        } else if (v != SR_SERVICE1 && v != SR_SERVICE2 &&
                   v != SR_UNLOCK1 && v != SR_UNLOCK2) {
            s->service_state = 0;
            s->unlock_state = 0;    /* invalid access resets both seqs */
        }
        break;
    case SWT_CO:
        if (!s32k3_swt_locked(s)) {
            s->co = v;
        }
        break;
    case SWT_SK:
        /* real HW: SK is read-only (holds service key). writes ignored. */
        break;
    case SWT_RRR:
        s->rrr &= ~v;     /* W1C */
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_swt: write of unimplemented reg 0x%03" HWADDR_PRIx
                      " = 0x%08" PRIx64 "\n", addr, value);
    }
}

static const MemoryRegionOps s32k3_swt_ops = {
    .read = s32k3_swt_read,
    .write = s32k3_swt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static void s32k3_swt_init(Object *obj)
{
    S32K3SwtState *s = S32K3_SWT(obj);

    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);

    memory_region_init_io(&s->iomem, obj, &s32k3_swt_ops, s,
                          TYPE_S32K3_SWT, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    s->timer = ptimer_init(s32k3_swt_expire, s, PTIMER_POLICY_LEGACY);
}

static void s32k3_swt_realize(DeviceState *dev, Error **errp)
{
    S32K3SwtState *s = S32K3_SWT(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_swt: module_clk must be connected");
        return;
    }
    s32k3_swt_reset(dev);
}

static const Property s32k3_swt_properties[] = {
    DEFINE_PROP_BOOL("reset-on-timeout", S32K3SwtState,
                     reset_on_timeout, false),
};

static void s32k3_swt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_swt_reset);
    dc->realize = s32k3_swt_realize;
    device_class_set_props(dc, s32k3_swt_properties);
    dc->desc = "NXP S32K3xx SWT (unlock/service sequences, optional reset)";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo s32k3_swt_types[] = {
    {
        .name          = TYPE_S32K3_SWT,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3SwtState),
        .instance_init = s32k3_swt_init,
        .class_init    = s32k3_swt_class_init,
    },
};

DEFINE_TYPES(s32k3_swt_types)
