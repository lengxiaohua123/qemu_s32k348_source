/*
 * NXP S32K3xx PIT (Periodic Interrupt Timer) QEMU device model
 *
 * 4 independent 32-bit down-counter channels with interrupt support,
 * suitable for system tick / delay / RTOS experiments.
 * Register layout per S32K3xx RM (PIT chapter).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/ptimer.h"
#include "hw/core/qdev-clock.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_S32K3_PIT "s32k3-pit"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3PitState, S32K3_PIT)

#define S32K3_PIT_CHANNELS 4

/* registers */
#define PIT_MCR        0x00
#define  MCR_MDIS      (1 << 1)
#define  MCR_FRZ       (1 << 0)
#define  MCR_MDIS_RTI  (1 << 2)   /* RM 68.6.1：bit2 禁用 RTI 定时器 */

#define PIT_CH_BASE    0x100
#define PIT_CH_STRIDE  0x10
#define  TCTRL_TEN     (1 << 0)
#define  TCTRL_TIE     (1 << 1)
#define  TFLG_TIF      (1 << 0)

struct PitTickCtx {
    S32K3PitState *s;
    int n;
};

struct S32K3PitState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock        *module_clk;
    qemu_irq     irq[S32K3_PIT_CHANNELS];

    uint32_t mcr;
    uint32_t ldval[S32K3_PIT_CHANNELS];
    uint32_t tctrl[S32K3_PIT_CHANNELS];
    uint32_t tflg[S32K3_PIT_CHANNELS];
    uint32_t ltmr64l_latch;   /* LTMR64L 锁存值 */
    uint32_t rti_ldval;      /* RTI_LDVAL */
    uint32_t rti_tctrl;      /* RTI_TCTRL */
    uint32_t rti_tflg;       /* RTI_TFLG */
    ptimer_state *rti_timer; /* RTI 独立定时器（RTI_TCTRL.TEN 启动） */

    ptimer_state *timer[S32K3_PIT_CHANNELS];
    struct PitTickCtx ctx[S32K3_PIT_CHANNELS];
};

static void s32k3_pit_update_irq(S32K3PitState *s, int n);

/* RTI 到期：置 RTI_TFLG.TIF + 中断（RTI 中断并入 PIT0 IRQ96） */
static void s32k3_pit_rti_expire(void *opaque)
{
    S32K3PitState *s = opaque;

    s->rti_tflg |= TFLG_TIF;
    s32k3_pit_update_irq(s, 0);
    /* 周期模式：重装载（ptimer_tick 已持事务——不包 begin） */
    ptimer_set_limit(s->rti_timer, s->rti_ldval ? s->rti_ldval : 1, 1);
    ptimer_run(s->rti_timer, 1);
}

static void s32k3_pit_update_irq(S32K3PitState *s, int n)
{
    bool rti_irq = (n == 0) &&
                   (s->rti_tctrl & TCTRL_TIE) && (s->rti_tflg & TFLG_TIF);
    qemu_set_irq(s->irq[n],
                 ((s->tctrl[n] & TCTRL_TIE) && (s->tflg[n] & TFLG_TIF)) ||
                 rti_irq);
}

static void s32k3_pit_expire(void *opaque)
{
    struct PitTickCtx *ctx = opaque;
    S32K3PitState *s = ctx->s;
    int n = ctx->n;

    s->tflg[n] |= TFLG_TIF;
    s32k3_pit_update_irq(s, n);

    /* 链式（CHN）：timer n+1 链到本 timer 时，本 timer 每次到期
     * 给 timer n+1 一次递减脉冲（简化：直接让其计数减一）。 */
    if (n + 1 < S32K3_PIT_CHANNELS && (s->tctrl[n + 1] & (1 << 2))) {
        uint64_t cnt = ptimer_get_count(s->timer[n + 1]);
        if (cnt > 0) {
            ptimer_set_count(s->timer[n + 1], cnt - 1);
        } else {
            /* 链式 timer 到期：置其 TIF 并 reload */
            s->tflg[n + 1] |= TFLG_TIF;
            s32k3_pit_update_irq(s, n + 1);
            ptimer_set_limit(s->timer[n + 1],
                             s->ldval[n + 1] ? s->ldval[n + 1] : 1, 1);
        }
    }

    /* periodic reload.
     * NB: ptimer_tick() already holds the transaction when it calls us,
     * so do NOT wrap ptimer_set_limit()/ptimer_run() in
     * ptimer_transaction_begin()/commit() here (would assert). */
    ptimer_set_limit(s->timer[n], s->ldval[n] ? s->ldval[n] : 1, 1);
    ptimer_run(s->timer[n], 1);
}

static void s32k3_pit_timer_config(S32K3PitState *s, int n)
{
    uint64_t hz = clock_get_hz(s->module_clk);

    ptimer_transaction_begin(s->timer[n]);
    ptimer_set_freq(s->timer[n], hz ? hz : 1);
    /* FRZ：调试冻结（RM：仅 Debug 模式生效——QEMU 无调试器，忽略）。
     * 固件写 FRZ=1 正常运行时定时器照跑——原 !FRZ 条件导致冻结。 */
    if ((s->tctrl[n] & TCTRL_TEN) && !(s->mcr & MCR_MDIS)) {
        ptimer_set_limit(s->timer[n], s->ldval[n] ? s->ldval[n] : 1, 1);
        ptimer_run(s->timer[n], 1);
    } else {
        ptimer_stop(s->timer[n]);
    }
    ptimer_transaction_commit(s->timer[n]);
}

static void s32k3_pit_reset(DeviceState *dev)
{
    S32K3PitState *s = S32K3_PIT(dev);
    int i;

    s->mcr = MCR_MDIS;   /* timers disabled after reset */
    s->rti_ldval = 0;
    s->rti_tctrl = 0;
    s->rti_tflg = 0;
    if (s->rti_timer) {
        ptimer_transaction_begin(s->rti_timer);
        ptimer_stop(s->rti_timer);
        ptimer_transaction_commit(s->rti_timer);
    }
    s->ltmr64l_latch = 0;
    for (i = 0; i < S32K3_PIT_CHANNELS; i++) {
        s->ldval[i] = 0;
        s->tctrl[i] = 0;
        s->tflg[i] = 0;
        s32k3_pit_timer_config(s, i);
        qemu_irq_lower(s->irq[i]);
    }
}

static uint64_t s32k3_pit_read(void *opaque, hwaddr addr, unsigned size)
{
    if (size == 8) {
        uint64_t lo = s32k3_pit_read(opaque, addr, 4);
        uint64_t hi = s32k3_pit_read(opaque, addr + 4, 4);
        return lo | (hi << 32);
    }
    if (size == 2) {
        uint32_t full = s32k3_pit_read(opaque, addr & ~3u, 4);
        return (addr & 2) ? (full >> 16) : (full & 0xFFFF);
    }
    if (size == 1) {
        uint32_t full = s32k3_pit_read(opaque, addr & ~3u, 4);
        return (full >> (8 * (addr & 3))) & 0xFF;
    }
    S32K3PitState *s = opaque;
    int n;

    if (addr >= PIT_CH_BASE &&
        addr < PIT_CH_BASE + S32K3_PIT_CHANNELS * PIT_CH_STRIDE) {
        n = (addr - PIT_CH_BASE) / PIT_CH_STRIDE;
        switch ((addr - PIT_CH_BASE) % PIT_CH_STRIDE) {
        case 0x0:
            return s->ldval[n];
        case 0x4:
            return ptimer_get_count(s->timer[n]);
        case 0x8:
            return s->tctrl[n];
        case 0xC:
            return s->tflg[n];
        }
    }

    switch (addr) {
    case PIT_MCR:
        return s->mcr;
    case 0xE0:
        /* LTMR64H: 链式 timer1 的当前值（lifetimer 高 32 位），
         * 读时锁存 timer0 到 LTMR64L */
        s->ltmr64l_latch = ptimer_get_count(s->timer[0]);
        return ptimer_get_count(s->timer[1]);
    case 0xE4:
        /* LTMR64L: 锁存的 timer0 值 */
        return s->ltmr64l_latch;
    case 0xE8:
        /* 保留区（RTI_LDVAL_STAT@0xEC 之前）——固件初始化读，静默返回 0 */
        return 0;
    case 0xEC:
        /* RTI_LDVAL_STAT：RTI 装载完成状态（简化：0=装载已接受） */
        return 0;
    case 0xF0:
        return s->rti_ldval;
    case 0xF4:
        return s->rti_timer ? ptimer_get_count(s->rti_timer)
                            : s->rti_ldval;
    case 0xF8:
        return s->rti_tctrl;
    case 0xFC:
        return s->rti_tflg;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_pit: read of unimplemented reg 0x%03" HWADDR_PRIx "\n",
                      addr);
        return 0;
    }
}

static void s32k3_pit_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    if (size == 8) {
        s32k3_pit_write(opaque, addr, value & 0xFFFFFFFF, 4);
        s32k3_pit_write(opaque, addr + 4, value >> 32, 4);
        return;
    }
    if (size == 2) {
        uint32_t full = s32k3_pit_read(opaque, addr & ~3u, 4);
        uint32_t w = value & 0xFFFF;
        uint32_t merged = (addr & 2) ? ((full & 0xFFFF) | (w << 16))
                                     : ((full & 0xFFFF0000u) | w);
        s32k3_pit_write(opaque, addr & ~3u, merged, 4);
        return;
    }
    if (size == 1) {
        uint32_t full = s32k3_pit_read(opaque, addr & ~3u, 4);
        uint32_t sh = 8 * (addr & 3);
        uint32_t merged = (full & ~(0xFFu << sh)) | ((value & 0xFF) << sh);
        s32k3_pit_write(opaque, addr & ~3u, merged, 4);
        return;
    }
    S32K3PitState *s = opaque;
    uint32_t v = value;
    int n;

    if (addr >= PIT_CH_BASE &&
        addr < PIT_CH_BASE + S32K3_PIT_CHANNELS * PIT_CH_STRIDE) {
        n = (addr - PIT_CH_BASE) / PIT_CH_STRIDE;
        switch ((addr - PIT_CH_BASE) % PIT_CH_STRIDE) {
        case 0x0:
            /* writing LDVAL loads CVAL immediately */
            s->ldval[n] = v;
            ptimer_transaction_begin(s->timer[n]);
            ptimer_set_limit(s->timer[n], v ? v : 1, 1);
            ptimer_transaction_commit(s->timer[n]);
            return;
        case 0x8:
            s->tctrl[n] = v & (TCTRL_TEN | TCTRL_TIE | (1 << 2));  /* +CHN */
            /* per RM: enabling timer also clears TIF */
            if (v & TCTRL_TEN) {
                s->tflg[n] &= ~TFLG_TIF;
            }
            s32k3_pit_timer_config(s, n);
            s32k3_pit_update_irq(s, n);
            return;
        case 0xC:
            s->tflg[n] &= ~v;   /* W1C */
            s32k3_pit_update_irq(s, n);
            return;
        default:
            return;
        }
    }

    switch (addr) {
    case PIT_MCR:
        s->mcr = v & (MCR_MDIS | MCR_FRZ | MCR_MDIS_RTI);
        for (n = 0; n < S32K3_PIT_CHANNELS; n++) {
            s32k3_pit_timer_config(s, n);
        }
        break;
    case 0xEC:
        break;   /* RTI_LDVAL_STAT 只读 */
    case 0xF0:
        s->rti_ldval = v;
        if (s->rti_timer && (s->rti_tctrl & TCTRL_TEN)) {
            ptimer_set_limit(s->rti_timer, v ? v : 1, 1);
        }
        break;
    case 0xF8:
        s->rti_tctrl = v & 0x7;
        if (s->rti_timer) {
            uint64_t rti_hz = clock_get_hz(s->module_clk);
            ptimer_transaction_begin(s->rti_timer);
            ptimer_set_freq(s->rti_timer, rti_hz ? rti_hz : 1);
            if ((s->rti_tctrl & TCTRL_TEN) && !(s->mcr & MCR_MDIS)) {
                ptimer_set_limit(s->rti_timer,
                                 s->rti_ldval ? s->rti_ldval : 1, 1);
                ptimer_run(s->rti_timer, 1);
            } else {
                ptimer_stop(s->rti_timer);
            }
            ptimer_transaction_commit(s->rti_timer);
        }
        break;
    case 0xFC:
        s->rti_tflg &= ~v;   /* W1C */
        s32k3_pit_update_irq(s, 0);
        break;
    case 0xE8:
        break;   /* 保留区写忽略 */
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_pit: write of unimplemented reg 0x%03" HWADDR_PRIx
                      " = 0x%08" PRIx64 "\n", addr, value);
    }
}

static const MemoryRegionOps s32k3_pit_ops = {
    .read = s32k3_pit_read,
    .write = s32k3_pit_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static void s32k3_pit_init(Object *obj)
{
    S32K3PitState *s = S32K3_PIT(obj);
    int i;

    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);

    memory_region_init_io(&s->iomem, obj, &s32k3_pit_ops, s,
                          TYPE_S32K3_PIT, 0x200);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);

    for (i = 0; i < S32K3_PIT_CHANNELS; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq[i]);
        s->ctx[i].s = s;
        s->ctx[i].n = i;
        s->timer[i] = ptimer_init(s32k3_pit_expire, &s->ctx[i],
                                  PTIMER_POLICY_LEGACY);
    }
    s->rti_timer = ptimer_init(s32k3_pit_rti_expire, s,
                               PTIMER_POLICY_LEGACY);
}

static void s32k3_pit_realize(DeviceState *dev, Error **errp)
{
    S32K3PitState *s = S32K3_PIT(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_pit: module_clk must be connected");
        return;
    }
    s32k3_pit_reset(dev);
}

static void s32k3_pit_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_pit_reset);
    dc->realize = s32k3_pit_realize;
    dc->desc = "NXP S32K3xx PIT (4 channels)";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo s32k3_pit_types[] = {
    {
        .name          = TYPE_S32K3_PIT,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3PitState),
        .instance_init = s32k3_pit_init,
        .class_init    = s32k3_pit_class_init,
    },
};

DEFINE_TYPES(s32k3_pit_types)
