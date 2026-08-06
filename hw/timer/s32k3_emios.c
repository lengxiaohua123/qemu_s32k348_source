/*
 * NXP S32K3xx eMIOS (enhanced Modular IO Subsystem) QEMU device model
 *
 * 24 unified channels per instance.  Models the register interface plus
 * OPWMB (PWM output) and MCB (modulus counter) modes, which covers the
 * standard embedded-learning PWM experiments (LED dimming, servo,
 * motor drive).  Each channel is exposed as a qdev gpio output line;
 * channel frequency is derived from the module clock.
 *
 * Register layout per S32K3xx RM (eMIOS chapter).
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

#define TYPE_S32K3_EMIOS "s32k3-emios"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3EmiosState, S32K3_EMIOS)

#define S32K3_EMIOS_CHANNELS 24

/* registers */
#define EMIOS_MCR        0x00
#define  MCR_GPREN       (1 << 26)
#define  MCR_MDIS        (1 << 30)
#define  MCR_FRZ         (1 << 31)
#define  MCR_GPRE_MASK   0xff0000
#define  MCR_GPRE_SHIFT  16
#define EMIOS_GFLAG      0x04
#define EMIOS_OUDIS      0x08
#define EMIOS_UCDIS      0x0C

/* unified channel registers: base 0x20, stride 0x20 */
#define UC_BASE          0x20
#define UC_STRIDE        0x20
#define UC_A(n)          (UC_BASE + (n) * UC_STRIDE + 0x00)
#define UC_B(n)          (UC_BASE + (n) * UC_STRIDE + 0x04)
#define UC_CNT(n)        (UC_BASE + (n) * UC_STRIDE + 0x08)
#define UC_C(n)          (UC_BASE + (n) * UC_STRIDE + 0x0C)
#define  UC_C_UCPREN     (1 << 26)
#define  UC_C_UCPRE_MASK 0x00030000
#define  UC_C_UCPRE_SHIFT 16
#define  UC_C_MODE_MASK  0x7f
#define  UC_C_EDSEL      (1 << 7)
#define  UC_C_EDPOL      (1 << 8)
#define  UC_C_FEN        (1 << 9)
#define  UC_C_ODIS       (1 << 4)
#define  UC_C_BSL_MASK   (3 << 23)
#define  UC_C_BSL_SHIFT  23
#define UC_S(n)          (UC_BASE + (n) * UC_STRIDE + 0x10)
#define  UC_S_FLAG       (1 << 0)
#define  UC_S_OVR        (1 << 16)
#define UC_ALTA(n)       (UC_BASE + (n) * UC_STRIDE + 0x14)

/* channel modes we model */
#define UC_MODE_MCB_UP   0x50   /* modulus counter buffered, up */
#define UC_MODE_OPWMB    0x60   /* output PWM buffered */
#define UC_MODE_OPWMCB   0x5D
#define UC_MODE_SAIC     0x04   /* input capture */
#define UC_MODE_SAOC     0x03   /* output compare */

struct EmiosChCtx {
    S32K3EmiosState *s;
    int n;
};

struct S32K3EmiosState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock        *module_clk;
    qemu_irq     irq[S32K3_EMIOS_CHANNELS];   /* per-channel flags */
    qemu_irq     irq_out[6];                   /* 6 NVIC lines: ch23/19/15/11/7/3 groups */
    qemu_irq     out[S32K3_EMIOS_CHANNELS];

    uint32_t mcr;
    uint32_t gflag;
    uint32_t oudis;
    uint32_t ucdis;

    uint32_t uc_a[S32K3_EMIOS_CHANNELS];
    uint32_t uc_b[S32K3_EMIOS_CHANNELS];
    uint32_t uc_c[S32K3_EMIOS_CHANNELS];
    uint32_t uc_s[S32K3_EMIOS_CHANNELS];
    uint32_t uc_alta[S32K3_EMIOS_CHANNELS];

    ptimer_state *timer[S32K3_EMIOS_CHANNELS];
    struct EmiosChCtx ctx[S32K3_EMIOS_CHANNELS];
    uint8_t  out_level[S32K3_EMIOS_CHANNELS];
    uint8_t  in_level[S32K3_EMIOS_CHANNELS];   /* 输入捕获电平 */
    uint8_t  pwm_phase[S32K3_EMIOS_CHANNELS];  /* OPWMB 阶段 0=低段 1=高段 */
};

static uint32_t s32k3_emios_period(S32K3EmiosState *s, int n)
{
    uint32_t gpre = ((s->mcr & MCR_GPRE_MASK) >> MCR_GPRE_SHIFT) + 1;
    uint32_t ucpre = ((s->uc_c[n] & UC_C_UCPRE_MASK) >> UC_C_UCPRE_SHIFT) + 1;
    uint32_t period = s->uc_a[n] ? s->uc_a[n] : 1;

    return (uint32_t)(((uint64_t)period * ucpre * gpre) & 0xffffffff) ?: 1;
}

static void s32k3_emios_update_irq(S32K3EmiosState *s, int n);

static void s32k3_emios_in_set(void *opaque, int line, int level);

static void s32k3_emios_drive_out(S32K3EmiosState *s, int n, int level)
{
    bool disabled = (s->uc_c[n] & UC_C_ODIS) ||
                    (s->oudis & (1 << n)) ||
                    (s->ucdis & (1 << n));

    s->out_level[n] = disabled ? 0 : (level & 1);
    qemu_set_irq(s->out[n], s->out_level[n]);
}

static void s32k3_emios_ch_expire(void *opaque)
{
    struct EmiosChCtx *ctx = opaque;
    S32K3EmiosState *s = ctx->s;
    int n = ctx->n;
    uint32_t mode = s->uc_c[n] & UC_C_MODE_MASK;

    if (mode == UC_MODE_OPWMB) {
        /* OPWMB 双点翻转（单回调双段）：
         *   phase 0：到期 = 到 B，输出拉高，切 phase 1（计到 A）
         *   phase 1：到期 = 到 A，输出拉低 + FLAG，切 phase 0（计到 B） */
        uint32_t a = s->uc_a[n] ? s->uc_a[n] : 1;
        uint32_t b = s->uc_b[n] > a ? a : s->uc_b[n];
        if (s->pwm_phase[n] == 0) {
            s32k3_emios_drive_out(s, n, 1);
            s->pwm_phase[n] = 1;
            ptimer_set_limit(s->timer[n], a - b ? a - b : 1, 1);
        } else {
            s32k3_emios_drive_out(s, n, 0);
            s->pwm_phase[n] = 0;
            s->uc_s[n] |= UC_S_FLAG;
            if (s->uc_c[n] & UC_C_FEN) {
                qemu_irq_raise(s->irq[n]);
            }
            s32k3_emios_update_irq(s, n);
            ptimer_set_limit(s->timer[n], b ? b : 1, 1);
        }
        ptimer_run(s->timer[n], 1);
        return;
    }

    if (mode == UC_MODE_MCB_UP || mode == UC_MODE_OPWMCB) {
        s->uc_s[n] |= UC_S_FLAG;
        if (s->uc_c[n] & UC_C_FEN) {
            qemu_irq_raise(s->irq[n]);
        }
        s32k3_emios_update_irq(s, n);
        /* toggle output: crude 50%-ish PWM at A duty */
        s32k3_emios_drive_out(s, n, !s->out_level[n]);
        ptimer_set_limit(s->timer[n], s32k3_emios_period(s, n), 1);
        ptimer_run(s->timer[n], 1);
    }
}

static void s32k3_emios_ch_config(S32K3EmiosState *s, int n)
{
    uint64_t hz = clock_get_hz(s->module_clk);
    bool enabled = (s->mcr & MCR_GPREN) && !(s->mcr & MCR_MDIS);

    ptimer_transaction_begin(s->timer[n]);
    ptimer_set_freq(s->timer[n], hz ? hz : 1);
    if (enabled) {
        ptimer_set_limit(s->timer[n], s32k3_emios_period(s, n), 1);
        ptimer_run(s->timer[n], 1);
    } else {
        ptimer_stop(s->timer[n]);
    }
    ptimer_transaction_commit(s->timer[n]);
}

static void s32k3_emios_update_irq(S32K3EmiosState *s, int n)
{
    /* 手册：每 eMIOS 6 条 NVIC 线，组 g 覆盖通道 23-4g..20-4g
     * （即 ch23/19/15/11/7/3 各组 4 通道）。重算该组 OR。 */
    int g = (23 - n) / 4;
    int i;
    bool level = false;

    if (g < 0 || g > 5) {
        g = 0;
    }
    for (i = 20 - 4 * g; i <= 23 - 4 * g; i++) {
        if ((s->uc_s[i] & UC_S_FLAG) && (s->uc_c[i] & UC_C_FEN)) {
            level = true;
            break;
        }
    }
    qemu_set_irq(s->irq_out[g], level);
    /* per-channel irq 线保留给兼容 */
    if (!(s->uc_c[n] & UC_C_FEN)) {
        qemu_irq_lower(s->irq[n]);
    }
}

static void s32k3_emios_reset(DeviceState *dev)
{
    S32K3EmiosState *s = S32K3_EMIOS(dev);
    int i;

    s->mcr = MCR_MDIS;
    s->gflag = 0;
    s->oudis = 0;
    s->ucdis = 0;
    for (i = 0; i < S32K3_EMIOS_CHANNELS; i++) {
        s->uc_a[i] = 0;
        s->uc_b[i] = 0;
        s->uc_c[i] = 0;
        s->uc_s[i] = 0;
        s->uc_alta[i] = 0;
        s32k3_emios_ch_config(s, i);
        qemu_irq_lower(s->irq[i]);
        s32k3_emios_drive_out(s, i, 0);
        s->pwm_phase[i] = 0;
    }
    for (i = 0; i < 6; i++) {
        qemu_irq_lower(s->irq_out[i]);
    }
}

static uint64_t s32k3_emios_read(void *opaque, hwaddr addr, unsigned size)
{
    S32K3EmiosState *s = opaque;
    int n;

    if (addr >= UC_BASE &&
        addr < UC_BASE + S32K3_EMIOS_CHANNELS * UC_STRIDE) {
        n = (addr - UC_BASE) / UC_STRIDE;
        switch ((addr - UC_BASE) % UC_STRIDE) {
        case 0x00:
            return s->uc_a[n];
        case 0x04:
            return s->uc_b[n];
        case 0x08:
            return ptimer_get_count(s->timer[n]);
        case 0x0C:
            return s->uc_c[n];
        case 0x10:
            return s->uc_s[n];
        case 0x14:
            return s->uc_alta[n];
        }
    }

    switch (addr) {
    case EMIOS_MCR:
        return s->mcr;
    case EMIOS_GFLAG:
        return s->gflag;
    case EMIOS_OUDIS:
        return s->oudis;
    case EMIOS_UCDIS:
        return s->ucdis;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_emios: read of unimplemented reg 0x%03" HWADDR_PRIx "\n",
                      addr);
        return 0;
    }
}

static void s32k3_emios_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned size)
{
    S32K3EmiosState *s = opaque;
    uint32_t v = value;
    int n, i;

    if (addr >= UC_BASE &&
        addr < UC_BASE + S32K3_EMIOS_CHANNELS * UC_STRIDE) {
        n = (addr - UC_BASE) / UC_STRIDE;
        switch ((addr - UC_BASE) % UC_STRIDE) {
        case 0x00:
            s->uc_a[n] = v & 0xffffff;
            s32k3_emios_ch_config(s, n);
            return;
        case 0x04:
            s->uc_b[n] = v & 0xffffff;
            return;
        case 0x0C:
            s->uc_c[n] = v;
            s32k3_emios_update_irq(s, n);
            s32k3_emios_ch_config(s, n);
            return;
        case 0x10:
            s->uc_s[n] &= ~v;   /* W1C */
            qemu_irq_lower(s->irq[n]);
            return;
        case 0x14:
            s->uc_alta[n] = v & 0xffffff;
            return;
        default:
            return;
        }
    }

    switch (addr) {
    case EMIOS_MCR:
        s->mcr = v;
        for (i = 0; i < S32K3_EMIOS_CHANNELS; i++) {
            s32k3_emios_ch_config(s, i);
        }
        break;
    case EMIOS_GFLAG:
        s->gflag &= ~v;
        break;
    case EMIOS_OUDIS:
        s->oudis = v;
        for (i = 0; i < S32K3_EMIOS_CHANNELS; i++) {
            s32k3_emios_drive_out(s, i, s->out_level[i]);
        }
        break;
    case EMIOS_UCDIS:
        s->ucdis = v;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_emios: write of unimplemented reg 0x%03" HWADDR_PRIx
                      " = 0x%08" PRIx64 "\n", addr, value);
    }
}

static const MemoryRegionOps s32k3_emios_ops = {
    .read = s32k3_emios_read,
    .write = s32k3_emios_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void s32k3_emios_init(Object *obj)
{
    S32K3EmiosState *s = S32K3_EMIOS(obj);
    int i;

    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);

    memory_region_init_io(&s->iomem, obj, &s32k3_emios_ops, s,
                          TYPE_S32K3_EMIOS, 0x4000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);

    for (i = 0; i < S32K3_EMIOS_CHANNELS; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq[i]);
        s->ctx[i].s = s;
        s->ctx[i].n = i;
        s->timer[i] = ptimer_init(s32k3_emios_ch_expire, &s->ctx[i],
                                  PTIMER_POLICY_LEGACY);
    }
    for (i = 0; i < 6; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq_out[i]);
    }
    qdev_init_gpio_out_named(DEVICE(s), s->out, "pwm",
                             S32K3_EMIOS_CHANNELS);
    qdev_init_gpio_in_named(DEVICE(s), s32k3_emios_in_set, "input",
                            S32K3_EMIOS_CHANNELS);
}

/* SAIC 输入捕获：输入边沿（按 EDSEL/EDPOL 配置）捕获当前计数到 UC_A，
 * 置 FLAG 并触发中断（FEN 使能时）。 */
static void s32k3_emios_in_set(void *opaque, int line, int level)
{
    S32K3EmiosState *s = opaque;
    uint32_t mode;
    uint8_t prev;

    if (line < 0 || line >= S32K3_EMIOS_CHANNELS) {
        return;
    }
    prev = s->in_level[line];
    s->in_level[line] = level & 1;
    mode = s->uc_c[line] & UC_C_MODE_MASK;

    if (mode != UC_MODE_SAIC) {
        return;   /* 仅输入捕获模式响应 */
    }
    /* EDSEL=0 边沿捕获；EDPOL 选择上升(0)/下降(1)沿 */
    if (prev == (level & 1)) {
        return;   /* 无跳变 */
    }
    if (s->uc_c[line] & UC_C_EDPOL) {
        if (!(prev && !(level & 1))) {
            return;   /* 需下降沿 */
        }
    } else {
        if (!(!prev && (level & 1))) {
            return;   /* 需上升沿 */
        }
    }
    /* 捕获当前计数到 UC_A */
    s->uc_a[line] = ptimer_get_count(s->timer[line]);
    s->uc_s[line] |= UC_S_FLAG;
    if (s->uc_c[line] & UC_C_FEN) {
        qemu_irq_raise(s->irq[line]);
    }
    s32k3_emios_update_irq(s, line);
}

static void s32k3_emios_realize(DeviceState *dev, Error **errp)
{
    S32K3EmiosState *s = S32K3_EMIOS(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_emios: module_clk must be connected");
        return;
    }
    s32k3_emios_reset(dev);
}

static void s32k3_emios_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_emios_reset);
    dc->realize = s32k3_emios_realize;
    dc->desc = "NXP S32K3xx eMIOS (24 channels, PWM/counter modes)";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo s32k3_emios_types[] = {
    {
        .name          = TYPE_S32K3_EMIOS,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3EmiosState),
        .instance_init = s32k3_emios_init,
        .class_init    = s32k3_emios_class_init,
    },
};

DEFINE_TYPES(s32k3_emios_types)
