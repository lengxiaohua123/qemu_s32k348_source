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
#include "qapi/visitor.h"

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
/* S32K3 RM UC_C 位（表：bit8=EDSEL, bit7=EDPOL, bit17=FEN） */
#define  UC_C_EDSEL      (1 << 8)
#define  UC_C_EDPOL      (1 << 7)
#define  UC_C_FEN        (1 << 17)
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
#define UC_MODE_SAIC     0x02   /* input capture（S32K3 RM 表 408：SAIC=000_0010） */
#define UC_MODE_SAOC     0x03   /* output compare */

struct EmiosChCtx {
    S32K3EmiosState *s;
    int n;
};

/* S32K3 counter bus0 全局主：eMIOS0 ch0 MCB（跨实例共享，固件 eMIOS1 ICU
 * 的 bus0 来自 eMIOS0）。运行时由 ch0 写 MCB_UP 模式时登记。 */
static S32K3EmiosState *s32k3_emios_bus_master;

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

    /* PWM 占空比统计（诊断：qom-set pwm-dump 打印） */
    uint64_t stat_high[S32K3_EMIOS_CHANNELS];
    uint64_t stat_total[S32K3_EMIOS_CHANNELS];
    uint64_t stat_last[S32K3_EMIOS_CHANNELS];
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
    uint64_t now;

    /* 占空比统计：累计各电平虚拟时间 */
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (s->stat_last[n]) {
        uint64_t dt = now - s->stat_last[n];
        s->stat_total[n] += dt;
        if (s->out_level[n]) {
            s->stat_high[n] += dt;
        }
    }
    s->stat_last[n] = now;

    s->out_level[n] = disabled ? 0 : (level & 1);
    qemu_set_irq(s->out[n], s->out_level[n]);
}

/* OPWMB/OPWMT 的外部 counter bus 周期：
 * S32K3 counter bus 0 = 本实例 CH0（MCB/MCB_UP 的 modulus）。
 * 若 ch0 非 MCB（fallback），用通道自身 B 作周期。 */
static uint32_t s32k3_emios_counter_bus_count(S32K3EmiosState *s, int n)
{
    S32K3EmiosState *m = s;

    if (!((m->uc_c[0] & UC_C_MODE_MASK) == UC_MODE_MCB_UP)) {
        m = s32k3_emios_bus_master;   /* 跨实例 bus0：eMIOS0 ch0 */
    }
    if (m && (m->uc_c[0] & UC_C_MODE_MASK) == UC_MODE_MCB_UP) {
        return ptimer_get_count(m->timer[0]);
    }
    return 0;
}

static uint32_t s32k3_emios_counter_bus_period(S32K3EmiosState *s, int n)
{
    if ((s->uc_c[0] & UC_C_MODE_MASK) == UC_MODE_MCB_UP) {
        return s->uc_a[0] ? s->uc_a[0] : 1;
    }
    return s->uc_b[n] ? s->uc_b[n] : 1;
}

static void s32k3_emios_ch_expire(void *opaque)
{
    struct EmiosChCtx *ctx = opaque;
    S32K3EmiosState *s = ctx->s;
    int n = ctx->n;
    uint32_t mode = s->uc_c[n] & UC_C_MODE_MASK;

    if (mode == UC_MODE_OPWMB) {
        /* OPWMB（手册 63.5.3.18 + RTD SetDutyCycleOpwmb）：
         * 周期 = 外部 counter bus（实例内 MCB 通道 modulus，S32K3 bus0=eMIOS CH0）；
         * A(AS1)=leading edge（高段起点）、B(BS1)=trailing edge（低点）；
         * RTD: B = duty + A，占空比 = (B-A)/period。
         * 三段循环：0→A 低、A→B 高、B→P 低；FLAG 在 BS1 match（MODE=110_0000）。 */
        uint32_t a = s->uc_a[n] ? s->uc_a[n] : 1;
        uint32_t b = s->uc_b[n];
        uint32_t period = s32k3_emios_counter_bus_period(s, n);

        if (b > period) {
            b = period;
        }
        if (a > b) {
            a = b;
        }
        switch (s->pwm_phase[n]) {
        case 0:      /* 0 → A：低段 */
            s32k3_emios_drive_out(s, n, 0);
            s->pwm_phase[n] = 1;
            ptimer_set_limit(s->timer[n], a ? a : 1, 1);
            break;
        case 1:      /* A → B：高段 */
            s32k3_emios_drive_out(s, n, 1);
            s->pwm_phase[n] = 2;
            ptimer_set_limit(s->timer[n], b - a ? b - a : 1, 1);
            break;
        default:     /* B → P：低段，BS1 match 置 FLAG */
            s32k3_emios_drive_out(s, n, 0);
            s->pwm_phase[n] = 0;
            s->uc_s[n] |= UC_S_FLAG;
            if (s->uc_c[n] & UC_C_FEN) {
                qemu_irq_raise(s->irq[n]);
            }
            s32k3_emios_update_irq(s, n);
            ptimer_set_limit(s->timer[n], period - b ? period - b : 1, 1);
            break;
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
            if (n == 0 && (v & UC_C_MODE_MASK) == UC_MODE_MCB_UP &&
                !s32k3_emios_bus_master) {
                s32k3_emios_bus_master = s;
            }
            s32k3_emios_update_irq(s, n);
            s32k3_emios_ch_config(s, n);
            return;
        case 0x10:
            s->uc_s[n] &= ~v;   /* W1C */
            qemu_irq_lower(s->irq[n]);
            s32k3_emios_update_irq(s, n);   /* FLAG 已清，重算 irq_out[g] */
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
    /* 无跳变直接忽略 */
    if (prev == (level & 1)) {
        return;
    }
    /* EDSEL=1：双沿捕获；EDSEL=0：EDPOL 选沿（0=上升,1=下降） */
    if (!(s->uc_c[line] & UC_C_EDSEL)) {
        if (s->uc_c[line] & UC_C_EDPOL) {
            if (!(prev && !(level & 1))) {
                return;   /* 需下降沿 */
            }
        } else {
            if (!(!prev && (level & 1))) {
                return;   /* 需上升沿 */
            }
        }
    }
    /* SAIC：捕获 counter bus 时间基准到 AS1（UC_A）——手册 EDSEL=1 时
     * 上升沿捕获到 AS1（固件 TIMESTAMP 处理读 UC_A）；时间基准 = bus0
     * （本实例 CH0 MCB 计数）。下降沿才捕获到 AS2（UC_ALTA）。 */
    s->uc_a[line] = s32k3_emios_counter_bus_count(s, line);
    s->uc_s[line] |= UC_S_FLAG;
    if (s->uc_c[line] & UC_C_FEN) {
        qemu_irq_raise(s->irq[line]);
    }
    s32k3_emios_update_irq(s, line);
}

/* PWM 占空比统计打印（qom-set pwm-dump） */
static void s32k3_emios_pwm_dump_set(Object *obj, Visitor *v,
                                     const char *name, void *opaque,
                                     Error **errp)
{
    S32K3EmiosState *s = S32K3_EMIOS(obj);
    uint32_t dummy = 0;
    int n;

    visit_type_uint32(v, name, &dummy, errp);
    for (n = 0; n < S32K3_EMIOS_CHANNELS; n++) {
        uint64_t tot = s->stat_total[n];
        if (!tot) {
            continue;
        }
        fprintf(stderr, "[PWM] ch%-2d duty=%llu.%02llu%% (hi=%llu tot=%llu us) "
                "A=%u B=%u C=0x%x\n",
                n,
                (unsigned long long)(s->stat_high[n] * 100 / tot),
                (unsigned long long)((s->stat_high[n] * 10000 / tot) % 100),
                (unsigned long long)s->stat_high[n] / 1000,
                (unsigned long long)tot / 1000,
                s->uc_a[n], s->uc_b[n], s->uc_c[n]);
    }
}

static void s32k3_emios_realize(DeviceState *dev, Error **errp)
{
    S32K3EmiosState *s = S32K3_EMIOS(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_emios: module_clk must be connected");
        return;
    }
    object_property_add(OBJECT(dev), "pwm-dump", "uint32",
                        NULL, s32k3_emios_pwm_dump_set, NULL, NULL);
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
