/*
 * NXP S32K3xx ADC (12-bit SAR) QEMU device model
 *
 * Normal + injected conversion chains, software/hardware trigger, three
 * channel groups (precision/standard/external), DMA request and analog
 * watchdog. Register layout per S32K3xx RM Rev.11 (ADC chapter 60).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/ptimer.h"
#include "hw/core/qdev-clock.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_S32K3_ADC "s32k3-adc"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3AdcState, S32K3_ADC)

#define S32K3_ADC_NCH  32   /* data registers per group */

/* registers (RM 60.6.2.1) */
#define ADC_MCR        0x00
#define  MCR_PWDN      (1 << 0)
#define  MCR_NSTART    (1 << 7)
#define  MCR_JSTART    (1 << 24)
#define  MCR_OWREN     (1 << 15)
#define  MCR_MODE_MASK (3 << 3)
#define  MCR_MODE      (1 << 1)
#define ADC_MSR        0x04
#define  MSR_NSTART    (1 << 1)
#define ADC_ISR        0x10
#define  ISR_EOC       (1 << 0)
#define  ISR_ECH       (1 << 1)
#define  ISR_EOC_MASK  0x3
#define ADC_CEOCFR0    0x14    /* precision channels EOC */
#define ADC_CEOCFR1    0x18    /* standard channels EOC */
#define ADC_CEOCFR2    0x1C    /* external channels EOC */
#define ADC_IMR        0x20
#define ADC_CIMR0      0x24
#define ADC_CIMR1      0x28
#define ADC_CIMR2      0x2C
#define ADC_WTISR      0x30    /* analog watchdog threshold int status */
#define ADC_WTIMR      0x34
#define ADC_DMAE       0x40
#define ADC_DMAR0      0x44
#define ADC_DMAR1      0x48
#define ADC_DMAR2      0x4C
#define ADC_THRHLR(n)  (0x60 + 4 * (n))   /* watchdog thresholds 0..3 */
#define ADC_NCMR0      0xA4    /* normal precision ch mask */
#define ADC_NCMR1      0xA8    /* normal standard ch mask */
#define ADC_NCMR2      0xAC    /* normal external ch mask */
#define ADC_JCMR0      0xB4    /* injected precision ch mask */
#define ADC_JCMR1      0xB8
#define ADC_JCMR2      0xBC
#define ADC_CTR0       0x94    /* conversion timing: precision */
#define ADC_CTR1       0x98    /* standard */
#define ADC_CTR2       0x9C    /* external */
#define ADC_DSDR       0xC4    /* delay start of data conversion */
#define ADC_PCDR(n)    (0x100 + 4 * (n))   /* precision data ch 0-7 */
#define ADC_ICDR(n)    (0x180 + 4 * (n))   /* standard data ch 0-23 */
#define ADC_ECDR(n)    (0x200 + 4 * (n))   /* external data ch 0-31 */
#define  PCDR_VALID    (1 << 19)
#define  PCDR_OVERW    (1 << 20)

#define ADC_GROUPS     3

struct S32K3AdcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock        *module_clk;
    qemu_irq     irq;
    qemu_irq     conv_done;   /* 转换完成输出（接 BCTU adc-done） */

    uint32_t mcr;
    uint32_t msr;
    uint32_t isr;
    uint32_t imr;
    uint32_t ceocfr[ADC_GROUPS];
    uint32_t cimr[ADC_GROUPS];
    uint32_t ncmr[ADC_GROUPS];
    uint32_t jcmr[ADC_GROUPS];
    uint32_t dmae;
    uint32_t dmar[ADC_GROUPS];
    uint32_t wtisr;
    uint32_t tca0;   /* TEMPSENSE 温度校准系数（出厂烧录，模型用 25C 典型值） */
    uint32_t tca1;
    uint32_t tca2;
    uint32_t wtimr;
    uint32_t thrhlr[4];

    /* conversion result per group/channel (12-bit) + valid flag */
    uint16_t cdr[ADC_GROUPS][S32K3_ADC_NCH];
    uint32_t valid[ADC_GROUPS];

    /* conversion timing: CTR0-2 @ 0x94/0x98/0x9C */
    uint32_t ctr[ADC_GROUPS];
    uint32_t dsdr;      /* delay start of data conversion @ 0xC4 */

    /* analog input levels, 0..65535 (scaled to 12-bit result) */
    uint32_t ain[S32K3_ADC_NCH];

    /* pending conversion state (non-instantaneous conversion) */
    ptimer_state *conv_timer;
    int conv_grp;       /* group being converted */
    bool conv_busy;
    bool conv_injected; /* current conversion is injected chain */

    /* RTD CheckSelfTestProgress：NSTART 空转换模拟 4 阶段自检序列
     * （MSR[2:0] 交替 1,0,1,0），否则固件 2 轮等开始/结束会超时。 */
    ptimer_state *selftest_timer;
    int selftest_phase;
};

static void s32k3_adc_update_irq(S32K3AdcState *s)
{
    bool level = (s->isr & s->imr & ISR_EOC_MASK) != 0;
    int g;

    /* CIMR：通道 EOC 中断使能——任一组的 CEOCFR 有通道完成且 CIMR 使能 */
    for (g = 0; g < ADC_GROUPS; g++) {
        if (s->ceocfr[g] & s->cimr[g]) {
            level = true;
            break;
        }
    }
    /* 看门狗中断（WTIMR 使能 + WTISR 置位） */
    if (s->wtisr & s->wtimr) {
        level = true;
    }
    qemu_set_irq(s->irq, level);
}

static uint32_t s32k3_adc_group_data_base(int grp)
{
    switch (grp) {
    case 0: return 0x100;   /* PCDR0-7 */
    case 1: return 0x180;   /* ICDR0-23 */
    default: return 0x200;  /* ECDR0-31 */
    }
}

static void s32k3_adc_convert_normal(S32K3AdcState *s);

static void s32k3_adc_do_convert(S32K3AdcState *s, int grp, uint32_t mask,
                                 bool injected)
{
    uint64_t hz = clock_get_hz(s->module_clk);
    uint32_t sample_cycles;

    if (s->mcr & MCR_PWDN) {
        return;
    }
    if (mask == 0) {
        /* 无通道配置的触发（RTD 自检用 NSTART 空转换）：模拟 4 阶段自检
         * 序列（MSR[2:0] 交替 1,0,1,0），CheckSelfTestProgress 有 2 轮
         * 等开始（MSR==1）/等结束（MSR==0）。 */
        s->selftest_phase = 0;
        s->msr = (s->msr & ~0xF) | 1;
        ptimer_transaction_begin(s->selftest_timer);
        ptimer_set_count(s->selftest_timer, 1);
        ptimer_run(s->selftest_timer, 1);
        ptimer_transaction_commit(s->selftest_timer);
        return;
    }

    /* 转换非即时：置 busy + Converting 状态（RTD 等就绪 MSR==1 会等到
     * 转换完成 conv_done 回 Idle），按 CTR[grp] 采样周期调度完成。
     * 转换总周期 ≈ 采样(CTR低8位) + 转换(CTR高8位) + DSDR 延迟。 */
    s->conv_grp = grp;
    s->conv_busy = true;
    s->msr = (s->msr & ~0xF) | 2;   /* Converting */
    s->conv_injected = injected;
    s->msr |= MSR_NSTART;
    sample_cycles = (s->ctr[grp] & 0xFF) + ((s->ctr[grp] >> 8) & 0xFF) +
                    (s->dsdr & 0xFF);
    if (sample_cycles == 0) {
        sample_cycles = 1;
    }
    ptimer_transaction_begin(s->conv_timer);
    ptimer_set_freq(s->conv_timer, hz ? hz : 1);
    ptimer_set_count(s->conv_timer, sample_cycles);
    ptimer_run(s->conv_timer, 1);
    ptimer_transaction_commit(s->conv_timer);
}

static void s32k3_adc_selftest_step(void *opaque)
{
    S32K3AdcState *s = opaque;

    s->selftest_phase++;
    /* 阶段序列 1,0,1,0（phase1=0, phase2=1, phase3=0）。
     * 回调已在 ptimer 事务内，直接重调度。 */
    if (s->selftest_phase <= 3) {
        s->msr = (s->msr & ~0xF) | (s->selftest_phase & 1);
        ptimer_set_count(s->selftest_timer, 1);
        ptimer_run(s->selftest_timer, 1);
    }
}

static void s32k3_adc_conv_done(void *opaque)
{
    S32K3AdcState *s = opaque;
    int grp = s->conv_grp;
    uint32_t mask;
    int ch;

    if (!s->conv_busy) {
        return;
    }
    /* 完成：按该组 mask 写入结果（正常组用 ncmr，注入组用 jcmr） */
    mask = s->ncmr[grp];
    if (mask == 0) {
        mask = s->jcmr[grp];
    }
    for (ch = 0; ch < S32K3_ADC_NCH; ch++) {
        if (mask & (1u << ch)) {
            uint16_t val = (s->ain[ch] >> 4) & 0xFFF;
            int wi;

            s->cdr[grp][ch] = val;
            s->valid[grp] |= 1u << ch;
            s->ceocfr[grp] |= 1u << ch;

            /* analog watchdog: compare against THRHLR thresholds */
            for (wi = 0; wi < 4; wi++) {
                uint32_t thr = s->thrhlr[wi];
                uint16_t lo = thr & 0xFFFF;
                uint16_t hi = thr >> 16;
                if ((s->wtimr & (1u << (grp * 8 + ch))) &&
                    (val < lo || val > hi)) {
                    s->wtisr |= 1u << (grp * 8 + ch);
                }
            }
        }
    }
    s->conv_busy = false;
    /* 真实转换完成：ADSTATUS 回 Idle(1)。RTD 等就绪（MSR==1）。
     * 注：自检空转换（无通道）在 do_convert 里直接回 Reset(0)。 */
    s->msr = 1;
    s->isr |= ISR_EOC | ISR_ECH;
    s32k3_adc_update_irq(s);
    /* 通知 BCTU 转换完成 */
    qemu_set_irq(s->conv_done, 1);
    qemu_set_irq(s->conv_done, 0);

    /* 连续 Scan 模式（MCR[MODE]=10b）：正常转换链自动循环扫描 */
    if (((s->mcr & MCR_MODE_MASK) >> 3) == 2 && !s->conv_injected) {
        s32k3_adc_convert_normal(s);
    }
}

static void s32k3_adc_convert_normal(S32K3AdcState *s)
{
    int g;
    for (g = 0; g < ADC_GROUPS; g++) {
        s32k3_adc_do_convert(s, g, s->ncmr[g], false);
    }
}

static void s32k3_adc_convert_injected(S32K3AdcState *s)
{
    int g;
    for (g = 0; g < ADC_GROUPS; g++) {
        /* RTD TempSenseGetTemp 用 JSTART 触发但通道配置在 NCMR：
         * JCMR 为空时回退到 NCMR（否则 mask=0 空转、CEOCFR 不置位）。 */
        uint32_t mask = s->jcmr[g] ? s->jcmr[g] : s->ncmr[g];
        s32k3_adc_do_convert(s, g, mask, true);
    }
}

static void s32k3_adc_ain_set(void *opaque, int line, int level)
{
    S32K3AdcState *s = opaque;

    if (line < 0 || line >= S32K3_ADC_NCH) {
        return;
    }
    /* gpio level is 0/1; 1 -> full scale (scaled input via monitor) */
    s->ain[line] = level ? 65535 : 0;
}

/* hardware trigger input (from BCTU): rising edge starts a conversion */
static void s32k3_adc_trig_set(void *opaque, int line, int level)
{
    S32K3AdcState *s = opaque;

    if (level) {
        s32k3_adc_convert_normal(s);
    }
}

static void s32k3_adc_reset(DeviceState *dev)
{
    S32K3AdcState *s = S32K3_ADC(dev);

    s->mcr = MCR_PWDN;
    /* MSR 复位 = Idle（ADSTATUS=1）。RTD Adc_Sar_Ip_Init 复位后直接
     * 等 MSR[3:0]==1，不清 PWDN。 */
    s->msr = 1;
    s->isr = 0;
    s->imr = 0;
    s->dmae = 0;
    s->wtisr = 0;
    s->wtimr = 0;
    memset(s->ceocfr, 0, sizeof(s->ceocfr));
    /* TEMPSENSE 校准系数：25C 典型值（TCA0=25<<4 定点；一/二阶项 0） */
    s->tca0 = 25u << 4;
    s->tca1 = 0;
    s->tca2 = 0;
    memset(s->cimr, 0, sizeof(s->cimr));
    memset(s->ncmr, 0, sizeof(s->ncmr));
    memset(s->jcmr, 0, sizeof(s->jcmr));
    memset(s->dmar, 0, sizeof(s->dmar));
    memset(s->cdr, 0, sizeof(s->cdr));
    memset(s->valid, 0, sizeof(s->valid));
    memset(s->thrhlr, 0, sizeof(s->thrhlr));
    /* CTR0-2 复位 0x16（采样 2 + 转换 4 周期） */
    s->ctr[0] = s->ctr[1] = s->ctr[2] = 0x16;
    s->dsdr = 0;
    s->conv_busy = false;
    s->conv_injected = false;
    s32k3_adc_update_irq(s);
}

static uint64_t s32k3_adc_read(void *opaque, hwaddr addr, unsigned size)
{
    S32K3AdcState *s = opaque;
    int grp, ch;

    for (grp = 0; grp < ADC_GROUPS; grp++) {
        uint32_t base = s32k3_adc_group_data_base(grp);
        uint32_t count = (grp == 0) ? 8 : 24;
        if (addr >= base && addr < base + 4 * count) {
            ch = (addr - base) / 4;
            return s->cdr[grp][ch] |
                   ((s->valid[grp] >> ch) & 1 ? PCDR_VALID : 0);
        }
    }

    switch (addr) {
    case ADC_MCR:      return s->mcr;
    case ADC_MSR:      return s->msr;
    case ADC_ISR:      return s->isr;
    case ADC_IMR:      return s->imr;
    case ADC_CEOCFR0:  return s->ceocfr[0];
    case ADC_CEOCFR1:  return s->ceocfr[1];
    case ADC_CEOCFR2:  return s->ceocfr[2];
    case ADC_CIMR0:    return s->cimr[0];
    case ADC_CIMR1:    return s->cimr[1];
    case ADC_CIMR2:    return s->cimr[2];
    case ADC_NCMR0:    return s->ncmr[0];
    case ADC_NCMR1:    return s->ncmr[1];
    case ADC_NCMR2:    return s->ncmr[2];
    case ADC_JCMR0:    return s->jcmr[0];
    case ADC_JCMR1:    return s->jcmr[1];
    case ADC_JCMR2:    return s->jcmr[2];
    case ADC_CTR0:     return s->ctr[0];
    case ADC_CTR1:     return s->ctr[1];
    case ADC_CTR2:     return s->ctr[2];
    case ADC_DSDR:     return s->dsdr;
    case ADC_DMAE:     return s->dmae;
    case ADC_DMAR0:    return s->dmar[0];
    case ADC_DMAR1:    return s->dmar[1];
    case ADC_DMAR2:    return s->dmar[2];
    case ADC_WTISR:    return s->wtisr;
    case ADC_WTIMR:    return s->wtimr;
    case ADC_THRHLR(0): case ADC_THRHLR(1):
    case ADC_THRHLR(2): case ADC_THRHLR(3):
        return s->thrhlr[(addr - ADC_THRHLR(0)) / 4];
    case 0x3A0: return s->tca0;   /* TEMPSENSE TCA0 */
    case 0x3A4: return s->tca1;   /* TEMPSENSE TCA1 */
    case 0x3A8: return s->tca2;   /* TEMPSENSE TCA2 */
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_adc: read of unimplemented reg 0x%03" HWADDR_PRIx "\n",
                      addr);
        return 0;
    }
}

static void s32k3_adc_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    S32K3AdcState *s = opaque;
    uint32_t v = value;

    switch (addr) {
    case ADC_MCR: {
        bool nstart = (v & MCR_NSTART) && !(s->mcr & MCR_NSTART);
        bool jstart = (v & MCR_JSTART) && !(s->mcr & MCR_JSTART);
        bool power_on = !(v & MCR_PWDN) && (s->mcr & MCR_PWDN);
        s->mcr = v & ~(MCR_NSTART | MCR_JSTART); /* self-clearing */
        if (power_on) {
            /* 上电完成：进入 Idle（RTD Adc_Sar_Ip_Init 等 MSR[3:0]==1） */
            s->msr = (s->msr & ~0xF) | 1;
        } else if ((v & MCR_PWDN) && (v & MCR_MODE)) {
            /* 掉电 + MODE 写（RTD 自检第 2 轮 CheckSelfTestProgress）：
             * 模拟算法执行完成，ADSTATUS 回 Reset(0)。 */
            s->msr = (s->msr & ~0xF) | 0;
        }
        if (nstart) {
            s32k3_adc_convert_normal(s);
        }
        if (jstart) {
            s32k3_adc_convert_injected(s);
        }
        break;
    }
    case ADC_MSR:
        /* 写 MSR 触发自检（算法 S）：模拟立即完成 -> ADSTATUS 回 Reset。
         * RTD CheckSelfTestProgress 等 MSR[2:0]==0。 */
        s->msr = (s->msr & ~0xF) | 0;
        break;
    case ADC_ISR:
        s->isr &= ~v;   /* W1C */
        s32k3_adc_update_irq(s);
        break;
    case ADC_IMR:
        s->imr = v & ISR_EOC_MASK;
        s32k3_adc_update_irq(s);
        break;
    case ADC_CEOCFR0: s->ceocfr[0] &= ~v; break;
    case ADC_CEOCFR1: s->ceocfr[1] &= ~v; break;
    case ADC_CEOCFR2: s->ceocfr[2] &= ~v; break;
    case ADC_CIMR0: s->cimr[0] = v; break;
    case ADC_CIMR1: s->cimr[1] = v; break;
    case ADC_CIMR2: s->cimr[2] = v; break;
    case ADC_NCMR0: s->ncmr[0] = v; break;
    case ADC_NCMR1: s->ncmr[1] = v; break;
    case ADC_NCMR2: s->ncmr[2] = v; break;
    case ADC_JCMR0: s->jcmr[0] = v; break;
    case ADC_JCMR1: s->jcmr[1] = v; break;
    case ADC_JCMR2: s->jcmr[2] = v; break;
    case ADC_CTR0:  s->ctr[0] = v; break;
    case ADC_CTR1:  s->ctr[1] = v; break;
    case ADC_CTR2:  s->ctr[2] = v; break;
    case ADC_DSDR:  s->dsdr = v; break;
    case ADC_DMAE:  s->dmae = v; break;
    case ADC_DMAR0: s->dmar[0] = v; break;
    case ADC_DMAR1: s->dmar[1] = v; break;
    case ADC_DMAR2: s->dmar[2] = v; break;
    case ADC_WTISR: s->wtisr &= ~v; break;
    case ADC_WTIMR: s->wtimr = v; break;
    case ADC_THRHLR(0): case ADC_THRHLR(1):
    case ADC_THRHLR(2): case ADC_THRHLR(3):
        s->thrhlr[(addr - ADC_THRHLR(0)) / 4] = v;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_adc: write of unimplemented reg 0x%03" HWADDR_PRIx
                      " = 0x%08" PRIx64 "\n", addr, value);
    }
}

static const MemoryRegionOps s32k3_adc_ops = {
    .read = s32k3_adc_read,
    .write = s32k3_adc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void s32k3_adc_init(Object *obj)
{
    S32K3AdcState *s = S32K3_ADC(obj);

    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);

    memory_region_init_io(&s->iomem, obj, &s32k3_adc_ops, s,
                          TYPE_S32K3_ADC, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    qdev_init_gpio_in_named(DEVICE(s), s32k3_adc_ain_set, "adc-in",
                            S32K3_ADC_NCH);
    qdev_init_gpio_in_named(DEVICE(s), s32k3_adc_trig_set, "hw-trig", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->conv_done, "conv-done", 1);
    s->conv_timer = ptimer_init(s32k3_adc_conv_done, s,
                                PTIMER_POLICY_LEGACY);
    s->selftest_timer = ptimer_init(s32k3_adc_selftest_step, s,
                                    PTIMER_POLICY_LEGACY);
    ptimer_transaction_begin(s->selftest_timer);
    ptimer_set_freq(s->selftest_timer, 1000000);  /* 1us 阶段 */
    ptimer_transaction_commit(s->selftest_timer);

    /* QOM 属性：ain-ch0..7 允许运行时设置模拟输入电压（0-65535，
     * 映射 16 位到 12 位 ADC 结果）。可用 HMP: qom-set /machine/
     * 或 -global 初始化。 */
    object_property_add_uint32_ptr(obj, "ain-ch0", &s->ain[0],
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "ain-ch1", &s->ain[1],
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "ain-ch2", &s->ain[2],
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "ain-ch3", &s->ain[3],
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "ain-ch4", &s->ain[4],
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "ain-ch5", &s->ain[5],
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "ain-ch6", &s->ain[6],
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "ain-ch7", &s->ain[7],
                                   OBJ_PROP_FLAG_READWRITE);
}

static void s32k3_adc_realize(DeviceState *dev, Error **errp)
{
    S32K3AdcState *s = S32K3_ADC(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_adc: module_clk must be connected");
        return;
    }
    s32k3_adc_reset(dev);
}

static void s32k3_adc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_adc_reset);
    dc->realize = s32k3_adc_realize;
    dc->desc = "NXP S32K3xx ADC (12-bit SAR, normal+injected, 3 groups)";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo s32k3_adc_types[] = {
    {
        .name          = TYPE_S32K3_ADC,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3AdcState),
        .instance_init = s32k3_adc_init,
        .class_init    = s32k3_adc_class_init,
    },
};

DEFINE_TYPES(s32k3_adc_types)
