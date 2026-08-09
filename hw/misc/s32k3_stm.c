/*
 * NXP S32K3xx STM (System Timer Module) QEMU device model
 *
 * 手册 RM 67：32 位自由运行计数器（CNT）+ 4 个比较通道，每通道独立
 * 中断源。寄存器（67.7.1）：
 *   0x00 CR   Control（TEN=bit0 使能，FRZ=bit1 调试停止）
 *   0x04 CNT  Count（32 位，向上计数，可写初值）
 *   通道 n（stride 0x10）：
 *   0x10 CCRn Channel Control（CEN=bit0 比较使能）
 *   0x14 CIRn Channel Interrupt（CIF=bit0 标志 W1C，CIE=bit1 中断使能）
 *   0x18 CMPn Channel Compare（匹配值）
 *
 * 实现：CNT 按虚拟时间换算（module_clk 频率）；每通道 ptimer 在
 * CMP 匹配时置 CIF 并触发 IRQ（CNT 32 位回绕后重载下一轮）。
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/ptimer.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/irq.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"

#define TYPE_S32K3_STM "s32k3-stm"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3StmState, S32K3_STM)

#define STM_CHANNELS 4

#define STM_CR      0x00
#define  STM_CR_TEN (1 << 0)
#define  STM_CR_FRZ (1 << 1)
#define STM_CNT     0x04
#define STM_CCR(n)  (0x10 + (n) * 0x10)
#define STM_CIR(n)  (0x14 + (n) * 0x10)
#define STM_CMP(n)  (0x18 + (n) * 0x10)
#define  CIR_CIF    (1 << 0)
#define  CIR_CIE    (1 << 1)

typedef struct StmChCtx {
    struct S32K3StmState *s;
    int n;
} StmChCtx;

struct S32K3StmState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock        *module_clk;
    qemu_irq     irq[STM_CHANNELS];

    uint32_t cr;
    uint32_t cnt_anchor;    /* 写 CNT 时的值 */
    int64_t  anchor_ns;     /* 写 CNT 时的虚拟时间 */
    uint32_t ccr[STM_CHANNELS];
    uint32_t cir[STM_CHANNELS];
    uint32_t cmp[STM_CHANNELS];

    ptimer_state *cmp_timer[STM_CHANNELS];
    StmChCtx     *ctx[STM_CHANNELS];
};

static uint64_t s32k3_stm_tick_ns(S32K3StmState *s)
{
    uint64_t hz = clock_get_hz(s->module_clk);
    return hz ? 1000000000ULL / hz : 1000;
}

/* 当前 CNT 值（虚拟时间换算） */
static uint32_t s32k3_stm_cnt(S32K3StmState *s)
{
    uint64_t tick_ns = s32k3_stm_tick_ns(s);
    int64_t elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->anchor_ns;

    if (elapsed < 0) {
        elapsed = 0;
    }
    return s->cnt_anchor + (uint32_t)(elapsed / tick_ns);
}

static void s32k3_stm_ch_update(S32K3StmState *s, int n)
{
    uint64_t tick_ns = s32k3_stm_tick_ns(s);
    uint32_t now = s32k3_stm_cnt(s);

    ptimer_transaction_begin(s->cmp_timer[n]);
    if ((s->cr & STM_CR_TEN) && (s->ccr[n] & 0x1)) {
        uint32_t remain = s->cmp[n] - now;
        ptimer_set_period(s->cmp_timer[n], tick_ns);
        ptimer_set_count(s->cmp_timer[n], remain);
        ptimer_run(s->cmp_timer[n], 1);
    } else {
        ptimer_stop(s->cmp_timer[n]);
    }
    ptimer_transaction_commit(s->cmp_timer[n]);
}

/* CMP 匹配：置 CIF + 触发 IRQ（STM 无 CIE 位——中断使能在 NVIC 层，手册 CIR 仅 CIF=bit0）。
 * 注：ptimer 回调内不可 begin/commit（QEMU 断言），一次匹配后由
 * 下一次 CCR/CMP/CNT 写重新武装。 */
static void s32k3_stm_ch_expire(void *opaque)
{
    StmChCtx *ctx = opaque;
    S32K3StmState *s = ctx->s;
    int n = ctx->n;

    s->cir[n] |= CIR_CIF;
    qemu_set_irq(s->irq[n], 1);
}

static void s32k3_stm_reset(DeviceState *dev)
{
    S32K3StmState *s = S32K3_STM(dev);
    int n;

    s->cr = 0;
    s->cnt_anchor = 0;
    s->anchor_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    for (n = 0; n < STM_CHANNELS; n++) {
        s->ccr[n] = 0;
        s->cir[n] = 0;
        s->cmp[n] = 0;
        ptimer_transaction_begin(s->cmp_timer[n]);
        ptimer_stop(s->cmp_timer[n]);
        ptimer_transaction_commit(s->cmp_timer[n]);
        qemu_irq_lower(s->irq[n]);
    }
}

static uint64_t s32k3_stm_read(void *opaque, hwaddr addr, unsigned size)
{
    S32K3StmState *s = opaque;

    switch (addr) {
    case STM_CR:
        return s->cr;
    case STM_CNT:
        return s32k3_stm_cnt(s);
    case STM_CCR(0): case STM_CCR(1): case STM_CCR(2): case STM_CCR(3):
        return s->ccr[(addr - STM_CCR(0)) / 0x10];
    case STM_CIR(0): case STM_CIR(1): case STM_CIR(2): case STM_CIR(3):
        return s->cir[(addr - STM_CIR(0)) / 0x10];
    case STM_CMP(0): case STM_CMP(1): case STM_CMP(2): case STM_CMP(3):
        return s->cmp[(addr - STM_CMP(0)) / 0x10];
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_stm: read of unimplemented reg 0x%02" HWADDR_PRIx "\n",
                      addr);
        return 0;
    }
}

static void s32k3_stm_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    S32K3StmState *s = opaque;
    uint32_t v = value;
    int n;

    switch (addr) {
    case STM_CR:
        s->cr = v & 0x3FF;
        for (n = 0; n < STM_CHANNELS; n++) {
            s32k3_stm_ch_update(s, n);
        }
        break;
    case STM_CNT:
        s->cnt_anchor = v;
        s->anchor_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        for (n = 0; n < STM_CHANNELS; n++) {
            s32k3_stm_ch_update(s, n);
        }
        break;
    case STM_CCR(0): case STM_CCR(1): case STM_CCR(2): case STM_CCR(3):
        n = (addr - STM_CCR(0)) / 0x10;
        s->ccr[n] = v;
        s32k3_stm_ch_update(s, n);
        break;
    case STM_CIR(0): case STM_CIR(1): case STM_CIR(2): case STM_CIR(3):
        n = (addr - STM_CIR(0)) / 0x10;
        s->cir[n] = v & 0x3;
        qemu_irq_lower(s->irq[n]);
        break;
    case STM_CMP(0): case STM_CMP(1): case STM_CMP(2): case STM_CMP(3):
        n = (addr - STM_CMP(0)) / 0x10;
        s->cmp[n] = v;
        s32k3_stm_ch_update(s, n);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_stm: write of unimplemented reg 0x%02" HWADDR_PRIx
                      " = 0x%08" PRIx32 "\n", addr, v);
    }
}

static const MemoryRegionOps s32k3_stm_ops = {
    .read = s32k3_stm_read,
    .write = s32k3_stm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void s32k3_stm_init(Object *obj)
{
    S32K3StmState *s = S32K3_STM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int n;

    memory_region_init_io(&s->iomem, obj, &s32k3_stm_ops, s,
                          TYPE_S32K3_STM, 0x60);
    sysbus_init_mmio(sbd, &s->iomem);
    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);
    for (n = 0; n < STM_CHANNELS; n++) {
        sysbus_init_irq(sbd, &s->irq[n]);
        s->ctx[n] = g_new0(StmChCtx, 1);
        s->ctx[n]->s = s;
        s->ctx[n]->n = n;
        s->cmp_timer[n] = ptimer_init(s32k3_stm_ch_expire, s->ctx[n],
                                      PTIMER_POLICY_LEGACY);
    }
}

static void s32k3_stm_realize(DeviceState *dev, Error **errp)
{
    S32K3StmState *s = S32K3_STM(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_stm: module_clk must be connected");
        return;
    }
    s32k3_stm_reset(dev);
}

static void s32k3_stm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_stm_reset);
    dc->realize = s32k3_stm_realize;
    dc->desc = "NXP S32K3xx STM (32-bit counter, 4 compare channels)";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo s32k3_stm_types[] = {
    {
        .name          = TYPE_S32K3_STM,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3StmState),
        .instance_init = s32k3_stm_init,
        .class_init    = s32k3_stm_class_init,
    },
};

DEFINE_TYPES(s32k3_stm_types)
