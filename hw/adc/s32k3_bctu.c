/*
 * NXP S32K3xx BCTU (Body Cross Triggering Unit) QEMU device model
 *
 * Trigger routing matrix between timer/LCU/PIT sources and the three
 * ADC instances.  The model implements:
 *   - 8 hardware trigger sources, each mappable to an ADC + channel mask
 *   - trigger input arrives via "trig-in" qdev gpio lines (wired by the
 *     board to eMIOS flags / LCU outputs / PIT flags)
 *   - on trigger, the selected ADC's conversion mask is applied and a
 *     hardware conversion is started via the ADC "hw-trig" gpio line
 *   - result FIFO (data registers) with watermark interrupt
 *
 * Register layout per S32K3xx RM (BCTU chapter).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "system/address-spaces.h"

#define TYPE_S32K3_BCTU "s32k3-bctu"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3BctuState, S32K3_BCTU)

/* ADC0 base for reading back conversion results (AIPS0) */
#define S32K348_ADC0_BASE 0x400A0000u

#define S32K3_BCTU_TRIGGERS 8
#define S32K3_BCTU_FIFO     16

/* registers */
#define BCTU_MCR        0x00
#define  MCR_MDIS       (1 << 30)  /* S32K348.h BCTU_MCR_MDIS_SHIFT=30 */
#define BCTU_MSR        0x08       /* S32K348.h MSR（TRGF=bit15 等） */
#define  MSR_TRGF       (1 << 15)
#define BCTU_TRGCFG(n)  (0x18 + 4 * (n))   /* S32K348.h TRGCFG[72]@0x18 */
/* S32K348.h BCTU TRGCFG 权威位：TRIGEN=bit15、TRG_FLAG=bit14、
 * TRS=bit13、ADC_SEL0/1/2=bit8/9/10、CHANNEL_VALUE=bit0-6 */
#define  TRGCFG_TRGEN   (1 << 15)      /* TRIGEN_MASK=0x8000 */
#define  TRGCFG_TRG_FLAG (1 << 14)     /* TRG_FLAG_MASK=0x4000 */
#define  TRGCFG_ADCSEL  (7 << 8)       /* ADC_SEL0/1/2 = bit8/9/10 */
#define  TRGCFG_ADCSEL_SHIFT 8
#define  TRGCFG_CHMASK  0x7F           /* channel value bits 0..6 */
/* S32K348.h BCTU FIFO 区：FIFO1DR@0x450/FIFOCR@0x460/FIFOWM@0x464/
 * FIFOERR@0x468/FIFOSR@0x46C（原 0x80/0x84/0x88 虚构——未对齐） */
#define BCTU_FIFO1DR    0x450
#define  FIFODR_VALID   (1 << 19)
#define  FIFODR_CHN_SHIFT 20
#define BCTU_FIFOCR     0x460
#define  FIFOCR_IEN_FIFO1 (1 << 16)
#define BCTU_FIFOWM     0x464
#define  FIFOWM_WM_FIFO1 0xF
#define BCTU_FIFOERR    0x468
#define  FIFOERR_WM_INT_FIFO1 (1 << 16)
#define BCTU_FIFOSR     0x46C
#define  FIFOSR_FULL_FIFO1 (1 << 0)
#define BCTU_FIFOCNTR   0x470    /* FIFO Counter（RM 64.4.1） */
#define BCTU_FIFO2DR    0x454    /* FIFO2 Result Data（RM 64.4.1，固件若用） */
#define BCTU_SFTRGR1    0x22C    /* Software Trigger 1（固件若用软件触发） */

struct S32K3BctuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock        *module_clk;
    qemu_irq     irq;

    uint32_t mcr;
    uint32_t msr;
    uint32_t fifocr;
    uint32_t fifoerr;
    uint32_t trgcfg[S32K3_BCTU_TRIGGERS];

    /* result fifo */
    uint32_t fifo[S32K3_BCTU_FIFO];
    uint32_t fifo_len;
    uint32_t fifo_wm;
    uint32_t regs[0x100];   /* 影子 */

    /* output trigger lines to ADC instances (board-wired) */
    qemu_irq adc_trig[3];
    /* input "conversion done" lines from each ADC (board-wired) */
    qemu_irq adc_done[3];

    /* pending ADC index waiting for its conversion result */
    int pending_adc;

    int pending_ch;
    bool pending_valid;
};

static void s32k3_bctu_update_irq(S32K3BctuState *s)
{
    /* S32K348 BCTU 中断：FIFO 超水位置 FIFOERR.WM_INT_FIFO1，
     * 与 FIFOCR.IEN_FIFO1 一起触发 IRQ87（固件 ISR 清 FIFOERR W1C）。 */
    qemu_set_irq(s->irq,
                 (s->fifoerr & FIFOERR_WM_INT_FIFO1) &&
                 (s->fifocr & FIFOCR_IEN_FIFO1));
}

static void s32k3_bctu_fire(S32K3BctuState *s, int n)
{
    uint32_t cfg = s->trgcfg[n];
    int adc = (cfg & TRGCFG_ADCSEL) >> TRGCFG_ADCSEL_SHIFT;
    uint32_t mask = cfg & TRGCFG_CHMASK;

    if (!(cfg & TRGCFG_TRGEN)) {
        return;
    }
    if (s->irq) {
    }
    if (s->mcr & MCR_MDIS) {
        return;
    }

    /* forward trigger pulse to the selected ADC */
    if (adc < 3) {
        qemu_irq_pulse(s->adc_trig[adc]);
        /* remember which channel we expect back from this ADC */
        s->pending_adc = adc;
        s->pending_ch = mask ? __builtin_ctz(mask) : 0;
        s->pending_valid = true;
    }
}

/* ADC conversion done: read the result back and push into the FIFO.
 * The ADC drives "conv-done" out; the board wires it to this input. */
static void s32k3_bctu_adc_done(void *opaque, int line, int level)
{
    S32K3BctuState *s = opaque;
    uint32_t pcdr = 0;

    if (!level) {
        return;
    }
    if (line < 0 || line >= 3 || !s->pending_valid || s->pending_adc != line) {
        return;
    }
    /* read PCDR from the ADC via system bus */
    if (address_space_read(&address_space_memory,
                           S32K348_ADC0_BASE + line * 0x4000 +
                           0x100 + 4 * s->pending_ch,
                           MEMTXATTRS_UNSPECIFIED, &pcdr, 4) == MEMTX_OK) {
    if (s->fifo_len < S32K3_BCTU_FIFO) {
            s->fifo[s->fifo_len] =
                FIFODR_VALID | (s->pending_ch << FIFODR_CHN_SHIFT) |
                (pcdr & 0xFFF);
            s->fifo_len++;
        }
        s->pending_valid = false;
    }
    if (s->fifo_len > s->fifo_wm) {
        s->fifoerr |= FIFOERR_WM_INT_FIFO1;
        s32k3_bctu_update_irq(s);
    }
}

static void s32k3_bctu_trig_set(void *opaque, int line, int level)
{
    S32K3BctuState *s = opaque;

    if (line < 0 || line >= S32K3_BCTU_TRIGGERS) {
        return;
    }
    if (level) {
        s32k3_bctu_fire(s, line);
    }
}

static void s32k3_bctu_reset(DeviceState *dev)
{
    S32K3BctuState *s = S32K3_BCTU(dev);

    s->mcr = 0;
    s->msr = 0;
    s->fifocr = 0;
    s->fifoerr = 0;
    memset(s->trgcfg, 0, sizeof(s->trgcfg));
    s->fifo_len = 0;
    s->fifo_wm = 4;
    s->pending_valid = false;
    s32k3_bctu_update_irq(s);
}

static uint64_t s32k3_bctu_read(void *opaque, hwaddr addr, unsigned size)
{
    if (size == 8) {
        uint64_t lo = s32k3_bctu_read(opaque, addr, 4);
        uint64_t hi = s32k3_bctu_read(opaque, addr + 4, 4);
        return lo | (hi << 32);
    }
    if (size == 2) {
        uint32_t full = s32k3_bctu_read(opaque, addr & ~3u, 4);
        return (addr & 2) ? (full >> 16) : (full & 0xFFFF);
    }
    if (size == 1) {
        uint32_t full = s32k3_bctu_read(opaque, addr & ~3u, 4);
        return (full >> (8 * (addr & 3))) & 0xFF;
    }
    S32K3BctuState *s = opaque;

    if (addr >= BCTU_TRGCFG(0) && addr < BCTU_TRGCFG(S32K3_BCTU_TRIGGERS)) {
        return s->trgcfg[(addr - BCTU_TRGCFG(0)) / 4];
    }

    switch (addr) {
    case BCTU_MCR:
        return s->mcr;
    case BCTU_MSR:
        return s->msr;
    case BCTU_FIFO1DR:
        if (s->fifo_len > 0) {
            uint32_t r = s->fifo[0];
            memmove(s->fifo, s->fifo + 1,
                    (--s->fifo_len) * sizeof(uint32_t));
            return r;
        }
        return 0;
    case BCTU_FIFOCR:
        return s->fifocr;
    case BCTU_FIFOWM:
        return s->fifo_wm;
    case BCTU_FIFOERR:
        return s->fifoerr;
    case BCTU_FIFOSR:
        return s->fifo_len ? FIFOSR_FULL_FIFO1 : 0;
    case BCTU_FIFOCNTR:
        return s->fifo_len;
    default:
        /* 影子数组 0x100 项而 MMIO 窗口 0x4000 字节：限界防越界读。 */
        return addr < sizeof(s->regs) ? s->regs[addr / 4] : 0;
    }
}

static void s32k3_bctu_write(void *opaque, hwaddr addr,
                             uint64_t value, unsigned size)
{
    if (size == 8) {
        s32k3_bctu_write(opaque, addr, value & 0xFFFFFFFF, 4);
        s32k3_bctu_write(opaque, addr + 4, value >> 32, 4);
        return;
    }
    if (size == 2) {
        uint32_t full = s32k3_bctu_read(opaque, addr & ~3u, 4);
        uint32_t w = value & 0xFFFF;
        uint32_t merged = (addr & 2) ? ((full & 0xFFFF) | (w << 16))
                                     : ((full & 0xFFFF0000u) | w);
        s32k3_bctu_write(opaque, addr & ~3u, merged, 4);
        return;
    }
    if (size == 1) {
        uint32_t full = s32k3_bctu_read(opaque, addr & ~3u, 4);
        uint32_t sh = 8 * (addr & 3);
        uint32_t merged = (full & ~(0xFFu << sh)) | ((value & 0xFF) << sh);
        s32k3_bctu_write(opaque, addr & ~3u, merged, 4);
        return;
    }
    S32K3BctuState *s = opaque;
    uint32_t v = value;

    if (addr >= BCTU_TRGCFG(0) && addr < BCTU_TRGCFG(S32K3_BCTU_TRIGGERS)) {
        s->trgcfg[(addr - BCTU_TRGCFG(0)) / 4] = v;
        return;
    }

    switch (addr) {
    case BCTU_MCR:
        s->mcr = v & MCR_MDIS;
        break;
    case BCTU_MSR:
        s->msr &= ~v;   /* W1C（TRGF_CLR/NDATA*_CLR 等） */
        break;
    case BCTU_FIFOCR:
        s->fifocr = v & (FIFOCR_IEN_FIFO1 | 0x03000000u);
        s32k3_bctu_update_irq(s);
        break;
    case BCTU_FIFOWM:
        s->fifo_wm = v & FIFOWM_WM_FIFO1;
        break;
    case BCTU_FIFOERR:
        s->fifoerr &= ~(v & FIFOERR_WM_INT_FIFO1);   /* W1C */
        s32k3_bctu_update_irq(s);
        break;
    case BCTU_SFTRGR1:
        /* 软件触发：写非 0 -> 触发对应 trig（RM SFTRGR 位对应触发线） */
        if (v) {
            int tg = __builtin_ctz(v);
            if (tg < S32K3_BCTU_TRIGGERS) {
                s32k3_bctu_fire(s, tg);
            }
        }
        break;
    default:
        /* 影子数组 0x100 项而 MMIO 窗口 0x4000 字节：越界写会踩到
         * 紧随其后的 adc_trig[3]/adc_done[3] qemu_irq 指针，必须限界。 */
        if (addr < sizeof(s->regs)) {
            s->regs[addr / 4] = v;
        }
    }
}

static const MemoryRegionOps s32k3_bctu_ops = {
    .read = s32k3_bctu_read,
    .write = s32k3_bctu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static void s32k3_bctu_init(Object *obj)
{
    S32K3BctuState *s = S32K3_BCTU(obj);

    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);

    memory_region_init_io(&s->iomem, obj, &s32k3_bctu_ops, s,
                          TYPE_S32K3_BCTU, 0x4000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);

    qdev_init_gpio_in_named(DEVICE(s), s32k3_bctu_trig_set, "trig-in",
                            S32K3_BCTU_TRIGGERS);
    qdev_init_gpio_out_named(DEVICE(s), s->adc_trig, "adc-trig", 3);
    qdev_init_gpio_in_named(DEVICE(s), s32k3_bctu_adc_done, "adc-done", 3);
}

static void s32k3_bctu_realize(DeviceState *dev, Error **errp)
{
    S32K3BctuState *s = S32K3_BCTU(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_bctu: module_clk must be connected");
        return;
    }
    s32k3_bctu_reset(dev);
}

static void s32k3_bctu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_bctu_reset);
    dc->realize = s32k3_bctu_realize;
    dc->desc = "NXP S32K3xx BCTU (ADC trigger router + result FIFO)";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo s32k3_bctu_types[] = {
    {
        .name          = TYPE_S32K3_BCTU,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3BctuState),
        .instance_init = s32k3_bctu_init,
        .class_init    = s32k3_bctu_class_init,
    },
};

DEFINE_TYPES(s32k3_bctu_types)
