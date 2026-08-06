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
#define  MCR_MDIS       (1 << 0)
#define BCTU_GSR        0x04
#define BCTU_IER        0x08
#define  IER_I0         (1 << 0)
#define BCTU_IFR        0x0C
#define BCTU_TRGCFG(n)  (0x40 + 4 * (n))
#define  TRGCFG_TRGEN   (1 << 31)
#define  TRGCFG_ADCSEL  (3 << 24)      /* 0=ADC0 1=ADC1 2=ADC2 */
#define  TRGCFG_ADCSEL_SHIFT 24
#define  TRGCFG_CHMASK  0x00FFFFFF     /* channel mask bits 0..23 */
#define BCTU_FIFO_DR    0x80
#define  FIFODR_VALID   (1 << 19)
#define  FIFODR_CHN_SHIFT 20
#define BCTU_FIFO_SR    0x84
#define BCTU_FIFO_WM    0x88

struct S32K3BctuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock        *module_clk;
    qemu_irq     irq;

    uint32_t mcr;
    uint32_t ier;
    uint32_t ifr;
    uint32_t trgcfg[S32K3_BCTU_TRIGGERS];

    /* result fifo */
    uint32_t fifo[S32K3_BCTU_FIFO];
    uint32_t fifo_len;
    uint32_t fifo_wm;

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
    qemu_set_irq(s->irq, (s->ier & IER_I0) && (s->ifr & IER_I0));
}

static void s32k3_bctu_fire(S32K3BctuState *s, int n)
{
    uint32_t cfg = s->trgcfg[n];
    int adc = (cfg & TRGCFG_ADCSEL) >> TRGCFG_ADCSEL_SHIFT;
    uint32_t mask = cfg & TRGCFG_CHMASK;

    if (!(cfg & TRGCFG_TRGEN)) {
        return;
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
            s->fifo[s->fifo_len++] =
                FIFODR_VALID | (s->pending_ch << FIFODR_CHN_SHIFT) |
                (pcdr & 0xFFF);
        }
        s->pending_valid = false;
    }
    if (s->fifo_len > s->fifo_wm) {
        s->ifr |= IER_I0;
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
    s->ier = 0;
    s->ifr = 0;
    memset(s->trgcfg, 0, sizeof(s->trgcfg));
    s->fifo_len = 0;
    s->fifo_wm = 4;
    s->pending_valid = false;
    s32k3_bctu_update_irq(s);
}

static uint64_t s32k3_bctu_read(void *opaque, hwaddr addr, unsigned size)
{
    S32K3BctuState *s = opaque;

    if (addr >= BCTU_TRGCFG(0) && addr < BCTU_TRGCFG(S32K3_BCTU_TRIGGERS)) {
        return s->trgcfg[(addr - BCTU_TRGCFG(0)) / 4];
    }

    switch (addr) {
    case BCTU_MCR:
        return s->mcr;
    case BCTU_GSR:
        return 0;
    case BCTU_IER:
        return s->ier;
    case BCTU_IFR:
        return s->ifr;
    case BCTU_FIFO_DR:
        if (s->fifo_len > 0) {
            uint32_t r = s->fifo[0];
            memmove(s->fifo, s->fifo + 1,
                    (--s->fifo_len) * sizeof(uint32_t));
            return r;
        }
        return 0;
    case BCTU_FIFO_SR:
        return s->fifo_len;
    case BCTU_FIFO_WM:
        return s->fifo_wm;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_bctu: read of unimplemented reg 0x%03" HWADDR_PRIx "\n",
                      addr);
        return 0;
    }
}

static void s32k3_bctu_write(void *opaque, hwaddr addr,
                             uint64_t value, unsigned size)
{
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
    case BCTU_IER:
        s->ier = v & IER_I0;
        s32k3_bctu_update_irq(s);
        break;
    case BCTU_IFR:
        s->ifr &= ~v;
        s32k3_bctu_update_irq(s);
        break;
    case BCTU_FIFO_WM:
        s->fifo_wm = v & 0xf;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_bctu: write of unimplemented reg 0x%03" HWADDR_PRIx
                      " = 0x%08" PRIx64 "\n", addr, value);
    }
}

static const MemoryRegionOps s32k3_bctu_ops = {
    .read = s32k3_bctu_read,
    .write = s32k3_bctu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
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
