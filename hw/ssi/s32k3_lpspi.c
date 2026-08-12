/*
 * NXP S32K3xx LPSPI QEMU device model
 *
 * Functional master-mode LPSPI.  Every TDR write shifts data out on the
 * QEMU SSI bus (hw/ssi).  Attach slave devices on the board or loop a
 * ssi-loopback for self-tests:
 *
 *   qemu-system-arm -M s32k348evb -device ssi-loopback
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/ssi/s32k3_lpspi.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "qemu/log.h"
#include "qemu/module.h"

static void s32k3_lpspi_update_irq(S32K3LpspiState *s)
{
    bool level = false;

    if ((s->ier & SR_TDF) && (s->sr & SR_TDF)) {
        level = true;
    }
    if ((s->ier & SR_RDF) && (s->sr & SR_RDF)) {
        level = true;
    }
    if ((s->ier & SR_TEF) && (s->sr & SR_TEF)) {
        level = true;
    }
    qemu_set_irq(s->irq, level);
}

static void s32k3_lpspi_flush_rx_fifo(S32K3LpspiState *s)
{
    s->rx_fifo_len = 0;
    s->sr &= ~SR_RDF;
}

static void s32k3_lpspi_reset(DeviceState *dev)
{
    S32K3LpspiState *s = S32K3_LPSPI(dev);

    s->verid = 0x01040004;
    s->mcr = 0;
    s->modir = 0;
    s->param = (S32K3_LPSPI_FIFO_DEPTH << 16) |
               (S32K3_LPSPI_FIFO_DEPTH << 24) | 4; /* txfifo|rxfifo|pcsnum */
    s->cr    = 0;
    s->sr    = SR_TDF;
    s->ier   = 0;
    s->der   = 0;
    s->cfgr0 = 0;
    s->cfgr1 = 0x00000000;
    s->ccr   = 0;
    s->fcr   = 0;
    s->tcr   = (31) << TCR_FRAMESZ_SHIFT;  /* FRAMESZ=31: 32 位帧复位（手册 0000_001Fh） */
    s->rsr   = 0x2;   /* rxempty */
    s->tx_fifo_len = 0;
    s32k3_lpspi_flush_rx_fifo(s);
    s32k3_lpspi_update_irq(s);
}

static void s32k3_lpspi_tdr_write(S32K3LpspiState *s, uint32_t value)
{
    uint32_t framesz = ((s->tcr & TCR_FRAMESZ_MASK) >> TCR_FRAMESZ_SHIFT) + 1;
    uint32_t nbytes = (framesz + 7) / 8;
    uint32_t i, rx = 0;

    if (!(s->cr & CR_MEN)) {
        s->sr |= SR_TEF;
        s32k3_lpspi_update_irq(s);
        return;
    }

    s->sr &= ~SR_TDF;
    s->sr |= SR_MBF;
    if (s->tx_fifo_len < S32K3_LPSPI_FIFO_DEPTH) {
        s->tx_fifo_len++;
    }

    if (nbytes == 0) {
        nbytes = 1;
    }
    if (nbytes > 4) {
        nbytes = 4;   /* model up to 32-bit words */
    }

    for (i = 0; i < nbytes; i++) {
        uint8_t tx_byte = (value >> (8 * i)) & 0xff;
        uint8_t rx_byte;

        if (s->tcr & TCR_TXMSK) {
            tx_byte = 0xff;
        }
        if (s->tcr & TCR_RXMSK) {
            rx_byte = 0;
        } else {
            rx_byte = ssi_transfer(s->spi, tx_byte);
        }
        rx |= rx_byte << (8 * i);
    }

    s->sr |= SR_MBF;
    s->sr &= ~SR_MBF;
    /* 帧传输完成：置 TCF（传输完成标志，W1C 清）——RTD
     * Lpspi_Ip_SyncTransmit 收尾轮询 SR.TCF，原从不置位会超时挂死。 */
    s->sr |= SR_TCF;

    if (!(s->tcr & TCR_RXMSK)) {
        if (s->rx_fifo_len < S32K3_LPSPI_FIFO_DEPTH) {
            s->rx_fifo[s->rx_fifo_len++] = rx;
            s->sr |= SR_RDF;
            s->rsr &= ~0x2;
        } else {
            s->sr |= SR_TEF;
        }
    }
    s->sr |= SR_TDF;
    s32k3_lpspi_update_irq(s);
}

static uint64_t s32k3_lpspi_read(void *opaque, hwaddr addr, unsigned size)
{
    S32K3LpspiState *s = opaque;
    uint32_t r = 0;

    switch (addr) {
    case LPSPI_VERID:
        r = s->verid;
        break;
    case LPSPI_PARAM:
        r = s->param;
        break;
    case LPSPI_MCR:
        r = s->mcr;
        break;
    case LPSPI_MODIR:
        r = s->modir;
        break;
    case LPSPI_CR:
        r = s->cr;
        break;
    case LPSPI_SR:
        r = s->sr;
        break;
    case LPSPI_IER:
        r = s->ier;
        break;
    case LPSPI_DER:
        r = s->der;
        break;
    case LPSPI_CFGR0:
        r = s->cfgr0;
        break;
    case LPSPI_CFGR1:
        r = s->cfgr1;
        break;
    case LPSPI_CCR:
        r = s->ccr;
        break;
    case LPSPI_FCR:
        r = s->fcr;
        break;
    case LPSPI_FSR:
        r = s->tx_fifo_len << FSR_TXCOUNT_SHIFT |
            (s->rx_fifo_len << FSR_RXCOUNT_SHIFT);
        break;
    case LPSPI_TCR:
        r = s->tcr;
        break;
    case LPSPI_RSR:
        r = s->rsr;
        break;
    case LPSPI_RDR:
        if (s->rx_fifo_len > 0) {
            r = s->rx_fifo[0];
            memmove(s->rx_fifo, s->rx_fifo + 1,
                    (--s->rx_fifo_len) * sizeof(uint32_t));
            if (s->rx_fifo_len == 0) {
                s->sr &= ~SR_RDF;
                s->rsr |= 0x2;
            }
            s32k3_lpspi_update_irq(s);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_lpspi: read of unimplemented reg 0x%02" HWADDR_PRIx "\n",
                      addr);
    }
    return r;
}

static void s32k3_lpspi_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned size)
{
    S32K3LpspiState *s = opaque;
    uint32_t v = value;

    switch (addr) {
    case LPSPI_MCR:
        s->mcr = v;
        break;
    case LPSPI_MODIR:
        s->modir = v;
        break;
    case LPSPI_CR:
        if (v & CR_RST) {
            s32k3_lpspi_reset(DEVICE(s));
            return;
        }
        s->cr = v & ~(CR_RST | CR_RTF | CR_RRF);
        if (v & CR_RRF) {
            s32k3_lpspi_flush_rx_fifo(s);
        }
        s32k3_lpspi_update_irq(s);
        break;
    case LPSPI_SR:
        s->sr &= ~(v & SR_W1C_MASK); /* W1C */
        s32k3_lpspi_update_irq(s);
        break;
    case LPSPI_IER:
        s->ier = v;
        s32k3_lpspi_update_irq(s);
        break;
    case LPSPI_DER:
        s->der = v;
        break;
    case LPSPI_CFGR0:
        s->cfgr0 = v;
        break;
    case LPSPI_CFGR1:
        s->cfgr1 = v;
        break;
    case LPSPI_CCR:
        s->ccr = v;
        /* SCK = module_clk / (2^PRESCALE * (SCKDIV + 1))
         * SCKDIV = CCR[11:0]，PRESCALE = TCR[29:27]（RM 70.6.3.15/70.6.3.11） */
        {
            uint32_t sckdiv = (v & CCR_SCKDIV_MASK) >> CCR_SCKDIV_SHIFT;
            uint32_t prescale = (s->tcr & TCR_PRESCALE_MASK) >> TCR_PRESCALE_SHIFT;
            uint32_t clk = clock_get_hz(s->module_clk);
            s->baud_hz = clk / ((1u << prescale) * (sckdiv + 1));
        }
        break;
    case LPSPI_FCR:
        s->fcr = v & 0x001f001f;
        break;
    case LPSPI_TCR:
        /* TCR may only be written when not busy */
        s->tcr = v;
        break;
    case LPSPI_TDR:
        s32k3_lpspi_tdr_write(s, v);
        break;
    case LPSPI_RSR:
        /* W1C SOF/RXEMPTY bits ignored */
        break;
    case LPSPI_VERID:
    case LPSPI_PARAM:
    case LPSPI_FSR:
    case LPSPI_RDR:
        break; /* read-only */
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_lpspi: write of unimplemented reg 0x%02" HWADDR_PRIx
                      " = 0x%08" PRIx64 "\n", addr, value);
    }
}

static const MemoryRegionOps s32k3_lpspi_ops = {
    .read = s32k3_lpspi_read,
    .write = s32k3_lpspi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void s32k3_lpspi_init(Object *obj)
{
    S32K3LpspiState *s = S32K3_LPSPI(obj);

    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);
    s->spi = ssi_create_bus(DEVICE(s), "spi");

    memory_region_init_io(&s->iomem, obj, &s32k3_lpspi_ops, s,
                          TYPE_S32K3_LPSPI, S32K3_LPSPI_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

static void s32k3_lpspi_realize(DeviceState *dev, Error **errp)
{
    S32K3LpspiState *s = S32K3_LPSPI(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_lpspi: module_clk must be connected");
        return;
    }
    s32k3_lpspi_reset(dev);
}

static void s32k3_lpspi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_lpspi_reset);
    dc->realize = s32k3_lpspi_realize;
    dc->desc = "NXP S32K3xx LPSPI";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo s32k3_lpspi_types[] = {
    {
        .name          = TYPE_S32K3_LPSPI,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3LpspiState),
        .instance_init = s32k3_lpspi_init,
        .class_init    = s32k3_lpspi_class_init,
    },
};

DEFINE_TYPES(s32k3_lpspi_types)
