/*
 * NXP S32K3xx LPI2C (Low Power I2C) master QEMU device model
 *
 * Master-mode transmit/receive FIFOs wired to the QEMU I2C framework
 * (hw/i2c), so any QEMU i2c slave device (EEPROMs, sensors) can be
 * attached to the bus.  Register layout per S32K3xx RM (LPI2C chapter).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/i2c/i2c.h"
#include "hw/core/qdev-clock.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_S32K3_LPI2C "s32k3-lpi2c"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3Lpi2cState, S32K3_LPI2C)

#define S32K3_LPI2C_FIFO_DEPTH 16

/* master registers */
#define LPI2C_VERID    0x00
#define LPI2C_PARAM    0x04
#define LPI2C_MCR      0x10
#define  MCR_MEN       (1 << 0)
#define  MCR_RST       (1 << 1)
#define  MCR_RTF       (1 << 9)
#define  MCR_RRF       (1 << 8)
#define LPI2C_MSR      0x14
#define  MSR_TDF       (1 << 0)
#define  MSR_RDF       (1 << 1)
#define  MSR_NDF       (1 << 5)   /* NACK detect */
#define  MSR_SDF       (1 << 6)   /* stop detect */
#define  MSR_ALF       (1 << 7)   /* arbitration lost */
#define  MSR_EPF       (1 << 8)   /* end/error packet */
#define  MSR_FEF       (1 << 9)   /* FIFO error */
#define  MSR_PLTF      (1 << 10)
#define  MSR_DMF       (1 << 13)
#define  MSR_BBF       (1 << 25)
#define LPI2C_MIER     0x18
#define  MIER_TDIE     (1 << 0)
#define  MIER_RDIE     (1 << 1)
#define LPI2C_MCFGR0   0x20
#define LPI2C_MCFGR1   0x24
#define  MCFGR1_AUTOSTOP (1 << 8)
#define LPI2C_MCFGR2   0x28
#define LPI2C_MCCR0    0x48    /* controller clock config 0 */
#define LPI2C_MCCR1    0x50    /* controller clock config 1 */
#define  MCCR_CLKLO_SHIFT 0
#define  MCCR_CLKHI_SHIFT 8
#define  MCCR_SETHOLD_SHIFT 16
#define  MCCR_DATAVD_SHIFT 24
#define LPI2C_MFCR     0x58
#define LPI2C_MFSR     0x5C
#define LPI2C_MTDR     0x60
#define  MTDR_CMD_MASK  (7 << 8)
#define  MTDR_CMD_TXDATA (0 << 8)
#define  MTDR_CMD_RXDATA (1 << 8)
#define  MTDR_CMD_STOP   (2 << 8)
#define  MTDR_CMD_RXDISC (3 << 8)
#define  MTDR_CMD_START  (4 << 8)
#define  MTDR_CMD_START_STOP (6 << 8)
#define LPI2C_MRDR     0x70
#define  MRDR_RXEMPTY  (1 << 14)

struct S32K3Lpi2cState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    I2CBus       *bus;
    Clock        *module_clk;
    qemu_irq     irq;

    uint32_t mcr;
    uint32_t msr;
    uint32_t mier;
    uint32_t mcfgr0;
    uint32_t mcfgr1;
    uint32_t baud_hz;   /* 计算的 SCL 波特率（MCCR+PRESCALE） */
    uint32_t mcfgr2;
    uint32_t mccr0;
    uint32_t mccr1;
    uint32_t mfcr;

    uint8_t  rx_fifo[S32K3_LPI2C_FIFO_DEPTH];
    uint32_t rx_fifo_len;
    uint32_t tx_fifo_len;

    /* transfer state */
    bool     started;
    uint8_t  cur_addr;
    bool     nak_seen;
};

static void s32k3_lpi2c_update_irq(S32K3Lpi2cState *s)
{
    bool tdf = (s->mier & MIER_TDIE) && (s->msr & MSR_TDF);
    bool rdf = (s->mier & MIER_RDIE) && (s->msr & MSR_RDF);
    qemu_set_irq(s->irq, tdf || rdf);
}

static void s32k3_lpi2c_rx_push(S32K3Lpi2cState *s, uint8_t b)
{
    if (s->rx_fifo_len < S32K3_LPI2C_FIFO_DEPTH) {
        s->rx_fifo[s->rx_fifo_len++] = b;
        s->msr |= MSR_RDF;
    }
    s32k3_lpi2c_update_irq(s);
}

static void s32k3_lpi2c_flush_rx(S32K3Lpi2cState *s)
{
    s->rx_fifo_len = 0;
    s->msr &= ~MSR_RDF;
}

static void s32k3_lpi2c_reset(DeviceState *dev)
{
    S32K3Lpi2cState *s = S32K3_LPI2C(dev);

    s->mcr = 0;
    s->msr = MSR_TDF | MSR_BBF | MSR_EPF;   /* idle: tx empty, bus free */
    s->mier = 0;
    s->mcfgr0 = 0;
    s->mcfgr1 = 0;
    s->mcfgr2 = 0;
    s->mccr0 = 0;
    s->mccr1 = 0;
    s->mfcr = 0;
    s->rx_fifo_len = 0;
    s->tx_fifo_len = 0;
    s->started = false;
    s->nak_seen = false;
    s32k3_lpi2c_flush_rx(s);
    s32k3_lpi2c_update_irq(s);
}

static void s32k3_lpi2c_do_stop(S32K3Lpi2cState *s)
{
    if (s->started) {
        i2c_end_transfer(s->bus);
        s->started = false;
    }
    s->msr |= MSR_EPF | MSR_BBF;
    s32k3_lpi2c_update_irq(s);
}

static void s32k3_lpi2c_mtdr_write(S32K3Lpi2cState *s, uint32_t v)
{
    uint32_t cmd = v & MTDR_CMD_MASK;
    uint8_t data = v & 0xff;

    if (!(s->mcr & MCR_MEN)) {
        return;
    }

    switch (cmd) {
    case MTDR_CMD_START:
    case MTDR_CMD_START_STOP:
        /* 7-bit address in data[7:1], R/W in data[0] */
        s->cur_addr = data >> 1;
        s->nak_seen = i2c_start_transfer(s->bus, s->cur_addr, data & 1);
        s->started = true;
        s->msr &= ~MSR_BBF;
        if (s->nak_seen) {
            /* NACK on address: set NDF + EPF (error flag) */
            s->msr |= MSR_NDF | MSR_EPF;
        }
        if (cmd == MTDR_CMD_START_STOP) {
            s32k3_lpi2c_do_stop(s);
        }
        break;
    case MTDR_CMD_TXDATA:
        if (s->started) {
            if (i2c_send(s->bus, data) < 0) {
                s->msr |= MSR_NDF | MSR_EPF;   /* NAK */
            }
        }
        break;
    case MTDR_CMD_RXDATA: {
        /* data field = number of bytes to receive - 1 */
        int count = data + 1, i;
        if (s->started) {
            for (i = 0; i < count; i++) {
                s32k3_lpi2c_rx_push(s, i2c_recv(s->bus));
            }
        }
        break;
    }
    case MTDR_CMD_STOP:
        s32k3_lpi2c_do_stop(s);
        break;
    case MTDR_CMD_RXDISC:
        /* receive and discard: consume without storing */
        if (s->started) {
            int count = data + 1, i;
            for (i = 0; i < count; i++) {
                i2c_recv(s->bus);
            }
        }
        break;
    default:
        break;
    }
    s32k3_lpi2c_update_irq(s);
}

static uint64_t s32k3_lpi2c_read(void *opaque, hwaddr addr, unsigned size)
{
    S32K3Lpi2cState *s = opaque;

    switch (addr) {
    case LPI2C_VERID:
        return 0x01040001;
    case LPI2C_PARAM:
        return (S32K3_LPI2C_FIFO_DEPTH << 24) |
               (S32K3_LPI2C_FIFO_DEPTH << 16) | 0x2;
    case LPI2C_MCR:
        return s->mcr;
    case LPI2C_MSR:
        return s->msr;
    case LPI2C_MIER:
        return s->mier;
    case LPI2C_MCFGR0:
        return s->mcfgr0;
    case LPI2C_MCFGR1:
        return s->mcfgr1;
    case LPI2C_MCFGR2:
        return s->mcfgr2;
    case LPI2C_MCCR0:
        return s->mccr0;
    case LPI2C_MCCR1:
        return s->mccr1;
    case LPI2C_MFCR:
        return s->mfcr;
    case LPI2C_MFSR:
        return s->tx_fifo_len | (s->rx_fifo_len << 16);
    case LPI2C_MRDR:
        if (s->rx_fifo_len > 0) {
            uint8_t b = s->rx_fifo[0];
            memmove(s->rx_fifo, s->rx_fifo + 1, --s->rx_fifo_len);
            if (s->rx_fifo_len == 0) {
                s->msr &= ~MSR_RDF;
            }
            s32k3_lpi2c_update_irq(s);
            return b;
        }
        return MRDR_RXEMPTY;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_lpi2c: read of unimplemented reg 0x%03" HWADDR_PRIx "\n",
                      addr);
        return 0;
    }
}

static void s32k3_lpi2c_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned size)
{
    S32K3Lpi2cState *s = opaque;
    uint32_t v = value;

    switch (addr) {
    case LPI2C_MCR:
        if (v & MCR_RST) {
            s32k3_lpi2c_reset(DEVICE(s));
            return;
        }
        s->mcr = v & ~(MCR_RST | MCR_RTF | MCR_RRF);
        if (v & MCR_RRF) {
            s32k3_lpi2c_flush_rx(s);
        }
        s->msr |= MSR_TDF;
        s32k3_lpi2c_update_irq(s);
        break;
    case LPI2C_MSR:
        s->msr &= ~(v & (MSR_EPF | MSR_NDF | MSR_SDF | MSR_ALF |
                         MSR_FEF | MSR_PLTF | MSR_DMF));   /* W1C */
        s32k3_lpi2c_update_irq(s);
        break;
    case LPI2C_MIER:
        s->mier = v & (MIER_TDIE | MIER_RDIE | (1 << 8) | (1 << 9));
        s32k3_lpi2c_update_irq(s);
        break;
    case LPI2C_MCFGR0:
        s->mcfgr0 = v;
        break;
    case LPI2C_MCFGR1:
        s->mcfgr1 = v;
        break;
    case LPI2C_MCFGR2:
        s->mcfgr2 = v;
        break;
    case LPI2C_MCCR0:
        s->mccr0 = v;
        /* fSCL = clk / ((PRESCALE+1) * (CLKHI + CLKLO + SETHOLD + DATAVD + 2) * 2)
         * PRESCALE = MCFGR1 bit30:24 */
        {
            uint32_t prescale = (s->mcfgr1 >> 24) & 0x7f;
            uint32_t lo = v & 0xff, hi = (v >> 8) & 0xff;
            uint32_t sh = (v >> 16) & 0xff, dv = (v >> 24) & 0xff;
            uint32_t clk = clock_get_hz(s->module_clk);
            s->baud_hz = clk / ((prescale + 1) * (hi + lo + sh + dv + 2) * 2);
        }
        break;
    case LPI2C_MCCR1:
        s->mccr1 = v;
        break;
    case LPI2C_MFCR:
        s->mfcr = v & 0x00030003;
        break;
    case LPI2C_MTDR:
        s32k3_lpi2c_mtdr_write(s, v);
        /* TX FIFO 计数（命令也占 FIFO 槽；TDF 表示空则递减） */
        if (s->tx_fifo_len < S32K3_LPI2C_FIFO_DEPTH) {
            s->tx_fifo_len++;
        }
        if (s->msr & MSR_TDF) {
            s->tx_fifo_len = 0;
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_lpi2c: write of unimplemented reg 0x%03" HWADDR_PRIx
                      " = 0x%08" PRIx64 "\n", addr, value);
    }
}

static const MemoryRegionOps s32k3_lpi2c_ops = {
    .read = s32k3_lpi2c_read,
    .write = s32k3_lpi2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void s32k3_lpi2c_init(Object *obj)
{
    S32K3Lpi2cState *s = S32K3_LPI2C(obj);

    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);
    s->bus = i2c_init_bus(DEVICE(s), "i2c");

    memory_region_init_io(&s->iomem, obj, &s32k3_lpi2c_ops, s,
                          TYPE_S32K3_LPI2C, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

static void s32k3_lpi2c_realize(DeviceState *dev, Error **errp)
{
    S32K3Lpi2cState *s = S32K3_LPI2C(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_lpi2c: module_clk must be connected");
        return;
    }
    s32k3_lpi2c_reset(dev);
}

static void s32k3_lpi2c_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_lpi2c_reset);
    dc->realize = s32k3_lpi2c_realize;
    dc->desc = "NXP S32K3xx LPI2C master";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo s32k3_lpi2c_types[] = {
    {
        .name          = TYPE_S32K3_LPI2C,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3Lpi2cState),
        .instance_init = s32k3_lpi2c_init,
        .class_init    = s32k3_lpi2c_class_init,
    },
};

DEFINE_TYPES(s32k3_lpi2c_types)
