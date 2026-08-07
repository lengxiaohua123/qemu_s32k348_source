/*
 * NXP S32K3xx LPSPI (Low Power Serial Peripheral Interface)
 * QEMU device model - header
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SPI_S32K3_LPSPI_H
#define HW_SPI_S32K3_LPSPI_H

#include "hw/core/sysbus.h"
#include "hw/ssi/ssi.h"
#include "hw/core/clock.h"
#include "qom/object.h"

#define TYPE_S32K3_LPSPI "s32k3-lpspi"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3LpspiState, S32K3_LPSPI)

#define S32K3_LPSPI_MMIO_SIZE 0x1000

/* Register offsets (S32K3xx RM, LPSPI chapter) */
#define LPSPI_VERID     0x000
#define LPSPI_PARAM     0x004
#define LPSPI_MCR       0x008   /* Module Control (MEN/MASTER/RST) */
#define LPSPI_MODIR     0x00C   /* Module Configuration */
#define LPSPI_CR        0x010   /* Control */
#define LPSPI_SR        0x014   /* Status */
#define LPSPI_IER       0x018   /* Interrupt Enable */
#define LPSPI_DER       0x01C   /* DMA Enable */
#define LPSPI_CFGR0     0x020   /* Configuration 0 */
#define LPSPI_CFGR1     0x024   /* Configuration 1 */
#define LPSPI_DMR0      0x030
#define LPSPI_DMR1      0x034
#define LPSPI_CCR       0x040   /* Clock Configuration */
#define LPSPI_FCR       0x058   /* FIFO Control */
#define LPSPI_FSR       0x05C   /* FIFO Status */
#define LPSPI_TCR       0x060   /* Transmit Command */
#define LPSPI_TDR       0x064   /* Transmit Data */
#define LPSPI_RSR       0x070   /* Receive Status */
#define LPSPI_RDR       0x074   /* Receive Data */

/* CR */
#define CR_RRF          (1 << 9)    /* reset RX fifo */
#define CR_RTF          (1 << 8)    /* reset TX fifo */
#define CR_DBG          (1 << 3)
#define CR_DOZEN        (1 << 2)
#define CR_RST          (1 << 1)
#define CR_MEN          (1 << 0)

/* SR */
#define SR_FCF          (1 << 8)
#define SR_TEF          (1 << 4)
#define SR_TDF          (1 << 0)
#define SR_RDF          (1 << 1)
#define SR_DMF          (1 << 13)
#define SR_MBF          (1 << 24)

/* TCR（S32K3 RM：TXMSK31/RXMSK30/CONTC29/CONT28/BYSWAP27/LSBF26/
 * PCS25-24/PRESCALE22-19/PCSPOL18/FRAMESZ17-6/WIDTH5-3） */
#define TCR_TXMSK       (1 << 31)
#define TCR_RXMSK       (1 << 30)
#define TCR_CONTC       (1 << 29)
#define TCR_CONT        (1 << 28)
#define TCR_BYSWAP      (1 << 27)
#define TCR_LSBF        (1 << 26)
#define TCR_PCS_SHIFT   24
#define TCR_PCS_MASK    (3 << TCR_PCS_SHIFT)
#define TCR_FRAMESZ_SHIFT 6
#define TCR_FRAMESZ_MASK (0xfff << TCR_FRAMESZ_SHIFT)

/* FSR */
#define FSR_TXCOUNT_SHIFT 0
#define FSR_RXCOUNT_SHIFT 16

#define S32K3_LPSPI_FIFO_DEPTH 16

struct S32K3LpspiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SSIBus       *spi;
    Clock        *module_clk;
    qemu_irq     irq;

    uint32_t verid;
    uint32_t param;
    uint32_t mcr;
    uint32_t modir;
    uint32_t cr;
    uint32_t sr;
    uint32_t ier;
    uint32_t der;
    uint32_t cfgr0;
    uint32_t cfgr1;
    uint32_t ccr;
    uint32_t baud_hz;   /* 计算出的 SCK 波特率（CCR SCKDIV/SCALE） */
    uint32_t fcr;
    uint32_t tcr;
    uint32_t rsr;

    /* rx fifo */
    uint32_t rx_fifo[S32K3_LPSPI_FIFO_DEPTH];
    uint32_t rx_fifo_len;
    uint32_t tx_fifo_len;
};

#endif /* HW_SPI_S32K3_LPSPI_H */
