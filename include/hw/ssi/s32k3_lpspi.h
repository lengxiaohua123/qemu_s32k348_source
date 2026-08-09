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

/* SR（S32K3xx RM 70.6.3.8：TDF0/RDF1/WCF8/FCF9/TCF10/TEF11/REF12/DMF13/MBF24） */
#define SR_WCF          (1 << 8)
#define SR_FCF          (1 << 9)
#define SR_TCF          (1 << 10)
#define SR_TEF          (1 << 11)
#define SR_REF          (1 << 12)
#define SR_DMF          (1 << 13)
#define SR_MBF          (1 << 24)
#define SR_TDF          (1 << 0)
#define SR_RDF          (1 << 1)
#define SR_W1C_MASK     (SR_WCF | SR_FCF | SR_TCF | SR_TEF | SR_REF | SR_DMF)

/* TCR（S32K3xx RM 70.6.3.15：CPOL31/CPHA30/PRESCALE29-27/PCS26-24/LSBF23/
 * BYSW22/CONT21/RXMSK20/TXMSK19/WIDTH17-16/FRAMESZ11-0） */
#define TCR_CPOL        (1 << 31)
#define TCR_CPHA        (1 << 30)
#define TCR_PRESCALE_SHIFT 27
#define TCR_PRESCALE_MASK (0x7 << TCR_PRESCALE_SHIFT)
#define TCR_PCS_SHIFT   24
#define TCR_PCS_MASK    (0x7 << TCR_PCS_SHIFT)
#define TCR_LSBF        (1 << 23)
#define TCR_BYSW        (1 << 22)
#define TCR_CONT        (1 << 21)
#define TCR_RXMSK       (1 << 20)
#define TCR_TXMSK       (1 << 19)
#define TCR_WIDTH_SHIFT 16
#define TCR_WIDTH_MASK  (0x3 << TCR_WIDTH_SHIFT)
#define TCR_FRAMESZ_SHIFT 0   /* S32K3: FRAMESZ 低位 bit0-11（复位 0x1F=31） */
#define TCR_FRAMESZ_MASK (0xfff << TCR_FRAMESZ_SHIFT)

/* CCR（S32K3xx RM：SCKPCS31-24/PCSSCK23-16/DBT15-12/SCKDIV11-0） */
#define CCR_SCKPCS_SHIFT 24
#define CCR_SCKPCS_MASK (0xff << CCR_SCKPCS_SHIFT)
#define CCR_PCSSCK_SHIFT 16
#define CCR_PCSSCK_MASK (0xff << CCR_PCSSCK_SHIFT)
#define CCR_DBT_SHIFT   12
#define CCR_DBT_MASK    (0xf << CCR_DBT_SHIFT)
#define CCR_SCKDIV_SHIFT 0
#define CCR_SCKDIV_MASK (0xfff << CCR_SCKDIV_SHIFT)

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
