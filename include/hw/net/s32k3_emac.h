/*
 * NXP S32K3xx EMAC (Ethernet MAC, 10/100/1000) QEMU device model - header
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_S32K3_EMAC_H
#define HW_NET_S32K3_EMAC_H

#include "hw/core/sysbus.h"
#include "net/net.h"
#include "hw/core/clock.h"
#include "qom/object.h"

#define TYPE_S32K3_EMAC "s32k3-emac"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3EmacState, S32K3_EMAC)

#define S32K3_EMAC_MMIO_SIZE 0x2000

/* Register offsets (subset of DWC-ether_qos style layout used by S32K3) */
#define EMAC_MAC_CONFIGURATION            0x0000
#define EMAC_MAC_EXT_CONFIGURATION        0x0004
#define EMAC_MAC_PACKET_FILTER            0x0008
#define EMAC_MAC_HASH_TAB_0               0x0010
#define EMAC_MAC_HASH_TAB_1               0x0014
#define EMAC_MAC_Q0_TX_FLOW_CTRL          0x0070
#define EMAC_MAC_RX_FLOW_CTRL             0x0090
#define EMAC_MAC_INTERRUPT_STATUS         0x00B0
#define EMAC_MAC_INTERRUPT_ENABLE         0x00B4
#define EMAC_MAC_RX_TX_STATUS             0x00B8
#define EMAC_MAC_PMT_CTRL_STATUS          0x00C0
#define EMAC_MAC_ADDRESS0_HIGH            0x0300
#define EMAC_MAC_ADDRESS0_LOW             0x0304
#define EMAC_MAC_ADDRESS_HIGH(n)          (0x0300 + 8 * (n))
#define EMAC_MAC_ADDRESS_LOW(n)           (0x0304 + 8 * (n))
#define EMAC_MAC_PHYIF_CTRL_STATUS        0x00F8
#define EMAC_MTL_OPERATION_MODE           0x0C00
#define EMAC_MTL_TXQ0_OP_MODE             0x0D00
#define EMAC_MTL_RXQ0_OP_MODE             0x0D30
#define EMAC_DMA_MODE                     0x1000
#define EMAC_DMA_SYSBUS_MODE              0x1004
#define EMAC_DMA_INTERRUPT_STATUS         0x1008
#define EMAC_DMA_CH0_CONTROL              0x1100
#define EMAC_DMA_CH0_TX_CONTROL           0x1104
#define EMAC_DMA_CH0_RX_CONTROL           0x1108
#define EMAC_DMA_CH0_TXDESC_LIST_ADDR     0x1114
#define EMAC_DMA_CH0_RXDESC_LIST_ADDR     0x111C
#define EMAC_DMA_CH0_TXDESC_TAIL_PTR      0x1120
#define EMAC_DMA_CH0_RXDESC_TAIL_PTR      0x1128
#define EMAC_DMA_CH0_TXDESC_RING_LEN      0x112C
#define EMAC_DMA_CH0_RXDESC_RING_LEN      0x1130
#define EMAC_DMA_CH0_INT_ENABLE           0x1134
#define EMAC_DMA_CH0_STATUS               0x1160

/* MAC_CONFIGURATION bits */
#define MAC_CONF_RE          (1 << 0)   /* receiver enable */
#define MAC_CONF_TE          (1 << 1)   /* transmitter enable */
#define MAC_CONF_PS          (1 << 15)

/* MAC_PACKET_FILTER bits (RM: 0x14) */
#define MAC_PF_PR            (1 << 0)   /* promiscuous mode */

/* DMA_MODE bits */
#define DMA_MODE_SWR         (1 << 0)   /* software reset */

/* DMA_CH0 status interrupt bits */
#define DMA_CH0_INT_TI       (1 << 0)   /* transmit interrupt */
#define DMA_CH0_INT_RI       (1 << 6)   /* receive interrupt */
#define DMA_CH0_INT_NIS      (1 << 15)
#define DMA_CH0_INT_AIS      (1 << 14)

/* DMA_CH0_CONTROL bits */
#define DMA_CH0_CTRL_ST      (1 << 0)   /* start TX DMA */
#define DMA_CH0_CTRL_SR      (1 << 1)   /* start RX DMA */

/* simplified frame data window (board-level learning model):
 * instead of full DMA descriptor parsing, TX frames are pushed through
 * a 2KB TX FIFO window and RX frames delivered into a 2KB RX window.
 * This keeps the model compact while remaining functional for learning.
 */
#define EMAC_TX_FIFO_WINDOW  0x1800
#define EMAC_RX_FIFO_WINDOW  0x1A00
#define EMAC_FIFO_WINDOW_SIZE 512
#define EMAC_TX_DOORBELL     0x1F00
#define EMAC_RX_FRAME_LEN    0x1F04

struct S32K3EmacState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock        *module_clk;
    qemu_irq     irq;

    NICState *nic;
    NICConf   conf;

    uint32_t mac_configuration;
    uint32_t mac_packet_filter;
    uint32_t mac_interrupt_status;
    uint32_t mac_interrupt_enable;
    uint32_t mac_addr_high[32];
    uint32_t mac_addr_low[32];
    uint32_t dma_mode;
    uint32_t dma_sysbus_mode;
    uint32_t dma_ch0_control;
    uint32_t dma_ch0_tx_control;
    uint32_t dma_ch0_rx_control;
    uint32_t dma_ch0_int_enable;
    uint32_t dma_ch0_status;
    uint32_t dma_ch0_txdesc_list;
    uint32_t dma_ch0_rxdesc_list;

    uint8_t  tx_fifo[2048];
    uint32_t tx_fifo_len;
    uint8_t  rx_fifo[2048];
    uint32_t rx_fifo_len;
};

#endif /* HW_NET_S32K3_EMAC_H */
