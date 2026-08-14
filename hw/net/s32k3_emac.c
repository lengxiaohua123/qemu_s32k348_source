/*
 * NXP S32K3xx EMAC QEMU device model (simplified DWC-style GMAC)
 *
 * Functional Ethernet model attached to a QEMU netdev backend:
 *
 *   Linux:   -netdev tap,id=net0,ifname=tap0,script=no \
 *            -device ...   (board wires nic automatically)
 *   Windows: -netdev socket / bridge via TAP-Windows driver.
 *
 * For embedded-learning firmware this model uses a compact FIFO-window
 * datapath: the guest pushes a full frame into the TX window and rings
 * the doorbell; received frames appear in the RX window with an
 * interrupt.  The register block mirrors the S32K3 EMAC address map so
 * driver probing code finds the expected layout.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/net/s32k3_emac.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-clock.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "system/address-spaces.h"

static void s32k3_emac_update_irq(S32K3EmacState *s)
{
    bool level = (s->dma_ch0_status & s->dma_ch0_int_enable) != 0;
    qemu_set_irq(s->irq, level);
}

static bool s32k3_emac_can_receive(NetClientState *nc)
{
    S32K3EmacState *s = qemu_get_nic_opaque(nc);

    return (s->mac_configuration & MAC_CONF_RE) && s->rx_fifo_len == 0;
}

static ssize_t s32k3_emac_receive(NetClientState *nc, const uint8_t *buf,
                                  size_t size)
{
    S32K3EmacState *s = qemu_get_nic_opaque(nc);

    if (!(s->mac_configuration & MAC_CONF_RE)) {
        return -1;
    }
    /* MAC 地址过滤（MAC_ADDRESS0_HIGH/LOW）：目标地址匹配自身 MAC 才接收。
     * 广播/组播（bit0=1）始终接收；PR（promiscuous）模式下接收全部。 */
    if (!(s->mac_packet_filter & MAC_PF_PR) && size >= 6) {
        uint8_t da[6];
        int i;
        bool match = true;

        memcpy(da, buf, 6);
        if ((da[0] & 1) == 1) {
            match = true;   /* multicast/broadcast */
        } else {
            /* MAC_ADDRESS0: low[0]=byte0, low[1]=byte1, ..., high byte4/5 */
            uint8_t own[6];
            own[0] = s->mac_addr_low[0] & 0xFF;
            own[1] = (s->mac_addr_low[0] >> 8) & 0xFF;
            own[2] = (s->mac_addr_low[0] >> 16) & 0xFF;
            own[3] = (s->mac_addr_low[0] >> 24) & 0xFF;
            own[4] = s->mac_addr_high[0] & 0xFF;
            own[5] = (s->mac_addr_high[0] >> 8) & 0xFF;
            for (i = 0; i < 6; i++) {
                if (own[i] != 0 && da[i] != own[i]) {
                    match = false;
                    break;
                }
            }
        }
        if (!match) {
            return 0;   /* 地址不匹配，丢弃 */
        }
    }
    /* RX DMA descriptor ring：优先写入 RDES buffer（dma_ch0_rxdesc_list
     * 指向 Synopsys RDES：RDES0=状态/OWN、RDES2=buffer 指针）。 */
    if (s->dma_ch0_rxdesc_list != 0 &&
        (s->dma_ch0_control & DMA_CH0_CTRL_SR)) {
        uint32_t rdes[4];
        if (address_space_read(&address_space_memory, s->dma_ch0_rxdesc_list,
                               MEMTXATTRS_UNSPECIFIED, rdes, 16) == MEMTX_OK &&
            (rdes[0] & (1u << 31)) &&    /* OWN=1 可写 */
            rdes[2] != 0) {
            uint32_t len = size > 2048 ? 2048 : size;
            address_space_write(&address_space_memory, rdes[2],
                                MEMTXATTRS_UNSPECIFIED, buf, len);
            /* RDES0: 清 OWN，置 LAST(bit9)/FS(bit8)/长度[15:0] */
            rdes[0] &= ~(1u << 31);
            rdes[0] |= (1u << 9) | (1u << 8) | (len & 0xFFFF);
            address_space_write(&address_space_memory, s->dma_ch0_rxdesc_list,
                                MEMTXATTRS_UNSPECIFIED, rdes, 16);
            s->dma_ch0_status |= DMA_CH0_INT_RI | DMA_CH0_INT_NIS;
            s->mac_interrupt_status |= 1;
            s32k3_emac_update_irq(s);
            return size;
        }
    }
    /* 兼容模式：FIFO window */
    if (size > sizeof(s->rx_fifo)) {
        size = sizeof(s->rx_fifo);
    }
    memcpy(s->rx_fifo, buf, size);
    s->rx_fifo_len = size;
    s->dma_ch0_status |= DMA_CH0_INT_RI | DMA_CH0_INT_NIS;
    s->mac_interrupt_status |= 1;
    s32k3_emac_update_irq(s);
    return size;
}

static void s32k3_emac_tx_frame(S32K3EmacState *s)
{
    if (!(s->mac_configuration & MAC_CONF_TE)) {
        return;
    }

    /* 优先使用 DMA descriptor ring：dma_ch0_txdesc_list 指向内存中的
     * Synopsys GMAC TDES 数组。TDES0=状态/控制（bit0 OWN、长度在 [29:16]），
     * TDES2=buffer 指针。模型即时发送整个 ring（简化：只发首描述符）。 */
    if (s->dma_ch0_txdesc_list != 0 &&
        (s->dma_ch0_control & DMA_CH0_CTRL_ST)) {
        uint32_t tdes[4];
        uint8_t pkt[2048];
        uint32_t len;

        if (address_space_read(&address_space_memory, s->dma_ch0_txdesc_list,
                               MEMTXATTRS_UNSPECIFIED, tdes, 16) == MEMTX_OK) {
            len = (tdes[0] >> 16) & 0x3FFF;
            if (len == 0 || len > sizeof(pkt)) {
                len = 0;
            }
            if (len > 0 && tdes[2] != 0) {
                address_space_read(&address_space_memory, tdes[2],
                                   MEMTXATTRS_UNSPECIFIED, pkt, len);
                qemu_send_packet(qemu_get_queue(s->nic), pkt, len);
            }
            /* 清 OWN 位，置 TDES0[31]=OWN 清 0、[9]=LAST */
            tdes[0] &= ~(1u << 31);
            tdes[0] |= (1u << 9);
            address_space_write(&address_space_memory, s->dma_ch0_txdesc_list,
                                MEMTXATTRS_UNSPECIFIED, tdes, 16);
            s->dma_ch0_status |= DMA_CH0_INT_TI | DMA_CH0_INT_NIS;
            s32k3_emac_update_irq(s);
        }
        return;
    }

    /* 兼容模式：FIFO window */
    if (s->tx_fifo_len > 0) {
        qemu_send_packet(qemu_get_queue(s->nic), s->tx_fifo, s->tx_fifo_len);
        s->tx_fifo_len = 0;
        s->dma_ch0_status |= DMA_CH0_INT_TI | DMA_CH0_INT_NIS;
        s32k3_emac_update_irq(s);
    }
}

static void s32k3_emac_reset(DeviceState *dev)
{
    S32K3EmacState *s = S32K3_EMAC(dev);
    int i;

    s->mac_configuration = 0;
    s->mac_packet_filter = 0;
    s->mac_interrupt_status = 0;
    s->mac_interrupt_enable = 0;
    for (i = 0; i < 32; i++) {
        s->mac_addr_high[i] = 0x8000; /* address enable */
        s->mac_addr_low[i] = 0;
    }
    s->mac_addr_low[0] = (s->conf.macaddr.a[3] << 24) |
                         (s->conf.macaddr.a[2] << 16) |
                         (s->conf.macaddr.a[1] << 8) |
                          s->conf.macaddr.a[0];
    s->mac_addr_high[0] = 0x8000 |
                          (s->conf.macaddr.a[5] << 8) |
                           s->conf.macaddr.a[4];
    s->dma_mode = 0;
    s->dma_sysbus_mode = 0;
    s->dma_ch0_control = 0;
    s->dma_ch0_tx_control = 0;
    s->dma_ch0_rx_control = 0;
    s->dma_ch0_int_enable = 0;
    s->dma_ch0_status = 0;
    s->tx_fifo_len = 0;
    s->rx_fifo_len = 0;
    s32k3_emac_update_irq(s);
}

static uint64_t s32k3_emac_read(void *opaque, hwaddr addr, unsigned size)
{
    if (size == 8) {
        uint64_t lo = s32k3_emac_read(opaque, addr, 4);
        uint64_t hi = s32k3_emac_read(opaque, addr + 4, 4);
        return lo | (hi << 32);
    }
    if (size == 2) {
        uint32_t full = s32k3_emac_read(opaque, addr & ~3u, 4);
        return (addr & 2) ? (full >> 16) : (full & 0xFFFF);
    }
    if (size == 1) {
        uint32_t full = s32k3_emac_read(opaque, addr & ~3u, 4);
        return (full >> (8 * (addr & 3))) & 0xFF;
    }
    S32K3EmacState *s = opaque;
    uint32_t r = 0;

    if (addr >= EMAC_RX_FIFO_WINDOW &&
        addr < EMAC_RX_FIFO_WINDOW + EMAC_FIFO_WINDOW_SIZE) {
        hwaddr off = addr - EMAC_RX_FIFO_WINDOW;
        if (off < s->rx_fifo_len) {
            r = s->rx_fifo[off];
            if (size >= 2 && off + 1 < s->rx_fifo_len) {
                r |= s->rx_fifo[off + 1] << 8;
            }
            if (size >= 4 && off + 3 < s->rx_fifo_len) {
                r |= s->rx_fifo[off + 2] << 16 | s->rx_fifo[off + 3] << 24;
            }
        }
        return r;
    }

    switch (addr) {
    case EMAC_MAC_CONFIGURATION:
        r = s->mac_configuration;
        break;
    case EMAC_MAC_PACKET_FILTER:
        r = s->mac_packet_filter;
        break;
    case EMAC_MAC_INTERRUPT_STATUS:
        r = s->mac_interrupt_status;
        break;
    case EMAC_MAC_INTERRUPT_ENABLE:
        r = s->mac_interrupt_enable;
        break;
    case EMAC_MAC_ADDRESS0_HIGH:
        r = s->mac_addr_high[0];
        break;
    case EMAC_MAC_ADDRESS0_LOW:
        r = s->mac_addr_low[0];
        break;
    case EMAC_DMA_MODE:
        r = s->dma_mode;
        break;
    case EMAC_DMA_SYSBUS_MODE:
        r = s->dma_sysbus_mode;
        break;
    case EMAC_DMA_CH0_CONTROL:
        r = s->dma_ch0_control;
        break;
    case EMAC_DMA_CH0_TX_CONTROL:
        r = s->dma_ch0_tx_control;
        break;
    case EMAC_DMA_CH0_RX_CONTROL:
        r = s->dma_ch0_rx_control;
        break;
    case EMAC_DMA_CH0_INT_ENABLE:
        r = s->dma_ch0_int_enable;
        break;
    case EMAC_DMA_CH0_STATUS:
        r = s->dma_ch0_status;
        break;
    case EMAC_DMA_CH0_TXDESC_LIST_ADDR:
        r = s->dma_ch0_txdesc_list;
        break;
    case EMAC_DMA_CH0_RXDESC_LIST_ADDR:
        r = s->dma_ch0_rxdesc_list;
        break;
    case EMAC_RX_FRAME_LEN:
        r = s->rx_fifo_len;
        break;
    case EMAC_MAC_PHYIF_CTRL_STATUS:
        r = 0x7;  /* link up, 1000Mbps, full duplex */
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_emac: read of unimplemented reg 0x%03" HWADDR_PRIx "\n",
                      addr);
    }
    return r;
}

static void s32k3_emac_write(void *opaque, hwaddr addr,
                             uint64_t value, unsigned size)
{
    if (size == 8) {
        s32k3_emac_write(opaque, addr, value & 0xFFFFFFFF, 4);
        s32k3_emac_write(opaque, addr + 4, value >> 32, 4);
        return;
    }
    if (size == 2 || size == 1) {
        uint32_t full = s32k3_emac_read(opaque, addr & ~3u, 4);
        uint32_t sh = 8 * (addr & 3);
        uint32_t wmask = (size == 1) ? 0xFFu : 0xFFFFu;
        uint32_t merged = (full & ~(wmask << sh)) | ((value & wmask) << sh);
        s32k3_emac_write(opaque, addr & ~3u, merged, 4);
        return;
    }
    S32K3EmacState *s = opaque;
    uint32_t v = value;

    if (addr >= EMAC_TX_FIFO_WINDOW &&
        addr < EMAC_TX_FIFO_WINDOW + EMAC_FIFO_WINDOW_SIZE) {
        hwaddr off = addr - EMAC_TX_FIFO_WINDOW;
        if (off < sizeof(s->tx_fifo)) {
            s->tx_fifo[off] = v & 0xff;
            if (size >= 2 && off + 1 < sizeof(s->tx_fifo)) {
                s->tx_fifo[off + 1] = (v >> 8) & 0xff;
            }
            if (size >= 4 && off + 3 < sizeof(s->tx_fifo)) {
                s->tx_fifo[off + 2] = (v >> 16) & 0xff;
                s->tx_fifo[off + 3] = (v >> 24) & 0xff;
            }
        }
        return;
    }

    switch (addr) {
    case EMAC_MAC_CONFIGURATION:
        s->mac_configuration = v;
        break;
    case EMAC_MAC_PACKET_FILTER:
        s->mac_packet_filter = v;
        break;
    case EMAC_MAC_INTERRUPT_STATUS:
        s->mac_interrupt_status &= ~v;
        break;
    case EMAC_MAC_INTERRUPT_ENABLE:
        s->mac_interrupt_enable = v;
        s32k3_emac_update_irq(s);
        break;
    case EMAC_MAC_ADDRESS0_HIGH:
        s->mac_addr_high[0] = v;
        break;
    case EMAC_MAC_ADDRESS0_LOW:
        s->mac_addr_low[0] = v;
        break;
    case EMAC_DMA_MODE:
        s->dma_mode = v & ~DMA_MODE_SWR;
        if (v & DMA_MODE_SWR) {
            s32k3_emac_reset(DEVICE(s));
        }
        break;
    case EMAC_DMA_SYSBUS_MODE:
        s->dma_sysbus_mode = v;
        break;
    case EMAC_DMA_CH0_CONTROL:
        s->dma_ch0_control = v;
        break;
    case EMAC_DMA_CH0_TX_CONTROL:
        s->dma_ch0_tx_control = v;
        break;
    case EMAC_DMA_CH0_RX_CONTROL:
        s->dma_ch0_rx_control = v;
        break;
    case EMAC_DMA_CH0_INT_ENABLE:
        s->dma_ch0_int_enable = v;
        s32k3_emac_update_irq(s);
        break;
    case EMAC_DMA_CH0_STATUS:
        s->dma_ch0_status &= ~v;
        s32k3_emac_update_irq(s);
        break;
    case EMAC_DMA_CH0_TXDESC_LIST_ADDR:
        s->dma_ch0_txdesc_list = v;
        break;
    case EMAC_DMA_CH0_RXDESC_LIST_ADDR:
        s->dma_ch0_rxdesc_list = v;
        break;
    case EMAC_TX_DOORBELL:
        s->tx_fifo_len = v & 0x7ff;
        s32k3_emac_tx_frame(s);
        break;
    case EMAC_RX_FRAME_LEN:
        /* consume rx frame */
        s->rx_fifo_len = 0;
        s->dma_ch0_status &= ~DMA_CH0_INT_RI;
        s32k3_emac_update_irq(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_emac: write of unimplemented reg 0x%03" HWADDR_PRIx
                      " = 0x%08" PRIx64 "\n", addr, value);
    }
}

static const MemoryRegionOps s32k3_emac_ops = {
    .read = s32k3_emac_read,
    .write = s32k3_emac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static NetClientInfo net_s32k3_emac_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = s32k3_emac_can_receive,
    .receive = s32k3_emac_receive,
};

static void s32k3_emac_init(Object *obj)
{
    S32K3EmacState *s = S32K3_EMAC(obj);

    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);

    memory_region_init_io(&s->iomem, obj, &s32k3_emac_ops, s,
                          TYPE_S32K3_EMAC, S32K3_EMAC_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

static void s32k3_emac_realize(DeviceState *dev, Error **errp)
{
    S32K3EmacState *s = S32K3_EMAC(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_emac: module_clk must be connected");
        return;
    }

    s->nic = qemu_new_nic(&net_s32k3_emac_info, &s->conf,
                          object_get_typename(OBJECT(dev)),
                          dev->id ? dev->id : TYPE_S32K3_EMAC,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);

    s32k3_emac_reset(dev);
}

static const Property s32k3_emac_properties[] = {
    DEFINE_NIC_PROPERTIES(S32K3EmacState, conf),
};

static void s32k3_emac_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_emac_reset);
    dc->realize = s32k3_emac_realize;
    device_class_set_props(dc, s32k3_emac_properties);
    dc->desc = "NXP S32K3xx EMAC (simplified DWC GMAC)";
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo s32k3_emac_types[] = {
    {
        .name          = TYPE_S32K3_EMAC,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3EmacState),
        .instance_init = s32k3_emac_init,
        .class_init    = s32k3_emac_class_init,
    },
};

DEFINE_TYPES(s32k3_emac_types)
