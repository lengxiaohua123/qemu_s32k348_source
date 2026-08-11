/*
 * NXP S32K3xx FlexCAN QEMU device model
 *
 * Functional CAN 2.0B / CAN FD controller wired to QEMU's CAN bus layer
 * (hw/net/can).  On the host side this connects to:
 *
 *   Linux:   -object can-bus,id=canbus0 \
 *            -object can-host-socketcan,if=can0,canbus=canbus0
 *   Windows: TCP/UDP bridge to an external CAN tool (see README), or a
 *            USB-CAN adapter forwarded via socat/ser2net.
 *
 * The model implements the register block plus 96 message buffers with
 * classic and FD payloads; MBs configured as TX send frames onto the
 * CAN bus, received bus frames are stored into the first matching
 * RX MB / ORed into an RX interrupt flag.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/net/s32k3_flexcan.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-clock.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "net/can_emu.h"

/* CAN DLC -> length table (classical + FD) */
static const uint8_t dlc2len[16] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
};

static void s32k3_flexcan_update_irq(S32K3FlexcanState *s)
{
    int i;
    bool level = false;

    for (i = 0; i < 3; i++) {
        if (s->iflag[i] & s->imask[i]) {
            level = true;
        }
    }
    if ((s->esr1 & (ESR1_BOFFINT | ESR1_ERRINT)) &&
        (s->ctrl1 & (CTRL1_BOFFMSK | CTRL1_ERRMSK))) {
        level = true;
    }
    qemu_set_irq(s->irq, level);
    qemu_set_irq(s->irq_mbor, level);
}

static void s32k3_flexcan_timer_tick(void *opaque)
{
    S32K3FlexcanState *s = opaque;

    s->timer_count++;
    s->timer = s->timer_count & 0xffff;
    /* 周期性继续。NB: ptimer_tick() 已持有事务，不要再包
     * ptimer_transaction_begin/commit（会断言）。 */
    ptimer_set_count(s->timer_ptimer, 1);
    ptimer_run(s->timer_ptimer, 1);
}

static void s32k3_flexcan_timer_config(S32K3FlexcanState *s)
{
    uint64_t hz = clock_get_hz(s->module_clk);

    /* 位时序：波特率 = clk / ((PRESDIV+1) × (1 + PROPSEG+PSEG1+1 + PSEG2+1))
     * CAN_TIMER 每 bit 一拍（自由运行定时器按位时钟走）。 */
    if (hz) {
        uint32_t presdiv = (s->ctrl1 >> CTRL1_PRESDIV_SHIFT) & 0xFF;
        uint32_t pseg1 = (s->ctrl1 >> CTRL1_PSEG1_SHIFT) & 0x7;
        uint32_t pseg2 = (s->ctrl1 >> CTRL1_PSEG2_SHIFT) & 0x7;
        uint32_t tq_total = (presdiv + 1) * (1 + (pseg1 + 1) + (pseg2 + 1));
        uint64_t bitrate = tq_total ? hz / tq_total : hz;
        ptimer_transaction_begin(s->timer_ptimer);
        ptimer_set_freq(s->timer_ptimer, bitrate ? bitrate : 1);
        if (!(s->mcr & MCR_MDIS) && !(s->mcr & MCR_HALT)) {
            ptimer_set_count(s->timer_ptimer, 1);
            ptimer_run(s->timer_ptimer, 1);
        } else {
            ptimer_stop(s->timer_ptimer);
        }
        ptimer_transaction_commit(s->timer_ptimer);
        return;
    }

    ptimer_transaction_begin(s->timer_ptimer);
    ptimer_set_freq(s->timer_ptimer, 1);
    if (!(s->mcr & MCR_MDIS) && !(s->mcr & MCR_HALT)) {
        ptimer_set_count(s->timer_ptimer, 1);
        ptimer_run(s->timer_ptimer, 1);
    } else {
        ptimer_stop(s->timer_ptimer);
    }
    ptimer_transaction_commit(s->timer_ptimer);
}

static bool s32k3_flexcan_ready(S32K3FlexcanState *s)
{
    /* Not in freeze/disable mode */
    return !(s->mcr & MCR_MDIS) && !(s->mcr & MCR_HALT) &&
           !(s->mcr & MCR_NOTRDY);
}

static bool s32k3_flexcan_can_receive(CanBusClientState *client)
{
    return 1;
}

static ssize_t s32k3_flexcan_receive(CanBusClientState *client,
                                     const struct qemu_can_frame *frames,
                                     size_t frames_cnt);

/* 外部注入：构造 CAN 帧走接收路径（测试用，等价于总线收到帧） */
void s32k3_flexcan_inject(S32K3FlexcanState *s, uint32_t id,
                          const uint8_t *data, int dlc)
{
    struct qemu_can_frame frame = { 0 };

    frame.can_id = id;
    frame.can_dlc = dlc;
    if (dlc > 8) {
        dlc = 8;
    }
    memcpy(frame.data, data, dlc);
    s32k3_flexcan_receive(&s->bus_client, &frame, 1);
}

static ssize_t s32k3_flexcan_receive(CanBusClientState *client,
                                     const struct qemu_can_frame *frames,
                                     size_t frames_cnt)
{
    S32K3FlexcanState *s = container_of(client, S32K3FlexcanState, bus_client);
    const struct qemu_can_frame *f = frames;
    int mb;

    if (!s32k3_flexcan_ready(s)) {
        return frames_cnt;
    }

    /* Legacy RX FIFO（MCR[RFEN]=1）：帧写入 FIFO 区（MB0-7），
     * ID 用 RXFGMASK 过滤（掩码位为 0 表示不关心，1 必须匹配）。 */
    if (s->mcr & MCR_RFEN) {
        uint32_t *fcs = &s->mb_ram[0];
        uint32_t code = (*fcs & MB_CS_CODE_MASK) >> MB_CS_CODE_SHIFT;
        uint32_t mask = s->rxfgmask;

        if (code == MB_CODE_RX_EMPTY) {
            bool id_match = true;
            if (mask) {
                if (f->can_id & QEMU_CAN_EFF_FLAG) {
                    id_match = ((s->mb_ram[1] & mask & 0x1fffffff) ==
                                (f->can_id & mask & 0x1fffffff));
                } else {
                    id_match = ((s->mb_ram[1] & mask & 0x1fffffff) ==
                                ((f->can_id & 0x7ff) << 18 & mask));
                }
            }
            if (id_match) {
                uint32_t dlc = f->can_dlc & 0xf;
                *fcs &= ~(MB_CS_DLC_MASK | MB_CS_EDL | MB_CS_BRS);
                *fcs |= (dlc << MB_CS_DLC_SHIFT);
                if (f->can_id & QEMU_CAN_FRMF_TYPE_FD) {
                    *fcs |= MB_CS_EDL;
                    if (f->can_id & QEMU_CAN_FRMF_BRS) {
                        *fcs |= MB_CS_BRS;
                    }
                }
                s->mb_ram[2] = (f->data[0] << 24) | (f->data[1] << 16) |
                               (f->data[2] << 8) | f->data[3];
                s->mb_ram[3] = (f->data[4] << 24) | (f->data[5] << 16) |
                               (f->data[6] << 8) | f->data[7];
                *fcs &= ~MB_CS_CODE_MASK;
                *fcs |= MB_CODE_RX_FULL << MB_CS_CODE_SHIFT;
                /* Enhanced Rx FIFO：同步存 0x2000 FIFO RAM + 置
                 * ERFSR.FRAME_AVAILABLE(bit28)——固件 Enhanced FIFO 从
                 * 0x2000 读帧，原模型只存 legacy MB0 导致超时。 */
                s->erfdsr[0] = *fcs;
                s->erfdsr[1] = s->mb_ram[1];
                s->erfdsr[2] = s->mb_ram[2];
                s->erfdsr[3] = s->mb_ram[3];
                s->erfsr |= (1u << 28);   /* FRAME_AVAILABLE */
                s->iflag[0] |= 1;   /* MB0 flag */
                s->esr1 |= ESR1_RX | ESR1_INT1;
                s->esr1 &= ~(ESR1_IDLE | ESR1_TX);
                s->ack_ok = true;
                s32k3_flexcan_update_irq(s);
                return frames_cnt;
            }
        }
    }

    /* Store into first RX_EMPTY message buffer */
    for (mb = 0; mb < CAN_MAX_MB; mb++) {
        uint32_t *cs = &s->mb_ram[mb * 4];
        uint32_t code = (*cs & MB_CS_CODE_MASK) >> MB_CS_CODE_SHIFT;
        uint32_t ide = *cs & MB_CS_IDE;
        uint32_t mb_id = s->mb_ram[mb * 4 + 1];
        bool id_match;

        if (code != MB_CODE_RX_EMPTY) {
            continue;
        }

        if (ide) {
            id_match = ((mb_id & 0x1fffffff) == (f->can_id & 0x1fffffff));
        } else {
            id_match = ((mb_id & 0x1fffffff) == (f->can_id & 0x7ff) << 18);
        }
        /* RXIMR 个体掩码过滤（MCR[IRMQ]=1 时）：
         * 掩码位为 1 表示"必须匹配"，为 0 表示"不关心"。
         * 简化模型：掩码为 0 接受全部，否则按掩码比较。 */
        if ((s->mcr & MCR_IRMQ) && mb < CAN_MAX_MB) {
            uint32_t mask = s->rximr[mb];
            if (mask) {
                if (ide) {
                    id_match = ((mb_id & mask & 0x1fffffff) ==
                                (f->can_id & mask & 0x1fffffff));
                } else {
                    id_match = ((mb_id & mask & 0x1fffffff) ==
                                ((f->can_id & 0x7ff) << 18 & mask));
                }
            }
        }
        if (!id_match && mb_id != 0) {
            continue;   /* simplified acceptance: exact match or wildcard 0 */
        }

        /* fill MB */
        *cs &= ~(MB_CS_DLC_MASK | MB_CS_EDL | MB_CS_BRS);
        *cs |= ((f->can_dlc & 0xf) << MB_CS_DLC_SHIFT);
        if (f->can_id & QEMU_CAN_FRMF_TYPE_FD) {
            *cs |= MB_CS_EDL;
            if (f->can_id & QEMU_CAN_FRMF_BRS) {
                *cs |= MB_CS_BRS;
            }
            memcpy(s->mb_fd_data[mb], f->data, dlc2len[f->can_dlc]);
        } else {
            memcpy(s->mb_fd_data[mb], f->data, dlc2len[f->can_dlc]);
            s->mb_ram[mb * 4 + 2] = (f->data[0] << 24) | (f->data[1] << 16) |
                                    (f->data[2] << 8) | f->data[3];
            s->mb_ram[mb * 4 + 3] = (f->data[4] << 24) | (f->data[5] << 16) |
                                    (f->data[6] << 8) | f->data[7];
        }
        *cs &= ~MB_CS_CODE_MASK;
        *cs |= MB_CODE_RX_FULL << MB_CS_CODE_SHIFT;
        *cs |= (s->timer & 0xffff);

        s->iflag[mb >> 5] |= 1 << (mb & 31);
        s->esr1 |= ESR1_RX | ESR1_INT1;
        s->esr1 &= ~(ESR1_IDLE | ESR1_TX);
        s->ack_ok = true;   /* 帧被接收 = 总线上有节点应答 */
        s32k3_flexcan_update_irq(s);
        break;
    }

    return frames_cnt;
}

static CanBusClientInfo s32k3_flexcan_bus_client_info = {
    .can_receive = s32k3_flexcan_can_receive,
    .receive = s32k3_flexcan_receive,
};

static void s32k3_flexcan_tx_mb(S32K3FlexcanState *s, int mb)
{
    uint32_t *cs = &s->mb_ram[mb * 4];
    struct qemu_can_frame frame;
    uint32_t code = (*cs & MB_CS_CODE_MASK) >> MB_CS_CODE_SHIFT;
    uint32_t dlc = (*cs & MB_CS_DLC_MASK) >> MB_CS_DLC_SHIFT;

    if (code != MB_CODE_TX_ONCE) {
        return;
    }
    if (!s->canbus) {
        *cs = (*cs & ~MB_CS_CODE_MASK) |
              (MB_CODE_TX_INACTIVE << MB_CS_CODE_SHIFT);
        return;
    }

    memset(&frame, 0, sizeof(frame));
    if (*cs & MB_CS_IDE) {
        frame.can_id = s->mb_ram[mb * 4 + 1] & 0x1fffffff;
        frame.can_id |= QEMU_CAN_EFF_FLAG;
    } else {
        frame.can_id = (s->mb_ram[mb * 4 + 1] >> 18) & 0x7ff;
    }
    if (*cs & MB_CS_RTR) {
        frame.can_id |= QEMU_CAN_RTR_FLAG;
    }
    if (*cs & MB_CS_EDL) {
        frame.can_id |= QEMU_CAN_FRMF_TYPE_FD;
        if (*cs & MB_CS_BRS) {
            frame.can_id |= QEMU_CAN_FRMF_BRS;
        }
    }
    frame.can_dlc = dlc & 0xf;
    memcpy(frame.data, s->mb_fd_data[mb], dlc2len[frame.can_dlc]);

    if (can_bus_client_send(&s->bus_client, &frame, 1) <= 0) {
        /*
         * can_bus_client_send() skips the sending client itself
         * ("No loopback support for now"), so on the board's private
         * loopback bus (this device is the only client) the frame would
         * be silently dropped.  Deliver it locally to make the
         * standalone loopback configuration actually loop back.
         */
        s32k3_flexcan_receive(&s->bus_client, &frame, 1);
    }

    /* transmission complete -> MB becomes INACTIVE */
    *cs = (*cs & ~MB_CS_CODE_MASK) |
          (MB_CODE_TX_INACTIVE << MB_CS_CODE_SHIFT);
    *cs |= (s->timer & 0xffff);

    s->iflag[mb >> 5] |= 1 << (mb & 31);
    s->esr1 |= ESR1_TX | ESR1_INT1;
    s->esr1 &= ~ESR1_IDLE;

    /* 协议引擎：错误计数与 fault confinement 状态机。
     * 总线有其他节点应答 -> 发送成功；无应答（单节点）-> ACK 错误。 */
    if (s->ack_ok) {
        /* 成功发送：TXERRCNT 减 1（不低于 0） */
        uint32_t txerr = (s->ecr >> 8) & 0xFF;
        if (txerr > 0) {
            txerr--;
            s->ecr = (s->ecr & 0xFF) | (txerr << 8);
        }
        s->ack_ok = false;
    } else {
        /* ACK 错误：TXERRCNT 增 8 */
        uint32_t txerr = ((s->ecr >> 8) & 0xFF) + 8;
        s->esr1 |= ESR1_ACKERR | ESR1_ERRINT;
        if (txerr > 255) {
            /* Bus Off */
            s->esr1 &= ~ESR1_FLTCONF;
            s->esr1 |= (2u << 4);   /* FLTCONF=10 bus off */
            s->esr1 |= ESR1_BOFFINT;
            txerr = 0;
        } else if (txerr >= 128) {
            /* Error Passive */
            s->esr1 &= ~ESR1_FLTCONF;
            s->esr1 |= (1u << 4);   /* FLTCONF=01 error passive */
        } else {
            s->esr1 &= ~ESR1_FLTCONF;   /* error active */
        }
        s->ecr = (s->ecr & 0xFF) | ((txerr & 0xFF) << 8);
    }
    s32k3_flexcan_update_irq(s);
}

static void s32k3_flexcan_reset(DeviceState *dev)
{
    S32K3FlexcanState *s = S32K3_FLEXCAN(dev);
    int i;

    s->mcr = MCR_MDIS | MCR_FRZ | MCR_HALT | MCR_NOTRDY | MCR_FRZACK |
             MCR_SUPV | MCR_IRMQ | (1 << 19) | 0xF;   /* 手册复位 D890_000Fh */
    s->ctrl1 = 0;
    s->timer = 0;
    s->timer_count = 0;
    s->ack_ok = false;
    memset(s->rximr, 0, sizeof(s->rximr));
    s->rxmgmask = 0;
    s->rx14mask = 0;
    s->rx15mask = 0;
    s->ecr = 0;
    s->esr1 = 0x0003B006;   /* RTD 默认值 FLEXCAN_IP_ESR1_DEFAULT_VALUE_U32 */
    for (i = 0; i < 3; i++) {
        s->imask[i] = 0;
        s->iflag[i] = 0xFFFFFFFF;   /* RTD 默认值 FLEXCAN_IP_IFLAG_DEFAULT_VALUE_U32 */
    }
    s->ctrl2 = 0x00100000;  /* RTD 默认值 FLEXCAN_IP_CTRL2_DEFAULT_VALUE_U32 */
    s->esr2 = 0;
    s->crcr = 0;
    s->rxfgmask = 0;
    s->cbt = 0;
    s->fdctrl = 0x80004100;  /* RTD 默认值 FLEXCAN_IP_FDCTRL_DEFAULT_VALUE_U32 */
    s->fdcbt = 0;
    s->fdcrc = 0;
    s->erfcr = 0;
    s->erfier = 0;
    s->erfsr = 0xF8000000;  /* RTD 默认值 FLEXCAN_IP_ERFSR_DEFAULT_VALUE_U32 */
    s->mecr = 0x000C0080;   /* RTD 默认值 FLEXCAN_IP_MECR_DEFAULT_VALUE_U32 */
    s->encbt = 0;
    s->edcbt = 0;
    memset(s->mb_ram, 0, sizeof(s->mb_ram));
    memset(s->mb_fd_data, 0, sizeof(s->mb_fd_data));
    for (i = 0; i < CAN_MAX_MB; i++) {
        /* all MBs inactive */
        s->mb_ram[i * 4] = MB_CODE_RX_INACTIVE << MB_CS_CODE_SHIFT;
    }
    s32k3_flexcan_update_irq(s);
}

static uint64_t s32k3_flexcan_read(void *opaque, hwaddr addr, unsigned size)
{
    S32K3FlexcanState *s = opaque;
    uint32_t r = 0;

    if (addr >= CAN_MB_BASE && addr < CAN_MB_BASE + sizeof(s->mb_ram)) {
        r = s->mb_ram[(addr - CAN_MB_BASE) / 4];
        return r;
    }
    if (addr >= 0x880 && addr < 0x880 + CAN_MAX_MB * 4) {
        /* RX Individual Mask Registers (RXIMR0-95) */
        r = s->rximr[(addr - 0x880) / 4];
        return r;
    }

    switch (addr) {
    case CAN_MCR:
        r = s->mcr;
        break;
    case CAN_CTRL1:
        r = s->ctrl1;
        break;
    case CAN_TIMER:
        r = s->timer;
        break;
    case CAN_RXMGMASK:
        r = s->rxmgmask;
        break;
    case CAN_RX14MASK:
        r = s->rx14mask;
        break;
    case CAN_RX15MASK:
        r = s->rx15mask;
        break;
    case CAN_ECR:
        r = s->ecr;
        break;
    case CAN_ESR1:
        r = s->esr1;
        break;
    case CAN_IMASK1:
        r = s->imask[0];
        break;
    case CAN_IMASK2:
        r = s->imask[1];
        break;
    case CAN_IMASK3:
        r = s->imask[2];
        break;
    case CAN_IFLAG1:
        r = s->iflag[0];
        break;
    case CAN_IFLAG2:
        r = s->iflag[1];
        break;
    case CAN_IFLAG3:
        r = s->iflag[2];
        break;
    case CAN_CTRL2:
        r = s->ctrl2;
        break;
    case CAN_ESR2:
        r = s->esr2;
        break;
    case CAN_CRCR:
        r = s->crcr;
        break;
    case CAN_RXFGMASK:
        r = s->rxfgmask;
        break;
    case CAN_CBT:
        r = s->cbt;
        break;
    case CAN_FDCTRL:
        r = s->fdctrl;
        break;
    case CAN_FDCBT:
        r = s->fdcbt;
        break;
    case CAN_FDCRC:
        r = s->fdcrc;
        break;
    case CAN_ERFCR:
        r = s->erfcr;
        break;
    case CAN_ERFIER:
        r = s->erfier;
        break;
    case CAN_ERFSR:
        r = s->erfsr;
        break;
    case CAN_MECR:
        r = s->mecr;
        break;
    case CAN_ENCBT:
        r = s->encbt;
        break;
    case CAN_EDCBT:
        r = s->edcbt;
        break;
    default:
        if (addr >= 0x2000 && addr < 0x2000 + sizeof(s->erfdsr)) {
            /* Enhanced Rx FIFO RAM：固件读帧数据 */
            r = s->erfdsr[(addr - 0x2000) / 4];
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "s32k3_flexcan: read of unimplemented reg 0x%03" HWADDR_PRIx "\n",
                          addr);
        }
    }
    return r;
}

static void s32k3_flexcan_write(void *opaque, hwaddr addr,
                                uint64_t value, unsigned size)
{
    S32K3FlexcanState *s = opaque;
    uint32_t v = value;

    if (addr >= CAN_MB_BASE && addr < CAN_MB_BASE + sizeof(s->mb_ram)) {
        int word = (addr - CAN_MB_BASE) / 4;
        int mb = word / 4;

        s->mb_ram[word] = v;
        if ((word & 3) == 0) {
            /* control/status word written: maybe trigger TX */
            s32k3_flexcan_tx_mb(s, mb);
        } else if ((word & 3) >= 2) {
            /* classic payload also mirrors into FD data array */
            s->mb_fd_data[mb][(word & 1) * 4 + 0] = (v >> 24) & 0xff;
            s->mb_fd_data[mb][(word & 1) * 4 + 1] = (v >> 16) & 0xff;
            s->mb_fd_data[mb][(word & 1) * 4 + 2] = (v >> 8) & 0xff;
            s->mb_fd_data[mb][(word & 1) * 4 + 3] = v & 0xff;
        }
        return;
    }
    if (addr >= 0x880 && addr < 0x880 + CAN_MAX_MB * 4) {
        /* RX Individual Mask Registers (RXIMR0-95) */
        s->rximr[(addr - 0x880) / 4] = v;
        return;
    }

    switch (addr) {
    case CAN_MCR:
        if (v & MCR_SOFTRST) {
            s32k3_flexcan_reset(DEVICE(s));
            return;
        }
        s->mcr = v;
        /* model mode transitions as instantaneous */
        if (!(v & MCR_HALT)) {
            s->mcr &= ~(MCR_NOTRDY | MCR_FRZACK | MCR_LPMACK);
            s->esr1 |= ESR1_IDLE;
        } else {
            s->mcr |= MCR_NOTRDY | MCR_FRZACK | MCR_LPMACK;
        }
        if (v & MCR_FDEN) {
            s->mcr |= MCR_FDEN;
        }
        s32k3_flexcan_timer_config(s);
        break;
    case CAN_CTRL1:
        s->ctrl1 = v;
        s32k3_flexcan_update_irq(s);
        break;
    case CAN_TIMER:
        s->timer = v & 0xffff;
        s->timer_count = s->timer;
        break;
    case CAN_RXMGMASK:
        s->rxmgmask = v;
        break;
    case CAN_RX14MASK:
        s->rx14mask = v;
        break;
    case CAN_RX15MASK:
        s->rx15mask = v;
        break;
    case CAN_ECR:
        s->ecr = v & 0xffff;
        break;
    case CAN_ESR1:
        s->esr1 &= ~(v & 0x0003fff6); /* W1C, keep SYNCH/IDLE/TX/RX state */
        s32k3_flexcan_update_irq(s);
        break;
    case CAN_IMASK1:
        s->imask[0] = v;
        s32k3_flexcan_update_irq(s);
        break;
    case CAN_IMASK2:
        s->imask[1] = v;
        s32k3_flexcan_update_irq(s);
        break;
    case CAN_IMASK3:
        s->imask[2] = v;
        s32k3_flexcan_update_irq(s);
        break;
    case CAN_IFLAG1:
        s->iflag[0] &= ~v;
        s32k3_flexcan_update_irq(s);
        break;
    case CAN_IFLAG2:
        s->iflag[1] &= ~v;
        s32k3_flexcan_update_irq(s);
        break;
    case CAN_IFLAG3:
        s->iflag[2] &= ~v;
        s32k3_flexcan_update_irq(s);
        break;
    case CAN_CTRL2:
        s->ctrl2 = v;
        break;
    case CAN_ESR2:
        s->esr2 = v;
        break;
    case CAN_CRCR:
        s->crcr = v;
        break;
    case CAN_RXFGMASK:
        s->rxfgmask = v;
        break;
    case CAN_CBT:
        s->cbt = v;
        break;
    case CAN_FDCTRL:
        s->fdctrl = v & (FDCTRL_FDRATE | FDCTRL_MBDSR0 | FDCTRL_TDCEN |
                         FDCTRL_TDCOFF);
        break;
    case CAN_FDCBT:
        s->fdcbt = v;
        break;
    case CAN_ERFCR:
        s->erfcr = v;
        break;
    case CAN_ERFIER:
        s->erfier = v;
        s32k3_flexcan_update_irq(s);
        break;
    case CAN_ERFSR:
        s->erfsr &= ~v;
        break;
    case CAN_FDCRC:
        break; /* read-only */
    case CAN_MECR:
        /* MECR: ECRWRDIS=1 复位默认；写需先清该位（简化存储） */
        s->mecr = v & ~(1u << 31);
        break;
    case CAN_ENCBT:
        s->encbt = v;
        break;
    case CAN_EDCBT:
        s->edcbt = v;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s32k3_flexcan: write of unimplemented reg 0x%03" HWADDR_PRIx
                      " = 0x%08" PRIx64 "\n", addr, value);
    }
}

static const MemoryRegionOps s32k3_flexcan_ops = {
    .read = s32k3_flexcan_read,
    .write = s32k3_flexcan_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    /*
     * 真实 S32K3 FlexCAN 允许 8/16/32 位访问（RTD 驱动用 8 位写
     * MAXMB、16 位写 MB 控制字等）。valid.min_access_size=4 会拒绝
     * 小 size 访问（memory_region_access_valid -> Data Abort）。
     */
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void s32k3_flexcan_init(Object *obj)
{
    S32K3FlexcanState *s = S32K3_FLEXCAN(obj);

    s->module_clk = qdev_init_clock_in(DEVICE(s), "module_clk", NULL, NULL, 0);

    memory_region_init_io(&s->iomem, obj, &s32k3_flexcan_ops, s,
                          TYPE_S32K3_FLEXCAN, S32K3_FLEXCAN_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq_mbor);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq_fd);

    s->timer_ptimer = ptimer_init(s32k3_flexcan_timer_tick, s,
                                  PTIMER_POLICY_LEGACY);
}

static void s32k3_flexcan_realize(DeviceState *dev, Error **errp)
{
    S32K3FlexcanState *s = S32K3_FLEXCAN(dev);

    if (!s->module_clk) {
        error_setg(errp, "s32k3_flexcan: module_clk must be connected");
        return;
    }

    if (s->canbus) {
        s->bus_client.info = &s32k3_flexcan_bus_client_info;
        if (can_bus_insert_client(s->canbus, &s->bus_client) < 0) {
            error_setg(errp, "s32k3_flexcan: cannot attach to CAN bus");
            return;
        }
    }

    s32k3_flexcan_reset(dev);
}

static const Property s32k3_flexcan_properties[] = {
    DEFINE_PROP_LINK("canbus", S32K3FlexcanState, canbus,
                     TYPE_CAN_BUS, CanBusState *),
};

static void s32k3_flexcan_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_flexcan_reset);
    dc->realize = s32k3_flexcan_realize;
    device_class_set_props(dc, s32k3_flexcan_properties);
    dc->desc = "NXP S32K3xx FlexCAN (CAN 2.0B / CAN FD)";
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo s32k3_flexcan_types[] = {
    {
        .name          = TYPE_S32K3_FLEXCAN,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3FlexcanState),
        .instance_init = s32k3_flexcan_init,
        .class_init    = s32k3_flexcan_class_init,
    },
};

DEFINE_TYPES(s32k3_flexcan_types)
