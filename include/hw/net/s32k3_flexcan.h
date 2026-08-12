/*
 * NXP S32K3xx FlexCAN (CAN 2.0B / CAN FD) QEMU device model - header
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_CAN_S32K3_FLEXCAN_H
#define HW_CAN_S32K3_FLEXCAN_H

#include "hw/core/sysbus.h"
#include "hw/core/ptimer.h"
#include "net/can_emu.h"
#include "hw/core/clock.h"
#include "qom/object.h"

#define TYPE_S32K3_FLEXCAN "s32k3-flexcan"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3FlexcanState, S32K3_FLEXCAN)

#define S32K3_FLEXCAN_MMIO_SIZE 0x4000

/* Register offsets (FlexCAN chapter, S32K3xx RM) */
#define CAN_MCR         0x000
#define CAN_CTRL1       0x004
#define CAN_TIMER       0x008
#define CAN_RXMGMASK    0x010
#define CAN_RX14MASK    0x014
#define CAN_RX15MASK    0x018
#define CAN_ECR         0x01C
#define CAN_ESR1        0x020
#define CAN_IMASK2      0x024   /* S32K3: IMASK2/IMASK3 exist */
#define CAN_IMASK1      0x028
#define CAN_IFLAG2      0x02C
#define CAN_IFLAG1      0x030
#define CAN_CTRL2       0x034
#define CAN_ESR2        0x038
#define CAN_CRCR        0x044
#define CAN_RXFGMASK    0x048
#define CAN_RXFIR       0x04C
#define CAN_CBT         0x050
#define CAN_IMASK3      0x06C
#define CAN_IFLAG3      0x074
#define CAN_CTRL1_PN    0x0B00
#define CAN_FDCTRL      0x0C00
#define CAN_FDCBT       0x0C04
#define CAN_FDCRC       0x0C08
#define CAN_ERFCR       0x0C0C
#define CAN_ERFIER      0x0C10
#define CAN_ERFSR       0x0C14
#define CAN_RXIMR       0x880   /* Receive Individual Mask[96]（0x880-0xB7C） */
#define CAN_MECR        0xAE0
#define CAN_ERRIAR      0xAE4   /* error injection: IAR/IDPR/IPPR */
#define CAN_ERRIDPR     0xAE8
#define CAN_ERRIPPR     0xAEC
#define CAN_RERRAR      0xAF0   /* RAM error report: RAR/RDR/RSYNR/SR */
#define CAN_RERRDR      0xAF4
#define CAN_RERRSYNR    0xAF8
#define CAN_ERRSR       0xAFC
#define CAN_EPRS        0xBF0   /* S32K348.h: Enhanced CAN Bit Timing Prescalers */
#define CAN_ENCBT       0xBF4   /* Enhanced Nominal CAN Bit Timing */
#define CAN_EDCBT       0xBF8   /* Enhanced Data Phase CAN Bit Timing */
#define CAN_ETDC        0xBFC   /* Enhanced Transceiver Delay Compensation */

/* Message buffer region starts at 0x080 on S32K3 (legacy+FD MBs) */
#define CAN_MB_BASE     0x080
#define CAN_MB_SIZE     0x10    /* classic 8-byte payload MB = 16 bytes */
#define CAN_MAX_MB      96

/* MCR bits */
#define MCR_MDIS        (1 << 31)
#define MCR_FRZ         (1 << 30)
#define MCR_RFEN        (1 << 29)
#define MCR_HALT        (1 << 28)
#define MCR_NOTRDY      (1 << 27)
#define MCR_SOFTRST     (1 << 25)
#define MCR_FRZACK      (1 << 24)
#define MCR_SUPV        (1 << 23)
#define MCR_WRNEN       (1 << 21)
#define MCR_LPMACK      (1 << 20)
#define MCR_SRXDIS      (1 << 17)
#define MCR_IRMQ        (1 << 16)
#define MCR_LPRIOEN     (1 << 13)
#define MCR_AEN         (1 << 12)
#define MCR_FDEN        (1 << 11)
#define MCR_IDAM        (3 << 8)
#define MCR_MAXMB_MASK  0x7f

/* CTRL1 bits */
#define CTRL1_PRESDIV_SHIFT 24
#define CTRL1_RJW_SHIFT  22
#define CTRL1_PSEG1_SHIFT 19
#define CTRL1_PSEG2_SHIFT 16
#define CTRL1_BOFFMSK    (1 << 15)
#define CTRL1_ERRMSK     (1 << 14)
#define CTRL1_TWRNMSK    (1 << 11)
#define CTRL1_RWRNMSK    (1 << 10)
#define CTRL1_LPB        (1 << 12)
#define CTRL1_SMP        (1 << 7)
#define CTRL1_PROPSEG_MASK 0x7

/* ESR1 bits */
#define ESR1_SYNCH       (1 << 18)
#define ESR1_TWRNINT     (1 << 17)
#define ESR1_RWRNINT     (1 << 16)
#define ESR1_BIT1ERR     (1 << 15)
#define ESR1_BIT0ERR     (1 << 14)
#define ESR1_ACKERR      (1 << 13)
#define ESR1_CRCERR      (1 << 12)
#define ESR1_FRMERR      (1 << 11)
#define ESR1_STFERR      (1 << 10)
#define ESR1_TXWRN       (1 << 9)
#define ESR1_RXWRN       (1 << 8)
#define ESR1_IDLE        (1 << 7)
#define ESR1_TX          (1 << 6)
#define ESR1_FLTCONF     (3 << 4)
#define ESR1_RX          (1 << 3)
#define ESR1_BOFFINT     (1 << 2)
#define ESR1_ERRINT      (1 << 1)
#define ESR1_INT1        (1 << 0)

/* FDCTRL bits */
#define FDCTRL_FDRATE    (1 << 31)
#define FDCTRL_MBDSR0    (3 << 16)
#define FDCTRL_TDCEN     (1 << 15)
#define FDCTRL_TDCOFF    (0x1f << 8)

/* MB control/status word (first word of MB) */
#define MB_CS_CODE_SHIFT 24
#define MB_CS_CODE_MASK  (0xf << MB_CS_CODE_SHIFT)
#define MB_CS_SRR        (1 << 22)
#define MB_CS_IDE        (1 << 21)
#define MB_CS_RTR        (1 << 20)
#define MB_CS_DLC_MASK   (0xf << 16)
#define MB_CS_DLC_SHIFT  16
#define MB_CS_EDL        (1 << 15)  /* CAN FD frame */
#define MB_CS_BRS        (1 << 14)  /* bit rate switch */
#define MB_CS_ESI        (1 << 13)

/* MB code values */
#define MB_CODE_RX_INACTIVE  0x0
#define MB_CODE_RX_EMPTY     0x4
#define MB_CODE_RX_FULL      0x2
#define MB_CODE_RX_OVERRUN   0x6
#define MB_CODE_TX_INACTIVE  0x8
#define MB_CODE_TX_ABORT     0x9
#define MB_CODE_TX_ONCE      0xC
#define MB_CODE_TX_TANSWER   0xE

void s32k3_flexcan_inject(S32K3FlexcanState *s, uint32_t id,
                          const uint8_t *data, int dlc);

struct S32K3FlexcanState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock        *module_clk;
    qemu_irq     irq;
    qemu_irq     irq_mbor;      /* message buffer ORed irq */
    qemu_irq     irq_fd;

    CanBusState *canbus;
    CanBusClientState bus_client;

    uint32_t mcr;
    uint32_t ctrl1;
    uint32_t timer;
    uint32_t rxmgmask;
    uint32_t rx14mask;
    uint32_t rx15mask;
    uint32_t ecr;
    uint32_t esr1;
    uint32_t imask[3];
    uint32_t iflag[3];
    uint32_t ctrl2;
    uint32_t esr2;
    uint32_t crcr;
    uint32_t rxfgmask;
    uint32_t cbt;
    uint32_t fdctrl;
    uint32_t fdcbt;
    uint32_t fdcrc;
    uint32_t erfcr;
    uint32_t erfier;
    uint32_t erfsr;
    /* Enhanced Rx FIFO RAM（0x2000 起）：最近一帧（CS/ID/data），
     * 固件 Enhanced FIFO 从 0x2000 读帧 + ERFSR.FRAME_AVAILABLE(bit28) */
    uint32_t erfdsr[8];
    uint32_t mecr;      /* memory error control (RM 73.6.2.23) */
    uint32_t encbt;     /* FD nominal bit timing (RM 73.6.2.24) */
    uint32_t edcbt;     /* FD data bit timing (RM 73.6.2.25) */
    uint32_t eprs;      /* enhanced CAN bit timing prescalers (0xBF0) */
    uint32_t etdc;      /* enhanced transceiver delay compensation (0xBFC) */
    uint32_t erriar, erridpr, errippr;   /* error injection */
    uint32_t rerrar, rerrdr, rerrsynr, errsr;  /* RAM error report */
    uint32_t hrtime[4];                  /* HR_TIME_STAMP (0xC30, S32K348 无 HR 定时器则保留) */

    /* RX individual masks (IRMQ=1 时生效, 96 个 MB, 0x880 起) */
    uint32_t rximr[CAN_MAX_MB];

    /* message buffer RAM (96 MBs x 16 bytes, raw little-endian words) */
    uint32_t mb_ram[CAN_MAX_MB * (CAN_MB_SIZE / 4)];

    /* FD MB data storage (up to 64 bytes per MB when FD enabled) */
    uint8_t  mb_fd_data[CAN_MAX_MB][64];



    /* free-running timer (CAN_TIMER) */
    ptimer_state *timer_ptimer;
    struct QEMUTimer *timer_qemu;
    uint32_t timer_count;
    bool ack_ok;    /* 上次发送是否被应答（回环收到） */
    uint64_t timer_ns;
};

#endif /* HW_CAN_S32K3_FLEXCAN_H */
