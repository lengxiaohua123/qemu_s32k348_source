/*
 * NXP S32K3xx LPUART (Low Power Universal Asynchronous Receiver/Transmitter)
 * QEMU device model - header
 *
 * Compatible with S32K344 / S32K348 / S32K358 LPUART IP (register layout
 * per S32K3xx Reference Manual).
 *
 * Copyright (c) 2026 S32K348 QEMU port contributors
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_CHAR_S32K3_LPUART_H
#define HW_CHAR_S32K3_LPUART_H

#include "hw/core/sysbus.h"
#include "hw/core/ptimer.h"
#include "chardev/char-fe.h"
#include "hw/core/clock.h"
#include "qom/object.h"

#define TYPE_S32K3_LPUART "s32k3-lpuart"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3LpuartState, S32K3_LPUART)

#define S32K3_LPUART_MMIO_SIZE 0x1000

/* Register offsets (S32K3xx RM, LPUART chapter) */
#define LPUART_VERID    0x000   /* Version ID */
#define LPUART_PARAM    0x004   /* Parameter */
#define LPUART_GLOBAL   0x008   /* Global */
#define LPUART_PINCFG   0x00C   /* Pin Configuration */
#define LPUART_BAUD     0x010   /* Baud Rate */
#define LPUART_STAT     0x014   /* Status */
#define LPUART_CTRL     0x018   /* Control */
#define LPUART_DATA     0x01C   /* Data */
#define LPUART_MATCH    0x020   /* Match Address */
#define LPUART_MODIR    0x024   /* Modem IrDA */
#define LPUART_FIFO     0x028   /* FIFO */
#define LPUART_WATER    0x02C   /* Watermark */

/* VERID fields */
#define VERID_FEATURE(maj, min) (((maj) << 16) | (min))

/* PARAM fields */
#define PARAM_TXFIFO_SHIFT 8
#define PARAM_TXFIFO_MASK  (0xff << PARAM_TXFIFO_SHIFT)
#define PARAM_RXFIFO_MASK  0xff

/* GLOBAL */
#define GLOBAL_RST       (1 << 1)

/* BAUD */
#define BAUD_OSR_SHIFT   24
#define BAUD_OSR_MASK    (0x1f << BAUD_OSR_SHIFT)
#define BAUD_SBR_MASK    0x1fff
#define BAUD_MATCFG_SHIFT 18
#define BAUD_MATCFG_MASK (3 << BAUD_MATCFG_SHIFT)
#define BAUD_MAEN1       (1 << 31)   /* S32K348.h LPUART_BAUD_MAEN1_SHIFT=31 */
#define BAUD_MAEN2       (1 << 30)   /* S32K348.h LPUART_BAUD_MAEN2_SHIFT=30 */

/* MODIR bits */
#define MODIR_TXDIR      (1 << 2)   /* TXD pin direction (half duplex) */
#define MODIR_RTSW       (1 << 4)
#define MODIR_TXCTSE     (1 << 5)   /* TX CTS enable */
#define MODIR_RXRTSE     (1 << 6)   /* RX RTS enable */

/* CTRL bits (additions) */
#define CTRL_LOOPS       (1 << 7)
#define CTRL_RSRC        (1 << 5)

/* STAT */
#define STAT_TDRE        (1 << 23)
#define STAT_TC          (1 << 22)
#define STAT_RDRF        (1 << 21)
#define STAT_IDLE        (1 << 20)
#define STAT_OR          (1 << 19)
#define STAT_NF          (1 << 18)
#define STAT_FE          (1 << 17)
#define STAT_PF          (1 << 16)

/* CTRL */
#define CTRL_TIE         (1 << 23)
#define CTRL_TCIE        (1 << 22)
#define CTRL_RIE         (1 << 21)
#define CTRL_M           (1 << 4)
#define CTRL_SBK         (1 << 16)
#define CTRL_TE          (1 << 19)
#define CTRL_RE          (1 << 18)
#define CTRL_PE          (1 << 1)
#define CTRL_PT          (1 << 0)

/* FIFO */
#define FIFO_TXEMPT      (1 << 23)
#define FIFO_RXEMPT      (1 << 22)
#define FIFO_TXFLUSH     (1 << 15)
#define FIFO_RXFLUSH     (1 << 14)
#define FIFO_TXFE        (1 << 7)
#define FIFO_RXFE        (1 << 3)

/* FIFO depth modeled */
#define S32K3_LPUART_FIFO_DEPTH 32

struct S32K3LpuartState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    CharFrontend  chr;
    Clock        *module_clk;
    qemu_irq     irq;

    uint32_t verid;
    uint32_t param;
    uint32_t global;
    uint32_t pincfg;
    uint32_t baud;
    uint32_t stat;
    uint32_t ctrl;
    uint32_t match;
    uint32_t modir;
    uint32_t fifo;
    uint32_t water;

    /* rx software fifo */
    uint8_t  rx_fifo[S32K3_LPUART_FIFO_DEPTH];
    uint32_t rx_fifo_len;
    uint8_t  rx_pending[64];      /* chardev 输入暂存（按波特率注入 FIFO） */
    int      rx_pending_len;
    bool     rx_baud_busy;
    uint64_t rx_baud_period_ns;   /* 每字符位时间（模拟串口接收时序） */
    ptimer_state *rx_baud_timer;


    /* tx software fifo + transmit timing */
    uint8_t  tx_fifo[S32K3_LPUART_FIFO_DEPTH];
    uint32_t tx_fifo_len;
    uint32_t tx_fifo_head;
    ptimer_state *tx_timer;
    ptimer_state *rx_pump_timer;
    bool     tx_busy;

    /* true for LPUART0/1 (with TDBR / eDMA burst capability) */
    bool     lpuart_type;

    uint64_t hit_cnt;
};

#endif /* HW_CHAR_S32K3_LPUART_H */
