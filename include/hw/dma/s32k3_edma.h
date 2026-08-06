/*
 * NXP S32K3xx eDMA (enhanced DMA, 32 channels) QEMU device model - header
 *
 * Native model for S32K344/348 (no dependency on S32K358 driver).
 * Register layout per S32K3xx RM (eDMA chapter).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DMA_S32K3_EDMA_H
#define HW_DMA_S32K3_EDMA_H

#include "hw/core/sysbus.h"
#include "hw/core/ptimer.h"
#include "hw/core/clock.h"
#include "qom/object.h"

#define TYPE_S32K3_EDMA "s32k3-edma"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3EdmaState, S32K3_EDMA)

#define S32K3_EDMA_CHANNELS 32

/* main register block offsets (region 0) */
#define EDMA_CR          0x00
#define  CR_EDBG         (1 << 1)
#define  CR_ERCA         (1 << 2)
#define  CR_EMLM         (1 << 7)
#define  CR_CX           (1 << 17)
#define  CR_ECX          (1 << 16)
#define EDMA_ES          0x04
#define EDMA_ERQ         0x0C
#define EDMA_EEI         0x14
#define EDMA_CEEI        0x18
#define EDMA_SEEI        0x19
#define EDMA_CERQ        0x1A
#define EDMA_SERQ        0x1B
#define EDMA_CDNE        0x1C
#define EDMA_SSRT        0x1D
#define EDMA_CERR        0x1E
#define EDMA_CINT        0x1F
#define EDMA_INT         0x24
#define EDMA_ERR         0x2C
#define EDMA_HRS         0x34

/* per-channel control regs: 0x100 + 4*n (CHn_CSR, CHn_ES, CHn_INT, CHn_SBR
   follow the S32K3 layout at stride 0x10) */
#define EDMA_CH_BASE     0x100
#define EDMA_CH_STRIDE   0x10
#define EDMA_CH_CSR(n)   (EDMA_CH_BASE + (n) * EDMA_CH_STRIDE + 0x0)
#define EDMA_CH_ES(n)    (EDMA_CH_BASE + (n) * EDMA_CH_STRIDE + 0x4)
#define EDMA_CH_INT(n)   (EDMA_CH_BASE + (n) * EDMA_CH_STRIDE + 0x8)
#define EDMA_CH_SBR(n)   (EDMA_CH_BASE + (n) * EDMA_CH_STRIDE + 0xC)

/* TCD (transfer control descriptor), 32 bytes each, region 1/2 */
#define TCD_SADDR        0x00
#define TCD_SOFF         0x04
#define TCD_ATTR         0x06
#define TCD_NBYTES       0x08
#define TCD_SLAST        0x0C
#define TCD_DADDR        0x10
#define TCD_DOFF         0x14
#define TCD_CITER        0x16
#define TCD_DLASTSGA     0x18
#define TCD_CSR          0x1C
#define  TCD_CSR_START   (1 << 0)
#define  TCD_CSR_INTMAJOR (1 << 1)
#define  TCD_CSR_INTHALF (1 << 2)
#define  TCD_CSR_DONE    (1 << 15)
#define  TCD_CSR_MAJORELINK (1 << 3)   /* major loop channel link enable */
#define  TCD_CSR_ESG     (1 << 4)       /* scatter/gather enable */
#define  TCD_CSR_MAJORLINKCH_SHIFT 8
#define  TCD_CSR_MAJORLINKCH_MASK (0x1f << TCD_CSR_MAJORLINKCH_SHIFT)
#define TCD_BITER        0x1E
#define TCD_SIZE         0x20

struct S32K3EdmaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion tcd1;      /* TCD region 1 (channels 0-11) */
    MemoryRegion tcd2;      /* TCD region 2 (channels 12-31) */
    MemoryRegion *mem;      /* link: system memory for DMA transfers */
    AddressSpace  as;

    Clock        *module_clk;
    qemu_irq     irq[S32K3_EDMA_CHANNELS];
    qemu_irq     err_irq;

    uint32_t cr;
    uint32_t es;
    uint32_t erq;
    uint32_t eei;
    uint32_t intr;
    uint32_t err;
    uint32_t hrs;

    uint32_t ch_csr[S32K3_EDMA_CHANNELS];
    uint32_t ch_es[S32K3_EDMA_CHANNELS];
    uint32_t ch_int[S32K3_EDMA_CHANNELS];
    uint32_t ch_sbr[S32K3_EDMA_CHANNELS];

    uint8_t  tcd[S32K3_EDMA_CHANNELS][TCD_SIZE];

    /* 周期化传输：ptimer 逐 minor loop 节拍 */
    ptimer_state *timer;
    int active_ch;          /* 当前传输通道，-1=空闲 */
    uint32_t ch_saddr;      /* 当前 minor loop 源地址 */
    uint32_t ch_daddr;
    uint32_t ch_nbytes;
    int16_t  ch_soff;
    int16_t  ch_doff;
    uint32_t ch_citer;
    uint32_t ch_biter;
    int32_t  ch_slast;
    uint32_t ch_dlastsga;
};

#endif /* HW_DMA_S32K3_EDMA_H */
