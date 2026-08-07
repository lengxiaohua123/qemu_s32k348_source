/*
 * QEMU S32K348EVB Board Definition
 * Based on NXP S32K3xx Data Sheet Rev. 14 (April 2026)
 * Target: BCOM Motor Controller (240MHz, 8MB Flash, 1152KB RAM)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_S32K348EVB_H
#define HW_ARM_S32K348EVB_H

#include "hw/core/boards.h"
#include "hw/arm/armv7m.h"
#include "hw/char/s32k3_lpuart.h"
#include "hw/ssi/s32k3_lpspi.h"
#include "hw/gpio/s32k3_siul2.h"
#include "hw/net/s32k3_flexcan.h"
#include "hw/net/s32k3_emac.h"
#include "hw/dma/s32k3_edma.h"      /* native S32K3xx eDMA */
#include "qom/object.h"
#include "hw/core/clock.h"

/* ------------------------------------------------------------------
 *  S32K348 Memory Map (from S32K3xx RM / Data Sheet)
 * ------------------------------------------------------------------ */
#define S32K348_ITCM_BASE       0x00000000
#define S32K348_ITCM_SIZE       (128 * 1024)    /* 128 KB */

#define S32K348_DTCM_BASE       0x20000000
#define S32K348_DTCM_SIZE       (256 * 1024)    /* 256 KB */

#define S32K348_SRAM0_BASE      0x20400000
#define S32K348_SRAM1_BASE      0x20440000
#define S32K348_SRAM2_BASE      0x20480000
#define S32K348_SRAM_BLK_SIZE   (256 * 1024)    /* 256 KB each */
#define S32K348_SRAM_BLOCKS     3

#define S32K348_FLASH0_BASE     0x00400000
#define S32K348_FLASH1_BASE     0x00600000
#define S32K348_FLASH2_BASE     0x00800000
#define S32K348_FLASH3_BASE     0x00A00000
#define S32K348_FLASH_BLK_SIZE  (2 * 1024 * 1024)   /* 2 MB each */
#define S32K348_FLASH_BLOCKS    4

#define S32K348_DATA_FLASH_BASE 0x10000000
#define S32K348_DATA_FLASH_SIZE (128 * 1024)    /* 128 KB */

#define S32K348_UTEST_BASE      0x1B000000
#define S32K348_UTEST_SIZE      (8 * 1024)      /* 8 KB */

/* ------------------------------------------------------------------
 *  AIPS-Lite Peripheral Base Addresses
 *  (S32K3xx RM Rev.11 附件 S32K3xx_memory_map.xlsx, S32K348 列)
 * ------------------------------------------------------------------ */
#define S32K348_LCU0_BASE       0x40098000
#define S32K348_LCU1_BASE       0x4009C000

#define S32K348_EMIOS0_BASE     0x40088000
#define S32K348_EMIOS1_BASE     0x4008C000
#define S32K348_EMIOS2_BASE     0x40090000

#define S32K348_BCTU_BASE       0x40084000

#define S32K348_ADC0_BASE       0x400A0000
#define S32K348_ADC1_BASE       0x400A4000
#define S32K348_ADC2_BASE       0x400A8000

/* PIT: S32K348 有 PIT0/PIT1/PIT2 三个实例（每个 4 通道） */
#define S32K348_PIT0_BASE       0x400B0000
#define S32K348_PIT1_BASE       0x400B4000
#define S32K348_PIT2_BASE       0x402FC000
#define S32K348_PIT_INSTANCES   3
#define S32K348_PIT_CHANNELS    4

/* Clock / reset / watchdog blocks (S32K3xx system map) */
#define S32K348_FXOSC_BASE      0x402D4000
#define S32K348_PLL_BASE        0x402E0000
#define S32K348_MCCGM_BASE      0x402D8000
#define S32K348_MCME_BASE       0x402DC000
#define S32K348_MCRGM_BASE      0x4028C000
#define S32K348_STM0_BASE      0x40274000
#define S32K348_SWT0_BASE       0x40270000

#define S32K348_LPI2C0_BASE     0x40350000
#define S32K348_LPI2C1_BASE     0x40354000
#define S32K348_LPI2C_INSTANCES 2

#define S32K348_FLEXCAN0_BASE   0x40304000
#define S32K348_FLEXCAN1_BASE   0x40308000
#define S32K348_FLEXCAN2_BASE   0x4030C000
#define S32K348_FLEXCAN3_BASE   0x40310000
#define S32K348_FLEXCAN4_BASE   0x40314000
#define S32K348_FLEXCAN5_BASE   0x40318000
#define S32K348_FLEXCAN6_BASE   0x4031C000
#define S32K348_FLEXCAN7_BASE   0x40320000
#define S32K348_FLEXCAN_INSTANCES 8

#define S32K348_LPSPI0_BASE     0x40358000
#define S32K348_LPSPI1_BASE     0x4035C000
#define S32K348_LPSPI2_BASE     0x40360000
#define S32K348_LPSPI3_BASE     0x40364000
#define S32K348_LPSPI4_BASE     0x404BC000
#define S32K348_LPSPI5_BASE     0x404C0000
#define S32K348_LPSPI_INSTANCES 6

#define S32K348_SIUL2_BASE      0x40290000

/* one EMAC (GMAC0) @ AIPS2 */
#define S32K348_EMAC_BASE       0x40484000

#define S32K348_EDMA_BASE       0x4020C000
#define S32K348_EDMA_TCD1_BASE  0x40210000
#define S32K348_EDMA_TCD2_BASE  0x40410000

#define S32K348_LPUART0_BASE    0x40328000
#define S32K348_LPUART1_BASE    0x4032C000
/* LPUART 2-7: 0x40330000 - 0x40344000, step 0x4000 */
#define S32K348_LPUART8_BASE    0x4048C000
/* LPUART 9-15: 0x40490000 - 0x404A8000, step 0x4000 */
#define S32K348_LPUART_OFFSET   0x4000
#define S32K348_LPUART_INSTANCES 16

/* ------------------------------------------------------------------
 *  Clock Frequencies (from BCOM HLR / S32K348 Data Sheet)
 * ------------------------------------------------------------------ */
#define S32K348_SYSCLK_HZ       240000000   /* 240 MHz */
#define S32K348_AIPS_PLAT_HZ    240000000   /* eMIOS, ADC, BCTU, LCU */
#define S32K348_AIPS_SLOW_HZ    60000000    /* PIT, SPI, CAN, TRGMUX, FLASH */

/* ------------------------------------------------------------------
 *  NVIC IRQ numbers
 *  (S32K3xx RM 附件 S32K3xx_interrupt_map.xlsx, S32K348 列)
 *  vector = irq + 16
 * ------------------------------------------------------------------ */
#define S32K348_IRQ_LPUART0     141
#define S32K348_IRQ_LPUART1     142
#define S32K348_IRQ_LPUART2     143
#define S32K348_IRQ_LPUART3     144
#define S32K348_IRQ_LPUART4     145
#define S32K348_IRQ_LPUART5     146
#define S32K348_IRQ_LPUART6     147
#define S32K348_IRQ_LPUART7     148
#define S32K348_IRQ_LPUART8     149
#define S32K348_IRQ_LPUART9     150
#define S32K348_IRQ_LPUART10    151
#define S32K348_IRQ_LPUART11    152
#define S32K348_IRQ_LPUART12    153
#define S32K348_IRQ_LPUART13    154
#define S32K348_IRQ_LPUART14    155
#define S32K348_IRQ_LPUART15    156

#define S32K348_IRQ_LPSPI0      165
#define S32K348_IRQ_LPSPI1      166
#define S32K348_IRQ_LPSPI2      167
#define S32K348_IRQ_LPSPI3      168
#define S32K348_IRQ_LPSPI4      169
#define S32K348_IRQ_LPSPI5      170

#define S32K348_IRQ_FLEXCAN0    109   /* 0: BOFF, 1: MB line0, 2: MB line32 */
#define S32K348_IRQ_FLEXCAN1    113
#define S32K348_IRQ_FLEXCAN2    116
#define S32K348_IRQ_FLEXCAN3    119
#define S32K348_IRQ_FLEXCAN4    121
#define S32K348_IRQ_FLEXCAN5    123
#define S32K348_IRQ_FLEXCAN6    125
#define S32K348_IRQ_FLEXCAN7    127

#define S32K348_IRQ_SIUL2_EIRQ0 53
#define S32K348_IRQ_EMAC        224   /* GMAC0 common interrupt */
#define S32K348_IRQ_BCTU        87

/* PIT 实例 x 通道 0-3：IRQ = 96 + 实例*1 + 通道 */
#define S32K348_IRQ_PIT0        96
#define S32K348_IRQ_PIT1        97
#define S32K348_IRQ_PIT2        98
#define S32K348_IRQ_PIT3        99

#define S32K348_IRQ_ADC0        180
#define S32K348_IRQ_ADC1        181
#define S32K348_IRQ_ADC2        182

/* eMIOS: 每实例 6 条（ch23/19/15/11/7/3） */
#define S32K348_IRQ_EMIOS0      61
#define S32K348_IRQ_EMIOS1      69
#define S32K348_IRQ_EMIOS2      77

#define S32K348_IRQ_LPI2C0      161
#define S32K348_IRQ_LPI2C1      162

#define S32K348_IRQ_LCU0        92
#define S32K348_IRQ_LCU1        93
#define S32K348_IRQ_SWT0        42

/* eDMA: 通道 0-31 完成/错误共用 IRQ 4-35 */
#define S32K348_IRQ_EDMA_CH0    4     /* DMA channels 0-31: irq 4-35 */

/* ------------------------------------------------------------------
 *  QOM Type Definitions
 * ------------------------------------------------------------------ */
#define TYPE_S32K348EVB_MACHINE "s32k348evb-machine"
OBJECT_DECLARE_SIMPLE_TYPE(S32K348EVBMachineState, S32K348EVB_MACHINE)

#define TYPE_S32K348_MCU        "s32k348"

/* ------------------------------------------------------------------
 *  Board Machine State
 * ------------------------------------------------------------------ */
struct S32K348EVBMachineState {
    MachineState parent_obj;

    /* Cortex-M7 CPU core (Lockstep core, QEMU sees single M7) */
    ARMv7MState armv7m;

    /* System memory bus */
    MemoryRegion *system_memory;

    /* Tightly-Coupled Memory (384KB total: 128KB ITCM + 256KB DTCM) */
    MemoryRegion cpu_itcm;
    MemoryRegion cpu_dtcm;

    /* SRAM blocks (3 x 256KB = 768KB) */
    MemoryRegion sram[S32K348_SRAM_BLOCKS];

    /* Code Flash blocks (4 x 2MB = 8MB) */
    MemoryRegion rom[S32K348_FLASH_BLOCKS];

    /* Data Flash (128KB) */
    MemoryRegion data_flash;

    /* UTEST / OTP (8KB) */
    MemoryRegion utest;

    /* Clocks: FIRC const + tree blocks (FXOSC/PLL/MC_CGM) */
    Clock *sysclk;
    Clock *aips_plat_clk;
    Clock *aips_slow_clk;
    Clock *firc_clk;
    Clock *sirc_clk;
    Clock *fxosc_clk;
    uint32_t fxosc_hz;   /* FXOSC 频率（属性可配，默认 8MHz） */
    Clock *pll_clk;
    DeviceState *clkgen;

    /* Functional peripherals */
    S32K3LpuartState   *uart[S32K348_LPUART_INSTANCES];
    S32K3LpspiState    *spi[S32K348_LPSPI_INSTANCES];
    S32K3FlexcanState  *can[S32K348_FLEXCAN_INSTANCES];
    S32K3Siul2State    *siul2;
    S32K3EmacState     *emac;

    /* 外部信号注入（inject-ext-irq）：保持高电平至 timer 到期后拉低，
     * 满足 SIUL2 IFER 滤波（两拍同电平）确认沿检测 */
    QEMUTimer         *inject_timer;
    qemu_irq           inject_last_irq;

    /* eDMA (native S32K3xx model) */
    S32K3EdmaState *dma;

    /* timers / adc / i2c (opaque pointers, realized by board init) */
    DeviceState *pit;
    DeviceState *emios[3];
    DeviceState *adc[3];
    DeviceState *i2c[S32K348_LPI2C_INSTANCES];
    DeviceState *lcu[2];
    DeviceState *bctu;
    DeviceState *swt[2];
};

#endif /* HW_ARM_S32K348EVB_H */
