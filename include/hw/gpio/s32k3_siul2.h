/*
 * NXP S32K3xx SIUL2 (System Integration Unit Lite 2) - GPIO/pad control
 * QEMU device model - header
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_GPIO_S32K3_SIUL2_H
#define HW_GPIO_S32K3_SIUL2_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "qom/object.h"

#define TYPE_S32K3_SIUL2 "s32k3-siul2"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3Siul2State, S32K3_SIUL2)

#define S32K3_SIUL2_MMIO_SIZE 0x4000

/* SIUL2 register offsets (subset, S32K3xx RM SIUL2 chapter) */
#define SIUL2_MIDR1      0x0004
#define SIUL2_MIDR2      0x0008
#define SIUL2_DISR0      0x0010  /* DMA/Interrupt status flag 0 */
#define SIUL2_DIRER0     0x0018  /* DMA/Interrupt request enable 0 */
#define SIUL2_DIRSR0     0x0020  /* DMA/Interrupt request select 0 */
#define SIUL2_IREER0     0x0028  /* Interrupt rising-edge enable */
#define SIUL2_IFEER0     0x0030  /* Interrupt falling-edge enable */
#define SIUL2_IFER0      0x0038  /* Interrupt filter enable */

/* MSCR pad multiplexing registers: 0x0240 + 4*n  (n = 0..511) */
#define SIUL2_MSCR_BASE  0x0240
#define SIUL2_MSCR_COUNT 512
#define SIUL2_IMCR_BASE  0x0A40
#define SIUL2_IMCR_COUNT 512

/* GPIO pad data out: 0x1300 + 1*n (8-bit, n=0..511) */
#define SIUL2_GPDO_BASE  0x1300
/* GPIO pad data in:  0x1500 + 1*n */
#define SIUL2_GPDI_BASE  0x1500
/* Parallel GPIO pad data out: 0x1320 + 4*n */
#define SIUL2_PGPDO_BASE 0x1700
/* Parallel GPIO pad data in:  0x1500... use 0x1540 + 4*n */
#define SIUL2_PGPDI_BASE 0x1740

/* MSCR fields */
#define MSCR_SSS_MASK    0xF        /* SSS[0:3] 4 位 */
#define MSCR_OBE         (1 << 21)  /* output buffer enable */
#define MSCR_IBE         (1 << 19)  /* input buffer enable */
#define MSCR_SRC         (1 << 14)  /* slew rate control */

#define S32K3_NUM_GPIO   512

struct S32K3Siul2State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock        *module_clk;
    qemu_irq     irq[4];   /* EIRQ0-7 / 8-15 / 16-23 / 24-31 */

    uint32_t mscr[SIUL2_MSCR_COUNT];
    uint32_t imcr[SIUL2_IMCR_COUNT];
    uint32_t shadow[0x1000 / 4];   /* 全区影子：未实现偏移读回写值 */
    uint32_t disr0;
    uint32_t direr0;
    uint32_t dirsr0;
    uint32_t ireer0;
    uint32_t ifeer0;
    uint32_t ifer0;

    uint8_t  gpio_out[S32K3_NUM_GPIO];  /* value written to GPDO */
    uint8_t  gpio_in[S32K3_NUM_GPIO];   /* value driven into the pad */

    /* IFER 滤波（去毛刺）：输入变化后保持稳定一个采样周期才确认沿 */
    QEMUTimer   *filt_timer;
    int          filt_pin;      /* -1 = 无待确认 */
    uint8_t      filt_level;    /* 待确认的目标电平 */

    qemu_irq gpios[S32K3_NUM_GPIO];     /* qdev gpio output lines */
};

#endif /* HW_GPIO_S32K3_SIUL2_H */
