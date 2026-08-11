/*
 * NXP S32K3xx clock tree (FXOSC / PLL / MC_CGM / MC_ME / MC_RGM)
 * QEMU device model - header
 *
 * Implements a functional clock tree per S32K3xx RM Rev.11:
 *   - FXOSC : 8-40 MHz crystal oscillator, CTRL[OSCON] -> STATUS[OSC_STAT]
 *   - PLL   : PLLDIG, fVCO = fREF * MFI / RDIV,
 *             fPHI0 = fVCO / (ODIV2+1) / (PLLODIV_0[DIV]+1), LOCK on enable
 *   - MC_CGM: MUX_0 selects FIRC_CLK or PLL_PHI0_CLK, dividers DC_0..DC_7
 *             produce CORE_CLK / AIPS_PLAT_CLK / AIPS_SLOW_CLK / ...
 *   - MC_ME : mode entry, CTL_KEY sequence, MODE_STAT[PUPD]
 *   - MC_RGM: reset status flags
 *
 * The MC_CGM instance exposes three clock outputs (sysclk, aips_plat,
 * aips_slow) that the board feeds to the ARM core and the peripherals,
 * so firmware clock configuration really changes peripheral clock rates.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_S32K3_CLKGEN_H
#define HW_MISC_S32K3_CLKGEN_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "qom/object.h"

#define TYPE_S32K3_CLKGEN "s32k3-clkgen"
OBJECT_DECLARE_SIMPLE_TYPE(S32K3ClkgenState, S32K3_CLKGEN)

/* clkgen instance kinds (property "kind") */
#define CLKGEN_KIND_FXOSC   0
#define CLKGEN_KIND_PLL     1
#define CLKGEN_KIND_MC_CGM  2
#define CLKGEN_KIND_MC_ME   3
#define CLKGEN_KIND_MC_RGM  4
#define CLKGEN_KIND_SXOSC   5
#define CLKGEN_KIND_FIRC    6

/* Clock output names of the MC_CGM instance */
#define S32K3_CLKGEN_CLK_SYSCLK     "sysclk"
#define S32K3_CLKGEN_CLK_AIPS_PLAT  "aips-plat-clk"
#define S32K3_CLKGEN_CLK_AIPS_SLOW  "aips-slow-clk"

#endif /* HW_MISC_S32K3_CLKGEN_H */
