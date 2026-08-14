/*
 * NXP S32K3xx clock tree (FXOSC / PLL / MC_CGM / MC_ME / MC_RGM)
 * QEMU device model
 *
 * Functional clock tree per S32K3xx RM Rev.11 (chapters 24-30):
 *
 *   FXOSC (8-40 MHz)  ---+                     +-- MUX_0_DC_0 -> CORE_CLK
 *                        |  PLLDV[RDIV]/[MFI]  |-- MUX_0_DC_1 -> AIPS_PLAT_CLK
 *   FIRC (48 MHz)   --+  |  fVCO = fREF*MFI/RDIV
 *                     |  +-> PLL PHI0 -------->+-- MUX_0_DC_2 -> AIPS_SLOW_CLK
 *                     +---> MUX_0_CSC[SELCTL]  +-- MUX_0_DC_3 -> HSE_CLK ...
 *
 * The MC_CGM instance computes the actual clock frequencies from the
 * MUX_0_CSC / MUX_0_DC_n register values and drives its three clock
 * outputs (sysclk, aips-plat-clk, aips-slow-clk) which the board feeds
 * to the ARM core and the peripheral models.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/s32k3_clkgen.h"
#include "qemu/log.h"
#include "qemu/module.h"

typedef enum {
    CLKGEN_FXOSC = CLKGEN_KIND_FXOSC,
    CLKGEN_PLL = CLKGEN_KIND_PLL,
    CLKGEN_MC_CGM = CLKGEN_KIND_MC_CGM,
    CLKGEN_MC_ME = CLKGEN_KIND_MC_ME,
    CLKGEN_MC_RGM = CLKGEN_KIND_MC_RGM,
    CLKGEN_SXOSC = CLKGEN_KIND_SXOSC,
    CLKGEN_FIRC = CLKGEN_KIND_FIRC,
} S32K3ClkgenKind;

/* ---------------- register bit definitions (RM) ---------------- */

/* FXOSC: 0x00 CTRL, 0x04 STATUS */
/* FXOSC (S32K3 RM + S32K348.h)：CTRL[0]=OSCON（S32K348.h 权威
 * FXOSC_CTRL_OSCON_SHIFT=0）、CTRL[29]=OSC_BYP、CTRL[23:16]=EOCV、
 * CTRL[0]=COMP_EN 同一位？——按 S32K348.h：OSCON=bit0；
 * STATUS[31]=OSC_STAT */
#define FXOSC_CTRL_OSCON      (1u << 0)
#define FXOSC_CTRL_OSC_BYP    (1u << 29)
#define FXOSC_STATUS_OSC_STAT (1u << 31)

/* PLLDIG (S32K348): 0x00 PLLCR, 0x04 PLLSR, 0x08 PLLDV, 0x80 PLLODIV_0
 * S32K348 无 PLLCLKMUX（参考源固定；PLLCLKMUX 仅 S32K310/311 有） */
#define PLLCR_PLLPD           (1u << 31)   /* 1=power down（复位默认） */
#define PLLSR_LOCK            (1u << 2)   /* S32K348.h PLL_PLLSR_LOCK_MASK=0x4（bit2） */
#define PLLDV_RDIV_SHIFT      12
#define PLLDV_RDIV_MASK       (0x7 << PLLDV_RDIV_SHIFT)   /* bits 14:12 */
#define PLLDV_MFI_SHIFT       0
#define PLLDV_MFI_MASK        0xff                        /* bits 7:0 */
#define PLLDV_ODIV2_SHIFT     25
#define PLLDV_ODIV2_MASK      (0x3f << PLLDV_ODIV2_SHIFT)
#define PLLODIV0_OFFSET       0x80
#define PLLODIV0_DIV_MASK     0x3f

/* MC_CGM: MUX_0 @ 0x300, DC_0..7 @ 0x308..0x324,
 * MUX_0_DIV_TRIG @ 0x338, MUX_0_DIV_UPD_STAT @ 0x33C */
#define CGM_MUX0_CSC          0x300
#define CGM_MUX0_CSS          0x304
#define CGM_MUX0_DC0          0x308
#define CGM_MUX0_DC1          0x30C
#define CGM_MUX0_DC2          0x310
#define CGM_MUX0_DC3          0x314
#define CGM_MUX0_DC4          0x318
#define CGM_MUX0_DC5          0x31C
#define CGM_MUX0_DC6          0x320
#define CGM_MUX0_DC7          0x324
#define CGM_MUX0_DIV_TRIG     0x338
#define CGM_MUX0_DIV_UPD_STAT 0x33C
#define CGM_SELCTL_SHIFT      24
#define CGM_SELCTL_MASK       (0x1f << CGM_SELCTL_SHIFT)
#define CGM_SEL_FIRC          0x00
#define CGM_SEL_PLL_PHI0      0x08
#define CGM_CSC_SAFE_SW       (1u << 4)    /* RM 25.5.6：bit4 安全切 FIRC */
#define CGM_CSC_CLK_SW        (1u << 3)    /* RM：bit3 时钟切换请求 */
#define CGM_DIV_MASK          0x7f
#define CGM_DIV_DE            (1u << 31)  /* divider enable */
#define CGM_CSS_SEL_STAT_SHIFT 22   /* RM 25.5.7：SELSTAT=bits26-22 */
#define CGM_CSS_SWTRG         (4u << 18)  /* RM：SWTRG=bits20-18，4=安全切换 */
#define CGM_CSS_CLK_SW        (1u << 2)   /* RM：CLK_SW=bit2 切换请求 */
#define CGM_CSS_SAFE_SW       (1u << 3)   /* RM：SAFE_SW=bit3 安全切换 */

/* MC_ME: 0x00 CTL_KEY, 0x04 MODE_CONF, 0x08 MODE_UPD, 0x0C MODE_STAT,
 * 0x100 PRTN0_PCONF, 0x104 PRTN0_PUPD, 0x108 PRTN0_STAT,
 * 0x140 PRTN0_CORE0_PCONF, 0x148 PRTN0_CORE0_STAT */
#define MCME_CTL_KEY_KEY      0x00005AF0
#define MCME_CTL_KEY_KEY_INV  0x0000A50F
#define MCME_MODE_STAT_PUPD    (1u << 0)
#define MCME_MODE_UPD_UPD      (1u << 0)
#define MCME_PCONF_CCE         (1u << 0)   /* core clock enable */
#define MCME_STAT_CCS          (1u << 0)   /* core clock status */
#define MCME_DEV_PSTAT_EN      (1u << 24)  /* PRTN0_DEV0_PSTAT: clock on */

/* MC_RGM (S32K348): 0x00 DES, 0x08 FES, 0x0C FERD, 0x10 FBRE, 0x14 FREC,
 * 0x18 FRET, 0x1C DRET, 0x20 ERCTRL, 0x24 RDSS */
#define MCRGM_DES             0x00
#define MCRGM_FES             0x08
#define MCRGM_FERD            0x0C
#define MCRGM_FBRE            0x10
#define MCRGM_FREC            0x14
#define MCRGM_FRET            0x18
#define MCRGM_DRET            0x1C
#define MCRGM_ERCTRL          0x20
#define MCRGM_RDSS            0x24
#define MCRGM_DES_POR         (1u << 0)   /* power-on reset */
#define MCRGM_FES_F_EXR       (1u << 0)   /* external reset */

struct S32K3ClkgenState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    S32K3ClkgenKind kind;
    uint32_t regs[0x400];   /* generic 4KB register image */

    /* clock inputs: reference clocks feeding this block */
    Clock *firc_clk;
    Clock *fxosc_clk;
    Clock *pll_in_clk;

    /* clock outputs (MC_CGM only): generated clocks for the board */
    Clock *pll_clk;      /* PLL PHI0 output (PLL instance) */
    Clock *clk_sys;
    Clock *clk_plat;
    Clock *clk_slow_out;

    /* computed PLL PHI0 rate (Hz), shared with MC_CGM via prop */
    uint64_t pll_phi0_hz;

    /* MC_RGM: reset request output + safe-clock request (board-wired) */
    qemu_irq reset_req;      /* 复位请求输出 */
    qemu_irq safe_sw_req;    /* 安全时钟切换请求输出 */
    qemu_irq safe_sw_in;     /* 安全时钟切换请求输入（MC_CGM 用） */

    uint32_t fxosc_hz;       /* FXOSC 频率（属性，默认 8MHz） */

    /* MC_ME state */
    uint8_t me_key_state;       /* 0=idle, 1=first key written */
    uint8_t me_mode;            /* current mode: 0=reset, 1=run, 2=standby */
    uint32_t me_mode_conf;      /* latched target mode config */
};

/* ---------------- helpers ---------------- */

static uint64_t s32k3_clkgen_div_apply(Clock *src, uint32_t dc)
{
    uint32_t div;
    uint64_t hz;

    if (!(dc & CGM_DIV_DE)) {
        return 0;   /* divider disabled */
    }
    div = (dc & CGM_DIV_MASK) + 1;
    hz = clock_get_hz(src);
    return div > 1 ? hz / div : hz;
}

/* ---------------- MC_CGM: recompute the clock tree ---------------- */

static void s32k3_cgm_update_clocks(S32K3ClkgenState *s)
{
    uint32_t csc = s->regs[CGM_MUX0_CSC / 4];
    uint32_t sel = (csc & CGM_SELCTL_MASK) >> CGM_SELCTL_SHIFT;
    Clock *src;
    uint64_t core_hz, plat_hz, slow_hz;

    /* SAFE_SW：切到安全时钟（FIRC），任何时刻都完成 */
    if (csc & CGM_CSC_SAFE_SW) {
        sel = CGM_SEL_FIRC;
    }

    src = (sel == CGM_SEL_PLL_PHI0) ? s->pll_in_clk : s->firc_clk;
    if (src == NULL) {
        src = s->firc_clk;
    }
    if (src == NULL) {
        return;
    }

    core_hz = s32k3_clkgen_div_apply(src, s->regs[CGM_MUX0_DC0 / 4]);
    plat_hz = s32k3_clkgen_div_apply(src, s->regs[CGM_MUX0_DC1 / 4]);
    slow_hz = s32k3_clkgen_div_apply(src, s->regs[CGM_MUX0_DC2 / 4]);

    /* DC_0..2 default reset values 0x8000_0000: enabled, DIV=0 -> /1 */
    clock_update_hz(s->clk_sys, core_hz);
    clock_update_hz(s->clk_plat, plat_hz);
    clock_update_hz(s->clk_slow_out, slow_hz);

    /* CSS: report the selected source back to firmware.
     * clkSw(bit2)/safeSw(bit3) 置位 = 切换请求被接受（固件
     * `while (CSS.clkSw == 0)` 等这些位变 1）；swIP(bit16) 恒 0 =
     * 无切换进行（固件 `while (CSS.swIP == 1)` 立即退出）。
     * 模型时钟切换即时完成，故置位后保持。 */
    s->regs[CGM_MUX0_CSS / 4] = (sel << CGM_CSS_SEL_STAT_SHIFT) |
                                (csc & CGM_CSC_SAFE_SW ? CGM_CSS_SWTRG : 0) |
                                CGM_CSS_CLK_SW | CGM_CSS_SAFE_SW;
}

/* ---------------- PLL: compute PHI0 from config ---------------- */

static void s32k3_pll_update(S32K3ClkgenState *s)
{
    uint32_t dv = s->regs[0x08 / 4];
    uint32_t odiv0 = s->regs[PLLODIV0_OFFSET / 4] & PLLODIV0_DIV_MASK;
    Clock *ref = s->fxosc_clk;   /* S32K348: PLL 参考源固定为 FXOSC_CLK */
    uint32_t rdiv, mfi, odiv2;
    uint64_t fref, fvco;

    if ((s->regs[0x00 / 4] & PLLCR_PLLPD)) {
        clock_update_hz(s->pll_clk, 0);
        s->pll_phi0_hz = 0;
        return;
    }
    if (ref == NULL) {
        return;
    }

    rdiv = (dv & PLLDV_RDIV_MASK) >> PLLDV_RDIV_SHIFT;
    mfi  = (dv & PLLDV_MFI_MASK) >> PLLDV_MFI_SHIFT;
    odiv2 = (dv & PLLDV_ODIV2_MASK) >> PLLDV_ODIV2_SHIFT;
    fref = clock_get_hz(ref);

    /* fVCO = fREF * (MFI / (RDIV+1)); PHI0 = fVCO / ((ODIV2+1) * (ODIV0+1)) */
    fvco = (fref * (mfi ? mfi : 1)) / (rdiv + 1);
    s->pll_phi0_hz = fvco / ((odiv2 + 1) * (odiv0 + 1));
    clock_update_hz(s->pll_clk, s->pll_phi0_hz);
}

/* MC_CGM 安全时钟切换输入：外部请求（MC_RGM）切到 FIRC */
static void s32k3_clkgen_safe_sw(void *opaque, int line, int level)
{
    S32K3ClkgenState *s = opaque;

    if (s->kind != CLKGEN_MC_CGM || !level) {
        return;
    }
    /* 置 SAFE_SW 位并重算时钟（切 FIRC） */
    s->regs[CGM_MUX0_CSC / 4] |= CGM_CSC_SAFE_SW;
    s32k3_cgm_update_clocks(s);
}

/* ---------------- register access ---------------- */

static void s32k3_clkgen_reset(DeviceState *dev)
{
    S32K3ClkgenState *s = S32K3_CLKGEN(dev);
    int i;

    memset(s->regs, 0, sizeof(s->regs));

    switch (s->kind) {
    case CLKGEN_FXOSC:
    case CLKGEN_SXOSC:
    case CLKGEN_FIRC:
        break;
    case CLKGEN_PLL:
        /* PLLDV reset 0C3F_1032: ODIV2=6, RDIV=1, MFI=0x32(50) */
        s->regs[0x08 / 4] = (6u << PLLDV_ODIV2_SHIFT) |
                            (1u << PLLDV_RDIV_SHIFT) |
                            0x32u;
        /* PLLCR reset: PLLPD=1 (disabled) */
        s->regs[0x00 / 4] = 0x80000000u;
        break;
    case CLKGEN_MC_CGM:
        /* MUX_0 dividers default enabled with DIV=0 -> divide by 1 */
        for (i = 0; i < 8; i++) {
            s->regs[(CGM_MUX0_DC0 + 4 * i) / 4] = CGM_DIV_DE;
        }
        /* 全部 15 个 MUX 的 CSS 复位：SELSTAT=0(FIRC)、SWTRG=4
         *（RM 复位 0010_0000h）。MUX_1..14 虽无建模时钟输出，
         * 固件仍会读其 CSS，须给合法复位值。 */
        for (i = 0; i < 15; i++) {
            s->regs[(CGM_MUX0_CSS + i * 0x40) / 4] =
                (CGM_SEL_FIRC << CGM_CSS_SEL_STAT_SHIFT) | CGM_CSS_SWTRG;
        }
        /* 复位后时钟树即输出 FIRC 48MHz 分频（真机复位默认时钟运行） */
        s32k3_cgm_update_clocks(s);
        break;
    case CLKGEN_MC_ME:
        s->me_key_state = 0;
        s->me_mode = 0;             /* start in reset mode */
        s->me_mode_conf = 0;
        s->regs[0x00 / 4] = MCME_CTL_KEY_KEY;      /* CTL_KEY reset 5AF0 */
        /* PRTN0_PCONF reset 1 (partition 0 ready), STAT reset 1 */
        s->regs[0x100 / 4] = 0x1;
        s->regs[0x108 / 4] = 0x1;
        /* COFB 时钟状态寄存器（PRTN0/1_COFB0/1_STAT @0x110/0x114/0x310/0x314）：
         * 固件上电流程轮询这些位等待外设时钟就绪（如 PRTN0_COFB0_STAT bit3、
         * PRTN0_COFB1_STAT & 0xF7DF、PRTN1_COFB0_STAT & 0xFFBFFFFF）。
         * 模型时钟即时生效（无真实上电延迟），故状态直接置"全开"避免死等
         * （手册复位值 0x0C000004/0x00001000/0x5E3F0007/0x7CFE2FFC 含未就绪位，
         * 固件使能后位才置位——模型模拟使能完成态）。 */
        s->regs[0x110 / 4] = 0xFFFFFFFFu;   /* PRTN0_COFB0_STAT：时钟全开 */
        s->regs[0x114 / 4] = 0xFFFFFFFFu;   /* PRTN0_COFB1_STAT */
        s->regs[0x310 / 4] = 0xFFFFFFFFu;   /* PRTN1_COFB0_STAT */
        s->regs[0x314 / 4] = 0xFFFFFFFFu;   /* PRTN1_COFB1_STAT */
        s->regs[0x318 / 4] = 0xFFFFFFFFu;   /* PRTN1_COFB2_STAT */
        s->regs[0x31C / 4] = 0xFFFFFFFFu;   /* PRTN1_COFB3_STAT */
        s->regs[0x510 / 4] = 0xFFFFFFFFu;   /* PRTN2_COFB0_STAT */
        s->regs[0x514 / 4] = 0xFFFFFFFFu;   /* PRTN2_COFB1_STAT */
        s->regs[0x518 / 4] = 0xFFFFFFFFu;   /* PRTN2_COFB2_STAT */
        /* PRTN1/2_PCONF：分区就绪（pce=ENABLE）——固件等 pce 置位 */
        s->regs[0x300 / 4] = 0x1;
        s->regs[0x500 / 4] = 0x1;
        break;
    case CLKGEN_MC_RGM:
        /* DES: POR set; FRET reset threshold 0xF */
        s->regs[MCRGM_DES / 4] = MCRGM_DES_POR;
        s->regs[MCRGM_FRET / 4] = 0xF;
        break;
    }
}

static uint64_t s32k3_clkgen_read(void *opaque, hwaddr addr, unsigned size)
{
    S32K3ClkgenState *s = opaque;
    uint32_t r = 0;

    if (addr >= sizeof(s->regs)) {
        return 0;
    }

    switch (s->kind) {
    case CLKGEN_KIND_FIRC:
        switch (addr) {
        case 0x04:
            /* FIRC STATUS_REGISTER.fircStat（bit0）：FIRC 常开 48MHz——
             * 固件 SystemBypassPll 写 STDBY_ENABLE.fircEn 后轮询此位，
             * 原模型读 0 会死等。 */
            r = 1;
            break;
        default:
            r = s->regs[addr / 4];
            break;
        }
        break;

    case CLKGEN_SXOSC:
        switch (addr) {
        case 0x04:
            /* SXOSC 常开（32 KHz，无 OSCON 位），STATUS 恒稳定 */
            r = FXOSC_STATUS_OSC_STAT;
            break;
        default:
            r = s->regs[addr / 4];
            break;
        }
        break;

    case CLKGEN_FXOSC:
        switch (addr) {
        case 0x04:
            /* STATUS: stable once OSCON was written */
            r = (s->regs[0x00 / 4] & FXOSC_CTRL_OSCON) ?
                FXOSC_STATUS_OSC_STAT : 0;
            break;
        default:
            r = s->regs[addr / 4];
            break;
        }
        break;

    case CLKGEN_PLL:
        switch (addr) {
        case 0x04:
            /* PLLSR: LOCK once PLLPD is cleared (enabled) */
            /* 固件 Mcu_GetPllStatus 等 PLL lock；模型时钟即时生效（无上电延迟），
             * lock 恒置（pllPd=1 断电态也置——避免固件死等；时钟频率仍由
             * pll_update 按 PLLCR/DV 计算）。 */
            r = PLLSR_LOCK;
            break;
        default:
            r = s->regs[addr / 4];
            break;
        }
        break;

    case CLKGEN_MC_CGM:
        switch (addr) {
        case CGM_MUX0_CSS:
            r = s->regs[CGM_MUX0_CSS / 4];
            break;
        default:
            r = s->regs[addr / 4];
            break;
        }
        break;

    case CLKGEN_MC_ME:
        switch (addr) {
        case 0x0C:
            /* MODE_STAT: PREV_MODE bit0 (0=reset, 1=standby).
             * 运行期间无 mode update 挂起。 */
            r = (s->me_mode == 2) ? MCME_MODE_STAT_PUPD : 0;
            break;
        case 0x108:
            /* PRTN0_STAT: partition 0 ready (always) */
            r = 0x1;
            break;
        case 0x148:
            /* PRTN0_CORE0_STAT: CCS reflects CCE (clock on if enabled) */
            r = (s->regs[0x140 / 4] & MCME_PCONF_CCE) ? MCME_STAT_CCS : 0;
            break;
        case 0x188:
            /* PRTN0_CORE2_STAT[31] WFI：安全 BAF 已进入 WFI（恒置位，
             * 否则固件 SetFircDivSelHSEb 等 WFI 会超时报错） */
            r = (1u << 31);
            break;
        default:
            r = s->regs[addr / 4];
            break;
        }
        break;

    case CLKGEN_MC_RGM:
        r = s->regs[addr / 4];
        break;
    }
    return r;
}

static void s32k3_clkgen_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned size)
{
    S32K3ClkgenState *s = opaque;
    uint32_t v = value;

    if (addr >= sizeof(s->regs)) {
        return;
    }

    if (s->kind == CLKGEN_MC_ME) {
        switch (addr) {
        case 0x00:
            /* CTL_KEY：key 序列 0x5AF0 -> 0xA50F 触发模式切换/进程 */
            s->regs[0] = v;
            if (v == MCME_CTL_KEY_KEY) {
                s->me_key_state = 1;
            } else if (v == MCME_CTL_KEY_KEY_INV && s->me_key_state == 1) {
                s->me_key_state = 0;
                /* 触发：应用 MODE_CONF 目标模式 */
                if (s->me_mode_conf & MCME_MODE_UPD_UPD) {
                    s->me_mode = 1;   /* 进入 RUN（简化：不实现 standby） */
                }
                /* 外设/核心时钟使能完成：PRTN0_DEV0_PSTAT(0x310) bit24 置位，
                 * 固件启动序列 WaitForClock 轮询此位。 */
                s->regs[0x310 / 4] |= MCME_DEV_PSTAT_EN;   /* bit24 */
            } else {
                s->me_key_state = 0;
            }
            return;
        case 0x04:
            /* MODE_CONF：暂存目标模式配置 */
            s->me_mode_conf = v;
            s->regs[1] = v;
            return;
        case 0x08:
            /* MODE_UPD：写 UPD=1 发起模式变更请求（配合 key 序列） */
            s->me_mode_conf = (s->me_mode_conf & ~MCME_MODE_UPD_UPD) |
                              (v & MCME_MODE_UPD_UPD);
            s->regs[2] = v;
            return;
        case 0x130:
        case 0x134:
        case 0x330:
        case 0x334:
            /* COFB CLKEN（0x130/0x134/0x330/0x334）：固件使能/禁用外设时钟。
             * STAT（0x110/0x114/0x310/0x314）同步——使能写 1 置位、禁用写 0
             * 清位（固件 Clock_Ip_Gate 关时钟等 STAT 位清 0，全 1 不回落会超时）。 */
            s->regs[addr / 4] = v;
            s->regs[(addr - 0x20) / 4] = v;   /* CLKEN - 0x20 = STAT */
            return;
        case 0x104:   /* PRTN0_PUPD */
        case 0x304:   /* PRTN1_PUPD */
        case 0x504:   /* PRTN2_PUPD */
            /* 分区更新触发：写 PUPD=1 后硬件应用 PCONF 配置并自动清 PUPD
             * （固件 while (PRTNx_PUPD.pcud != 0) 死等该位清零）。
             * 模型即时完成：PCONF 锁存进 STAT（0x108/0x308/0x508）、PUPD 清 0。 */
            if (v & 1u) {
                unsigned base = addr & ~0xffu;   /* 0x100 / 0x300 / 0x500 */
                s->regs[(base + 0x08) / 4] = s->regs[(base + 0x00) / 4];
                v &= ~1u;
            }
            s->regs[addr / 4] = v;
            return;
        case 0x140:
            /* PRTN0_CORE0_PCONF[CCE]：核心时钟门控 */
            s->regs[0x140 / 4] = v & MCME_PCONF_CCE;
            return;
        default:
            s->regs[addr / 4] = v;
            return;
        }
    }
    if (s->kind == CLKGEN_MC_RGM) {
        switch (addr) {
        case MCRGM_DES:
        case MCRGM_FES:
            /* W1C 事件标志；写 1 清除，写 0 保持。
             * 复位事件设置（非清除）时发出复位请求。 */
            {
                uint32_t before = s->regs[addr / 4];
                s->regs[addr / 4] &= ~v;
                /* 新事件（本次写入设置了某位）-> 发复位请求脉冲 */
                if (v & ~before) {
                    qemu_set_irq(s->reset_req, 1);
                    qemu_set_irq(s->reset_req, 0);
                }
            }
            return;
        case MCRGM_FERD:
        case MCRGM_FBRE:
        case MCRGM_FREC:
        case MCRGM_FRET:
        case MCRGM_DRET:
        case MCRGM_ERCTRL:
        case MCRGM_RDSS:
            s->regs[addr / 4] = v;
            return;
        default:
            s->regs[addr / 4] = v;
            return;
        }
    }
    s->regs[addr / 4] = v;

    /* recompute the affected clock domain on config writes */
    switch (s->kind) {
    case CLKGEN_FXOSC:
        if (addr == 0x00) {
            /* OSCON toggled: drive the FXOSC clock output (fxosc-hz 属性，
             * 板卡 -global s32k3-clkgen.fxosc-hz=16000000 生效；
             * 原写死 8MHz 导致 PLL 链路 2x 失真） */
            clock_update_hz(s->fxosc_clk,
                            (v & FXOSC_CTRL_OSCON) ? s->fxosc_hz : 0);
        }
        break;
    case CLKGEN_PLL:
        if (addr == 0x00 || addr == 0x08 || addr == PLLODIV0_OFFSET) {
            s32k3_pll_update(s);
        }
        break;
    case CLKGEN_MC_CGM:
        if (addr == CGM_MUX0_DIV_TRIG) {
            /* 写 TRIG 触发分频更新：时钟立即重算，
             * DIV_UPD_STAT 瞬时置位后清除（模型即时完成） */
            s32k3_cgm_update_clocks(s);
            s->regs[CGM_MUX0_DIV_UPD_STAT / 4] = 0x1;
            s->regs[CGM_MUX0_DIV_UPD_STAT / 4] = 0x0;
            return;
        }
        if ((addr >= CGM_MUX0_CSC && addr <= CGM_MUX0_DC7)) {
            s32k3_cgm_update_clocks(s);
            break;
        }
        /* MUX_1..14 CSC @0x340 + n*0x40：时钟输出未建模，但固件启动流程
         * 轮询 MUX_n_CSS（clkSw/safeSw/swIP），必须镜像 CSC->CSS，
         * 否则 while (CSS.safeSw == 0) / while (CSS.clkSw == 0) 死循环。 */
        if (addr >= 0x340 && addr <= 0x300 + 14 * 0x40 &&
            (addr % 0x40) == 0) {
            uint32_t sel = (v & CGM_SELCTL_MASK) >> CGM_SELCTL_SHIFT;
            s->regs[(addr + 4) / 4] =
                (sel << CGM_CSS_SEL_STAT_SHIFT) |
                (v & CGM_CSC_SAFE_SW ? CGM_CSS_SWTRG : 0) |
                CGM_CSS_CLK_SW | CGM_CSS_SAFE_SW;
        }
        break;
    default:
        break;
    }
}

static const MemoryRegionOps s32k3_clkgen_ops = {
    .read = s32k3_clkgen_read,
    .write = s32k3_clkgen_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static void s32k3_clkgen_init(Object *obj)
{
    S32K3ClkgenState *s = S32K3_CLKGEN(obj);

    memory_region_init_io(&s->iomem, obj, &s32k3_clkgen_ops, s,
                          TYPE_S32K3_CLKGEN, sizeof(s->regs));
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->reset_req);
    qdev_init_gpio_in_named(DEVICE(s), s32k3_clkgen_safe_sw, "safe-sw", 1);

    /* 统一声明全部 clock（在 realize 前），板卡在 realize 前 connect
     * 输入、realize 后取输出。角色（in/out）对全部 kind 一致声明，
     * 未被某 kind 使用的 clock 忽略即可。 */
    s->firc_clk = qdev_init_clock_in(DEVICE(s), "firc-clk", NULL, NULL, 0);
    s->fxosc_clk = qdev_init_clock_in(DEVICE(s), "fxosc-clk", NULL, NULL, 0);
    s->pll_clk = qdev_init_clock_out(DEVICE(s), "pll-clk");   /* PLL PHI0 输出 */
    s->pll_in_clk = qdev_init_clock_in(DEVICE(s), "pll-in-clk", NULL, NULL, 0);
    s->clk_sys  = qdev_init_clock_out(DEVICE(s), S32K3_CLKGEN_CLK_SYSCLK);
    s->clk_plat = qdev_init_clock_out(DEVICE(s), S32K3_CLKGEN_CLK_AIPS_PLAT);
    s->clk_slow_out = qdev_init_clock_out(DEVICE(s), S32K3_CLKGEN_CLK_AIPS_SLOW);
}

static void s32k3_clkgen_realize(DeviceState *dev, Error **errp)
{
    S32K3ClkgenState *s = S32K3_CLKGEN(dev);

    /* provide default reference frequencies so the tree works even if
     * the board does not connect every source */
    if (s->kind == CLKGEN_MC_CGM) {
        if (!clock_has_source(s->firc_clk)) {
            clock_set_hz(s->firc_clk, 48000000);   /* FIRC 48 MHz */
        }
        if (!clock_has_source(s->pll_in_clk)) {
            clock_set_hz(s->pll_in_clk, 0);
        }
        s32k3_cgm_update_clocks(s);
    }
    if (s->kind == CLKGEN_PLL) {
        if (!clock_has_source(s->fxosc_clk)) {
            clock_set_hz(s->fxosc_clk, 8000000);   /* FXOSC 8 MHz default */
        }
        s32k3_pll_update(s);
    }
}

static const Property s32k3_clkgen_properties[] = {
    DEFINE_PROP_UINT32("kind", S32K3ClkgenState, kind, CLKGEN_MC_CGM),
    DEFINE_PROP_UINT32("fxosc-hz", S32K3ClkgenState, fxosc_hz, 8000000),
};

static void s32k3_clkgen_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, s32k3_clkgen_reset);
    device_class_set_props(dc, s32k3_clkgen_properties);
    dc->realize = s32k3_clkgen_realize;
    dc->desc = "NXP S32K3xx clock tree (FXOSC/PLL/MC_CGM/MC_ME/MC_RGM)";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo s32k3_clkgen_types[] = {
    {
        .name          = TYPE_S32K3_CLKGEN,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(S32K3ClkgenState),
        .instance_init = s32k3_clkgen_init,
        .class_init    = s32k3_clkgen_class_init,
    },
};

DEFINE_TYPES(s32k3_clkgen_types)
