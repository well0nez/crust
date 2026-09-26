/*
 * Copyright © 2020-2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#ifndef PLATFORM_PRCM_H
#define PLATFORM_PRCM_H

#include <util.h>
#include <platform/devices.h>

/*
 * R_PRCM in the D1 layout (kernel sun20i-d1-r-ccu model for this SoC;
 * gates confirmed by the blob's clock helper 0x5320: 0x11c, 0x1cc, 0x20c,
 * 0x260, and by 0xdd20/0xdea8: 0x244, 0x250, 0x260).
 */
#define CPUS_CLK_REG                     (DEV_R_PRCM + 0x0000)
#define CPUS_CLK_REG_CLK_SRC(x)          ((x) << 24)  /* 0 hosc 1 losc 2 iosc 3 pll */
#define CPUS_CLK_REG_CLK_SRC_MASK        (0x3 << 24)
#define CPUS_CLK_REG_DIV_P(x)            ((x) << 8)
#define CPUS_CLK_REG_DIV_P_MASK          (0x3 << 8)
#define CPUS_CLK_REG_PRE_DIV(x)          ((x) << 0)
#define CPUS_CLK_REG_PRE_DIV_MASK        (0x1f << 0)

#define R_APB0_CLK_REG                   (DEV_R_PRCM + 0x000c)
#define R_APB0_CLK_REG_DIV_M(x)          ((x) << 0)
#define R_APB0_CLK_REG_DIV_M_MASK        (0x3 << 0)

/* Read by the blob at 0x59e4 (bits 31..24); TF-A/kernel do not name it. */
#define R_APB2_CLK_REG                   (DEV_R_PRCM + 0x0010)

/* boot0 "prcm cpus timer clock enable": bit 0 of these three (driver 0091). */
#define R_CPUS_TIMER_GATE_REG(n)         (DEV_R_PRCM + 0x0110 + 4 * (n))

#define R_TIMER_GATE_REG                 (DEV_R_PRCM + 0x011c)
#define R_HDMI_AUX_GATE_REG              (DEV_R_PRCM + 0x0120) /* blob 0x120f0: 0, 5 ms, 0x10001 */
#define R_HDMI_AUX_CLK_REG               (DEV_R_PRCM + 0x0124) /* blob 0x120f0: bit 31 */
#define R_TWDOG_GATE_REG                 (DEV_R_PRCM + 0x012c)
#define R_PWM_CLK_REG                    (DEV_R_PRCM + 0x0130)
#define R_PWM_GATE_REG                   (DEV_R_PRCM + 0x013c)
#define R_UART_GATE_REG                  (DEV_R_PRCM + 0x018c)
#define R_I2C_GATE_REG                   (DEV_R_PRCM + 0x019c)
#define R_PPU_GATE_REG                   (DEV_R_PRCM + 0x01ac)
#define R_CIR_RX_CLK_REG                 (DEV_R_PRCM + 0x01c0) /* blob 0x8f14 */
#define R_CIR_RX_GATE_REG                (DEV_R_PRCM + 0x01cc)
#define R_RTC_GATE_REG                   (DEV_R_PRCM + 0x020c)
#define R_CPUCFG_GATE_REG                (DEV_R_PRCM + 0x022c)

/* blob 0xdd20: key 0xa7 << 24 written twice (H6 PLL_CTRL_REG1 shape). */
#define PLL_CTRL_REG1                    (DEV_R_PRCM + 0x0244)
#define PLL_CTRL_REG1_KEY                (0xa7 << 24)
#define PLL_CTRL_REG1_KEY_FIELD          (0xff << 24)
#define PLL_CTRL_REG1_PLL_LDO_OUT(x)     ((x) << 16)
#define PLL_CTRL_REG1_PLL_LDO_OUT_MASK   (0x7 << 16)
#define PLL_CTRL_REG1_PLL_IN_PWR_SEL     BIT(15)
#define PLL_CTRL_REG1_CLKTEST_EN         BIT(3)
#define PLL_CTRL_REG1_CRYSTAL_EN         BIT(2)
#define PLL_CTRL_REG1_LDO_EN             BIT(0)

/* blob 0xdea8: |= 4 and |= 0x100 on the way down (H6 names). */
#define VDD_SYS_PWROFF_GATING_REG        (DEV_R_PRCM + 0x0250)
#define VDD_CPUS_GATING                  BIT(3)
#define AVCC_GATING                      BIT(2)
#define UNK_GATING_BIT_08                BIT(8)

#define GPU_PWROFF_GATING_REG            (DEV_R_PRCM + 0x0254) /* blob touches it */

/* blob 0xdea8: &= ~1 on the way down. */
#define VDD_SYS_RESET_REG                (DEV_R_PRCM + 0x0260)
#define VDD_SYS_RESET                    BIT(0)

#endif /* PLATFORM_PRCM_H */
