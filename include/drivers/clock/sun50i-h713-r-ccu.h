/*
 * Copyright © 2017-2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#ifndef DRIVERS_CLOCK_SUN50I_H713_R_CCU_H
#define DRIVERS_CLOCK_SUN50I_H713_R_CCU_H

enum {
	CLK_IOSC,
	CLK_OSC24M,
	CLK_OSC32K,
	CLK_AR100,
	CLK_R_AHB,
	CLK_R_APB1,        /* D1 name: r_apb0, R_PRCM 0x00c */
	CLK_R_APB2,        /* R_PRCM 0x010, read by the blob only */
	CLK_BUS_R_PIO,
	CLK_BUS_R_TIMER,
	CLK_BUS_R_TWD,
	CLK_BUS_R_PWM,
	CLK_BUS_R_UART,
	CLK_BUS_R_I2C,
	CLK_BUS_R_RSB,     /* unverified on this SoC: no gate wired */
	CLK_BUS_R_CIR,
	CLK_R_CIR,
	CLK_BUS_R_HDMI_AUX,
	CLK_R_HDMI_AUX,
	SUN50I_H713_R_CCU_CLOCKS
};

#endif /* DRIVERS_CLOCK_SUN50I_H713_R_CCU_H */
