/*
 * Copyright © 2017-2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#include <bitfield.h>
#include <bitmap.h>
#include <clock.h>
#include <debug.h>
#include <device.h>
#include <mmio.h>
#include <stddef.h>
#include <stdint.h>
#include <clock/ccu.h>
#include <platform/devices.h>
#include <platform/prcm.h>

#include "ccu.h"

/*
 * R_PRCM of the H713 in the D1 layout (kernel ccu-sun20i-d1-r.c is the model
 * the device tree binds; the blob's gate helper 0x5320 names 0x11c, 0x1cc,
 * 0x20c, 0x260 and its EDID reset 0x120f0 names 0x120/0x124).
 */
static DEFINE_FIXED_RATE(r_ccu_get_osc24m_rate, 24000000U)
static DEFINE_FIXED_RATE(r_ccu_get_osc32k_rate, 32768U)

static const struct clock_handle r_ccu_bus_parents[] = {
	{ .dev = &r_ccu.dev, .id = CLK_OSC24M },
	{ .dev = &r_ccu.dev, .id = CLK_OSC32K },
	{ .dev = &r_ccu.dev, .id = CLK_IOSC },
	{ .dev = &ccu.dev, .id = CLK_PLL_PERIPH0 },
};

static const struct clock_handle *
r_ccu_get_bus_parent(const struct ccu *self, const struct ccu_clock *clk)
{
	uint32_t val = mmio_read_32(self->regs + clk->reg);

	return &r_ccu_bus_parents[bitfield_get(val, 24, 2)];
}

static uint32_t
r_ccu_get_mp_rate(const struct ccu *self,
                  const struct ccu_clock *clk, uint32_t rate)
{
	return ccu_helper_get_rate_mp(self, clk, rate, 0, 5, 8, 2);
}

static DEFINE_FIXED_PARENT(r_ccu_get_ar100, r_ccu, CLK_AR100)
static DEFINE_FIXED_PARENT(r_ccu_get_r_ahb, r_ccu, CLK_R_AHB)

static uint32_t
r_ccu_get_r_apb1_rate(const struct ccu *self,
                      const struct ccu_clock *clk, uint32_t rate)
{
	return ccu_helper_get_rate_m(self, clk, rate, 0, 2);
}

static DEFINE_FIXED_PARENT(r_ccu_get_r_apb1, r_ccu, CLK_R_APB1)

static const struct clock_handle r_ccu_module_parents[] = {
	{ .dev = &r_ccu.dev, .id = CLK_OSC32K },
	{ .dev = &r_ccu.dev, .id = CLK_OSC24M },
};

static const struct clock_handle *
r_ccu_get_module_parent(const struct ccu *self, const struct ccu_clock *clk)
{
	uint32_t val = mmio_read_32(self->regs + clk->reg);

	return &r_ccu_module_parents[bitfield_get(val, 24, 1)];
}

#define R_APB1_GATE(_reg) \
	.get_parent = r_ccu_get_r_apb1, .get_rate = ccu_get_parent_rate, \
	.gate = BITMAP_INDEX(_reg, 0), .reset = BITMAP_INDEX(_reg, 16)

static const struct ccu_clock r_ccu_clocks[SUN50I_H713_R_CCU_CLOCKS] = {
	[CLK_IOSC] = {
		.get_parent = ccu_get_null_parent,
		.get_rate   = r_ccu_common_get_iosc_rate,
	},
	[CLK_OSC24M] = {
		.get_parent = ccu_get_null_parent,
		.get_rate   = r_ccu_get_osc24m_rate,
	},
	[CLK_OSC32K] = {
		.get_parent = ccu_get_null_parent,
		.get_rate   = r_ccu_get_osc32k_rate,
	},
	[CLK_AR100] = {
		.get_parent = r_ccu_get_bus_parent,
		.get_rate   = r_ccu_get_mp_rate,
		.reg        = 0x0000,
	},
	[CLK_R_AHB] = {
		.get_parent = r_ccu_get_ar100,
		.get_rate   = ccu_get_parent_rate,
	},
	[CLK_R_APB1] = {
		.get_parent = r_ccu_get_r_ahb,
		.get_rate   = r_ccu_get_r_apb1_rate,
		.reg        = 0x000c,
	},
	[CLK_R_APB2] = {
		.get_parent = r_ccu_get_bus_parent,
		.get_rate   = r_ccu_get_mp_rate,
		.reg        = 0x0010,
	},
	[CLK_BUS_R_PIO] = {
		.get_parent = r_ccu_get_r_apb1,
		.get_rate   = ccu_get_parent_rate,
	},
	[CLK_BUS_R_TIMER]    = { R_APB1_GATE(0x011c) },
	[CLK_BUS_R_TWD]      = { R_APB1_GATE(0x012c) },
	[CLK_BUS_R_PWM]      = { R_APB1_GATE(0x013c) },
	[CLK_BUS_R_UART]     = { R_APB1_GATE(0x018c) },
	[CLK_BUS_R_I2C]      = { R_APB1_GATE(0x019c) },
	[CLK_BUS_R_RSB] = {
		.get_parent = r_ccu_get_r_apb1,
		.get_rate   = ccu_get_parent_rate,
	},
	[CLK_BUS_R_CIR]      = { R_APB1_GATE(0x01cc) },
	[CLK_R_CIR] = {
		.get_parent = r_ccu_get_module_parent,
		.get_rate   = r_ccu_get_mp_rate,
		.reg        = 0x01c0,
		.gate       = BITMAP_INDEX(0x01c0, 31),
	},
	[CLK_BUS_R_HDMI_AUX] = { R_APB1_GATE(0x0120) },
	[CLK_R_HDMI_AUX] = {
		.get_parent = r_ccu_get_module_parent,
		.get_rate   = ccu_get_parent_rate,
		.reg        = 0x0124,
		.gate       = BITMAP_INDEX(0x0124, 31),
	},
};

const struct ccu r_ccu = {
	.dev = {
		.name  = "r_ccu",
		.drv   = &ccu_driver.drv,
		.state = CLOCK_DEVICE_STATE_INIT(SUN50I_H713_R_CCU_CLOCKS),
	},
	.clocks = r_ccu_clocks,
	.regs   = DEV_R_PRCM,
};

/*
 * CPUS clock register as the boot loader left it. Persists across firmware
 * restarts (.data), so a restart after an exception can still hand the
 * original value back at resume; the linker checks the size of .data.
 */
static uint32_t cpus_clk_at_boot = 0xffffffff;

void
r_ccu_suspend(uint8_t depth)
{
	/*
	 * Light standby only: PLLs, HOSC and the R domain stay as they are.
	 * The blob's deeper steps (0xdd20: PLL_CTRL_REG1 key writes; 0xdea8:
	 * VDD_SYS_PWROFF_GATING |= 4, |= 0x100, VDD_SYS_RESET &= ~1) are
	 * written down in tables/arisc-contract.csv and wait for the DRAM
	 * self-refresh port (drivers/dram/sun50i-h713-dram.c).
	 */
	(void)depth;
}

void
r_ccu_resume(void)
{
	if (cpus_clk_at_boot != 0xffffffff)
		mmio_write_32(CPUS_CLK_REG, cpus_clk_at_boot);
}

void
r_ccu_init(void)
{
	/*
	 * CPUS on OSC24M/1: a fixed, precise clock for the tick timer, so no
	 * IOSC calibration against the system counter is needed (and none of
	 * the blob's 0x4f70 loop). boot0 left the three "cpus timer" gates
	 * 0x110/0x114/0x118 on; the kernel driver sets them again.
	 */
	if (cpus_clk_at_boot == 0xffffffff)
		cpus_clk_at_boot = mmio_read_32(CPUS_CLK_REG);
	mmio_write_32(CPUS_CLK_REG,
	              CPUS_CLK_REG_CLK_SRC(0) |
	              CPUS_CLK_REG_PRE_DIV(0) |
	              CPUS_CLK_REG_DIV_P(0));
}
