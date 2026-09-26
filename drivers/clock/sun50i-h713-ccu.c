/*
 * Copyright © 2017-2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#include <bitmap.h>
#include <clock.h>
#include <device.h>
#include <stdint.h>
#include <clock/ccu.h>
#include <platform/devices.h>

#include "ccu.h"

/*
 * Main CCU, the few clocks this firmware touches. Offsets from the kernel's
 * ccu-sun50i-h713.c (pll-ddr 0x010, mbus 0x540, bus-msgbox 0x71c, dram
 * 0x800, bus-dram 0x80c); the blob snapshots 0x2001020 (PLL_PERIPH0) and
 * 0x2001540 (DRAM clock) on the way into standby.
 */
static DEFINE_FIXED_RATE(ccu_get_pll_periph0_rate, 600000000U)
static DEFINE_FIXED_PARENT(ccu_get_pll_ddr0, ccu, CLK_PLL_DDR0)
static DEFINE_FIXED_PARENT(ccu_get_apb2_parent, r_ccu, CLK_OSC24M)

static const struct ccu_clock ccu_clocks[SUN50I_H713_CCU_CLOCKS] = {
	[CLK_PLL_DDR0] = {
		.get_parent = ccu_get_null_parent,
		.get_rate   = ccu_get_parent_rate,
		.reg        = 0x0010,
		.lock       = 28,
		.gate       = BITMAP_INDEX(0x0010, 31),
	},
	[CLK_PLL_PERIPH0] = {
		.get_parent = ccu_get_null_parent,
		.get_rate   = ccu_get_pll_periph0_rate,
	},
	[CLK_APB2] = {
		.get_parent = ccu_get_apb2_parent,
		.get_rate   = ccu_get_parent_rate,
	},
	[CLK_MBUS] = {
		.get_parent = ccu_get_null_parent,
		.get_rate   = ccu_get_parent_rate,
		.gate       = BITMAP_INDEX(0x0540, 31),
		.reset      = BITMAP_INDEX(0x0540, 30),
	},
	[CLK_BUS_MSGBOX] = {
		.get_parent = ccu_get_null_parent,
		.get_rate   = ccu_get_parent_rate,
		.gate       = BITMAP_INDEX(0x071c, 0),
		.reset      = BITMAP_INDEX(0x071c, 16),
	},
	[CLK_DRAM] = {
		.get_parent = ccu_get_pll_ddr0,
		.get_rate   = ccu_get_parent_rate,
		.reg        = 0x0800,
		.update     = 27,
		.reset      = BITMAP_INDEX(0x0800, 30),
	},
	[CLK_BUS_DRAM] = {
		.get_parent = ccu_get_null_parent,
		.get_rate   = ccu_get_parent_rate,
		.gate       = BITMAP_INDEX(0x080c, 0),
	},
	[CLK_BUS_PIO] = {
		.get_parent = ccu_get_null_parent,
		.get_rate   = ccu_get_parent_rate,
	},
};

const struct ccu ccu = {
	.dev = {
		.name  = "ccu",
		.drv   = &ccu_driver.drv,
		.state = CLOCK_DEVICE_STATE_INIT(SUN50I_H713_CCU_CLOCKS),
	},
	.clocks = ccu_clocks,
	.regs   = DEV_CCU,
};

/* Light standby: the bus clocks of the main domain are not touched yet. */
void
ccu_suspend(void)
{
}

void
ccu_suspend_cluster(uint32_t cluster UNUSED)
{
}

void
ccu_resume(void)
{
}

void
ccu_resume_cluster(uint32_t cluster UNUSED)
{
}

void
ccu_init(void)
{
}
