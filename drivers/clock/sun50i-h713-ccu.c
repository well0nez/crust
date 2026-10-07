/*
 * Copyright © 2017-2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#include <bitmap.h>
#include <clock.h>
#include <device.h>
#include <stdbool.h>
#include <stdint.h>
#include <clock/ccu.h>
#include <delay.h>
#include <mmio.h>
#include <util.h>
#include <platform/devices.h>
#include <platform/memory.h>
#include <platform/prcm.h>

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

#if CONFIG(H713_DEEP_SLEEP)
static uint32_t mbus_saved;

/* Before each write of the rebuild: SUBSTEP (platform/memory.h, DEEP-07). */
static void
mbus_step(uint8_t i)
{
	mmio_write_32(H713_K_SUBSTEP, H713_SUB(H713_SUB_MBUS, i));
}

/*
 * The main-domain fingerprint (platform/memory.h, X): read before the deep
 * path and after it, so the kernel can name a word it changed (PRUEFUNG5
 * design A). CCU msgbox and UART gates, MBUS, the display bus, the UART0
 * pins (PH CFG0, 0x30 a bank), the SoC watchdog's MODE, R_PRCM 0x250 and
 * 0x260, and the status of the five PPU domains (kernel sun50i-h713-ppu.c),
 * those only while the PPU's bus gate and reset (R_PRCM 0x1ac) are open,
 * else 0xffffffff. Reads only.
 */
#define PPU_STATUS(d)   (0x07001024 + 0x80 * (d))
#define R_PPU_BUS       (DEV_R_PRCM + 0x1ac)
#define FP_PPU          8       /* the first PPU word */
static const uintptr_t fp_reg[H713_X_FP_WORDS] = {
	DEV_CCU + 0x71c, DEV_CCU + 0x90c, DEV_CCU + 0x540, DEV_CCU + 0xdd8,
	DEV_PIO + 0x150, DEV_WDOG + 0x14, VDD_SYS_PWROFF_GATING_REG,
	VDD_SYS_RESET_REG, PPU_STATUS(0), PPU_STATUS(1), PPU_STATUS(2),
	PPU_STATUS(3), PPU_STATUS(4),
};

static void
fingerprint(uint8_t after)
{
	uintptr_t to = after ? H713_X_FP_AFTER : H713_X_FP_BEFORE;
	uint32_t  cycle = mmio_read_32(H713_K_CYCLE) & 0xff;
	bool      ppu = (mmio_read_32(R_PPU_BUS) & 0x10001) == 0x10001;

	mmio_write_32(H713_K_SUBSTEP, H713_SUB(H713_SUB_FP, after));
	for (uint8_t i = 0; i < ARRAY_SIZE(fp_reg); ++i) {
		bool skip = !ppu && i >= FP_PPU;

		mmio_write_32(to + 4 * i,
		              skip ? 0xffffffff : mmio_read_32(fp_reg[i]));
	}
	mmio_clrset_32(H713_X_FP_CYCLE, 0xff << 8 * after, cycle << 8 * after);
}

/* 0xe164: the blob keeps MBUS (0x540) and PLL_PERIPH0 before the DRAM goes. */
void __attribute__((noinline))
ccu_suspend(void)
{
	fingerprint(0);
	mbus_saved = mmio_read_32(DEV_CCU + 0x540);
}

/*
 * 0xe3f4 (MBUS and the NSI bus back after the PLLs) and the start of the
 * blob's "power-up dram" step 37005 (MBUS from the saved value), both
 * before the DRAM leaves self-refresh.
 */
void __attribute__((noinline))
ccu_resume(void)
{
	mbus_step(1);
	mmio_write_32(DEV_CCU + 0x540, BIT(31));                /* 0xe40c */
	udelay(1);
	mbus_step(2);
	mmio_clr_32(DEV_CCU + 0x540, BIT(30));                  /* 0xe430 */
	udelay(1);
	mbus_step(3);
	mmio_set_32(DEV_CCU + 0xdd8, BIT(0));                   /* 0xe444 */
	udelay(1);
	mbus_step(4);
	mmio_set_32(DEV_CCU + 0xdd8, BIT(16));                  /* 0xe45c */
	udelay(1);
	mbus_step(5);
	mmio_set_32(DEV_TVTOP + 0x40, BIT(4));                  /* 0xe478 */
	udelay(1);
	mbus_step(6);
	mmio_set_32(DEV_TVTOP + 0x44, BIT(4));                  /* 0xe494 */
	udelay(1);
	mbus_step(7);
	mmio_set_32(DEV_TVTOP + 0x80, BIT(4));                  /* 0xe4a8 */
	udelay(1);
	mbus_step(8);
	mmio_set_32(DEV_CCU + 0x540, BIT(1));                   /* 0xe4bc */
	udelay(1);
	mbus_step(9);
	mmio_set_32(DEV_CCU + 0x540, 0x03000000);               /* 0xe4d4 */
	udelay(1);
	mbus_step(10);
	mmio_set_32(DEV_CCU + 0x540, BIT(30));                  /* 0xe4ec */
	udelay(1);
	/* 37005, 0xe6e0 */
	mbus_step(11);
	mmio_write_32(DEV_CCU + 0x540, mbus_saved & 0x7);
	udelay(200);
	mbus_step(12);
	mmio_write_32(DEV_CCU + 0x540, mbus_saved & 0x03000007);
	udelay(20);
	mbus_step(13);
	mmio_set_32(DEV_CCU + 0x540, BIT(31));
	udelay(20);
	mbus_step(14);
	mmio_set_32(DEV_CCU + 0x540, BIT(30));
	udelay(10000);
	fingerprint(1);
}
#else
/* Light standby: the bus clocks of the main domain are not touched. */
void
ccu_suspend(void)
{
}
#endif

void
ccu_suspend_cluster(uint32_t cluster UNUSED)
{
}

#if !CONFIG(H713_DEEP_SLEEP)
void
ccu_resume(void)
{
}
#endif

void
ccu_resume_cluster(uint32_t cluster UNUSED)
{
}

void
ccu_init(void)
{
}
