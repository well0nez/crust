/*
 * Copyright © 2017-2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#include <bitfield.h>
#include <bitmap.h>
#include <clock.h>
#include <debug.h>
#include <delay.h>
#include <device.h>
#include <mmio.h>
#include <stddef.h>
#include <stdint.h>
#include <clock/ccu.h>
#include <hdmi/sun50i-h713-hdmi.h>
#include <platform/devices.h>
#include <platform/memory.h>
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
 * CPUS clock register as the boot loader (or the vendor firmware before us)
 * left it: 0x03000002, PLL_PERIPH0 / 3, when crust is loaded over the blob.
 * Kept only for the diagnostic word; persists across firmware restarts
 * (.data), and the linker checks the size of .data.
 */
static uint32_t cpus_clk_at_boot = 0xffffffff;

uint32_t
r_ccu_cpus_clk_at_boot(void)
{
	return cpus_clk_at_boot;
}

/*
 * The one clock this firmware runs on: OSC24M / 1. Every delay and timeout
 * counts the tick timer at CPUCLK_MHz (24), so the AR100 must be back on it
 * after a resume, whatever it ran on before crust started - handing the
 * boot value back would run every later wait 2.8x to 8.3x short.
 */
static void
cpus_clk_osc24m(void)
{
	mmio_write_32(CPUS_CLK_REG,
	              CPUS_CLK_REG_CLK_SRC(0) |
	              CPUS_CLK_REG_PRE_DIV(0) |
	              CPUS_CLK_REG_DIV_P(0));
}

#if CONFIG(H713_DEEP_SLEEP)
/*
 * Deep sleep, ported from the vendor ARISC firmware (blob addresses):
 * 0xdd20 on the way down, after the DRAM is in self-refresh and the cluster
 * is reset - bus and root clocks to OSC24M (0xcdd8 mode 0), every PLL off
 * (0xcc2c mode 0), the RTC's LOSC to the internal oscillator and the 32k
 * crystal off, the PLL LDO off - and 0xdea8 (VDD_SYS reset, AVCC and bit 8
 * gating); 0xe3a8 and 0xddec on the way back. 0x6d04(n) is the blob's
 * microsecond busy wait, 0x6b68(n) its millisecond timer wait.
 */
#define RTC_LOSC_CTRL   (DEV_RTC + 0x000)
#define RTC_LOSC_OUT    (DEV_RTC + 0x060)
#define KEY_16AA        0x16aa0000
#define LOSC_EXT_EN     BIT(4)  /* the 32k crystal's oscillator on */
#define LOSC_EXT_SEL    BIT(0)  /* LOSC from the crystal, not the RC */
/* LOSC_CTRL after the resume, for the device test (0033; behind the
 * power key words of sun50i-h713-hdmi.c, +0x24c..+0x25b). */
#define LOSC_SEEN       (H713_SHARED_BASE + 0x25c)

/*
 * What Linux had before the sleep. The blob turns the 32k crystal on and
 * selects it on the way back; on the HY310 Linux never did (07.10.2026:
 * LOSC_CTRL 0x8000 - the internal RC, the crystal off; the DT rtc node has
 * no clocks), and nobody knows whether a crystal is fitted. A dead LOSC
 * would stop the RTC and the EINT sampler that clocks the power key and
 * IR edges. So the resume puts back what the suspend found.
 */
static uint32_t losc_saved, losc_out_saved;

/* 0xcdd8's table 0x17124: the clocks it moves to source 0, divider 1. */
static const uintptr_t root_clk[8] = {
	DEV_CCU + 0x500, DEV_CCU + 0x510, DEV_CCU + 0x520, DEV_CCU + 0x524,
	DEV_R_PRCM + 0x124, DEV_R_PRCM + 0x000, DEV_R_PRCM + 0x00c,
	DEV_R_PRCM + 0x010,
};
static uint32_t root_saved[8];
static bool     root_valid[8];

/* 0xcc2c's table 0x170b4: the PLLs. */
static const uint16_t pll_reg[12] = {
	0x000, 0x010, 0x020, 0x028, 0x030, 0x040, 0x048, 0x050, 0x058, 0x060,
	0x068, 0x078,
};
static uint32_t pll_saved[12];

static void
root_clocks_to_osc24m(void)                             /* 0xcdd8(0) */
{
	for (uint8_t i = 0; i < ARRAY_SIZE(root_clk); ++i) {
		uint32_t v = mmio_read_32(root_clk[i]);

		if (!root_valid[i]) {                           /* 0xcec0 */
			root_saved[i] = v & 0x07000303;
			root_valid[i] = true;
		}
		v &= 0xf8ffffff;                                /* source 0 */
		mmio_write_32(root_clk[i], v);                  /* 0xcefc */
		mmio_write_32(root_clk[i], v & 0xfffffc00);     /* 0xcf0c */
		udelay(10000);                                  /* 0xcf14 */
	}
}

static void
root_clocks_restore(void)                               /* 0xcdd8(4) */
{
	for (uint8_t i = 0; i < ARRAY_SIZE(root_clk); ++i) {
		uint32_t v = mmio_read_32(root_clk[i]);

		if (root_valid[i]) {                            /* 0xce6c */
			v |= root_saved[i] & 0x00000303;
			mmio_write_32(root_clk[i], v);
			v = (v & 0xf8ffffff) | (root_saved[i] & 0x07000000);
			mmio_write_32(root_clk[i], v);
			root_valid[i] = false;
		}
		udelay(10000);
	}
}

static void
plls_off(void)                                          /* 0xcc2c(0) */
{
	for (uint8_t i = 0; i < ARRAY_SIZE(pll_reg); ++i) {
		uint32_t v = mmio_read_32(DEV_CCU + pll_reg[i]);

		pll_saved[i] = v & 0xa000ff03;   /* enable, lock-en, N, M */
		mmio_write_32(DEV_CCU + pll_reg[i], v & 0x7fffffff);
	}
}

static void
plls_restore(void)                                      /* 0xcc2c(1) */
{
	for (uint8_t i = 0; i < ARRAY_SIZE(pll_reg); ++i) {
		uint32_t v = mmio_read_32(DEV_CCU + pll_reg[i]);

		mmio_write_32(DEV_CCU + pll_reg[i],
		              (v & 0x5fff00fc) | pll_saved[i]);
	}
}

void __attribute__((noinline))
r_ccu_suspend(uint8_t depth UNUSED)
{
	uint32_t v;

	root_clocks_to_osc24m();                                /* 0xdd30 */
	udelay(100);
	plls_off();                                             /* 0xdd40 */
	udelay(100);
	losc_saved     = mmio_read_32(RTC_LOSC_CTRL);
	losc_out_saved = mmio_read_32(RTC_LOSC_OUT);
	v = (mmio_read_32(RTC_LOSC_CTRL) & ~BIT(0)) | KEY_16AA; /* 0xdd50 */
	mmio_write_32(RTC_LOSC_CTRL, v);
	mmio_write_32(RTC_LOSC_CTRL, v);
	udelay(1);
	v = (mmio_read_32(RTC_LOSC_CTRL) & ~BIT(4)) | KEY_16AA; /* 0xdd74 */
	mmio_write_32(RTC_LOSC_CTRL, v);
	mmio_write_32(RTC_LOSC_CTRL, v);
	udelay(1);
	mmio_clr_32(RTC_LOSC_OUT, BIT(0));                      /* 0xdd9c */
	udelay(100);
	v = (mmio_read_32(PLL_CTRL_REG1) & ~PLL_CTRL_REG1_LDO_EN) |
	    PLL_CTRL_REG1_KEY;                                  /* 0xddbc */
	mmio_write_32(PLL_CTRL_REG1, v);
	mmio_write_32(PLL_CTRL_REG1, v);
	udelay(100);
	/* 0xdea8 (the blob skips it for power_down or a USB wake source) */
	mmio_clr_32(VDD_SYS_RESET_REG, VDD_SYS_RESET);
	mmio_set_32(VDD_SYS_PWROFF_GATING_REG, AVCC_GATING);
	mmio_set_32(VDD_SYS_PWROFF_GATING_REG, UNK_GATING_BIT_08);
}

void __attribute__((noinline))
r_ccu_resume(void)
{
	uint32_t v;

	/* 0xe3a8 */
	mmio_clr_32(VDD_SYS_PWROFF_GATING_REG, UNK_GATING_BIT_08);
	mmio_clr_32(VDD_SYS_PWROFF_GATING_REG, AVCC_GATING);
	mmio_set_32(VDD_SYS_RESET_REG, VDD_SYS_RESET);
	/* 0xddec */
	v = mmio_read_32(PLL_CTRL_REG1) | PLL_CTRL_REG1_KEY |
	    PLL_CTRL_REG1_LDO_EN;
	mmio_write_32(PLL_CTRL_REG1, v);
	mmio_write_32(PLL_CTRL_REG1, v);
	udelay(100);
	/* 0xde24/0xde44/0xde68 turn the crystal on, select it and gate the
	 * 32k output on; here the state from before the sleep comes back
	 * instead (see losc_saved): first the oscillator, then the source.
	 * A board whose Linux runs LOSC from the crystal gets the blob's 1 us
	 * between the two - the crystal kept running only if the suspend left
	 * it on, which it does not; not measured, the HY310 has both bits 0. */
	v = (mmio_read_32(RTC_LOSC_CTRL) & ~LOSC_EXT_EN) |
	    (losc_saved & LOSC_EXT_EN) | KEY_16AA;
	mmio_write_32(RTC_LOSC_CTRL, v);
	mmio_write_32(RTC_LOSC_CTRL, v);
	udelay(1);
	v = (mmio_read_32(RTC_LOSC_CTRL) & ~LOSC_EXT_SEL) |
	    (losc_saved & LOSC_EXT_SEL) | KEY_16AA;
	mmio_write_32(RTC_LOSC_CTRL, v);
	mmio_write_32(RTC_LOSC_CTRL, v);
	udelay(1);
	mmio_clrset_32(RTC_LOSC_OUT, BIT(0), losc_out_saved & BIT(0));
	mmio_write_32(LOSC_SEEN, mmio_read_32(RTC_LOSC_CTRL));
	udelay(100);
	plls_restore();                                         /* 0xde7c */
	udelay(100);
	root_clocks_restore();                                  /* 0xde8c */
	udelay(100);
	cpus_clk_osc24m();      /* the blob's PLL setting is not ours */
}

#else /* !CONFIG(H713_DEEP_SLEEP) */

void
r_ccu_suspend(uint8_t depth)
{
	/*
	 * Light standby: PLLs, HOSC and the R domain stay as they are
	 * (H713_DEEP_SLEEP ports the blob's 0xdd20/0xdea8).
	 */
	(void)depth;
}

void
r_ccu_resume(void)
{
	cpus_clk_osc24m();      /* unchanged in a light standby; made sure */
}

#endif

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
	cpus_clk_osc24m();
}
