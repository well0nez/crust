/*
 * Copyright © 2017-2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#include <bitfield.h>
#include <bitmap.h>
#include <clock.h>
#include <counter.h>
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
#include <platform/time.h>

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
 *
 * The LOSC steps (0xdd50..0xdd9c down, 0xde24..0xde68 up) are not ported:
 * the HY310 runs LOSC from the internal RC with the crystal off (LOSC_CTRL
 * 0x8000, 07.10.2026; the DT rtc node has no clocks), nothing in the sleep
 * needs it changed, and a dead LOSC would stop the RTC and the EINT sampler
 * of the power key and IR edges (PRUEFUNG5 deep-clocks). Upstream crust
 * never touches the RTC in a suspend either. LOSC_CTRL is only read, after
 * the resume, for the device test.
 */
#define RTC_LOSC_CTRL   (DEV_RTC + 0x000)
#define LOSC_SEEN       H713_SEEN_LOSC

/*
 * 0xcdd8's table 0x17124: the clocks it moves to source 0, divider 1. The
 * blob saves M 1..0, P 9..8 and the source; R_PRCM 0x000/0x00c/0x010 have
 * a 5-bit M (the kernel's D1 model), so bits 4..2 are saved too (PRUEFUNG4
 * clocks F8).
 */
#define ROOT_DIV        0x0000031f
#define ROOT_SRC        0x07000000
static const uintptr_t root_clk[8] = {
	DEV_CCU + 0x500, DEV_CCU + 0x510, DEV_CCU + 0x520, DEV_CCU + 0x524,
	DEV_R_PRCM + 0x124, DEV_R_PRCM + 0x000, DEV_R_PRCM + 0x00c,
	DEV_R_PRCM + 0x010,
};
static uint32_t root_saved[8];

/* 0xcc2c's table 0x170b4: the PLLs. */
static const uint16_t pll_reg[12] = {
	0x000, 0x010, 0x020, 0x028, 0x030, 0x040, 0x048, 0x050, 0x058, 0x060,
	0x068, 0x078,
};
static uint32_t pll_saved[12];

/*
 * The way back: PLL_PERIPH0 (the buses) first, then PLL_CPUX, then the rest
 * (PRUEFUNG5 deep-clocks). A root clock on source 3 runs from PLL_CPUX
 * (0x500) or PLL_PERIPH0 (the others, kernel ccu-sun50i-h713.c, D1 model).
 * The deadline of one lock is the kernel's (ccu_helper_wait_for_lock); the
 * H713's lock time is not measured - X_LOCK_TICKS records it.
 */
#define PLL_EN          BIT(31)
#define PLL_LOCK_EN     BIT(29)
#define PLL_LOCK        BIT(28)
#define PLL_CPUX        0
#define PLL_PERIPH0     2
#define PLL_LOCK_MS     70
static const uint8_t pll_order[12] = { 2, 0, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

/*
 * root_saved and pll_saved hold this sleep's state: set once both are
 * saved, cleared by the restore. In .bss, so a firmware restart clears it:
 * the blob's per-root flags (0xcec0/0xce6c) left the PLLs out, and a
 * restart in the sleep put them back from zeros, N 0 and EN 0 (PRUEFUNG4
 * evidence F8). Without it nothing is restored, and the state machine goes
 * on to the reset.
 */
static bool saved;

/* Before each step of the deep path: what and which (platform/memory.h). */
static void
substep(uint8_t step, uint8_t i)
{
	mmio_write_32(H713_K_SUBSTEP, H713_SUB(step, i));
}

static void
root_clocks_to_osc24m(void)                             /* 0xcdd8(0) */
{
	for (uint8_t i = 0; i < ARRAY_SIZE(root_clk); ++i) {
		uint32_t v;

		substep(H713_SUB_ROOT_OSC, i);
		v = mmio_read_32(root_clk[i]);
		root_saved[i] = v & (ROOT_SRC | ROOT_DIV);      /* 0xcec0 */
		v &= ~ROOT_SRC;                                 /* source 0 */
		mmio_write_32(root_clk[i], v);                  /* 0xcefc */
		mmio_write_32(root_clk[i], v & 0xfffffc00);     /* 0xcf0c */
		udelay(10000);                                  /* 0xcf14 */
	}
}

/*
 * The blob's order, divider first, then the source (0xce6c); each field is
 * replaced, not ORed into what the register reads, so a divider the way
 * down did not clear (a domain reset, a restart) cannot merge with the
 * saved one (PRUEFUNG5 DEEP-04). A root whose PLL did not lock stays on
 * OSC24M, divider 1.
 */
static void
root_clocks_restore(uint32_t no_lock)                   /* 0xcdd8(4) */
{
	for (uint8_t i = 0; i < ARRAY_SIZE(root_clk); ++i) {
		uint8_t  pll = i ? PLL_PERIPH0 : PLL_CPUX;
		uint32_t v;

		substep(H713_SUB_ROOT_BACK, i);
		if ((root_saved[i] >> 24 & 3) == 3 && (no_lock & BIT(pll))) {
			mmio_set_32(H713_K_DEEP_LOCK, BIT(H713_LOCK_ROOTS + i));
			continue;
		}
		v = mmio_read_32(root_clk[i]);
		v = (v & ~ROOT_DIV) | (root_saved[i] & ROOT_DIV);
		mmio_write_32(root_clk[i], v);
		v = (v & ~ROOT_SRC) | (root_saved[i] & ROOT_SRC);
		mmio_write_32(root_clk[i], v);
		udelay(10000);
	}
}

static void
plls_off(void)                                          /* 0xcc2c(0) */
{
	for (uint8_t i = 0; i < ARRAY_SIZE(pll_reg); ++i) {
		uint32_t v;

		substep(H713_SUB_PLL_OFF, i);
		v = mmio_read_32(DEV_CCU + pll_reg[i]);
		pll_saved[i] = v & 0xa000ff03;   /* enable, lock-en, N, M */
		mmio_write_32(DEV_CCU + pll_reg[i], v & 0x7fffffff);
	}
	saved = true;
}

/*
 * 0xcc2c(1) in pll_order, and what the blob leaves out: a PLL that ran
 * before the sleep gets its lock detector restarted (lock enable off and
 * on, as U-Boot and dram.c do) and its deadline, then U-Boot's 20 us.
 * Returns the PLLs that did not lock.
 */
static uint32_t
plls_restore(void)                                      /* 0xcc2c(1) */
{
	uint32_t no_lock = 0, longest = 0;

	for (uint8_t k = 0; k < ARRAY_SIZE(pll_order); ++k) {
		uint8_t   i   = pll_order[k];
		uintptr_t reg = DEV_CCU + pll_reg[i];
		uint32_t  start, t, v;

		substep(H713_SUB_PLL_ON, i);
		v = mmio_read_32(reg);
		mmio_write_32(reg, (v & 0x5fff00fc) | pll_saved[i]);
		if (!(pll_saved[i] & PLL_EN))
			continue;
		mmio_clr_32(reg, PLL_LOCK_EN);
		mmio_set_32(reg, PLL_LOCK_EN);
		start = cycle_counter_read();
		do {
			v = mmio_read_32(reg);
			t = cycle_counter_read() - start;
		} while (!(v & PLL_LOCK) && t < PLL_LOCK_MS * CPUCLK_kHz);
		if (!(v & PLL_LOCK))
			no_lock |= BIT(i);
		if (t > (longest & 0xffffff))
			longest = (uint32_t)i << 24 | t;
		if (!(pll_saved[i] & PLL_LOCK_EN))
			mmio_clr_32(reg, PLL_LOCK_EN);
		udelay(20);
	}
	mmio_set_32(H713_K_DEEP_LOCK, no_lock);
	mmio_write_32(H713_X_LOCK_TICKS, longest);

	return no_lock;
}

#if CONFIG(H713_DEEP_PLL_LDO)
/*
 * 0xddbc (off) and 0xddec (on), the blob's two keyed writes. The key field
 * is cleared before the key goes in, as upstream's write_pll_ctrl_reg1()
 * does, so a key left in it cannot mix with ours; CRYSTAL_EN and the LDO
 * output stay as they are (PRUEFUNG5 deep-clocks D3).
 */
static void
pll_ldo(bool on)
{
	uint32_t v = mmio_read_32(PLL_CTRL_REG1) &
	             ~(PLL_CTRL_REG1_KEY_FIELD | PLL_CTRL_REG1_LDO_EN);

	v |= PLL_CTRL_REG1_KEY | (on ? PLL_CTRL_REG1_LDO_EN : 0);
	mmio_write_32(PLL_CTRL_REG1, v);
	mmio_write_32(PLL_CTRL_REG1, v);
	udelay(100);
}

/* The LDO feeds the PLLs: it goes off only when every one reads off. */
static bool
plls_all_off(void)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(pll_reg); ++i) {
		if (mmio_read_32(DEV_CCU + pll_reg[i]) & PLL_EN)
			return false;
	}

	return true;
}
#endif

void __attribute__((noinline))
r_ccu_suspend(uint8_t depth UNUSED)
{
	mmio_write_32(LOSC_SEEN, 0);
	mmio_write_32(H713_K_DEEP_LOCK, 0);
	root_clocks_to_osc24m();                                /* 0xdd30 */
	udelay(100);
	plls_off();                                             /* 0xdd40 */
	udelay(100);
#if CONFIG(H713_DEEP_PLL_LDO)
	substep(H713_SUB_LDO_OFF, 0);
	if (plls_all_off())
		pll_ldo(false);                                 /* 0xddbc */
	else
		mmio_set_32(H713_K_DEEP_LOCK, BIT(H713_LOCK_LDO_KEPT));
#endif
#if CONFIG(H713_DEEP_VDD_SYS)
	/*
	 * 0xdea8. When the blob runs this pair is not settled: the RE note
	 * doku/nachtlog/S39:92 gives the condition of 0xe3a8 as "power_down
	 * != 0 or wake mask bit 13 clear", so with the stock defaults
	 * (power_down 0, mask 0x2000) neither step would run - an earlier
	 * comment here said the blob skips it for power_down. Not checked
	 * against the disassembly (PRUEFUNG5 DEEP-02).
	 */
	substep(H713_SUB_DOMAIN_OFF, 0);
	mmio_clr_32(VDD_SYS_RESET_REG, VDD_SYS_RESET);
#endif
#if CONFIG(H713_DEEP_AVCC_GATING)
	/* 0xdea8's gating, on its own switch (PRUEFUNG5 DEEP-09) */
	substep(H713_SUB_DOMAIN_OFF, 1);
	mmio_set_32(VDD_SYS_PWROFF_GATING_REG, AVCC_GATING);
	mmio_set_32(VDD_SYS_PWROFF_GATING_REG, UNK_GATING_BIT_08);
#endif
}

void __attribute__((noinline))
r_ccu_resume(void)
{
	/* 0xe3a8 */
#if CONFIG(H713_DEEP_AVCC_GATING)
	substep(H713_SUB_DOMAIN_ON, 0);
	mmio_clr_32(VDD_SYS_PWROFF_GATING_REG, UNK_GATING_BIT_08);
	mmio_clr_32(VDD_SYS_PWROFF_GATING_REG, AVCC_GATING);
#endif
#if CONFIG(H713_DEEP_VDD_SYS)
	substep(H713_SUB_DOMAIN_ON, 1);
	mmio_set_32(VDD_SYS_RESET_REG, VDD_SYS_RESET);
#endif
#if CONFIG(H713_DEEP_PLL_LDO)
	substep(H713_SUB_LDO_ON, 0);
	pll_ldo(true);                                          /* 0xddec */
#endif
	mmio_write_32(LOSC_SEEN, mmio_read_32(RTC_LOSC_CTRL));
	if (saved) {
		uint32_t no_lock = plls_restore();              /* 0xde7c */
		uint8_t  i = 0;

		udelay(100);
		root_clocks_restore(no_lock);                   /* 0xde8c */
		saved = false;
		/* A PLL Linux ran is dead: the clean reset (cause 9). */
		if (no_lock) {
			while (!(no_lock & BIT(i)))
				++i;
			h713_fail(H713_CAUSE_PLL_LOCK, i,
			          mmio_read_32(DEV_CCU + pll_reg[i]));
		}
	}
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
