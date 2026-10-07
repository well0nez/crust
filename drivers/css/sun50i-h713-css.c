/*
 * Copyright © 2017-2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#include <delay.h>
#include <mmio.h>
#include <scpi_protocol.h>
#include <stdbool.h>
#include <util.h>
#include <stdint.h>
#include <platform/cpucfg.h>
#include <platform/memory.h>

#include "css.h"

/*
 * The cluster down and up exactly as the vendor ARISC firmware does it
 * (blob addresses; FB-CRUST had approximated both with TF-A's cpu_on ladder):
 *   down 0x4b28: 0x4968(0) (C0_CPU_CTRL 3..0 &= ~7, CPUCFG +0 = 0),
 *        C0_PWRON_RESET = 0, nPORESET of CPU_UNK 3..0 cleared,
 *        CPU_SYS_RESET = 0, 1 ms, 0x5598(1, 1) (core 0 clamp on),
 *        0x43f0(0, 0, 0) (core 0 power switch 0xff, 30 us, until it reads 0xff)
 *   up   0x4cc4: 0x43f0(0, 0, 1) (the switch ladder fe f8 e0 c0 80 40 00),
 *        0x5598(1, 0) (clamp off), CPU_SYS_RESET = 1, C0_PWRON_RESET =
 *        nH_RST, 0x4768(0, 0) (C0_CPU_CTRL(0) bit 0 = 0), 0x4968(0),
 *        0x48cc(0) (CPUCFG +0x10 and C0_CPU_CTRL 3..0 bit 4 = 0), 0x485c(0)
 *        (CPUCFG +0x14 bit 0 = 0), 0x4968(1) (CPUCFG +0 = 0x13000100,
 *        C0_CPU_CTRL 0..3 |= 0x106), ALT_RVBAR (0x4c2c), AArch64 (0x4c6c)
 *   release 0x4be4: CPU_UNK(0) |= nPORESET, 0x4768(0, 3) (C0_CPU_CTRL(0)
 *        bit 0 = 1)
 * The blob works on core 0 only: Linux suspends from CPU0 with 1..3 off.
 */
#define CPUCFG_CLS0             (DEV_CPUCFG + 0x0000)
#define CPUCFG_10               (DEV_CPUCFG + 0x0010)
#define CPUCFG_14               (DEV_CPUCFG + 0x0014)

static uint32_t warm_entry_lo, warm_entry_hi;

/*
 * The cluster down resets the CPU subsystem (CPU_SYS_RESET = 0), and with it,
 * as far as the HY310 shows (07.10.2026), the cores' start addresses: after
 * the first standby CPU1..3 did not come back ("failed to come online") and
 * the system stopped. TF-A programs ALT_RVBAR for all cores once at boot
 * (sunxi_pm.c plat_setup_psci_ops) and its cpu_on does not do it again; the
 * blob restores only core 0, since the vendor BL31 does the rest. So the
 * four start addresses are saved before the cluster goes down and written
 * back after it is up again. What the cluster up left in them is kept for
 * the record at H713_SEEN_RVBAR (cores 1..3, low words), so the next run
 * shows whether the reset cleared them.
 */
#define RVBAR_SEEN_BASE         H713_SEEN_RVBAR
static uint32_t rvbar_lo[MAX_CORES_PER_CLUSTER], rvbar_hi[MAX_CORES_PER_CLUSTER];

void
css_set_warm_entry(uint32_t lo, uint32_t hi)
{
	warm_entry_lo = lo;
	warm_entry_hi = hi;
}

/* 0x4968: the cluster control word down (mode 0) or up (mode 1). */
static void
cluster_ctrl(bool up)
{
	if (!up) {
		for (int n = MAX_CORES_PER_CLUSTER - 1; n >= 0; --n)
			mmio_clr_32(C0_CPU_CTRL_REG(n), 0x7);
		mmio_write_32(CPUCFG_CLS0, 0);
	} else {
		mmio_write_32(CPUCFG_CLS0, 0x13000100);
		for (uint32_t n = 0; n < MAX_CORES_PER_CLUSTER; ++n)
			mmio_set_32(C0_CPU_CTRL_REG(n), 0x106);
	}
}

void __attribute__((noipa))
css_suspend_css(uint32_t new_state)
{
	if (new_state < SCPI_CSS_OFF)
		return;

	for (uint32_t n = 0; n < MAX_CORES_PER_CLUSTER; ++n) {
		rvbar_lo[n] = mmio_read_32(ALT_RVBAR_LO_REG(n));
		rvbar_hi[n] = mmio_read_32(ALT_RVBAR_HI_REG(n));
	}
	cluster_ctrl(false);                                    /* 0x4b38 */
	mmio_write_32(C0_PWRON_RESET_REG, 0);                   /* 0x4b50 */
	for (int n = MAX_CORES_PER_CLUSTER - 1; n >= 0; --n)   /* 0x4b5c.. */
		mmio_clr_32(CPU_UNK_REG(n), CPU_UNK_REG_nPORESET);
	mmio_write_32(CPU_SYS_RESET_REG, 0);                    /* 0x4ba0 */
	udelay(1000);                                           /* 0x4ba4 */
	mmio_set_32(CPU_UNK_REG(0), CPU_UNK_REG_CLAMP);         /* 0x5598(1, 1) */
	if (mmio_read_32(C0_CPUn_PWR_SWITCH_REG(0)) != 0xff) {  /* 0x43f0(0, 0, 0) */
		mmio_write_32(C0_CPUn_PWR_SWITCH_REG(0), 0xff);
		udelay(30);
		while (mmio_read_32(C0_CPUn_PWR_SWITCH_REG(0)) != 0xff) {
			/* 0x4528 */
		}
	}
}

void __attribute__((noipa))
css_resume_css(uint32_t old_state)
{
	static const uint8_t ladder[] = { 0xfe, 0xf8, 0xe0, 0xc0, 0x80, 0x40 };
	static const uint8_t wait_us[] = { 20, 10, 10, 10, 10, 10 };

	if (old_state < SCPI_CSS_OFF)
		return;

	/* 0x43f0(0, 0, 1): the core 0 power switch, unless it is on */
	if (mmio_read_32(C0_CPUn_PWR_SWITCH_REG(0)) != 0) {
		for (uint32_t i = 0; i < ARRAY_SIZE(ladder); ++i) {
			mmio_write_32(C0_CPUn_PWR_SWITCH_REG(0), ladder[i]);
			udelay(wait_us[i]);
		}
		mmio_write_32(C0_CPUn_PWR_SWITCH_REG(0), 0);
		udelay(10);                                     /* 0x44ac */
		while (mmio_read_32(C0_CPUn_PWR_SWITCH_REG(0)) != 0)
			udelay(20);
		while (mmio_read_32(C0_CPUn_PWR_SWITCH_REG(0)) != 0) {
			/* 0x44c4 */
		}
	}
	mmio_clr_32(CPU_UNK_REG(0), CPU_UNK_REG_CLAMP);         /* 0x5598(1, 0) */
	mmio_write_32(CPU_SYS_RESET_REG, CPU_SYS_RESET);        /* 0x4d14 */
	mmio_write_32(C0_PWRON_RESET_REG, C0_PWRON_RESET_REG_nH_RST); /* 0x4d1c */
	mmio_clr_32(C0_CPU_CTRL_REG(0), C0_CPU_CTRL_REG_CORE_RESET); /* 0x4768(0, 0) */
	cluster_ctrl(false);                                    /* 0x4968(0) */
	mmio_clr_32(CPUCFG_10, BIT(4));                         /* 0x48cc(0) */
	for (int n = MAX_CORES_PER_CLUSTER - 1; n >= 0; --n)
		mmio_clr_32(C0_CPU_CTRL_REG(n), BIT(4));
	mmio_clr_32(CPUCFG_14, BIT(0));                         /* 0x485c(0) */
	cluster_ctrl(true);                                     /* 0x4968(1) */
	for (uint32_t n = 1; n < MAX_CORES_PER_CLUSTER; ++n)
		mmio_write_32(RVBAR_SEEN_BASE + 4 * (n - 1),
		              mmio_read_32(ALT_RVBAR_LO_REG(n)));
	for (uint32_t n = 0; n < MAX_CORES_PER_CLUSTER; ++n) {
		mmio_write_32(ALT_RVBAR_LO_REG(n), rvbar_lo[n]);
		mmio_write_32(ALT_RVBAR_HI_REG(n), rvbar_hi[n]);
	}
}

void __attribute__((noipa))
css_resume_core(uint32_t cluster UNUSED, uint32_t core, uint32_t old_state)
{
	if (old_state < SCPI_CSS_OFF)
		return;

	mmio_write_32(ALT_RVBAR_LO_REG(core), warm_entry_lo);   /* 0x4c2c */
	mmio_write_32(ALT_RVBAR_HI_REG(core), warm_entry_hi);
	mmio_set_32(CPU_CTRL_REG(core), CPU_CTRL_REG_AA64nAA32); /* 0x4c6c */
	mmio_set_32(CPU_UNK_REG(core), CPU_UNK_REG_nPORESET);   /* 0x4bfc */
	mmio_set_32(C0_CPU_CTRL_REG(core), C0_CPU_CTRL_REG_CORE_RESET); /* 0x4768(0, 3) */
}

void
css_init(void)
{
	/* The cores are already running with U-Boot's/TF-A's RVBAR. */
}
