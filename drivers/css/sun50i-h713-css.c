/*
 * Copyright © 2017-2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#include <delay.h>
#include <mmio.h>
#include <scpi_protocol.h>
#include <stdint.h>
#include <platform/cpucfg.h>

#include "css.h"

/*
 * Cluster down: what the blob's 0x4b28 does after "cpu op req" (FB-STANDBY
 * excerpt D): C0_PWRON_RESET = 0, bit 0 of CPU_UNK 0..3 cleared (power-on
 * reset of every core), CPU_SYS_RESET = 0, then the clamps.
 * Core up: TF-A's proven sunxi_cpu_on() for the ncat register file, with the
 * warm entry the ARM handed us in ALT_RVBAR (blob 0x4c40/0x4be4).
 */
static uint32_t warm_entry_lo, warm_entry_hi;

void
css_set_warm_entry(uint32_t lo, uint32_t hi)
{
	warm_entry_lo = lo;
	warm_entry_hi = hi;
}

void
css_suspend_css(uint32_t new_state)
{
	if (new_state < SCPI_CSS_OFF)
		return;

	mmio_write_32(C0_PWRON_RESET_REG, 0);
	for (uint32_t i = 0; i < MAX_CORES_PER_CLUSTER; ++i)
		mmio_clr_32(CPU_UNK_REG(i), CPU_UNK_REG_nPORESET);
	mmio_write_32(CPU_SYS_RESET_REG, 0);
	for (uint32_t i = 0; i < MAX_CORES_PER_CLUSTER; ++i)
		mmio_write_32(C0_CPUn_PWR_SWITCH_REG(i), 0xff);
}

void
css_resume_css(uint32_t old_state)
{
	if (old_state < SCPI_CSS_OFF)
		return;

	/* Mirror of the way down; the blob's 0xddec/0xe3f4 order is a guess. */
	mmio_write_32(CPU_SYS_RESET_REG, CPU_SYS_RESET);
	mmio_write_32(C0_PWRON_RESET_REG, C0_PWRON_RESET_REG_nH_RST);
}

void
css_resume_core(uint32_t cluster UNUSED, uint32_t core, uint32_t old_state)
{
	if (old_state < SCPI_CSS_OFF)
		return;

	mmio_write_32(ALT_RVBAR_LO_REG(core), warm_entry_lo);
	mmio_write_32(ALT_RVBAR_HI_REG(core), warm_entry_hi);

	/* TF-A sunxi_cpu_on(), non-per-cluster path. */
	mmio_clr_32(C0_CPU_CTRL_REG(core), C0_CPU_CTRL_REG_CORE_RESET);
	mmio_clr_32(CPU_UNK_REG(core), CPU_UNK_REG_nPORESET);
	mmio_set_32(CPU_CTRL_REG(core), CPU_CTRL_REG_AA64nAA32);
	mmio_write_32(C0_CPUn_PWR_SWITCH_REG(core), 0xfe);
	mmio_write_32(C0_CPUn_PWR_SWITCH_REG(core), 0xf8);
	mmio_write_32(C0_CPUn_PWR_SWITCH_REG(core), 0xe0);
	mmio_write_32(C0_CPUn_PWR_SWITCH_REG(core), 0x80);
	mmio_write_32(C0_CPUn_PWR_SWITCH_REG(core), 0x00);
	udelay(1);
	mmio_clr_32(CPU_UNK_REG(core), CPU_UNK_REG_CLAMP);
	mmio_set_32(CPU_UNK_REG(core), CPU_UNK_REG_nPORESET);
	mmio_set_32(C0_CPU_CTRL_REG(core), C0_CPU_CTRL_REG_CORE_RESET);
	mmio_set_32(C0_CPU_CTRL_REG(core), C0_CPU_CTRL_REG_DBG_PWRUP);
}

void
css_init(void)
{
	/* The cores are already running with U-Boot's/TF-A's RVBAR. */
}
