/*
 * Copyright © 2017-2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#include <debug.h>

/*
 * The H713 DRAM controller is not the H6's: the blob's standby driver
 * (enter 0x142dc -> 0x14004, exit 0x145b0) works on DRAMCOM 0x04002000,
 * DRAMCTL 0x04810000, PHY 0x04820000 and a 128-byte copy at 0x04800100.
 * Until that routine is ported (FOLLOW-UP TASKS), the DRAM stays clocked
 * and never enters self-refresh: "light standby".
 */
void
dram_suspend(void)
{
	debug("DRAM: no self-refresh on this SoC yet");
}

void
dram_resume(void)
{
}

void
dram_init(void)
{
}
