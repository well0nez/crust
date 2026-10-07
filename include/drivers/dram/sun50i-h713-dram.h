/*
 * Copyright © 2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#ifndef DRIVERS_DRAM_SUN50I_H713_DRAM_H
#define DRIVERS_DRAM_SUN50I_H713_DRAM_H

#include <stdbool.h>
#include <stdint.h>

/**
 * The last dram_resume(): PGSR0 if the PHY reported an error (bits 27..20),
 * else 0. Always 0 without self-refresh.
 */
uint32_t h713_dram_error(void);

/**
 * Does PLL_DDR0 run at the rate the self-refresh exit programs (N from
 * para.clk)? Stores the register in @pll. Always true, with @pll 0,
 * without self-refresh.
 */
bool h713_dram_clk_ok(uint32_t *pll);

#endif /* DRIVERS_DRAM_SUN50I_H713_DRAM_H */
