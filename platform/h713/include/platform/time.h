/*
 * Copyright © 2017-2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#ifndef PLATFORM_TIME_H
#define PLATFORM_TIME_H

/*
 * r_ccu_init() puts CPUS on OSC24M/1 (R_PRCM 0x000, mux 24..25 = 0, the
 * D1-style layout the kernel's sun20i-d1-r-ccu model gives this SoC), so
 * the tick timer runs at exactly 24 MHz and needs no IOSC calibration.
 */
#define CPUCLK_MHz 24
#define CPUCLK_kHz (CPUCLK_MHz * 1000)
#define CPUCLK_Hz  (CPUCLK_MHz * 1000000)

#define REFCLK_MHZ 24
#define REFCLK_KHZ (REFCLK_MHZ * 1000)
#define REFCLK_HZ  (REFCLK_MHZ * 1000000)

#endif /* PLATFORM_TIME_H */
