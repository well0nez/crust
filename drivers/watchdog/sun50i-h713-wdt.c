/*
 * Copyright © 2022-2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#include <delay.h>
#include <error.h>
#include <intrusive.h>
#include <mmio.h>
#include <util.h>
#include <watchdog/sun6i-a31-wdt.h>
#include <platform/devices.h>

#include "watchdog.h"

/*
 * sun50iw12 R_WDOG (blob 0xcd8c, TF-A sunxi_power.c): CFG at +0x10 and
 * MODE at +0x14 take the key 0x16aa in bits 31..16; the blob writes
 * +0x00 = 0, CFG = key | 1 (system reset), MODE = key | 1 (enable, shortest
 * interval). No feed sequence exists in the blob and the vendor firmware
 * never arms a watchdog, so neither does this driver: probe and restart do
 * nothing, reset_system is the proven sequence.
 */
#define WDOG_IRQ_EN_REG  0x00
#define WDOG_CFG_REG     0x10
#define WDOG_MODE_REG    0x14
#define WDOG_KEY         (0x16aa << 16)

static void
h713_wdt_reset_system(const struct device *dev)
{
	const struct sun6i_a31_wdt *self = container_of(dev, const struct sun6i_a31_wdt, dev);

	mmio_write_32(self->regs + WDOG_IRQ_EN_REG, 0);
	mmio_write_32(self->regs + WDOG_CFG_REG, WDOG_KEY | 1);
	mmio_write_32(self->regs + WDOG_MODE_REG, WDOG_KEY | 1);

	udelay(1000000);
}

static void
h713_wdt_restart(const struct device *dev UNUSED)
{
}

static const struct watchdog_driver h713_wdt_driver = {
	.drv = {
		.probe   = dummy_probe,
		.release = dummy_release,
	},
	.ops = {
		.reset_system = h713_wdt_reset_system,
		.restart      = h713_wdt_restart,
	},
};

const struct sun6i_a31_wdt r_wdog = {
	.dev = {
		.name  = "r_wdog",
		.drv   = &h713_wdt_driver.drv,
		.state = DEVICE_STATE_INIT,
	},
	.regs = DEV_R_WDOG,
};
