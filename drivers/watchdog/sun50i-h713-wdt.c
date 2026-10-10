/*
 * Copyright © 2022-2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#include <delay.h>
#include <error.h>
#include <intrusive.h>
#include <mmio.h>
#include <util.h>
#include <hdmi/sun50i-h713-hdmi.h>
#include <watchdog/sun6i-a31-wdt.h>
#include <platform/devices.h>

#include "watchdog.h"

/*
 * sun50iw12 R_WDOG (blob 0xcd8c, TF-A sunxi_power.c) and the SoC watchdog
 * 0x02051000 (DEV_WDOG) in the sun55i-a523 layout Linux uses for them
 * (sunxi_wdt.c): CTRL +0x0c takes the feed 0x14af (key 0xa57 << 1, bit 0
 * reload), CFG +0x10 and MODE +0x14 take the key 0x16aa in bits 31..16.
 * Measured on the SoC watchdog (07.10.2026): the feed, MODE 0x16aa00b1
 * resets after 16.4 s, and a MODE write into the running counter without a
 * feed first does not reset. The R_WDOG's CFG and MODE are the blob's
 * (it writes +0x00 = 0, CFG = key | 1, MODE = key | 1 to reset and never
 * arms it); its feed, clock, interval and reset scope are unmeasured.
 *
 * Probe and restart do nothing, so the AWAKE loop never feeds and the
 * generic get/put arm nothing. Only the sleep net (H713_SLEEP_NET,
 * sun50i-h713-sleep.c) arms, feeds and disarms the R_WDOG, through the
 * h713_rwdog_*() helpers below; h713_wdog_reset() is the clean reset.
 */
#define WDOG_IRQ_EN_REG  0x00
#define WDOG_IRQ_STA_REG 0x04 /* a523 layout; not read on the H713 yet */
#define WDOG_CTRL_REG    0x0c
#define WDOG_CFG_REG     0x10
#define WDOG_MODE_REG    0x14
#define WDOG_KEY         (0x16aa << 16)
#define WDOG_FEED        0x14af

/* TF-A's reset sequence (T1): feed, reset on fire, shortest interval, feed. */
void
h713_wdog_reset(uintptr_t base)
{
	mmio_write_32(base + WDOG_CTRL_REG, WDOG_FEED);
	mmio_write_32(base + WDOG_CFG_REG, WDOG_KEY | 1);
	mmio_write_32(base + WDOG_MODE_REG, WDOG_KEY | 1);
	mmio_write_32(base + WDOG_CTRL_REG, WDOG_FEED);
}

/* Arm with interval @code; returns CFG << 16 | MODE as read back. */
uint32_t
h713_rwdog_arm(uint8_t code)
{
	mmio_write_32(DEV_R_WDOG + WDOG_CTRL_REG, WDOG_FEED);
	mmio_write_32(DEV_R_WDOG + WDOG_CFG_REG, WDOG_KEY | 1);
	mmio_write_32(DEV_R_WDOG + WDOG_MODE_REG,
	              WDOG_KEY | (code & 0xf) << 4 | 1);
	mmio_write_32(DEV_R_WDOG + WDOG_CTRL_REG, WDOG_FEED);

	return (mmio_read_32(DEV_R_WDOG + WDOG_CFG_REG) & 0xffff) << 16 |
	       (mmio_read_32(DEV_R_WDOG + WDOG_MODE_REG) & 0xffff);
}

void
h713_rwdog_feed(void)
{
	mmio_write_32(DEV_R_WDOG + WDOG_CTRL_REG, WDOG_FEED);
}

/* Feed, then off; returns MODE as read back. */
uint32_t
h713_rwdog_disarm(void)
{
	h713_rwdog_feed();
	mmio_write_32(DEV_R_WDOG + WDOG_MODE_REG, WDOG_KEY);

	return mmio_read_32(DEV_R_WDOG + WDOG_MODE_REG);
}

/* Either watchdog: feed, then off, three tries; returns MODE as read back. */
uint32_t
h713_wdog_stop(uintptr_t base)
{
	uint32_t mode = 0;

	for (uint8_t i = 0; i < 3; ++i) {
		mmio_write_32(base + WDOG_CTRL_REG, WDOG_FEED);
		mmio_write_32(base + WDOG_MODE_REG, WDOG_KEY);
		mode = mmio_read_32(base + WDOG_MODE_REG);
		if (!(mode & 1))
			break;
		udelay(1000);
	}
	return mode;
}

/* As found: STA << 16 | MODE. */
uint32_t
h713_rwdog_state(void)
{
	return (mmio_read_32(DEV_R_WDOG + WDOG_IRQ_STA_REG) & 0xffff) << 16 |
	       (mmio_read_32(DEV_R_WDOG + WDOG_MODE_REG) & 0xffff);
}

static void
h713_wdt_reset_system(const struct device *dev)
{
	const struct sun6i_a31_wdt *self = container_of(dev, const struct sun6i_a31_wdt, dev);

	if (CONFIG(H713_SLEEP_NET)) {
		/* the net may be running: a MODE write alone is swallowed */
		h713_wdog_reset(self->regs);
	} else {
		mmio_write_32(self->regs + WDOG_IRQ_EN_REG, 0);
		mmio_write_32(self->regs + WDOG_CFG_REG, WDOG_KEY | 1);
		mmio_write_32(self->regs + WDOG_MODE_REG, WDOG_KEY | 1);
	}

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
