/*
 * Copyright © 2017-2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#include <clock/ccu.h>
#include <gpio/sunxi-gpio.h>
#include <platform/devices.h>

#include "uart.h"

/*
 * R_UART on PL10/PL11, function 3: the vendor DT's arisc_config/s_uart_config
 * (disabled there, so the pins are free). Never PL2/PL3 as on the H6: PL2 is
 * the audio amplifier enable and PL3 the USB VBUS switch on this board.
 */
#if CONFIG(SERIAL_DEV_R_UART)

const struct simple_device uart = {
	.dev = {
		.name  = "r_uart",
		.drv   = &uart_driver,
		.state = DEVICE_STATE_INIT,
	},
	.clock = { .dev = &r_ccu.dev, .id = CLK_BUS_R_UART },
	.pins  = SIMPLE_DEVICE_PINS_INIT {
		{
			.dev   = &r_pio.dev,
			.id    = SUNXI_GPIO_PIN(0, 10), /* PL10 */
			.drive = DRIVE_10mA,
			.mode  = 3,
			.pull  = PULL_NONE,
		},
		{
			.dev   = &r_pio.dev,
			.id    = SUNXI_GPIO_PIN(0, 11), /* PL11 */
			.drive = DRIVE_10mA,
			.mode  = 3,
			.pull  = PULL_UP,
		},
	},
	.regs = DEV_R_UART,
};

#else
#error "The H713 port supports the R_UART only (PL10/PL11)"
#endif
