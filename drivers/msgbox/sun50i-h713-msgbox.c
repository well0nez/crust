/*
 * Copyright © 2017-2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#include <debug.h>
#include <error.h>
#include <mmio.h>
#include <stdbool.h>
#include <stddef.h>
#include <util.h>
#include <clock/ccu.h>
#include <msgbox/sunxi-msgbox.h>
#include <platform/devices.h>

#include "msgbox.h"

/*
 * The H713 message box is three "users" of 0x400 bytes (stock DT
 * msgbox-amp: 0x03003000, 0x03003400, 0x03003800), each with sub-blocks of
 * 0x100 holding four ports: FIFO_STAT at +0x60 + 4p (0 = empty, 8 = full),
 * MSG_DATA at +0x70 + 4p, an IRQ status W1C at +0x24 (blob 0x7b10 writes
 * 0x40 to 0x03003424 = port 3's RX bit), a TX IRQ enable at +0x30.
 *
 * user0 sub-block 0 : ARISC -> ARM (the driver's "msgbox-rx", ch 0..3)
 * user1 sub-block 0 : ARM -> ARISC (the driver's "msgbox-tx", port 0..3)
 * user0/1 sub-block 1, user2: the MIPS's pairs (cpu_comm), not ours.
 *
 * Crust channel numbering: even n = receive from ARM port n/2, odd n = send
 * to ARM channel n/2. There is no "last tx done" flag on this box: the FIFO
 * count is the only state, so a send is done when it has been popped.
 */
#define USER0               0x0000
#define USER1               0x0400
#define FIFO_STAT_REG(u, p) ((u) + 0x0060 + 0x4 * (p))
#define MSG_DATA_REG(u, p)  ((u) + 0x0070 + 0x4 * (p))
#define IRQ_STAT_REG(u)     ((u) + 0x0024)
#define FIFO_DEPTH          8

static bool
h713_msgbox_last_tx_done(const struct device *dev, uint8_t chan)
{
	const struct simple_device *self = to_simple_device(dev);

	assert(chan & 1);

	return mmio_read_32(self->regs + FIFO_STAT_REG(USER0, chan / 2)) == 0;
}

static void
h713_msgbox_ack_rx(const struct device *dev, uint8_t chan)
{
	const struct simple_device *self = to_simple_device(dev);

	mmio_write_32(self->regs + IRQ_STAT_REG(USER1), BIT(2 * (chan / 2)));
}

static int
h713_msgbox_receive(const struct device *dev, uint8_t chan, uint32_t *msg)
{
	const struct simple_device *self = to_simple_device(dev);

	assert(!(chan & 1));

	if (!mmio_read_32(self->regs + FIFO_STAT_REG(USER1, chan / 2)))
		return ENOENT;
	*msg = mmio_read_32(self->regs + MSG_DATA_REG(USER1, chan / 2));

	return SUCCESS;
}

static int
h713_msgbox_send(const struct device *dev, uint8_t chan, uint32_t msg)
{
	const struct simple_device *self = to_simple_device(dev);

	assert(chan & 1);

	if (mmio_read_32(self->regs + FIFO_STAT_REG(USER0, chan / 2)) >=
	    FIFO_DEPTH)
		return EBUSY;
	mmio_write_32(self->regs + MSG_DATA_REG(USER0, chan / 2), msg);

	return SUCCESS;
}

static int
h713_msgbox_probe(const struct device *dev)
{
	/*
	 * The clock (CCU 0x71c) is on: cpu_comm and the ARISC driver need
	 * it before this firmware runs. Nothing is drained here: the ARM
	 * driver empties its RX channels itself, and words the ARM already
	 * queued for us must not be lost.
	 */
	return simple_device_probe(dev);
}

static const struct msgbox_driver h713_msgbox_driver = {
	.drv = {
		.probe   = h713_msgbox_probe,
		.release = simple_device_release,
	},
	.ops = {
		.ack_rx       = h713_msgbox_ack_rx,
		.last_tx_done = h713_msgbox_last_tx_done,
		.receive      = h713_msgbox_receive,
		.send         = h713_msgbox_send,
	},
};

const struct simple_device msgbox = {
	.dev = {
		.name  = "msgbox",
		.drv   = &h713_msgbox_driver.drv,
		.state = DEVICE_STATE_INIT,
	},
	.clock = { .dev = &ccu.dev, .id = CLK_BUS_MSGBOX },
	.regs  = DEV_MSGBOX,
};
