/*
 * Copyright © 2021-2022 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#include <debug.h>
#include <mmio.h>
#include <steps.h>
#include <util.h>
#if CONFIG(PLATFORM_H713)
#include <hdmi/sun50i-h713-hdmi.h>
#include <platform/memory.h>
#endif

#define DRAM_BASE  0x40000000

#define MIN_OFFSET sizeof(uint32_t)
#define MAX_OFFSET BIT(26) /* 64 MiB */

static uint32_t saved_checksum;

static uint32_t
dram_calc_checksum(void)
{
	uint32_t checksum = 0;

	for (uint32_t offset = MIN_OFFSET; offset < MAX_OFFSET; offset <<= 1) {
		checksum += mmio_read_32(DRAM_BASE + 1 * offset);
		checksum += mmio_read_32(DRAM_BASE + 3 * offset);
		checksum += 1;
		checksum *= ~offset;
	}

	return checksum;
}

void
dram_save_checksum(void)
{
	saved_checksum = dram_calc_checksum();
}

void
dram_verify_checksum(void)
{
	uint32_t checksum;

	record_step(STEP_RESUME_DRAM_CHECKSUM);
	checksum = dram_calc_checksum();
	if (checksum == saved_checksum)
		return;
#if CONFIG(PLATFORM_H713)
	/*
	 * A record, not yet a reset: the words lie in Linux's low memory, and
	 * whether a DMA master writes there between the save and the
	 * self-refresh is not measured. A false alarm must not reset a
	 * healthy system; this becomes fatal once long runs show no mismatch.
	 */
	h713_err(H713_ERR_DRAM_SUM, 0, checksum);
#else
	panic("DRAM checksum mismatch!");
#endif
}
