/*
 * Copyright © 2017-2022 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#include <debug.h>
#include <exception.h>
#include <asm/exception.h>
#include <spr.h>
#include <steps.h>

void
report_exception(uint32_t exception)
{
	uint32_t pc;

	if (!exception)
		return;
	/* H713: a start through the reset vector is not a fault. */
	if (CONFIG(PLATFORM_H713) && exception == RESET_EXCEPTION)
		return;

	pc = mfspr(SPR_SYS_EPCR_ADDR(0));
	record_exception(exception, pc);
	error("Exception %u at %p!", exception, (void *)pc);
}
