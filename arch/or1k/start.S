/*
 * Copyright © 2017-2022 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#include <macros.S>
#include <asm/spr.h>
#if CONFIG(PLATFORM_H713)
#include <platform/devices.h>
#include <platform/memory.h>
#endif

func start
	# Save the exception vector address
	l.mfspr	r2, r0, SPR_SYS_PPC_ADDR

	# Invalidate the instruction cache
	l.addi	r3, r0, 0
	l.addi	r4, r0, 4096		# Cache lines (256) * block size (16)
1:	l.mtspr	r0, r3, SPR_ICACHE_ICBIR_ADDR
	l.sfltu	r3, r4
	l.bf	1b
	l.addi	r3, r3, 16		# Cache block size

	# Flush the CPU pipeline
	l.psync

	# Enable the instruction cache
	l.mfspr	r3, r0, SPR_SYS_SR_ADDR
	l.ori	r3, r3, SPR_SYS_SR_ICE_MASK
	l.mtspr	r0, r3, SPR_SYS_SR_ADDR

	# One cache block of nops
	l.nop
	l.nop
	l.nop
	l.nop

#if CONFIG(PLATFORM_H713)
	# The vendor ARISC firmware opens a "little-endian window" in R_CPUCFG
	# at every start (blob 0x45b0: enable bit n in +0x0c, start/end at
	# +0x10/+0x14 + 8n) over [RTC GP3, GP3 + 1 MiB). With GP3 = 0 or the
	# 0xb00f marker that window covers all of SRAM A2, and the registers
	# survive the core reset the kernel driver uses to swap images. Inside
	# it the byte lanes of every data access flip, so this firmware would
	# read its own .data/.rodata bytes and every byte the ARM writes from
	# the wrong lane. Close all four windows the way the blob's own remove
	# routine does (0x4668: enable bit cleared, start and end zeroed)
	# before the first byte access, and leave what was found in the
	# contract page's diagnostic words for the kernel driver to report.
	# Word accesses only: they are lane-independent.
	l.movhi	r3, hi(DEV_R_CPUCFG)
	l.ori	r3, r3, lo(DEV_R_CPUCFG)
	l.lwz	r4, 0x0c(r3)		# window enable bits 3..0
	l.andi	r5, r4, 0xf
	l.sfeqi	r5, 0
	l.bf	2f
	l.nop
	l.movhi	r6, hi(H713_DIAG_BASE)
	l.ori	r6, r6, lo(H713_DIAG_BASE)
	l.sw	0(r6), r4		# diag +0: enable word as found
	l.lwz	r5, 0x10(r3)
	l.sw	4(r6), r5		# diag +4: window 0 start
	l.lwz	r5, 0x14(r3)
	l.sw	8(r6), r5		# diag +8: window 0 end
	l.addi	r5, r0, -16
	l.and	r4, r4, r5
	l.sw	0x0c(r3), r4		# all four windows off
	l.sw	0x10(r3), r0
	l.sw	0x14(r3), r0
	l.sw	0x18(r3), r0
	l.sw	0x1c(r3), r0
	l.sw	0x20(r3), r0
	l.sw	0x24(r3), r0
	l.sw	0x28(r3), r0
	l.sw	0x2c(r3), r0
2:
#endif

	# Clear .bss
	l.movhi	r10, hi(start)		# High word is common to all symbols
	l.ori	r3, r10, lo(__bss_start)
	l.ori	r4, r10, lo(__bss_end)
1:	l.sw	0(r3), r0
	l.sfltu	r3, r4
	l.bf	1b
	l.addi	r3, r3, 4

	# Prepare function arguments
	l.sfltui r2, 0x4000		# Did PC come from an exception vector?
	l.bnf	1f
	l.movhi	r3, 0			# Set to zero if not an exception
	l.srli	r3, r2, 8		# Else compute the exception number

	# Jump to the C entry point
1:	l.j	system_state_machine
	l.ori	r1, r10, lo(__stack_end)
endfunc start
