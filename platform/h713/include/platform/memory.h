/*
 * Copyright © 2017-2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#ifndef PLATFORM_MEMORY_H
#define PLATFORM_MEMORY_H

/*
 * The vendor blob occupies AR100 0x0000..0x23000 (vectors in the image,
 * body from 0x4000). Crust takes the 64 KiB block above it instead, so the
 * two images never overlap and a switch between them is a load, not a
 * relayout. The linker forbids a 64 KiB boundary inside the firmware.
 */
#define FIRMWARE_BASE  0x00010000
#define FIRMWARE_LIMIT 0x00020000
#define FIRMWARE_SIZE  (FIRMWARE_LIMIT - FIRMWARE_BASE)

/* Top 1 KiB: SCPI shared memory (crust ABI, two clients). */
#define SCPI_MEM_BASE  (SRAM_A2_LIMIT - 0x400)
#define SCPI_MEM_LIMIT SRAM_A2_LIMIT
#define SCPI_MEM_SIZE  (SCPI_MEM_LIMIT - SCPI_MEM_BASE)

/*
 * SRAM A2 is 128 KiB at ARM 0x00104000 (TF-A sunxi_mmap.h, doku/68). The
 * first 16 KiB of the AR100 address space (0x0000..0x3fff) are the vector
 * area and a reserved block, as on the H6.
 */
#define SRAM_A2_BASE   0x00004000
#define SRAM_A2_LIMIT  0x00024000
#define SRAM_A2_SIZE   (SRAM_A2_LIMIT - SRAM_A2_BASE)

/* Difference between SRAM_A2_BASE in the AR100 and ARM address spaces. */
#define SRAM_A2_OFFSET 0x00100000

/*
 * Contract page (this port, tables/arisc-contract.csv): the variables the
 * Linux driver sun50i-h713-arisc reads and writes, at fixed addresses
 * instead of the vendor build's BSS. ARM address = these + 0x100000.
 * +0x1fc (version 4): the bytes of .text and .rodata from the image start.
 */
#define H713_SHARED_BASE  0x00023800
#define H713_SHARED_LIMIT SCPI_MEM_BASE

/*
 * Diagnostic words inside the contract page (+0x200..+0x23f), kept across
 * firmware restarts; the kernel driver zeroes them before it releases the
 * core and reports them. +0x00 R_CPUCFG little-endian window enable word as
 * found at start (0 = no window was open), +0x04/+0x08 window 0 start/end,
 * +0x0c CPUS_CLK_REG as found at start, +0x10 the tick timer (TTCR) of the
 * last main loop pass, +0x14 number of h713_init() runs, +0x18/+0x1c
 * R_INTC +0x80/+0x88 as found at start, +0x20 frames received from the
 * display MIPS, +0x24 the first four bytes of the last one, +0x28 frames
 * from the ARM with a type other than 0, +0x2c the last standby step
 * (common/debug/steps.c, 0 = none), +0x30 the last exception (number << 24
 * | pc), +0x34 milliseconds counted asleep in the last standby (TF-A puts
 * the system counter forward by them when the counter was reset), +0x38
 * the armed wake timer in milliseconds (0x26 type 3; 0 = none) - here, not
 * in .bss, so a firmware restart while asleep still ends the standby, with
 * a reset - and +0x3c the build: image size in bits 19..0, bit 24 DRAM
 * self-refresh, bit 25 deep sleep, bit 26 IR wake codes, bit 27 the deep VDD_SYS step,
 * bit 28 the deep PLL LDO step (0034; D1a has bit 25 without 27 and 28).
 */
#define H713_DIAG_BASE       (H713_SHARED_BASE + 0x200)
#define H713_DIAG_STEP       (H713_DIAG_BASE + 0x2c)
#define H713_DIAG_EXCEPTION  (H713_DIAG_BASE + 0x30)
#define H713_DIAG_SLEPT_MS   (H713_DIAG_BASE + 0x34)
#define H713_DIAG_WAKE_TIMER (H713_DIAG_BASE + 0x38)
#define H713_DIAG_BUILD      (H713_DIAG_BASE + 0x3c)
#define H713_DIAG_LIMIT      (H713_SHARED_BASE + 0x240)

#define STACK_SIZE     0x00000400

#endif /* PLATFORM_MEMORY_H */
