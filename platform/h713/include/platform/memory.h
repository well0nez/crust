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
 */
#define H713_SHARED_BASE  0x00023800
#define H713_SHARED_LIMIT SCPI_MEM_BASE

#define STACK_SIZE     0x00000400

#endif /* PLATFORM_MEMORY_H */
