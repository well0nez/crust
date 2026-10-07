/*
 * Copyright © 2017-2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#ifndef PLATFORM_MEMORY_H
#define PLATFORM_MEMORY_H

/*
 * The vendor blob occupies AR100 0x0000..0x23000 (vectors in the image,
 * body from 0x4000); crust's 64 KiB block lies inside it - the blob's code,
 * data and stack (bss 0x15f38..0x175b0, stack top 0x179b0) - so the two
 * images overlap, and a switch between them is always a core reset and a
 * full reload. The linker forbids a 64 KiB boundary inside the firmware.
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
 * The contract page, version 5: the one table of it (the kernel's
 * h713-crust-contract.h and TF-A mirror it). AR100 0x23800..0x23bff, ARM =
 * these + 0x100000. Words only: the ARM and the AR100 have no atomic
 * read-modify-write between them, so every word has one writer.
 *
 * class  range          writer           zeroed by                 kept across
 *   R    +0x000..+0x1ff crust            crust at every start      -
 *                                        (an acknowledged restart
 *                                        keeps +0x010..+0x0ff)
 *   D    +0x200..+0x23f crust, start.S   the loaders               restarts
 *   S    +0x240..+0x25f crust            crust at every start      -
 *   K    +0x260..+0x2bf crust only       crust at its first start  restarts
 *                                        (diag starts == 0), the
 *                                        loaders after copying to P
 *   A    +0x2c0..+0x2cf the ARM, one     the loaders               restarts
 *                       writer per word
 *   P    +0x2d0..+0x38f the ARM only     its writer                crust never
 *                       (TF-A, kernel)                             touches P
 *   X    +0x390..+0x3ff reserved, rules  as K                      restarts
 *                       of K
 * The loaders (TF-A at boot, the kernel before a reload) copy D, K and A to
 * P and then zero everything but P.
 */
#define H713_SHARED_BASE  0x00023800
#define H713_SHARED_LIMIT SCPI_MEM_BASE
#define H713_PAGE(off)    (H713_SHARED_BASE + (off))

/*
 * R: +0x000 magic 0x33313748, +0x004 version, +0x008 status (bits 0..3,
 * heartbeat 31..16), +0x00c wake word, +0x010..+0x1c7 the vendor's HDMI
 * variables at its distances (sun50i-h713-hdmi.c), +0x1fc the bytes of
 * .text and .rodata (version 4).
 */
#define H713_R_KEPT_BASE     H713_PAGE(0x010)  /* acknowledged restart */
#define H713_R_KEPT_LIMIT    H713_PAGE(0x100)

/*
 * D: diagnostic words, +0x00 R_CPUCFG little-endian window enable word as
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
 * in .bss, so a firmware restart while asleep still ends the standby - and
 * +0x3c the build word: image size in bits 19..0, bit 24 DRAM self-refresh,
 * 25 deep sleep, 26 IR wake codes, 27 the deep VDD_SYS step, 28 the deep
 * PLL LDO step, 29 the sleep net, 30 a test build, 31 AVCC gating.
 */
#define H713_DIAG_BASE       H713_PAGE(0x200)
#define H713_DIAG_INITS      (H713_DIAG_BASE + 0x14)
#define H713_DIAG_STEP       (H713_DIAG_BASE + 0x2c)
#define H713_DIAG_EXCEPTION  (H713_DIAG_BASE + 0x30)
#define H713_DIAG_SLEPT_MS   (H713_DIAG_BASE + 0x34)
#define H713_DIAG_WAKE_TIMER (H713_DIAG_BASE + 0x38)
#define H713_DIAG_BUILD      (H713_DIAG_BASE + 0x3c)
#define H713_DIAG_LIMIT      H713_PAGE(0x240)

/* S: what one start saw, for the device tests. */
#define H713_SEEN_RVBAR      H713_PAGE(0x240) /* ALT_RVBAR lo of cores 1..3 after the cluster up */
#define H713_SEEN_EINT       H713_PAGE(0x24c) /* PL EINT status: before, switch, settled, after */
#define H713_SEEN_LOSC       H713_PAGE(0x25c) /* LOSC_CTRL after a deep resume */
#define H713_SEEN_LIMIT      H713_PAGE(0x260)

/* K: crust's own record, kept across its restarts. */
#define H713_K_BASE          H713_PAGE(0x260)
#define H713_K_COMPAT        H713_PAGE(0x260) /* 7..0 oldest reader version, 15..8 newest NET_REQ format */
#define H713_K_BUILD2        H713_PAGE(0x264) /* 11..0 DRAM MHz, 15..12 board, 23..16 R_WDOG interval */
#define H713_K_SB_STATE      H713_PAGE(0x268) /* standby phase, see below */
#define H713_K_NET_ACC       H713_PAGE(0x26c) /* budgets taken, s: 15..0 presleep, 31..16 resuming */
#define H713_K_NET_RB        H713_PAGE(0x270) /* R_WDOG read back at arming: 15..0 MODE, 31..16 CFG */
#define H713_K_NET_MS        H713_PAGE(0x274) /* ms in the current (or the last ended) net phase */
#define H713_K_NET_FOUND     H713_PAGE(0x278) /* R_WDOG at the first start: 15..0 MODE, 31..16 STA */
#define H713_K_ERR           H713_PAGE(0x27c) /* sticky error bits, below */
#define H713_K_ERR_DETAIL    H713_PAGE(0x280) /* 31..24 last error bit, 23..16 detail, 15..0 step */
#define H713_K_ERR_DATA      H713_PAGE(0x284) /* the register value of the last error */
#define H713_K_CNT0          H713_PAGE(0x288) /* 8-bit counters, wrapping, below */
#define H713_K_CNT1          H713_PAGE(0x28c)
#define H713_K_CNT2          H713_PAGE(0x290)
#define H713_K_CYCLE         H713_PAGE(0x294) /* standby attempts */
#define H713_K_WAKE_MS       H713_PAGE(0x298) /* ms from the cpu op req to CPU0's release */
#define H713_K_MIPS_PORT     H713_PAGE(0x29c) /* 7..0 port of the last DispMips frame */
#define H713_K_FAIL_PATH     H713_PAGE(0x2a0) /* 7..0 reset path tried (1 R_WDOG, 2 SoC WDOG), 15..8 try */
#define H713_K_DRAM_PLL      H713_PAGE(0x2a4) /* PLL_DDR0 when a request came in */
#define H713_K_SUBSTEP       H713_PAGE(0x2a8) /* deep stage only */
#define H713_K_DEEP_LOCK     H713_PAGE(0x2ac) /* deep stage only */
#define H713_K_LIMIT         H713_PAGE(0x2c0)

/* A: written by the ARM, read by crust. */
#define H713_A_NET_REQ       H713_PAGE(0x2c0) /* kernel: 31..24 0xa5, 23..16 format, 15..8 cycle */
#define H713_A_NET_BUDGET    H713_PAGE(0x2c4) /* kernel: s, 15..0 presleep, 31..16 resuming */
#define H713_A_NET_CONFIRM   H713_PAGE(0x2c8) /* kernel: 31..16 0xc0f1, 7..0 cycle */
#define H713_A_TFA_WORD      H713_PAGE(0x2cc) /* TF-A: 31..24 event, 23..16 number, 15..0 detail */

/* P: the previous run's record, ARM only; X: reserved. */
#define H713_P_BASE          H713_PAGE(0x2d0)
#define H713_X_BASE          H713_PAGE(0x390)
#define H713_X_FP_CYCLE      H713_PAGE(0x390) /* deep: 7..0 cycle of the set before, 15..8 after */
#define H713_X_FP_BEFORE     H713_PAGE(0x394) /* deep: main-domain fingerprint, 13 words */
#define H713_X_FP_AFTER      H713_PAGE(0x3c8)
#define H713_X_FP_WORDS      13
#define H713_X_LOCK_TICKS    H713_PAGE(0x3fc) /* deep: longest PLL lock wait, 31..24 PLL, 23..0 ticks */

/*
 * SB_STATE: 3..0 phase; 4 R_WDOG armed (read back); 5 request taken;
 * 7..6 restarts in the phase; 15..8 cycle (CYCLE & 0xff); 23..16 cause of
 * the last clean reset.
 */
#define H713_PH_OFF          0  /* no request since the first start */
#define H713_PH_PRESLEEP     1  /* request taken (0x25/0x26) */
#define H713_PH_CPU_OP       2  /* cpu op req (0x22) taken */
#define H713_PH_SLEEP        3  /* the system state machine suspends */
#define H713_PH_RESUMING     4  /* CPU0 released, or 0x11 without a sleep */
#define H713_PH_RELEASED     5  /* confirmed, the net off */
#define H713_PH_RESET        6  /* in h713_fail() */
#define H713_SB_PHASE        0x0000000f
#define H713_SB_ARMED        0x00000010
#define H713_SB_TAKEN        0x00000020
#define H713_SB_RESTARTS     0x000000c0
#define H713_SB_CYCLE        0x0000ff00
#define H713_SB_CAUSE        0x00ff0000

#define H713_NET_REQ_MARK    0xa5
#define H713_NET_FORMAT      1
#define H713_NET_CONFIRM_MARK 0xc0f1
#define H713_NET_BUDGET_MAX  120  /* s */

/* Causes of a clean reset (SB_STATE 23..16), with their ERR bit. */
#define H713_CAUSE_PRESLEEP  1  /* ERR 9, detail 0, data NET_MS */
#define H713_CAUSE_RESUMING  2  /* ERR 9, detail 1, data NET_MS */
#define H713_CAUSE_DRAM      3  /* ERR 6, data PGSR0 */
#define H713_CAUSE_POLL      4  /* ERR 7, detail poll id 1..9, data register */
#define H713_CAUSE_RESTART   5  /* ERR 8, detail phase, data exception word */
#define H713_CAUSE_CPU_OP    6  /* ERR 4, detail count, data mpidr */
#define H713_CAUSE_LOOP      7  /* ERR 8, detail phase, data exception word */
#define H713_CAUSE_PLL_LOCK  9  /* ERR 10, deep stage only */

/* The bounded polls of cause 4 (ERR_DETAIL 23..16), data the value read. */
#define H713_POLL_DRAM_MODE  1  /* STATR operating mode (dram.c) */
#define H713_POLL_DRAM_IDONE 2  /* PGSR0 idone after a PIR trigger */
#define H713_POLL_DRAM_PLL   3  /* PLL_DDR0 lock */
#define H713_POLL_DRAM_STATR 4  /* STATR bit 0 at the end of the exit */
#define H713_POLL_CPU_OFF    5  /* core 0 power switch off (css.c) */
#define H713_POLL_CPU_ON     6  /* core 0 power switch on */

/*
 * SUBSTEP, deep stage only: written before each clock and bus step of the
 * deep path, 15..8 the step, 7..0 its index, so a hang or a reset in it
 * names the register (PRUEFUNG5 DEEP-07). A good cycle leaves the last one.
 */
#define H713_SUB(step, i)    ((step) << 8 | (i))
#define H713_SUB_FP          1  /* main-domain fingerprint, 0 before, 1 after (ccu.c) */
#define H713_SUB_ROOT_OSC    2  /* root clock i to OSC24M (r-ccu.c root_clk[]) */
#define H713_SUB_PLL_OFF     3  /* PLL i off (r-ccu.c pll_reg[]) */
#define H713_SUB_LDO_OFF     4
#define H713_SUB_DOMAIN_OFF  5  /* 0 VDD_SYS reset, 1 AVCC and bit 8 gated */
#define H713_SUB_DOMAIN_ON   6  /* 0 AVCC and bit 8 ungated, 1 VDD_SYS reset off */
#define H713_SUB_LDO_ON      7
#define H713_SUB_PLL_ON      8  /* PLL i back */
#define H713_SUB_ROOT_BACK   9  /* root clock i back to its source */
#define H713_SUB_MBUS        10 /* MBUS/NSI rebuild write i (ccu.c) */

/*
 * DEEP_LOCK, deep stage only, zeroed at each suspend: 11..0 the PLLs that ran
 * before the sleep and did not lock in time, 23..16 the root clocks left on
 * OSC24M for it, 31 the PLL LDO left on because a PLL still read enabled
 * (H713_DEEP_PLL_LDO). A lock failure ends in the clean reset, cause 9
 * (detail the first such PLL, data its register). X_LOCK_TICKS: the longest
 * lock wait of the last resume in tick timer cycles (24 MHz), and its PLL.
 */
#define H713_LOCK_ROOTS      16
#define H713_LOCK_LDO_KEPT   31

/* ERR bits; fatal ones (4, 6..10) come with a clean reset. */
#define H713_ERR_KEY_PIN     0
#define H713_ERR_CIR         1
#define H713_ERR_TXQ_FULL    2
#define H713_ERR_SB_DROP     3
#define H713_ERR_CPU_OP      4
#define H713_ERR_AUX_CLOCK   5  /* detail 1 publish, 2 HPD pin, 3 RequestEDID, 4 MIPS */
#define H713_ERR_DRAM_PHY    6
#define H713_ERR_POLL        7
#define H713_ERR_RESTART     8
#define H713_ERR_BUDGET      9
#define H713_ERR_PLL_LOCK    10
#define H713_ERR_DRAM_PLL    11
#define H713_ERR_NET_REQ     12
#define H713_ERR_KEY_HELD    13
#define H713_ERR_DRAM_SUM    14
#define H713_ERR_RWDOG_FOUND 15
#define H713_ERR_RWDOG_RESET 16

/* Counter bytes for h713_cnt(), at the byte the ARM reads them from. */
#define H713_CNT_SB_DROP     (H713_K_CNT0 + 0) /* port 3 words, partial frames dropped */
#define H713_CNT_PUMP_LONG   (H713_K_CNT0 + 1) /* pump frame > 67 bytes */
#define H713_CNT_ARM_UNKNOWN (H713_K_CNT0 + 2) /* ARM type, sub-command, no answer */
#define H713_CNT_TXQ_FULL    (H713_K_CNT0 + 3) /* tx queue full, answer lost */
#define H713_CNT_STALE_REPLY (H713_K_CNT1 + 0) /* reply nobody read */
#define H713_CNT_MIPS_DROP   (H713_K_CNT1 + 1) /* DispMips words/frames dropped, aux unclocked */
#define H713_CNT_MIPS_17     (H713_K_CNT1 + 2) /* DispMips ResetEDIDModule */
#define H713_CNT_MIPS_16     (H713_K_CNT1 + 3) /* DispMips PullHotPlug */
#define H713_CNT_TAKEN       (H713_K_CNT2 + 0) /* requests taken */
#define H713_CNT_WOKEN       (H713_K_CNT2 + 1) /* wakes done */
#define H713_CNT_RESTARTS    (H713_K_CNT2 + 2) /* restarts after the first start */
#define H713_CNT_RESETS      (H713_K_CNT2 + 3) /* clean resets tried */

#define STACK_SIZE     0x00000400

#endif /* PLATFORM_MEMORY_H */
