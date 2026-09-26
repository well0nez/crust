/*
 * Copyright © 2017-2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#ifndef PLATFORM_DEVICES_H
#define PLATFORM_DEVICES_H

/*
 * Allwinner H713 (sun50iw12). Every base below is either materialised by
 * the vendor ARISC firmware (scp.bin, l.movhi/l.ori pairs; "blob") or
 * defined by our TF-A sunxi_mmap.h / kernel device tree ("tfa", "dt").
 */
#define DEV_PIO              0x02000000 /* tfa; the blob touches none of it */
#define DEV_CCU              0x02001000 /* blob 0x2001020/0x2001540/0x200171c */
#define DEV_WDOG             0x02051000 /* tfa: main watchdog, key 0x16aa */
#define DEV_UART0            0x02500000 /* tfa */
#define DEV_SYSCON           0x03000000 /* blob 0x3000007 (byte) */
#define DEV_MSGBOX           0x03003000 /* blob 0x300307c/0x300347c; dt msgbox */
#define DEV_SPINLOCK         0x03004000 /* dt hwspinlock@3004000 */
#define DEV_SID              0x03006000 /* blob 0x30060a0/0x3006200 */
#define DEV_DRAMCOM          0x04002000 /* blob 0x4002000 (DRAM standby driver) */
#define DEV_DRAMPHY_LO       0x04800000 /* blob 0x4800100.. (8 x 16 B copy, R4) */
#define DEV_DRAMCTL          0x04810000 /* blob 0x4810008/14/50c */
#define DEV_DRAMPHY          0x04820000 /* blob 0x4820002.. (25 registers) */
#define DEV_TVTOP            0x05700000 /* blob 0x5700080 */
#define DEV_RTC              0x07090000 /* blob GP words 0x7090100.. */
#define DEV_R_CPUCFG         0x07000400 /* blob 0x7000440/450/470..47c/4a0; tfa */
#define DEV_R_PRCM           0x07010000 /* blob 0x7010010..0x7010260; tfa */
#define DEV_R_TIMER          0x07020000 /* guess from the R_WDOG offset */
#define DEV_R_WDOG           0x07020400 /* blob 0x7020400/0x7020414 */
#define DEV_R_PWM            0x07020c00 /* dt r_pwm@7020c00 */
#define DEV_R_INTC           0x07021000 /* blob 0x7021000, base pointer 0x16f30 */
#define DEV_R_PIO            0x07022000 /* blob 0x7022004/0x70221f4; dt */
#define DEV_R_CIR_RX         0x07040000 /* blob 0x7040020/0x7040030; dt r_ir */
#define DEV_R_UART           0x07080000 /* blob 0x7080080.. */
#define DEV_R_I2C            0x07081400 /* tfa; not touched by the blob */
#define DEV_R_RSB            0x07083000 /* tfa; not touched by the blob */
#define DEV_HDMI_AUX         0x07091000 /* blob 0x7091014 HPD, 0x7091b04 gate, 0x7091c00 DDC */
#define DEV_CPUSUBSYS        0x08100000 /* blob 0x8100044 ALT_RVBAR_HI; tfa */
#define DEV_TIMESTAMP_STATUS 0x08110000 /* tfa/H6 layout; blob touches 0x0812000c/20 */
#define DEV_TIMESTAMP_CTRL   0x08120000 /* blob 0x812000c/0x8120020 */
#define DEV_CPUCFG           0x09010000 /* blob 0x9010014/60/6c/80; tfa */

#endif /* PLATFORM_DEVICES_H */
