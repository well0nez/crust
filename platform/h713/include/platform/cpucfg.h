/*
 * Copyright © 2020-2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#ifndef PLATFORM_CPUCFG_H
#define PLATFORM_CPUCFG_H

#include <util.h>
#include <platform/devices.h>

/*
 * The H713 uses the "ncat" register file with per-core control registers
 * (TF-A sunxi_cpucfg_ncat.h, the non-per-cluster path; stock BL31 cpu_on).
 * Names follow TF-A; where TF-A says "???" so do we.
 */
#define CLS_CTRL_REG0                     (DEV_CPUCFG + 0x0010)
#define CLS_CTRL_REG1                     (DEV_CPUCFG + 0x0014) /* blob 0x9010014 */
#define CACHE_CFG_REG                     (DEV_CPUCFG + 0x0024)

#define C0_CPU_CTRL_REG(n)                (DEV_CPUCFG + 0x0060 + 0x04 * (n))
#define C0_CPU_CTRL_REG_CORE_RESET        BIT(0)  /* 0 = held */
#define C0_CPU_CTRL_REG_DBG_PWRUP         BIT(8)

/* Read by the blob at 0x9010080; TF-A does not use it. Guess: WFI status. */
#define C0_CPU_STATUS_REG                 (DEV_CPUCFG + 0x0080)

#define CPU_CTRL_REG(n)                   (DEV_CPUSUBSYS + 0x0020 + 0x04 * (n))
#define CPU_CTRL_REG_AA64nAA32            BIT(0)

/* The warm entry the ARM hands us in "cpu op req" goes here (blob 0x4c40). */
#define ALT_RVBAR_LO_REG(n)               (DEV_CPUSUBSYS + 0x0040 + 0x08 * (n))
#define ALT_RVBAR_HI_REG(n)               (DEV_CPUSUBSYS + 0x0044 + 0x08 * (n))

#define CPUS_RESET_REG                    (DEV_R_CPUCFG + 0x0000)
#define CPUS_RESET                        BIT(0)

/* blob 0x4b28 writes 0 here at cluster reset (H6 name: C0_PWRON_RESET). */
#define C0_PWRON_RESET_REG                (DEV_R_CPUCFG + 0x0040)
#define C0_PWRON_RESET_REG_nH_RST         BIT(16)
#define C0_PWRON_RESET_REG_nCPUPORESET(n) BIT(0 + (n))

#define C0_PWROFF_GATING_REG              (DEV_R_CPUCFG + 0x0044)

/* Power switch, one byte-wide ladder per core: 0xfe 0xf8 0xe0 0x80 0x00 (TF-A). */
#define C0_CPUn_PWR_SWITCH_REG(n)         (DEV_R_CPUCFG + 0x0050 + 0x04 * (n))

/* TF-A "CPU_UNK": bit 0 power-on reset (0 = held), bit 1 output clamp. */
#define CPU_UNK_REG(n)                    (DEV_R_CPUCFG + 0x0070 + 0x04 * (n))
#define CPU_UNK_REG_nPORESET              BIT(0)
#define CPU_UNK_REG_CLAMP                 BIT(1)

/* blob 0x4b28 writes 0 here after the four cores (H6 name: CPU_SYS_RESET). */
#define CPU_SYS_RESET_REG                 (DEV_R_CPUCFG + 0x00a0)
#define CPU_SYS_RESET                     BIT(0)

#define CPUIDLE_EN_REG                    (DEV_R_CPUCFG + 0x0100)
#define CLOSE_FLAG_REG                    (DEV_R_CPUCFG + 0x0104)
#define PWR_SW_DELAY_REG                  (DEV_R_CPUCFG + 0x0140)
#define CONFIG_DELAY_REG                  (DEV_R_CPUCFG + 0x0144)

#endif /* PLATFORM_CPUCFG_H */
