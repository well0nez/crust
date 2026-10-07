/*
 * Copyright © 2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#ifndef DRIVERS_HDMI_SUN50I_H713_HDMI_H
#define DRIVERS_HDMI_SUN50I_H713_HDMI_H

#include <stdint.h>

#if CONFIG(H713_HDMI)

/** Boot: contract page, message box, startup notify. */
void h713_init(void);
/** Awake: HDMI pump (port 0), standby dispatcher (port 3), replies, HPD. */
void h713_poll(void);
/** Suspend: arm the wake sources this firmware polls itself. */
void h713_suspend_prepare(void);
/** Asleep: nonzero (the wake word) when the system should resume. */
uint32_t h713_wake_poll(void);
/** Resume: release the wake sources, LEDs. */
void h713_resume_done(void);
/** Not fatal: set error bit @bit (platform/memory.h) with detail and data. */
void h713_err(uint8_t bit, uint8_t detail, uint32_t data);
/** Count one event in the counter byte at @at (H713_CNT_*), wrapping. */
void h713_cnt(uintptr_t at);

#else

static inline void h713_init(void) {}
static inline void h713_poll(void) {}
static inline void h713_suspend_prepare(void) {}
static inline uint32_t h713_wake_poll(void) { return 0; }
static inline void h713_resume_done(void) {}
static inline void h713_err(uint8_t bit UNUSED, uint8_t detail UNUSED,
                            uint32_t data UNUSED) {}
static inline void h713_cnt(uintptr_t at UNUSED) {}

#endif

/* drivers/watchdog/sun50i-h713-wdt.c: the watchdogs' reset sequence (both
 * blocks) and the R_WDOG for the sleep net. */
void h713_wdog_reset(uintptr_t base);
uint32_t h713_rwdog_arm(uint8_t code);
void h713_rwdog_feed(void);
uint32_t h713_rwdog_disarm(void);
uint32_t h713_rwdog_state(void);

/* drivers/css/sun50i-h713-css.c: the warm entry from "cpu op req". */
void css_set_warm_entry(uint32_t lo, uint32_t hi);

/* drivers/clock/sun50i-h713-r-ccu.c: CPUS_CLK_REG as r_ccu_init() found it. */
uint32_t r_ccu_cpus_clk_at_boot(void);

#endif /* DRIVERS_HDMI_SUN50I_H713_HDMI_H */
