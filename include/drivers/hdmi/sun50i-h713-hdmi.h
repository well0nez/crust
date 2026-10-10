/*
 * Copyright © 2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#ifndef DRIVERS_HDMI_SUN50I_H713_HDMI_H
#define DRIVERS_HDMI_SUN50I_H713_HDMI_H

#include <stdbool.h>
#include <stdint.h>

/* What happens to a standby (sun50i-h713-sleep.c), as h713_sleep_event(). */
enum {
	H713_EV_CPU_OP,         /* 0x22 taken, the cluster goes down */
	H713_EV_RESTORE,        /* 0x11 */
	H713_EV_SUSPEND,        /* the system state machine suspends */
	H713_EV_RESUMED,        /* CPU0 released */
	H713_EV_POWER_OFF,      /* 0x19 taken, the system shuts down */
};

#if CONFIG(H713_SLEEP_NET)
#define H713_NET_CODE CONFIG_H713_SLEEP_NET_INTERVAL
#else
#define H713_NET_CODE 0
#endif

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
/** Power off: the board's rail enable low (false) or high again (true). */
void h713_power_off_rails(bool on);
/** The reset out of a power off: RTC GP5 "GATE" becomes "RUN1". */
void h713_power_off_reset(void);
/** Not fatal: set error bit @bit (platform/memory.h) with detail and data. */
void h713_err(uint8_t bit, uint8_t detail, uint32_t data);
/** Count one event in the counter byte at @at (H713_CNT_*), wrapping. */
void h713_cnt(uintptr_t at);

/** Record the failure (cause, detail, data), undo the sleep state, reset. */
noreturn void h713_fail(uint8_t cause, uint8_t detail, uint32_t data);
/** First thing at every start: a restart inside a standby resets. */
void h713_restart_check(uint32_t exception);
/** From h713_init(): what a first start finds of the sleep net. */
void h713_sleep_init(bool first);
/** 0x25/0x26: take the kernel's net request, if it wrote a valid one. */
void h713_sleep_request(void);
/** A standby event (H713_EV_*). */
void h713_sleep_event(uint8_t ev);
/** A step boundary (record_step): the net is fed. */
void h713_sleep_step(void);
/** Awake and asleep loops: time, feed, confirm, budgets. */
void h713_net_poll(void);
/** A hardware poll since @start not done yet: past its limit the clean
 * reset with cause 4, poll @id (H713_POLL_*) and the @value it read. */
void h713_poll_limit(uint32_t start, uint8_t id, uint32_t value);

#else

static inline void h713_init(void) {}
static inline void h713_poll(void) {}
static inline void h713_suspend_prepare(void) {}
static inline uint32_t h713_wake_poll(void) { return 0; }
static inline void h713_resume_done(void) {}
static inline void h713_power_off_rails(bool on UNUSED) {}
static inline void h713_power_off_reset(void) {}
static inline void h713_err(uint8_t bit UNUSED, uint8_t detail UNUSED,
                            uint32_t data UNUSED) {}
static inline void h713_cnt(uintptr_t at UNUSED) {}

/* Without the contract there is no standby to fail: stop, as before. */
static inline noreturn void
h713_fail(uint8_t cause UNUSED, uint8_t detail UNUSED, uint32_t data UNUSED)
{
	for (;;) {
	}
}

static inline void h713_restart_check(uint32_t exception UNUSED) {}
static inline void h713_sleep_init(bool first UNUSED) {}
static inline void h713_sleep_request(void) {}
static inline void h713_sleep_event(uint8_t ev UNUSED) {}
static inline void h713_sleep_step(void) {}
static inline void h713_net_poll(void) {}
static inline void h713_poll_limit(uint32_t start UNUSED, uint8_t id UNUSED,
                                   uint32_t value UNUSED) {}

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
