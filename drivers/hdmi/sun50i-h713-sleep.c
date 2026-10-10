/*
 * Copyright © 2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

/*
 * The standby phases of the H713 and their net (contract v5, platform/
 * memory.h). From the kernel's request to its confirm every phase has one
 * guard, and every failure crust sees ends in h713_fail():
 *   OFF       no request since the first start
 *   PRESLEEP  0x25/0x26 took a valid NET_REQ: the kernel's budget (at most
 *             120 s), with H713_SLEEP_NET the R_WDOG armed
 *   CPU_OP    0x22 taken: the cluster goes down; a restart here resets
 *   SLEEP     the system state machine suspends, sleeps, resumes; ditto
 *   RESUMING  CPU0 released, or 0x11 without a sleep: the budget again
 *   RELEASED  NET_CONFIRM with the cycle came: the net off
 *   RESET     inside h713_fail()
 *   POWEROFF  0x19 taken (H713_POWER_OFF): rails off, the wake resets; a
 *             restart here resets
 * A cpu op req without a request (a v4 TF-A and kernel) sleeps as before,
 * without net or budget, and goes straight to RELEASED after the wake.
 * The net is fed at step boundaries (record_step) and at most once a
 * second in the loops, never inside a step, so a hung poll fires it.
 */

#include <counter.h>
#include <mmio.h>
#include <spr.h>
#include <stdbool.h>
#include <stdint.h>
#include <util.h>
#include <dram/sun50i-h713-dram.h>
#include <hdmi/sun50i-h713-hdmi.h>
#include <platform/devices.h>
#include <platform/memory.h>
#include <platform/prcm.h>
#include <platform/time.h>

#define MS              CPUCLK_kHz      /* tick timer cycles per ms */
#define FEED_MS         1000
#define FAIL_WAIT_MS    2000            /* for a reset before the next path */
/*
 * The limit of a hardware poll in a standby step: what these polls wait
 * for (self-refresh, a PHY calibration, a PLL lock, a power switch) takes
 * microseconds on a working SoC, and the net's 16 s must not come first.
 */
#define POLL_LIMIT_MS   1000

static uint32_t ms_next, feed_next;
static bool     ms_armed;               /* .bss: false after a restart */

static uint32_t
budget(uint32_t s)
{
	return s < H713_NET_BUDGET_MAX ? s : H713_NET_BUDGET_MAX;
}

static bool
due(uint32_t deadline)
{
	return (int32_t)(cycle_counter_read() - deadline) >= 0;
}

static uint32_t
state(void)
{
	return mmio_read_32(H713_K_SB_STATE);
}

static uint8_t
phase(void)
{
	return state() & H713_SB_PHASE;
}

static bool
net_phase(uint8_t ph)
{
	return ph >= H713_PH_PRESLEEP && ph <= H713_PH_RESUMING;
}

static bool
armed(void)
{
	return CONFIG(H713_SLEEP_NET) && (state() & H713_SB_ARMED);
}

/* Into @ph; a net phase counts its milliseconds from 0, an end keeps them. */
static void
set_phase(uint32_t st, uint8_t ph)
{
	mmio_write_32(H713_K_SB_STATE,
	              (st & ~(H713_SB_PHASE | H713_SB_RESTARTS)) | ph);
	if (net_phase(ph))
		mmio_write_32(H713_K_NET_MS, 0);
}

/*
 * NET_MS from the tick timer, stepped from the deadline so nothing drifts;
 * WAKE_MS from the cpu op req to CPU0's release (CPU_OP and SLEEP), which
 * TF-A puts the system counter forward by (PRUEFUNG5 CS-15).
 */
static void
tick(void)
{
	uint8_t ph = phase();
	bool count = net_phase(ph);
	bool wake = ph == H713_PH_CPU_OP || ph == H713_PH_SLEEP;

	if (!ms_armed) {
		ms_next   = cycle_counter_read() + MS;
		feed_next = cycle_counter_read();
		ms_armed  = true;
	}
	while (due(ms_next)) {
		ms_next += MS;
		if (count)
			mmio_write_32(H713_K_NET_MS,
			              mmio_read_32(H713_K_NET_MS) + 1);
		if (wake)
			mmio_write_32(H713_K_WAKE_MS,
			              mmio_read_32(H713_K_WAKE_MS) + 1);
	}
}

static void
feed(void)
{
	if (CONFIG(H713_TEST_STARVE_NET) && phase() == H713_PH_SLEEP &&
	    mmio_read_32(H713_K_NET_MS) >= 2000)
		return;
	h713_rwdog_feed();
	feed_next = cycle_counter_read() + FEED_MS * MS;
}

/*
 * The net off (feed, MODE off, read back); one that still reads armed is
 * recorded (error bit 17, @where 0 release, 1 first start). After a
 * standby it keeps SB_STATE bit 4 and stays fed: a reset of a box that is
 * awake and well would be worse.
 */
static bool
disarm(uint8_t where)
{
	uint32_t mode = h713_rwdog_disarm();

	if (mode & 1)
		h713_err(H713_ERR_RWDOG_DISARM, where, mode);
	return mode & 1;
}

/* The net off, the phase RELEASED. */
static void
release(void)
{
	uint32_t st = state() & ~(H713_SB_ARMED | H713_SB_TAKEN);

	if (CONFIG(H713_SLEEP_NET) &&
	    (state() & (H713_SB_ARMED | H713_SB_TAKEN)) && disarm(0))
		st |= H713_SB_ARMED;
	set_phase(st, H713_PH_RELEASED);
}

void
h713_sleep_request(void)
{
	uint32_t req = mmio_read_32(H713_A_NET_REQ);
	uint32_t bud = mmio_read_32(H713_A_NET_BUDGET);
	uint32_t cycle = mmio_read_32(H713_K_CYCLE) + 1, st = state(), pll;
	uint8_t  fmt = req >> 16, ph = st & H713_SB_PHASE, why = 0;

	if (!req)
		return;         /* the kernel's prepare round trip, or v4 */
	if (req >> 24 != H713_NET_REQ_MARK || !fmt || fmt > H713_NET_FORMAT)
		why = 1;
	else if (((req >> 8) & 0xff) != (cycle & 0xff))
		why = 2;
	else if (ph != H713_PH_OFF && ph != H713_PH_RELEASED)
		why = 3;
	else if (!(bud & 0xffff) || !(bud >> 16))
		why = 4;
	if (why) {
		h713_err(H713_ERR_NET_REQ, why, req);
		return;
	}
	/* The DRAM clock the self-refresh exit will set: on a mismatch the
	 * request is not taken, and the kernel refuses the standby. */
	why = !h713_dram_clk_ok(&pll);
	mmio_write_32(H713_K_DRAM_PLL, pll);
	if (why) {
		h713_err(H713_ERR_DRAM_PLL, 0, pll);
		return;
	}
	mmio_write_32(H713_K_CYCLE, cycle);
	/* a wake word no 0x11 took is not this cycle's */
	mmio_write_32(H713_K_WAKE_KEPT, 0);
	mmio_write_32(H713_K_NET_ACC,
	              budget(bud & 0xffff) | budget(bud >> 16) << 16);
	st = (st & H713_SB_CAUSE) | H713_SB_TAKEN | (cycle & 0xff) << 8;
	if (CONFIG(H713_SLEEP_NET)) {
		uint32_t rb = h713_rwdog_arm(H713_NET_CODE);

		mmio_write_32(H713_K_NET_RB, rb);
		if (rb & 1)
			st |= H713_SB_ARMED;
		feed_next = cycle_counter_read() + FEED_MS * MS;
	}
	set_phase(st, H713_PH_PRESLEEP);
	h713_cnt(H713_CNT_TAKEN);
}

void
h713_sleep_event(uint8_t ev)
{
	uint32_t st = state(), cycle;
	uint8_t  ph = st & H713_SB_PHASE;

	switch (ev) {
	case H713_EV_CPU_OP:
		mmio_write_32(H713_K_WAKE_MS, 0);
		if (ph != H713_PH_PRESLEEP) {
			/* no request taken: a v4 TF-A; counted, no budget */
			cycle = mmio_read_32(H713_K_CYCLE) + 1;
			mmio_write_32(H713_K_CYCLE, cycle);
			st = (st & ~(H713_SB_TAKEN | H713_SB_CYCLE)) |
			     (cycle & 0xff) << 8;
		}
		set_phase(st, H713_PH_CPU_OP);
		break;
	case H713_EV_SUSPEND:
		/* a power off keeps its own phase down to the reset */
		if (ph != H713_PH_POWEROFF)
			set_phase(st, H713_PH_SLEEP);
		break;
	case H713_EV_POWER_OFF:
		set_phase(st, H713_PH_POWEROFF);
		break;
	case H713_EV_RESUMED:
		h713_cnt(H713_CNT_WOKEN);
		if (st & H713_SB_TAKEN)
			set_phase(st, H713_PH_RESUMING);
		else
			release();
		break;
	case H713_EV_RESTORE:
		/* 0x11 without a sleep: the suspend was given up after 0x25 */
		if (ph == H713_PH_PRESLEEP)
			set_phase(st, H713_PH_RESUMING);
		break;
	}
}

void
h713_sleep_step(void)
{
	tick();
	if (armed())
		feed();
}

void
h713_net_poll(void)
{
	uint32_t conf, acc, ms;
	uint8_t  ph;

	tick();
	if (armed() && due(feed_next))
		feed();
	ph = phase();
	if (ph != H713_PH_PRESLEEP && ph != H713_PH_RESUMING)
		return;
	conf = mmio_read_32(H713_A_NET_CONFIRM);
	if (conf >> 16 == H713_NET_CONFIRM_MARK &&
	    (conf & 0xff) == (state() & H713_SB_CYCLE) >> 8) {
		release();
		return;
	}
	acc = mmio_read_32(H713_K_NET_ACC);
	ms  = mmio_read_32(H713_K_NET_MS);
	if (ph == H713_PH_PRESLEEP && ms >= (acc & 0xffff) * 1000)
		h713_fail(H713_CAUSE_PRESLEEP, 0, ms);
	if (ph == H713_PH_RESUMING && ms >= (acc >> 16) * 1000)
		h713_fail(H713_CAUSE_RESUMING, 1, ms);
}

void
h713_sleep_init(bool first)
{
	uint32_t found;

	if (!CONFIG(H713_SLEEP_NET) || !first)
		return;
	found = h713_rwdog_state();
	mmio_write_32(H713_K_NET_FOUND, found);
	if (found & 1) {
		h713_err(H713_ERR_RWDOG_FOUND, 0, found);
		disarm(1);
	}
}

void
h713_restart_check(uint32_t exception)
{
	uint32_t st = state(), exc = 0, n;
	uint8_t  ph = st & H713_SB_PHASE;

	if (!mmio_read_32(H713_DIAG_INITS))
		return;         /* the first start after a load: K is not ours */
	h713_cnt(H713_CNT_RESTARTS);
	if (exception)
		exc = exception << 24 | mfspr(SPR_SYS_EPCR_ADDR(0));
	/*
	 * In CPU_OP the cluster goes down from the awake loop, and a restart
	 * there would run the boot initialisation under it; asleep, the state
	 * machine would wait for a wake with the system half down. Both reset.
	 */
	if (ph == H713_PH_CPU_OP || ph == H713_PH_SLEEP ||
	    ph == H713_PH_RESET || ph == H713_PH_POWEROFF)
		h713_fail(H713_CAUSE_RESTART, ph, exc);
	if (ph != H713_PH_PRESLEEP && ph != H713_PH_RESUMING)
		return;
	/* awake with the net: go on feeding, but not for an exception loop */
	n = ((st & H713_SB_RESTARTS) >> 6) + 1;
	if (n >= 3)
		h713_fail(H713_CAUSE_LOOP, ph, exc);
	mmio_write_32(H713_K_SB_STATE, (st & ~H713_SB_RESTARTS) | n << 6);
}

void
h713_poll_limit(uint32_t start, uint8_t id, uint32_t value)
{
	if (cycle_counter_read() - start >= POLL_LIMIT_MS * MS)
		h713_fail(H713_CAUSE_POLL, id, value);
}

static void
wait_ms(uint32_t ms)
{
	uint32_t end = cycle_counter_read() + ms * MS;

	while (!due(end)) {
		/* nothing */
	}
}

noreturn void
h713_fail(uint8_t cause, uint8_t detail, uint32_t data)
{
	static const uint8_t err_bit[] = {
		[H713_CAUSE_PRESLEEP] = H713_ERR_BUDGET,
		[H713_CAUSE_RESUMING] = H713_ERR_BUDGET,
		[H713_CAUSE_DRAM]     = H713_ERR_DRAM_PHY,
		[H713_CAUSE_POLL]     = H713_ERR_POLL,
		[H713_CAUSE_RESTART]  = H713_ERR_RESTART,
		[H713_CAUSE_CPU_OP]   = H713_ERR_CPU_OP,
		[H713_CAUSE_LOOP]     = H713_ERR_RESTART,
		[H713_CAUSE_PLL_LOCK] = H713_ERR_PLL_LOCK,
	};
	uint32_t tries = 0;

	cycle_counter_init();   /* this may run before system.c set it up */
	h713_err(cause < ARRAY_SIZE(err_bit) ? err_bit[cause] : H713_ERR_RESTART,
	         detail, data);
	mmio_write_32(H713_K_SB_STATE,
	              (state() & ~(H713_SB_PHASE | H713_SB_CAUSE)) |
	              H713_PH_RESET | (uint32_t)cause << 16);
	h713_cnt(H713_CNT_RESETS);
	/*
	 * The R_PRCM sleep state this image can leave, in the deep-net
	 * design's order: VDD_SYS out of reset and ungated, the PLL LDO on,
	 * the DRAM pad hold (dram.c) off. GP5 stays as it is.
	 */
	if (CONFIG(H713_DEEP_VDD_SYS))
		mmio_set_32(VDD_SYS_RESET_REG, VDD_SYS_RESET);
	if (CONFIG(H713_DEEP_AVCC_GATING))
		mmio_clr_32(VDD_SYS_PWROFF_GATING_REG,
		            AVCC_GATING | UNK_GATING_BIT_08);
	if (CONFIG(H713_DEEP_PLL_LDO)) {
		/* the key field cleared first, as r-ccu.c's pll_ldo() */
		uint32_t v = (mmio_read_32(PLL_CTRL_REG1) &
		              ~PLL_CTRL_REG1_KEY_FIELD) |
		             PLL_CTRL_REG1_KEY | PLL_CTRL_REG1_LDO_EN;

		mmio_write_32(PLL_CTRL_REG1, v);
		mmio_write_32(PLL_CTRL_REG1, v);
		wait_ms(1);
	}
	mmio_clr_32(VDD_SYS_PWROFF_GATING_REG, 0x3);
	/*
	 * Without the net, the SoC watchdog, whose sequence and reset are
	 * measured; with it, first the R_WDOG (stage S0c' measures it), then
	 * the SoC watchdog if nothing happened. FAIL_PATH says which.
	 */
	for (;;) {
		if (CONFIG(H713_SLEEP_NET)) {
			mmio_write_32(H713_K_FAIL_PATH, 1 | (++tries & 0xff) << 8);
			h713_wdog_reset(DEV_R_WDOG);
			wait_ms(FAIL_WAIT_MS);
			mmio_set_32(H713_K_ERR, BIT(H713_ERR_RWDOG_RESET));
		}
		mmio_write_32(H713_K_FAIL_PATH, 2 | (++tries & 0xff) << 8);
		h713_wdog_reset(DEV_WDOG);
		wait_ms(FAIL_WAIT_MS);
	}
}
