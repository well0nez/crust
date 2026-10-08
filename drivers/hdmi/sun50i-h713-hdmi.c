/*
 * Copyright © 2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

/*
 * The vendor ARISC firmware's contract on the H713 as sun50i-h713-arisc
 * (kernel 0091*) and TF-A (FB-STANDBY tfa-0001) speak it; every row with
 * its evidence is in tables/arisc-contract.csv. ARM -> ARISC: user1 ports
 * 0 (BOP pump) and 3 (standby dispatcher); ARISC -> ARM: user0 channels 1
 * (framed replies) and 3 (notify, answers). Shared variables sit in the
 * contract page with the vendor build's relative distances. Bytes the ARM
 * writes with writeb() are read through mmio_read_8() (address ^ 3: the
 * AR100 data bus swaps every word) and written the same way.
 */

#include <cir.h>
#include <counter.h>
#include <css.h>
#include <debug.h>
#include <delay.h>
#include <device.h>
#include <error.h>
#include <mmio.h>
#include <msgbox.h>
#include <scpi_protocol.h>
#include <stdbool.h>
#include <stdint.h>
#include <system.h>
#include <timeout.h>
#include <util.h>
#include <version.h>
#include <cir/sunxi-cir.h>
#include <gpio/sunxi-gpio.h>
#include <hdmi/sun50i-h713-hdmi.h>
#include <msgbox/sunxi-msgbox.h>
#include <platform/devices.h>
#include <platform/memory.h>
#include <platform/prcm.h>
#include <platform/time.h>

/* ---- the contract page (AR100 view; ARM = + SRAM_A2_OFFSET) ------------- */
#define SH_MAGIC        (H713_SHARED_BASE + 0x000) /* "H713" as ARM reads it */
#define SH_VERSION      (H713_SHARED_BASE + 0x004)
#define SH_STATUS       (H713_SHARED_BASE + 0x008)
#define SH_WAKE         (H713_SHARED_BASE + 0x00c)
#define SH_HPD_COUNTER  (H713_SHARED_BASE + 0x010) /* 3 bytes, g_Data - 16 */
#define SH_GDATA        (H713_SHARED_BASE + 0x020)
#define SH_EDID_READY   (SH_GDATA + 2)
#define SH_EDID_GATE    (SH_GDATA + 3)
#define SH_EDID_MASK    (SH_GDATA + 4)
#define SH_PORTMAP      (SH_GDATA + 12)            /* 3 records of 8 bytes */
#define SH_SCRATCH      (SH_GDATA + 228)           /* 65 bytes */
#define SH_RPM_STATE    (H713_SHARED_BASE + 0x180)
#define SH_RPM_BUF      (SH_RPM_STATE + 5)         /* 67 bytes */
#define SH_RO_SIZE      (H713_SHARED_BASE + 0x1fc) /* .text + .rodata bytes, v4 */
#define RPM_BUF_LEN     67                         /* the kernel's PAYLOAD_MAX */

#define H713_MAGIC      0x33313748                 /* 'H','7','1','3' */
/* 2: diagnostic words; 3: step/exception records, sleep length, wake timer
 * and the build word (platform/memory.h); 4: the size of the part of the
 * image that never changes (+0x1fc), which the kernel compares with the
 * file; 5: the K and A blocks (standby phases, the net request, error bits
 * and counters). A version 4 reader still reads everything it knows. */
#define H713_VERSION    0x00000005
#define H713_COMPAT     (4 | H713_NET_FORMAT << 8)
#if CONFIG(H713_DRAM_SELF_REFRESH)
#define BUILD2_DRAM_MHZ CONFIG_H713_DRAM_CLK
#else
#define BUILD2_DRAM_MHZ 0
#endif
#if CONFIG(H713_SLEEP_NET)
#define BUILD2_NET_CODE CONFIG_H713_SLEEP_NET_INTERVAL
#else
#define BUILD2_NET_CODE 0
#endif

/* Diagnostic words (platform/memory.h), kept across firmware restarts. */
#define DIAG_LEWIN_EN   (H713_DIAG_BASE + 0x00)    /* written by start.S */
#define DIAG_CPUS_CLK   (H713_DIAG_BASE + 0x0c)
#define DIAG_TTCR       (H713_DIAG_BASE + 0x10)
#define DIAG_INITS      H713_DIAG_INITS
#define DIAG_INTC_80    (H713_DIAG_BASE + 0x18)
#define DIAG_INTC_88    (H713_DIAG_BASE + 0x1c)
#define DIAG_MIPS_N     (H713_DIAG_BASE + 0x20)    /* frames from the MIPS */
#define DIAG_MIPS_LAST  (H713_DIAG_BASE + 0x24)    /* their first 4 bytes */
#define DIAG_ARM_TYPE   (H713_DIAG_BASE + 0x28)    /* ARM frames with type != 0 */
#define ST_RUNNING      BIT(0)
#define ST_HANDSHAKE    BIT(1)
#define ST_EDID_MODULE  BIT(2)
#define ST_ASLEEP       BIT(3)

#define PORTS           3
#define REC_PHYS        1
#define REC_PIN         2
#define REC_PIN_BIT     3
#define REC_REVERSE     4
#define SCRATCH_LEN     65
#define EDID_BLOCK      256

/* ---- message box channels (drivers/msgbox/sun50i-h713-msgbox.c) --------- */
/*
 * The display MIPS reaches the ARISC through sub-block 1 of the ARISC's own
 * user (user1, 0x03003500): the pump's receive address is ((port +
 * 0x00c00d1c) + (ch - 1) * 64) << 2 (blob 0x7e4c), so channel 2 is that
 * sub-block. Its frames go to the "DispMips" handler 0x1180c -> 0x1242c:
 * payload[0] 0x16 is PullHotPlug(action payload[2], port payload[1]) (0x12330),
 * 0x17 is ResetEDIDModule (0x120f0); nothing is answered. Which of the four
 * ports the MIPS writes is not read out of its firmware, so all four are
 * polled; only the ARISC reads its own user, nothing else is consumed.
 */
#define MIPS_FIFO_STAT(p) (DEV_MSGBOX + 0x400 + 0x100 + 0x60 + 4 * (p))
#define MIPS_MSG_DATA(p)  (DEV_MSGBOX + 0x400 + 0x100 + 0x70 + 4 * (p))
#define MIPS_PASS_WORDS 16 /* per port and main loop pass */
/*
 * Words without the marker outside a frame: the blob reads them and drops
 * them without a trace (0x80c4/0x80c8). The HY310's MIPS sends three with
 * every 0x16 (08.10.2026, R1 image: three 0x16, CNT1 0x03000900, no error
 * bit, DIAG_MIPS_N 3); where they sit is not measured. Up to MIPS_STRAYS
 * of them between two valid 0x16, and before the first, are taken as its
 * companions and only recorded (MIPS_STRAY_INFO); any further one counts.
 */
#define MIPS_STRAYS     3
#define STRAY_TAKEN     (H713_K_MIPS_STRAY_INFO + 2)
#define STRAY_COUNTED   (H713_K_MIPS_STRAY_INFO + 3)
#define RX_PORT(p)      (2 * (p))      /* ARM -> ARISC, user1 port p */
#define TX_CHAN(c)      (2 * (c) + 1)  /* ARISC -> ARM, user0 channel c */
#define PORT_HDMI       RX_PORT(0)
#define PORT_STANDBY    RX_PORT(3)
#define CHAN_REPLY      TX_CHAN(1)
#define CHAN_NOTIFY     TX_CHAN(3)

/* ---- the HDMI aux block (RTC domain) ------------------------------------ */
#define AUX_HPD_REG     (DEV_HDMI_AUX + 0x014) /* active-low, one bit per pin */
#define AUX_UNK_020     (DEV_HDMI_AUX + 0x020) /* blob 0x1105c(1): = 3 */
#define AUX_UNK_030     (DEV_HDMI_AUX + 0x030) /* blob 0x10fac(0x18): = 0 */
#define AUX_UNK_034     (DEV_HDMI_AUX + 0x034) /* blob 0x10fac(0x18): = 0x18 */
#define AUX_UNK_038     (DEV_HDMI_AUX + 0x038) /* blob 0x10fac(0x18): = 0x18 */
#define AUX_UNK_B00     (DEV_HDMI_AUX + 0xb00)
#define AUX_GATE_REG    (DEV_HDMI_AUX + 0xb04) /* EDID/DDC output, bit per port */
#define AUX_UNK_B08     (DEV_HDMI_AUX + 0xb08)
#define AUX_DDC_RAM(p)  (DEV_HDMI_AUX + 0xc00 + 0x100 * (p))
#define R_PIO_PL_CFG0   (DEV_R_PIO + 0x000)    /* PL0..PL7 function */
#define R_PIO_PL_CFG1   (DEV_R_PIO + 0x004)    /* PL8..PL15 function */
#define R_PIO_PL_DATA   (DEV_R_PIO + 0x010)
/* D1-style banks (kernel pinctrl-sun50i-h713-r.c): drive 4 bits per pin. */
#define R_PIO_PL_DRV0   (DEV_R_PIO + 0x014)    /* PL0..PL7 drive */
#define R_PIO_PL_DRV1   (DEV_R_PIO + 0x018)    /* PL8..PL15 drive */
#define R_PIO_PL_PULL0  (DEV_R_PIO + 0x024)    /* PL0..PL15 pull */
#define R_PIO_EINT_STATUS (DEV_R_PIO + 0x214) /* PL bank (IRQ bank 0), W1C */
#define R_PIO_EINT_DEB  (DEV_R_PIO + 0x218)    /* PL bank debounce clock */
/*
 * The PL bank's EINT status at four points of h713_resume_done() (0032;
 * read through /dev/mem): before the switch back, right after it, after
 * the settle time, and one settle time after the clear.
 */
#define EINT_SEEN_BEFORE  (H713_SEEN_EINT + 0x0)
#define EINT_SEEN_SWITCH  (H713_SEEN_EINT + 0x4)
#define EINT_SEEN_SETTLED (H713_SEEN_EINT + 0x8)
#define EINT_SEEN_AFTER   (H713_SEEN_EINT + 0xc)

#define BOP_MARKER      0xa5
#define BOP_MAX_LEN     128
#define REPLY_TYPE      1
#define FRAME_HEAD      0xff
#define FRAME_TAIL      0xfe
#define SB_STATE        2
#define TYPE_NOTIFY     0x90 /* startup notify / its acknowledgement */
#define TYPE_RESTORE    0x11 /* cpu0 restore finished -> wake word */
#define TYPE_CPU_OP     0x22 /* cpu op req: {mpidr, entry, 3, 3, 3} */
#define TYPE_WAKE_CLR   0x25
#define TYPE_WAKE_SET   0x26
#define RESULT_UNKNOWN  0xfd
#define HPD_RESET_TICKS 84    /* ~840 ms low, then the rising edge */
#define HPD_TICK_US     10000
#define WAKE_KEY        0x95  /* GIC SPI 149, PL EINT */
#define WAKE_IR         0x9b  /* GIC SPI 155, s_cir */
#define WAKE_TIMER      0xc0000000 /* the vendor's timer callback 0xc7e0 */
#define WAKE_SRC_TIMER  3U    /* 0x26 word: type in 31..30, 3 = timer, ms */
/*
 * From the cpu op req to the cluster reset. The vendor firmware spends this
 * on its standby work (parameter parse, the 0x4f70 calibration, wake sources,
 * DRAM) before 0x4b28; here it is a plain wait, so the requesting core has
 * printed its NOTICE line, released TF-A's locks and reached WFI - a reset
 * in the middle of that would leave a lock taken for the warm boot.
 */
#define CPU_OP_DELAY_US 50000
#define KEY_POLL_US     20000
#define KEY_DEBOUNCE    3
#define KEY_RELEASE_MS  3000  /* the longest a held key delays the resume */
#define SB_FRAME_US     100000 /* a port 3 frame not complete by then is dropped */

static uint8_t  edid[2 * EDID_BLOCK];
static bool     edid_valid;	/* .bss: no upload since this start */
/*
 * Two transmit queues, one per ARM-RX channel, so a word the ARM does not
 * read on one channel never holds up the other: framed replies on channel 1
 * (RequestEDID queues four frames of 20 words), the notify and the standby
 * answers on channel 3.
 */
enum { Q_REPLY, Q_NOTIFY, Q_COUNT };
#define TXQ_STALL_US    1000000 /* a reply nobody read for 1 s is stale */
static uint32_t txq_reply[128], txq_notify[32];	/* powers of two */
static uint32_t *const q_buf[Q_COUNT] = { txq_reply, txq_notify };
static const uint8_t   q_size[Q_COUNT] = { ARRAY_SIZE(txq_reply),
	                                   ARRAY_SIZE(txq_notify) };
static const uint8_t   q_chan[Q_COUNT] = { CHAN_REPLY, CHAN_NOTIFY };
static uint8_t  q_head[Q_COUNT], q_tail[Q_COUNT];
static uint32_t q_stall[Q_COUNT];
static uint8_t  seq;
static uint8_t  pump_state, pump_len, pump_idx, pump_type;
static uint8_t  sb_n, sb_count;
static uint32_t sb_hdr, sb_data[8];
static uint32_t hpd_deadline, key_deadline, ms_deadline, sb_deadline;
static bool     wake_armed;	/* .bss: false again after a firmware restart */
/*
 * The power key asleep: ENTRY until it reads high once (a key held when
 * the standby began is not a press), ARMED until KEY_DEBOUNCE low samples,
 * OPEN until it is released or KEY_RELEASE_MS are up.
 */
enum { KEY_ENTRY, KEY_ARMED, KEY_OPEN };
static uint8_t  key_state, key_count;
static uint32_t key_open_ms;
static uint32_t pl_cfg_saved;
/*
 * PL0..PL7 as Linux left them: the power key PL4 is a gpio-keys interrupt
 * there (EINT function), the LEDs PL0/PL1 plain outputs. Taking PL4 as an
 * input and the LEDs for the sleep changes their function, drive and pull,
 * and gpio_put() gives none of that back.
 */
static uint32_t pl_cfg0_saved, pl_drv0_saved, pl_drv1_saved, pl_pull0_saved;
static uint32_t pl_data_saved;
static const struct device *cir;
static struct {
	uint8_t state, len, idx, buf[8];
} mips[4];

#define PL(pin, m, p) { .dev = &r_pio.dev, .id = SUNXI_GPIO_PIN(0, pin), \
	.drive = DRIVE_10mA, .mode = m, .pull = p }
static const struct gpio_handle key_pin  = PL(CONFIG_H713_POWER_KEY_PIN, MODE_INPUT, PULL_UP);
/*
 * The standby LEDs (Kconfig, -1 = none): red on and blue off asleep, the
 * other way round after the wake. Only the HY310's are known (its stock
 * U-Boot prj node: PL0 red, PL1 blue), where the LED follows PB5 anyway;
 * a board without that evidence gets no pin driven (PRUEFUNG5 N-08).
 */
#define LED_RED         CONFIG_H713_LED_RED_PIN
#define LED_BLUE        CONFIG_H713_LED_BLUE_PIN
#define LED_PIN(pin)    ((pin) >= 0 ? (pin) : 0)
#define LED_BIT(pin)    ((pin) >= 0 ? BIT(LED_PIN(pin)) : 0)
#define LED_MASK        (LED_BIT(LED_RED) | LED_BIT(LED_BLUE))
static const struct gpio_handle led_red  = PL(LED_PIN(LED_RED), MODE_OUTPUT, PULL_NONE);
static const struct gpio_handle led_blue = PL(LED_PIN(LED_BLUE), MODE_OUTPUT, PULL_NONE);

#if CONFIG(H713_IR_WAKE_NECX)
/* cir_param of the HY310 and HY300 Pro vendor DTs, in crust's NECX order. */
#define IR(addr, cmd) ((((addr) & 0xff) << 16) | ((addr) & 0xff00) | (cmd))
static const uint32_t ir_wake_codes[] = {	/* hy310-stock.dts 2569-2611, in its order */
	IR(0xff00, 0x14), IR(0xff00, 0x45), IR(0x2992, 0xf2), IR(0x9f00, 0x57),
	IR(0x4cb3, 0xdc), IR(0xff11, 0x18), IR(0xdd22, 0xdc), IR(0xbc00, 0x0d),
	IR(0x4040, 0x4d), IR(0xfb04, 0x08), IR(0xfc03, 0x00), IR(0xbf00, 0x00),
	IR(0xfb04, 0xea), IR(0xbf00, 0x42), IR(0xff00, 0xeb), IR(0x00ff, 0x14),
	IR(0xfd02, 0x1d), IR(0xfe01, 0x00), IR(0x0707, 0x02),
};
#endif

extern char __data_start[];	/* the end of .text and .rodata (scp.ld.S) */
extern char __bss_start[];	/* the end of the image */

#define sh_rd8(a)    mmio_read_8(a)
#define sh_wr8(a, v) mmio_write_8(a, v)
#define sh_status(set, clr) \
	mmio_write_32(SH_STATUS, (mmio_read_32(SH_STATUS) & ~(clr)) | (set))

/*
 * Has the tick timer reached @deadline? The signed difference holds for
 * anything less than 2^31 cycles (89 s) away on either side. timeout_expired()
 * waits for the top bits to agree, so a deadline that was passed while the
 * timer crossed bit 31 - a long suspend pass, the deep sleep's clock waits -
 * would read "not yet" for up to 179 s, the power key included.
 */
static bool
passed(uint32_t deadline)
{
	return (int32_t)(cycle_counter_read() - deadline) >= 0;
}

/* ---- the record in the K block (platform/memory.h) ------------------------ */
void
h713_err(uint8_t bit, uint8_t detail, uint32_t data)
{
	mmio_write_32(H713_K_ERR, mmio_read_32(H713_K_ERR) | BIT(bit));
	mmio_write_32(H713_K_ERR_DETAIL, (uint32_t)bit << 24 | detail << 16 |
	              (mmio_read_32(H713_DIAG_STEP) & 0xffff));
	mmio_write_32(H713_K_ERR_DATA, data);
}

void
h713_cnt(uintptr_t at)
{
	uintptr_t word = at & ~3U;
	uint32_t  sh   = 8 * (at & 3);
	uint32_t  v    = mmio_read_32(word);

	mmio_write_32(word, (v & ~(0xffU << sh)) | (((v >> sh) + 1) & 0xff) << sh);
}

/* ---- transmit queues: never block on an 8-deep FIFO ---------------------- */
static bool
q_empty(uint8_t q)
{
	return q_head[q] == q_tail[q];
}

static uint8_t
q_free(uint8_t q)
{
	return q_size[q] - 1 - ((q_tail[q] - q_head[q]) & (q_size[q] - 1));
}

static void
q_put(uint8_t q, uint32_t w)
{
	uint8_t next = (q_tail[q] + 1) & (q_size[q] - 1);

	if (next == q_head[q]) {
		h713_err(H713_ERR_TXQ_FULL, q, w);
		h713_cnt(H713_CNT_TXQ_FULL);
		warn("H713: tx queue %u full", q); return;
	}
	if (q_empty(q))
		q_stall[q] = timeout_set(TXQ_STALL_US);
	q_buf[q][q_tail[q]] = w;
	q_tail[q] = next;
}

static void
q_drain(uint8_t q)
{
	while (!q_empty(q)) {
		if (msgbox_send(&msgbox.dev, q_chan[q], q_buf[q][q_head[q]]))
			break;
		q_head[q] = (q_head[q] + 1) & (q_size[q] - 1);
		q_stall[q] = timeout_set(TXQ_STALL_US);
	}
	/*
	 * Replies are only ever asked for: one the ARM has not taken for a
	 * second belongs to a request it gave up on (an aborted RequestEDID),
	 * and the pump reads no new request while it waits.
	 */
	if (q == Q_REPLY && !q_empty(q) && passed(q_stall[q])) {
		warn("H713: stale reply dropped");
		h713_cnt(H713_CNT_STALE_REPLY);
		q_head[q] = q_tail[q];
	}
}

/* Framed sender (blob 0x118e4/0x81c8): header, zero word, payload LSB first. */
static void
reply_frame(uint8_t *payload, uint8_t len)
{
	uint32_t sum = 0;

	payload[0] = FRAME_HEAD;
	for (uint8_t i = 1; i + 6 <= len; ++i)
		sum += payload[i];
	payload[len - 2] = (uint8_t)-sum;
	payload[len - 1] = FRAME_TAIL;

	q_put(Q_REPLY, BOP_MARKER | seq << 8 | REPLY_TYPE << 16 |
	      (uint32_t)len << 24);
	q_put(Q_REPLY, 0);
	seq = seq % 255 + 1;
	for (uint8_t i = 0; i < len; i += 4) {
		uint32_t w = 0;

		for (uint8_t b = 0; b < 4 && i + b < len; ++b)
			w |= (uint32_t)payload[i + b] << (8 * b);
		q_put(Q_REPLY, w);
	}
}

/* ---- the HDMI aux block --------------------------------------------------- */
/*
 * Is the aux block clocked and out of reset? ResetEDIDModule's first two
 * stores (R_PRCM 0x120 bit 16, 0x124 bit 31) do that, and from the ARM an
 * access before them hangs the SoC. Hardware state, unlike the status
 * bit, which a firmware restart clears (PRUEFUNG5 F9).
 */
static bool
aux_clocked(void)
{
	return (mmio_read_32(R_HDMI_AUX_GATE_REG) & BIT(16)) &&
	       (mmio_read_32(R_HDMI_AUX_CLK_REG) & BIT(31));
}

/* Every way into the aux block asks first: refused, recorded and counted
 * without the clock (@who: 1 publish, 2 HPD pin, 3 RequestEDID). A MIPS
 * 0x16 is only dropped (mips_frame). */
static bool
aux_ok(uint8_t who)
{
	if (aux_clocked())
		return true;
	h713_err(H713_ERR_AUX_CLOCK, who, mmio_read_32(R_HDMI_AUX_GATE_REG));
	h713_cnt(H713_CNT_MIPS_DROP);
	return false;
}

static void
aux_gate(uint8_t mask, bool open)
{
	uint32_t v = mmio_read_32(AUX_GATE_REG);

	mmio_write_32(AUX_GATE_REG, open ? (v | mask) : (v & ~(uint32_t)mask));
}

static void
hpd_pin(uint8_t port, bool high)
{
	uint32_t bit = BIT(sh_rd8(SH_PORTMAP + 8 * port + REC_PIN) & 7);
	uint32_t v;

	if (!aux_ok(2))
		return;
	v = mmio_read_32(AUX_HPD_REG);
	if (!sh_rd8(SH_EDID_GATE))
		high = false; /* blob 0x121e4: EDID-HPD LOW while gated */
	mmio_write_32(AUX_HPD_REG, high ? (v & ~bit) : (v | bit));
}

/*
 * Publish (blob 0x11d88): block by port mask, the port's physical-address
 * byte into the HDMI vendor block, checksum, gate closed while writing.
 * Only a store that holds an upload: the blob also publishes the empty one
 * from ResetEDIDModule, and a source read an all-zero EDID (HY310,
 * 07.10.2026; PRUEFUNG5 F12) - the gate stays closed until fragment 7.
 */
static void
publish(void)
{
	uint8_t mask = sh_rd8(SH_EDID_MASK);

	if (!sh_rd8(SH_EDID_READY) || !edid_valid || !aux_ok(1))
		return;
	aux_gate(0x7, false);
	for (uint8_t port = 0; port < PORTS; ++port) {
		const uint8_t *src = &edid[((mask >> port) & 1) * EDID_BLOCK];
		uint8_t blk[EDID_BLOCK], phys, sum = 0;

		for (int i = 0; i < EDID_BLOCK; ++i)
			blk[i] = src[i];
		phys = sh_rd8(SH_PORTMAP + 8 * port + REC_PHYS);
		/*
		 * CEA-861 data block collection: tag 3 with IEEE 00 0c 03. It
		 * ends at the DTD offset (byte 2 of the extension, whose tag
		 * is 2) and before the checksum; a block that runs past that
		 * end is not followed, so a malformed upload never reaches
		 * beyond blk[] (PRUEFUNG5 F1). With len >= 5 inside the end,
		 * the byte written, i + 4, lies inside it too.
		 */
		int end = 128 + blk[128 + 2] < 255 ? 128 + blk[128 + 2] : 255;

		for (int i = 128 + 4; blk[128] == 0x02 && blk[128 + 2] >= 4 &&
		     i < end;) {
			uint8_t tag = blk[i] >> 5, len = blk[i] & 0x1f;

			if (i + 1 + len > end)
				break;
			if (tag == 3 && len >= 5 && blk[i + 1] == 0x03 &&
			    blk[i + 2] == 0x0c && blk[i + 3] == 0x00)
				blk[i + 4] = phys;
			i += 1 + len;
		}
		for (int i = 128; i < 255; ++i)
			sum += blk[i];
		blk[255] = (uint8_t)-sum;
		for (int i = 0; i < EDID_BLOCK; i += 4)
			mmio_write_32(AUX_DDC_RAM(port) + i,
			              blk[i] | blk[i + 1] << 8 |
			              blk[i + 2] << 16 | (uint32_t)blk[i + 3] << 24);
	}
	aux_gate(0x7, true);
	sh_wr8(SH_EDID_GATE, 1);
}

/*
 * ResetEDIDModule (blob 0x120f0..0x121dc), store for store:
 *   0x12108 R_PRCM 0x120 = 0; 0x12114 delay 5; 0x12124 0x120 = 0x00010000
 *   (reset released, bit 0 stays clear); 0x12130 0x124 = 0x80000000;
 *   0x12140..0x12150 PL_CFG1 = (v & 0xff) | 0x22222200 (PL8/PL9 kept, PL10..15
 *   function 2 -- PL9 is the kernel's IR pin); 0x12164 b08 = 0xc0c0c0c0;
 *   0x12170..0x1217c b04 &= 0x00ffffff; g_Data +2/+3/+4 = 0; 0x12198 the gate
 *   helper 0x11cac(7, 1), which closes while +2 is 0 (0x11d54); 0x121a8 b00 =
 *   0x00a0a0a0; +2 = 1 and publish(7); then 0x10fac(0x18): +0x030 = 0, +0x034
 *   = +0x038 = 0x18; 0x1105c(1): +0x020 = 3 (0x1105c stores arg ? 3 : 0);
 *   0x11084(1) enables ARISC irq 0x28
 *   (0x5fd0) -- not done here: this firmware polls and has no handler for it.
 * The meaning of b00/b08/0x020..0x038 is unread; they are written because
 * the blob writes them on every reset, and nothing else is.
 */
static void
reset_module(void)
{
	mmio_write_32(R_HDMI_AUX_GATE_REG, 0);
	udelay(5000);
	mmio_write_32(R_HDMI_AUX_GATE_REG, 0x00010000);
	mmio_write_32(R_HDMI_AUX_CLK_REG, BIT(31));
	mmio_write_32(R_PIO_PL_CFG1,
	              (mmio_read_32(R_PIO_PL_CFG1) & 0xff) | 0x22222200);
	mmio_write_32(AUX_UNK_B08, 0xc0c0c0c0);
	mmio_write_32(AUX_GATE_REG, mmio_read_32(AUX_GATE_REG) & 0x00ffffff);
	sh_wr8(SH_EDID_READY, 0);
	sh_wr8(SH_EDID_GATE, 0);
	sh_wr8(SH_EDID_MASK, 0);
	aux_gate(0x7, false);
	mmio_write_32(AUX_UNK_B00, 0x00a0a0a0);
	sh_wr8(SH_EDID_READY, 1);
	publish();
	mmio_write_32(AUX_UNK_030, 0);
	mmio_write_32(AUX_UNK_034, 0x18);
	mmio_write_32(AUX_UNK_038, 0x18);
	mmio_write_32(AUX_UNK_020, 3);
	sh_status(ST_EDID_MODULE, 0);
}

static void
portmap_init(void) /* the ROM identity table, blob 0x15edc */
{
	for (uint8_t p = 0; p < PORTS; ++p) {
		uintptr_t rec = SH_PORTMAP + 8 * p;

		sh_wr8(rec + 0, BIT(p));
		sh_wr8(rec + REC_PHYS, 0x10 * (p + 1));
		sh_wr8(rec + REC_PIN, p);
		sh_wr8(rec + REC_PIN_BIT, BIT(p));
		sh_wr8(rec + REC_REVERSE, p);
	}
}

/* ---- the sub-commands (classify 0x114e8, group handler 0x11550) ---------- */
static void
group_11(uint8_t hi)
{
	uint8_t a1 = sh_rd8(SH_RPM_BUF + 2), a2 = sh_rd8(SH_RPM_BUF + 3);

	for (uint8_t i = 0; i < SCRATCH_LEN; ++i)
		sh_wr8(SH_SCRATCH + i, 0);

	switch (hi) {
	case 0: {	/* HostHDMIMAP: the pins, then a republish */
		uint8_t map[PORTS] = { a1, a2, sh_rd8(SH_RPM_BUF + 4) };

		for (uint8_t p = 0; p < PORTS; ++p) {
			uintptr_t rec = SH_PORTMAP + 8 * p;

			sh_wr8(rec + REC_PIN, map[p] & 7);
			sh_wr8(rec + REC_PIN_BIT, BIT(map[p] & 7));
			sh_wr8(rec + REC_REVERSE, map[p]);
		}
		publish();
		break;
	}
	case 1: {	/* UpdateEDID: fragment a1, byte 0 in a2, 63 more */
		uint8_t frag = a1 & 7, *dst = &edid[64 * frag];

		for (uint8_t i = 0; i < SCRATCH_LEN; ++i)
			sh_wr8(SH_SCRATCH + i, sh_rd8(SH_RPM_BUF + 2 + i));
		dst[0] = a2;
		for (uint8_t i = 1; i < 64; ++i)
			dst[i] = sh_rd8(SH_RPM_BUF + 3 + i);
		if (frag == 0 || frag == 7)	/* an upload starts, ends */
			edid_valid = frag == 7;
		sh_wr8(SH_EDID_READY, frag == 7);
		if (frag == 7)
			publish();
		break;
	}
	case 2:		/* PullHotPlug: port a1, value 1 up / 2 down / 3 reset */
		sh_wr8(SH_SCRATCH + 0, a1);
		sh_wr8(SH_SCRATCH + 1, a2);
		if (a1 >= PORTS)
			warn("H713: wrong HDMI port %u", a1);
		else if (a2 == 1)
			sh_wr8(SH_HPD_COUNTER + a1, sh_rd8(SH_HPD_COUNTER + a1) + 1);
		else if (a2 == 2 || a2 == 3)
			hpd_pin(a1, false), sh_wr8(SH_HPD_COUNTER + a1, a2 == 3 ? HPD_RESET_TICKS : 0);
		else
			warn("H713: unknown hotplug type %u", a2);
		break;
	case 3:		/* SetEDIDVersion: the per-port block mask */
		sh_wr8(SH_SCRATCH + 0, a1);
		sh_wr8(SH_EDID_MASK, a1 & 0xf);
		break;
	case 0x20:	/* ResetEDIDModule */
		reset_module();
		break;
	default:	/* SET5VFlag (4), SetEDIDAudioMode (5): accepted, no effect */
		break;
	}
}

static void
group_15(uint8_t hi)
{
	uint8_t port = sh_rd8(SH_RPM_BUF + 2), frame[72];

	if (hi == 2) {		/* CheckEDIDUpdateStatus: ff f8 01 01 <ready> 00 cks fe */
		frame[1] = 0xf8, frame[2] = 0x01, frame[3] = 0x01;
		frame[4] = sh_rd8(SH_EDID_READY), frame[5] = 0;
		reply_frame(frame, 8);
	} else if (hi == 3 && port < PORTS && aux_ok(3)) { /* RequestEDID */
		/*
		 * Blob 0x1269c..0x12728: the gate closes before the DDC RAM is
		 * read and opens again after the four frames (0x11cac(7, 0) and
		 * (7, 1), the latter only with the EDID ready). With the gate
		 * open the RAM belongs to the DDC side and reads as zero, from
		 * the AR100 and from the ARM alike (HY310, 07.10.2026). It
		 * opens again only on an EDID publish() put out (the gate
		 * byte), not on the empty store after a ResetEDIDModule.
		 */
		aux_gate(0x7, false);
		for (uint8_t idx = 0; idx < 4; ++idx) {
			frame[1] = 0xf8, frame[2] = 0x02, frame[3] = 0x41, frame[4] = idx;
			for (uint8_t i = 0; i < 64; ++i)
				frame[5 + i] = mmio_read_32(AUX_DDC_RAM(port) + 64 * idx + (i & ~3)) >> (8 * (i & 3));
			frame[69] = 0;
			reply_frame(frame, 72);
		}
		if (sh_rd8(SH_EDID_READY) && sh_rd8(SH_EDID_GATE))
			aux_gate(0x7, true);
	} else {
		/* no answer: the ARM gives up after its 2 s (PRUEFUNG5 F16) */
		h713_cnt(H713_CNT_ARM_UNKNOWN);
	}
}

static void
classify(void)
{
	uint8_t lo = sh_rd8(SH_RPM_BUF + 0), hi = sh_rd8(SH_RPM_BUF + 1);

	if (pump_type != 0) {
		mmio_write_32(DIAG_ARM_TYPE, mmio_read_32(DIAG_ARM_TYPE) + 1);
		h713_cnt(H713_CNT_ARM_UNKNOWN);
		warn("H713: unknown ARM message type %u", pump_type); return;
	}
	if (lo == 0x11)
		group_11(hi);
	else if (lo == 0x15)
		group_15(hi);
	else if (lo < 0x10 || lo > 0x15)
		h713_cnt(H713_CNT_ARM_UNKNOWN),
		warn("H713: unknown sub-command %02x%02x", hi, lo);
}

/* BOP pump (blob 0x7fd8): header word, pad word, then <len> payload bytes. */
static void
pump_word(uint32_t w)
{
	switch (pump_state) {
	case 0:
		if ((w & 0xff) != BOP_MARKER)
			return;
		pump_type = w >> 16;
		pump_len  = w >> 24;
		pump_idx  = 0;
		/* The kernel writes the payload into the buffer itself and
		 * sends length 0; a longer frame must still end inside it,
		 * not in the words behind it (the diagnostic words at +0x200). */
		pump_state = pump_len > RPM_BUF_LEN ? 0 : 1;
		if (!pump_state)
			h713_cnt(H713_CNT_PUMP_LONG);
		break;
	case 1:
		if (pump_len == 0) {
			pump_state = 0;
			classify();
		} else {
			pump_state = 2;
		}
		break;
	default:
		for (uint8_t b = 0; b < 4 && pump_idx < pump_len; ++b, ++pump_idx)
			sh_wr8(SH_RPM_BUF + pump_idx, w >> (8 * b));
		if (pump_idx >= pump_len) {
			pump_state = 0;
			classify();
		}
		break;
	}
}

/* ---- the display MIPS (blob 0x1180c "DispMips", 0x1242c) ------------------ */
static void
mips_frame(uint8_t q, const uint8_t *p, uint8_t len)
{
	mmio_write_32(DIAG_MIPS_N, mmio_read_32(DIAG_MIPS_N) + 1);
	mmio_write_32(DIAG_MIPS_LAST, p[0] | p[1] << 8 | p[2] << 16 |
	              (uint32_t)p[3] << 24);
	mmio_write_32(H713_K_MIPS_PORT, q);
	if (len >= 3 && p[0] == 0x16 && p[1] < PORTS) {	/* 0x12330 */
		uint8_t port = p[1], action = p[2];

		h713_cnt(H713_CNT_MIPS_16);
		mmio_write_32(H713_K_MIPS_STRAY_INFO,	/* its companions anew */
		              mmio_read_32(H713_K_MIPS_STRAY_INFO) & ~0xff00U);
		/*
		 * No aux clock before the first ResetEDIDModule: dropped as
		 * the blob drops it, counted above with every 0x16 and not as
		 * an error - the MIPS may well send it that early at boot.
		 */
		if (!aux_clocked())
			return;
		if (action == 1)
			sh_wr8(SH_HPD_COUNTER + port, sh_rd8(SH_HPD_COUNTER + port) + 1);
		else if (action == 2 || action == 3)
			hpd_pin(port, false),
			sh_wr8(SH_HPD_COUNTER + port, action == 3 ? HPD_RESET_TICKS : 0);
	} else if (len >= 1 && p[0] == 0x17) {			/* 0x120f0 */
		h713_cnt(H713_CNT_MIPS_17);
		reset_module();
	} else {
		h713_cnt(H713_CNT_MIPS_DROP);
	}
}

/* A word without the marker outside a frame on MIPS port @q (MIPS_STRAYS). */
static void
mips_stray(uint8_t q, uint32_t w)
{
	uint32_t info = mmio_read_32(H713_K_MIPS_STRAY_INFO);
	uint32_t n = (info >> 8 & 0xff) + 1;

	if (n > 0xff)
		n = 0xff;
	mmio_write_32(H713_K_MIPS_STRAY, w);
	mmio_write_32(H713_K_MIPS_STRAY_INFO, (info & 0xffff0000U) | n << 8 | q);
	if (n > MIPS_STRAYS) {
		h713_cnt(STRAY_COUNTED);
		h713_cnt(H713_CNT_MIPS_DROP);
	} else {
		h713_cnt(STRAY_TAKEN);
	}
}

/*
 * The pump 0x7fd8 once more, for the MIPS: payload in the words, LSB first.
 * At most MIPS_PASS_WORDS per port and pass: a FIFO that does not pop on
 * read would otherwise hold the main loop (PRUEFUNG5 F11).
 */
static void
mips_poll(void)
{
	for (uint8_t q = 0; q < ARRAY_SIZE(mips); ++q) {
		for (uint8_t n = 0; n < MIPS_PASS_WORDS &&
		     mmio_read_32(MIPS_FIFO_STAT(q)); ++n) {
			uint32_t w = mmio_read_32(MIPS_MSG_DATA(q));

			switch (mips[q].state) {
			case 0:
				if ((w & 0xff) != BOP_MARKER) {
					mips_stray(q, w);
					break;
				}
				mmio_write_32(H713_K_MIPS_HEAD, w);
				mips[q].len = w >> 24;
				mips[q].idx = 0;
				for (uint8_t i = 0; i < sizeof(mips[q].buf); ++i)
					mips[q].buf[i] = 0;
				mips[q].state = mips[q].len > BOP_MAX_LEN ? 0 : 1;
				if (!mips[q].state)
					h713_cnt(H713_CNT_MIPS_DROP);
				break;
			case 1:
				mips[q].state = mips[q].len ? 2 : 0;
				if (!mips[q].len)
					mips_frame(q, mips[q].buf, 0);
				break;
			default:
				for (uint8_t b = 0; b < 4 && mips[q].idx < mips[q].len;
				     ++b, ++mips[q].idx) {
					if (mips[q].idx < sizeof(mips[q].buf))
						mips[q].buf[mips[q].idx] = w >> (8 * b);
				}
				if (mips[q].idx >= mips[q].len) {
					mips[q].state = 0;
					mips_frame(q, mips[q].buf, mips[q].len);
				}
				break;
			}
		}
	}
}

/* ---- the standby dispatcher on port 3 (blob 0xbca0) ----------------------- */
static void
sb_reply(uint8_t type, uint8_t result, uint32_t *data, uint8_t count)
{
	uint8_t attr = sb_hdr >> 8;

	if (!(attr & 3))
		return;
	if (q_free(Q_NOTIFY) < 2 + count) {
		h713_err(H713_ERR_TXQ_FULL, Q_NOTIFY, type);
		h713_cnt(H713_CNT_TXQ_FULL);
		warn("H713: no room for the answer to %x", type); return;
	}
	q_put(Q_NOTIFY, SB_STATE | (uint32_t)attr << 8 | (uint32_t)type << 16 |
	      (uint32_t)result << 24);
	q_put(Q_NOTIFY, count);
	for (uint8_t i = 0; i < count; ++i)
		q_put(Q_NOTIFY, data[i]);
}

static void
sb_frame(void)
{
	uint8_t type = sb_hdr >> 16;

	/*
	 * A request that wants an answer comes after the ARM emptied channel 3
	 * (arisc_sb_request): whatever is still queued there - a notify after
	 * a firmware restart nobody listened to - would be read as the answer.
	 */
	if (type != TYPE_NOTIFY && (sb_hdr >> 8 & 3))
		q_head[Q_NOTIFY] = q_tail[Q_NOTIFY];

	switch (type) {
	case TYPE_NOTIFY:	/* the ARM's acknowledgement of our notify */
		sh_status(ST_HANDSHAKE, 0);
		info("H713: startup notify acknowledged");
		break;
	case TYPE_CPU_OP:	/* {mpidr, entrypoint, cpu, cluster, system} */
		/*
		 * TF-A asks for no answer (attr 0) and parks the core in WFI:
		 * a request crust cannot carry out - too short, no entry, not
		 * core 0 (the cluster would stay up and never suspend), or one
		 * the CSS refuses - ends in the clean reset, not in a hang.
		 */
		if (sb_count < 2 || sb_data[1] == 0 || (sb_data[0] & 0xff))
			h713_fail(H713_CAUSE_CPU_OP, sb_count, sb_data[0]);
		info("H713: cpu op req, warm entry %p", (void *)sb_data[1]);
		h713_sleep_event(H713_EV_CPU_OP);
		css_set_warm_entry(sb_data[1], 0);
		udelay(CPU_OP_DELAY_US);
		if (css_set_power_state(0, 0, SCPI_CSS_OFF, SCPI_CSS_OFF,
		                        SCPI_CSS_OFF) != SCPI_OK)
			h713_fail(H713_CAUSE_CPU_OP, sb_count, sb_data[0]);
		break;
	case TYPE_RESTORE: {	/* answered with the wake word */
		/*
		 * Only once per resume: after a suspend Linux abandoned before
		 * TF-A's cpu op req (an abort after the noirq phase), the word
		 * of the previous standby must not come back as this one's;
		 * the answer is 0 then, and a timer armed for it is cleared.
		 * The word waits in K (h713_resume_done()), where a restart
		 * in RESUMING does not clear it.
		 */
		uint32_t wake = mmio_read_32(H713_K_WAKE_KEPT);

		mmio_write_32(H713_K_WAKE_KEPT, 0);
		mmio_write_32(H713_DIAG_WAKE_TIMER, 0);
		h713_sleep_event(H713_EV_RESTORE);
		sb_reply(type, 0, &wake, 1);
		break;
	}
	case TYPE_WAKE_SET:	/* key and IR are armed without it; type 3 = timer */
		if (sb_count >= 1 && sb_data[0] >> 30 == WAKE_SRC_TIMER)
			mmio_write_32(H713_DIAG_WAKE_TIMER,
			              sb_data[0] & 0x3fffffff);
		h713_sleep_request();
		sb_reply(type, 0, NULL, 0); break;
	case TYPE_WAKE_CLR:
		mmio_write_32(H713_DIAG_WAKE_TIMER, 0);
		h713_sleep_request();
		sb_reply(type, 0, NULL, 0); break;
	default:
		sb_reply(type, RESULT_UNKNOWN, NULL, 0); break;
	}
}

/*
 * Resynchronisation (PRUEFUNG5 F7, K15): a frame the ARM left half-sent -
 * a push that failed after the header, a firmware restart with words in
 * the FIFO - shifted every later frame, and standby requests failed until
 * the stream realigned by chance. A header now needs state 0 or 2 and a
 * type this dispatcher knows, a count of at most 8 words, and the frame
 * must be complete within SB_FRAME_US; anything else is dropped (error
 * bit 3 with the position and the word, CNT0[7:0]) and the next word is
 * looked at as a header.
 */
static bool
sb_head(uint32_t w)
{
	uint8_t state = w & 0xff, type = w >> 16;

	return (state == 0 || state == SB_STATE) &&
	       (type == TYPE_RESTORE || type == TYPE_CPU_OP ||
	        type == TYPE_WAKE_CLR || type == TYPE_WAKE_SET ||
	        type == TYPE_NOTIFY);
}

static void
sb_drop(uint32_t w)
{
	h713_err(H713_ERR_SB_DROP, sb_n, w);
	h713_cnt(H713_CNT_SB_DROP);
	sb_n = 0;
}

static void
sb_word(uint32_t w)
{
	if (sb_n && passed(sb_deadline))
		sb_drop(sb_hdr);
	if (sb_n == 0) {
		if (!sb_head(w)) {
			sb_drop(w);
			return;
		}
		sb_hdr = w;
		sb_deadline = timeout_set(SB_FRAME_US);
	} else if (sb_n == 1) {
		sb_count = w & 0xff;
		if (sb_count > ARRAY_SIZE(sb_data)) {
			sb_drop(w);
			return;
		}
	} else {
		sb_data[sb_n - 2] = w;
	}
	++sb_n;
	if (sb_n >= 2 && sb_n == 2 + sb_count) {
		sb_n = 0;
		sb_frame();
	}
}

/* Startup notify (blob 0xc4b0): 13 words of version text in the odd byte
 * order that the driver's arisc_notify_version() undoes; byte 48 is never
 * sent, so the text has 48 characters at most. The contract version in it
 * is H713_VERSION's; the commit and a dirty tree come from version.sh. */
static void
send_notify(void)
{
	static const char version[] = "crust-h713 v5 " VERSION_STRING "-"
	                              VERSION_HASH;
	uint8_t s[52];

	static_assert(sizeof(version) <= 48 + 1, "notify text too long");
	static_assert(H713_VERSION == 5, "notify text names version 5");

	for (uint8_t i = 0, end = 0; i < sizeof(s); ++i) {
		end = end || !version[i];	/* never read past the NUL */
		s[i] = end ? 0 : version[i];
	}
	q_put(Q_NOTIFY, (uint32_t)TYPE_NOTIFY << 16 | 2 << 8);
	q_put(Q_NOTIFY, 13);
	for (uint8_t k = 0; k < 13; ++k) {
		uint32_t w = (k ? s[4 * k - 4] : 0) | (uint32_t)s[4 * k + 3] << 8 |
		             (uint32_t)s[4 * k + 2] << 16 | (uint32_t)s[4 * k + 1] << 24;

		q_put(Q_NOTIFY, w);
	}
}

void
h713_init(void)
{
	/*
	 * A restart through an exception while awake, after Linux took the
	 * notify (both loaders zero the start counter, the magic and the
	 * status first): nobody listens for a second notify, and its words
	 * would come before the answer to the next standby request. The
	 * handshake stands; no notify.
	 */
	bool first = mmio_read_32(DIAG_INITS) == 0;
	bool acked = !first &&
	             mmio_read_32(SH_MAGIC) == H713_MAGIC &&
	             (mmio_read_32(SH_STATUS) & ST_HANDSHAKE);

	/*
	 * Contract v5 (platform/memory.h): R and S at every start; K and X
	 * only at the first one after a load (a v4 kernel zeroes the diag
	 * words, not K), so the record survives an exception restart. D, A
	 * and P are the loaders'. An acknowledged restart also keeps R's
	 * HDMI bytes (+0x010..+0x0ff: HPD counters, ready, gate, mask, port
	 * map): the aux block, the pin and the DDC RAM keep their state
	 * across it, so a pulse under way ends and the pin can rise again
	 * (PRUEFUNG5 F2). edid[] is gone with .bss; publish() waits for the
	 * next upload.
	 */
	for (uintptr_t a = H713_SHARED_BASE; a < H713_SEEN_LIMIT; a += 4) {
		bool kept = (a >= H713_DIAG_BASE && a < H713_DIAG_LIMIT) ||
		            (acked && a >= H713_R_KEPT_BASE &&
		             a < H713_R_KEPT_LIMIT);

		if (!kept)
			mmio_write_32(a, 0);
	}
	for (uintptr_t a = H713_K_BASE; first && a < H713_SHARED_LIMIT; a += 4) {
		if (a < H713_K_LIMIT || a >= H713_X_BASE)
			mmio_write_32(a, 0);
	}
	mmio_write_32(SH_WAKE, mmio_read_32(H713_K_WAKE_KEPT));	/* 0x11 to come */
	h713_sleep_init(first);
	mmio_write_32(DIAG_CPUS_CLK, r_ccu_cpus_clk_at_boot());
	mmio_write_32(DIAG_INITS, mmio_read_32(DIAG_INITS) + 1);
	mmio_write_32(DIAG_INTC_80, mmio_read_32(DEV_R_INTC + 0x80));
	mmio_write_32(DIAG_INTC_88, mmio_read_32(DEV_R_INTC + 0x88));
	mmio_write_32(SH_MAGIC, H713_MAGIC);
	mmio_write_32(SH_VERSION, H713_VERSION);
	/*
	 * Which crust this is: the kernel keeps a running image only when it
	 * is the one in the file it was asked to load - same size, and the
	 * words up to SH_RO_SIZE (code and constants, nothing crust writes)
	 * equal to the file's - and shows the variant: light, self-refresh
	 * and deep share version and notify.
	 */
	mmio_write_32(SH_RO_SIZE,
	              (uint32_t)((uintptr_t)__data_start - FIRMWARE_BASE));
	mmio_write_32(H713_DIAG_BUILD,
	              (uint32_t)((uintptr_t)__bss_start - FIRMWARE_BASE) |
	              (uint32_t)CONFIG(H713_DRAM_SELF_REFRESH) << 24 |
	              (uint32_t)CONFIG(H713_DEEP_SLEEP) << 25 |
	              (uint32_t)CONFIG(H713_IR_WAKE_NECX) << 26 |
	              (uint32_t)CONFIG(H713_DEEP_VDD_SYS) << 27 |
	              (uint32_t)CONFIG(H713_DEEP_PLL_LDO) << 28 |
	              (uint32_t)CONFIG(H713_SLEEP_NET) << 29 |
	              (uint32_t)CONFIG(H713_TEST) << 30 |
	              (uint32_t)CONFIG(H713_DEEP_AVCC_GATING) << 31);
	mmio_write_32(H713_K_COMPAT, H713_COMPAT);
	mmio_write_32(H713_K_BUILD2, (BUILD2_DRAM_MHZ & 0xfff) |
	              (CONFIG_H713_BOARD_ID & 0xf) << 12 |
	              (BUILD2_NET_CODE & 0xff) << 16);
	if (!acked)
		portmap_init();
	if (aux_clocked())	/* hardware state, also after a restart */
		sh_status(ST_EDID_MODULE, 0);
	/*
	 * Load-bearing: system.c puts its own msgbox reference at every
	 * suspend, and a last put would assert the msgbox reset (ccu.c); this
	 * one keeps the box out of reset through the standby.
	 */
	device_get(&msgbox.dev);
	if (acked)
		sh_status(ST_HANDSHAKE, 0);
	else
		send_notify();
	sh_status(ST_RUNNING, 0);
	hpd_deadline = timeout_set(HPD_TICK_US);
}

void
h713_poll(void)
{
	uint32_t w;

	q_drain(Q_REPLY);
	/* Standby frames always: their answers are three words at most, and
	 * TF-A's cpu op req must never wait behind an unread reply. Read
	 * before channel 3 is refilled, so a request that clears the queue
	 * (sb_frame) does so before stale words reach the emptied FIFO. */
	while (!(CONFIG(H713_TEST_PORT3_SILENT) &&
	         (mmio_read_32(H713_K_SB_STATE) & H713_SB_PHASE) ==
	         H713_PH_PRESLEEP) &&
	       msgbox_receive(&msgbox.dev, PORT_STANDBY, &w) == SUCCESS)
		sb_word(w);
	q_drain(Q_NOTIFY);
	/* The pump one request at a time: the next after the last reply left. */
	while (q_empty(Q_REPLY) &&
	       msgbox_receive(&msgbox.dev, PORT_HDMI, &w) == SUCCESS)
		pump_word(w);
	mips_poll();

	/* The tick (blob 0x10db4) and the wake-up (0x10f3c) of the HPD pin. */
	bool tick = passed(hpd_deadline);

	if (tick)
		hpd_deadline = timeout_set(HPD_TICK_US);
	for (uint8_t p = 0; p < PORTS; ++p) {
		uint8_t c = sh_rd8(SH_HPD_COUNTER + p);

		if (c == 1)
			sh_wr8(SH_HPD_COUNTER + p, 0), hpd_pin(p, true);
		else if (c > 1 && tick)
			sh_wr8(SH_HPD_COUNTER + p, c - 1);
	}
	mmio_write_32(SH_STATUS, mmio_read_32(SH_STATUS) + 0x10000); /* heartbeat */
	mmio_write_32(DIAG_TTCR, cycle_counter_read()); /* clock check from the ARM */
}

void
h713_suspend_prepare(void)
{
	pl_cfg_saved = mmio_read_32(R_PIO_PL_CFG1);	/* PL9's function */
	pl_cfg0_saved  = mmio_read_32(R_PIO_PL_CFG0);
	pl_drv0_saved  = mmio_read_32(R_PIO_PL_DRV0);
	pl_drv1_saved  = mmio_read_32(R_PIO_PL_DRV1);	/* PL9, the IR pin */
	pl_pull0_saved = mmio_read_32(R_PIO_PL_PULL0);
	pl_data_saved  = mmio_read_32(R_PIO_PL_DATA);
	if (!(cir = cir_get()) && CONFIG(H713_IR_WAKE_NECX))
		h713_err(H713_ERR_CIR, 0, 0);
	if (gpio_get(&key_pin))
		h713_err(H713_ERR_KEY_PIN, CONFIG_H713_POWER_KEY_PIN, 0),
		warn("H713: power key pin unavailable");
	if (LED_RED >= 0 && !gpio_get(&led_red))
		gpio_set_value(&led_red, true);
	if (LED_BLUE >= 0 && !gpio_get(&led_blue))
		gpio_set_value(&led_blue, false);
	key_state = KEY_ENTRY;
	key_count = 0;
	key_deadline = timeout_set(KEY_POLL_US);
	ms_deadline = timeout_set(1000);
	wake_armed = true;
	mmio_write_32(H713_DIAG_SLEPT_MS, 0);
	mmio_write_32(SH_WAKE, 0);
	sh_status(ST_ASLEEP, 0);
}

uint32_t
h713_wake_poll(void)
{
	bool level;

	uint32_t timer = mmio_read_32(H713_DIAG_WAKE_TIMER);

	h713_net_poll();
	/*
	 * After a firmware restart while asleep (system.c goes to SS_OFF) the
	 * deadlines are gone with .bss; without fresh ones a deadline in the
	 * other half of the tick timer would not expire for up to 89 s.
	 */
	if (!wake_armed) {
		key_deadline = timeout_set(KEY_POLL_US);
		ms_deadline  = timeout_set(1000);
		wake_armed   = true;
	}
	/* Milliseconds asleep, stepped from the deadline so nothing drifts;
	 * a long pass is caught up. */
	while (passed(ms_deadline)) {
		ms_deadline += CPUCLK_kHz;
		mmio_write_32(H713_DIAG_SLEPT_MS,
		              mmio_read_32(H713_DIAG_SLEPT_MS) + 1);
	}
	if (timer && key_state != KEY_OPEN &&
	    mmio_read_32(H713_DIAG_SLEPT_MS) >= timer) {
		mmio_write_32(SH_WAKE, WAKE_TIMER);
		return WAKE_TIMER;
	}

	if (passed(key_deadline)) {
		bool low = gpio_get_value(&key_pin, &level) == SUCCESS && !level;

		key_deadline = timeout_set(KEY_POLL_US);
		switch (key_state) {
		case KEY_ENTRY:
			/* lows count only after a high (PRUEFUNG5 CS-10) */
			if (!low)
				key_state = KEY_ARMED, key_count = 0;
			else if (!key_count++)
				h713_err(H713_ERR_KEY_HELD, 0, 0);
			break;
		case KEY_ARMED:
			key_count = low ? key_count + 1 : 0;
			if (key_count >= KEY_DEBOUNCE) {
				mmio_write_32(SH_WAKE, WAKE_KEY);
				key_state = KEY_OPEN, key_count = 0;
				key_open_ms = mmio_read_32(H713_DIAG_SLEPT_MS);
			}
			break;
		default:
			/*
			 * The wake is the key's. Resume once it is released:
			 * PL4 stays an input until then, so the release edge
			 * never reaches the EINT detector, gpio-keys reports no
			 * press and logind does not power the box off
			 * (PRUEFUNG5 CS-07). A key held longer resumes after
			 * KEY_RELEASE_MS; Linux has to ignore that one press.
			 */
			key_count = low ? 0 : key_count + 1;
			if (key_count >= KEY_DEBOUNCE ||
			    mmio_read_32(H713_DIAG_SLEPT_MS) - key_open_ms >=
			    KEY_RELEASE_MS)
				return WAKE_KEY;
			break;
		}
	}
	if (key_state == KEY_OPEN)
		return 0;
#if CONFIG(H713_IR_WAKE_NECX)
	if (cir) {
		uint32_t code = sunxi_cir_poll(cir);

		for (uint8_t i = 0; code && i < ARRAY_SIZE(ir_wake_codes); ++i) {
			if (code == ir_wake_codes[i]) {
				mmio_write_32(SH_WAKE, WAKE_IR);
				return WAKE_IR;
			}
		}
	}
#endif
	return 0;
}

/*
 * The EINT edge detector samples the pin with the bank's debounce clock
 * (PIO_INT_DEB: bit 0 LOSC 32 kHz or HOSC 24 MHz, bits 6..4 a 2^n divider).
 * On the HY310 it reads 0 (measured 07.10.2026; Linux leaves it alone
 * without "input-debounce"): one sample every 30.5 us. An edge from the
 * mux switch reaches the status register only with the next samples, so
 * a clear right after the switch comes too early. Eight samples, at least
 * 1 ms.
 */
static uint32_t
eint_settle_us(void)
{
	uint32_t deb = mmio_read_32(R_PIO_EINT_DEB);
	uint32_t us  = 8 * (((deb & BIT(0)) ? 1U : 31U) << ((deb >> 4) & 0x7));

	return us < 1000 ? 1000 : us;
}

void
h713_resume_done(void)
{
	const uint32_t key_bit = BIT(CONFIG_H713_POWER_KEY_PIN);

	if (LED_BLUE >= 0) {
		gpio_set_value(&led_blue, true);
		gpio_put(&led_blue);
	}
	if (LED_RED >= 0) {
		gpio_set_value(&led_red, false);
		gpio_put(&led_red);
	}
	gpio_put(&key_pin);
	device_put(cir), cir = NULL;
	mmio_write_32(R_PIO_PL_CFG1, pl_cfg_saved);	/* the kernel's IR pin */
	/* the kernel's power key interrupt and its LED state, as before */
	if (LED_MASK)
		mmio_clrset_32(R_PIO_PL_DATA, LED_MASK, pl_data_saved & LED_MASK);
	mmio_write_32(R_PIO_PL_PULL0, pl_pull0_saved);
	mmio_write_32(R_PIO_PL_DRV0, pl_drv0_saved);
	mmio_write_32(R_PIO_PL_DRV1, pl_drv1_saved);
	/*
	 * gpio-keys turns any interrupt of a wakeup key that comes in while it
	 * is suspended into a key press ("simulate wakeup key press"), and
	 * logind powers the box off: on the HY310 (07.10.2026) the box shut
	 * down 0.2 s after a timer wake (run B1), and runs B2 and C1 recorded
	 * the press. PL4's mux switches latch edges: the one away from EINT at
	 * the start of the standby, and the one back here, which reaches the
	 * status register only with the next debounce samples - 0031 cleared
	 * right after the switch, too early. So: clear what the standby left,
	 * switch back, wait, clear, wait again and clear once more, all before
	 * the cores run (Linux takes interrupts only some 30 ms later; the
	 * call before css_resume() is for a strict order, not a cause).
	 */
	mmio_write_32(EINT_SEEN_BEFORE, mmio_read_32(R_PIO_EINT_STATUS));
	mmio_write_32(R_PIO_EINT_STATUS, key_bit);
	mmio_write_32(R_PIO_PL_CFG0, pl_cfg0_saved);
	mmio_write_32(EINT_SEEN_SWITCH, mmio_read_32(R_PIO_EINT_STATUS));
	udelay(eint_settle_us());
	mmio_write_32(EINT_SEEN_SETTLED, mmio_read_32(R_PIO_EINT_STATUS));
	mmio_write_32(R_PIO_EINT_STATUS, key_bit);
	udelay(eint_settle_us());
	mmio_write_32(EINT_SEEN_AFTER, mmio_read_32(R_PIO_EINT_STATUS));
	mmio_write_32(R_PIO_EINT_STATUS, key_bit);
	sh_status(0, ST_ASLEEP);
	mmio_write_32(H713_K_WAKE_KEPT, mmio_read_32(SH_WAKE));	/* for 0x11 */
	/* The HPD tick did not run asleep: a deadline from before the standby
	 * would not expire for up to 89 s after a long one. */
	hpd_deadline = timeout_set(HPD_TICK_US);
	/*
	 * The transfers start fresh: a frame the MIPS left half-sent before
	 * it was parked would take the next one's header as payload, and a
	 * reply's stall deadline from before a long sleep would hold port 0
	 * (PRUEFUNG5 F11, F16 d).
	 */
	for (uint8_t q = 0; q < ARRAY_SIZE(mips); ++q)
		mips[q].state = 0;
	pump_state = 0;
	for (uint8_t q = 0; q < Q_COUNT; ++q)
		q_stall[q] = timeout_set(TXQ_STALL_US);
	/* one standby per arming, as the vendor's 0x26 */
	mmio_write_32(H713_DIAG_WAKE_TIMER, 0);
	info("H713: resumed, wake word %x", mmio_read_32(SH_WAKE));
}
