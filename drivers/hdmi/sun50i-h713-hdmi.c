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

#define H713_MAGIC      0x33313748                 /* 'H','7','1','3' */
#define H713_VERSION    0x00000001
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
#define RX_PORT(p)      (2 * (p))      /* ARM -> ARISC, user1 port p */
#define TX_CHAN(c)      (2 * (c) + 1)  /* ARISC -> ARM, user0 channel c */
#define PORT_HDMI       RX_PORT(0)
#define PORT_STANDBY    RX_PORT(3)
#define CHAN_REPLY      TX_CHAN(1)
#define CHAN_NOTIFY     TX_CHAN(3)

/* ---- the HDMI aux block (RTC domain) ------------------------------------ */
#define AUX_HPD_REG     (DEV_HDMI_AUX + 0x014) /* active-low, one bit per pin */
#define AUX_UNK_B00     (DEV_HDMI_AUX + 0xb00)
#define AUX_GATE_REG    (DEV_HDMI_AUX + 0xb04) /* EDID/DDC output, bit per port */
#define AUX_UNK_B08     (DEV_HDMI_AUX + 0xb08)
#define AUX_DDC_RAM(p)  (DEV_HDMI_AUX + 0xc00 + 0x100 * (p))
#define R_PIO_PL_CFG1   (DEV_R_PIO + 0x004)    /* PL8..PL15 function */

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
#define KEY_POLL_US     20000
#define KEY_DEBOUNCE    3

static uint8_t  edid[2 * EDID_BLOCK];
static uint32_t txq[128];
static uint8_t  txq_head, txq_tail, txq_chan, seq;
static uint8_t  pump_state, pump_len, pump_idx, pump_type;
static uint8_t  sb_n, sb_count;
static uint32_t sb_hdr, sb_data[8];
static uint32_t hpd_deadline, key_deadline;
static uint8_t  key_low;
static uint32_t pl_cfg_saved;
static const struct device *cir;

#define PL(pin, m, p) { .dev = &r_pio.dev, .id = SUNXI_GPIO_PIN(0, pin), \
	.drive = DRIVE_10mA, .mode = m, .pull = p }
static const struct gpio_handle key_pin  = PL(CONFIG_H713_POWER_KEY_PIN, MODE_INPUT, PULL_UP);
static const struct gpio_handle led_red  = PL(0, MODE_OUTPUT, PULL_NONE); /* dt led0 */
static const struct gpio_handle led_blue = PL(1, MODE_OUTPUT, PULL_NONE); /* dt led1 */

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

#define sh_rd8(a)    mmio_read_8(a)
#define sh_wr8(a, v) mmio_write_8(a, v)
#define sh_status(set, clr) \
	mmio_write_32(SH_STATUS, (mmio_read_32(SH_STATUS) & ~(clr)) | (set))

/* ---- transmit queue: never block on an 8-deep FIFO ----------------------- */
static void
txq_put(uint32_t w)
{
	uint8_t next = (txq_tail + 1) % ARRAY_SIZE(txq);

	if (next == txq_head) {
		warn("H713: tx queue full"); return;
	}
	txq[txq_tail] = w;
	txq_tail = next;
}

static void
txq_drain(void)
{
	while (txq_head != txq_tail) {
		if (msgbox_send(&msgbox.dev, txq_chan, txq[txq_head]))
			return;
		txq_head = (txq_head + 1) % ARRAY_SIZE(txq);
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

	txq_chan = CHAN_REPLY;
	txq_put(BOP_MARKER | seq << 8 | REPLY_TYPE << 16 | (uint32_t)len << 24);
	txq_put(0);
	seq = seq % 255 + 1;
	for (uint8_t i = 0; i < len; i += 4) {
		uint32_t w = 0;

		for (uint8_t b = 0; b < 4 && i + b < len; ++b)
			w |= (uint32_t)payload[i + b] << (8 * b);
		txq_put(w);
	}
}

/* ---- the HDMI aux block --------------------------------------------------- */
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
	uint32_t v   = mmio_read_32(AUX_HPD_REG);

	if (!sh_rd8(SH_EDID_GATE))
		high = false; /* blob 0x121e4: EDID-HPD LOW while gated */
	mmio_write_32(AUX_HPD_REG, high ? (v & ~bit) : (v | bit));
}

/* Publish (blob 0x11d88): block by port mask, the port's physical-address
 * byte into the HDMI vendor block, checksum, gate closed while writing. */
static void
publish(void)
{
	uint8_t mask = sh_rd8(SH_EDID_MASK);

	if (!sh_rd8(SH_EDID_READY))
		return;
	aux_gate(0x7, false);
	for (uint8_t port = 0; port < PORTS; ++port) {
		const uint8_t *src = &edid[((mask >> port) & 1) * EDID_BLOCK];
		uint8_t blk[EDID_BLOCK], phys, sum = 0;

		for (int i = 0; i < EDID_BLOCK; ++i)
			blk[i] = src[i];
		phys = sh_rd8(SH_PORTMAP + 8 * port + REC_PHYS);
		/* CEA-861 data block collection: tag 3 with IEEE 00 0c 03. */
		for (int i = 128 + 4; i < 128 + blk[128 + 2] && i < 255;) {
			uint8_t tag = blk[i] >> 5, len = blk[i] & 0x1f;

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

/* ResetEDIDModule (blob 0x120f0): clocks, pins, the block, the state bytes. */
static void
reset_module(void)
{
	mmio_write_32(R_HDMI_AUX_GATE_REG, 0);
	udelay(5000);
	mmio_write_32(R_HDMI_AUX_GATE_REG, 0x10001);
	mmio_write_32(R_HDMI_AUX_CLK_REG, BIT(31));
	mmio_set_32(R_PIO_PL_CFG1, 0x22222200);	/* PL10..PL15 function 2 */
	mmio_write_32(AUX_UNK_B08, 0xc0c0c0c0);
	mmio_write_32(AUX_GATE_REG, (mmio_read_32(AUX_GATE_REG) & 0x00ffffff) | 1);
	sh_wr8(SH_EDID_READY, 0);
	sh_wr8(SH_EDID_GATE, 0);
	sh_wr8(SH_EDID_MASK, 0);
	aux_gate(0x7, false);
	mmio_write_32(AUX_UNK_B00, 0x00a0a0a0);
	sh_wr8(SH_EDID_READY, 1);
	publish();
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
	} else if (hi == 3 && port < PORTS) {	/* RequestEDID: 4 x 64 bytes */
		for (uint8_t idx = 0; idx < 4; ++idx) {
			frame[1] = 0xf8, frame[2] = 0x02, frame[3] = 0x41, frame[4] = idx;
			for (uint8_t i = 0; i < 64; ++i)
				frame[5 + i] = mmio_read_32(AUX_DDC_RAM(port) + 64 * idx + (i & ~3)) >> (8 * (i & 3));
			frame[69] = 0;
			reply_frame(frame, 72);
		}
	}
}

static void
classify(void)
{
	uint8_t lo = sh_rd8(SH_RPM_BUF + 0), hi = sh_rd8(SH_RPM_BUF + 1);

	if (pump_type != 0) {
		warn("H713: unknown ARM message type %u", pump_type); return;
	}
	if (lo == 0x11)
		group_11(hi);
	else if (lo == 0x15)
		group_15(hi);
	else if (lo < 0x10 || lo > 0x15)
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
		pump_state = pump_len > BOP_MAX_LEN ? 0 : 1;
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

/* ---- the standby dispatcher on port 3 (blob 0xbca0) ----------------------- */
static void
sb_reply(uint8_t type, uint8_t result, uint32_t *data, uint8_t count)
{
	uint8_t attr = sb_hdr >> 8;

	if (!(attr & 3))
		return;
	txq_chan = CHAN_NOTIFY;
	txq_put(SB_STATE | (uint32_t)attr << 8 | (uint32_t)type << 16 |
	        (uint32_t)result << 24);
	txq_put(count);
	for (uint8_t i = 0; i < count; ++i)
		txq_put(data[i]);
}

static void
sb_frame(void)
{
	uint8_t type = sb_hdr >> 16;

	switch (type) {
	case TYPE_NOTIFY:	/* the ARM's acknowledgement of our notify */
		sh_status(ST_HANDSHAKE, 0);
		info("H713: startup notify acknowledged");
		break;
	case TYPE_CPU_OP:	/* {mpidr, entrypoint, cpu, cluster, system} */
		if (sb_count < 2 || sb_data[1] == 0) {
			sb_reply(type, RESULT_UNKNOWN, NULL, 0);
			break;
		}
		info("H713: cpu op req, warm entry %p", (void *)sb_data[1]);
		css_set_warm_entry(sb_data[1], 0);
		udelay(2000);
		css_set_power_state(0, sb_data[0] & 0xf, SCPI_CSS_OFF,
		                    SCPI_CSS_OFF, SCPI_CSS_OFF);
		break;
	case TYPE_RESTORE: {	/* answered with the wake word */
		uint32_t wake = mmio_read_32(SH_WAKE);

		sb_reply(type, 0, &wake, 1);
		break;
	}
	case TYPE_WAKE_SET:
	case TYPE_WAKE_CLR:	/* key, IR and CEC are armed without them */
		sb_reply(type, 0, NULL, 0); break;
	default:
		sb_reply(type, RESULT_UNKNOWN, NULL, 0); break;
	}
}

static void
sb_word(uint32_t w)
{
	if (sb_n == 0) {
		sb_hdr = w;
	} else if (sb_n == 1) {
		sb_count = w & 0xff;
	} else if (sb_n < 2 + ARRAY_SIZE(sb_data)) {
		sb_data[sb_n - 2] = w;
	}
	++sb_n;
	if (sb_n >= 2 && sb_n == 2 + sb_count) {
		sb_n = 0;
		sb_frame();
	}
}

/* Startup notify (blob 0xc4b0): 13 words of version text in the odd byte
 * order that the driver's arisc_notify_version() undoes. */
static void
send_notify(void)
{
	static const char version[] = "crust-h713 contract v1 " VERSION_STRING;
	uint8_t s[52];

	for (uint8_t i = 0, end = 0; i < sizeof(s); ++i)
		s[i] = (end || !(end = !version[i])) ? 0 : version[i];
	txq_chan = CHAN_NOTIFY;
	txq_put((uint32_t)TYPE_NOTIFY << 16 | 2 << 8);
	txq_put(13);
	for (uint8_t k = 0; k < 13; ++k) {
		uint32_t w = (k ? s[4 * k - 4] : 0) | (uint32_t)s[4 * k + 3] << 8 |
		             (uint32_t)s[4 * k + 2] << 16 | (uint32_t)s[4 * k + 1] << 24;

		txq_put(w);
	}
}

void
h713_init(void)
{
	for (uintptr_t a = H713_SHARED_BASE; a < H713_SHARED_LIMIT; a += 4)
		mmio_write_32(a, 0);
	mmio_write_32(SH_MAGIC, H713_MAGIC);
	mmio_write_32(SH_VERSION, H713_VERSION);
	portmap_init();
	device_get(&msgbox.dev);
	send_notify();
	sh_status(ST_RUNNING, 0);
	hpd_deadline = timeout_set(HPD_TICK_US);
}

void
h713_poll(void)
{
	uint32_t w;

	txq_drain();
	while (txq_head == txq_tail &&
	       msgbox_receive(&msgbox.dev, PORT_STANDBY, &w) == SUCCESS)
		sb_word(w);
	while (txq_head == txq_tail &&
	       msgbox_receive(&msgbox.dev, PORT_HDMI, &w) == SUCCESS)
		pump_word(w);

	/* The tick (blob 0x10db4) and the wake-up (0x10f3c) of the HPD pin. */
	bool tick = timeout_expired(hpd_deadline);

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
}

void
h713_suspend_prepare(void)
{
	pl_cfg_saved = mmio_read_32(R_PIO_PL_CFG1);	/* PL9's function */
	cir = cir_get();
	if (gpio_get(&key_pin))
		warn("H713: power key pin unavailable");
	if (!gpio_get(&led_red))
		gpio_set_value(&led_red, true);
	if (!gpio_get(&led_blue))
		gpio_set_value(&led_blue, false);
	key_low = 0;
	key_deadline = timeout_set(KEY_POLL_US);
	mmio_write_32(SH_WAKE, 0);
	sh_status(ST_ASLEEP, 0);
}

uint32_t
h713_wake_poll(void)
{
	bool level;

	if (timeout_expired(key_deadline)) {
		key_deadline = timeout_set(KEY_POLL_US);
		if (gpio_get_value(&key_pin, &level) == SUCCESS && !level)
			key_low++;
		else
			key_low = 0;
		if (key_low >= KEY_DEBOUNCE) {
			mmio_write_32(SH_WAKE, WAKE_KEY);
			return WAKE_KEY;
		}
	}
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

void
h713_resume_done(void)
{
	gpio_set_value(&led_blue, true);
	gpio_set_value(&led_red, false);
	gpio_put(&led_blue);
	gpio_put(&led_red);
	gpio_put(&key_pin);
	device_put(cir), cir = NULL;
	mmio_write_32(R_PIO_PL_CFG1, pl_cfg_saved);	/* the kernel's IR pin */
	sh_status(0, ST_ASLEEP);
	info("H713: resumed, wake word %x", mmio_read_32(SH_WAKE));
}
