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
#define RPM_BUF_LEN     67                         /* the kernel's PAYLOAD_MAX */

#define H713_MAGIC      0x33313748                 /* 'H','7','1','3' */
/* 2: diagnostic words; 3: step/exception records, sleep length, wake timer
 * and the build word (platform/memory.h) */
#define H713_VERSION    0x00000003

/* Diagnostic words (platform/memory.h), kept across firmware restarts. */
#define DIAG_LEWIN_EN   (H713_DIAG_BASE + 0x00)    /* written by start.S */
#define DIAG_CPUS_CLK   (H713_DIAG_BASE + 0x0c)
#define DIAG_TTCR       (H713_DIAG_BASE + 0x10)
#define DIAG_INITS      (H713_DIAG_BASE + 0x14)
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

static uint8_t  edid[2 * EDID_BLOCK];
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
static uint32_t hpd_deadline, key_deadline, ms_deadline;
static bool     wake_armed;	/* .bss: false again after a firmware restart */
static bool     woke;		/* a resume since the last 0x11 */
static uint8_t  key_low;
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

extern char __bss_start[];	/* the end of the image (scp.ld.S) */

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
		mmio_write_32(DIAG_ARM_TYPE, mmio_read_32(DIAG_ARM_TYPE) + 1);
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
		/* The kernel writes the payload into the buffer itself and
		 * sends length 0; a longer frame must still end inside it,
		 * not in the words behind it (the diagnostic words at +0x200). */
		pump_state = pump_len > RPM_BUF_LEN ? 0 : 1;
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
mips_frame(const uint8_t *p, uint8_t len)
{
	mmio_write_32(DIAG_MIPS_N, mmio_read_32(DIAG_MIPS_N) + 1);
	mmio_write_32(DIAG_MIPS_LAST, p[0] | p[1] << 8 | p[2] << 16 |
	              (uint32_t)p[3] << 24);
	/* The aux block has no clock before the first ResetEDIDModule. */
	if (len >= 3 && p[0] == 0x16 && p[1] < PORTS &&
	    (mmio_read_32(SH_STATUS) & ST_EDID_MODULE)) {	/* 0x12330 */
		uint8_t port = p[1], action = p[2];

		if (action == 1)
			sh_wr8(SH_HPD_COUNTER + port, sh_rd8(SH_HPD_COUNTER + port) + 1);
		else if (action == 2 || action == 3)
			hpd_pin(port, false),
			sh_wr8(SH_HPD_COUNTER + port, action == 3 ? HPD_RESET_TICKS : 0);
	} else if (len >= 1 && p[0] == 0x17) {			/* 0x120f0 */
		reset_module();
	}
}

/* The pump 0x7fd8 once more, for the MIPS: payload in the words, LSB first. */
static void
mips_poll(void)
{
	for (uint8_t q = 0; q < ARRAY_SIZE(mips); ++q) {
		while (mmio_read_32(MIPS_FIFO_STAT(q))) {
			uint32_t w = mmio_read_32(MIPS_MSG_DATA(q));

			switch (mips[q].state) {
			case 0:
				if ((w & 0xff) != BOP_MARKER)
					break;
				mips[q].len = w >> 24;
				mips[q].idx = 0;
				for (uint8_t i = 0; i < sizeof(mips[q].buf); ++i)
					mips[q].buf[i] = 0;
				mips[q].state = mips[q].len > BOP_MAX_LEN ? 0 : 1;
				break;
			case 1:
				mips[q].state = mips[q].len ? 2 : 0;
				if (!mips[q].len)
					mips_frame(mips[q].buf, 0);
				break;
			default:
				for (uint8_t b = 0; b < 4 && mips[q].idx < mips[q].len;
				     ++b, ++mips[q].idx) {
					if (mips[q].idx < sizeof(mips[q].buf))
						mips[q].buf[mips[q].idx] = w >> (8 * b);
				}
				if (mips[q].idx >= mips[q].len) {
					mips[q].state = 0;
					mips_frame(mips[q].buf, mips[q].len);
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
		if (sb_count < 2 || sb_data[1] == 0) {
			sb_reply(type, RESULT_UNKNOWN, NULL, 0);
			break;
		}
		info("H713: cpu op req, warm entry %p", (void *)sb_data[1]);
		css_set_warm_entry(sb_data[1], 0);
		udelay(CPU_OP_DELAY_US);
		css_set_power_state(0, sb_data[0] & 0xf, SCPI_CSS_OFF,
		                    SCPI_CSS_OFF, SCPI_CSS_OFF);
		break;
	case TYPE_RESTORE: {	/* answered with the wake word */
		/*
		 * Only once per resume: after a suspend Linux abandoned before
		 * TF-A's cpu op req (an abort after the noirq phase), the word
		 * of the previous standby must not come back as this one's;
		 * the answer is 0 then, and a timer armed for it is cleared.
		 */
		uint32_t wake = woke ? mmio_read_32(SH_WAKE) : 0;

		woke = false;
		mmio_write_32(H713_DIAG_WAKE_TIMER, 0);
		sb_reply(type, 0, &wake, 1);
		break;
	}
	case TYPE_WAKE_SET:	/* key and IR are armed without it; type 3 = timer */
		if (sb_count >= 1 && sb_data[0] >> 30 == WAKE_SRC_TIMER)
			mmio_write_32(H713_DIAG_WAKE_TIMER,
			              sb_data[0] & 0x3fffffff);
		sb_reply(type, 0, NULL, 0); break;
	case TYPE_WAKE_CLR:
		mmio_write_32(H713_DIAG_WAKE_TIMER, 0);
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
	bool acked = mmio_read_32(DIAG_INITS) != 0 &&
	             mmio_read_32(SH_MAGIC) == H713_MAGIC &&
	             (mmio_read_32(SH_STATUS) & ST_HANDSHAKE);

	for (uintptr_t a = H713_SHARED_BASE; a < H713_SHARED_LIMIT; a += 4) {
		if (a < H713_DIAG_BASE || a >= H713_DIAG_LIMIT)
			mmio_write_32(a, 0);
	}
	mmio_write_32(DIAG_CPUS_CLK, r_ccu_cpus_clk_at_boot());
	mmio_write_32(DIAG_INITS, mmio_read_32(DIAG_INITS) + 1);
	mmio_write_32(DIAG_INTC_80, mmio_read_32(DEV_R_INTC + 0x80));
	mmio_write_32(DIAG_INTC_88, mmio_read_32(DEV_R_INTC + 0x88));
	mmio_write_32(SH_MAGIC, H713_MAGIC);
	mmio_write_32(SH_VERSION, H713_VERSION);
	/*
	 * Which crust this is: the kernel keeps a running image only when it
	 * is the one in the file it was asked to load (same size), and shows
	 * the variant - light, self-refresh and deep share version and notify.
	 */
	mmio_write_32(H713_DIAG_BUILD,
	              (uint32_t)((uintptr_t)__bss_start - FIRMWARE_BASE) |
	              (uint32_t)CONFIG(H713_DRAM_SELF_REFRESH) << 24 |
	              (uint32_t)CONFIG(H713_DEEP_SLEEP) << 25 |
	              (uint32_t)CONFIG(H713_IR_WAKE_NECX) << 26);
	portmap_init();
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
	while (msgbox_receive(&msgbox.dev, PORT_STANDBY, &w) == SUCCESS)
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
	cir = cir_get();
	if (gpio_get(&key_pin))
		warn("H713: power key pin unavailable");
	if (!gpio_get(&led_red))
		gpio_set_value(&led_red, true);
	if (!gpio_get(&led_blue))
		gpio_set_value(&led_blue, false);
	key_low = 0;
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
	if (timer && mmio_read_32(H713_DIAG_SLEPT_MS) >= timer) {
		mmio_write_32(SH_WAKE, WAKE_TIMER);
		return WAKE_TIMER;
	}

	if (passed(key_deadline)) {
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
	/* the kernel's power key interrupt and its LED state, as before */
	mmio_clrset_32(R_PIO_PL_DATA, 0x3, pl_data_saved & 0x3);
	mmio_write_32(R_PIO_PL_PULL0, pl_pull0_saved);
	mmio_write_32(R_PIO_PL_DRV0, pl_drv0_saved);
	mmio_write_32(R_PIO_PL_DRV1, pl_drv1_saved);
	mmio_write_32(R_PIO_PL_CFG0, pl_cfg0_saved);
	sh_status(0, ST_ASLEEP);
	woke = true;
	/* The HPD tick did not run asleep: a deadline from before the standby
	 * would not expire for up to 89 s after a long one. */
	hpd_deadline = timeout_set(HPD_TICK_US);
	/* one standby per arming, as the vendor's 0x26 */
	mmio_write_32(H713_DIAG_WAKE_TIMER, 0);
	info("H713: resumed, wake word %x", mmio_read_32(SH_WAKE));
}
