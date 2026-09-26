/*
 * Copyright © 2017-2026 The Crust Firmware Authors.
 * SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 */

#ifndef PLATFORM_IRQ_H
#define PLATFORM_IRQ_H

/*
 * R_INTC line numbers of the ARISC, from the vendor firmware's wake-source
 * table 0x15a40 ({ARISC irq, GIC SPI + 0x20}, FB-STANDBY excerpt F) and its
 * ISR registrations (0x6058 callers). 96 lines in three registers.
 */
#define IRQ_NMI      0x00 /* SPI 140 */
#define IRQ_R_TIMER0 0x01 /* guess, H6 numbering */
#define IRQ_R_TIMER1 0x02
#define IRQ_R_ALARM0 0x09 /* SPI 147, the RTC alarm */
#define IRQ_R_PIO_PL 0x0e /* SPI 149: the power key on PL4 */
#define IRQ_UNK_0F   0x0f /* SPI 148 */
#define IRQ_R_PIO_PM 0x10 /* SPI 151: bt_powerkey */
#define IRQ_UNK_11   0x11 /* SPI 150 */
#define IRQ_USB      0x12 /* SPI 30..36, one line for all controllers */
#define IRQ_R_CIR_RX 0x20 /* SPI 155 */
#define IRQ_HDMI_CEC 0x28 /* registered at 0xff44 with handler 0x111b4 */

#define IRQ_MSGBOX   0x13 /* guess, H6 numbering; the blob polls instead */

#endif /* PLATFORM_IRQ_H */
