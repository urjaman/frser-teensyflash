/*
	This file is part of frser-teensyflash.
	Copyright (C) 2015 Urja Rannikko <urjaman@gmail.com>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "main.h"
#include "uart.h"

static int usb_rxbuf_wp = 0;
static int usb_rxbuf_rp = 0;

static uint8_t usb_rxbuf[UART_BUFLEN];

static void usb_rxbuf_put(uint8_t b)
{
	usb_rxbuf[usb_rxbuf_wp++] = b;
	if (usb_rxbuf_wp >= UART_BUFLEN) usb_rxbuf_wp = 0;
}

static int usb_rxbuf_get(void)
{
	int r = -1;
	if (usb_rxbuf_rp != usb_rxbuf_wp) {
		r = usb_rxbuf[usb_rxbuf_rp++];
		if (usb_rxbuf_rp >= UART_BUFLEN) usb_rxbuf_rp = 0;
	}
	return r;
}

/* Wrap out the -1 return ... */
uint8_t usbs_recv(void) {
	int c=-1;
	do {
		while (usb_serial_available()) {
			usb_rxbuf_put(usb_serial_getchar());
		}
		c = usb_rxbuf_get();
		yield();
		if (c==-1) {
			asm volatile ("wfi");
		}
	} while(c==-1);
	return c;
}

int usbs_has_data(void) {
	if (usb_rxbuf_wp != usb_rxbuf_rp) return 1;
	return usb_serial_available();
}

void usbs_send(uint8_t b) {
	int c = -1;
	do {
		while (usb_serial_available()) {
			usb_rxbuf_put(usb_serial_getchar());
		}
		if (usb_serial_write_buffer_free()) {
			c = usb_serial_putchar(b);
		}
		yield();
	} while (c==-1);
}
