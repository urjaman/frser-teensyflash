/*
	This file was part of bbflash, now frser-teensyflash.
	Copyright (C) 2013, Hao Liu and Robert L. Thompson
	Copyright (C) 2013,2015 Urja Rannikko <urjaman@gmail.com>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "main.h"
#include "nibble.h"


#define nopx() asm volatile("nop")

#define delay() nopx()

/* CLK: PC0
 * FRAME: PC1
 * INIT: PC6
 * RST: PC7
 * LAD0: PD4
 * LAD1: PD5
 * LAD2: PD6
 * LAD3: PD7
 */

void nibble_set_dir(uint8_t dir) {
	if (!dir) {
		GPIOD_PDDR &= ~(0xF0);
	} else {
		GPIOD_PDDR |= 0xF0;
	}
}

uint8_t nibble_read(void) {
	return (GPIOD_PDIR>>4)&0x0F;
}

static void nibble_write_hi(uint8_t data) {
	GPIOD_PDOR = (GPIOD_PDOR & ~0xF0) | (data & 0xF0);
}

void nibble_write(uint8_t data) {
	GPIOD_PDOR = (GPIOD_PDOR & ~0xF0) | (data << 4);
}

#define clock_low() do { GPIOC_PCOR = _BV(0); } while(0)
#define clock_high() do { GPIOC_PSOR = _BV(0); } while(0)

void clock_cycle(void) {
	clock_low();
	delay();
	clock_high();
}


bool nibble_init(void) {
	int i;

	GPIOC_PCOR = _BV(6); // INIT=0

	clock_high();

	GPIOC_PSOR = _BV(1); // FRAME

	nibble_set_dir(OUTPUT);
	nibble_write(0);

	for (i = 0; i < 24; i++)
		clock_cycle();
	GPIOC_PSOR = _BV(6); // INIT=1
	for (i = 0; i < 42; i++)
		clock_cycle();

	return true;
}

void nibble_cleanup(void) {
	GPIOC_PDDR &= ~(_BV(7) | _BV(6) | _BV(1) | _BV(0));
	nibble_set_dir(INPUT);
}

void clocked_nibble_write(uint8_t value) {
	clock_low();
	nibble_write(value);
	clock_high();
}

void clocked_nibble_write_hi(uint8_t value) {
	clock_low();
	nibble_write_hi(value);
	clock_high();
}

uint8_t clocked_nibble_read(void) {
	clock_cycle();
	delay();
	return nibble_read();
}

void nibble_start(uint8_t start) {
	GPIOC_PSOR = _BV(1); // FRAME=1
	nibble_set_dir(OUTPUT);
	clock_high();
	GPIOC_PCOR = _BV(1); // FRAME=0
	nibble_write(start);
	clock_cycle();
	GPIOC_PSOR = _BV(1); // FRAME=1
}


void nibble_hw_init(void) {
	/* Port Init */
	PORTC_PCR0 = PORT_PCR_MUX(1) | PORT_PCR_DSE | PORT_PCR_PE | PORT_PCR_PS;
	PORTC_PCR1 = PORT_PCR_MUX(1) | PORT_PCR_DSE | PORT_PCR_PE | PORT_PCR_PS;
	PORTC_PCR6 = PORT_PCR_MUX(1) | PORT_PCR_DSE | PORT_PCR_PE | PORT_PCR_PS;
	PORTC_PCR7 = PORT_PCR_MUX(1) | PORT_PCR_DSE | PORT_PCR_PE | PORT_PCR_PS;

	PORTD_PCR4 = PORT_PCR_MUX(1) | PORT_PCR_DSE | PORT_PCR_PE | PORT_PCR_PS;
	PORTD_PCR5 = PORT_PCR_MUX(1) | PORT_PCR_DSE | PORT_PCR_PE | PORT_PCR_PS;
	PORTD_PCR6 = PORT_PCR_MUX(1) | PORT_PCR_DSE | PORT_PCR_PE | PORT_PCR_PS;
	PORTD_PCR7 = PORT_PCR_MUX(1) | PORT_PCR_DSE | PORT_PCR_PE | PORT_PCR_PS;

	GPIOC_PDDR |= _BV(7) | _BV(6) | _BV(1) | _BV(0);
	GPIOC_PSOR = _BV(7) | _BV(6) | _BV(1) | _BV(0);
	GPIOD_PDDR &= ~(0xF0);
	GPIOD_PSOR = 0xF0;

	/* Kick reset here so lpc/fwh.c doesnt need to know about how it is controlled. */
	GPIOC_PCOR = _BV(7); // RST=0
	_delay_us(1);
	GPIOC_PSOR = _BV(7); // RST=1
}
