/*
	This file was part of bbflash, now frser-m328lpcspi.
	Copyright (C) 2013, Hao Liu and Robert L. Thompson
	Copyright (C) 2013 Urja Rannikko <urjaman@gmail.com>

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

#define delay() asm volatile ("nop")
#define swap(x) do { x = (((x)>>4)&0xF) | (((x)<<4)&0xF0); } while(0)



#define FRAME_DDR			DDRD
#define FRAME_PORT			PORTD
#define FRAME				PD3


#define INIT_DDR			DDRD
#define INIT_PORT			PORTD
#define INIT				PD4

void nibble_set_dir(uint8_t dir) {
//	if (!dir) {
//		DDRC = 0;
//	}
}

uint8_t nibble_read(void) {
//	uint8_t rv;
//	rv = PINC & 0xF;
	return 0;
}

static void nibble_write_hi(uint8_t data) {
//	swap(data);
//	DDRC = (~data) & 0xF;
//	data &= 0xF;
//	while ((PINC & 0xF) != data);
}

void nibble_write(uint8_t data) {
//	DDRC = (~data) & 0xF;
//	data &= 0xF;
//	while ((PINC & 0xF) != data);
}

#define clock_low() do { CLK_PORT &= ~_BV(CLK); } while(0)
#define clock_high() do { CLK_PORT |= _BV(CLK); } while(0)



bool nibble_init(void) {
#if 0
	uint8_t i;

	INIT_PORT &= ~_BV(INIT);
	INIT_DDR |= _BV(INIT);

	CLK_DDR |= _BV(CLK);
	CLK_PORT |= _BV(CLK);

	FRAME_DDR |= _BV(FRAME);
	FRAME_PORT |=  _BV(FRAME);

	nibble_set_dir(OUTPUT);
	nibble_write(0);

	for (i = 0; i < 24; i++)
		clock_cycle();
	INIT_DDR &= ~_BV(INIT);
	_delay_us(1); // Let pullup work
	for (i = 0; i < 42; i++)
		clock_cycle();

#endif
	return true;
}

void nibble_cleanup(void) {
#if 0
	CLK_DDR &= ~_BV(CLK);
	FRAME_DDR &= ~_BV(FRAME);
	nibble_set_dir(INPUT);
#endif
}

void clocked_nibble_write(uint8_t value) {
#if 0
	clock_low();
	nibble_write(value);
	clock_high();
#endif
}

void clocked_nibble_write_hi(uint8_t value) {
#if 0
	clock_low();
	nibble_write_hi(value);
	clock_high();
#endif
}

uint8_t clocked_nibble_read(void) {
#if 0
	clock_cycle();
	delay();
	delay();
	return nibble_read();
#else
	return 0;
#endif
}

void nibble_start(uint8_t start) {
#if 0
	FRAME_PORT |= _BV(FRAME);
	nibble_set_dir(OUTPUT);
	clock_high();
	FRAME_PORT &= ~_BV(FRAME);
	nibble_write(start);
	clock_cycle();
	FRAME_PORT |= _BV(FRAME);
#endif
}

void nibble_hw_init(void) {
	/* Do your port init if needed etc... */

#if 0
	/* Kick reset here so lpc/fwh.c doesnt need to know about how it is controlled. */
	DDRD |= _BV(2); //!RST
	_delay_us(1);
	DDRD &= ~_BV(2);
	_delay_us(1); // slow pullup
#endif
}
