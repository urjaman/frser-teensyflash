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
#include "frser.h"
//#include "usb_dev.h"
int main(void)
{
#if 1
	int i=0;
	PORTC_PCR5 = 0x0140;
	GPIOC_PSOR = _BV(5);
	GPIOC_PDDR |= _BV(5);
	for (i=0;i<3;i++) {
		GPIOC_PSOR = _BV(5);
		_delay_ms(100);
		GPIOC_PCOR = _BV(5);
		_delay_ms(100);
	}

#endif
	frser_main();
}
