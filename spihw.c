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
#include "spihw.h"
#include "frser-cfg.h"

static uint8_t spi_initialized = 0;

/* Always used CTAR options. */
#define CTAR_BASE SPI_CTAR_FMSZ(7) | SPI_CTAR_CSSCK(1)

#ifdef FRSER_FEAT_SPISPEED

static int spi_set_spd = 0;

// F = (F_BUS/PBR) * ((1+DBR)/BR)
// We avoid all non-50/50-duty options and
// also we're not gonna have all the possible options,
// just a nice set of frequencies down to max. 15,625kHz.
const uint32_t spi_ctar_table[23] = {
	CTAR_BASE | SPI_CTAR_DBR,
	CTAR_BASE,
	CTAR_BASE | SPI_CTAR_PBR(1),
	CTAR_BASE | SPI_CTAR_BR(1),
	CTAR_BASE | SPI_CTAR_PBR(2),
	CTAR_BASE | SPI_CTAR_BR(2),
	CTAR_BASE | SPI_CTAR_PBR(3),
	CTAR_BASE | SPI_CTAR_BR(3)
	CTAR_BASE | SPI_CTAR_PBR(1) | SPI_CTAR_BR(2),
	CTAR_BASE | SPI_CTAR_PBR(2) | SPI_CTAR_BR(1),
	CTAR_BASE | SPI_CTAR_PBR(1) | SPI_CTAR_BR(3),
	CTAR_BASE | SPI_CTAR_PBR(3) | SPI_CTAR_BR(1),
	CTAR_BASE | SPI_CTAR_BR(4),
	CTAR_BASE | SPI_CTAR_PBR(2) | SPI_CTAR_BR(3),
	CTAR_BASE | SPI_CTAR_PBR(1) | SPI_CTAR_BR(4),
	CTAR_BASE | SPI_CTAR_BR(5),
	CTAR_BASE | SPI_CTAR_PBR(2) | SPI_CTAR_BR(4),
	CTAR_BASE | SPI_CTAR_PBR(1) | SPI_CTAR_BR(5),
	CTAR_BASE | SPI_CTAR_PBR(1) | SPI_CTAR_BR(6),
	CTAR_BASE | SPI_CTAR_PBR(1) | SPI_CTAR_BR(7),
	CTAR_BASE | SPI_CTAR_PBR(1) | SPI_CTAR_BR(8),
	CTAR_BASE | SPI_CTAR_PBR(1) | SPI_CTAR_BR(9),
	CTAR_BASE | SPI_CTAR_PBR(1) | SPI_CTAR_BR(10)
};

const uint32_t spi_hz_table[23] = {
	F_BUS / 2,
	F_BUS / 4,
	F_BUS / 6,
	F_BUS / 8,
	F_BUS / 10,
	F_BUS / 12,
	F_BUS / 14,
	F_BUS / 16,
	F_BUS / 18,
	F_BUS / 20,
	F_BUS / 24,
	F_BUS / 28,
	F_BUS / 32,
	F_BUS / 40,
	F_BUS / 48,
	F_BUS / 64,
	F_BUS / 80,
	F_BUS / 96,
	F_BUS / 192,
	F_BUS / 384,
	F_BUS / 768,
	F_BUS / 1536,
	F_BUS / 3072
};

uint32_t spi_set_speed(uint32_t hz) {
	int spd;
	uint32_t hz_spd;

	/* Range check. */
	if (hz<=(F_BUS/3072)) {
		spd = 22;
		hz_spd = F_BUS/3072;
	} else {
		for (spd=0;spd<23;spd++) {
			hz_spd = spi_hz_table[spd];
			if (hz >= hz_spd) break;
		}
	}
	spi_set_spd = spd;
	if (spi_initialized) { // change speed
		spi_init(); // re-init
	}
	return hz_spd;
}

#define CTAR_VALUE spi_ctar_table[spi_set_spd]
#else
/* Well, if you didnt want to tune speed, give max... tune this if you want less and no tuning. (wut). */
#define CTAR_VALUE (CTAR_BASE | SPI_CTAR_DBR)
#endif

void spi_init(void) {
	uint32_t sim6 = SIM_SCGC6;
	if (!(sim6 & SIM_SCGC6_SPI0)) {
		SIM_SCGC6 = sim6 | SIM_SCGC6_SPI0;
	}
	/* SPI module */
	SPI0_MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1f) | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;
	SPI0_CTAR0 = CTAR_VALUE;
	SPI0_RSER = 0;
	SPI0_SR = SPI_SR_RFDF | SPI_SR_TCF;
	/* Pin configuration */
	PORTD_PCR0 = PORT_PCR_MUX(1) | PORT_PCR_DSE | PORT_PCR_PE | PORT_PCR_PS; /* CS, PD0, manual mode. */
	PORTD_PCR1 = PORT_PCR_MUX(2) | PORT_PCR_DSE; /* PD1, SCK. */
	PORTD_PCR2 = PORT_PCR_MUX(2) | PORT_PCR_DSE; /* PD2, MOSI */
	PORTD_PCR3 = PORT_PCR_MUX(2);
	GPIOD_PDDR |= _BV(2) | _BV(1) | _BV(0);
	GPIOD_PDDR &= ~_BV(3);
	GPIOD_PSOR = _BV(0); // CS high
	GPIOD_PCOR = _BV(2) | _BV(1);
	spi_initialized = 1;
}

void spi_select(void) {
	_delay_us(1);
	GPIOD_PCOR = _BV(0);
}

void spi_deselect(void) {
	_delay_us(1); /* This might be too fast for the slowest speeds. TCF "exits" 1/2 cycle early. */
	GPIOD_PSOR = _BV(0);
}

uint8_t spi_uninit(void) {
	if (spi_initialized) {
		/* CS pullup, rest tristate (non-digital, since we need to disable the mux anyways :P).*/
		PORTD_PCR1 = PORT_PCR_MUX(0);
		PORTD_PCR2 = PORT_PCR_MUX(0);
		PORTD_PCR3 = PORT_PCR_MUX(0);
		GPIOD_PDDR &= ~(_BV(0)|_BV(1)|_BV(2));
		spi_initialized = 0;
		return 1;
	}
	return 0;
}

uint8_t spi_txrx(const uint8_t c) {
	uint8_t r;
	SPI0_PUSHR = SPI_PUSHR_CONT | c;
	while ((SPI0_SR & (SPI_SR_TCF|SPI_SR_RFDF))==0);
	r = SPI0_POPR;
	SPI0_SR = SPI_SR_TCF | SPI_SR_RFDF;
	return r;
}


void spi_init_cond(void) {
	if (!spi_initialized) spi_init();
}

