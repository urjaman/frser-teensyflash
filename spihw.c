#include "main.h"
#include "spihw.h"
#include "frser-cfg.h"

static uint8_t spi_initialized = 0;

#ifdef FRSER_FEAT_SPISPEED

uint32_t spi_set_speed(uint32_t hz) {
	return hz;
}


void spi_init(void) {
	spi_initialized = 1;
}
#else
void spi_init(void) {
	spi_initialized = 1;
}
#endif

uint8_t spi_uninit(void) {
	if (spi_initialized) {
		spi_initialized = 0;
		return 1;
	}
	return 0;
}

uint8_t spi_txrx(const uint8_t c) {
	uint8_t r=c;
	return r;
}

/* This is here only to keep spi_initialized a static variable,
 * hidden from other bits of code. */

void spi_init_cond(void) {
	if (!spi_initialized) spi_init();
}

/* Select and deselect are not part of the "domain" of this .c / target-specific */
