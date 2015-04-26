
#ifndef _SPIHW_H_
#define _SPIHW_H_
/* This will give spi_set_speed if needed. A bit illogically maybe, but anyways... */
#include "frser-flashapi.h"

void spi_init(void);
uint8_t spi_uninit(void);
uint8_t spi_txrx(const uint8_t c);
void spi_init_cond(void);

void spi_select(void);
void spi_deselect(void);

#endif
