#include "core_pins.h"
#include "util/delay.h"

int main(void) __attribute__((noinline));

int main(void)
{
	PORTC_PCR5 = 0x0140;
	GPIOC_PSOR = _BV(5);
	GPIOC_PDDR |= _BV(5);
	while (1) {
		GPIOC_PSOR = _BV(5);
		_delay_ms(500);
		GPIOC_PCOR = _BV(5);
		_delay_ms(1000);
	}
}
