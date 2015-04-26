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
