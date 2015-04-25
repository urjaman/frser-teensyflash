#include "main.h"
#include "uart.h"
#include "frser.h"
#include "usb_dev.h"

int main(void)
{
	int i=0;
//	usb_init();
//	__enable_irq();
#if 1
	PORTC_PCR5 = 0x0140;
	GPIOC_PSOR = _BV(5);
	GPIOC_PDDR |= _BV(5);
	for (i=0;i<10;i++) {
		GPIOC_PSOR = _BV(5);
		_delay_ms(500);
		GPIOC_PCOR = _BV(5);
		_delay_ms(500);
	}
#endif
	frser_main();
}
