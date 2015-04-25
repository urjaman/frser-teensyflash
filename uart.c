#include "main.h"
#include "uart.h"

/* Wrap out the -1 return ... */
uint8_t usb_recv(void) {
	int c=-1;
	do {
		if (usb_serial_available()) {
			c = usb_serial_getchar();
		}
		yield();
	} while(c==-1);
	return c;
}
