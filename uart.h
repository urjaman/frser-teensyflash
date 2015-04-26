
#ifndef _UART_H_
#define _UART_H_

#include "usb_serial.h"

/* These defines are what is really used: */

uint8_t usb_recv(void);

#define RECEIVE() usb_recv()
#define SEND(n) usb_serial_putchar(n)

/* These are hokey pokey wimey dimey ..*/
#define UART_BUFLEN 3072
#define UARTTX_BUFLEN 0
#define BAUD 2000000

#endif
