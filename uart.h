#ifndef _UART_H_
#define _UART_H_

#include "usb_serial.h"

/* These defines are what is really used: */

uint8_t usbs_recv(void);
void usbs_send(uint8_t b);
int usbs_has_data(void);

#define RECEIVE() usbs_recv()
#define SEND(n) usbs_send(n)

/* These are hokey pokey wimey dimey ..*/
#define UART_BUFLEN 2048
#define UARTTX_BUFLEN 0
#define BAUD 2000000

#endif
