#include <avr/pgmspace.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <setjmp.h>
#include "core_pins.h"
#include "util/delay.h"


#define LED_ON() do { GPIOC_PSOR = _BV(5); } while(0)
#define LED_OFF() do { GPIOC_PCOR = _BV(5); } while(0)
