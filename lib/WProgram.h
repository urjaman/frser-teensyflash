#ifndef WProgram_h
#define WProgram_h

#include <stdlib.h>
#include <string.h>
#include <math.h>

// some libraries and sketches depend on this
// AVR stuff, assuming Arduino.h or WProgram.h
// automatically includes it...
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "avr_functions.h"
#include "wiring.h"

#define DMAMEM __attribute__ ((section(".dmabuffers"), used))
#define FASTRUN __attribute__ ((section(".fastrun")))

#ifdef __cplusplus

#include "avr_emulation.h"
#include "usb_serial.h"

//#include "WCharacter.h"
#include "elapsedMillis.h"

uint16_t makeWord(uint16_t w);
uint16_t makeWord(byte h, byte l);

#define word(...) makeWord(__VA_ARGS__)

#include "pins_arduino.h"

#endif // __cplusplus

#endif // WProgram_h
