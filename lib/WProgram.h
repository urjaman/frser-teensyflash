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

#endif // WProgram_h
