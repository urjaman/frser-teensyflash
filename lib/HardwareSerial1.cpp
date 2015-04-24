#include "HardwareSerial.h"

#ifdef HW_SERIAL
HardwareSerial Serial1;
#endif

void serialEvent1() __attribute__((weak));
void serialEvent1() {}
