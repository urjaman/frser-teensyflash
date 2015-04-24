#include "HardwareSerial.h"

#ifdef HW_SERIAL2
HardwareSerial2 Serial2;
#endif

void serialEvent2() __attribute__((weak));
void serialEvent2() {}
