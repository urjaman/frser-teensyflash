#include "HardwareSerial.h"

#ifdef HW_SERIAL3
HardwareSerial3 Serial3;
#endif

void serialEvent3() __attribute__((weak));
void serialEvent3() {}
