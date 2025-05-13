#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

extern byte Buffer[9];
extern byte Buffer_Len;
extern bool flag;

extern const bool level;
extern const byte relays_num;
extern byte relays[6]; // match size if changed

extern unsigned char Temperature[8];
extern unsigned char Humidity[8];
extern unsigned char Power[8];
extern unsigned char Voltage[8];
extern unsigned char ATpressure[8];
extern unsigned char Energy[8];

#endif
