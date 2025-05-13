#ifndef __DWINMONITOR__
#define __DWINMONITOR__

#include <Arduino.h>
#include "globals.h"


/* Adresses of all sensors */
#define temperature_add   0x1000
#define humidity_add      0x1002
#define power_add         0x1004
#define voltage_add       0x1006
#define ATpressure_add    0x1008
#define Energy_add        0x100A


void Data_Arduino_to_Display();
double dewPointFast(double celsius, double humidity);




#endif // __DWINMONITOR__