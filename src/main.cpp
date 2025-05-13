#include "DWINAuth.h"
#include "DWINCtrl.h"
#include "DWINMonitor.h"


unsigned long lastSensorTime = 0;
const unsigned long sensorInterval = 2000; // every 2 seconds
const unsigned long displayInterval = 1000; // every 1 second

const bool level = 0;        // 0 = active LOW relay
const byte relays_num = 6;   // Now controlling only 6 relays
byte relays[relays_num] = {2, 3, 4, 5, 6, 7}; // Pins for relays


// Serial buffer
byte Buffer[9];
byte Buffer_Len = 0;
bool flag = false;
void ReadSerial();

/* Adresses of all sensors */
unsigned char Temperature[8] = {0x5a, 0xa5, 0x05, 0x82, highByte(0x1000), lowByte(0x1000), 0x00, 0x00};
unsigned char Humidity[8] = {0x5a, 0xa5, 0x05, 0x82, highByte(0x1002), lowByte(0x1002), 0x00, 0x00};
unsigned char Power[8] = {0x5a, 0xa5, 0x05, 0x82, highByte(0x1004), lowByte(0x1004), 0x00, 0x00};
unsigned char Voltage[8] = {0x5a, 0xa5, 0x05, 0x82, highByte(0x1006), lowByte(0x1006), 0x00, 0x00};
unsigned char ATpressure[8] = {0x5a, 0xa5, 0x05, 0x82, highByte(0x1008), lowByte(0x1008), 0x00, 0x00};
unsigned char Energy[8] = {0x5a, 0xa5, 0x05, 0x82, highByte(0x100A), lowByte(0x100A), 0x00, 0x00};


void setup()
{
  if (DEBUG) {
        Serial.begin(115200);
        Serial.println("Start program");
      }
    
      for (byte i = 0; i < relays_num; i++) {
        pinMode(relays[i], OUTPUT);
        if (!level)
          digitalWrite(relays[i], HIGH);  // Default OFF state
      }
}

void loop()
{
  ReadSerial();
  // Send sensor data every sensorInterval ms
  if (millis() - lastSensorTime > sensorInterval) {
    lastSensorTime = millis();
    Data_Arduino_to_Display();
  }
  checkLoginRequest();
}
