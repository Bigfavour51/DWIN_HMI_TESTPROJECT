#include <Arduino.h>

#define DEBUG 1              // Use correct spelling
const bool level = 0;        // 0 = active LOW relay
const byte relays_num = 6;   // Now controlling only 6 relays
byte relays[relays_num] = {2, 3, 4, 5, 6, 7}; // Pins for relays

// Serial buffer
byte Buffer[9];
byte Buffer_Len = 0;
bool flag = false;
void ReadSerial();
void setup() {
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

void loop() {
  ReadSerial();
}

void ReadSerial() {
  while (Serial.available()) {
    byte incoming = Serial.read();

    if (Buffer_Len < sizeof(Buffer)) {
      Buffer[Buffer_Len++] = incoming;
    } else {
      // Prevent overflow
      Buffer_Len = 0;
    }

    // When we have all 9 bytes, process it
    if (Buffer_Len == 9) {
      flag = true;
      Buffer_Len = 0;

      if (DEBUG) {
        for (byte i = 0; i < 9; i++) {
          Serial.print(Buffer[i], HEX); Serial.print(" ");
        }
        Serial.println();
      }

      // Check header and command
      if (Buffer[0] == 0x5A && Buffer[4] == 0x50) {
        byte relay_states = Buffer[8];

        if (DEBUG) {
          Serial.println(relay_states, BIN);
        }

        for (byte i = 0; i < relays_num; i++) {
          bool bit_state = bitRead(relay_states, i);
          digitalWrite(relays[i], level == 0 ? !bit_state : bit_state);
        }
      }
    }
  }
}





























































// /* For Software serial */
// #include <Arduino.h>
// #include <stdint.h>
// #include <SoftwareSerial.h>
// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BME280.h>


// #define RXPIN 2
// #define TXPIN 3
// SoftwareSerial mySerial (RXPIN, TXPIN);


// /* For BME280 sensor */
// #define SEALEVELPRESSURE_HPA (1013.25)
// Adafruit_BME280 bme; // I2C
// /* appliances pins */
// int light = 4;
// int fan = 5;
// int ac = 6;
// int tv = 7;
// /* Adresses of all sensors */

// unsigned char Buffer[9];
// #define temperature_add   0x10
// #define humidity_add     0x20
// #define pressure_add   0x30
// #define altitude_add     0x40
// #define dewpoint_add     0x50
// unsigned char   Temperature[8] = {0x5a, 0xa5, 0x05, 0x82, temperature_add , 0x00, 0x00, 0x00};
// unsigned char      Humidity[8] = {0x5a, 0xa5, 0x05, 0x82, humidity_add, 0x00, 0x00, 0x00};
// unsigned char   Pressure[8] = {0x5a, 0xa5, 0x05, 0x82, pressure_add , 0x00, 0x00, 0x00};
// unsigned char      Altitude[8] = {0x5a, 0xa5, 0x05, 0x82, altitude_add, 0x00, 0x00, 0x00};
// unsigned char      DewPoint[8] = {0x5a, 0xa5, 0x05, 0x82, dewpoint_add, 0x00, 0x00, 0x00};


// void Data_Arduino_to_Display();
// void Data_Display_to_Arduino();
// double dewPointFast(double celsius, double humidity);

// void setup()
// {
//   Serial.begin(9600);
//   mySerial.begin(115200);
//   unsigned status;
//   // status = bme.begin();
//   // if (!status) {
//   //   Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
//   //   while (1) delay(10);
//   // }
//   pinMode(light, OUTPUT);
//   digitalWrite(light, HIGH);
//   pinMode(fan, OUTPUT);
//   digitalWrite(fan, HIGH);
//   pinMode(ac, OUTPUT);
//   digitalWrite(ac, HIGH);
//   pinMode(tv, OUTPUT);
//   digitalWrite(tv, HIGH);
// }

// void loop()
// {
//   Data_Arduino_to_Display();
//   delay(50);
//   Data_Display_to_Arduino();
//   delay(80);
// }

// void Data_Display_to_Arduino()
// {
//   if (mySerial.available())
//   {
//     for (int i = 0; i <= 8; i++) //this loop will store whole frame in buffer array.
//     {
//       Buffer[i] = mySerial.read();
//     }

//     if (Buffer[0] == 0X5A)
//     {
//       switch (Buffer[4])
//       {
//         case 0x51:   //for light
//           if (Buffer[8] == 1)
//           {
//             digitalWrite(light, LOW);
//           }
//           else
//           {
//             digitalWrite(light, HIGH);
//           }
//           break;

//         case 0x52:   //for fan
//           if (Buffer[8] == 1)
//           {
//             digitalWrite(fan, LOW);
//           }
//           else
//           {
//             digitalWrite(fan, HIGH);
//           }
//           break;

//         case 0x53:   //for AC
//           if (Buffer[8] == 1)
//           {
//             digitalWrite(ac, LOW);
//           }
//           else
//           {
//             digitalWrite(ac, HIGH);
//           }
//           break;


//         case 0x54:   //for TV
//           if (Buffer[8] == 1)
//           {
//             digitalWrite(tv, LOW);
//           }
//           else
//           {
//             digitalWrite(tv, HIGH);
//           }
//           break;

//         default:
//           break;
//       }
//     }
//   }
// }

// void Data_Arduino_to_Display()
// {
//   int t = bme.readTemperature();
//   int h = bme.readHumidity();
//   int p = bme.readPressure() / 100.0F;
//   int a = bme.readAltitude(SEALEVELPRESSURE_HPA);
//   int d = dewPointFast(t, h);
//   /*------Print data to Serial Monitor------*/
//     Serial.print("Temperature = ");
//   Serial.print(t);
//   Serial.println(" °C");

//   Serial.print("Humidity = ");
//   Serial.print(h);
//   Serial.println(" %");

//   Serial.print("Pressure = ");
//   Serial.print(p);
//   Serial.println(" hPa");

//   Serial.print("Approx. Altitude = ");
//   Serial.print(a);
//   Serial.println(" m");

//   Serial.print("DewPoint = ");
//   Serial.print(d);
//   Serial.println(" °C");
//   Serial.println();

//   Temperature[6] = highByte(t);
//   Temperature[7] = lowByte(t);
//   mySerial.write(Temperature, 8);
//   Humidity[6] = highByte(h);
//   Humidity[7] = lowByte(h);
//   mySerial.write(Humidity, 8);
//   Pressure[6] = highByte(p);
//   Pressure[7] = lowByte(p);
//   mySerial.write(Pressure, 8);
//   Altitude[6] = highByte(a);
//   Altitude[7] = lowByte(a);
//   mySerial.write(Altitude, 8);
//   DewPoint[6] = highByte(d);
//   DewPoint[7] = lowByte(d);
//   mySerial.write(DewPoint, 8);
// }
// /*----------DewPoint Calculation--------*/
// double dewPointFast(double celsius, double humidity)
// {
//   double a = 17.271;
//   double b = 237.7;
//   double temp = (a * celsius) / (b + celsius) + log(humidity * 0.01);
//   double Td = (b * temp) / (a - temp);
//   return Td;
// }
