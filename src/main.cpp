// // #include <Arduino.h>

// // Define sensor addresses (ensure these match your DGUS Tool configuration)
// // #define TEMP_ADDR      0x1000
// // #define HUMIDITY_ADDR  0x2000
// // #define POWER_ADDR     0x3000
// // #define VOLTAGE_ADDR   0x4000
// // #define PRESSURE_ADDR  0x5100
// // #define ENERGY_ADDR    0x5200

// // void sendToDWIN(uint16_t address, int value);

// // void setup() {
// //   Serial.begin(115200);
// //   // Initialize your sensors here
// // }

// // void loop() {
// //   // Read sensor data
// //   int temperature = 10;       // Replace with actual sensor reading
// //   int humidity = 24;             // Replace with actual sensor reading
// //   int power = 32;                   // Replace with actual sensor reading
// //   int voltage = 43;               // Replace with actual sensor reading
// //   int pressure = 65;             // Replace with actual sensor reading
// //   int energy = 12;                 // Replace with actual sensor reading

// //   // Send data to DWIN display
// //   sendToDWIN(TEMP_ADDR, temperature);
// //   sendToDWIN(HUMIDITY_ADDR, humidity);
// //   sendToDWIN(POWER_ADDR, power);
// //   sendToDWIN(VOLTAGE_ADDR, voltage);
// //   sendToDWIN(PRESSURE_ADDR, pressure);
// //   sendToDWIN(ENERGY_ADDR, energy);

// //   delay(1000); // Wait for a second before next update
// // }

// // // Function to send data to DWIN display
// // void sendToDWIN(uint16_t address, int value) {
// //   byte packet[8];
// //   packet[0] = 0x5A;                 // Start byte 1
// //   packet[1] = 0xA5;                 // Start byte 2
// //   packet[2] = 0x05;                 // Length of the remaining bytes
// //   packet[3] = 0x82;                 // Write variable command
// //   packet[4] = highByte(address);    // Address high byte
// //   packet[5] = lowByte(address);     // Address low byte
// //   packet[6] = highByte(value);      // Data high byte
// //   packet[7] = lowByte(value);       // Data low byte

// //   Serial.write(packet, 8);          // Send the packet over serial
// // }























































// #include <Arduino.h>

// #define DEBUG 1              // Use correct spelling
// const bool level = 0;        // 0 = active LOW relay
// const byte relays_num = 6;   // Now controlling only 6 relays
// byte relays[relays_num] = {2, 3, 4, 5, 6, 7}; // Pins for relays

// // Serial buffer
// byte Buffer[9];
// byte Buffer_Len = 0;
// bool flag = false;
// void ReadSerial();
// void setup() {
//   if (DEBUG) {
//     Serial.begin(115200);
//     Serial.println("Start program");
//   }

//   for (byte i = 0; i < relays_num; i++) {
//     pinMode(relays[i], OUTPUT);
//     if (!level)
//       digitalWrite(relays[i], HIGH);  // Default OFF state
//   }
// }

// void loop() {
//   ReadSerial();
// }

// void ReadSerial() {
//   while (Serial.available()) {
//     byte incoming = Serial.read();

//     if (Buffer_Len < sizeof(Buffer)) {
//       Buffer[Buffer_Len++] = incoming;
//     } else {
//       // Prevent overflow
//       Buffer_Len = 0;
//     }

//     // When we have all 9 bytes, process it
//     if (Buffer_Len == 9) {
//       flag = true;
//       Buffer_Len = 0;

//       if (DEBUG) {
//         for (byte i = 0; i < 9; i++) {
//           Serial.print(Buffer[i], HEX); Serial.print(" ");
//         }
//         Serial.println();
//       }

//       // Check header and command
//       if (Buffer[0] == 0x5A && Buffer[4] == 0x50) {
//         byte relay_states = Buffer[8];

//         if (DEBUG) {
//           Serial.println(relay_states, BIN);
//         }

//         for (byte i = 0; i < relays_num; i++) {
//           bool bit_state = bitRead(relay_states, i);
//           digitalWrite(relays[i], level == 0 ? !bit_state : bit_state);
//         }
//       }
//     }
//   }
// }



















/* For Software serial */
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

/* Adresses of all sensors */


#define temperature_add   0x1000
#define humidity_add      0x1002
#define power_add         0x1004
#define voltage_add       0x1006
#define ATpressure_add    0x1008
#define Energy_add        0x100A

unsigned char   Temperature[8] = {0x5a, 0xa5, 0x05, 0x82, highByte(temperature_add), lowByte(temperature_add), 0x00, 0x00};
unsigned char      Humidity[8] = {0x5a, 0xa5, 0x05, 0x82, highByte(humidity_add), lowByte(humidity_add), 0x00, 0x00};
unsigned char   Power[8] = {0x5a, 0xa5, 0x05, 0x82, highByte(power_add), lowByte(power_add), 0x00, 0x00};
unsigned char      Voltage[8] = {0x5a, 0xa5, 0x05, 0x82, highByte(voltage_add), lowByte(voltage_add), 0x00, 0x00};
unsigned char      ATpressure[8] = {0x5a, 0xa5, 0x05, 0x82, highByte(ATpressure_add), lowByte(ATpressure_add), 0x00, 0x00};
unsigned char      Energy[8] = {0x5a, 0xa5, 0x05, 0x82, highByte(Energy_add), lowByte(Energy_add), 0x00, 0x00};



void Data_Arduino_to_Display();
void Data_Display_to_Arduino();
double dewPointFast(double celsius, double humidity);

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
  Data_Arduino_to_Display();
  delay(50);
  ReadSerial();
  delay(80);
}

// void Data_Display_to_Arduino()
// {
//   if (Serial.available())
//   {
//     for (int i = 0; i <= 8; i++) //this loop will store whole frame in buffer array.
//     {
//       Buffer[i] = Serial.read();
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
//             digitalWrite(light, LOW);
//           }
//           else
//           {
//             digitalWrite(light, HIGH);
//           }
//           break;

//         case 0x53:   //for AC
//           if (Buffer[8] == 1)
//           {
//             digitalWrite(motor, LOW);
//           }
//           else
//           {
//             digitalWrite(motor, HIGH);
//           }
//           break;


//         case 0x54:   //for TV
//           if (Buffer[8] == 1)
//           {
//             digitalWrite(refrigirator, LOW);
//           }
//           else
//           {
//             digitalWrite(refrigirator, HIGH);
//           }
//           break;
//           case 0x55:   //for TV
//           if (Buffer[8] == 1)
//           {
//             digitalWrite(plugload, LOW);
//           }
//           else
//           {
//             digitalWrite(plugload, HIGH);
//           }
//           break;

//         default:
//           break;
//       }
//     }
//   }
// }

void Data_Arduino_to_Display()
{
  int t = 10;
  int h = 20;
  int p = 30;
  int v = 40;
  int ap = dewPointFast(t, h);
  int e = 50;
  /*------Print data to Serial Monitor------*/
   Serial.print("Temperature = ");
  Serial.print(t);
  Serial.println(" °C");

  Serial.print("Humidity = ");
  Serial.print(h);
  Serial.println(" %");

  Serial.print("Pressure = ");
  Serial.print(p);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(v);
  Serial.println(" m");

  Serial.print("Atmospheric Pressure = ");
  Serial.print(ap);
  Serial.println(" °C");
  Serial.println();

  Serial.print("Energy = ");
  Serial.print(e);
  Serial.println(" KWh");
  Serial.println();
  

  Temperature[6] = highByte(t);
  Temperature[7] = lowByte(t);
  Serial.write(Temperature, 8);
  Humidity[6] = highByte(h);
  Humidity[7] = lowByte(h);
  Serial.write(Humidity, 8);
  Power[6] = highByte(p);
  Power[7] = lowByte(p);
  Serial.write(Power, 8);
  Voltage[6] = highByte(v);
  Voltage[7] = lowByte(v);
  Serial.write(Voltage, 8);
  ATpressure[6] = highByte(v);
  ATpressure[7] = lowByte(v);
  Serial.write(ATpressure, 8);
  Energy[6] = highByte(ap);
  Energy[7] = lowByte(ap);
  Serial.write(Energy, 8);
}

double dewPointFast(double celsius, double humidity)
{
  double a = 17;
  double b = 237;
  double temp = (a * celsius) / (b + celsius) + log(humidity * 0.01);
  double Td = (b * temp) / (a - temp);
  return Td;
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
