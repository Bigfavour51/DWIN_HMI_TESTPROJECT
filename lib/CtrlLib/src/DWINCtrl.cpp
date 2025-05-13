#include "DWINCtrl.h"
#include "globals.h"



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