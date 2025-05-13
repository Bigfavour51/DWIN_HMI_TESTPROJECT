#include "DWINAuth.h"






void checkLoginRequest() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 500) return; // Check every 500ms
  lastCheck = millis();

  // Request value from VP 0x2300 (login trigger)
  Serial.write(0x5A); Serial.write(0xA5);
  Serial.write(0x04); Serial.write(0x83);
  Serial.write(highByte(DWIN_logintrigger_ADDX));
  Serial.write(lowByte(DWIN_logintrigger_ADDX));

  delay(100); // Give it time to reply

  if (Serial.available() >= 9) {
    byte loginBuf[9];
    for (byte i = 0; i < 9; i++) loginBuf[i] = Serial.read();

    // If login trigger == 1, process login
    if (loginBuf[0] == 0x5A && loginBuf[1] == 0xA5 && loginBuf[4] == highByte(DWIN_logintrigger_ADDX) && loginBuf[5] == lowByte(DWIN_logintrigger_ADDX)) {
      if (loginBuf[7] == 0x00 && loginBuf[8] == 0x01) {  // Value == 1
        if (DEBUG) {
          Serial.println("Login trigger received");
        }
        handleLogin();  // Trigger authentication
      }
    }
  }
}

void handleLogin() {
  String email = readStringFromVP(DWIN_email_ADDX);
  String pass  = readStringFromVP(DWIN_password_ADDX );

  if (DEBUG) {
    Serial.print("Email: "); Serial.println(email);
    Serial.print("Pass: "); Serial.println(pass);
  }

  if (email == valid_email && pass == valid_pass) {
    sendLoginResult(1); // Success
  } else {
    sendLoginResult(2); // Failure
  }

  // Reset login trigger back to 0
  sendByteToVP(DWIN_logintrigger_ADDX, 0x00);
}

void sendLoginResult(byte value) {
  byte buf[8] = {0x5A, 0xA5, 0x05, 0x82, highByte(login_result_add), lowByte(login_result_add), 0x00, value};
  Serial.write(buf, 8);
}

void sendByteToVP(uint16_t addr, byte value) {
  byte buf[8] = {0x5A, 0xA5, 0x05, 0x82, highByte(addr), lowByte(addr), 0x00, value};
  Serial.write(buf, 8);
}

String readStringFromVP(uint16_t addr) {
  String result = "";
  for (byte i = 0; i < 10; i++) { // Assume max 10 characters
    Serial.write(0x5A); Serial.write(0xA5);
    Serial.write(0x04); Serial.write(0x83);
    Serial.write(highByte(addr + i));
    Serial.write(lowByte(addr + i));
    delay(50); // Wait for reply

    if (Serial.available() >= 9) {
      byte r[9];
      for (byte j = 0; j < 9; j++) r[j] = Serial.read();
      if (r[6] == 0x00 && r[7] == 0x00) break; // Null-terminator
      result += (char)r[8];
    }
  }
  return result;
}
