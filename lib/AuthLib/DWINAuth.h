#ifndef DWINAuth_h
#define DWINAuth_h
#include <Arduino.h>
#include "globals.h"



#define DEBUG 1  

// Expected credentials
const String valid_email = "nameless.com";
const String valid_pass  = "nameless";


#define DWIN_email_ADDX 0x2100
#define DWIN_password_ADDX 0x2200
#define DWIN_logintrigger_ADDX 0x2300
#define login_result_add   0x2302  // Where you will send 1 = success, 2 = failure





void checkLoginRequest();
void handleLogin();
void sendLoginResult(byte value);
void sendByteToVP(uint16_t addr, byte value);
String readStringFromVP(uint16_t addr);





#endif