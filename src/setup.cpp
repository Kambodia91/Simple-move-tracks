//------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------ 
#include <Arduino.h>
#include <ArduinoLogger.h>                         // [Serial / Terminal]
#include "defines.h"
#include "config.h"
#include "setup.h"
#include "moveTracks.h"
#include "sendCmd.h"
#include "sbusRx.h"
#include "starter.h"
#include "prm01.h"
#include "controlServo.h"
#include "temperatureDS18B20.h"
#include "webTerminal.h"
#include "cuttingHeight.h"

#if defined(VARIANT_ESP32_S2)
#include <BlynkSimpleEsp32.h>                      // [Blynk]
#endif

#if defined(VARIANT_ESP32_LOLIN)
#include <ArduinoJson.h>                           // https://github.com/bblanchon/ArduinoJson
#include <DNSServer.h>
#include <ESPmDNS.h>

#include <WebServer.h>
#include <ArduinoLogger.h>                         // [Serial / Terminal]

#include <WiFiUdp.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>                           // [Ota HTTP]
#endif

#if defined(VARIANT_ESP8266)
#include <LittleFS.h>                              // [Panięć FLASH] 
#include <EEPROM.h>                                // [Panięć EEPROM] 
//#include <SPIFFS.h>
#include <Arduino.h>                               // [Arduino]
#include <ArduinoJson.h>                           // https://github.com/bblanchon/ArduinoJson

#include <ArduinoOTA.h>                            // [Ota Wifi Local]
#include <BlynkSimpleEsp8266.h>                    // [Blynk]
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>                           // [Wifi Manager]
#include <ArduinoLogger.h>                         // [Serial / Terminal]

#include <WiFiUdp.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>                     // [Ota Połączenie Do Serwera]
#include <ESP8266httpUpdate.h>                     // [Ota HTTP]
#endif

//------------------------------------------------------------------------
// objects
//------------------------------------------------------------------------ 
#if defined(VARIANT_ESP32_S2) || defined(VARIANT_ESP32_LOLIN) || defined(VARIANT_ESP8266)
/*
WidgetTerminal Terminal(V0);
BlynkTimer timer;   // Nazwa Timera Blynk.
//WidgetBridge BlynkBridge (blynk_bridge_pin);       // [Most]
WidgetRTC rtc;
WidgetLED led1(V10);
WidgetLED led2(V11);
WidgetLED led3(V12);
WidgetLED led4(V13);
WidgetLED led5(V14);*/
#endif


#if defined(VARIANT_ESP32_LOLIN) || defined(VARIANT_ESP8266)
WiFiClient espClient;
HTTPClient httpRead;                               // WebHook.
HTTPClient httpWrite;                              // WebHook.
HTTPClient http;                                   // WebHook.
WiFiClient UpdateEspClient;
HTTPClient httpWebHook;                            // WebHook.
#endif

#if defined(VARIANT_ESP8266)
ESP8266HTTPUpdate httpUpdate;
#endif

//------------------------------------------------------------------------
// external variables
//------------------------------------------------------------------------ 
extern bool serial1Blynk;
extern bool serial2Blynk;
// extern int BLYNK_PID_KI;
// extern int BLYNK_PID_KD;

extern int axisValueX;
extern int axisValueY;

extern uint8_t test;
extern uint8_t enable_off;

//extern uint16_t testBatery;

extern uint8_t movement;
extern uint16_t speed_Blynk;
extern uint16_t steer_Blynk;
extern bool enable_Blynk;
extern byte controlMode_Blynk;

//------------------------------------------------------------------------
// variables
//------------------------------------------------------------------------ 
String esp_name = "CONFIG-" + String(Name_ESP);

bool SetStaticIP = false;                          // Ustaw Statyczny Adres IP
bool shouldSaveConfig = false;
bool TimeActivation = false;
unsigned long DifferenceTimer_1;
unsigned long DifferenceTimer_1_last = 0;
unsigned long Timer_1;

char ServerClock[20] = "";

uint8_t Signal;
uint8_t LastSignal;

bool checkSpeedButton = false;
uint32_t SpeedTest;

uint16_t RealYear;
uint8_t RealMonth;
uint8_t RealDay;
uint8_t RealHour;
uint8_t RealMinute;
uint8_t RealSecond;
String DayOfWeak;
uint8_t DayOfWeakNumber;

// uint16_t OutputLewa;
// uint16_t OutputPrawa;

// float rpmMower;

//------------------------------------------------------------------------
// procedures serial setup
//------------------------------------------------------------------------ 
void SerialSetup() {
  //Serial.begin(SpeedRate);
  //Serial.println("Setup Serial Begin.");
  delay(100);
  logger.add(Serial, LOG_LEVEL_INFO, true); // This will log everything on Serial
 // logger.add(Terminal, LOG_LEVEL_VERBOSE, true); // This will log everything on Serial
  logger.add(webTerminal, LOG_LEVEL_VERBOSE, true); // This will log everything on Serial

  //logger.disableLevelName(Terminal);
}

//------------------------------------------------------------------------
// procedures gpio initialize
//------------------------------------------------------------------------ 
void GpioInit() {
#if defined(VARIANT_ESP32_S2)
    analogReadResolution(ANALOG_IN_RES);
    
    ledcSetup(0, PWM_FREQ, PWM_RES);
    ledcAttachPin(OUTPUT_PIN, 0);
#endif

    // pinMode(SENSOR_PIN, INPUT);
    pinMode(safetyStopPin, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
}

//------------------------------------------------------------------------
// procedures blynk setup
//------------------------------------------------------------------------ 

//------------------------------------------------------------------------
// procedures blynk timeout restart
//------------------------------------------------------------------------ 


//------------------------------------------------------------------------
// procedures blynk logo
//------------------------------------------------------------------------ 
void BlynkLogo() {
    // inf << np <<"    ___  __          __" << endl;
    // inf << np <<"   / _ )/ /_ _____  / /__" << endl;
    // inf << np <<"  / _  / / // / _ \\/  '_/" << endl;
    // inf << np <<" /____/_/\\_, /_//_/_/\\_\\" << endl;
    // inf << np <<"        /___/ ver. " << BLYNK_VERSION <<  endl;
    // inf << np <<" on " << BLYNK_INFO_DEVICE << " Instaled." << endl;
    // inf << np << endl;


    inf << np <<"     __                           __  ___                          " << endl;
    inf << np <<"    / /  ____ __      ______     /  |/  /___ _      _____  _____   " << endl;
    inf << np <<"   / /  / __ `/ | /| / / __ \\   / /|_/ / __ \\ | /| / / _ \\/ ___/" << endl;
    inf << np <<"  / /__/ /_/ /| |/ |/ / / / /  / /  / / /_/ / |/ |/ /  __/ /       " << endl;
    inf << np <<" /_____|__,_/ |__/|__/_/ /_/  /_/  /_/\\____/|__/|__/\\___/_/      " << endl;
    inf << np << endl;
}

//------------------------------------------------------------------------
// procedures blynk timer set
//------------------------------------------------------------------------ 


//------------------------------------------------------------------------
// procedures blynk widget show
//------------------------------------------------------------------------ 


//------------------------------------------------------------------------
// procedures blynk terminal command
//------------------------------------------------------------------------ 


//------------------------------------------------------------------------
// procedures blynk declaration virtual pin
//------------------------------------------------------------------------ 

//------------------------------------------------------------------------
// procedures wifi signal check
//------------------------------------------------------------------------ 
void WifiSignal() {
  Signal = (100 - abs(WiFi.RSSI()));
  if (Signal != LastSignal) {
    // Blynk.virtualWrite(V101, Signal);
    LastSignal = Signal;
  } 
}

//------------------------------------------------------------------------
// procedures cycle esp check
//------------------------------------------------------------------------ 
void CheckCycleESP() {
  if (checkSpeedButton == true) {
    //BlynkTerminal(SpeedTest);
    inf << np << terminal_name_device << "Cykli na 1s: " << SpeedTest << endl;
    SpeedTest = 0;
  }
}

//------------------------------------------------------------------------
// procedures save config wifimanager
//------------------------------------------------------------------------ 


//------------------------------------------------------------------------
// procedures flash check only esp8266
//------------------------------------------------------------------------ 
#if defined(VARIANT_ESP8266)
void flashCheck() {
  uint32_t realSize = ESP.getFlashChipRealSize();
  uint32_t ideSize = ESP.getFlashChipSize();
  FlashMode_t ideMode = ESP.getFlashChipMode();

  inf << "[FLASH] Cheking.." << endl;
  inf << np << "real_id = " << ESP.getFlashChipId() << endl;
  inf << np << "real_size = " << realSize << " bytes" << endl;

  inf << np << "ide_size = " << ideSize << " bytes" << endl;
  inf << np << "ide_speed = " << ESP.getFlashChipSpeed() << " Hz" << endl;
  inf << np << "ide_mode = " <<
                (ideMode == FM_QIO
                     ? "QIO"
                     : ideMode == FM_QOUT
                           ? "QOUT"
                           : ideMode == FM_DIO
                                 ? "DIO"
                                 : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN") << endl;

  if (ideSize != realSize) {
    err << "[FLASH] Chip configuration wrong!" << endl;
  } else {
    inf << "[FLASH] Chip configuration ok." << endl;
  }
}
#endif

//------------------------------------------------------------------------
// procedures flash format
//------------------------------------------------------------------------ 


//------------------------------------------------------------------------
// procedures flash setup
//------------------------------------------------------------------------ 


//------------------------------------------------------------------------
// procedures wifi manager reset
//------------------------------------------------------------------------ 


//------------------------------------------------------------------------
// procedures wifi manager setup
//------------------------------------------------------------------------ 


//------------------------------------------------------------------------
// procedures http update from server
//------------------------------------------------------------------------ 


//------------------------------------------------------------------------
// procedures ota setup
//------------------------------------------------------------------------ 


//------------------------------------------------------------------------
// procedures rtc from server blynk
//------------------------------------------------------------------------ 


//------------------------------------------------------------------------
// procedures setup
//------------------------------------------------------------------------ 
void setupPlatform() {
    SerialSetup();
    GpioInit();
    // BlynkSetup();
    BlynkLogo();
    #if defined(VARIANT_ESP8266) || defined(VARIANT_ESP32_LOLIN)
    // ManagerSetup();
    // FsSetup();
    // OtaSetup();
    #endif
}

//------------------------------------------------------------------------
// procedures loop
//------------------------------------------------------------------------ 
void loopPlatform() {
    // BlynkLoop();
    SpeedTest = (SpeedTest+1);
    #if defined(VARIANT_ESP8266) || defined(VARIANT_ESP32_LOLIN)
    // ArduinoOTA.handle();
    #endif
    // blinkLedWidget();
    // BlynkTimeOutRestart();
    //Terminal.flush();
}
//------------------------------------------------------------------------
// end file
//------------------------------------------------------------------------ 