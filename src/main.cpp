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
// #include "DS18B20.h"

// #define ONE_WIRE_BUS              32

// OneWire oneWire(ONE_WIRE_BUS);
// DS18B20 sensor(&oneWire);

int roznicaCzasu;
int zapamietanyCzas;
//------------------------------------------------------------------------
// main setup
//------------------------------------------------------------------------ 
void setup () {
  setupSerialSbusRx();
  setupPlatform();  // installation of necessary things
  setupSendCmd();
  setupStarter();
  setupPrm01();
  // sensor.begin();
  // sensor.setResolution(10);
}

//------------------------------------------------------------------------
// main loop
//------------------------------------------------------------------------ 
void loop () {
  loopPlatform();   // installation of necessary things
  loopReadSbusRx();
  loopTracks();
  loopSendCmd();
  loopStarter();
  loopPrm01();

 

  

  //---Blink Led---//
  unsigned long timeNow = millis();
  digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
  // digitalWrite(25, (timeNow%2000)<1000);
  //---Blink Led---//
 roznicaCzasu = timeNow - zapamietanyCzas;
   
  //     if (roznicaCzasu >= 1000UL) {
  //         sensor.requestTemperatures();
  // Serial.print("Temp: ");
  // Serial.println(sensor.getTempC());
  // zapamietanyCzas = timeNow;}

}

//------------------------------------------------------------------------
// end files
//------------------------------------------------------------------------ 