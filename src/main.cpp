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

// #include "voltageRegulator.h"

extern unsigned long timeNow; // wysylanie do innych plikow

//------------------------------------------------------------------------
// main setup
//------------------------------------------------------------------------ 
void setup () {
  setupSerialSbusRx();
  setupPlatform();  // installation of necessary things
  setupWebTerminal();
  setupSendCmd();
  setupStarter();
  setupPrm01();
  setupControlServo();
  setupTemperatureDs18b20();
  // setupVoltageRegulator();
  setupCuttingHeight();
  
}

//------------------------------------------------------------------------
// main loop
//------------------------------------------------------------------------ 
void loop () {
  loopPlatform();   // installation of necessary things
  loopWebTerminal();
  loopReadSbusRx();
  loopTracks();
  loopSendCmd();
  loopStarter();
  loopPrm01();
  loopControlServo();
  loopTemperatureDs18b20();
  // loopVoltageRegulator();
  loopCuttingHeight();

 

  

  //---Blink Led---//
  timeNow = millis();
  digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
  // digitalWrite(25, (timeNow%2000)<1000);
  //---Blink Led---//
 

}

//------------------------------------------------------------------------
// end files
//------------------------------------------------------------------------ 