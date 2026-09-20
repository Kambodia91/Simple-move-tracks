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
#include "controlClutchBlade.h"

// #include "voltageRegulator.h"

unsigned long timeNow = 0; // wysylanie do innych plikow

//------------------------------------------------------------------------
// main setup
//------------------------------------------------------------------------ 
void setup () {
  setupSerialSbusRx();
  setupWebTerminal();
  setupPlatform();  // installation of necessary things
  setupSendCmd();
  setupStarter();
  setupPrm01();
  setupControlServo();
  setupTemperatureDs18b20();
  // setupVoltageRegulator();
  setupCuttingHeight();
  setupControlClutchBlade();
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
  loopTemperatureDs18b20();
  loopControlServo();
  // loopVoltageRegulator();
  loopCuttingHeight();
  loopControlClutchBlade();

 

  

  //---Blink Led---//
  timeNow = millis();
  digitalWrite(LED_BUILTIN, (timeNow%500)<250);
  // digitalWrite(25, (timeNow%2000)<1000);
  //---Blink Led---//
 

}

//------------------------------------------------------------------------
// end files
//------------------------------------------------------------------------ 