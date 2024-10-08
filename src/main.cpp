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



//------------------------------------------------------------------------
// main setup
//------------------------------------------------------------------------ 
void setup () {
  setupSerialSbusRx();
  setupPlatform();  // installation of necessary things
  setupSendCmd();
  setupStarter();
  
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
  

  //---Blink Led---//
  unsigned long timeNow = millis();
  digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
  // digitalWrite(25, (timeNow%2000)<1000);
  //---Blink Led---//

}

//------------------------------------------------------------------------
// end files
//------------------------------------------------------------------------ 