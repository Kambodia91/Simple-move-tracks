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



//------------------------------------------------------------------------
// main setup
//------------------------------------------------------------------------ 
void setup () {
  setupSerialSbusRx();
  setupPlatform();  // installation of necessary things
  setupSendCmd();
  
}

//------------------------------------------------------------------------
// main loop
//------------------------------------------------------------------------ 
void loop () {
  loopPlatform();   // installation of necessary things
  loopReadSbusRx();
  loopTracks();
  loopSendCmd();
  

  //---Blink Led---//
  unsigned long timeNow = millis();
  digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
  //---Blink Led---//

}

//------------------------------------------------------------------------
// end files
//------------------------------------------------------------------------ 