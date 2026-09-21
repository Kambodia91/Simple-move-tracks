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
#include <OneWire.h>
#include "DS18B20.h"
#include "webTerminal.h"
#include "cuttingHeight.h"


//------------------------------------------------------------------------
// variables const
//------------------------------------------------------------------------ 

//------------------------------------------------------------------------
// objects
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// variables 
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// procedures setup Cutting Height
//------------------------------------------------------------------------ 
void setupCuttingHeight() {
    pinMode(SWITCH_3_PIN, OUTPUT);
    pinMode(SWITCH_4_PIN, OUTPUT);
    digitalWrite(SWITCH_3_PIN, LOW);
    digitalWrite(SWITCH_4_PIN, LOW);
}

//------------------------------------------------------------------------
// procedures loop Cutting Height
//------------------------------------------------------------------------ 
void loopCuttingHeight() {
switch (threePositionSwitchC)
{
case 0:
    digitalWrite(SWITCH_3_PIN, HIGH);
    digitalWrite(SWITCH_4_PIN, LOW);
    break;
case 1:
    digitalWrite(SWITCH_3_PIN, LOW);
    digitalWrite(SWITCH_4_PIN, LOW);
    break;
case 2:
    digitalWrite(SWITCH_3_PIN, LOW);
    digitalWrite(SWITCH_4_PIN, HIGH);
    break;
default:
    break;
}

}

//------------------------------------------------------------------------
// end file
//------------------------------------------------------------------------ 