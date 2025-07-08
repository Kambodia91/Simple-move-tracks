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
    pinMode(SWITCH_3, OUTPUT);
    pinMode(SWITCH_4, OUTPUT);
    digitalWrite(SWITCH_3, LOW);
    digitalWrite(SWITCH_4, LOW);
}

//------------------------------------------------------------------------
// procedures loop Cutting Height
//------------------------------------------------------------------------ 
void loopCuttingHeight() {
switch (threePositionSwitchC)
{
case 0:
    digitalWrite(SWITCH_3, HIGH);
    digitalWrite(SWITCH_4, LOW);
    break;
case 1:
    digitalWrite(SWITCH_3, LOW);
    digitalWrite(SWITCH_4, LOW);
    break;
case 2:
    digitalWrite(SWITCH_3, LOW);
    digitalWrite(SWITCH_4, HIGH);
    break;
default:
    break;
}

}

//------------------------------------------------------------------------
// end file
//------------------------------------------------------------------------ 