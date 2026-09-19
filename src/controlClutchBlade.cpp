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
#include "controlClutchBlade.h"

//------------------------------------------------------------------------
// variables const
//------------------------------------------------------------------------ 

//------------------------------------------------------------------------
// objects
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// variables 
//------------------------------------------------------------------------
bool activaiteClutch; // [9] Channel RC 

//------------------------------------------------------------------------
// procedures setup Cutting Height
//------------------------------------------------------------------------ 
void setupControlClutchBlade() {
    pinMode(        SWITCH_CLUTCH_PIN,  OUTPUT);
    digitalWrite(   SWITCH_CLUTCH_PIN,  LOW);
}

//------------------------------------------------------------------------
// procedures loop Cutting Height
//------------------------------------------------------------------------ 
void loopControlClutchBlade() {
    if (activaiteClutch) {
        digitalWrite(SWITCH_CLUTCH_PIN, HIGH);      // Włączam sprzęgło
    } else {
        digitalWrite(SWITCH_CLUTCH_PIN, LOW);       // Wyłączam sprzęgło
    }
}

//------------------------------------------------------------------------
// end file
//------------------------------------------------------------------------ 