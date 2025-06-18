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

#include <stdint.h>

//------------------------------------------------------------------------
// variables const
//------------------------------------------------------------------------ 

//------------------------------------------------------------------------
// objects
//------------------------------------------------------------------------ 
TrackSpeeds speeds;

//------------------------------------------------------------------------
// variables           
//------------------------------------------------------------------------ 
static int16_t INPUT_MAX = 1000;                                          // [-] Input target maximum limitation.
static int16_t INPUT_MIN = -1000;                                         // [-] Input target minimum limitation.

uint16_t OutputLewa;     
uint16_t OutputPrawa;

bool enable_1 = 0;
bool enable_2 = 0;

uint32_t axisValueY;
uint32_t axisValueX;
//uint16_t speed_Blynk;
bool enable_Blynk;                                                       // [-] Variable from Blynk app.

uint8_t test;                                                            // [-] Variable from Blynk app.
uint8_t enable_off;                                                      // [-] Variable from Blynk app.
uint8_t movement;                                                        // [-] Variable from Blynk app.

//------------------------------------------------------------------------
// procedures move tracks
//------------------------------------------------------------------------ 
void moveTracks(int leftStick, int rightStick) {
    int speed = map(leftStick, -1000, 1000, -1000, 1000);
    int rotation = map(rightStick, -1000, 1000, -1000, 1000);

    int leftSpeed;
    int rightSpeed;

    if (speed >= 0) {
    leftSpeed = speed + rotation;
    rightSpeed = speed - rotation;
    } else {
    leftSpeed = speed - rotation;
    rightSpeed = speed + rotation;
    }

    // Normalizacja wartości leftSpeed i rightSpeed
    if (leftSpeed > 1000) {
        leftSpeed = 1000;
    } else if (leftSpeed < -1000) {
        leftSpeed = -1000;
    }
    
    if (rightSpeed > 1000) {
        rightSpeed = 1000;
    } else if (rightSpeed < -1000) {
        rightSpeed = -1000;
    }
    
    enable_1 = 1; // enable_Blynk;      // Prawe
    enable_2 = 1; // enable_Blynk;      // Lewe
    
    speeds.leftSpeed = leftSpeed;
    speeds.rightSpeed = rightSpeed;
}

//------------------------------------------------------------------------
// procedures loop tracks
//------------------------------------------------------------------------ 
void loopTracks() {
        moveTracks(leftStickY, leftStickX);                               // [-] RC remote control.
}

//------------------------------------------------------------------------
// end file
//------------------------------------------------------------------------ 