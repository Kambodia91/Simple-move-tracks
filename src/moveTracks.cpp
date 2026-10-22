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

//------------------------------------------------------------------------
// procedures move tracks
//------------------------------------------------------------------------ 
void moveTracks(int stick_Y, int stick_X) {
    int speed = map(stick_Y, -1000, 1000, -1000, 1000);
    int rotation = map(stick_X, -1000, 1000, -1000, 1000);

    int leftSpeed;
    int rightSpeed;

    if (speed > 0) {
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
    
    speeds.masterSpeed = leftSpeed;
    speeds.slaveSpeed = rightSpeed;
}

//------------------------------------------------------------------------
// procedures loop tracks
//------------------------------------------------------------------------ 
void loopTracks() {
        moveTracks(leftStickY, rightStickX);                               // [-] RC remote control.
}

//------------------------------------------------------------------------
// end file
//------------------------------------------------------------------------ 
