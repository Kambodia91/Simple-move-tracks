//------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------ 
#include "defines.h"
#include "config.h"
#include "setup.h"
#include "moveTracks.h"
#include "sendCmd.h"
#include "sbusRx.h"
#include "starter.h"

#include <stdint.h>
#include <ArduinoLogger.h>                         // [Serial / Terminal]

static int16_t INPUT_MAX = 1000;             // [-] Input target maximum limitation
static int16_t INPUT_MIN = -1000;             // [-] Input target minimum limitation

//------------------------------------------------------------------------
// objects
//------------------------------------------------------------------------ 
extern TrackSpeeds speeds;

//------------------------------------------------------------------------
// extern variables     // Wysyłane do innych plików
//------------------------------------------------------------------------ 
extern uint16_t OutputLewa;     
extern uint16_t OutputPrawa;

extern bool enable_1;
extern bool enable_2;
//------------------------------------------------------------------------
// variables            // Odbierane od innych plikow
//------------------------------------------------------------------------ 

int16_t leftStickX;    //
int16_t leftStickY;    //

uint32_t axisValueY;
uint32_t axisValueX;
//uint16_t speed_Blynk;
bool enable_Blynk;

uint8_t test;
uint8_t enable_off;
uint8_t movement;

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
    
    enable_1 = enable_Blynk;      // Prawe
    enable_2 = enable_Blynk;      // Lewe
    
    speeds.leftSpeed = leftSpeed;
    speeds.rightSpeed = rightSpeed;
}

//------------------------------------------------------------------------
// procedures loop tracks
//------------------------------------------------------------------------ 
void loopTracks() {
        moveTracks(leftStickY, leftStickX);
        // moveTracks(axisValueY, axisValueX);
}

//------------------------------------------------------------------------
// end file
//------------------------------------------------------------------------ 