//------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------ 
#include "defines.h"
#include "config.h"
#include "setup.h"
#include "moveTracks.h"
#include "sendCmd.h"

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
    //Serial.println(movement);
    // if (test) {

    //     switch (movement) {
    //         case 1 :                                                    // Do przodu
    //         if (enable_off){
    //             enable_1 = enable_Blynk;                                    // Prawe
    //             enable_2 = enable_Blynk;                                    // Lewe
    //         } else {
    //             enable_1 = enable_Blynk;                                    // Prawe
    //             enable_2 = enable_Blynk;                                    // Lewe
    //         }

    //         // speeds.leftSpeed = speed_Blynk;                             // Prawa
    //         // speeds.rightSpeed = speed_Blynk;                            // Lewa
    //         //inf << "Jazda do przodu." << endl;
    //         break;
        
    //         case 2 :                                                    // Do przodu i w Prawo
    //         if (enable_off){
    //             enable_1 = 0;                                               // Prawe
    //             enable_2 = enable_Blynk;                                    // Lewe
    //         } else {
    //             enable_1 = enable_Blynk;                                    // Prawe
    //             enable_2 = enable_Blynk;                                    // Lewe
    //         }
    //         speeds.leftSpeed = 0;                                       // Prawa
    //         // speeds.rightSpeed = speed_Blynk;                            // Lewa
    //         //Serial.println("Jazda do przodu w lewo");
    //         break;

    //         case 3 :                                                    // Do przodu i w lewo
    //         if (enable_off){
    //             enable_1 = enable_Blynk;                                    // Prawe
    //             enable_2 = 0;                                               // Lewe
    //         } else {
    //             enable_1 = enable_Blynk;                                    // Prawe
    //             enable_2 = enable_Blynk;                                    // Lewe
    //         }
    //         // speeds.leftSpeed = speed_Blynk;                             // Prawa
    //         speeds.rightSpeed = 0;                                      // Lewa
    //         //Serial.println("Jazda do przodu w prawo.");
    //         break;

    //         case 4 :                                                    // Do tyłu
    //         if (enable_off){
    //             enable_1 = enable_Blynk;                                    // Prawe
    //             enable_2 = enable_Blynk;                                    // Lewe
    //         } else {
    //             enable_1 = enable_Blynk;                                    // Prawe
    //             enable_2 = enable_Blynk;                                    // Lewe
    //         }
    //         // speeds.leftSpeed = -speed_Blynk;                            // Prawa
    //         // speeds.rightSpeed = -speed_Blynk;                           // Lewa
    //         //Serial.println("Jazda do tyłu.");
    //         break;

    //         case 5 :                                                    // Do tyłu i w Lewo
    //         if (enable_off){
    //             enable_1 = enable_Blynk;                                    // Prawe
    //             enable_2 = 0;                                               // Lewe
    //         } else {
    //             enable_1 = enable_Blynk;                                    // Prawe
    //             enable_2 = enable_Blynk;                                    // Lewe
    //         }
    //         // speeds.leftSpeed = -speed_Blynk;                            // Prawa
    //         speeds.rightSpeed = 0;                                      // Lewa
    //         //Serial.println("Jazda do tyłu w lewo.");
    //         break;

    //         case 6 :                                                    // Do tyłu i w Prawo
    //         if (enable_off){
    //             enable_1 = 0;                                               // Prawe
    //             enable_2 = enable_Blynk;                                    // Lewe
    //         } else {
    //             enable_1 = enable_Blynk;                                    // Prawe
    //             enable_2 = enable_Blynk;                                    // Lewe
    //         }
    //         speeds.leftSpeed = 0;                                       // Prawa
    //         // speeds.rightSpeed = -speed_Blynk;                           // Lewa
    //         //Serial.println("Jazda do tyłu w prawo.");
    //         break;
            
    //         case 7 :                                                    // Koła zablokowane
    //         if (enable_off){
    //             enable_1 = enable_Blynk;                                    // Prawe
    //             enable_2 = enable_Blynk;                                    // Lewe
    //         } else {
    //             enable_1 = enable_Blynk;                                    // Prawe
    //             enable_2 = enable_Blynk;                                    // Lewe
    //         }
    //         speeds.leftSpeed = 0;                                       // Prawa
    //         speeds.rightSpeed = 0;                                      // Lewa
    //         //Serial.println("Jazda do tyłu w prawo.");
    //         break;
            
    //         default:                                                    // Koła odblokowane
    //         if (enable_off){
    //             enable_1 = 0;                                               // Prawe
    //             enable_2 = 0;                                               // Lewe
    //         } else {
    //             enable_1 = enable_Blynk;                                    // Prawe
    //             enable_2 = enable_Blynk;                                    // Lewe
    //         }
                

    //         speeds.leftSpeed = 0;                                       // Prawa
    //         speeds.rightSpeed = 0;                                      // Lewa
    //         //Serial.println("Stop.");
    //         break;
    //     } 
    // } else {
        moveTracks(leftStickY, leftStickX);
    //}
}

//------------------------------------------------------------------------
// end file
//------------------------------------------------------------------------ 