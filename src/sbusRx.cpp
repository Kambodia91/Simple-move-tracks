
#include "defines.h"
#include "config.h"
#include "setup.h"
#include "moveTracks.h"
#include "sendCmd.h"
#include "sbusRx.h"
#include "starter.h"

#include <stdint.h>
#include <ArduinoLogger.h>                         // [Serial / Terminal]
#include <sbus.h>

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial, 3, 1, false);

/* SBUS data */
bfs::SbusData data;

uint16_t  timeoutCntSbusRx      = 0;               // Timeout counter for Rx Serial command
uint8_t   timeoutFlgSbusRx      = 0;               // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)
bool      timeoutMsgSbusRx      = 0;
bool      timeoutMsgfailsafe    = 0;
bool      timeoutMsglost_frame  = 0;

extern int16_t leftStickX;    //
extern int16_t leftStickY;    //
bool starter;
bool safetyStop;                                   // Sygnał z pilota RC
float rpm;                                         // Obroty z czujnika halla [starter.cpp]
// extern int16_t RightStickX;   //
// extern int16_t RightStickY;   //

uint16_t speed_Blynk;
uint16_t steer_Blynk;

unsigned long startTime;
unsigned long endTime;
unsigned long elapsedTime;



int scaleValue(int channelValue, int inputMin, int inputMax, int outputMin, int outputMax, int neutralMin = -1, int neutralMax = -1) {
  if (neutralMin != -1 && neutralMax != -1) {
    if (channelValue >= neutralMin && channelValue <= neutralMax) {
      return (outputMin + outputMax) / 2;
    } else if (channelValue < neutralMin) {
      return map(channelValue, inputMin, neutralMin, outputMin, (outputMin + outputMax) / 2);
    } else { // channelValue > neutralMax
      return map(channelValue, neutralMax, inputMax, (outputMin + outputMax) / 2, outputMax);
    }
  } else {
    return map(channelValue, inputMin, inputMax, outputMin, outputMax);
  }
}

void setupSerialSbusRx() {
  /* Begin the SBUS communication */
  sbus_rx.Begin();
  data.failsafe = 1;
  data.lost_frame = 1;

}

void loopReadSbusRx() {  
  if (sbus_rx.Read()) {
    /* Grab the received data */
    data = sbus_rx.data();
    /* Display the received data */
    // for (int8_t i = 0; i < 12; i++) {
    //   Serial.print(data.ch[i]);
    //   Serial.print(" ");
    // }
    // // /* Display lost frames and failsafe data */
    // Serial.print(data.lost_frame);
    // Serial.print(" ");
    // Serial.println(data.failsafe);
    timeoutFlgSbusRx = 0; 
    timeoutCntSbusRx = 0;
    
  } else {


    if (timeoutCntSbusRx++ >= SERIAL_TIMEOUT) {       // Timeout qualification
        timeoutFlgSbusRx = 1;                         // Timeout detected
        timeoutCntSbusRx = SERIAL_TIMEOUT;            // Limit timout counter value
      } else {
        // /Serial.println(timeoutCntSbusRx);
      }
  }

//--------------------------------------------// [0]-OK [1]-NOK
    if (timeoutFlgSbusRx == 1) {              // Utrata Połączenie SbusRX do Esp
      if (!timeoutMsgSbusRx) {
      inf << "Sbus no connect." <<  endl;
      timeoutMsgSbusRx = 1;
      }
      data.ch[0] = 1000;                      // Ustawiam 0 na joystiku
      data.ch[1] = 1000;                      // Ustawiam 0 na joystiku
      leftStickY = 0;                         // Ustawiam 0 na joystiku
      leftStickX = 0;                         // Ustawiam 0 na joystiku
      data.failsafe = 1;
    } else {
    timeoutMsgSbusRx = 0;
    }

//--------------------------------------------// [0]-OK [1]-NOK
    if (data.lost_frame == 1) {               // Odbiornik stracił zasięg 
      if (!timeoutMsglost_frame) {
      inf << "Transmiter lost frame." <<  endl;
      timeoutMsglost_frame = 1;
      }
      data.ch[0] = 1000;                      // Ustawiam 0 na joystiku
      data.ch[1] = 1000;                      // Ustawiam 0 na joystiku
      leftStickY = 0;                         // Ustawiam 0 na joystiku
      leftStickX = 0;                         // Ustawiam 0 na joystiku
    } else {
    timeoutMsglost_frame = 0;
    }

//--------------------------------------------// [0]-OK [1]-NOK
    if (data.failsafe == 1) {                 // Odbiornik wykrył wyłączony nadajnik
      if (!timeoutMsgfailsafe) {
      inf << "SAFETY STOP." <<  endl;
      timeoutMsgfailsafe = 1;
      }
      safetyStop = 1;
      data.ch[0] = 1000;
      data.ch[1] = 1000;
      leftStickY = 0; 
      leftStickX = 0;
      digitalWrite(safetyStopPin, LOW);       // Wyłączam zapłom

    } else {
      timeoutMsgfailsafe = 0;
      safetyStop = 0;
      digitalWrite(safetyStopPin, HIGH);      // Włączam zapłom
    }

//----------------------KANAŁY Z ODBIORNIKA---------------------//
//---------------------------------------------------------Kanał 0
  leftStickX = scaleValue(data.ch[0], 306, 1693, -steer_Blynk, steer_Blynk, 985, 1015);

//---------------------------------------------------------Kanał 1
  leftStickY = scaleValue(data.ch[1], 306, 1693, -speed_Blynk, speed_Blynk, 985, 1015);

//---------------------------------------------------------Kanał 2
  int StickX = scaleValue(data.ch[2], 306, 1693, -1000, 1000, 985, 1015);

//---------------------------------------------------------Kanał 3
  int StickY = scaleValue(data.ch[3], 306, 1693, -1000, 1000, 985, 1015);

//---------------------------------------------------------Kanał 4
  int threePositionSwitchC = (data.ch[4] < 500) ? 0 : (data.ch[4] < 1500) ? 1 : 2;

//---------------------------------------------------------Kanał 5
  int potentiometerValueA = scaleValue(data.ch[5], 306, 1694, 0, 1023);

//---------------------------------------------------------Kanał 6
  int potentiometerValueC = scaleValue(data.ch[6], 306, 1694, 0, 1023);

//---------------------------------------------------------Kanał 7
  int potentiometerValueB = scaleValue(data.ch[7], 306, 1694, 0, 1023);

//---------------------------------------------------------Kanał 8
  int buttonB = (data.ch[8] > 1000) ? 1 : 0;

//---------------------------------------------------------Kanał 9
  int buttonH = (data.ch[9] > 1000) ? 1 : 0;

  starter = buttonH;// włącznik rozrusznika
  if ((starter == 0) && (rpm > 200)) {
    digitalWrite(starterPin, LOW);
  } else {
    digitalWrite(starterPin, HIGH);
  }
//---------------------------------------------------------Kanał 10
  int buttonD = (data.ch[10] > 1000) ? 1 : 0;
//---------------------------------------------------------Kanał 11
  int potentiometerValueRight = scaleValue(data.ch[11], 306, 1694, 0, 1023);
  


// Serial.print(leftStickX);
// Serial.print(" ");
// Serial.print(leftStickY);
// Serial.print(" ");
// Serial.print(StickX);
// Serial.print(" ");
// Serial.print(StickY);
// Serial.print(" ");
// Serial.print(threePositionSwitchC);
// Serial.print(" ");
// Serial.print(potentiometerValueA);
// Serial.print(" ");
// Serial.print(potentiometerValueC);
// Serial.print(" ");
// Serial.print(potentiometerValueB);
// Serial.print(" ");
// Serial.print(buttonB);
// Serial.print(" ");
// Serial.print(buttonH);
// Serial.print(" ");
// Serial.print(buttonD);
// Serial.print(" ");
// Serial.println(potentiometerValueRight);



}
