
#include "defines.h"
#include "config.h"
#include "setup.h"
#include "moveTracks.h"
#include "sendCmd.h"
#include "sbusRx.h"

#include <stdint.h>
#include <ArduinoLogger.h>                         // [Serial / Terminal]
#include <sbus.h>

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial, 3, 1, false);

/* SBUS data */
bfs::SbusData data;

extern int16_t leftStickX;    //
extern int16_t leftStickY;    //

// extern int16_t RightStickX;   //
// extern int16_t RightStickY;   //

uint16_t speed_Blynk;
uint16_t steer_Blynk;


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
    // /* Display lost frames and failsafe data */
    // Serial.print(data.lost_frame);
    // Serial.print(" ");
    // Serial.println(data.failsafe);
  }

if (data.failsafe == 1 || data.lost_frame == 1) {
    
    // Serial.println("Brak nadajnika");
    data.ch[0] = 1000;
    data.ch[1] = 1000;
    leftStickY = 0; 
    leftStickX = 0;
  } else {

//---------------------------------------------------------Kanał 0
  leftStickX = scaleValue(data.ch[0], 314, 1693, -steer_Blynk, steer_Blynk, 985, 1015);
//---------------------------------------------------------Kanał 1
  leftStickY = scaleValue(data.ch[1], 306, 1693, -speed_Blynk, speed_Blynk, 985, 1015);
//---------------------------------------------------------Kanał 2
  int StickX = scaleValue(data.ch[2], 306, 1693, -1000, 1000, 985, 1015);
//---------------------------------------------------------Kanał 3
  int StickY = scaleValue(data.ch[3], 306, 1693, -1000, 1000, 985, 1015);
//---------------------------------------------------------Kanał 4
  int threePositionSwitchC = (data.ch[4] < 500) ? 0 : (data.ch[5] < 1500) ? 1 : 2;
//---------------------------------------------------------Kanał 5
  int potentiometerValueA = scaleValue(data.ch[5], 0, 2000, 0, 1023);
//---------------------------------------------------------Kanał 6
  int potentiometerValueC = scaleValue(data.ch[6], 0, 2000, 0, 1023);
//---------------------------------------------------------Kanał 7
  int potentiometerValueB = scaleValue(data.ch[7], 0, 2000, 0, 1023);
//---------------------------------------------------------Kanał 8
  int buttonB = (data.ch[8] > 1000) ? 1 : 0;
//---------------------------------------------------------Kanał 9
  int buttonH = (data.ch[9] > 1000) ? 1 : 0;
//---------------------------------------------------------Kanał 10
  int buttonD = (data.ch[10] > 1000) ? 1 : 0;
//---------------------------------------------------------Kanał 11
  int potentiometerValueRight = scaleValue(data.ch[11], 0, 2000, 0, 1023);



// Serial.print(data.ch[0]);
// Serial.print(" ");
// Serial.println(data.ch[1]);

}

}
