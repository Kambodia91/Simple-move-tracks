
#include "defines.h"
#include "config.h"
#include "setup.h"
#include "moveTracks.h"
#include "sendCmd.h"
#include "sbusRx.h"
#include "starter.h"
#include "prm01.h"

#include <stdint.h>
#include <ArduinoLogger.h>                         // [Serial / Terminal]
#include <sbus.h>

union u32_tag  {
    byte       b[4];
    int32_t    i32;
  } latit, longt;

STREAM_DATA streamData;

byte set = 1;

void setupPrm01() {
  delay(1000);
  Wire.begin(4, 23, 19, 400);
  Wire.onRequest(onRequest);
}

void loopPrm01() {

  double log = -23.123456;
  double lat =  75.123456;
    
  longt.i32 = (int32_t)(log * 1000000);
  latit.i32 = (int32_t)(lat * 1000000);

  streamData.battVoltage = Feedback_Serial1.batVoltage * 10;
}

void set1() {
  int  alt = streamData.altitude * 10;
  byte  altHi = highByte(alt )  ;
  byte  altLo =  lowByte(alt ) ;

  int  yaw = streamData.yaw * 100;
  byte yawHi = highByte(yaw );
  byte yawLo = lowByte(yaw );

  int  speed2 = streamData.gps_speed * 10;
  byte speedHi = highByte(speed2);
  byte speedLo = lowByte(speed2);

  int  roll = streamData.roll * 10;
  byte rollHi = highByte(roll);
  byte rollLo = lowByte(roll);

  int  pitch = streamData.pitch * 10;
  byte pitchHi = highByte(pitch);
  byte pitchLo = lowByte(pitch);

  int distance = 0;//calc_dist(streamData.gps_lat, streamData.gps_lon, streamData.home_lat, streamData.home_lon) * 100;
  byte distanceHi = highByte(distance);
  byte distanceLo = lowByte(distance);

  byte buffer[16] = {
    0x89, 0xAB, 
    streamData.gps_sats, 
    altHi, altLo, 
    yawHi, yawLo, 
    speedHi, speedLo, 
    rollHi , rollLo, 
    pitchHi, pitchLo, 
    distanceHi, distanceLo, 
    0x00
  };

Wire.write(buffer, 16);
set=2;
}

void set2() {
  int  rise = streamData.climb * 10;
  byte  riseHi = highByte(rise);
  byte  riseLo =  lowByte(rise);
  
  byte voltesHi = highByte(streamData.battVoltage);
  byte voltesLo = lowByte(streamData.battVoltage);
  
  byte buffer[16] = {
    0x89, 0xCD, 
    streamData.gps_sats, 
    riseHi, riseLo, 
    voltesHi, voltesLo,
    latit.b[3], latit.b[2], latit.b[1], latit.b[0],
    longt.b[3], longt.b[2], longt.b[1], longt.b[0],
    0x00
  };

Wire.write(buffer, 16);
set=3;
}

void set3(){
  byte buffer[16] = {
    0x89, 0xEF, 
    0x0, 
    0x0, 0x0, 
    0x0, 0x0,
    0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0,
    0x00
  };
    
Wire.write(buffer, 16);
set=1;
}

void onRequest() {
  switch (set)  {
   case 1:
    set1();
    //Serial.println("set1 ok");
    break;
    case 2:
    set2();
    //Serial.println("set2 ok");
    break;
    case 3:
    set3();
    //Serial.println("set3 ok");
    break;
  
  default:
    break;
  }
  
}
