#ifndef PRM01_H
#define PRM01_H


//------------------------------------------------------------------------
// include
//------------------------------------------------------------------------ 
#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>
//------------------------------------------------------------------------
// struct
//------------------------------------------------------------------------ 
struct STREAM_DATA {
  uint16_t battVoltage = (uint16_t)42000;
  float altitude = 0;
  float climb = 0;
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
  uint8_t gps_sats = 0;
  float gps_lon = 0;
  float gps_lat = 0;
  float home_lon = 0;
  float home_lat = 0;
  uint16_t gps_speed = 0;
  boolean gps_fix = false;
  uint32_t home_distance = 0;
};

//------------------------------------------------------------------------
// procedures
//------------------------------------------------------------------------ 
void setupPrm01();
void loopPrm01();
void set1();
void set2();
void set3();
void onRequest();

#endif

//------------------------------------------------------------------------
// endfile
//------------------------------------------------------------------------ 