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

#define ONE_WIRE_BUS              5

//------------------------------------------------------------------------
// variables const
//------------------------------------------------------------------------ 

//------------------------------------------------------------------------
// objects
//------------------------------------------------------------------------
OneWire oneWire(ONE_WIRE_BUS);
DS18B20 sensor(&oneWire);

//------------------------------------------------------------------------
// variables 
//------------------------------------------------------------------------
unsigned long timeNow;                                                    // czas z main.cpp
float oilTemperature;                                                     // wysylanie zmiennej di innych plikow
int roznicaCzasu;
int zapamietanyCzas;

//------------------------------------------------------------------------
// procedures setup Temperature Ds18b20
//------------------------------------------------------------------------ 
void setupTemperatureDs18b20() {
    sensor.begin();
    sensor.setResolution(10);
}

//------------------------------------------------------------------------
// procedures loop Temperature Ds18b20
//------------------------------------------------------------------------ 
void loopTemperatureDs18b20() {
    roznicaCzasu = timeNow - zapamietanyCzas;
   
    if (roznicaCzasu >= 250UL) {
        sensor.requestTemperatures();
        oilTemperature = sensor.getTempC();
        //verb << "Temp: " << oilTemperature << " kąd serwa: " << angle << endl;
        zapamietanyCzas = timeNow;
    }
}