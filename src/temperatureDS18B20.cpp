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
float oilTemperature;                                                     // wysylanie zmiennej di innych plikow

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
    static unsigned long lastTempRead = 0;
    unsigned long now = millis();

    if (now - lastTempRead >= 250UL) {
        sensor.requestTemperatures();
        oilTemperature = sensor.getTempC();
        // inf << "Temp: " << oilTemperature << " kąd serwa: " << angle << endl;
        lastTempRead = now;
    }
}