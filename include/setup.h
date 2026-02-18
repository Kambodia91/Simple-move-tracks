#ifndef SETUP_H
#define SETUP_H

#include <Arduino.h>
#include <ArduinoLogger.h>                         // [Serial / Terminal]

void SerialSetup();
void GpioInit();
void MowerLogo();
void WifiSignal();
void CheckCycleESP();
void setupPlatform();
void loopPlatform();

#endif