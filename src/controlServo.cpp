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

//------------------------------------------------------------------------
// define
//------------------------------------------------------------------------ 
#define TEMP_SSANIE_ON     30.0  // Poniżej tej temperatury włącz ssanie
#define TEMP_SSANIE_OFF    60.0  // Powyżej tej temperatury wyłącz ssanie (bieg szybki)

//------------------------------------------------------------------------
// variables const
//------------------------------------------------------------------------ 
const int servoPin = 18;        // dowolny pin obsługujący PWM (np. 18)
const int pwmChannel = 0;       // kanał PWM (od 0 do 15)
const int pwmFreq = 50;         // 50 Hz dla serwa
const int pwmResolution = 16;   // rozdzielczość 16-bitowa (0–65535)

// Minimalna i maksymalna szerokość impulsu w mikrosekundach
const int minUs = 500;
const int maxUs = 2400;

// kąty serwa (dopasuj do swojej dźwigni)
const int SLOW = 0;
const int FAST = 90;
const int CHOKE = 180;

int angle;
bool lowSpeedEngine; // [8] Channel RC 0 = 
//------------------------------------------------------------------------
// objects
//------------------------------------------------------------------------ 

//------------------------------------------------------------------------
// variables           
//------------------------------------------------------------------------ 
// Konfiguracja pinu i kanału PWM
// int angle;                                                             // zmienna z pilota RC kanał 11
void chokeLeverControl() {
  if (!lowSpeedEngine) {
    angle = SLOW;
  } else {
    if (oilTemperature < TEMP_SSANIE_ON) {
      angle = CHOKE;
    } else if (oilTemperature >= TEMP_SSANIE_OFF) {
      angle = FAST;
    } else {
      // Płynne przejście ssania od CHOKE do FAST
      float ratio = (oilTemperature - TEMP_SSANIE_ON) / (TEMP_SSANIE_OFF - TEMP_SSANIE_ON);
      angle = map(ratio * 100, 0, 100, CHOKE, FAST);
    }
  }
}

//------------------------------------------------------------------------
// procedures angel To Duty
//------------------------------------------------------------------------ 
// Funkcja konwertująca kąt (0–180°) na wartość PWM
uint32_t angleToDuty(int angle) {
  angle = constrain(angle, 0, 180);
  int pulseWidthUs = map(angle, 0, 180, minUs, maxUs);
  // Oblicz duty w zakresie 0–65535 dla zadanej szerokości impulsu
  return (uint32_t)((pulseWidthUs / 20000.0) * 65535);
}

//------------------------------------------------------------------------
// procedures setup Control Servo
//------------------------------------------------------------------------ 
void setupControlServo() {
  // Konfiguracja sprzętowego PWM (LEDC)
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(servoPin, pwmChannel);
}

//------------------------------------------------------------------------
// procedures loop Control Servo
//------------------------------------------------------------------------ 
void loopControlServo() {
  chokeLeverControl();
  ledcWrite(pwmChannel, angleToDuty(angle));
}

//------------------------------------------------------------------------
// end file
//------------------------------------------------------------------------ 