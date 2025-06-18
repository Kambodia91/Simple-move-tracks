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

#include <stdint.h>
//------------------------------------------------------------------------
// variables const
//------------------------------------------------------------------------ 
const int numMagnets = 1; // Używana liczba magnesów
const float rpmThreshold = 2300.0; // obroty po przekroczeniu uruchamia ładowanie

//------------------------------------------------------------------------
// objects
//------------------------------------------------------------------------ 
Tachometer tachometer(HALL_PIN, CHARGING_PIN, rpmThreshold, numMagnets);
Tachometer* Tachometer::_instance = nullptr;

//------------------------------------------------------------------------
// variables           
//------------------------------------------------------------------------ 

float rpmMower;

static const int numReadings = 20; // Liczba odczytów do uśredniania
float readings[numReadings]; // Tablica do przechowywania odczytów
int readIndex = 0; // Indeks bieżącego odczytu
float total = 0; // Suma odczytów
float average = 0; // Uśredniona wartość RPM

//------------------------------------------------------------------------
// method Tachometer
//------------------------------------------------------------------------ 
Tachometer::Tachometer(int hallPin, int ledPin, float rpmThreshold, int numMagnets)
    : _hallPin(hallPin), _ledPin(ledPin), _rpmThreshold(rpmThreshold),
      _lastMicros(0), _intervalMicros(0), _newData(false), _rpm(0.0), _numMagnets(numMagnets) {
    _instance = this;

    // Inicjalizacja tablicy odczytów
    for (int i = 0; i < numReadings; i++) {
        readings[i] = 0;
    }
}

//------------------------------------------------------------------------
// method begin
//------------------------------------------------------------------------ 
void Tachometer::begin() {
    pinMode(_hallPin, INPUT_PULLUP);
    pinMode(_ledPin, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(_hallPin), handleInterrupt, FALLING);
}

//------------------------------------------------------------------------
// method update
//------------------------------------------------------------------------ 
void Tachometer::update() {
    unsigned long currentMicros = micros();
    if (_newData) {
        noInterrupts();
        unsigned long interval = _intervalMicros;
        _newData = false;
        interrupts();

        calculateRPM(interval);
    } else if (currentMicros - _lastMicros > 1000000) { // Jeśli nie było nowych impulsów przez 1 sekundę
        _rpm = 0.0;
        digitalWrite(_ledPin, LOW); // Upewnij się, że dioda LED jest zgaszona
    }

        if (_rpm > _rpmThreshold) {
            digitalWrite(_ledPin, HIGH);
        } else {
            digitalWrite(_ledPin, LOW);
        }
}

//------------------------------------------------------------------------
// method getRPM
//------------------------------------------------------------------------ 
float Tachometer::getRPM() const {
    return _rpm;
}

//------------------------------------------------------------------------
// method handleInterrupt
//------------------------------------------------------------------------ 
void IRAM_ATTR Tachometer::handleInterrupt() {
    unsigned long currentMicros = micros();
    unsigned long debounceTime = 5000; // 5 ms (5000 µs) odszumianie

    // Sprawdzenie, czy od ostatniego przerwania minęło wystarczająco dużo czasu
    if (currentMicros - _instance->_lastMicros > debounceTime) {
        _instance->_intervalMicros = currentMicros - _instance->_lastMicros;
        _instance->_lastMicros = currentMicros;
        _instance->_newData = true;
    }
}

//------------------------------------------------------------------------
// method calculateRPM
//------------------------------------------------------------------------ 
void Tachometer::calculateRPM(unsigned long interval) {
    if (interval > 0) {
        float frequency = 1000000.0 / interval; // Hz (częstotliwość impulsów)
        float rpm = (frequency * 60.0) / _numMagnets; // RPM (obroty na minutę)

        // Usuwanie starej wartości z sumy
        total = total - readings[readIndex];

        // Dodanie nowej wartości do tablicy i do sumy
        readings[readIndex] = rpm;
        total = total + readings[readIndex];

        // Przesunięcie indeksu w tablicy
        readIndex = (readIndex + 1) % numReadings;

        // Obliczenie średniej wartości
        average = total / numReadings;

        // Ustawienie aktualnego RPM na uśrednioną wartość
        _rpm = average;
    }
}

//------------------------------------------------------------------------
// procedures setup Starter
//------------------------------------------------------------------------ 
void setupStarter(){
    tachometer.begin();
    pinMode(safetyStopPin, OUTPUT);
  }

//------------------------------------------------------------------------
// procedures loop Starter
//------------------------------------------------------------------------ 
void loopStarter(){
    tachometer.update();                                                 // RPM Reader.
    rpmMower = tachometer.getRPM();
  }

//------------------------------------------------------------------------
// end file
//------------------------------------------------------------------------ 