#ifndef STARTER_H
#define STARTER_H


//------------------------------------------------------------------------
// include
//------------------------------------------------------------------------ 
#include <stdint.h>
#include <Arduino.h>

//------------------------------------------------------------------------
// external variables 
//------------------------------------------------------------------------ 
extern float rpmMower;

//------------------------------------------------------------------------
// struct
//------------------------------------------------------------------------ 
class Tachometer {
public:
    Tachometer(int hallPin, int ledPin, float rpmThreshold, int numMagnets);
    void begin();
    void update();
    float getRPM() const;
    
private:
    static void IRAM_ATTR handleInterrupt();
    void calculateRPM(unsigned long interval);

    int _hallPin;
    int _ledPin;
    float _rpmThreshold;
    int _numMagnets; // Liczba magnesów na wale

    volatile unsigned long _lastMicros;
    volatile unsigned long _intervalMicros;
    volatile bool _newData;

    float _rpm;

    static Tachometer* _instance; // singleton instance for ISR
};

//------------------------------------------------------------------------
// external objects
//------------------------------------------------------------------------ 

//------------------------------------------------------------------------
// procedures
//------------------------------------------------------------------------ 
void setupStarter();
void loopStarter();

#endif
//------------------------------------------------------------------------
// endfile
//------------------------------------------------------------------------ 
