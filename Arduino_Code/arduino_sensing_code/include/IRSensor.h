#ifndef IRSENSOR.H
#define IRSENSOR.H
#include <Arduino.h>

class IRSensor {
private:
    int m_IRPin;

public:
    IRSensor(int IRPin);
    void IRBegin();
    int IRPinGetReadings();
};


#endif

