#ifndef PIRSENSOR_H
#define PIRSENSOR_H
#include <Arduino.h>

class PIRSensor
{
private:
    int m_pin;

public:
    PIRSensor(int pin);
    void m_begin();
    bool m_motionDetect();
};
#endif