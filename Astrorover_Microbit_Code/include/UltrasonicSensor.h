#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include <Arduino.h>

class UltrasonicSensor{
private:
    uint8_t m_trigPin;
    uint8_t m_echoPin;

public:
    UltrasonicSensor(uint8_t m_trigPin, uint8_t m_echoPin);
    float measureDistanceUltrasonicCM();
};



#endif