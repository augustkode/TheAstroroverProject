#include "PirSensor.h"

PIRSensor::PIRSensor(int pin) : m_pin(pin) {}

void PIRSensor::m_begin()
{
    pinMode(m_pin, INPUT);
}

bool PIRSensor::m_motionDetect()
{
    return digitalRead(m_pin) == HIGH;
}