#include "IRSensor.h"

IRSensor::IRSensor(int IRPin) : m_IRPin(IRPin) {}

void IRSensor::IRBegin() {
    pinMode(m_IRPin, INPUT);
}

int IRSensor::IRPinGetReadings() {
    return digitalRead(m_IRPin);
}
