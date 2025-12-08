#include "PhotoResistor.h"

PhotoResistor::PhotoResistor(int pin) : m_pin(pin) {}

void PhotoResistor::m_begin()
{
    pinMode(m_pin, INPUT);
}

bool PhotoResistor::isPhotoHigh()
{
    return digitalRead(m_pin) == HIGH;
}