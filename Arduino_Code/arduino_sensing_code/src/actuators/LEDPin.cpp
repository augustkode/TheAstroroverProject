#include "LEDPin.h"

LEDPin::LEDPin(int pin) : m_pin(pin) {}

void LEDPin::m_begin()
{
    pinMode(m_pin, OUTPUT);
}

void LEDPin::m_on()
{
    digitalWrite(m_pin, HIGH);
}

void LEDPin::m_off(){
    digitalWrite(m_pin, LOW);
}

