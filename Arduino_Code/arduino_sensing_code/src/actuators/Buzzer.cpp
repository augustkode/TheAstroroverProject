#include "Buzzer.h"

Buzzer::Buzzer(int pin) : m_pin(pin) {}

void Buzzer::m_begin()
{
    pinMode(m_pin, OUTPUT);
}

void Buzzer::m_on(int hz)
{
    tone(m_pin, hz);
}

void Buzzer::m_off()
{
    noTone(m_pin);
}