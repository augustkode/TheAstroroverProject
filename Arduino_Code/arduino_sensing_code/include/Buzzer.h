#ifndef BUZZER_H
#define BUZZER_H
#include <Arduino.h>

class Buzzer
{
private:
    int m_pin;

public:
    Buzzer(int pin);
    void m_begin();
    void m_on(int hz);
    void m_off();
};

#endif