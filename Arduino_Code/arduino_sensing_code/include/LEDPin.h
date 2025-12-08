#ifndef LEDPIN_H
#define LEDPIN_H
#include <Arduino.h>

class LEDPin
{
private:
    int m_pin;

public: 
    LEDPin(int pin);
    void m_begin();
    void m_on();
    void m_off();
};
#endif