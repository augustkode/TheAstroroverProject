
#ifndef RGBLED_H
#define RGBLED_H
#include <Arduino.h>

class RGBLed
{
private:
    int m_r, m_g, m_b;
public:
    RGBLed(int redPin, int greenPin, int bluePin);
    void m_begin();
    void m_setColor(int red, int green, int blue);
    void m_off();
};
#endif