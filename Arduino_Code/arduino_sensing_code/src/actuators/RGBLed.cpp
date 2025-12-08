#include "RGBLed.h"

RGBLed::RGBLed(int redPin, int greenPin, int bluePin) : m_r(redPin), m_g(greenPin), m_b(bluePin) {}

void RGBLed::m_begin()
{
    pinMode(m_r, OUTPUT);
    pinMode(m_g, OUTPUT);
    pinMode(m_b, OUTPUT);
}

void RGBLed::m_setColor(int red, int green, int blue)
{
    analogWrite(m_r, red);
    analogWrite(m_g, green);
    analogWrite(m_b, blue);
}

void RGBLed::m_off()
{
    m_setColor(0, 0, 0);
}