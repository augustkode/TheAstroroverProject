#ifndef LCDDISPLAY_H
#define LCDDISPLAY_H
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

class LCDDisplay
{
private:
    LiquidCrystal_I2C m_lcd;
    String m_lastLine1, m_lastLine2;

public:
    LCDDisplay(uint8_t address, uint8_t cols, uint8_t rows);
    void m_begin();
    void m_show(const String& lin1, const String& line2);
};

#endif