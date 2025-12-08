#include "LCDDisplay.h"

LCDDisplay::LCDDisplay(uint8_t address, uint8_t cols, uint8_t rows)
    : m_lcd(address, cols, rows), m_lastLine1(""), m_lastLine2("") {}

void LCDDisplay::m_begin()
{
    m_lcd.init();
    m_lcd.backlight();
}
void LCDDisplay::m_show(const String& line1, const String& line2) {
    if(line1 != m_lastLine1 || line2 != m_lastLine2) {
        m_lcd.clear();
        m_lcd.setCursor(0, 0);
        m_lcd.print(line1);
        m_lcd.setCursor(0, 1);
        m_lcd.print(line2);
        m_lastLine1 = line1;
        m_lastLine2 = line2;
    }
}