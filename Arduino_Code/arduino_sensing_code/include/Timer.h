#ifndef TIMER_H
#define TIMER_H
#include <Arduino.h>

class Timer {
private:
    unsigned long m_previousMillis;
    unsigned long m_interval;

public:
    Timer(unsigned long intervalMs) 
        : m_previousMillis(0), m_interval(intervalMs) {}

    void reset() {
        m_previousMillis = millis();
    }

    bool expired() {
        return (millis() - m_previousMillis) >= m_interval;
    }

    void setInterval(unsigned long intervalMs) {
        m_interval = intervalMs;
    }

    unsigned long interval() const {
        return m_interval;
    }
};

#endif
