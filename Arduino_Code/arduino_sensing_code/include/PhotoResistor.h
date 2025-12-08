#ifndef PHOTORESISTOR_H
#define PHOTORESISTOR_H
#include <Arduino.h>

class PhotoResistor
{
private:
    int m_pin;

public:
    PhotoResistor(int pin);
    void m_begin();
    bool isPhotoHigh();
};

#endif