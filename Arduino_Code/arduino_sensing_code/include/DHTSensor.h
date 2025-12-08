#ifndef DHTSENSOR_H
#define DHTSENSOR_H

#include <DHT.h>
#include <Arduino.h>

class DHTSensor
{
private:
    DHT m_dht;

public:
    DHTSensor(uint8_t pin, uint8_t type);
    void m_begin();
    float m_temperature();
    float m_humidity();
};

#endif