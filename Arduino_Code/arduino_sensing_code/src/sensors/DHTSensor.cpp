#include "DHTSensor.h"

DHTSensor::DHTSensor(uint8_t pin, uint8_t type) : m_dht(pin, type) {}

void DHTSensor::m_begin()
{
    m_dht.begin();
}

float DHTSensor::m_temperature()
{
    return m_dht.readTemperature();
}

float DHTSensor::m_humidity()
{
    return m_dht.readHumidity();
}