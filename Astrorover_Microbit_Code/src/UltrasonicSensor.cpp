#include "UltrasonicSensor.h"

UltrasonicSensor::UltrasonicSensor(uint8_t trigPin, uint8_t echoPin)
    : m_trigPin(trigPin), m_echoPin(echoPin) {
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);
    }

float UltrasonicSensor::measureDistanceUltrasonicCM() {
    digitalWrite(m_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(m_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(m_trigPin, LOW);

    long duration = pulseIn(m_echoPin, HIGH, 30000);
    if (duration == 0) return -1;

    float distance = duration * 0.0343 / 2.0;
    return distance;
}