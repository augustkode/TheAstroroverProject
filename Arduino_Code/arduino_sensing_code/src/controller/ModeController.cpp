// ModeController.cpp
#include "ModeController.h"
#include <Arduino.h>

ModeController::ModeController(LCDDisplay &lcdRef, LEDPin &ledRef, PIRSensor &pirRef,
                               RGBLed &rgbRef, Buzzer &buzzRef, PhotoResistor &photoRef, DHTSensor &dhtRef)
    : m_currentMode(AUTO),
      m_prevoiusMode(AUTO),
      m_lcdDisplay(lcdRef),
      m_ledPin(ledRef),
      m_pirPin(pirRef),
      m_rgbLed(rgbRef),
      m_buzzer(buzzRef),
      m_photoResistor(photoRef),
      m_dhtSensor(dhtRef),
      alarmActive(false),
      alarmStartTime(0)
{}

void ModeController::setMode(Mode mode) {
    m_prevoiusMode = m_currentMode;
    m_currentMode = mode;
    Serial.print("ACK:");
    Serial.println(modeName());

    // Slå av alt når modus endres
    m_rgbLed.m_off();
    m_buzzer.m_off();
    alarmActive = false;
}

Mode ModeController::getMode() const { return m_currentMode; }
bool ModeController::modeChanged() const { return m_currentMode != m_prevoiusMode; }

String ModeController::modeName() const {
    switch(m_currentMode) {
        case AUTO: return "AUTO";
        case FORCE_ON: return "ON";
        case FORCE_OFF: return "OFF";
        case GUARD_MODE: return "GUARDING";
        case ALARM: return "ALARM";
        default: return "UNKNOWN";
    }
}

void ModeController::update() {
    // Slå av RGB og buzzer hvis ikke i guard/ALARM
    if (m_currentMode != GUARD_MODE && m_currentMode != ALARM) {
        m_buzzer.m_off();
        m_rgbLed.m_off();
        alarmActive = false;
    }

    switch (m_currentMode) {
        case AUTO: handleAuto(); break;
        case FORCE_ON: handleOn(); break;
        case FORCE_OFF: handleOff(); break;
        case GUARD_MODE: handleGuard(); break;
        case ALARM: handleAlarm(); break;
    }
}

void ModeController::handleAuto() {
    if(!m_photoResistor.isPhotoHigh()) {
        m_ledPin.m_off();
        m_lcdDisplay.m_show("Mode: AUTO", "LED: OFF");
    } else {
        m_ledPin.m_on();
        m_lcdDisplay.m_show("Mode: AUTO", "LED: ON");
    }
    m_rgbLed.m_setColor(255, 0, 255); //magenta<3 toxic
}

void ModeController::handleOn() {
    m_ledPin.m_on();
    m_lcdDisplay.m_show("Mode: ON", "LED: ON");
    m_rgbLed.m_setColor(0, 0, 255); //blue, when on
}

void ModeController::handleOff() {
    m_ledPin.m_off();
    m_lcdDisplay.m_show("Mode: OFF", "LED: OFF");
    m_rgbLed.m_setColor(0, 0, 0); //off, when off
}

void ModeController::handleGuard() {
    m_ledPin.m_off();
    m_lcdDisplay.m_show("Mode: GUARD", "GUARDING");
    m_rgbLed.m_setColor(0, 255, 128);

    if(m_pirPin.m_motionDetect()) {
        setMode(ALARM); 
    }
}

void ModeController::handleAlarm() {
    if (!alarmActive) {
        // Start alarm
        m_buzzer.m_on(1000);
        m_rgbLed.m_setColor(255, 0, 0);
        m_lcdDisplay.m_show("!!ALARM!!", "E-POST");
        alarmActive = true;
        alarmStartTime = millis();
    }

    // Sjekk om alarm skal stoppes
    if (alarmActive && !m_pirPin.m_motionDetect() && (millis() - alarmStartTime >= alarmDuration)) {
        m_buzzer.m_off();
        m_rgbLed.m_off();
        alarmActive = false;
        setMode(GUARD_MODE);
    }
}
