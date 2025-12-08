// ModeController.h
#pragma once
#include "LEDPin.h"
#include "PirSensor.h"
#include "Buzzer.h"
#include "PhotoResistor.h"
#include "LCDDisplay.h"
#include "RGBLed.h"
#include "DHTSensor.h"

enum Mode { AUTO, FORCE_ON, FORCE_OFF, GUARD_MODE, ALARM };

class ModeController {
public:
    ModeController(LCDDisplay &lcdRef, LEDPin &ledRef, PIRSensor &pirRef,
                   RGBLed &rgbRef, Buzzer &buzzRef, PhotoResistor &photoRef, DHTSensor &dhtRef);

    void setMode(Mode mode);
    Mode getMode() const;
    bool modeChanged() const;
    String modeName() const;

    void update(); // Kall denne i loop()

private:
    // Håndteringsfunksjoner
    void handleAuto();
    void handleOn();
    void handleOff();
    void handleGuard();
    void handleAlarm();
    void handleReboot();

    // Medlem-variabler
    Mode m_currentMode;
    Mode m_prevoiusMode;

    LCDDisplay &m_lcdDisplay;
    LEDPin &m_ledPin;
    PIRSensor &m_pirPin;
    RGBLed &m_rgbLed;
    Buzzer &m_buzzer;
    PhotoResistor &m_photoResistor;
    DHTSensor &m_dhtSensor;

    // Alarmvariabler
    bool alarmActive;
    unsigned long alarmStartTime;
    const unsigned long alarmDuration = 5000; // 5 sek

    // Alarm "trigger delay"
    bool guardWaiting = false;        // om vi venter på PIR
    unsigned long guardStartTime = 0; // starttid for ventetid
    const unsigned long guardDelay = 5000; // 5 sekunder

};
