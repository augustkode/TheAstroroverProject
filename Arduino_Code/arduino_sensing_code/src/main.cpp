#include "LEDPin.h"
#include "PirSensor.h"
#include "Buzzer.h"
#include "PhotoResistor.h"
#include "LCDDisplay.h"
#include "RGBLed.h"
#include "DHTSensor.h"
#include "ModeController.h"
#include "RPiCommands.h"
#include "IRSensor.h"

LEDPin ledPin(8);
PIRSensor pirPin(7);
Buzzer buzzer(6);
PhotoResistor photoResistor(2);
LCDDisplay lcdDisplay(0x27, 16, 2);
RGBLed rgbLed(10, 11, 9);
DHTSensor dhtSensor(4, DHT11);
IRSensor irSensor(5);
ModeController modeController(lcdDisplay, ledPin, pirPin, rgbLed, buzzer, photoResistor, dhtSensor);
float t, h;

void setup()
{
  ledPin.m_begin();
  pirPin.m_begin();
  buzzer.m_begin();
  lcdDisplay.m_begin();
  photoResistor.m_begin();
  rgbLed.m_begin();
  dhtSensor.m_begin();
  irSensor.IRBegin();
  Serial.begin(9600);
  Serial.println("Starting");
}



void loop()
{
  modeController.update();
  commandsFromRaspberryPi(dhtSensor, modeController);
}