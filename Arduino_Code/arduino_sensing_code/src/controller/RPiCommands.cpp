#include "RPiCommands.h"
#include "ModeController.h"
#include "DHTSensor.h"

extern ModeController modeController;

void commandsFromRaspberryPi(DHTSensor& dht, ModeController& modeCtrl)
{
    if(Serial.available())
    {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if(cmd.equalsIgnoreCase("AUTO")) { modeController.setMode(AUTO);}
    else if(cmd.equalsIgnoreCase("ON")) { modeController.setMode(FORCE_ON); }    
    else if(cmd.equalsIgnoreCase("OFF")) { modeController.setMode(FORCE_OFF); }
    else if(cmd.equalsIgnoreCase("GUARDIAN")) { modeController.setMode(GUARD_MODE); }
    else if(cmd.equalsIgnoreCase("SENSORMEASUREMENT")) {
    float t = dht.m_temperature();
    float h = dht.m_humidity();
    String ledState = digitalRead(8) ? "ON" : "OFF"; // LED pin 8
    String modeStr = modeController.modeName();
    String pirrActive = digitalRead(7) ? "YES" : "NO";
    
    Serial.print(t, 2); Serial.print(",");
    Serial.print(h, 2); Serial.print(",");
    Serial.print(ledState); Serial.print(",");
    Serial.print(modeStr); Serial.print(",");
    Serial.println(pirrActive);
    }
  }
}
