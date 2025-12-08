#ifndef RPICOMMANDS_H
#define RPICOMMANDS_H

#include "ModeController.h"
#include <ModeController.h>
#include <Arduino.h>

//extern ModeController modeController;

void commandsFromRaspberryPi(DHTSensor& dht, ModeController& modeCtrl);

#endif