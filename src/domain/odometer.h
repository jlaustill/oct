#ifndef ODOMETER_H
#define ODOMETER_H

#include <Arduino.h>
#include "data/app-data.h"

// Owns the full lifecycle of AppData::totalVehicleDistance:
//   1. load from EEPROM on boot
//   2. accumulate every 1s from PCI-derived vehicle speed
//   3. persist to EEPROM every 30s
//
// Seeding is a separate, one-time operation — not handled here.
class Odometer {
    public:
        static void setup(AppData* appData);
        static void loop();

    private:
        static AppData* _appData;
        static elapsedMillis _sinceAccumulate;
        static elapsedMillis _sinceSave;
        static void load();
        static void accumulate();
        static void save();
};

#endif // ODOMETER_H
