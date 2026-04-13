#ifndef ENGINE_HOURS_TRUCK_LIFETIME_H
#define ENGINE_HOURS_TRUCK_LIFETIME_H

#include <Arduino.h>
#include "data/app-data.h"

// Owns the full lifecycle of AppData::engineHoursTruckLifetime:
//   1. load from EEPROM on boot
//   2. accumulate every 1s when RPM > running threshold
//   3. persist to EEPROM every 30s
//
// Lifetime counter — never reset on rebuild. Runs in parallel with
// EngineHoursSinceRebuild under the same accumulation rule.
//
// Seeding is a separate, one-time operation — not handled here.
class EngineHoursTruckLifetime {
    public:
        static void setup(AppData* appData);
        static void loop();
        static void setAndSave(float hours);   // intentional seed from operator

    private:
        static AppData* _appData;
        static elapsedMillis _sinceAccumulate;
        static elapsedMillis _sinceSave;
        static void load();
        static void accumulate();
        static void save();
};

#endif // ENGINE_HOURS_TRUCK_LIFETIME_H
