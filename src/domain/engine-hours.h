#ifndef ENGINE_HOURS_H
#define ENGINE_HOURS_H

#include <Arduino.h>
#include "data/app-data.h"

// Owns the full lifecycle of AppData::engineTotalHours:
//   1. load from EEPROM on boot
//   2. accumulate every 1s when RPM > running threshold
//   3. persist to EEPROM every 30s
//
// Running threshold matches the CM848D firmware (300 RPM — see
// kuminz-re firmware_types.hpp: RUNNING_RPM_THRESHOLD).
//
// Seeding is a separate, one-time operation — not handled here.
class EngineHours {
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

#endif // ENGINE_HOURS_H
