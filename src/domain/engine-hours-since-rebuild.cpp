#include "engine-hours-since-rebuild.h"
#include <EEPROM.h>

// EEPROM slice (bytes 0x04..0x07) — preserves any existing value
// from the pre-split "engineTotalHours" layout.
#define EEPROM_ADDR_VALUE_HOURS  0x04   // float, hours
#define EEPROM_UNINITIALIZED     0xFFFFFFFFu

// Matches CM848D firmware RUNNING_RPM_THRESHOLD (kuminz-re
// firmware_types.hpp:1035). Below this, the engine is cranking.
#define RUNNING_RPM_THRESHOLD 300.0f

#define ACCUMULATE_INTERVAL_MS 1000
#define SAVE_INTERVAL_MS       30000
#define RPM_STALE_MS           2000

AppData* EngineHoursSinceRebuild::_appData = nullptr;
elapsedMillis EngineHoursSinceRebuild::_sinceAccumulate;
elapsedMillis EngineHoursSinceRebuild::_sinceSave;

void EngineHoursSinceRebuild::setup(AppData* appData) {
    _appData = appData;
    load();
    _sinceAccumulate = 0;
    _sinceSave = 0;
}

void EngineHoursSinceRebuild::loop() {
    if (_sinceAccumulate >= ACCUMULATE_INTERVAL_MS) {
        _sinceAccumulate = 0;
        accumulate();
    }

    if (_sinceSave >= SAVE_INTERVAL_MS) {
        _sinceSave = 0;
        save();
    }
}

void EngineHoursSinceRebuild::load() {
    uint32_t raw;
    EEPROM.get(EEPROM_ADDR_VALUE_HOURS, raw);

    if (raw == EEPROM_UNINITIALIZED) {
        Serial.println("EEPROM: EngineHoursSinceRebuild NOT SEEDED — value will start at 0");
        return;
    }

    float hours;
    memcpy(&hours, &raw, sizeof(float));
    _appData->engineHoursSinceRebuild.update(hours);
    Serial.print("EEPROM: EngineHoursSinceRebuild loaded ");
    Serial.print(hours, 1);
    Serial.println(" hrs");
}

void EngineHoursSinceRebuild::accumulate() {
    float rpm;
    _appData->engineRpm.read(rpm);
    if (rpm <= RUNNING_RPM_THRESHOLD || _appData->engineRpm.isStale(RPM_STALE_MS)) return;

    float hours;
    _appData->engineHoursSinceRebuild.read(hours);
    hours += 1.0f / 3600.0f;  // 1 s → hours
    _appData->engineHoursSinceRebuild.update(hours);
}

void EngineHoursSinceRebuild::save() {
    float hours;
    _appData->engineHoursSinceRebuild.read(hours);
    EEPROM.put(EEPROM_ADDR_VALUE_HOURS, hours);
}

void EngineHoursSinceRebuild::setAndSave(float hours) {
    _appData->engineHoursSinceRebuild.update(hours);
    save();
    _sinceSave = 0;
}
