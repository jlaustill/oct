#include "engine-hours-truck-lifetime.h"
#include <EEPROM.h>

// EEPROM slice (bytes 0x08..0x0B)
#define EEPROM_ADDR_VALUE_HOURS  0x08   // float, hours
#define EEPROM_UNINITIALIZED     0xFFFFFFFFu

// Matches CM848D firmware RUNNING_RPM_THRESHOLD (kuminz-re
// firmware_types.hpp:1035). Below this, the engine is cranking.
#define RUNNING_RPM_THRESHOLD 300.0f

#define ACCUMULATE_INTERVAL_MS 1000
#define SAVE_INTERVAL_MS       30000
#define RPM_STALE_MS           2000

AppData* EngineHoursTruckLifetime::_appData = nullptr;
elapsedMillis EngineHoursTruckLifetime::_sinceAccumulate;
elapsedMillis EngineHoursTruckLifetime::_sinceSave;

void EngineHoursTruckLifetime::setup(AppData* appData) {
    _appData = appData;
    load();
    _sinceAccumulate = 0;
    _sinceSave = 0;
}

void EngineHoursTruckLifetime::loop() {
    if (_sinceAccumulate >= ACCUMULATE_INTERVAL_MS) {
        _sinceAccumulate = 0;
        accumulate();
    }

    if (_sinceSave >= SAVE_INTERVAL_MS) {
        _sinceSave = 0;
        save();
    }
}

void EngineHoursTruckLifetime::load() {
    uint32_t raw;
    EEPROM.get(EEPROM_ADDR_VALUE_HOURS, raw);

    if (raw == EEPROM_UNINITIALIZED) {
        Serial.println("EEPROM: EngineHoursTruckLifetime NOT SEEDED — value will start at 0");
        return;
    }

    float hours;
    memcpy(&hours, &raw, sizeof(float));
    _appData->engineHoursTruckLifetime.update(hours);
    Serial.print("EEPROM: EngineHoursTruckLifetime loaded ");
    Serial.print(hours, 1);
    Serial.println(" hrs");
}

void EngineHoursTruckLifetime::accumulate() {
    float rpm;
    _appData->ecu.engineRpm.read(rpm);
    if (rpm <= RUNNING_RPM_THRESHOLD || _appData->ecu.engineRpm.isStale(RPM_STALE_MS)) return;

    float hours;
    _appData->engineHoursTruckLifetime.read(hours);
    hours += 1.0f / 3600.0f;  // 1 s → hours
    _appData->engineHoursTruckLifetime.update(hours);
}

void EngineHoursTruckLifetime::save() {
    float hours;
    _appData->engineHoursTruckLifetime.read(hours);
    EEPROM.put(EEPROM_ADDR_VALUE_HOURS, hours);
}

void EngineHoursTruckLifetime::setAndSave(float hours) {
    _appData->engineHoursTruckLifetime.update(hours);
    save();
    _sinceSave = 0;
}
