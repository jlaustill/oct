#include "odometer.h"
#include <EEPROM.h>

// EEPROM slice (bytes 0x00..0x03)
#define EEPROM_ADDR_VALUE_KM  0x00   // float, km
#define EEPROM_UNINITIALIZED  0xFFFFFFFFu  // fresh flash reads all 0xFF

#define ACCUMULATE_INTERVAL_MS 1000
#define SAVE_INTERVAL_MS       30000
#define SPEED_STALE_MS         2000

AppData* Odometer::_appData = nullptr;
elapsedMillis Odometer::_sinceAccumulate;
elapsedMillis Odometer::_sinceSave;

void Odometer::setup(AppData* appData) {
    _appData = appData;
    load();
    _sinceAccumulate = 0;
    _sinceSave = 0;
}

void Odometer::loop() {
    if (_sinceAccumulate >= ACCUMULATE_INTERVAL_MS) {
        _sinceAccumulate = 0;
        accumulate();
    }

    if (_sinceSave >= SAVE_INTERVAL_MS) {
        _sinceSave = 0;
        save();
    }
}

void Odometer::load() {
    uint32_t raw;
    EEPROM.get(EEPROM_ADDR_VALUE_KM, raw);

    if (raw == EEPROM_UNINITIALIZED) {
        Serial.println("EEPROM: Odometer NOT SEEDED — value will start at 0");
        return;
    }

    float km;
    memcpy(&km, &raw, sizeof(float));
    _appData->totalVehicleDistance.update(km);
    Serial.print("EEPROM: Odometer loaded ");
    Serial.print(km, 1);
    Serial.println(" km");
}

void Odometer::accumulate() {
    float speed;
    _appData->vehicleSpeed.read(speed);
    if (speed <= 0.0f || _appData->vehicleSpeed.isStale(SPEED_STALE_MS)) return;

    float km;
    _appData->totalVehicleDistance.read(km);
    km += speed / 3600.0f;  // km/h × 1 s → km
    _appData->totalVehicleDistance.update(km);
}

void Odometer::save() {
    float km;
    _appData->totalVehicleDistance.read(km);
    EEPROM.put(EEPROM_ADDR_VALUE_KM, km);
}
