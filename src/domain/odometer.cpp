#include "odometer.h"
#include <EEPROM.h>

// EEPROM slice
#define EEPROM_ADDR_TOTAL    0x00          // uint32_t, 1/8000 mi units
#define EEPROM_UNINITIALIZED 0xFFFFFFFFu

// 1 count = 1/8000 mi = 0.000125 mi = 0.000201168 km = 0.201168 m
#define COUNTS_PER_MILE      8000u
#define KM_PER_MILE          1.609344

#define SAVE_INTERVAL_MS     30000

uint32_t Odometer::_total = 0;
elapsedMillis Odometer::_sinceSave;

void Odometer::setup() {
    load();
    _sinceSave = 0;
}

void Odometer::loop() {
    if (_sinceSave >= SAVE_INTERVAL_MS) {
        _sinceSave = 0;
        save();
    }
}

void Odometer::onPulses(uint8_t pulses) {
    _total += pulses;
}

void Odometer::setAndSave(float km) {
    double miles = (double)km / KM_PER_MILE;
    _total = (uint32_t)(miles * COUNTS_PER_MILE + 0.5);
    save();
    _sinceSave = 0;
}

uint32_t Odometer::rawCounts() {
    return _total;
}

float Odometer::km() {
    return (float)((double)_total / COUNTS_PER_MILE * KM_PER_MILE);
}

void Odometer::load() {
    uint32_t raw;
    EEPROM.get(EEPROM_ADDR_TOTAL, raw);

    if (raw == EEPROM_UNINITIALIZED) {
        Serial.println("EEPROM: Odometer NOT SEEDED — value will start at 0");
        return;
    }

    _total = raw;
    Serial.print("EEPROM: Odometer loaded ");
    Serial.print(_total);
    Serial.print(" counts (");
    Serial.print(km(), 3);
    Serial.println(" km)");
}

void Odometer::save() {
    EEPROM.put(EEPROM_ADDR_TOTAL, _total);
}
