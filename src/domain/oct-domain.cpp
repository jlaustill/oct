#ifndef OCT_DOMAIN_CPP
#define OCT_DOMAIN_CPP

#include "oct-domain.h"
#include <Arduino.h>

#include "data/j1939-bus.h"
#include "data/cummins-bus.h"
#include "data/pci-bus.h"

AppData OctDomain::appData = {
    .engineRpm = {0.0f, "RPM", 0},
    .engineTorque = {0.0f, "%", 0},
    .vehicleSpeed = {0.0f, "km/h", 0},
    .totalVehicleDistance = {0.0f, "km", 0},
    .engineTotalHours = {0.0f, "hours", 0},
    .vin = {{'\0'}, "VIN", 0},
    .coolantTemp = {0.0f, "C", 0},
    .batteryVoltage = {0.0f, "V", 0},
    .ambientTemp = {0.0f, "C", 0}
};

uint32_t lastDebugPrint = 0;

void OctDomain::setup() {
    Serial.begin(115200);
    J1939Bus::setup();
    CumminsBus::setup();
    PciBus::setup(&appData);
}

void OctDomain::loop() {
    // Debug: print AppData every 1 second
    uint32_t now = millis();
    if (now - lastDebugPrint >= 1000) {
        lastDebugPrint = now;

        float rpm, speed, coolant, battery, ambient;
        appData.engineRpm.read(rpm);
        appData.vehicleSpeed.read(speed);
        appData.coolantTemp.read(coolant);
        appData.batteryVoltage.read(battery);
        appData.ambientTemp.read(ambient);

        Serial.print("RPM:");
        Serial.print(rpm, 0);
        Serial.print(" | Speed:");
        Serial.print(speed, 1);
        Serial.print("km/h | Coolant:");
        Serial.print(coolant, 1);
        Serial.print("C | Batt:");
        Serial.print(battery, 2);
        Serial.print("V | Ambient:");
        Serial.print(ambient, 1);
        Serial.print("C");

        // PCI bus stats
        Serial.print(" | PCI msgs:");
        Serial.print(PciBus::_msgCount);
        Serial.print(" errs:");
        Serial.print(PciBus::_errCount);

        // Staleness indicators
        if (appData.engineRpm.isStale(2000)) Serial.print(" [RPM STALE]");
        if (appData.vehicleSpeed.isStale(2000)) Serial.print(" [SPEED STALE]");

        Serial.println();
    }
}

#endif // OCT_DOMAIN_CPP