#ifndef OCT_DOMAIN_CPP
#define OCT_DOMAIN_CPP

#include "oct-domain.h"
#include <Arduino.h>

#include "data/j1939-bus.h"
#include "data/cummins-bus.h"

AppData OctDomain::appData = {
    .engineRpm = {0.0f, "RPM", 0},
    .engineTorque = {0.0f, "%", 0},
    .vehicleSpeed = {0.0f, "km/h", 0},
    .totalVehicleDistance = {0.0f, "km", 0},
    .engineTotalHours = {0.0f, "hours", 0},
    .vin = {{'\0'}, "VIN", 0}
};

void OctDomain::setup() {
    Serial.begin(115200);
    J1939Bus::setup();
    CumminsBus::setup();
}

void OctDomain::loop() {
}

#endif // OCT_DOMAIN_CPP