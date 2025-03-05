#ifndef OCT_DOMAIN_CPP
#define OCT_DOMAIN_CPP

#include "oct-domain.h"
#include <Arduino.h>

#include "data/j1939-bus.h"
#include "data/cummins-bus.h"

void OctDomain::setup() {
    Serial.begin(115200);
    J1939Bus::setup();
    CumminsBus::setup();
}

void OctDomain::loop() {
}

#endif // OCT_DOMAIN_CPP