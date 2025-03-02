#ifndef OCT_MAIN_CPP
#define OCT_MAIN_CPP

#include <Arduino.h>
#include "domain/oct-domain.h"

void setup() {
    OctDomain::setup();
}

void loop() {
    OctDomain::loop();
}

#endif