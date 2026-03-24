#ifndef OCT_DOMAIN_H
#define OCT_DOMAIN_H

#include "data/app-data.h"

#define CUMMINS_BUS CAN3
#define J1939_BUS CAN2

class OctDomain {
    public:
        static void setup();
        static void loop();
        static AppData appData;
};

#endif // OCT_DOMAIN_H