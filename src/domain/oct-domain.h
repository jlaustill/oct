#ifndef OCT_DOMAIN_H
#define OCT_DOMAIN_H

#include "data/app-data.h"

#define CUMMINS_BUS CAN2
#define J1939_BUS CAN3

class OctDomain {
    public:
        static void setup();
        static void loop();
        static AppData appData;
};

#endif // OCT_DOMAIN_H