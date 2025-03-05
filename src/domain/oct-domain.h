#ifndef OCT_DOMAIN_H
#define OCT_DOMAIN_H

#define CUMMINS_BUS CAN3
#define J1939_BUS CAN2

class OctDomain {
    public:
        static void setup();
        static void loop();
};

#endif // OCT_DOMAIN_H