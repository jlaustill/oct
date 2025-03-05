#ifndef CUMMINS_BUS_H
#define CUMMINS_BUS_H

#include <FlexCAN_T4.h>
#include "j1939-bus.h"

class CumminsBus {
    public:
        static void setup();
        static FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CumminsBusCan;

    private:
        static void CumminsBusSniff(const CAN_message_t &msg);
};

#endif // CUMMINS_BUS_H