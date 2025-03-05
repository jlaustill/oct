#ifndef J1939_BUS_H
#define J1939_BUS_H

#include <FlexCAN_T4.h>
#include <J1939Message.h>
#include "cummins-bus.h"

class J1939Bus {
    public:
        static void setup();
        static FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> J1939BusCan;

    private:
        static void J1939BusSniff(const CAN_message_t &msg);

};

#endif // J1939_BUS_H