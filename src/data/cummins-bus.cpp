#ifndef CUMMINS_BUS_CPP
#define CUMMINS_BUS_CPP

#include "cummins-bus.h"

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CumminsBus::CumminsBusCan;


void CumminsBus::setup() {
    CumminsBus::CumminsBusCan.begin();
    CumminsBus::CumminsBusCan.setBaudRate(250 * 1000);
    CumminsBus::CumminsBusCan.setMaxMB(16);
    CumminsBus::CumminsBusCan.enableFIFO();
    CumminsBus::CumminsBusCan.enableFIFOInterrupt();
    CumminsBus::CumminsBusCan.onReceive(CumminsBusSniff);
    CumminsBus::CumminsBusCan.mailboxStatus();
}

void CumminsBus::CumminsBusSniff(const CAN_message_t &msg) {
    J1939Bus::J1939BusCan.write(msg);
}

#endif // CUMMINS_BUS_CPP