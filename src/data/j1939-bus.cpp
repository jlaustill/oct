#ifndef J1939_BUS_CPP
#define J1939_BUS_CPP

#include "j1939-bus.h"

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> J1939Bus::J1939BusCan;


void J1939Bus::setup() {
    J1939Bus::J1939BusCan.begin();
    J1939Bus::J1939BusCan.setBaudRate(250 * 1000);
    J1939Bus::J1939BusCan.setMaxMB(16);
    J1939Bus::J1939BusCan.enableFIFO();
    J1939Bus::J1939BusCan.enableFIFOInterrupt();
    J1939Bus::J1939BusCan.onReceive(J1939BusSniff);
    J1939Bus::J1939BusCan.mailboxStatus();
}

void J1939Bus::J1939BusSniff(const CAN_message_t &msg) {
    J1939Message message = J1939Message();
    message.setCanId(msg.id);
    message.setData(msg.buf);
  
    if (message.pgn == 59904) {
      // Only send if it is a request
      CumminsBus::CumminsBusCan.write(msg);
    }
}

#endif // J1939_BUS_CPP