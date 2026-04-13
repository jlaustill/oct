#include "cummins-bus.h"

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CumminsBus::CumminsBusCan;
volatile AppData* CumminsBus::_appData = nullptr;
volatile uint32_t CumminsBus::msgCount = 0;
elapsedMillis CumminsBus::sinceLastRx;

void CumminsBus::setup(AppData* appData) {
    _appData = appData;
    
    CumminsBusCan.begin();
    CumminsBusCan.setBaudRate(250000);
    CumminsBusCan.setMaxMB(16);
    CumminsBusCan.enableFIFO();
    CumminsBusCan.enableFIFOInterrupt();
    CumminsBusCan.onReceive(onReceive);
}

void CumminsBus::loop() {
    CumminsBusCan.events();
}

void CumminsBus::requestPgn(uint32_t pgn) {
    CAN_message_t msg;
    msg.id = 0x18EAFFFA;
    msg.flags.extended = true;
    msg.len = 3;
    msg.buf[0] = pgn & 0xFF;
    msg.buf[1] = (pgn >> 8) & 0xFF;
    msg.buf[2] = (pgn >> 16) & 0xFF;
    CumminsBusCan.write(msg);
}

void CumminsBus::onReceive(const CAN_message_t &msg) {
    msgCount++;
    sinceLastRx = 0;
    // No PGN handlers wired — OCT is the authoritative source for
    // odometer, hours, coolant, speed, and VIN (PCI provides the
    // live values). Resurrect handlers here when ECU-only data is
    // needed in the future.
}
