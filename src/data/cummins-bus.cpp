#ifndef CUMMINS_BUS_CPP
#define CUMMINS_BUS_CPP

#include "cummins-bus.h"

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CumminsBus::CumminsBusCan;
AppData* CumminsBus::_appData = nullptr;
volatile uint32_t CumminsBus::msgCount = 0;

void CumminsBus::setup(AppData* appData) {
    _appData = appData;

    CumminsBusCan.begin();
    CumminsBusCan.setBaudRate(250000);
    CumminsBusCan.setMaxMB(16);
    CumminsBusCan.enableFIFO();
    CumminsBusCan.enableFIFOInterrupt();
    CumminsBusCan.onReceive(onReceive);
}

// Send a PGN 59904 request to the ECU (broadcast)
// Request format: 3 bytes = PGN little-endian
// CAN ID: 0x18EAFF00 (priority 6, PGN 59904/0xEA00, dest 0xFF, src 0x00)
void CumminsBus::requestPgn(uint32_t pgn) {
    CAN_message_t msg;
    msg.id = 0x18EAFFFA;  // src 0xFA (service tool address)
    msg.flags.extended = true;
    msg.len = 3;
    msg.buf[0] = pgn & 0xFF;
    msg.buf[1] = (pgn >> 8) & 0xFF;
    msg.buf[2] = (pgn >> 16) & 0xFF;
    CumminsBusCan.write(msg);
}

void CumminsBus::onReceive(const CAN_message_t &msg) {
    msgCount++;

    // DEBUG: print raw CAN messages
    Serial.print("CAN3 RX ID:0x");
    Serial.print(msg.id, HEX);
    Serial.print(" [");
    Serial.print(msg.len);
    Serial.print("]: ");
    for (uint8_t i = 0; i < msg.len; i++) {
        if (msg.buf[i] < 0x10) Serial.print("0");
        Serial.print(msg.buf[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    if (_appData == nullptr) return;

    // Parse J1939 PGN from CAN ID
    J1939Message j1939;
    j1939.setCanId(msg.id);
    j1939.setData(msg.buf);

    switch (j1939.pgn) {
        // ET1 - Engine Temperature 1
        case 65262: {
            float coolantC = (float)msg.buf[0] - 40.0f;
            _appData->coolantTemp.update(coolantC);
            break;
        }

        // EEC2 - Accelerator Pedal
        case 61443: {
            break;
        }

        default:
            break;
    }
}

#endif // CUMMINS_BUS_CPP
