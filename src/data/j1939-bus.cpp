#ifndef J1939_BUS_CPP
#define J1939_BUS_CPP

#include "j1939-bus.h"

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> J1939Bus::J1939BusCan;
AppData* J1939Bus::_appData = nullptr;

void J1939Bus::setup(AppData* appData) {
    _appData = appData;

    J1939BusCan.begin();
    J1939BusCan.setBaudRate(250000);
    J1939BusCan.setMaxMB(16);
    J1939BusCan.enableFIFO();
    J1939BusCan.enableFIFOInterrupt();
    J1939BusCan.onReceive(onReceive);
}

// PGN 61444 - EEC1 (Electronic Engine Controller 1)
// CAN ID: 0x0CF00400 (priority 3, PGN 61444, source 0x00)
// Broadcast rate: 100ms per J1939-71
void J1939Bus::broadcastEEC1() {
    if (_appData == nullptr) return;

    CAN_message_t msg;
    msg.id = 0x0CF00400;
    msg.flags.extended = true;
    msg.len = 8;

    // Default all bytes to 0xFF (not available)
    memset(msg.buf, 0xFF, 8);

    // Byte 3-4: Engine Speed (SPN 190)
    // Resolution: 0.125 RPM/bit, offset 0
    if (!_appData->engineRpm.isStale(STALE_FAST_MS)) {
        float rpm;
        _appData->engineRpm.read(rpm);
        uint16_t rpmRaw = (uint16_t)(rpm / 0.125f);
        msg.buf[3] = rpmRaw & 0xFF;
        msg.buf[4] = (rpmRaw >> 8) & 0xFF;
    }

    J1939BusCan.write(msg);
}

void J1939Bus::onReceive(const CAN_message_t &msg) {
    // DEBUG: log any J1939 bus traffic
    Serial.print("CAN2 RX ID:0x");
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
}

#endif // J1939_BUS_CPP
