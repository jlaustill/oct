#ifndef CUMMINS_BUS_CPP
#define CUMMINS_BUS_CPP

#include "cummins-bus.h"

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CumminsBus::CumminsBusCan;
AppData* CumminsBus::_appData = nullptr;
volatile uint32_t CumminsBus::msgCount = 0;
volatile uint32_t CumminsBus::lastRxTime = 0;

void CumminsBus::setup(AppData* appData) {
    _appData = appData;

    // Baud scan: try 250k then 500k
    uint32_t baudRates[] = {250000, 500000};
    for (int i = 0; i < 2; i++) {
        CumminsBusCan.begin();
        CumminsBusCan.setBaudRate(baudRates[i]);
        CumminsBusCan.setMaxMB(16);
        CumminsBusCan.enableFIFO();
        CumminsBusCan.enableFIFOInterrupt();
        CumminsBusCan.onReceive(onReceive);

        // Send a test request and check if it gets ACKed
        CAN_message_t test;
        test.id = 0x18EAFFFA;
        test.flags.extended = true;
        test.len = 3;
        test.buf[0] = 0xEE;  // PGN 65262 (ET1)
        test.buf[1] = 0xFE;
        test.buf[2] = 0x00;

        delay(100);
        int result = CumminsBusCan.write(test);
        delay(50);
        CumminsBusCan.events();
        delay(10);
        CumminsBusCan.events();
        int result2 = CumminsBusCan.write(test);

        Serial.print("Cummins baud scan ");
        Serial.print(baudRates[i]);
        Serial.print(": write=");
        Serial.print(result);
        Serial.print(" second=");
        Serial.println(result2);

        if (result2 == 1) {
            Serial.print("Cummins: using ");
            Serial.print(baudRates[i]);
            Serial.println(" bps");
            return;
        }
    }

    Serial.println("Cummins: WARNING - no ACK, defaulting to 250k");
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

// Service 0x4A memory read — send to ECU on Proprietary A (PGN 0xEF00)
// CAN ID: 0x18EF00F9 (priority 6, PGN EF00, dest 0x00=ECU, src 0xF9=tool)
void CumminsBus::readMemory(uint32_t addr, uint8_t len) {
    CAN_message_t msg;
    msg.id = 0x18EF00F9;
    msg.flags.extended = true;
    msg.len = 8;
    msg.buf[0] = 0x4A;                    // Service ID
    msg.buf[1] = (addr >> 24) & 0xFF;     // Address byte 3 (MSB)
    msg.buf[2] = (addr >> 16) & 0xFF;     // Address byte 2
    msg.buf[3] = (addr >> 8) & 0xFF;      // Address byte 1
    msg.buf[4] = addr & 0xFF;             // Address byte 0 (LSB)
    msg.buf[5] = len;                      // Length
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    CumminsBusCan.write(msg);
}

void CumminsBus::onReceive(const CAN_message_t &msg) {
    msgCount++;
    lastRxTime = millis();

    // Handle Service 0x4B responses (memory read reply)
    // CAN ID 0x18EFF900 = ECU(0x00) → Tool(0xF9) on PGN 0xEF00
    if (msg.id == 0x18EFF900 && msg.buf[0] == 0x4B) {
        Serial.print("SVC4A RESP addr:0x");
        uint32_t addr = ((uint32_t)msg.buf[1] << 24) | ((uint32_t)msg.buf[2] << 16) |
                        ((uint32_t)msg.buf[3] << 8) | msg.buf[4];
        Serial.print(addr, HEX);
        Serial.print(" len:");
        Serial.print(msg.buf[5]);
        Serial.print(" data:");
        for (uint8_t i = 6; i < 8; i++) {
            if (msg.buf[i] < 0x10) Serial.print("0");
            Serial.print(msg.buf[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        return;
    }

    if (_appData == nullptr) return;

    // Parse J1939 PGN from CAN ID
    // For PDU2 (pduFormat >= 0xF0): PGN = pduFormat << 8 | pduSpecific
    // For PDU1 (pduFormat < 0xF0): PGN = pduFormat << 8 (pduSpecific is destination)
    J1939Message j1939;
    j1939.setCanId(msg.id);

    uint32_t pgn = (j1939.pduFormat >= 0xF0)
        ? ((uint32_t)j1939.pduFormat << 8) | j1939.pduSpecific
        : ((uint32_t)j1939.pduFormat << 8);

    switch (pgn) {
        // PGN 65248 (0xFEE0) - Vehicle Distance
        case 65248: {
            // Byte 4-7: Total Vehicle Distance, 0.125 km/bit
            uint32_t distRaw = (uint32_t)msg.buf[4] | ((uint32_t)msg.buf[5] << 8) |
                               ((uint32_t)msg.buf[6] << 16) | ((uint32_t)msg.buf[7] << 24);
            float km = distRaw * 0.125f;
            _appData->totalVehicleDistance.update(km);
            Serial.print("CUMMINS odometer: ");
            Serial.print(km, 1);
            Serial.println(" km");
            break;
        }

        // PGN 65262 (0xFEEE) - Engine Temperature 1
        case 65262: {
            float coolantC = (float)msg.buf[0] - 40.0f;
            _appData->coolantTemp.update(coolantC);
            break;
        }

        // PGN 65253 (0xFEE5) - Engine Hours
        case 65253: {
            uint32_t hoursRaw = (uint32_t)msg.buf[0] | ((uint32_t)msg.buf[1] << 8) |
                                ((uint32_t)msg.buf[2] << 16) | ((uint32_t)msg.buf[3] << 24);
            float hours = hoursRaw * 0.05f;
            _appData->engineTotalHours.update(hours);
            Serial.print("CUMMINS hours: ");
            Serial.print(hours, 1);
            Serial.println(" hrs");
            break;
        }

        default:
            break;
    }
}

#endif // CUMMINS_BUS_CPP
