#include "cummins-bus.h"

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CumminsBus::CumminsBusCan;
volatile AppData* CumminsBus::_appData = nullptr;
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

        CAN_message_t test;
        test.id = 0x18EAFFFA;
        test.flags.extended = true;
        test.len = 3;
        test.buf[0] = 0xEE;
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

// Service 0x4A memory read — only request 1-2 bytes (single-frame response)
void CumminsBus::readMemory(uint32_t addr, uint8_t len) {
    CAN_message_t msg;
    msg.id = 0x18EF00F9;
    msg.flags.extended = true;
    msg.len = 8;
    msg.buf[0] = 0x4A;
    msg.buf[1] = (addr >> 24) & 0xFF;
    msg.buf[2] = (addr >> 16) & 0xFF;
    msg.buf[3] = (addr >> 8) & 0xFF;
    msg.buf[4] = addr & 0xFF;
    msg.buf[5] = (len <= 2) ? len : 2;  // cap at 2 to stay single-frame
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    CumminsBusCan.write(msg);
}

void CumminsBus::onReceive(const CAN_message_t &msg) {
    msgCount++;
    lastRxTime = millis();

    // Handle Service 0x4B responses (single-frame memory read reply)
    if (msg.id == 0x18EFF900 && msg.buf[0] == 0x4B) {
        uint32_t addr = ((uint32_t)msg.buf[1] << 24) | ((uint32_t)msg.buf[2] << 16) |
                        ((uint32_t)msg.buf[3] << 8) | msg.buf[4];
        uint16_t data = ((uint16_t)msg.buf[6] << 8) | msg.buf[7];

        // Engine hours: combine hi/lo reads from EEPROM 0x01000035
        // Hi read always comes first (requested first in loop), lo follows immediately
        static volatile uint16_t engineHoursHi = 0;
        static volatile uint32_t engineHoursHiTime = 0;

        if (addr == 0x01000035) {
            engineHoursHi = data;
            engineHoursHiTime = millis();
        } else if (addr == 0x01000037) {
            // Only combine if hi word was read recently (within 5s)
            if (_appData != nullptr && (millis() - engineHoursHiTime) < 5000) {
                uint32_t raw = ((uint32_t)engineHoursHi << 16) | data;
                float hours = raw * 0.2f / 3600.0f;
                _appData->engineTotalHours.update(hours);
            }
        }

        return;
    }

    if (_appData == nullptr) return;

    // Parse J1939 PGN from CAN ID
    J1939Message j1939;
    j1939.setCanId(msg.id);

    uint32_t pgn = (j1939.pduFormat >= 0xF0)
        ? ((uint32_t)j1939.pduFormat << 8) | j1939.pduSpecific
        : ((uint32_t)j1939.pduFormat << 8);

    switch (pgn) {
        // PGN 65248 (0xFEE0) - Vehicle Distance
        case 65248: {
            uint32_t distRaw = (uint32_t)msg.buf[4] | ((uint32_t)msg.buf[5] << 8) |
                               ((uint32_t)msg.buf[6] << 16) | ((uint32_t)msg.buf[7] << 24);
            float km = distRaw * 0.125f;
            _appData->totalVehicleDistance.update(km);
            break;
        }

        // PGN 65262 (0xFEEE) - Engine Temperature 1
        // SPN 110: bytes 0-1, little-endian, 0.03125 °C/bit, offset -273 °C
        case 65262: {
            uint16_t coolantRaw = (uint16_t)msg.buf[0] | ((uint16_t)msg.buf[1] << 8);
            float coolantC = coolantRaw * 0.03125f - 273.0f;
            _appData->coolantTemp.update(coolantC);
            break;
        }

        default:
            break;
    }
}
