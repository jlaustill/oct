#ifndef J1939_BUS_CPP
#define J1939_BUS_CPP

#include "j1939-bus.h"

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> J1939Bus::J1939BusCan;
AppData* J1939Bus::_appData = nullptr;
TpBamState J1939Bus::_tp = {false, {0}, 0, 0, 0, 0, 0};

void J1939Bus::setup(AppData* appData) {
    _appData = appData;

    // Try 250k first, then 500k
    uint32_t baudRates[] = {250000, 500000};
    for (int i = 0; i < 2; i++) {
        J1939BusCan.begin();
        J1939BusCan.setBaudRate(baudRates[i]);
        J1939BusCan.setMaxMB(16);
        J1939BusCan.enableFIFO();
        J1939BusCan.enableFIFOInterrupt();
        J1939BusCan.onReceive(onReceive);

        CAN_message_t test;
        test.id = 0x18EEFF00;
        test.flags.extended = true;
        test.len = 8;
        memset(test.buf, 0x00, 8);
        test.buf[0] = 0x01;

        delay(100);
        int result = J1939BusCan.write(test);
        delay(50);
        J1939BusCan.events();
        delay(10);
        J1939BusCan.events();
        int result2 = J1939BusCan.write(test);

        Serial.print("J1939 baud scan ");
        Serial.print(baudRates[i]);
        Serial.print(": write=");
        Serial.print(result);
        Serial.print(" second=");
        Serial.println(result2);

        if (result2 == 1) {
            Serial.print("J1939: using ");
            Serial.print(baudRates[i]);
            Serial.println(" bps");
            return;
        }
    }

    Serial.println("J1939: WARNING - no ACK, defaulting to 250k");
    J1939BusCan.begin();
    J1939BusCan.setBaudRate(250000);
    J1939BusCan.setMaxMB(16);
    J1939BusCan.enableFIFO();
    J1939BusCan.enableFIFOInterrupt();
    J1939BusCan.onReceive(onReceive);
}

// PGN 60928 - Address Claim
void J1939Bus::sendAddressClaim() {
    CAN_message_t msg;
    msg.id = 0x18EEFF00;
    msg.flags.extended = true;
    msg.len = 8;
    msg.buf[0] = 0x01;
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x00;
    msg.buf[4] = 0x00;
    msg.buf[5] = 0x00;  // Function = Engine
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;

    int result = J1939BusCan.write(msg);
    Serial.print("J1939 Address Claim write: ");
    Serial.println(result);
}

// PGN 61444 - EEC1 (100ms)
void J1939Bus::broadcastEEC1() {
    if (_appData == nullptr) return;

    CAN_message_t msg;
    msg.id = 0x0CF00400;
    msg.flags.extended = true;
    msg.len = 8;
    memset(msg.buf, 0xFF, 8);

    if (!_appData->engineRpm.isStale(STALE_FAST_MS)) {
        float rpm;
        _appData->engineRpm.read(rpm);
        uint16_t rpmRaw = (uint16_t)(rpm / 0.125f);
        msg.buf[3] = rpmRaw & 0xFF;
        msg.buf[4] = (rpmRaw >> 8) & 0xFF;
    }

    J1939BusCan.write(msg);
}

// PGN 65265 - CCVS (100ms)
void J1939Bus::broadcastCCVS() {
    if (_appData == nullptr) return;

    CAN_message_t msg;
    msg.id = 0x18FEF100;
    msg.flags.extended = true;
    msg.len = 8;
    memset(msg.buf, 0xFF, 8);

    if (!_appData->vehicleSpeed.isStale(STALE_FAST_MS)) {
        float speed;
        _appData->vehicleSpeed.read(speed);
        uint16_t speedRaw = (uint16_t)(speed * 256.0f);
        msg.buf[1] = speedRaw & 0xFF;
        msg.buf[2] = (speedRaw >> 8) & 0xFF;
    }

    J1939BusCan.write(msg);
}

// PGN 65248 - VD (1s)
void J1939Bus::broadcastVD() {
    if (_appData == nullptr) return;

    CAN_message_t msg;
    msg.id = 0x18FEE000;
    msg.flags.extended = true;
    msg.len = 8;
    memset(msg.buf, 0xFF, 8);

    if (!_appData->totalVehicleDistance.isStale(STALE_SLOW_MS)) {
        float dist;
        _appData->totalVehicleDistance.read(dist);
        uint32_t distRaw = (uint32_t)(dist / 0.125f);
        msg.buf[4] = distRaw & 0xFF;
        msg.buf[5] = (distRaw >> 8) & 0xFF;
        msg.buf[6] = (distRaw >> 16) & 0xFF;
        msg.buf[7] = (distRaw >> 24) & 0xFF;
    }

    J1939BusCan.write(msg);
}

// PGN 65253 - EH (1s)
void J1939Bus::broadcastEH() {
    if (_appData == nullptr) return;

    CAN_message_t msg;
    msg.id = 0x18FEE500;
    msg.flags.extended = true;
    msg.len = 8;
    memset(msg.buf, 0xFF, 8);

    if (!_appData->engineTotalHours.isStale(STALE_SLOW_MS)) {
        float hours;
        _appData->engineTotalHours.read(hours);
        uint32_t hoursRaw = (uint32_t)(hours / 0.05f);
        msg.buf[0] = hoursRaw & 0xFF;
        msg.buf[1] = (hoursRaw >> 8) & 0xFF;
        msg.buf[2] = (hoursRaw >> 16) & 0xFF;
        msg.buf[3] = (hoursRaw >> 24) & 0xFF;
    }

    J1939BusCan.write(msg);
}

// Start a TP BAM multi-packet transfer (called from ISR, actual sending in loop)
void J1939Bus::startTpBam(uint32_t pgn, const uint8_t* data, uint8_t len) {
    if (_tp.active) return;  // already sending

    _tp.dataLen = len;
    memcpy(_tp.data, data, len);
    _tp.pgnToSend = pgn;
    _tp.totalPackets = (len + 6) / 7;
    _tp.nextPacket = 0;  // 0 = need to send BAM header first
    _tp.lastPacketTime = 0;
    _tp.active = true;
}

// Call from loop() - sends TP BAM packets with 50ms spacing
void J1939Bus::processTpBam() {
    if (!_tp.active) return;

    uint32_t now = millis();
    if (now - _tp.lastPacketTime < 50) return;  // 50ms inter-packet delay
    _tp.lastPacketTime = now;

    if (_tp.nextPacket == 0) {
        // Send BAM header (TP.CM)
        CAN_message_t bam;
        bam.id = 0x1CECFF00;
        bam.flags.extended = true;
        bam.len = 8;
        bam.buf[0] = 0x20;            // BAM control byte
        bam.buf[1] = _tp.dataLen;     // total size LSB
        bam.buf[2] = 0;               // total size MSB
        bam.buf[3] = _tp.totalPackets;
        bam.buf[4] = 0xFF;            // reserved
        bam.buf[5] = _tp.pgnToSend & 0xFF;
        bam.buf[6] = (_tp.pgnToSend >> 8) & 0xFF;
        bam.buf[7] = (_tp.pgnToSend >> 16) & 0xFF;
        J1939BusCan.write(bam);

        _tp.nextPacket = 1;
        Serial.print("TP BAM start PGN:");
        Serial.print(_tp.pgnToSend);
        Serial.print(" len:");
        Serial.print(_tp.dataLen);
        Serial.print(" pkts:");
        Serial.println(_tp.totalPackets);
    } else {
        // Send data packet
        CAN_message_t dt;
        dt.id = 0x1CEBFF00;
        dt.flags.extended = true;
        dt.len = 8;
        dt.buf[0] = _tp.nextPacket;  // sequence number (1-based)

        for (uint8_t i = 0; i < 7; i++) {
            uint8_t idx = (_tp.nextPacket - 1) * 7 + i;
            dt.buf[1 + i] = (idx < _tp.dataLen) ? _tp.data[idx] : 0xFF;
        }
        J1939BusCan.write(dt);

        if (_tp.nextPacket >= _tp.totalPackets) {
            _tp.active = false;  // done
            Serial.println("TP BAM complete");
        } else {
            _tp.nextPacket++;
        }
    }
}

void J1939Bus::onReceive(const CAN_message_t &msg) {
    J1939Message j1939;
    j1939.setCanId(msg.id);

    // Handle PGN 59904 (Request) - PDU Format 0xEA
    if (j1939.pduFormat == 0xEA && msg.len >= 3) {
        uint32_t requestedPgn = msg.buf[0] | ((uint32_t)msg.buf[1] << 8) | ((uint32_t)msg.buf[2] << 16);

        Serial.print("J1939 REQ PGN:");
        Serial.println(requestedPgn);

        switch (requestedPgn) {
            case 65260: {  // Vehicle Identification (VIN)
                if (_appData != nullptr) {
                    char vin[18];
                    _appData->vin.read(vin, 18);
                    if (vin[0] != '\0') {
                        startTpBam(0x00FEEC, (const uint8_t*)vin, 17);
                    }
                }
                break;
            }
            case 65259: {  // Component Identification
                // Format: Make*Model*SerialNumber*UnitNumber*
                const char* compId = "CUMMINS*CM848D*OCT001*1*";
                startTpBam(0x00FEEB, (const uint8_t*)compId, strlen(compId));
                break;
            }
            case 61444:  broadcastEEC1(); break;
            case 65265:  broadcastCCVS(); break;
            case 65248:  broadcastVD();   break;
            case 65253:  broadcastEH();   break;
            case 60928:  sendAddressClaim(); break;
            default:
                Serial.print("J1939 NACK PGN:");
                Serial.println(requestedPgn);
                break;
        }
        return;
    }

    // Log other J1939 traffic (uncomment for debug)
    // Serial.print("J1939 RX ID:0x");
    // Serial.print(msg.id, HEX);
    // Serial.println();
}

#endif // J1939_BUS_CPP
