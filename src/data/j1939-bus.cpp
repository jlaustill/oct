#include "j1939-bus.h"
#include "domain/odometer.h"

#define BROADCAST_100MS_INTERVAL 100   // EEC1, CCVS
#define BROADCAST_1S_INTERVAL    1000  // VD, EH
#define TP_BAM_INTERPACKET_MS    50

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> J1939Bus::J1939BusCan;
AppData* J1939Bus::_appData = nullptr;
TpBamState J1939Bus::_tp = {};
elapsedMillis J1939Bus::sinceLastRx;
elapsedMillis J1939Bus::_since100msBroadcast;
elapsedMillis J1939Bus::_since1sBroadcast;

void J1939Bus::setup(AppData* appData) {
    _appData = appData;

    J1939BusCan.begin();
    J1939BusCan.setBaudRate(250000);
    J1939BusCan.setMaxMB(16);
    J1939BusCan.enableFIFO();
    J1939BusCan.enableFIFOInterrupt();
    J1939BusCan.onReceive(onReceive);

    _since100msBroadcast = 0;
    _since1sBroadcast = 0;
}

void J1939Bus::loop() {
    J1939BusCan.events();
    processTpBam();

    if (_since100msBroadcast >= BROADCAST_100MS_INTERVAL) {
        _since100msBroadcast = 0;
        broadcastEEC1();
        broadcastCCVS();
    }

    if (_since1sBroadcast >= BROADCAST_1S_INTERVAL) {
        _since1sBroadcast = 0;
        broadcastVD();
        broadcastHighResolutionVehicleDistance();
        broadcastEH();
    }
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

// PGN 61444 - EEC1 (100ms). Broadcast 0 RPM if PCI data is stale —
// a quiet PCI bus almost always means key-off / engine-not-running.
void J1939Bus::broadcastEEC1() {
    if (_appData == nullptr) return;

    CAN_message_t msg;
    msg.id = 0x0CF00400;
    msg.flags.extended = true;
    msg.len = 8;
    memset(msg.buf, 0xFF, 8);

    float rpm = 0.0f;
    if (!_appData->engineRpm.isStale(STALE_FAST_MS)) {
        _appData->engineRpm.read(rpm);
    }
    uint16_t rpmRaw = (uint16_t)(rpm / 0.125f);
    msg.buf[3] = rpmRaw & 0xFF;
    msg.buf[4] = (rpmRaw >> 8) & 0xFF;

    J1939BusCan.write(msg);
}

// PGN 65265 - CCVS (100ms). Broadcast 0 km/h if PCI data is stale —
// a quiet PCI bus almost always means the vehicle is parked.
void J1939Bus::broadcastCCVS() {
    if (_appData == nullptr) return;

    CAN_message_t msg;
    msg.id = 0x18FEF100;
    msg.flags.extended = true;
    msg.len = 8;
    memset(msg.buf, 0xFF, 8);

    float speed = 0.0f;
    if (!_appData->vehicleSpeed.isStale(STALE_FAST_MS)) {
        _appData->vehicleSpeed.read(speed);
    }
    uint16_t speedRaw = (uint16_t)(speed * 256.0f);
    msg.buf[1] = speedRaw & 0xFF;
    msg.buf[2] = (speedRaw >> 8) & 0xFF;

    J1939BusCan.write(msg);
}

// PGN 65248 - VD (1s). Source: Odometer (1/8000 mi internal storage).
// SPN 244 Trip — not tracked, send 0xFF.
// SPN 245 Total — convert internal raw counts to J1939 0.125 km/bit:
//   total × (1.609344 km/mi) ÷ 8000 ÷ 0.125 km/LSB
// = total × 1,609,344 ÷ 1,000,000,000  (uint64 intermediate)
void J1939Bus::broadcastVD() {
    CAN_message_t msg;
    msg.id = 0x18FEE000;
    msg.flags.extended = true;
    msg.len = 8;
    memset(msg.buf, 0xFF, 8);

    uint32_t total = Odometer::rawCounts();
    uint32_t j1939Raw = (uint32_t)((uint64_t)total * 1609344ULL / 1000000000ULL);
    msg.buf[4] = j1939Raw & 0xFF;
    msg.buf[5] = (j1939Raw >> 8) & 0xFF;
    msg.buf[6] = (j1939Raw >> 16) & 0xFF;
    msg.buf[7] = (j1939Raw >> 24) & 0xFF;

    J1939BusCan.write(msg);
}

// PGN 65217 - HRVD (1s). Source: Odometer (1/8000 mi internal storage).
// SPN 917: convert internal raw to J1939 5 m/bit:
//   total × (0.000125 mi) × (1609.344 m/mi) ÷ 5 m/LSB
// = total × 1,609,344 ÷ 40,000,000  (uint64 intermediate)
// SPN 918 Trip — not tracked, send 0xFF.
void J1939Bus::broadcastHighResolutionVehicleDistance() {
    CAN_message_t msg;
    msg.id = 0x18FEC100;
    msg.flags.extended = true;
    msg.len = 8;
    memset(msg.buf, 0xFF, 8);

    uint32_t total = Odometer::rawCounts();
    uint32_t hrvdRaw = (uint32_t)((uint64_t)total * 1609344ULL / 40000000ULL);
    msg.buf[0] = hrvdRaw & 0xFF;
    msg.buf[1] = (hrvdRaw >> 8) & 0xFF;
    msg.buf[2] = (hrvdRaw >> 16) & 0xFF;
    msg.buf[3] = (hrvdRaw >> 24) & 0xFF;

    J1939BusCan.write(msg);
}

// PGN 65253 - EH (1s). OCT is source-of-record — always broadcast.
// SPN 247: Engine Total Hours of Operation, 4 bytes, 0.05 h/bit.
//          Per J1939-71 this is the LIFETIME counter, not since-rebuild.
// SPN 249: Engine Total Revolutions — not tracked, send 0xFF.
void J1939Bus::broadcastEH() {
    if (_appData == nullptr) return;

    CAN_message_t msg;
    msg.id = 0x18FEE500;
    msg.flags.extended = true;
    msg.len = 8;
    memset(msg.buf, 0xFF, 8);

    float hours;
    _appData->engineHoursTruckLifetime.read(hours);
    uint32_t hoursRaw = (uint32_t)(hours / 0.05f);
    msg.buf[0] = hoursRaw & 0xFF;
    msg.buf[1] = (hoursRaw >> 8) & 0xFF;
    msg.buf[2] = (hoursRaw >> 16) & 0xFF;
    msg.buf[3] = (hoursRaw >> 24) & 0xFF;

    J1939BusCan.write(msg);
}

// PGN 65173 - RBI (on request only, per J1939-71).
// SPN 1193: Engine Operation Time Since Rebuild, 4 bytes, 0.05 h/bit.
void J1939Bus::broadcastRebuildInformation() {
    if (_appData == nullptr) return;

    CAN_message_t msg;
    msg.id = 0x18FE9500;
    msg.flags.extended = true;
    msg.len = 8;
    memset(msg.buf, 0xFF, 8);

    float hours;
    _appData->engineHoursSinceRebuild.read(hours);
    uint32_t hoursRaw = (uint32_t)(hours / 0.05f);
    msg.buf[0] = hoursRaw & 0xFF;
    msg.buf[1] = (hoursRaw >> 8) & 0xFF;
    msg.buf[2] = (hoursRaw >> 16) & 0xFF;
    msg.buf[3] = (hoursRaw >> 24) & 0xFF;

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
    _tp.sinceLastPacket = TP_BAM_INTERPACKET_MS;  // fire immediately
    _tp.active = true;
}

// Called from loop() — sends TP BAM packets with 50ms spacing.
void J1939Bus::processTpBam() {
    if (!_tp.active) return;
    if (_tp.sinceLastPacket < TP_BAM_INTERPACKET_MS) return;
    _tp.sinceLastPacket = 0;

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
    sinceLastRx = 0;

    J1939Message j1939;
    j1939.setCanId(msg.id);

    // Handle PGN 59904 (Request) - PDU Format 0xEA
    if (j1939.pduFormat == 0xEA && msg.len >= 3) {
        uint32_t requestedPgn = msg.buf[0] | ((uint32_t)msg.buf[1] << 8) | ((uint32_t)msg.buf[2] << 16);

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
                const char* compId = "CUMMINS*CM848D*57185646*1*";
                startTpBam(0x00FEEB, (const uint8_t*)compId, strlen(compId));
                break;
            }
            case 61444:  broadcastEEC1(); break;
            case 65265:  broadcastCCVS(); break;
            case 65248:  broadcastVD();   break;
            case 65217:  broadcastHighResolutionVehicleDistance(); break;
            case 65253:  broadcastEH();   break;
            case 65173:  broadcastRebuildInformation();  break;
            case 60928:  sendAddressClaim(); break;
            default:
                Serial.print("J1939 UNHANDLED REQ PGN:");
                Serial.println(requestedPgn);
                break;
        }
        return;
    }

    // Log other J1939 traffic
    Serial.print("J1939 RX ID:0x");
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
