#include "j1939-bus.h"
#include "domain/odometer.h"
#include "cummins-bus.h"   // to forward Allison (SA 3) frames onto the ECU/Cummins bus

#define BROADCAST_100MS_INTERVAL 100   // EEC1, CCVS
#define BROADCAST_1S_INTERVAL    1000  // VD, EH
#define TP_BAM_INTERPACKET_MS    50

// TEMP: J1939 bus inventory — remove before truck use.
struct InvEntry { uint32_t id; uint8_t len; uint8_t buf[8]; };
static InvEntry s_inv[64];
static uint8_t s_invCount = 0;
static uint32_t s_sa3Forwarded = 0;   // count of Allison (SA 3) frames relayed to the ECU bus
static elapsedMillis s_invSinceDump;

// TEMP: Allison gear/range decode helpers — remove before truck use.
// Numeric gear (SPN 523/524): 0xFB=Park, 0x7C=R, 0x7D=N, 0x7E..0x83=1..6.
static void printGearLabel(uint8_t b) {
    Serial.print("0x");
    if (b < 0x10) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print("(");
    if (b == 0xFB) Serial.print("P");
    else if (b == 0x7C) Serial.print("R");
    else if (b == 0x7D) Serial.print("N");
    else if (b >= 0x7E && b <= 0x83) Serial.print(b - 0x7D);  // 1..6
    else Serial.print("?");
    Serial.print(")");
}

// Range fields (SPN 162/163) are ASCII — first byte is the P/R/N/D char.
static void printRangeChar(uint8_t b) {
    if (b >= 0x20 && b <= 0x7E) {
        Serial.print("'");
        Serial.print((char)b);
        Serial.print("'");
    } else {
        Serial.print("0x");
        if (b < 0x10) Serial.print("0");
        Serial.print(b, HEX);
    }
}

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> J1939Bus::J1939BusCan;
AppData* J1939Bus::_appData = nullptr;
TpBamState J1939Bus::_tp = {};
elapsedMillis J1939Bus::sinceLastRx;
elapsedMillis J1939Bus::_since100msBroadcast;
elapsedMillis J1939Bus::_since500msBroadcast;
elapsedMillis J1939Bus::_since1sBroadcast;

void J1939Bus::setup(AppData* appData) {
    _appData = appData;
    J1939SourceAddressHandler::setup(appData, &J1939BusCan);

    J1939BusCan.begin();
    J1939BusCan.setBaudRate(250000);
    J1939BusCan.setMaxMB(16);
    J1939BusCan.enableFIFO();
    J1939BusCan.enableFIFOInterrupt();
    J1939BusCan.onReceive(onReceive);

    _since100msBroadcast = 0;
    _since500msBroadcast = 0;
    _since1sBroadcast = 0;
}

// TEMP: decode Allison ETC2 (PGN 61445, SA 3) from the inventory table. Remove before truck use.
void J1939Bus::printAllisonEtc2() {
    for (uint8_t i = 0; i < s_invCount; i++) {
        uint32_t id = s_inv[i].id;
        if ((id & 0xFF) == 3 && ((id >> 8) & 0xFFFF) == 0xF005) {
            Serial.print("ALLISON | ETC2 SelGear=");
            printGearLabel(s_inv[i].buf[0]);
            Serial.print(" CurGear=");
            printGearLabel(s_inv[i].buf[3]);
            Serial.print(" ReqRange=");
            printRangeChar(s_inv[i].buf[4]);
            Serial.print(" CurRange=");
            printRangeChar(s_inv[i].buf[6]);
            // Show what OCT is broadcasting to the cluster (PCI 0x37).
            Serial.print("  -> PRND TX:");
            if (_appData != nullptr && !_appData->transmission.prndDisplayByte.isStale(2000)) {
                float d;
                _appData->transmission.prndDisplayByte.read(d);
                uint8_t b = (uint8_t)d;
                Serial.print(b == 0x01 ? "P" : b == 0x02 ? "R" : b == 0x03 ? "N" : b == 0x05 ? "D" : "?");
            } else {
                Serial.print("BLANK");
            }
            Serial.println();
            return;
        }
    }
    Serial.println("ALLISON | ETC2 not seen");
}

// TEMP: decode Allison DM1 (PGN 65226, SA 3) active fault codes. Remove before truck use.
// DM1 byte layout: [0]=lamp status, [1]=lamp flash, [2-5]=first DTC (SPN+FMI+OC).
void J1939Bus::printAllisonDm1() {
    for (uint8_t i = 0; i < s_invCount; i++) {
        uint32_t id = s_inv[i].id;
        if ((id & 0xFF) == 3 && ((id >> 8) & 0xFFFF) == 0xFECA) {
            const uint8_t* b = s_inv[i].buf;
            uint32_t spn = (uint32_t)b[2] | ((uint32_t)b[3] << 8) | (((uint32_t)(b[4] & 0xE0)) << 11);
            uint8_t fmi = b[4] & 0x1F;
            uint8_t oc  = b[5] & 0x7F;
            Serial.print("ALLISON | DM1 lamps=0x");
            if (b[0] < 0x10) Serial.print("0");
            Serial.print(b[0], HEX);
            if (spn == 0 && fmi == 0) {
                Serial.println(" — no active DTC");
            } else {
                Serial.print(" ACTIVE DTC SPN=");
                Serial.print(spn);
                Serial.print(" FMI=");
                Serial.print(fmi);
                Serial.print(" OC=");
                Serial.println(oc);
            }
            return;
        }
    }
    Serial.println("ALLISON | DM1 not broadcast (TCM reports no active faults)");
}

// TEMP: detect ETC1 (PGN 61442) on the public J1939 bus and decode output shaft
// speed. This is the signal the Cummins ECU needs for cruise; we want to know if
// the Allison TCM already broadcasts it and from which SA. Remove before truck use.
// ETC1 layout: byte1 status bits, bytes 2-3 SPN191 output shaft speed (0.125 rpm/bit,
// little-endian), byte 8 SPN1482 source addr of controlling device.
void J1939Bus::printAllisonEtc1() {
    for (uint8_t i = 0; i < s_invCount; i++) {
        uint32_t id = s_inv[i].id;
        if (((id >> 16) & 0xFF) == 0xF0 && ((id >> 8) & 0xFF) == 0x02) {
            const uint8_t* b = s_inv[i].buf;
            uint16_t shaftRaw = (uint16_t)b[1] | ((uint16_t)b[2] << 8);
            Serial.print("ALLISON | ETC1 seen from SA=0x");
            uint8_t sa = id & 0xFF;
            if (sa < 0x10) Serial.print("0");
            Serial.print(sa, HEX);
            Serial.print(" outputShaftSpeed=");
            if (shaftRaw >= 0xFB00) Serial.print("N/A");
            else { Serial.print((float)shaftRaw * 0.125f, 1); Serial.print("rpm"); }
            Serial.print(" lockup=");
            Serial.print((b[0] >> 2) & 0x03);
            Serial.print(" shiftInProc=");
            Serial.print((b[0] >> 4) & 0x03);
            Serial.print(" ctrlSA=0x");
            if (b[7] < 0x10) Serial.print("0");
            Serial.println(b[7], HEX);
            return;
        }
    }
    Serial.println("ALLISON | ETC1 (PGN 61442) NOT broadcast — ECU has no shaft-speed source");
}

void J1939Bus::loop() {
    J1939BusCan.events();
    processTpBam();

    // TEMP: dump bus inventory every 5s. Remove before truck use.
    if (s_invSinceDump >= 5000) {
        s_invSinceDump = 0;
        Serial.print("=== J1939 inventory (");
        Serial.print(s_invCount);
        Serial.print(" unique frames, SA3->ECU forwarded=");
        Serial.print(s_sa3Forwarded);
        Serial.println(") ===");
        for (uint8_t i = 0; i < s_invCount; i++) {
            uint32_t id = s_inv[i].id;
            uint8_t sa = id & 0xFF;
            uint8_t pf = (id >> 16) & 0xFF;
            uint8_t ps = (id >> 8) & 0xFF;
            uint16_t pgn = (pf >= 0xF0) ? (((uint16_t)pf << 8) | ps) : ((uint16_t)pf << 8);
            Serial.print("  src=");
            Serial.print(sa);
            Serial.print(" PGN=");
            Serial.print(pgn);
            Serial.print(" id=0x");
            Serial.print(id, HEX);
            Serial.print(" data=");
            for (uint8_t b = 0; b < s_inv[i].len; b++) {
                if (s_inv[i].buf[b] < 0x10) Serial.print("0");
                Serial.print(s_inv[i].buf[b], HEX);
                Serial.print(" ");
            }
            Serial.println();
        }
    }

    if (_since100msBroadcast >= BROADCAST_100MS_INTERVAL) {
        _since100msBroadcast = 0;
        broadcast65265();
    }

    if (_since500msBroadcast >= 500) {
        _since500msBroadcast = 0;
        broadcast65270();
        broadcast65263();
    }

    if (_since1sBroadcast >= BROADCAST_1S_INTERVAL) {
        _since1sBroadcast = 0;
        broadcast65262();
        broadcast65248();
        broadcast65217();
        broadcast65253();
    }
}

// PGN 65263 - EFL/P1 (500ms). Composite: ECU oil pressure (primary), turbo oil pressure (fallback).
// SPN 94:  buf[0], lift pump pressure from turbo controller, 4 kPa/bit
// SPN 100: buf[3], oil pressure — ECU if fresh, else turbo controller, 4 kPa/bit
void J1939Bus::broadcast65263() {
    if (_appData == nullptr) return;

    CAN_message_t msg;
    msg.id = 0x18FEEF00;
    msg.flags.extended = true;
    msg.len = 8;
    memset(msg.buf, 0xFF, 8);

    if (!_appData->turbo1.turboLiftPumpPressure.isStale(5000)) {
        float kpa;
        _appData->turbo1.turboLiftPumpPressure.read(kpa);
        msg.buf[0] = (uint8_t)(kpa / 4.0f);
    }

    if (!_appData->ecu.oilPressure.isStale(5000)) {
        float kpa;
        _appData->ecu.oilPressure.read(kpa);
        msg.buf[3] = (uint8_t)(kpa / 4.0f);
    } else if (!_appData->turbo1.turboOilPressure.isStale(5000)) {
        float kpa;
        _appData->turbo1.turboOilPressure.read(kpa);
        msg.buf[3] = (uint8_t)(kpa / 4.0f);
    }

    J1939BusCan.write(msg);
}

// PGN 65270 - IC1 (500ms). Composite: ECU boost/IAT + turbo controller EGT.
// SPN 102: buf[1], boost pressure, 2 kPa/bit
// SPN 105: buf[2], intake air temp, 1°C/bit, -40°C offset
// SPN 173: buf[4-5], turbine inlet EGT, 0.03125°C/bit, -273°C offset
void J1939Bus::broadcast65270() {
    if (_appData == nullptr) return;

    CAN_message_t msg;
    msg.id = 0x18FEF600;
    msg.flags.extended = true;
    msg.len = 8;
    memset(msg.buf, 0xFF, 8);

    if (!_appData->ecu.manifoldPressure.isStale(5000)) {
        float kpa;
        _appData->ecu.manifoldPressure.read(kpa);
        msg.buf[1] = (uint8_t)(kpa / 2.0f);
    }

    if (!_appData->ecu.intakeAirTemp.isStale(5000)) {
        float iat;
        _appData->ecu.intakeAirTemp.read(iat);
        msg.buf[2] = (uint8_t)(iat + 40.0f);
    }

    if (!_appData->turbo1.turboEgt.isStale(5000)) {
        float egt;
        _appData->turbo1.turboEgt.read(egt);
        uint16_t raw = (uint16_t)((egt + 273.0f) * 32.0f);
        msg.buf[4] = raw & 0xFF;
        msg.buf[5] = (raw >> 8) & 0xFF;
    }

    J1939BusCan.write(msg);
}

// PGN 65262 - ET1 (1s).
// SPN 110: buf[0],   coolant temp,    1°C/bit,      -40°C offset
// SPN 176: buf[4-5], turbo oil temp,  0.03125°C/bit, -273°C offset (from turbo1 controller)
void J1939Bus::broadcast65262() {
    if (_appData == nullptr) return;

    CAN_message_t msg;
    msg.id = 0x18FEEE00;
    msg.flags.extended = true;
    msg.len = 8;
    memset(msg.buf, 0xFF, 8);

    if (!_appData->ecu.coolantTemp.isStale(5000)) {
        float clt;
        _appData->ecu.coolantTemp.read(clt);
        msg.buf[0] = (uint8_t)(clt + 40.0f);
    }

    if (!_appData->turbo1.turboOilTemp.isStale(5000)) {
        float tot;
        _appData->turbo1.turboOilTemp.read(tot);
        uint16_t raw = (uint16_t)((tot + 273.0f) / 0.03125f);
        msg.buf[4] = raw & 0xFF;
        msg.buf[5] = (raw >> 8) & 0xFF;
    }

    J1939BusCan.write(msg);
}

// PGN 65265 - CCVS (100ms). Broadcast 0 km/h if PCI data is stale —
// a quiet PCI bus almost always means the vehicle is parked.
void J1939Bus::broadcast65265() {
    if (_appData == nullptr) return;

    CAN_message_t msg;
    msg.id = 0x18FEF100;
    msg.flags.extended = true;
    msg.len = 8;
    memset(msg.buf, 0xFF, 8);

    float speed = 0.0f;
    if (!_appData->pci.vehicleSpeed.isStale(STALE_FAST_MS)) {
        _appData->pci.vehicleSpeed.read(speed);
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
void J1939Bus::broadcast65248() {
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
void J1939Bus::broadcast65217() {
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
void J1939Bus::broadcast65253() {
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
void J1939Bus::broadcast65173() {
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
        } else {
            _tp.nextPacket++;
        }
    }
}

void J1939Bus::onReceive(const CAN_message_t &msg) {
    // TEMP: accumulate bus inventory (dumped from loop()). Remove before truck use.
    {
        bool found = false;
        for (uint8_t i = 0; i < s_invCount; i++) {
            if (s_inv[i].id == msg.id) {
                s_inv[i].len = msg.len;
                memcpy(s_inv[i].buf, msg.buf, 8);
                found = true;
                break;
            }
        }
        if (!found && s_invCount < 64) {
            s_inv[s_invCount].id = msg.id;
            s_inv[s_invCount].len = msg.len;
            memcpy(s_inv[s_invCount].buf, msg.buf, 8);
            s_invCount++;
        }
    }

    // GATEWAY (LOAD-BEARING — do not remove): forward every Allison TCM frame (source
    // address 0x03) straight onto the ECU/Cummins bus, unmodified, the instant it arrives —
    // so the ECU sees the transmission as if it shared the datalink (ETC1/ETC2/etc). This is
    // what makes CRUISE CONTROL engage: post-48RE the ECU had no output-shaft-speed reference
    // on its own bus; delivering the Allison's ETC1 here restores it.
    // Loop-safe: Cm848J1939Receiver only relays src==0x00 back the other way, so no ping-pong.
    // Currently forwards the WHOLE SA3 footprint unfiltered; a future pass may narrow it to the
    // specific PGN(s) cruise needs, but must NOT delete the relay.
    if ((msg.id & 0xFF) == 0x03) {
        CumminsBus::CumminsBusCan.write(msg);
        s_sa3Forwarded++;
    }

    sinceLastRx = 0;
    if (J1939SourceAddressHandler::onReceive(msg)) return;

    J1939Message j1939;
    j1939.setCanId(msg.id);

    // Handle PGN 59904 (Request) - PDU Format 0xEA
    if (j1939.pduFormat == 0xEA && msg.len >= 3) {
        uint32_t requestedPgn = msg.buf[0] | ((uint32_t)msg.buf[1] << 8) | ((uint32_t)msg.buf[2] << 16);

        switch (requestedPgn) {
            case 65260: {  // Vehicle Identification (VIN)
                if (_appData != nullptr) {
                    char vin[18];
                    _appData->pci.vin.read(vin, 18);
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
            case 65262:  broadcast65262();  break;
            case 65265:  broadcast65265();  break;
            case 65270:  broadcast65270();  break;
            case 65263:  broadcast65263();  break;
            case 65248:  broadcast65248();  break;
            case 65217:  broadcast65217(); break;
            case 65253:  broadcast65253();   break;
            case 65173:  broadcast65173();  break;
            case 60928:  J1939SourceAddressHandler::sendOwnClaim(); break;
            default:
                Serial.print("J1939 UNHANDLED REQ PGN:");
                Serial.println(requestedPgn);
                break;
        }
        return;
    }

    // Parse PDU2 broadcast data from other J1939 nodes
    if (j1939.pduFormat >= 0xF0 && msg.len >= 8 && _appData != nullptr) {
        uint8_t src = msg.id & 0xFF;
        uint16_t pgn = ((uint16_t)j1939.pduFormat << 8) | j1939.pduSpecific;
        switch (pgn) {
            case 65262: {
                // ET1 from turbo controller — SPN 176 turbo oil temp, buf[4-5] LE, 0.03125°C/bit, -273°C offset
                if (src != 0x00) {  // ignore our own rebroadcast
                    uint16_t raw = (uint16_t)msg.buf[4] | ((uint16_t)msg.buf[5] << 8);
                    if (raw < 0xFE00u)
                        _appData->turbo1.turboOilTemp.update(raw * 0.03125f - 273.0f);
                }
                break;
            }
            case 65270: {
                // IC1 from turbo controller — SPN 173 turbine inlet EGT, buf[4-5] LE, 0.03125°C/bit, -273°C offset
                if (src != 0x00) {
                    uint16_t raw = (uint16_t)msg.buf[4] | ((uint16_t)msg.buf[5] << 8);
                    if (raw < 0xFE00u)
                        _appData->turbo1.turboEgt.update(raw * 0.03125f - 273.0f);
                }
                break;
            }
            case 65263: {
                // EFL/P1 from turbo controller — SPN 94 lift pump (byte 0), SPN 100 oil pressure (byte 3), 4 kPa/bit
                if (src != 0x00) {
                    if (msg.buf[0] < 0xFEu)
                        _appData->turbo1.turboLiftPumpPressure.update(msg.buf[0] * 4.0f);
                    if (msg.buf[3] < 0xFEu)
                        _appData->turbo1.turboOilPressure.update(msg.buf[3] * 4.0f);
                }
                break;
            }
            case 61445: {
                // Allison ETC2 — SPN 162 Requested Range (ASCII, byte 5 = buf[4]).
                // Map to PCI 0x37 display byte: 'P'/'R'/'N' literal, any digit = D.
                if (src == 3) {
                    uint8_t range = msg.buf[4];
                    uint8_t display = 0;
                    if (range == 'P')      display = 0x01;
                    else if (range == 'R') display = 0x02;
                    else if (range == 'N') display = 0x03;
                    else if (range >= '1' && range <= '6') display = 0x05;  // any forward range = D
                    if (display != 0) _appData->transmission.prndDisplayByte.update((float)display);
                }
                break;
            }
        }
    }

}
