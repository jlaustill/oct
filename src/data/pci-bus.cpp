#include "pci-bus.h"
#include "domain/odometer.h"

AppData* PciBus::_appData = nullptr;
volatile uint32_t PciBus::msgCount = 0;
volatile uint32_t PciBus::errCount = 0;
elapsedMillis PciBus::sinceLastRx;
bool PciBus::_rawLogActive = false;
elapsedMillis PciBus::_rawLogTimer;
uint32_t PciBus::_rawLogDurationMs = 0;

void PciBus::setup(AppData* appData) {
    _appData = appData;

    _rawLogActive = false;

    VPW.onMessageReceived(onMessageReceived);
    VPW.onError(onError);
    VPW.begin(J1850VPW_RX, J1850VPW_TX, ACTIVE_HIGH);

    // DEBUG: listen to ALL — engine running capture
    // uint8_t filter[] = { PCI_MSG_PCM_ENGINE, PCI_MSG_MULTI_SENSOR, PCI_MSG_AMBIENT_TEMP, PCI_MSG_VIN, 0x00 };
    // VPW.ignoreAll();
    // VPW.listen(filter);
}

void PciBus::sendShiftLever(uint8_t gearByte) {
    uint8_t msg[] = {PCI_MSG_SHIFT_LEVER, gearByte, 0x00};
    VPW.write(msg, sizeof(msg));
}

void PciBus::startRawLog(uint32_t durationMs) {
    _rawLogDurationMs = durationMs;
    _rawLogTimer = 0;
    _rawLogActive = true;
    Serial.print("PRND capture active for ");
    Serial.print(durationMs / 1000);
    Serial.println("s — shift P/R/N/D/1/2 slowly, one line per change");
    Serial.println("  0x37 = cluster display  |  0x3A = engaged gear");
}

static void printGearMsg(const char* label, uint8_t* message, uint8_t messageLength) {
    Serial.print(label);
    Serial.print(" len=");
    Serial.print(messageLength);
    for (uint8_t i = 1; i < messageLength - 1; i++) {
        Serial.print("  b");
        Serial.print(i);
        Serial.print("=0x");
        if (message[i] < 0x10) Serial.print("0");
        Serial.print(message[i], HEX);
    }
    Serial.println();
}

static bool gearMsgChanged(uint8_t* last, uint8_t* lastLen, uint8_t* message, uint8_t messageLength) {
    uint8_t dataLen = (messageLength > 1) ? messageLength - 1 : 0;
    bool changed = (dataLen != *lastLen);
    for (uint8_t i = 0; i < dataLen && i < 8; i++) {
        if (message[i] != last[i]) { changed = true; }
    }
    if (changed) {
        *lastLen = dataLen;
        for (uint8_t i = 0; i < dataLen && i < 8; i++) last[i] = message[i];
    }
    return changed;
}

void PciBus::onMessageReceived(uint8_t* message, uint8_t messageLength) {
    msgCount++;
    sinceLastRx = 0;

    if (_rawLogActive) {
        if (_rawLogTimer >= _rawLogDurationMs) {
            _rawLogActive = false;
            Serial.println("PRND capture done");
        } else if (message[0] == PCI_MSG_SHIFT_LEVER) {
            static uint8_t last[8]; static uint8_t lastLen = 0;
            if (gearMsgChanged(last, &lastLen, message, messageLength))
                printGearMsg("0x37", message, messageLength);
        } else if (message[0] == PCI_MSG_TRANS_GEAR) {
            static uint8_t last[8]; static uint8_t lastLen = 0;
            if (gearMsgChanged(last, &lastLen, message, messageLength))
                printGearMsg("0x3A", message, messageLength);
        }
    }

    // DEBUG: log unknown messages (re-enable if diagnosing a new PCI ID)
    // uint8_t id = message[0];
    // if (id != 0x10 && id != 0x35 && id != 0x1E && id != 0xB0 && id != 0xB1 && id != 0xD1) {
    //     Serial.print("PCI ");
    //     if (id < 0x10) Serial.print("0");
    //     Serial.print(id, HEX);
    //     Serial.print(": ");
    //     for (uint8_t i = 1; i < messageLength; i++) {
    //         if (message[i] < 0x10) Serial.print("0");
    //         Serial.print(message[i], HEX);
    //         Serial.print(" ");
    //     }
    //     Serial.println();
    // }

    if (_appData == nullptr) return;

    switch (message[0]) {
        // 0x10: PCM Engine Data — RPM, speed, engine load
        case PCI_MSG_PCM_ENGINE: {
            if (messageLength < 7) break;

            // Bytes 1-2: Engine RPM (big-endian, /4)
            uint16_t rpmRaw = (message[1] << 8) | message[2];
            _appData->engineRpm.update((float)(rpmRaw / 4));

            // Bytes 3-4: Vehicle speed (big-endian, /128 = km/h)
            uint16_t speedRaw = (message[3] << 8) | message[4];
            float speedKmh = speedRaw / 128.0f;
            _appData->vehicleSpeed.update(speedKmh);

            // Byte 5: Engine load — inaccurate; sourced from Cummins CLIP instead
            // float loadPct = (message[5] / 255.0f) * 100.0f;
            // _appData->engineLoad.update(loadPct);

            break;
        }

        // 0x5D: ECM Distance/Fuel (Chry_Dat5D).
        // byte 0: VSS pulses since last $5D (1 pulse = 1/8000 mi)
        // bytes 1-2: injector on-time (u16 BE) — unused here
        // bytes 3-4: fuel delivered (u16 BE) — unused here
        case PCI_MSG_DISTANCE_PULSES: {
            if (messageLength < 2) break;  // header + at least pulse byte
            uint8_t pulses = message[1];
            Odometer::onPulses(pulses);
            break;
        }

        // 0xC0: Multi-sensor — battery, oil, coolant, intake air temp
        case PCI_MSG_MULTI_SENSOR: {
            if (messageLength < 5) break;

            // Byte 3: Coolant temp (Celsius, offset -40)
            float coolantC = message[3] - 40.0f;
            _appData->coolantTemp.update(coolantC);

            // Byte 1: Battery voltage (0.0625V/bit)
            float batteryV = message[1] * 0.0625f;
            _appData->batteryVoltage.update(batteryV);

            // Byte 2: Oil pressure (0.5 PSI/bit → kPa)
            float oilKpa = message[2] * 0.5f * 6.895f;
            _appData->oilPressure.update(oilKpa);

            // Byte 4: Intake air temp (Celsius, offset -40)
            float intakeC = message[4] - 40.0f;
            _appData->intakeAirTemp.update(intakeC);

            break;
        }

        // 0xCD: Outside air temperature
        case PCI_MSG_AMBIENT_TEMP: {
            if (messageLength < 3) break;

            // Byte 1: Ambient temp (Celsius, offset -70)
            float ambientC = message[1] - 70.0f;
            _appData->ambientTemp.update(ambientC);

            break;
        }

        // 0xF0: Vehicle Identification Number (5 segments)
        case PCI_MSG_VIN: {
            if (messageLength < 4) break;

            uint8_t subId = message[1];
            int offset = -1;
            int count = 0;

            switch (subId) {
                case 0x01: offset = 0;  count = 2; break;
                case 0x02: offset = 1;  count = 4; break;
                case 0x06: offset = 5;  count = 4; break;
                case 0x0A: offset = 9;  count = 4; break;
                case 0x0E: offset = 13; count = 4; break;
            }

            if (offset >= 0 && messageLength >= (uint8_t)(2 + count)) {
                char vin[18];
                _appData->vin.read(vin, 18);
                for (int i = 0; i < count && (offset + i) < 17; i++) {
                    vin[offset + i] = (char)message[2 + i];
                }
                vin[17] = '\0';
                _appData->vin.update(vin);
            }
            break;
        }

        default:
            break;
    }
}

void PciBus::onError(J1850VPW_Operations op, J1850VPW_Errors err) {
    if (err == J1850VPW_OK) return;
    if (err == J1850VPW_ERR_IFR_NOT_SUPPORTED) return;

    errCount++;
    const char* opStr = (op == J1850VPW_Read) ? "RX" : "TX";
    // Serial.print("PCI ");
    // Serial.print(opStr);
    // Serial.print(" ERR: 0x");
    // Serial.println(err, HEX);
}
