#ifndef PCI_BUS_CPP
#define PCI_BUS_CPP

#include "pci-bus.h"

AppData* PciBus::_appData = nullptr;
volatile uint32_t PciBus::_msgCount = 0;
volatile uint32_t PciBus::_errCount = 0;
volatile bool PciBus::_diagResponseReady = false;
volatile uint8_t PciBus::_diagResponseBuf[12] = {0};
volatile uint8_t PciBus::_diagResponseLen = 0;

void PciBus::setup(AppData* appData) {
    _appData = appData;

    VPW.onMessageReceived(onMessageReceived);
    VPW.onError(onError);
    VPW.begin(J1850VPW_RX, J1850VPW_TX, ACTIVE_HIGH);

    // Null-terminated filter array (J1850VPWCore iterates with while(*ids))
    uint8_t filter[] = { PCI_MSG_PCM_ENGINE, PCI_MSG_DIAG_RESPONSE, PCI_MSG_ODOMETER, PCI_MSG_MULTI_SENSOR, PCI_MSG_AMBIENT_TEMP, PCI_MSG_VIN, 0x00 };
    VPW.ignoreAll();
    VPW.listen(filter);
}

// Send odometer diagnostic request — call from loop(), NOT ISR
// Cycles through different modules and services to find odometer
void PciBus::requestOdometer() {
    static uint8_t attempt = 0;

    switch (attempt % 4) {
        case 0: {
            // ABS module (0x28), service 0x22, PID 0x30, sub 0x01
            uint8_t req[] = { 0x24, 0x28, 0x22, 0x30, 0x01, 0x00 };
            VPW.write(req, 6);
            break;
        }
        case 1: {
            // PCM (0x10), service 0x3C
            uint8_t req[] = { 0x24, 0x10, 0x3C, 0x01, 0x05, 0x00 };
            VPW.write(req, 6);
            break;
        }
        case 2: {
            // MIC/cluster (0x60), service 0x22, ODO/TRIP
            uint8_t req[] = { 0x24, 0x60, 0x22, 0x36, 0x01, 0x00 };
            VPW.write(req, 6);
            break;
        }
        case 3: {
            // Transmission (0x18), service 0x21, TCC mileage
            uint8_t req[] = { 0x24, 0x18, 0x21, 0x1C, 0x01, 0x00 };
            VPW.write(req, 6);
            break;
        }
    }
    attempt++;
}

void PciBus::onMessageReceived(uint8_t* message, uint8_t messageLength) {
    _msgCount++;

    // // DEBUG: uncomment to print raw PCI messages
    // Serial.print("PCI RX [");
    // Serial.print(messageLength);
    // Serial.print("]: ");
    // for (uint8_t i = 0; i < messageLength; i++) {
    //     if (message[i] < 0x10) Serial.print("0");
    //     Serial.print(message[i], HEX);
    //     Serial.print(" ");
    // }
    // Serial.println();

    if (_appData == nullptr) return;

    switch (message[0]) {
        // 0x10: PCM Engine Data — RPM, speed, engine load
        // Confirmed tested in 2004RamPci project
        case PCI_MSG_PCM_ENGINE: {
            if (messageLength < 7) break;

            // Bytes 1-2: Engine RPM (big-endian, /4)
            uint16_t rpmRaw = (message[1] << 8) | message[2];
            _appData->engineRpm.update((float)(rpmRaw / 4));

            // Bytes 3-4: Vehicle speed (big-endian, /128 = km/h)
            uint16_t speedRaw = (message[3] << 8) | message[4];
            float speedKmh = speedRaw / 128.0f;
            _appData->vehicleSpeed.update(speedKmh);

            // Byte 5: Engine load (raw 0-255 → 0-100%)
            float loadPct = (message[5] / 255.0f) * 100.0f;
            _appData->engineLoad.update(loadPct);

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

            // Sub-ID in byte 1 determines VIN segment offset
            // F0 01: positions 0-1   (2 chars)
            // F0 02: positions 1-4   (4 chars, overlaps pos 1)
            // F0 06: positions 5-8   (4 chars)
            // F0 0A: positions 9-12  (4 chars)
            // F0 0E: positions 13-16 (4 chars)
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
                // Build VIN into a local buffer, then update atomically
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

        // 0x72: Odometer broadcast (expected while driving)
        case PCI_MSG_ODOMETER: {
            // Log raw bytes — format unknown, decode once we see it
            Serial.print("PCI ODO [");
            Serial.print(messageLength);
            Serial.print("]: ");
            for (uint8_t i = 0; i < messageLength; i++) {
                if (message[i] < 0x10) Serial.print("0");
                Serial.print(message[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
            break;
        }

        // 0x64: Diagnostic response (echo of 0x24 request)
        case PCI_MSG_DIAG_RESPONSE: {
            // Copy to volatile buffer for loop() to log (not ISR-safe to print here)
            _diagResponseLen = (messageLength < 12) ? messageLength : 12;
            for (uint8_t i = 0; i < _diagResponseLen; i++) {
                _diagResponseBuf[i] = message[i];
            }
            _diagResponseReady = true;

            // Decode PCM odometer response: 64-10-3C-01-XX-XX-XX-XX
            if (messageLength >= 8 && message[1] == 0x10 && message[2] == 0x3C && message[3] == 0x01) {
                // 4-byte odometer at bytes 4-7, big-endian (to be verified in-truck)
                uint32_t rawOdo = ((uint32_t)message[4] << 24) | ((uint32_t)message[5] << 16) |
                                  ((uint32_t)message[6] << 8) | (uint32_t)message[7];
                // DRB: raw is miles, convert to km
                float km = rawOdo * 1.609344f;
                _appData->totalVehicleDistance.update(km);
            }
            break;
        }

        default:
            break;
    }
}

void PciBus::onError(J1850VPW_Operations op, J1850VPW_Errors err) {
    if (err == J1850VPW_OK) return;
    if (err == J1850VPW_ERR_IFR_NOT_SUPPORTED) return; // expected, ignore

    _errCount++;
    const char* opStr = (op == J1850VPW_Read) ? "RX" : "TX";
    Serial.print("PCI ");
    Serial.print(opStr);
    Serial.print(" ERR: 0x");
    Serial.println(err, HEX);
}

#endif // PCI_BUS_CPP
