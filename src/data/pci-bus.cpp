#include "pci-bus.h"

AppData* PciBus::_appData = nullptr;
volatile uint32_t PciBus::_msgCount = 0;
volatile uint32_t PciBus::_errCount = 0;
volatile uint32_t PciBus::_lastRxTime = 0;

void PciBus::setup(AppData* appData) {
    _appData = appData;

    VPW.onMessageReceived(onMessageReceived);
    VPW.onError(onError);
    VPW.begin(J1850VPW_RX, J1850VPW_TX, ACTIVE_HIGH);

    // Null-terminated filter array (J1850VPWCore iterates with while(*ids))
    uint8_t filter[] = { PCI_MSG_PCM_ENGINE, PCI_MSG_MULTI_SENSOR, PCI_MSG_AMBIENT_TEMP, PCI_MSG_VIN, 0x00 };
    VPW.ignoreAll();
    VPW.listen(filter);
}

void PciBus::onMessageReceived(uint8_t* message, uint8_t messageLength) {
    _msgCount++;
    _lastRxTime = millis();

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

    _errCount++;
    const char* opStr = (op == J1850VPW_Read) ? "RX" : "TX";
    Serial.print("PCI ");
    Serial.print(opStr);
    Serial.print(" ERR: 0x");
    Serial.println(err, HEX);
}
