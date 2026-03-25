#ifndef PCI_BUS_CPP
#define PCI_BUS_CPP

#include "pci-bus.h"

AppData* PciBus::_appData = nullptr;
volatile uint32_t PciBus::_msgCount = 0;
volatile uint32_t PciBus::_errCount = 0;

void PciBus::setup(AppData* appData) {
    _appData = appData;

    VPW.onMessageReceived(onMessageReceived);
    VPW.onError(onError);
    VPW.begin(J1850VPW_RX, J1850VPW_TX, ACTIVE_HIGH);

    // Only listen for messages we can decode
    uint8_t filter[] = { PCI_MSG_PCM_ENGINE, PCI_MSG_MULTI_SENSOR, PCI_MSG_AMBIENT_TEMP, PCI_MSG_VIN };
    VPW.ignoreAll();
    VPW.listen(filter);
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

        default:
            break;
    }
}

void PciBus::onError(J1850VPW_Operations op, J1850VPW_Errors err) {
    if (err == J1850VPW_OK) return;

    const char* opStr = (op == J1850VPW_Read) ? "RX" : "TX";
    Serial.print("PCI ");
    Serial.print(opStr);
    Serial.print(" ERR: 0x");
    Serial.println(err, HEX);
}

#endif // PCI_BUS_CPP
