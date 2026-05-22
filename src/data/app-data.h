#ifndef APP_DATA_H
#define APP_DATA_H

#include <Arduino.h>

struct FloatValue {
    volatile float value;
    const char* unit;
    elapsedMillis sinceUpdate;
    volatile bool hasValue;

    void update(float newValue) {
        noInterrupts();
        value = newValue;
        sinceUpdate = 0;
        hasValue = true;
        interrupts();
    }

    void read(float& outValue) {
        noInterrupts();
        outValue = value;
        interrupts();
    }

    bool isStale(uint32_t maxAgeMs) const {
        return !hasValue || sinceUpdate > maxAgeMs;
    }
};

struct StringValue {
    volatile char value[18];
    const char* unit;
    elapsedMillis sinceUpdate;
    volatile bool hasValue;

    void update(const char* newValue) {
        noInterrupts();
        for (int i = 0; i < 17; i++) {
            value[i] = newValue[i];
            if (newValue[i] == '\0') break;
        }
        value[17] = '\0';
        sinceUpdate = 0;
        hasValue = true;
        interrupts();
    }

    void read(char* outValue, uint8_t maxLen) {
        noInterrupts();
        for (uint8_t i = 0; i < maxLen - 1 && i < 17; i++) {
            outValue[i] = value[i];
            if (value[i] == '\0') break;
        }
        outValue[maxLen - 1] = '\0';
        interrupts();
    }

    bool isStale(uint32_t maxAgeMs) const {
        return !hasValue || sinceUpdate > maxAgeMs;
    }
};

#define J1939_MAX_NODES 16

struct J1939Node {
    uint8_t sourceAddress;
    uint8_t name[8];
    bool    inUse;
    bool    claimed;
    bool    requestSent;
};

struct J1939NodeTable {
    J1939Node nodes[J1939_MAX_NODES];

    J1939Node* findOrAllocate(uint8_t srcAddr) {
        for (uint8_t i = 0; i < J1939_MAX_NODES; i++) {
            if (nodes[i].inUse && nodes[i].sourceAddress == srcAddr) return &nodes[i];
        }
        for (uint8_t i = 0; i < J1939_MAX_NODES; i++) {
            if (!nodes[i].inUse) {
                nodes[i] = {};
                nodes[i].sourceAddress = srcAddr;
                nodes[i].inUse = true;
                return &nodes[i];
            }
        }
        Serial.println("J1939 node table full!");
        return nullptr;
    }

    const J1939Node* findNode(uint8_t srcAddr) const {
        for (uint8_t i = 0; i < J1939_MAX_NODES; i++) {
            if (nodes[i].inUse && nodes[i].sourceAddress == srcAddr) return &nodes[i];
        }
        return nullptr;
    }

    bool isClaimed(uint8_t srcAddr) const {
        const J1939Node* node = findNode(srcAddr);
        return node != nullptr && node->claimed;
    }
};

// Data received from the Cummins ECU via J1939 broadcasts and CLIP protocol
struct EcuData {
    FloatValue engineRpm;            // EEC1 PGN 61444 SPN 190, 100ms
    FloatValue engineTorqueMode;     // EEC1 PGN 61444 SPN 899,  100ms (enum 0-14)
    FloatValue driverDemandTorque;   // EEC1 PGN 61444 SPN 512,  100ms
    FloatValue engineTorque;         // EEC1 PGN 61444 SPN 513+4154, 100ms
    // EEC2 PGN 61443 — byte 0 bit fields
    FloatValue accelPedal1LowIdleSwitch;     // SPN 558,  bits 0-1: 0=not idle, 1=idle
    FloatValue accelPedalKickdownSwitch;     // SPN 559,  bits 2-3: 0=passive, 1=active
    FloatValue roadSpeedLimitStatus;         // SPN 1437, bits 4-5: 0=active, 1=not active
    FloatValue accelPedal2LowIdleSwitch;     // SPN 2970, bits 6-7: 0=not idle, 1=idle
    // EEC2 PGN 61443 — byte fields
    FloatValue appsPercent;     // SPN 91,  byte 1, 0.4%/bit / CLIP, 100ms
    FloatValue engineLoad;      // SPN 92,  byte 2, 1%/bit   / CLIP, 100ms
    // Other PGNs
    FloatValue coolantTemp;     // ET1 PGN 65262 SPN 110, 1s
    FloatValue fuelTemp;        // ET1 PGN 65262 SPN 174, 1s
    FloatValue oilTemp;         // ET1 PGN 65262 SPN 175, 1s
    FloatValue intakeAirTemp;   // ET1 PGN 65262 SPN 52 / IC1 PGN 65270, 500ms-1s
    FloatValue oilPressure;     // EFL/P1 PGN 65263, 500ms
    FloatValue batteryVoltage;  // VEP1 PGN 65271, 1s
    FloatValue ambientTemp;     // AMB PGN 65269, 1s
};

// Data received from the PCI (J1850 VPW) bus
struct PciData {
    FloatValue engineRpm;       // 0x10 PCM Engine
    FloatValue vehicleSpeed;    // 0x10 PCM Engine
    FloatValue coolantTemp;     // 0xC0 Multi-sensor
    FloatValue batteryVoltage;  // 0xC0 Multi-sensor
    FloatValue oilPressure;     // 0xC0 Multi-sensor
    FloatValue intakeAirTemp;   // 0xC0 Multi-sensor
    FloatValue ambientTemp;     // 0xCD Ambient temp
    StringValue vin;            // 0xF0 VIN
};

// Data received from turbo controllers on J1939
struct TurboData {
    FloatValue turboOilTemp;    // ET1 PGN 65262 SPN 176, 1s
};

struct AppData {
    EcuData ecu;
    PciData pci;
    TurboData turbo1;

    // Engine hours are EEPROM-persisted counters, managed by their own classes
    FloatValue engineHoursSinceRebuild;
    FloatValue engineHoursTruckLifetime;

    J1939NodeTable nodeTable;
};

#endif // APP_DATA_H
