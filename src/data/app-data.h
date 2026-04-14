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
    bool    inUse;        // slot is occupied
    bool    claimed;      // NAME received via PGN 60928
    bool    requestSent;  // PGN 59904 already sent — don't send again
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
                return &nodes[i];
            }
        }
        return nullptr;  // table full
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

struct AppData {
    // EEC1 - PGN 61444 (broadcast 100ms)
    FloatValue engineRpm;       // RPM
    FloatValue engineTorque;    // %

    // CCVS - PGN 65265 (broadcast 100ms)
    FloatValue vehicleSpeed;    // km/h

    // VD - PGN 65248 (broadcast 1s) — owned by Odometer class, not stored here

    // EH - PGN 65253 (broadcast 1s) — resets on engine rebuild
    FloatValue engineHoursSinceRebuild;  // hours
    // Lifetime counter — never reset
    FloatValue engineHoursTruckLifetime; // hours

    // VI - PGN 65260 (on request)
    StringValue vin;

    // ET1 - PGN 65262 (future: broadcast 1s)
    FloatValue coolantTemp;     // degrees C
    FloatValue batteryVoltage;  // volts
    FloatValue ambientTemp;     // degrees C

    // Additional PCI data
    FloatValue engineLoad;      // %
    FloatValue oilPressure;     // kPa
    FloatValue intakeAirTemp;   // degrees C

    // J1939 node address claim table
    J1939NodeTable nodeTable;
};

#endif // APP_DATA_H
