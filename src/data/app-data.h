#ifndef APP_DATA_H
#define APP_DATA_H

#include <Arduino.h>

struct FloatValue {
    volatile float value;
    const char* unit;
    volatile uint32_t lastUpdated;

    void update(float newValue) {
        noInterrupts();
        value = newValue;
        lastUpdated = millis();
        interrupts();
    }

    float read(float& outValue) {
        noInterrupts();
        outValue = value;
        uint32_t ts = lastUpdated;
        interrupts();
        return ts;
    }

    bool isStale(uint32_t maxAgeMs) const {
        return (millis() - lastUpdated) > maxAgeMs;
    }
};

struct StringValue {
    volatile char value[18];
    const char* unit;
    volatile uint32_t lastUpdated;

    void update(const char* newValue) {
        noInterrupts();
        for (int i = 0; i < 17; i++) {
            value[i] = newValue[i];
            if (newValue[i] == '\0') break;
        }
        value[17] = '\0';
        lastUpdated = millis();
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
        return (millis() - lastUpdated) > maxAgeMs;
    }
};

struct AppData {
    // EEC1 - PGN 61444 (broadcast 100ms)
    FloatValue engineRpm;       // RPM
    FloatValue engineTorque;    // %

    // CCVS - PGN 65265 (broadcast 100ms)
    FloatValue vehicleSpeed;    // km/h

    // VD - PGN 65248 (broadcast 1s)
    FloatValue totalVehicleDistance; // km

    // EH - PGN 65253 (broadcast 1s)
    FloatValue engineTotalHours;    // hours

    // VI - PGN 65260 (on request)
    StringValue vin;

    // ET1 - PGN 65262 (future: broadcast 1s)
    FloatValue coolantTemp;     // degrees C
    FloatValue batteryVoltage;  // volts
    FloatValue ambientTemp;     // degrees C
};

#endif // APP_DATA_H
