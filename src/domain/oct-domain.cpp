#include "oct-domain.h"
#include <Arduino.h>
#include <EEPROM.h>

#include "data/j1939-bus.h"
#include "data/cummins-bus.h"
#include "data/pci-bus.h"

// EEPROM layout for persisted values
#define EEPROM_MAGIC 0x4F435431  // "OCT1"
#define EEPROM_ADDR_MAGIC    0
#define EEPROM_ADDR_ODO      4   // float, km
#define EEPROM_ADDR_HOURS    8   // float, hours
#define EEPROM_ADDR_TRIP    12   // float, km (reserved for future)

static void loadFromEeprom(AppData& appData) {
    uint32_t magic;
    EEPROM.get(EEPROM_ADDR_MAGIC, magic);
    if (magic != EEPROM_MAGIC) {
        Serial.println("EEPROM: no saved data (first boot)");
        return;
    }

    float odo, hours;
    EEPROM.get(EEPROM_ADDR_ODO, odo);
    EEPROM.get(EEPROM_ADDR_HOURS, hours);

    // Seed AppData with saved values — these will be overwritten
    // once live data arrives from the ECU
    if (odo > 0.0f) {
        appData.totalVehicleDistance.update(odo);
        Serial.print("EEPROM: loaded odo=");
        Serial.print(odo, 1);
        Serial.println("km");
    }
    if (hours > 0.0f) {
        appData.engineTotalHours.update(hours);
        Serial.print("EEPROM: loaded hours=");
        Serial.println(hours, 1);
    }
}

static void saveToEeprom(AppData& appData) {
    float odo, hours;
    appData.totalVehicleDistance.read(odo);
    appData.engineTotalHours.read(hours);

    // Only save if we have real data (not zero)
    if (odo <= 0.0f && hours <= 0.0f) return;

    uint32_t magic = EEPROM_MAGIC;
    EEPROM.put(EEPROM_ADDR_MAGIC, magic);
    EEPROM.put(EEPROM_ADDR_ODO, odo);
    EEPROM.put(EEPROM_ADDR_HOURS, hours);
}

AppData OctDomain::appData = {
    .engineRpm = {0.0f, "RPM", 0},
    .engineTorque = {0.0f, "%", 0},
    .vehicleSpeed = {0.0f, "km/h", 0},
    .totalVehicleDistance = {0.0f, "km", 0},
    .engineTotalHours = {0.0f, "hours", 0},
    .vin = {{'\0'}, "VIN", 0},
    .coolantTemp = {0.0f, "C", 0},
    .batteryVoltage = {0.0f, "V", 0},
    .ambientTemp = {0.0f, "C", 0},
    .engineLoad = {0.0f, "%", 0},
    .oilPressure = {0.0f, "kPa", 0},
    .intakeAirTemp = {0.0f, "C", 0}
};

uint32_t lastDebugPrint = 0;
uint32_t lastFastBroadcast = 0;     // 100ms: EEC1, CCVS
uint32_t lastSlowBroadcast = 0;     // 1s: VD, EH
uint32_t lastCumminsRequest = 0;    // 2s: Cummins PGN requests
uint32_t lastEepromSave = 0;        // 30s: persist to EEPROM

void OctDomain::setup() {
    Serial.begin(115200);
    Serial.println("OCT: starting...");

    // Load persisted values before initializing buses
    // so J1939 broadcasts have real data from the first frame
    loadFromEeprom(appData);

    J1939Bus::setup(&appData);
    Serial.println("OCT: J1939 bus (CAN3/CAN_A pins 30/31) initialized");
    CumminsBus::setup(&appData);
    Serial.println("OCT: Cummins bus (CAN2/CAN_B) initialized");
    PciBus::setup(&appData);
    Serial.println("OCT: PCI bus initialized");
    Serial.println("OCT: sending address claim...");
    J1939Bus::sendAddressClaim();
    Serial.println("OCT: all PGNs broadcasting on J1939");
}

void OctDomain::loop() {
    // Process FlexCAN events (frees TX mailboxes after transmission)
    J1939Bus::J1939BusCan.events();
    CumminsBus::CumminsBusCan.events();

    // Process TP BAM multi-packet responses (50ms between packets)
    J1939Bus::processTpBam();

    uint32_t now = millis();

    // 100ms broadcasts: EEC1, CCVS
    if (now - lastFastBroadcast >= 100) {
        lastFastBroadcast = now;
        J1939Bus::broadcastEEC1();
        J1939Bus::broadcastCCVS();
    }

    // 1s broadcasts: VD, EH
    if (now - lastSlowBroadcast >= 1000) {
        lastSlowBroadcast = now;
        J1939Bus::broadcastVD();
        J1939Bus::broadcastEH();
    }

    // 2s: request PGNs from Cummins ECU
    if (now - lastCumminsRequest >= 2000) {
        lastCumminsRequest = now;
        static uint8_t cumminsReqIdx = 0;
        uint32_t pgns[] = {65248, 65262, 65253, 65265, 65260};
        uint8_t numPgns = sizeof(pgns) / sizeof(pgns[0]);
        CumminsBus::requestPgn(pgns[cumminsReqIdx % numPgns]);
        cumminsReqIdx++;

        // Service 0x4A: read live values from RAM
        // total_vehicle_distance_raw at 0x003fdd6c (4 bytes)
        // Also scan nearby for engine hours
        static uint8_t memReadIdx = 0;
        switch (memReadIdx % 6) {
            case 0: CumminsBus::readMemory(0x003fdd6c, 2); break;  // total_distance hi
            case 1: CumminsBus::readMemory(0x003fdd6e, 2); break;  // total_distance lo
            case 2: CumminsBus::readMemory(0x003fdd70, 2); break;  // trip_distance hi
            case 3: CumminsBus::readMemory(0x003fdd72, 2); break;  // trip_distance lo
            case 4: CumminsBus::readMemory(0x003fdd64, 2); break;  // nearby — maybe hours?
            case 5: CumminsBus::readMemory(0x003fdd68, 2); break;  // nearby — maybe hours?
        }
        memReadIdx++;
    }

    // 30s: persist AppData to EEPROM
    if (now - lastEepromSave >= 30000) {
        lastEepromSave = now;
        saveToEeprom(appData);
    }

    // Debug: print AppData every 2 seconds
    if (now - lastDebugPrint >= 2000) {
        lastDebugPrint = now;

        float battery, ambient, load, oilP, intakeT, odo;
        appData.batteryVoltage.read(battery);
        appData.ambientTemp.read(ambient);
        appData.engineLoad.read(load);
        appData.oilPressure.read(oilP);
        appData.intakeAirTemp.read(intakeT);
        appData.totalVehicleDistance.read(odo);

        Serial.print("Batt:");
        Serial.print(battery, 2);
        Serial.print("V | Ambient:");
        Serial.print(ambient, 1);
        Serial.print("C | Load:");
        Serial.print(load, 1);
        Serial.print("% | Oil:");
        Serial.print(oilP, 0);
        Serial.print("kPa | IAT:");
        Serial.print(intakeT, 1);
        Serial.print("C | Odo:");
        Serial.print(odo, 1);
        Serial.print("km");

        float hours;
        appData.engineTotalHours.read(hours);
        Serial.print(" | Hrs:");
        Serial.print(hours, 1);

        char vin[18];
        appData.vin.read(vin, 18);
        if (vin[0] != '\0') {
            Serial.print(" | VIN:");
            Serial.print(vin);
        }

        // Bus status: GOOD if received a message recently
        Serial.print(" | PCI:");
        Serial.print((millis() - PciBus::_lastRxTime < 2000) ? "GOOD" : "BAD");
        Serial.print(" J1939:");
        Serial.print((millis() - J1939Bus::lastRxTime < 10000) ? "GOOD" : "BAD");
        Serial.print(" CUMMINS:");
        Serial.print((millis() - CumminsBus::lastRxTime < 5000) ? "GOOD" : "BAD");

        Serial.println();
    }
}
