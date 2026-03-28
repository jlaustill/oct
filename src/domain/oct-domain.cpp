#ifndef OCT_DOMAIN_CPP
#define OCT_DOMAIN_CPP

#include "oct-domain.h"
#include <Arduino.h>

#include "data/j1939-bus.h"
#include "data/cummins-bus.h"
#include "data/pci-bus.h"

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
uint32_t lastCumminsRequest = 0;    // 2s: cycle through Cummins PGN requests

void OctDomain::setup() {
    Serial.begin(115200);
    Serial.println("OCT: starting...");
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

    // 2s: request PGNs from Cummins ECU (CM848D only responds to requests)
    if (now - lastCumminsRequest >= 2000) {
        lastCumminsRequest = now;
        static uint8_t cumminsReqIdx = 0;
        uint32_t pgns[] = {65248, 65262, 65253, 65265, 65260};
        uint8_t numPgns = sizeof(pgns) / sizeof(pgns[0]);
        CumminsBus::requestPgn(pgns[cumminsReqIdx % numPgns]);
        cumminsReqIdx++;

        // Also probe PCI for odometer from ABS/PCM/MIC/TCM
        PciBus::requestOdometer();
    }

    // Log diagnostic responses from PCI (printed from loop, not ISR)
    if (PciBus::_diagResponseReady) {
        PciBus::_diagResponseReady = false;
        Serial.print("PCI DIAG [");
        Serial.print(PciBus::_diagResponseLen);
        Serial.print("]: ");
        for (uint8_t i = 0; i < PciBus::_diagResponseLen; i++) {
            if (PciBus::_diagResponseBuf[i] < 0x10) Serial.print("0");
            Serial.print(PciBus::_diagResponseBuf[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    // Debug: print AppData every 2 seconds
    if (now - lastDebugPrint >= 2000) {
        lastDebugPrint = now;

        float battery, ambient;
        appData.batteryVoltage.read(battery);
        appData.ambientTemp.read(ambient);

        Serial.print("Batt:");
        Serial.print(battery, 2);
        Serial.print("V | Ambient:");
        Serial.print(ambient, 1);
        Serial.print("C");

        float load, oilP, intakeT, odo;
        appData.engineLoad.read(load);
        appData.oilPressure.read(oilP);
        appData.intakeAirTemp.read(intakeT);
        appData.totalVehicleDistance.read(odo);

        Serial.print(" | Load:");
        Serial.print(load, 1);
        Serial.print("% | Oil:");
        Serial.print(oilP, 0);
        Serial.print("kPa | IAT:");
        Serial.print(intakeT, 1);
        Serial.print("C | Odo:");
        Serial.print(odo, 1);
        Serial.print("km");

        char vin[18];
        appData.vin.read(vin, 18);
        if (vin[0] != '\0') {
            Serial.print(" | VIN:");
            Serial.print(vin);
        }

        Serial.print(" | PCI:");
        Serial.print(PciBus::_msgCount);

        Serial.println();
    }
}

#endif // OCT_DOMAIN_CPP
