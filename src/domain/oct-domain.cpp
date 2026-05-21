#include "oct-domain.h"
#include <Arduino.h>

#include "data/j1939-bus.h"
#include "data/j1939-source-address-handler.h"
#include "data/cummins-bus.h"
#include "data/pci-bus.h"
#include "data/cm848-broadcast-controller.h"
#include "odometer.h"
#include "engine-hours-since-rebuild.h"

#include "engine-hours-truck-lifetime.h"
#include "serial-commands.h"

#define DEBUG_PRINT_INTERVAL_MS 2000
#define BUS_HEALTH_PCI_MS       2000
#define BUS_HEALTH_J1939_MS     10000
#define BUS_HEALTH_CUMMINS_MS   5000

AppData OctDomain::appData = {
    .engineRpm = {0.0f, "RPM"},
    .engineTorque = {0.0f, "%"},
    .vehicleSpeed = {0.0f, "km/h"},
    .engineHoursSinceRebuild = {0.0f, "hours"},
    .engineHoursTruckLifetime = {0.0f, "hours"},
    .vin = {{'\0'}, "VIN"},
    .coolantTemp = {0.0f, "C"},
    .batteryVoltage = {0.0f, "V"},
    .ambientTemp = {0.0f, "C"},
    .engineLoad = {0.0f, "%"},
    .appsPercent = {0.0f, "%"},
    .oilPressure = {0.0f, "kPa"},
    .intakeAirTemp = {0.0f, "C"}
};

static elapsedMillis sinceDebugPrint;

static void debugPrint() {
    float rpm, speed, battery, ambient, load, apps, oilP, intakeT, hoursSR, hoursTL;
    OctDomain::appData.engineRpm.read(rpm);
    OctDomain::appData.vehicleSpeed.read(speed);
    OctDomain::appData.batteryVoltage.read(battery);
    OctDomain::appData.ambientTemp.read(ambient);
    OctDomain::appData.engineLoad.read(load);
    OctDomain::appData.appsPercent.read(apps);
    OctDomain::appData.oilPressure.read(oilP);
    OctDomain::appData.intakeAirTemp.read(intakeT);
    OctDomain::appData.engineHoursSinceRebuild.read(hoursSR);
    OctDomain::appData.engineHoursTruckLifetime.read(hoursTL);
    float odo = Odometer::km();

    Serial.print("RPM:");
    Serial.print(rpm, 0);
    Serial.print(" | Spd:");
    Serial.print(speed, 1);
    Serial.print("km/h | Batt:");
    Serial.print(battery, 2);
    Serial.print("V | Ambient:");
    Serial.print(ambient, 1);
    Serial.print("C | Load:");
    Serial.print(load, 1);
    Serial.print("% | APPS:");
    Serial.print(apps, 1);
    Serial.print("% | Oil:");
    Serial.print(oilP, 0);
    Serial.print("kPa | IAT:");
    Serial.print(intakeT, 1);
    Serial.print("C | Odo:");
    Serial.print(odo, 1);
    Serial.print("km | HrsSR:");
    Serial.print(hoursSR, 1);
    Serial.print(" | HrsTL:");
    Serial.print(hoursTL, 1);

    char vin[18];
    OctDomain::appData.vin.read(vin, 18);
    if (vin[0] != '\0') {
        Serial.print(" | VIN:");
        Serial.print(vin);
    }

    Serial.print(" | PCI:");
    Serial.print((PciBus::sinceLastRx < BUS_HEALTH_PCI_MS) ? "GOOD" : "BAD");
    Serial.print(" J1939:");
    Serial.print((J1939Bus::sinceLastRx < BUS_HEALTH_J1939_MS) ? "GOOD" : "BAD");
    Serial.print(" CUMMINS:");
    Serial.print((CumminsBus::sinceLastRx < BUS_HEALTH_CUMMINS_MS) ? "GOOD" : "BAD");

    static const char* BC_STATE_NAMES[] = {"IDLE","WAIT_0A","WAIT_07","WAIT_05","ENABLED","CHECK_PROT"};
    uint8_t bcState = Cm848BroadcastController::state();
    Serial.print(" | BC:");
    Serial.print(bcState < 6 ? BC_STATE_NAMES[bcState] : "?");
    Serial.print(" EEC1:");
    Serial.print(Cm848BroadcastController::msSinceEec1());
    Serial.print("ms");

    Serial.println();
}

void OctDomain::setup() {
    Serial.begin(115200);
    Serial.println("OCT: starting...");

    Odometer::setup();
    EngineHoursSinceRebuild::setup(&appData);
    EngineHoursTruckLifetime::setup(&appData);

    J1939Bus::setup(&appData);
    Serial.println("OCT: J1939 bus (CAN3/CAN_A pins 30/31) initialized");
    CumminsBus::setup(&appData);
    Serial.println("OCT: Cummins bus (CAN2/CAN_B) initialized");
    Cm848BroadcastController::setup();
    PciBus::setup(&appData);
    Serial.println("OCT: PCI bus initialized");



    Serial.println("OCT: sending address claim...");
    J1939SourceAddressHandler::sendOwnClaim();
    Serial.println("OCT: all PGNs broadcasting on J1939");

    SerialCommands::setup();
    sinceDebugPrint = 0;
}

void OctDomain::loop() {
    SerialCommands::loop();
    CumminsBus::loop();
    Cm848BroadcastController::loop();
    Odometer::loop();
    EngineHoursSinceRebuild::loop();
    EngineHoursTruckLifetime::loop();
    J1939Bus::loop();

    if (sinceDebugPrint >= DEBUG_PRINT_INTERVAL_MS) {
        sinceDebugPrint = 0;
        debugPrint();
    }
}
