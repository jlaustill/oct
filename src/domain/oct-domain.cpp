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

#define DEBUG_PRINT_INTERVAL_MS 1000
#define BUS_HEALTH_PCI_MS       2000
#define BUS_HEALTH_J1939_MS     10000
#define BUS_HEALTH_CUMMINS_MS   5000

AppData OctDomain::appData = {
    .ecu = {
        .engineRpm                     = {0.0f, "RPM"},
        .engineTorqueMode              = {0.0f, ""},
        .driverDemandTorque            = {0.0f, "%"},
        .engineTorque                  = {0.0f, "%"},
        .accelPedal1LowIdleSwitch      = {0.0f, ""},
        .accelPedalKickdownSwitch      = {0.0f, ""},
        .roadSpeedLimitStatus          = {0.0f, ""},
        .accelPedal2LowIdleSwitch      = {0.0f, ""},
        .appsPercent                   = {0.0f, "%"},
        .engineLoad                    = {0.0f, "%"},
        .coolantTemp                   = {0.0f, "C"},
        .fuelTemp                      = {0.0f, "C"},
        .oilTemp                       = {0.0f, "C"},
        .boostPressure                 = {0.0f, "kPa"},
        .intakeAirTemp                 = {0.0f, "C"},
        .oilPressure                   = {0.0f, "kPa"},
        .batteryPotential              = {0.0f, "V"},
        .ambientTemp                   = {0.0f, "C"},
    },
    .pci = {
        .engineRpm      = {0.0f, "RPM"},
        .vehicleSpeed   = {0.0f, "km/h"},
        .coolantTemp    = {0.0f, "C"},
        .batteryVoltage = {0.0f, "V"},
        .oilPressure    = {0.0f, "kPa"},
        .intakeAirTemp  = {0.0f, "C"},
        .ambientTemp    = {0.0f, "C"},
        .vin            = {{'\0'}, "VIN"},
    },
    .turbo1 = {
        .turboOilTemp            = {0.0f, "C"},
        .turboEgt                = {0.0f, "C"},
        .turboLiftPumpPressure   = {0.0f, "kPa"},
        .turboOilPressure        = {0.0f, "kPa"},
    },
    .engineHoursSinceRebuild  = {0.0f, "hours"},
    .engineHoursTruckLifetime = {0.0f, "hours"},
};

static elapsedMillis sinceDebugPrint;

// Print a FloatValue with staleness marker. Shows "--" if never received,
// the value with "?" if stale, or just the value if fresh.
static void printVal(FloatValue& fv, uint32_t staleMs, int decimals) {
    if (!fv.hasValue) {
        Serial.print("--");
        return;
    }
    float v;
    fv.read(v);
    Serial.print(v, decimals);
    if (fv.isStale(staleMs)) Serial.print("?");
}

static void debugPrint() {
    Serial.print("ECU | RPM:");
    printVal(OctDomain::appData.ecu.engineRpm, 500, 0);
    Serial.print(" TM:");
    printVal(OctDomain::appData.ecu.engineTorqueMode, 500, 0);
    Serial.print(" DDTrq:");
    printVal(OctDomain::appData.ecu.driverDemandTorque, 500, 0);
    Serial.print("% Trq:");
    printVal(OctDomain::appData.ecu.engineTorque, 500, 0);
    Serial.print("% APPS:");
    printVal(OctDomain::appData.ecu.appsPercent, 500, 1);
    Serial.print("% Ld:");
    printVal(OctDomain::appData.ecu.engineLoad, 500, 1);
    Serial.print("% P1Li:");
    printVal(OctDomain::appData.ecu.accelPedal1LowIdleSwitch, 500, 0);
    Serial.print(" KD:");
    printVal(OctDomain::appData.ecu.accelPedalKickdownSwitch, 500, 0);
    Serial.print(" RSL:");
    printVal(OctDomain::appData.ecu.roadSpeedLimitStatus, 500, 0);
    Serial.print(" P2Li:");
    printVal(OctDomain::appData.ecu.accelPedal2LowIdleSwitch, 500, 0);
    Serial.print(" | CLT:");
    printVal(OctDomain::appData.ecu.coolantTemp, 3000, 1);
    Serial.print("C Fuel:");
    printVal(OctDomain::appData.ecu.fuelTemp, 3000, 1);
    Serial.print("C OilT:");
    printVal(OctDomain::appData.ecu.oilTemp, 3000, 1);
    Serial.print("C TrbOil:");
    printVal(OctDomain::appData.turbo1.turboOilTemp, 3000, 1);
    Serial.print("C EGT:");
    printVal(OctDomain::appData.turbo1.turboEgt, 3000, 0);
    Serial.print("C LftP:");
    printVal(OctDomain::appData.turbo1.turboLiftPumpPressure, 3000, 0);
    Serial.print("kPa TrbOilP:");
    printVal(OctDomain::appData.turbo1.turboOilPressure, 3000, 0);
    Serial.print("kPa IAT:");
    printVal(OctDomain::appData.ecu.intakeAirTemp, 3000, 1);
    Serial.print("C Bst:");
    printVal(OctDomain::appData.ecu.boostPressure, 3000, 0);
    Serial.print("kPa | OilP:");
    printVal(OctDomain::appData.ecu.oilPressure, 3000, 0);
    Serial.print("kPa BattP:");
    printVal(OctDomain::appData.ecu.batteryPotential, 3000, 2);
    Serial.print("V Amb:");
    printVal(OctDomain::appData.ecu.ambientTemp, 3000, 1);
    Serial.println("C");

    float pRpm, spd, pClt, pBatt, pOil, pIat, pAmb;
    OctDomain::appData.pci.engineRpm.read(pRpm);
    OctDomain::appData.pci.vehicleSpeed.read(spd);
    OctDomain::appData.pci.coolantTemp.read(pClt);
    OctDomain::appData.pci.batteryVoltage.read(pBatt);
    OctDomain::appData.pci.oilPressure.read(pOil);
    OctDomain::appData.pci.intakeAirTemp.read(pIat);
    OctDomain::appData.pci.ambientTemp.read(pAmb);

    Serial.print("PCI | RPM:");
    Serial.print(pRpm, 0);
    Serial.print(" Spd:");
    Serial.print(spd, 1);
    Serial.print("km/h | CLT:");
    Serial.print(pClt, 1);
    Serial.print("C IAT:");
    Serial.print(pIat, 1);
    Serial.print("C Batt:");
    Serial.print(pBatt, 2);
    Serial.print("V Oil:");
    Serial.print(pOil, 0);
    Serial.print("kPa Amb:");
    Serial.print(pAmb, 1);
    Serial.print("C");
    char vin[18];
    OctDomain::appData.pci.vin.read(vin, 18);
    if (vin[0] != '\0') {
        Serial.print(" VIN:");
        Serial.print(vin);
    }
    Serial.println();

    float hoursSR, hoursTL;
    OctDomain::appData.engineHoursSinceRebuild.read(hoursSR);
    OctDomain::appData.engineHoursTruckLifetime.read(hoursTL);
    float odo = Odometer::km();

    static const char* BC_STATE_NAMES[] = {"IDLE","WAIT_0A","WAIT_07","WAIT_05","ENABLED","CHECK_PROT"};
    uint8_t bcState = Cm848BroadcastController::state();

    Serial.print("SYS | PCI:");
    Serial.print((PciBus::sinceLastRx < BUS_HEALTH_PCI_MS) ? "GOOD" : "BAD");
    Serial.print(" J1939:");
    Serial.print((J1939Bus::sinceLastRx < BUS_HEALTH_J1939_MS) ? "GOOD" : "BAD");
    Serial.print(" CUMMINS:");
    Serial.print((CumminsBus::sinceLastRx < BUS_HEALTH_CUMMINS_MS) ? "GOOD" : "BAD");
    Serial.print(" BC:");
    Serial.print(bcState < 6 ? BC_STATE_NAMES[bcState] : "?");
    Serial.print(" EEC1:");
    Serial.print(Cm848BroadcastController::msSinceEec1());
    Serial.print("ms | Odo:");
    Serial.print(odo, 1);
    Serial.print("km HrsSR:");
    Serial.print(hoursSR, 1);
    Serial.print(" HrsTL:");
    Serial.print(hoursTL, 1);
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
