#include "cm848-j1939-receiver.h"

// 0xFF in a single-byte SPN means "not available" per SAE J1939-71.
// 0xFFFF for 16-bit SPNs.
#define SPN_NA8  0xFF
#define SPN_NA16 0xFFFF

bool Cm848J1939Receiver::onReceive(const CAN_message_t& msg, volatile AppData* appData) {
    uint8_t src = msg.id & 0xFF;
    uint8_t pf  = (msg.id >> 16) & 0xFF;
    uint8_t ps  = (msg.id >> 8)  & 0xFF;

    // Only PDU2 broadcast frames from ECU (src=0x00)
    if (src != 0x00 || pf < 0xF0 || msg.len < 8) return false;
    if (appData == nullptr) return true;

    uint16_t pgn = ((uint16_t)pf << 8) | ps;

    switch (pgn) {

        case 61444: {
            // EEC1 — Engine Electronic Control 1 (100ms)
            // SPN 190: Engine Speed, bytes 3-4 LE, 0.125 rpm/bit
            uint16_t rpmRaw = (uint16_t)msg.buf[3] | ((uint16_t)msg.buf[4] << 8);
            if (rpmRaw != SPN_NA16) {
                appData->engineRpm.update(rpmRaw * 0.125f);
            }
            // SPN 513: Actual Engine - Percent Torque, byte 2, 1%/bit, -125% offset
            if (msg.buf[2] != SPN_NA8) {
                appData->engineTorque.update((float)msg.buf[2] - 125.0f);
            }
            break;
        }

        case 61443: {
            // EEC2 — Engine Electronic Control 2 (100ms)
            // SPN 91: Accelerator Pedal Position 1, byte 1, 0.4%/bit
            if (msg.buf[1] != SPN_NA8) {
                appData->appsPercent.update(msg.buf[1] * 0.4f);
            }
            // SPN 92: Engine % Load At Current Speed, byte 2, 1%/bit
            if (msg.buf[2] != SPN_NA8) {
                appData->engineLoad.update((float)msg.buf[2]);
            }
            break;
        }

        case 65262: {
            // ET1 — Engine Temperature 1 (1000ms)
            // SPN 110: Engine Coolant Temperature, byte 0, 1°C/bit, -40°C offset
            if (msg.buf[0] != SPN_NA8) {
                appData->coolantTemp.update((float)msg.buf[0] - 40.0f);
            }
            // SPN 52: Engine Intercooler Temperature, byte 4, 1°C/bit, -40°C offset
            // Closest available proxy for charge-air intake temperature in ET1
            if (msg.buf[4] != SPN_NA8) {
                appData->intakeAirTemp.update((float)msg.buf[4] - 40.0f);
            }
            break;
        }

        case 65270: {
            // IC1 — Inlet/Exhaust Conditions 1 (500ms)
            // SPN 105: Intake Manifold 1 Temperature, byte 3, 1°C/bit, -40°C offset
            if (msg.buf[3] != SPN_NA8) {
                // this should be manifold temp, the sensor lives after the CAC but before the intake valves
                appData->intakeAirTemp.update((float)msg.buf[3] - 40.0f);
            }
            break;
        }

        case 65263: {
            // EFL/P1 — Engine Fluid Level/Pressure 1 (500ms)
            // SPN 100: Engine Oil Pressure, byte 4, 4 kPa/bit
            if (msg.buf[4] != SPN_NA8) {
                appData->oilPressure.update((float)msg.buf[4] * 4.0f);
            }
            break;
        }

        case 65271: {
            // VEP1 — Vehicle Electrical Power 1 (1000ms)
            // SPN 168: Battery Potential, bytes 4-5 LE, 0.05V/bit
            uint16_t raw = (uint16_t)msg.buf[4] | ((uint16_t)msg.buf[5] << 8);
            if (raw != SPN_NA16) {
                appData->batteryVoltage.update(raw * 0.05f);
            }
            break;
        }

        case 65269: {
            // AMB — Ambient Conditions (1000ms)
            // SPN 171: Ambient Air Temperature, bytes 3-4 LE, 0.03125°C/bit, -273.15°C offset
            // Confirmed from firmware: barometric at buf[0], cab temp NA at buf[1-2], ambient at buf[3-4]
            uint16_t raw = (uint16_t)msg.buf[3] | ((uint16_t)msg.buf[4] << 8);
            if (raw != SPN_NA16) {
                // this should be intake, the sensor lives after the air filter but before the turbo.
                appData->ambientTemp.update(raw * 0.03125f - 273.15f);
            }
            break;
        }

        default:
            break;  // Unknown PGN — relay only, no parsing
    }

    return true;
}
