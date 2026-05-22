#include "cm848-j1939-receiver.h"

// J1939-71 validity guards: top two values of each SPN range are reserved.
// 1-byte: 0xFF = not available, 0xFE = error indicator — reject both (< 0xFE = valid).
// 2-byte: 0xFFFF = not available, 0xFExx = error indicator — reject both (< 0xFE00 = valid).
#define SPN_VALID2(v)  ((v) < 2u)
#define SPN_VALID8(v)  ((v) < 0xFEu)
#define SPN_VALID16(v) ((v) < 0xFE00u)

bool Cm848J1939Receiver::onReceive(const CAN_message_t &msg, volatile AppData *appData)
{
    uint8_t src = msg.id & 0xFF;
    uint8_t pf = (msg.id >> 16) & 0xFF;
    uint8_t ps = (msg.id >> 8) & 0xFF;

    // Only PDU2 broadcast frames from ECU (src=0x00)
    if (src != 0x00 || pf < 0xF0 || msg.len < 8)
        return false;
    if (appData == nullptr)
        return true;

    uint16_t pgn = ((uint16_t)pf << 8) | ps;

    switch (pgn)
    {

    case 61444:
    {
        // EEC1 — Engine Electronic Control 1 (100ms)
        // Byte 0 bits 0-3: SPN 899 Engine Torque Mode (enum 0-14, 15=NA)
        uint8_t torqueMode = msg.buf[0] & 0x0F;
        if (torqueMode < 15u)
            appData->ecu.engineTorqueMode.update((float)torqueMode);
        // SPN 512: Driver's Demand Engine - Percent Torque, byte 1, 1%/bit, -125% offset
        if (SPN_VALID8(msg.buf[1]))
            appData->ecu.driverDemandTorque.update((float)msg.buf[1] - 125.0f);
        // SPN 513+4154: Actual Engine - Percent Torque + high-res supplement
        // byte 2: 1%/bit, -125% offset; byte 0 bits 4-7: 0.125%/bit additive (valid if < 8)
        if (SPN_VALID8(msg.buf[2]))
        {
            float torque = (float)msg.buf[2] - 125.0f;
            uint8_t hiRes = (msg.buf[0] >> 4) & 0x0F;
            if (hiRes < 8u) torque += hiRes * 0.125f;
            appData->ecu.engineTorque.update(torque);
        }
        // SPN 190: Engine Speed, bytes 3-4 LE, 0.125 rpm/bit
        uint16_t rpmRaw = (uint16_t)msg.buf[3] | ((uint16_t)msg.buf[4] << 8);
        if (SPN_VALID16(rpmRaw))
            appData->ecu.engineRpm.update(rpmRaw * 0.125f);
        break;
    }

    case 61443: {
        // EEC2 — Engine Electronic Control 2 (50ms preferred / 100ms)
        // Byte 0: SPN 558/559/1437/2970 — 4 x 2-bit switch/status fields
        uint8_t b0 = msg.buf[0];
        uint8_t sw558  = (b0 >> 0) & 0x03;
        uint8_t sw559  = (b0 >> 2) & 0x03;
        uint8_t sw1437 = (b0 >> 4) & 0x03;
        uint8_t sw2970 = (b0 >> 6) & 0x03;
        if (SPN_VALID2(sw558))  appData->ecu.accelPedal1LowIdleSwitch.update((float)sw558);
        if (SPN_VALID2(sw559))  appData->ecu.accelPedalKickdownSwitch.update((float)sw559);
        if (SPN_VALID2(sw1437)) appData->ecu.roadSpeedLimitStatus.update((float)sw1437);
        if (SPN_VALID2(sw2970)) appData->ecu.accelPedal2LowIdleSwitch.update((float)sw2970);
        // SPN 91: Accelerator Pedal Position 1, byte 1, 0.4%/bit
        if (SPN_VALID8(msg.buf[1]))
            appData->ecu.appsPercent.update(msg.buf[1] * 0.4f);
        // SPN 92: Engine Percent Load At Current Speed, byte 2, 1%/bit
        if (SPN_VALID8(msg.buf[2]))
            appData->ecu.engineLoad.update((float)msg.buf[2]);
        break;
    }

    case 65262:
    {
        // ET1 — Engine Temperature 1 (1000ms)
        // buf[0]:   SPN 110  Coolant Temp,     1°C/bit,       -40°C offset
        // buf[1]:   SPN 174  Fuel Temp,         1°C/bit,       -40°C offset
        // buf[2-3]: SPN 175  Oil Temp,          0.03125°C/bit, -273°C offset (2-byte LE)
        // buf[4-5]: SPN 176  Turbo Oil Temp,    0.03125°C/bit, -273°C offset (2-byte LE)
        // buf[6]:   SPN 52   Intercooler Temp,  1°C/bit,       -40°C offset
        // buf[7]:   SPN 1134 Intercooler Thermostat Opening
        if (SPN_VALID8(msg.buf[0]))
        {
            appData->ecu.coolantTemp.update((float)msg.buf[0] - 40.0f);
        }
        return false;
        break;
    }

    case 65270:
    {
        // IC1 — Inlet/Exhaust Conditions 1 (500ms)
        // SPN 105: Intake Manifold 1 Temperature, byte 3, 1°C/bit, -40°C offset
        if (SPN_VALID8(msg.buf[3]))
        {
            // manifold temp sensor lives after the CAC but before the intake valves
            appData->ecu.intakeAirTemp.update((float)msg.buf[3] - 40.0f);
        }
        break;
    }

    case 65263:
    {
        // EFL/P1 — Engine Fluid Level/Pressure 1 (500ms)
        // SPN 100: Engine Oil Pressure, byte 4, 4 kPa/bit
        if (SPN_VALID8(msg.buf[4]))
        {
            appData->ecu.oilPressure.update((float)msg.buf[4] * 4.0f);
        }
        break;
    }

    case 65271:
    {
        // VEP1 — Vehicle Electrical Power 1 (1000ms)
        // SPN 168: Battery Potential, bytes 4-5 LE, 0.05V/bit
        uint16_t raw = (uint16_t)msg.buf[4] | ((uint16_t)msg.buf[5] << 8);
        if (SPN_VALID16(raw))
        {
            appData->ecu.batteryVoltage.update(raw * 0.05f);
        }
        break;
    }

    case 65269:
    {
        // AMB — Ambient Conditions (1000ms)
        // SPN 171: Ambient Air Temperature, bytes 3-4 LE, 0.03125°C/bit, -273.15°C offset
        // Confirmed from firmware: barometric at buf[0], cab temp NA at buf[1-2], ambient at buf[3-4]
        uint16_t raw = (uint16_t)msg.buf[3] | ((uint16_t)msg.buf[4] << 8);
        if (SPN_VALID16(raw))
        {
            // sensor lives after the air filter but before the turbo
            appData->ecu.ambientTemp.update(raw * 0.03125f - 273.15f);
        }
        break;
    }

    default:
        break; // Unknown PGN — relay only, no parsing
    }

    return true;
}
