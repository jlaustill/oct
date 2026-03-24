# ELD Bridge Design — OCT J1939 for Garmin eLog

**Date:** 2026-03-24
**Status:** Approved

## Problem

The 2004 Ram with CM848D ECU has three separate buses (Cummins CAN, J1939 CAN, PCI/J1850 VPW) but the Garmin eLog ELD expects all required data on J1939. The CM848D does not broadcast EEC1 (RPM) or Engine Hours on J1939, and vehicle speed may only be available from the PCI body computer bus.

## Architecture Decision

**Centralized AppData, Buses Are Dumb Pipes.**

Data flows one direction: source buses → AppData → J1939 bus. No bus-to-bus forwarding.

```
  Cummins Bus (CAN3)          PCI Bus (J1850 VPW)
       |                            |
       | write                      | write
       v                            v
  ┌─────────────────────────────────────┐
  │              AppData                │
  │                                     │
  │  FloatValue engineRpm      {RPM}    │
  │  FloatValue vehicleSpeed   {km/h}   │
  │  FloatValue totalDistance   {km}     │
  │  FloatValue engineHours    {hours}  │
  │  StringValue vin                    │
  │  ...                                │
  └─────────────────────────────────────┘
                    |
                    | read
                    v
            J1939 Bus (CAN2)
         broadcast / respond
```

## Value Types

Per-type value structs (no templates — C-Next compatible):

```cpp
struct FloatValue {
    float value;
    const char* unit;
    uint32_t lastUpdated;  // millis()
};

struct StringValue {
    char value[18];
    const char* unit;
    uint32_t lastUpdated;
};
```

`lastUpdated` enables staleness detection. When stale, J1939 bus sends `0xFF` bytes per spec ("not available").

**Staleness thresholds:**
- Fast PGNs (EEC1, CCVS — 100ms broadcast): stale after 500ms
- Slow PGNs (VD, EH — 1s broadcast): stale after 5000ms

## Concurrency

AppData is shared between ISR callbacks (CumminsBus, PciBus) and the main `loop()` (J1939Bus broadcasts). On Cortex-M7, `volatile` alone does not guarantee atomic reads of multi-byte structs — a `FloatValue` is 12 bytes and can be torn.

**Contract:**
- **ISR writers** (CumminsBus, PciBus): wrap multi-field AppData writes in `noInterrupts()`/`interrupts()` pairs (maps to C-Next `critical { }`)
- **Main loop readers** (J1939Bus): wrap multi-field AppData reads in `noInterrupts()`/`interrupts()` pairs
- Critical sections are short (copy a single `FloatValue` — 12 bytes) so interrupt latency is negligible at 600MHz

## AppData Struct

```cpp
struct AppData {
    // EEC1 - PGN 61444 (broadcast 100ms)
    FloatValue engineRpm;            // RPM
    FloatValue engineTorque;         // %

    // CCVS - PGN 65265 (broadcast 100ms)
    FloatValue vehicleSpeed;         // km/h

    // VD - PGN 65248 (broadcast 1s)
    FloatValue totalVehicleDistance;  // km

    // EH - PGN 65253 (broadcast 1s)
    FloatValue engineTotalHours;     // hours

    // VI - PGN 65260 (on request)
    StringValue vin;
};
```

## Bus Responsibilities

### CumminsBus (CAN3)
- Listens for proprietary Cummins messages
- May send PGN 59904 requests to get J1939 responses from ECU
- Writes decoded metric values into AppData
- Does NOT forward to other buses
- Does NOT apply tuning — existing bench tuning code will be stripped in M1 and re-introduced through AppData in M7

### PciBus (J1850 VPW)
- Listens for body computer messages (speed, VIN, temps, etc.)
- Writes decoded metric values into AppData
- Read-only on the bus

### J1939Bus (CAN2)
- Reads from AppData, encodes to J1939 raw format on the fly
- Broadcasts at spec-defined intervals:
  - PGN 61444 (EEC1): 100ms
  - PGN 65265 (CCVS): 100ms
  - PGN 65248 (VD): 1s
  - PGN 65253 (EH): 1s
- Responds to PGN 59904 requests for on-request PGNs (VIN)
- Fills unsupported bytes with 0xFF per J1939

### OctDomain
- Initializes all buses and AppData
- `loop()` drives broadcast timing
- Future home for tuning logic

## PGN 61444 — EEC1 (First Implementation)

8-byte message, broadcast every 100ms. This table describes the **outbound J1939 encoding** produced by J1939Bus, not the Cummins source message format.

| Byte | SPN | Parameter | Encoding | Source |
|------|-----|-----------|----------|--------|
| 0 | 899 | Engine torque mode | 0xFF | Not available |
| 1 | 512 | Driver demand torque | 0xFF | Not available |
| 2 | 513 | Actual engine torque | 0xFF | Not available |
| 3-4 | 190 | Engine speed | RPM / 0.125, uint16 LE | appData.engineRpm |
| 5 | 1483 | Source address | 0x00 | Hardcoded (ECU) |
| 6 | 2432 | Starter mode | 0xF | Not available |
| 7 | — | Reserved | 0xFF | — |

CAN ID: `0x0CF00400` (priority 3, PGN 61444, source 0x00, extended frame)

## FMCSA ELD Requirements (49 CFR 395.26)

| PGN | Parameter | Required | Broadcast Rate |
|------|-----------|----------|---------------|
| 61444 | Engine RPM | Yes — engine on/off detection | 100ms |
| 65265 | Vehicle Speed | Yes — driving state detection | 100ms |
| 65248 | Odometer | Yes — distance accumulation | 1s |
| 65253 | Engine Hours | Yes — hours accumulation | 1s |
| 65260 | VIN | Yes — vehicle identification | On request |

## Key Constraints

- All existing bench code is unverified against the real truck
- CM848D does NOT respond to PGN 61444 or 65253 on J1939
- CM848D DOES respond to PGN 65248 (odometer) and 65260 (VIN) on J1939
- Vehicle speed source (CM848D vs PCI bus) is unknown until in-truck testing
- No C++ templates — must be C-Next convertible
- Values stored as human-readable metric floats, converted to J1939 encoding at broadcast time
- J1939 source address: OCT uses 0x00 — the CM848D uses 0x00 natively but since OCT sits between the ECU and the J1939 bus, it replaces the ECU's presence. Address claiming (PGN 60928) deferred unless Garmin eLog requires it.
- VIN Transport Protocol framing/padding is handled at the J1939Bus encoding layer, not in AppData
- M1 will strip existing bench tuning code and bus-to-bus forwarding from CumminsBus
