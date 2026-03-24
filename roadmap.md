# OCT Roadmap — ELD-Ready J1939 Bridge

## Goal

Make OCT a fully functional J1939 bridge for a 2004 Ram with CM848D ECU, capable of providing all data required by a Garmin eLog ELD. Data comes from the Cummins proprietary CAN bus and the PCI (J1850 VPW) body computer bus, and is broadcast on the J1939 bus per spec.

## Architecture

**Approach A: Centralized AppData, Buses Are Dumb Pipes**

- `AppData` is the single source of truth for all vehicle/engine state
- Values stored as human-readable metric floats in per-type `Value` structs (`FloatValue`, `StringValue`)
- Each value carries its unit string and a timestamp for staleness detection
- **CumminsBus (CAN3):** reads from ECU, writes into AppData
- **PciBus (J1850 VPW):** reads from body computer, writes into AppData
- **J1939Bus (CAN2):** reads from AppData, broadcasts/responds per J1939 spec
- **No bus-to-bus forwarding.** Data flows one direction: Cummins/PCI → AppData → J1939

## Milestones

### M1: Foundation — AppData + PGN 61444 (EEC1)
> Get RPM broadcasting on J1939 at 100ms

- [ ] Create `FloatValue`, `StringValue` structs with staleness detection
- [ ] Create `AppData` struct with EEC1 fields (engineRpm, engineTorque)
- [ ] Strip existing bench tuning code and bus-to-bus forwarding from CumminsBus
- [ ] Refactor `J1939Bus` to read from AppData and broadcast PGN 61444 at 100ms
- [ ] Refactor `CumminsBus` to write RPM into AppData (source TBD — verify in-truck)
- [ ] In-truck: sniff Cummins bus to determine actual RPM source
- [ ] In-truck: verify Garmin eLog sees RPM

### M2: Vehicle Speed — PGN 65265 (CCVS)
> Get vehicle speed broadcasting on J1939 at 100ms

- [ ] Add PCI bus hardware (MC33390 VPW transceiver) — export schematic for pin assignment
- [ ] Create `PciBus` class, integrate J1850 VPW library
- [ ] Validate VPW comms work (receive any message before decoding specific ones)
- [ ] Decode PCI message 0x10 (speed) into AppData.vehicleSpeed
- [ ] Add PGN 65265 broadcast to J1939Bus at 100ms
- [ ] In-truck: verify speed accuracy against PCI 0x10 data
- [ ] In-truck: verify Garmin eLog sees vehicle speed
- [ ] Fallback: if CM848D provides speed via Cummins bus, use that instead

### M3: Odometer — PGN 65248 (Vehicle Distance)
> Get total vehicle distance broadcasting on J1939 at 1s

- [ ] Determine odometer source: CM848D J1939 response (already responds to 65248), PCI bus (0x5D?), or self-tracked
- [ ] Add PGN 65248 broadcast to J1939Bus at 1s
- [ ] In-truck: verify odometer value matches dash

### M4: Engine Hours — PGN 65253 (Engine Hours)
> Get engine hours broadcasting on J1939 at 1s

- [ ] Determine source: CM848D Service 0x4A memory read, or self-tracked from RPM > 0
- [ ] Add PGN 65253 broadcast to J1939Bus at 1s
- [ ] In-truck: verify hours value is reasonable

### M5: VIN — PGN 65260 (Vehicle Identification)
> Respond to VIN requests via J1939 Transport Protocol

- [ ] Determine VIN source: CM848D TP response (already responds to 65260), PCI 0xF0, or hardcoded
- [ ] Implement J1939 Transport Protocol for multi-packet VIN response
- [ ] Add PGN 59904 request handler for VIN in J1939Bus
- [ ] In-truck: verify Garmin eLog reads correct VIN

### M6: ELD Validation
> End-to-end Garmin eLog compliance check

- [ ] All 5 required PGNs broadcasting/responding
- [ ] Garmin eLog pairs and shows valid data
- [ ] Drive test: verify speed, odometer accumulation, engine hours accumulation
- [ ] Verify ELD enters driving mode at > 5 mph
- [ ] Verify ELD detects engine on/off from RPM

### M7: Tuning (Post-ELD)
> Re-integrate fuel/timing tuning against real truck data

- [ ] Add tuning fields to AppData (throttle, load, timing, fuel, coolant temp)
- [ ] Verify Cummins bus data sources in-truck
- [ ] Re-implement tuning logic reading/writing through AppData
- [ ] Warmup detection from coolant temp in AppData

### M8: Additional J1939 PGNs (Nice-to-Have)
> Broadcast supplementary data the ELD or other devices may use

- [ ] PGN 65262 — Engine Temperature 1 (coolant, oil temp)
- [ ] PGN 65263 — Engine Fluid Level/Pressure (oil pressure)
- [ ] PGN 65269 — Ambient Conditions (barometric pressure)
- [ ] PGN 65270 — Inlet/Exhaust Conditions (intake pressure/temp)
- [ ] PGN 65271 — Vehicle Electrical Power (battery voltage)
- [ ] PGN 61443 — EEC2 (throttle position)

## Hardware

- **Teensy Micromod** (existing)
- **CAN3** → Cummins proprietary bus (250 kbps)
- **CAN2** → J1939 bus (250 kbps)
- **J1850 VPW** → PCI body computer bus (MC33390 transceiver, pins TBD from schematic)

## Key Unknowns (Resolve In-Truck)

1. What does the CM848D actually broadcast on its proprietary bus vs. what must be requested?
2. Does CAN ID 256 carry RPM in the real truck as the bench code assumed?
3. Does the CM848D have a valid vehicle speed in PGN 65265, or does speed only come from PCI?
4. What is the actual odometer source?
5. Can engine hours be read from the CM848D, or must they be self-tracked?
6. Does the Garmin eLog require J1939 address claiming (PGN 60928), or does it accept broadcasts without it?
7. Can M3/M5 be skipped if the Garmin eLog requests odometer/VIN directly from the CM848D on the J1939 bus?

## Known Risks

- The Garmin eLog may validate J1939 source address or require address claim handshake — if so, PGN 60928 support must be added
- If the CM848D is not on the J1939 bus in the truck's wiring, PGNs it currently responds to (65248, 65260) will need to be synthesized by OCT instead

## Notes

- All existing code was bench-tested only — assume nothing is verified
- The truck is a 2004 Ram with CM848D ECU (S90140.06 firmware)
- ELD target: Garmin eLog (passive J1939 listener, FMCSA 49 CFR 395.26 compliant)
- C-Next conversion planned — no C++ templates, keep structures simple
