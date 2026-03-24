# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

OCT is a CAN bus-based engine tuning system for Cummins diesel engines, running on a Teensy 4.1 (Teensy Micromod). It bridges a Cummins proprietary CAN bus with J1939 standard protocol, intercepting and modifying engine parameters (timing and fuel injection) in real-time based on throttle position and engine load.

## Build Commands

```bash
pio run                              # Build
pio run -e teensymm --target upload  # Flash to Teensy
pio device monitor                   # Serial monitor
pio run -e teensymm --target clean   # Clean build
```

## Architecture

**Static class-based, interrupt-driven dual CAN bus system.**

- `src/main.cpp` — Entry point, delegates to `OctDomain::setup()` / `OctDomain::loop()`
- `src/domain/oct-domain` — Initializes both buses, defines bus macros (`CUMMINS_BUS` = CAN3, `J1939_BUS` = CAN2)
- `src/data/cummins-bus` — Core tuning logic. Interrupt callback parses Cummins CAN messages, extracts engine params (RPM, timing, fuel, throttle, load, water temp), modifies timing/fuel values, and forwards messages
- `src/data/j1939-bus` — J1939 side. Filters PGN 59904 (request messages) and forwards to Cummins bus

**Message flow:** J1939 Bus ↔ Teensy (CAN2/CAN3) ↔ Cummins Bus, with tuning modifications applied to Cummins-originated messages before forwarding.

**Tuning behavior:**
- Only active when engine is warmed up (coolant > 65°C)
- Uses `SeaDash::Floats::mapf()` for linear interpolation between current and max tuning values
- Scales based on the higher of throttle% and load%
- CAN ID 256 carries fuel (bytes 0-1, /40.95), timing (bytes 4-5, /128), and RPM (bytes 6-7, /4)

## Key Conventions

- All bus classes use static methods and volatile static state for ISR-safe data sharing
- Both CAN buses run at 250 kbps with interrupt-driven receive (`onReceive()` callbacks)
- Header guards use `#ifndef` with matching `.cpp` file guards
- PlatformIO project structure (`src/`, `include/`, `lib/`, `test/`)

## Hardware Pinout

Connector: USB (pins 1-4), Power (pin 7: 12-18V), Cummins CAN (pins 9-10), J1939 CAN (pins 11-12)
