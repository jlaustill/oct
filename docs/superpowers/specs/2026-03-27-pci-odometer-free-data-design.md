# PCI Odometer Request + Free Data Extraction

**Date:** 2026-03-27
**Status:** Approved

## Problem

The ELD needs odometer data (J1939 PGN 65248) but the PCM odometer is not broadcast on the PCI bus — it must be requested via a J1850 VPW diagnostic command. Additionally, several data fields from PCI messages we already receive are not being decoded.

## Changes

### 1. Free Data — Extract from existing PCI messages

No additional bus traffic. Three new AppData fields, three lines of decode:

| Source | Byte | Field | Conversion | Units |
|--------|------|-------|------------|-------|
| 0x10 | 5 | engineLoad | `byte / 255.0 * 100.0` | % |
| 0xC0 | 2 | oilPressure | `byte * 0.5 * 6.895` (0.5 PSI/bit per DRB, then PSI→kPa) | kPa |
| 0xC0 | 4 | intakeAirTemp | `byte - 40` | °C |

Minimum message length requirements already satisfied by existing checks (0x10 checks `< 7`, 0xC0 checks `< 5`).

### 2. PCM Odometer — Diagnostic request via PCI

**Request:** `24-10-3C-01-05-00` sent every 1 second via `VPW.write()` from `loop()` (not ISR).

**Expected response:** `64-10-3C-01-XX-XX-XX-XX-...` (diagnostic response echo)
- Response length: 7 bytes per DRB database
- Odometer: 4-byte (32-bit) raw value at offset 2 in the response payload (bytes 4-7 of full message after header `64-10-3C-01`)
- Formula: `raw * 1.609344` miles → km for AppData

**Response validation:** Match response bytes 0-3 as `64-10-3C-01` before decoding to avoid false matches from other diagnostic responses on the bus.

**Verification plan:** Initially log raw bytes of any `0x64` response to a volatile buffer and print from `loop()` (NOT from ISR callback) to confirm exact byte layout. Once verified, decode directly.

**VPW.write() blocking concern:** `VPW.write()` spins waiting for bus idle with a ~1s timeout. PCI bus idle gaps are frequent (~300us IFS between frames) so blocking is unlikely in practice. If it does block, J1939 broadcasts stall. Acceptable risk for now — if problematic in practice, add a `isBusy()` check before calling write.

**PCM unresponsive:** If the PCM never responds, `totalVehicleDistance` stays stale. J1939 PGN 65248 broadcast already checks `isStale()` and sends `0xFF` bytes (not available) when stale, so the ELD sees "no data" rather than a false zero.

### 3. AppData New Fields

```cpp
FloatValue engineLoad;      // %
FloatValue oilPressure;     // kPa
FloatValue intakeAirTemp;   // degrees C
```

`totalVehicleDistance` (km) already exists — odometer response writes to it.

### 4. PciBus Changes

- Fix message filter to be null-terminated: `{ 0x10, 0x64, 0xC0, 0xCD, 0xF0, 0x00 }`
- Add static `requestOdometer()` method using `VPW.write()` — called from `loop()`, not ISR
- Add volatile buffer for diagnostic response logging (verification phase)
- Decode `0x64-10-3C-01` responses: extract 4-byte odometer, convert miles→km, write to AppData
- Add engine load decode to existing `0x10` handler
- Add oil pressure + intake air temp decode to existing `0xC0` handler

### 5. OctDomain Changes

- Add 1s timer to call `PciBus::requestOdometer()`
- Add new fields to debug print output

### 6. No J1939 Changes

J1939Bus already broadcasts PGN 65248 (VD) at 1s from `appData.totalVehicleDistance`. Once PCI odometer populates that field, the ELD gets the data automatically.

## DRB Database Reference

PCM Odometer request from `all_drb_messages.csv`:
```
24-10-3C-01-05-00,"PCM ODOMETER",7,"SAE J1850 VPW PCI",Engine,4,2,"PCM ODOMETER",1.60934400558472,0.0,Miles
```

Fields: request bytes, name, response length=7, protocol, module, data_bytes=4, data_offset=2, name, multiplier=1.609344, offset=0, units=Miles
- `data_bytes=4`: odometer is a 4-byte (32-bit) value
- `data_offset=2`: odometer data starts at byte offset 2 in the response payload
- `multiplier=1.609344`: raw value is in miles, multiply to get km
