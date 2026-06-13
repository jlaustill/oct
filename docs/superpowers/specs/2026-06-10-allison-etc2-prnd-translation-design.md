# Allison ETC2 → PCI 0x37 PRND Translation

**Date:** 2026-06-10
**Status:** Approved, pending implementation
**Context:** Allison transmission swap. The dash cluster's PRND indicator was
mechanically/electronically driven by the 48RE's controller via PCI message
`0x37`. With the Allison, no module sources `0x37` (confirmed: 15s PCI capture
with the 48RE controller absent showed zero `0x37` frames). OCT must translate
the Allison's J1939 gear selection into `0x37` so the cluster PRND stays accurate.

## Source: Allison ETC2 (J1939 PGN 61445, SA 3)

Bench-confirmed 2026-06-10. Broadcast every 100ms, CAN ID `0x18F00503`.

| Byte (0-based) | SPN | Field | Notes |
|----------------|-----|-------|-------|
| buf[0] | 524 | Selected Gear | numeric: 0xFB=P, 0x7C=R, 0x7D=N, 0x7E–0x83=1–6 |
| buf[1-2] | 526 | Actual Gear Ratio | 0.001/bit |
| buf[3] | 523 | Current Gear | numeric, lags Selected by ~1 frame |
| **buf[4]** | **162** | **Requested Range** | **ASCII — the translation source** |
| buf[6] | 163 | Current Range | ASCII, lags; buf[7]=0x43 const, ignore |

**Why Requested Range (buf[4]):** Selected/Requested update the instant the lever
moves; Current fields lag ~1 frame during clutch engagement. Tracking the lever
gives a factory-correct dash feel. ASCII avoids the numeric `0xFB` special-case.

**Range encoding (live-confirmed):** `'P'`/`'R'`/`'N'` are literal; Drive reports
a **digit** — Requested Range = top allowed range (`'6'` default, `'5'` if limited).

## Target: PCI 0x37 (cluster PRND display)

`PciBus::sendShiftLever(b1)` emits `{0x37, b1, 0x00}` (+CRC). `b2=0x00` confirmed
on this truck. Display byte `b1`: 0x01=P, 0x02=R, 0x03=N, 0x05=D.

## Mapping

| ETC2 Requested Range (buf[4]) | PCI 0x37 b1 | Display |
|-------------------------------|-------------|---------|
| `'P'` (0x50) | 0x01 | P |
| `'R'` (0x52) | 0x02 | R |
| `'N'` (0x4E) | 0x03 | N |
| `'1'`–`'6'` (0x31–0x36) | 0x05 | D |
| anything else | — | hold last value |

## Design

1. **State** — add `TransmissionData transmission { FloatValue prndDisplayByte }`
   to `AppData`. Holds the mapped `0x37` b1 value; reuses the ISR-safe
   `FloatValue` update/read/isStale pattern.
2. **Decode** — in `J1939Bus::onReceive`, add `case 61445` to the existing PDU2
   switch. If `src == 3`, map `buf[4]` → display byte; on a valid range,
   `transmission.prndDisplayByte.update(display)`.
3. **Transmit** — new `PciBus::loop()`, called from `OctDomain::loop()`. Every
   **100 ms**: if `prndDisplayByte.isStale(2000)`, send nothing (cluster blanks);
   else read and `sendShiftLever((uint8_t)value)`.

## Error / edge handling

- **ETC2 dropout (>2s stale):** stop transmitting `0x37`. The cluster blanks PRND
  on its own — factory-like, and avoids displaying a stale gear if the trans
  drops off the bus.
- **Unknown range char:** hold the last valid display (don't blank mid-shift).
- **Bus safety:** 100ms TX rate matches ETC2 and is well within J1850 VPW limits.
  Prior collisions (memory 2026-05-21) were caused by the 48RE controller fighting
  for the `0x37` ID; with it removed, OCT owns the message exclusively.

## Verification

1. **Cluster-display confirmation (also the 0x37-direction proof):** flash, leave
   Allison in Park → dash should show **P**. This is the first proof the cluster
   *receives* and displays `0x37`.
2. Move selector P→R→N→D; dash PRND should follow within ~100ms.
3. Power down the Allison (or unplug) → after ~2s the dash PRND should blank.

## Out of scope

- Manual range display (2/3/L) — all forward ranges show **D** for now.
- Upshift verification (needs road speed; not bench-testable).
