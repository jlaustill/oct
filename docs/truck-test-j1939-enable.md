# Truck test: J1939 enable without dev mode, APPS/load from EEC2

This branch has two behavior changes. Both have to be confirmed on the truck before merging.

| # | Change | What the firmware says (kuminz-re `deeb3dcf`) |
|---|---|---|
| 1 | `Cm848BroadcastController` sends only `0x05 01 01`. The `0x0a` / `0x07` steps are gone. | `0x0a` is `enterDevelopmentMode` and needs the security key: `j1939_command_security_flags` (ROM 0x5a486 = `44 0f ff 03`) has bit 10 set, and `calibration_download_security_key_cal` is 0, so the check is live. The ECM refuses a plain `0x0a` with KEY_MISSING (0x1a). `0x07` (`exitDevelopmentMode`) then returns CANNOT_COMPLY, because dev mode never turned on. Service 5 has no key bit and does not check dev mode. Expected result: no change on the ECM. |
| 2 | The CLIP poll of 0x40a37a / 0x40b1f0 is gone. `ecu.appsPercent` and `ecu.engineLoad` now come only from EEC2 (SPN 91 / 92). | 0x40a37a is `egr_fraction_limited_command` and 0x40b1f0 is `transmission_output_speed_in_use`. Neither is throttle or load. Only the serial display read them. |

## Equipment
- A laptop on OCT USB running `pio device monitor` (115200). Save the log.
- A CAN logger on the J1939 bus (the Allison side). Add a second logger on the Cummins bus if you have one.
- Two builds:
  - **A:** `main` (with the J1939 pin from the build-fix PR).
  - **B:** this branch.
- Flash with `pio run -e teensymm --target upload`.

## Run each test with build A first (baseline), then with build B

| # | Step | Look for |
|---|---|---|
| 1 | Key on, engine off, for 30 s. | Serial (A): `0x0a`, `0x07`, `0x05`, then `enabled — watching EEC1`. Serial (B): `0x05 01 01`, then `enabled`. EEC1 `0x0CF00400` appears on J1939 within about 1 s of the `0x05` line. |
| 2 | Same window, Cummins-bus log (A only). | ECM replies on `0x18EFF900`. Byte 0 is `0x0d` (negative) after the `0x0a` and the `0x07` requests, and `0x0c` (positive) after `0x05`. This confirms change 1 removes only refused requests. |
| 3 | Start the engine and idle for 2 min. | No `CM848BC` re-enable loop on serial. That loop would mean the EEC1 watchdog tripped. EEC1 and EEC2 are steady. ET1, IC1, EFL/P1, CCVS, VD, HRVD and EH from OCT are present. |
| 4 | Pedal sweep: idle, about 50 %, WOT (in neutral, briefly). | Serial APPS reads about 0 / 50 / 100 and load moves with it. On A the value can jump between readings, because the EGR and output-speed reads fight EEC2. On B it is smooth. |
| 5 | Key cycle the ECM while OCT stays powered, if your wiring allows it. Otherwise power cycle the whole truck. | OCT re-enables broadcast. Serial shows the watchdog `CHECK_PROT`, then `0x05` again, and EEC1 comes back. |
| 6 | Drive: shifts 1 through top gear, a lockup cruise, and a stop. | Allison shifts normally and sets no transmission codes. The PRND decode on serial matches the selector. Cruise behaves the same on A and B. |

## Pass, then merge
B passes when tests 1 to 6 match A, except that test 4 is smooth on B. Any Allison code, a missing EEC1, or a watchdog loop on B means do not merge. Send me the serial log and the CAN logs.

## Side note (not changed here)
OCT's keyed 1 s "keepalive" sends service `0x16`, which is `forceExitDevelopmentMode`. With the engine stopped, it turns RAM refresh back on. With the engine running, it returns NOT_SAFE and does nothing. It is harmless today. It would undo any future dev-mode entry, for example the channel-47 cruise override, while the engine is off.
