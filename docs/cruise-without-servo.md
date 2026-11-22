# Cruise control without the vacuum servo

The goal: remove the cruise servo, its vacuum lines and the vacuum pump, and keep cruise control. The truck keeps its vehicle speed sensor and its steering wheel buttons.

**The truck:** a 2004.5 Dodge Ram 5.9 with the CM848 ECM. An Allison LCT1000 6-speed replaces the 48RE, and OCT bridges the ECM's bus and the Allison's J1939 bus.

**Firmware details:** the evidence is in [CM848 cruise control](https://github.com/jlaustill/kuminz-re/blob/main/firmware/CM848_S90140.06_analysis/docs/cruise_control.md) in kuminz-re. Its names follow kuminz-re PR #53.

Nothing here has been tested on the truck yet.

## Why it should work

- **The ECM already fuels cruise itself.** The servo only makes the throttle follow the fuel, so the pedal sensor agrees with the fueling.
- **One calibration bit turns the servo off:** EEPROM 0x170 mask 0x10 (e2m `Vac_Throttle_Cruise_Enable`). With the bit clear, the ECM keeps the servo's vent, vacuum and supply outputs off and still runs cruise.
  - Today the ECM already keeps those outputs off whenever cruise isn't engaged.
  - So clearing the bit makes every moment look like today's cruise-off state.
- **The Allison still gets a fuel-based throttle during cruise.**
  - OCT relays every 8-byte PDU2 broadcast from the ECM (source 0x00) to the Allison bus unchanged: `Cm848J1939Receiver::onReceive` accepts the frame and `CumminsBus` writes it to `J1939Bus`. EEC2 goes through that path.
  - The ECM fills EEC2's accelerator pedal position (SPN 91) from its equivalent throttle: the throttle that would give the current fuel. It does that with or without the servo.
- **The swap's cruise fix stays in place.** OCT's gateway (`J1939Bus::onReceive`) forwards every Allison frame (source 0x03) to the ECM. The Allison's ETC1 output shaft speed is what lets cruise engage (commit 829c146). Removing the servo doesn't touch that path.

## The calibration change

1. Change EEPROM 0x170 from `0x11dc` to `0x11cc`.
2. Fix the CRC16 at EEPROM 0x482. A mismatch sets P0602.

Check whether HP Tuners updates the CRC for you.

## Before you start

### Check the stored codes

Look for P0572 or P0573 in the history. The ECM builds a second brake signal from the servo supply's feedback line, raw switch 28. It reads that line raw whenever its supply output is off, which today is any time cruise isn't engaged. A clean history says that path already works on this harness.

### Meter test on circuit V32 (18 vt/yl)

**What the pinout doc shows.** kuminz-re's `ecu_pinout.md` describes the 2004 automatic wiring:
- V32 is "Speed Control Supply" on C2 pins 7 and 36.
- The 2005 wiring calls pin 36 "Brake Switch No. 2 Signal".
- The 2005 wiring leaves the vent control pin (4) and the vacuum control pin (34) empty.

Nobody has proven which ECM pin the firmware's switch 28 reads, so measure V32 wherever you can reach it.

**How to measure.** Put the key on with the engine off and cruise off; the ECM's supply output is off then. Measure V32 to ground, first with the brake released and then pressed. Do this once with the servo plugged in and once with it unplugged.

**If V32 follows the brake with the servo unplugged,** something other than the servo powers the circuit. The ECM can keep reading the brake after the servo comes out.

**If V32 stays at 0 V, or stops following the brake once the servo is unplugged,** the brake signal depends on the servo circuit:
- With the servo gone, the derived brake signal would stop changing.
- The ECM would then set P0573 or P0572 after 16 brake events and deny cruise.
- Give V32 another feed through the brake switch before you remove the servo. The 2005 8W-80 diagram is worth checking, since it labels pin 36 "Brake Switch No. 2 Signal".

## Test plan

Bench-test each step before you drive. At every step, confirm that the brake and the CANCEL button both drop cruise before you use it on the road.

1. **Flip the bit with the servo still connected.**
   - Cruise engages and holds speed on fuel alone.
   - There are no new codes, especially P0572, P0573, P0595, P0596 or P0602.
   - The Allison shifts normally in cruise, on flat road and on hills.
2. **Remove the vacuum pump and lines.** The servo can't pull without vacuum. First make sure nothing else on the truck uses that vacuum source. Then repeat the checks.
3. **Unplug and remove the servo.** Repeat the checks, and watch for P0572 or P0573 over at least 16 brake applications.

## What to watch on the Allison

During servo-free cruise, EEC2 will report a non-zero pedal position (the fuel-based value) along with "pedal at low idle" (SPN 558, from the real idle switch). With the servo, both moved together.

If the Allison reacts to that mix with odd shifts or odd lockup in cruise, OCT could rewrite the SPN 558 bits in the relayed EEC2 while cruise is active. OCT would need a cruise-active signal for that. The ECM's own CCVS1 broadcast is the likely source, but nobody has checked which bits the ECM fills in it.
