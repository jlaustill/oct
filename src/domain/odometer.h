#ifndef ODOMETER_H
#define ODOMETER_H

#include <Arduino.h>

// Owns the lifetime vehicle odometer.
//
// Source of truth: the CM848D Cummins ECU broadcasts VSS pulse counts
// on PCI message $5D. Byte 0 of each $5D is the pulse count since the
// previous $5D transmission, and each pulse = 1/8000 mi. PciBus parses
// $5D and calls onPulses() with the count; we add that directly to our
// lifetime total.
//
// Storage: uint32_t in 1/8000 mi units, matching CM848D's
// Vehicle_Odometer_Data and EEPROM `0x01000BD8` scale. Range = 4.29B
// counts = 536,870 mi. Integer math, no float swamping.
class Odometer {
    public:
        static void setup();
        static void loop();
        static void onPulses(uint8_t pulses);    // PCI $5D byte 0
        static void setAndSave(float km);         // operator seed
        static uint32_t rawCounts();              // for J1939 broadcast
        static float km();                         // for serial echo

    private:
        static uint32_t _total;          // lifetime, 1/8000 mi units
        static elapsedMillis _sinceSave;
        static void load();
        static void save();
};

#endif // ODOMETER_H
