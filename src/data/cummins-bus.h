#ifndef CUMMINS_BUS_H
#define CUMMINS_BUS_H

#include <FlexCAN_T4.h>
#include "j1939-bus.h"
#include <J1939Message.h>

class CumminsBus {
    public:
        static void setup();
        static FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CumminsBusCan;

    private:
        static void CumminsBusSniff(const CAN_message_t &msg);
        static void UpdateMaxTiming();

        volatile static float timing;
        volatile static float fuel;
        volatile static uint8_t throttlePercent;
        volatile static uint8_t load;
        volatile static uint16_t rpms;
        volatile static J1939Message message;

        // tuning data
        volatile static float maxTiming;
        volatile static float maxFuel;
        volatile static int maxOfThrottleAndLoad;
        volatile static float newTiming;
        volatile static float newFuel;
        volatile static uint32_t shortTimingValue;
        volatile static uint32_t shortFuelValue;
        volatile static int16_t waterTemp;
        volatile static boolean warmedUp;
};

#endif // CUMMINS_BUS_H