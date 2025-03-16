#ifndef CUMMINS_BUS_CPP
#define CUMMINS_BUS_CPP

#include "cummins-bus.h"
#include <J1939Message.h>
#include <SeaDash.hpp>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CumminsBus::CumminsBusCan;

volatile float CumminsBus::timing = 0.0;
volatile float CumminsBus::fuel = 0.0;
volatile uint8_t CumminsBus::throttlePercent = 0;
volatile uint8_t CumminsBus::load = 0;
volatile uint16_t CumminsBus::rpms = 0;
volatile J1939Message CumminsBus::message;
// tuning data
volatile float CumminsBus::maxTiming = 15.0f;
volatile float CumminsBus::maxFuel = 100.0f;
volatile int CumminsBus::maxOfThrottleAndLoad = 0;
volatile float CumminsBus::newTiming = 0.0;
volatile float CumminsBus::newFuel = 0.0;
volatile uint32_t CumminsBus::shortTimingValue = 0;
volatile uint32_t CumminsBus::shortFuelValue = 0;
volatile int16_t CumminsBus::waterTemp = -40;
volatile boolean CumminsBus::warmedUp = false;



void CumminsBus::setup() {
    CumminsBus::CumminsBusCan.begin();
    CumminsBus::CumminsBusCan.setBaudRate(250 * 1000);
    CumminsBus::CumminsBusCan.setMaxMB(16);
    CumminsBus::CumminsBusCan.enableFIFO();
    CumminsBus::CumminsBusCan.enableFIFOInterrupt();
    CumminsBus::CumminsBusCan.onReceive(CumminsBusSniff);
    CumminsBus::CumminsBusCan.mailboxStatus();
}

void CumminsBus::CumminsBusSniff(const CAN_message_t &msg) {
    const_cast<J1939Message&>(message).setCanId(msg.id);
    const_cast<J1939Message&>(message).setData(msg.buf);
    

    switch (message.canId) {
        case 256:
            rpms = ((uint16_t)message.data[7] << 8) + (uint16_t)message.data[6];
            rpms /= 4;
            timing = (float)((uint16_t)message.data[5] << 8) + (uint16_t)message.data[4];
            timing /= 128.0f;
            fuel = (float)((uint16_t)message.data[1] << 8) + (uint16_t)message.data[0];
            fuel /= 40.95f;
            // update timing and fuel here
            if (warmedUp) {
                CAN_message_t copyMsg = msg;
                UpdateMaxTiming();
                maxOfThrottleAndLoad = max(throttlePercent, load);
                newTiming = SeaDash::Floats::mapf<float>((float)maxOfThrottleAndLoad, 0.0f, 100.0f, timing, maxTiming);
                newTiming *= 128.0f;
                shortTimingValue = (unsigned short)newTiming;
                copyMsg.buf[4] = lowByte(shortTimingValue);
                copyMsg.buf[5] = highByte(shortTimingValue);
                newFuel = SeaDash::Floats::mapf<float>(maxOfThrottleAndLoad, 0.0f, 100.0f, fuel, maxFuel);
                newFuel *= 40.95f;
                shortFuelValue = (uint32_t)newFuel;
                copyMsg.buf[0] = lowByte(shortFuelValue);
                copyMsg.buf[1] = highByte(shortFuelValue);
                // write the modified message to the bus
                CumminsBus::CumminsBusCan.write(copyMsg);
            }
        return;
    }

    switch (message.pgn) {
        case 65262:
            waterTemp = (int16_t)message.data[0] - 40;
            if (!warmedUp && waterTemp > 65) {
                warmedUp = true;
            }
        return;
        case 61443:
            throttlePercent = message.data[1] * 0.4f;
            load = (float)message.data[2] * 0.8f;
    }


    J1939Bus::J1939BusCan.write(msg);
}

void CumminsBus::UpdateMaxTiming() {
    if (rpms < 1200) {
        maxTiming = timing;
    } else {
        maxTiming = 30.0f;
    }
}

#endif // CUMMINS_BUS_CPP