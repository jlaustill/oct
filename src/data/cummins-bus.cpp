#ifndef CUMMINS_BUS_CPP
#define CUMMINS_BUS_CPP

#include "cummins-bus.h"
#include <J1939Message.h>
#include <SeaDash.hpp>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CumminsBus::CumminsBusCan;

volatile float CumminsBus::timing = 0.0;
volatile uint8_t CumminsBus::throttlePercent = 0;
volatile uint8_t CumminsBus::load = 0;
volatile uint16_t CumminsBus::rpms = 0;
volatile J1939Message CumminsBus::message;
// tuning data
volatile float CumminsBus::maxTiming = 15.0f;
volatile int CumminsBus::maxOfThrottleAndLoad = 0;
volatile float CumminsBus::newTiming = 0.0;
volatile unsigned short CumminsBus::shortTimingValue = 0;
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
    message.setCanId(msg.id);
    message.setData(msg.buf);
    

    switch (message.canId) {
        case 256:
            rpms = ((uint16_t)message.data[7] << 8) + (uint16_t)message.data[6];
            rpms /= 4;
            timing = (float)((uint16_t)message.data[5] << 8) + (uint16_t)message.data[4];
            timing /= 128.0f;
            // update timing here
            if (warmedUp) {
                CAN_message_t copyMsg = msg;
                UpdateMaxTiming();
                maxOfThrottleAndLoad = max(throttlePercent, load);
                newTiming = SeaDash::Floats::mapf<float>((float)maxOfThrottleAndLoad, 0.0f, 100.0f, timing, maxTiming);
                newTiming *= 128.0f;
                shortTimingValue = (unsigned short)newTiming;
                copyMsg.buf[4] = lowByte(shortTimingValue);
                copyMsg.buf[5] = highByte(shortTimingValue);
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