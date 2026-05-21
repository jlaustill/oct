#ifndef CUMMINS_BUS_H
#define CUMMINS_BUS_H

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <J1939Message.h>
#include "app-data.h"

class CumminsBus {
    public:
        static void setup(AppData* appData);
        static void loop();
        static void requestPgn(uint32_t pgn);
        static void startRawLog(uint32_t durationMs);
        static FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CumminsBusCan;
        static volatile uint32_t msgCount;
        static elapsedMillis sinceLastRx;

    private:
        static volatile AppData* _appData;
        static elapsedMillis _sincePoll;
        static uint8_t _pollIdx;
        static bool _rawLogActive;
        static elapsedMillis _rawLogTimer;
        static uint32_t _rawLogDurationMs;
        static void sendClipRequest(uint32_t addr, uint8_t len);
        static void onReceive(const CAN_message_t &msg);
};

#endif // CUMMINS_BUS_H
