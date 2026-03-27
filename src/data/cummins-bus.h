#ifndef CUMMINS_BUS_H
#define CUMMINS_BUS_H

#include <FlexCAN_T4.h>
#include <J1939Message.h>
#include "app-data.h"

class CumminsBus {
    public:
        static void setup(AppData* appData);
        static void requestPgn(uint32_t pgn);
        static FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CumminsBusCan;
        static volatile uint32_t msgCount;

    private:
        static AppData* _appData;
        static void onReceive(const CAN_message_t &msg);
};

#endif // CUMMINS_BUS_H
