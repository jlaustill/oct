#ifndef J1939_BUS_H
#define J1939_BUS_H

#include <FlexCAN_T4.h>
#include <J1939Message.h>
#include "app-data.h"

// Staleness thresholds per spec
#define STALE_FAST_MS 500   // EEC1, CCVS (100ms broadcast)
#define STALE_SLOW_MS 5000  // VD, EH (1s broadcast)

class J1939Bus {
    public:
        static void setup(AppData* appData);
        static void broadcastEEC1();
        static FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> J1939BusCan;

    private:
        static AppData* _appData;
        static void onReceive(const CAN_message_t &msg);
};

#endif // J1939_BUS_H
