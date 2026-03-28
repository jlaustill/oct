#ifndef J1939_BUS_H
#define J1939_BUS_H

#include <FlexCAN_T4.h>
#include <J1939Message.h>
#include "app-data.h"

// Staleness thresholds per spec
#define STALE_FAST_MS 500   // EEC1, CCVS (100ms broadcast)
#define STALE_SLOW_MS 5000  // VD, EH (1s broadcast)

// TP BAM state machine for multi-packet responses
struct TpBamState {
    bool active;
    uint8_t data[64];       // max payload
    uint8_t dataLen;
    uint8_t totalPackets;
    uint8_t nextPacket;      // 1-based
    uint32_t pgnToSend;
    uint32_t lastPacketTime;
};

class J1939Bus {
    public:
        static void setup(AppData* appData);
        static void sendAddressClaim();
        static void broadcastEEC1();
        static void broadcastCCVS();
        static void broadcastVD();
        static void broadcastEH();
        static void processTpBam();  // call from loop()
        static FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> J1939BusCan;
        static volatile uint32_t lastRxTime;

    private:
        static AppData* _appData;
        static TpBamState _tp;
        static void startTpBam(uint32_t pgn, const uint8_t* data, uint8_t len);
        static void onReceive(const CAN_message_t &msg);
};

#endif // J1939_BUS_H
