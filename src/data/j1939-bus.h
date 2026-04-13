#ifndef J1939_BUS_H
#define J1939_BUS_H

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <J1939Message.h>
#include "app-data.h"

// EEC1, CCVS values come from PCI — stale if PCI goes quiet
#define STALE_FAST_MS 500

// TP BAM state machine for multi-packet responses
struct TpBamState {
    bool active;
    uint8_t data[64];       // max payload
    uint8_t dataLen;
    uint8_t totalPackets;
    uint8_t nextPacket;      // 1-based
    uint32_t pgnToSend;
    elapsedMillis sinceLastPacket;
};

class J1939Bus {
    public:
        static void setup(AppData* appData);
        static void loop();                  // owns all broadcast timing
        static void sendAddressClaim();
        static FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> J1939BusCan;
        static elapsedMillis sinceLastRx;

    private:
        static AppData* _appData;
        static TpBamState _tp;
        static elapsedMillis _since100msBroadcast;   // EEC1, CCVS
        static elapsedMillis _since1sBroadcast;      // VD, EH

        static void broadcastEEC1();
        static void broadcastCCVS();
        static void broadcastVD();
        static void broadcastEH();
        static void processTpBam();
        static void startTpBam(uint32_t pgn, const uint8_t* data, uint8_t len);
        static void onReceive(const CAN_message_t &msg);
};

#endif // J1939_BUS_H
