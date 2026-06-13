#ifndef PCI_BUS_H
#define PCI_BUS_H

#include <Arduino.h>
#include <J1850VPWCore.h>
#include "app-data.h"

// Pin assignments (swapped from schematic net names due to board wiring)
#define J1850VPW_RX 15
#define J1850VPW_TX 14

// PCI message IDs
#define PCI_MSG_PCM_ENGINE       0x10
#define PCI_MSG_DISTANCE_PULSES  0x5D   // ECM Tx, Chry_Dat5D, byte 0 = pulses (1/8000 mi each)
#define PCI_MSG_TRANS_GEAR       0x3A   // TCM Tx — actual engaged gear (internal); byte layout TBD via capture
// 0x37: TCM Tx — cluster PRND display (4 bytes, database-sourced, unconfirmed on this truck)
//   b1: 0x00=off 0x01=P 0x02=R 0x03=N 0x04=2 0x05=D 0x06=3 0x07=L
//   b2: 0x80=normal auto, 0x90-0xC0=autostick holding 1st-4th
#define PCI_MSG_SHIFT_LEVER      0x37
#define PCI_MSG_MULTI_SENSOR     0xC0
#define PCI_MSG_AMBIENT_TEMP     0xCD
#define PCI_MSG_VIN              0xF0

class PciBus {
    public:
        static void setup(AppData* appData);
        static void loop();                  // broadcasts 0x37 PRND from Allison ETC2
        static void startRawLog(uint32_t durationMs);
        static void sendShiftLever(uint8_t gearByte);
        static volatile uint32_t msgCount;
        static volatile uint32_t errCount;
        static elapsedMillis sinceLastRx;

    private:
        static AppData* _appData;
        static bool _rawLogActive;
        static elapsedMillis _rawLogTimer;
        static uint32_t _rawLogDurationMs;
        static void onMessageReceived(uint8_t* message, uint8_t messageLength);
        static void onError(J1850VPW_Operations op, J1850VPW_Errors err);
};

#endif // PCI_BUS_H
