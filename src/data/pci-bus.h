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
#define PCI_MSG_MULTI_SENSOR     0xC0
#define PCI_MSG_AMBIENT_TEMP     0xCD
#define PCI_MSG_VIN              0xF0

class PciBus {
    public:
        static void setup(AppData* appData);
        static volatile uint32_t msgCount;
        static volatile uint32_t errCount;
        static elapsedMillis sinceLastRx;

    private:
        static AppData* _appData;
        static void onMessageReceived(uint8_t* message, uint8_t messageLength);
        static void onError(J1850VPW_Operations op, J1850VPW_Errors err);
};

#endif // PCI_BUS_H
