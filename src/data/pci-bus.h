#ifndef PCI_BUS_H
#define PCI_BUS_H

#include <J1850VPWCore.h>
#include "app-data.h"

// Pin assignments (swapped from schematic net names due to board wiring)
#define J1850VPW_RX 14
#define J1850VPW_TX 15

// PCI message IDs
#define PCI_MSG_PCM_ENGINE  0x10
#define PCI_MSG_MULTI_SENSOR 0xC0
#define PCI_MSG_AMBIENT_TEMP 0xCD
#define PCI_MSG_VIN         0xF0

class PciBus {
    public:
        static void setup(AppData* appData);

    private:
        static AppData* _appData;
        static void onMessageReceived(uint8_t* message, uint8_t messageLength);
        static void onError(J1850VPW_Operations op, J1850VPW_Errors err);
};

#endif // PCI_BUS_H
