#ifndef CM848_J1939_RECEIVER_H
#define CM848_J1939_RECEIVER_H

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "app-data.h"

// Parses CM848 J1939 broadcast frames (PDU2, src=0x00) into AppData.
// Call from CumminsBus::onReceive (ISR context).
// Returns true if the frame was a J1939 broadcast (caller should relay to CAN3).
class Cm848J1939Receiver {
    public:
        static bool onReceive(const CAN_message_t& msg, volatile AppData* appData);
};

#endif // CM848_J1939_RECEIVER_H
