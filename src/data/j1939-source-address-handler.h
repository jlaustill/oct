#ifndef J1939_SOURCE_ADDRESS_HANDLER_H
#define J1939_SOURCE_ADDRESS_HANDLER_H

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "app-data.h"

class J1939SourceAddressHandler {
    public:
        static void setup(AppData* appData, FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>* can);

        // Send our own PGN 60928 address claim
        static void sendOwnClaim();

        // Print all known nodes to Serial
        static void logNodes();

        // Call from J1939Bus::onReceive for every incoming message.
        // Returns true if the message was a PGN 60928 (caller should return early).
        // For all other messages: sends PGN 59904 to unknown sources, once per address.
        static bool onReceive(const CAN_message_t& msg);

    private:
        static AppData* _appData;
        static FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>* _can;

        static void sendClaimRequest(uint8_t destAddr);
};

#endif // J1939_SOURCE_ADDRESS_HANDLER_H
