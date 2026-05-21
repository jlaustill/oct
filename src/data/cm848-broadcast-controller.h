#ifndef CM848_BROADCAST_CONTROLLER_H
#define CM848_BROADCAST_CONTROLLER_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

// Call onReceive() from CumminsBus::onReceive() to snoop EEC1 + prot-flags responses.
// Call setup() after CumminsBus::setup(), loop() from OctDomain::loop().
// Call enable() to force a re-enable pass on demand.
class Cm848BroadcastController {
    public:
        static void setup();
        static void loop();
        static void enable();
        static void onReceive(const CAN_message_t& msg);  // called from ISR
        static uint8_t state() { return _state; }
        static uint32_t msSinceEec1() { return (uint32_t)_sinceEec1; }

    private:
        static uint8_t          _state;
        static elapsedMillis    _sinceStep;
        static elapsedMillis    _sinceEec1;
        static volatile bool    _hasProtFlags;
        static volatile uint16_t _pendingProtFlags;

        static void sendEF00Service(const uint8_t* data, uint8_t len);
        static void sendProtFlagsRead();
};

#endif // CM848_BROADCAST_CONTROLLER_H
