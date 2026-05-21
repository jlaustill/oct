#include "cm848-broadcast-controller.h"
#include "cummins-bus.h"

#define CLIP_TX_ID       0x18EF00F9
#define CLIP_RX_ID       0x18EFF900
#define EEC1_CAN_ID      0x0CF00400   // PGN 61444, priority 3, src=ECU(0x00)
#define PROT_FLAGS_ADDR  0x0040ADEE   // protection_condition_flags — bits 2/3 gate broadcasts

#define STEP_WAIT_MS     200    // inter-service delay (ECU needs time to process)
#define PROT_TIMEOUT_MS  300    // max wait for 0x4B prot-flags response
#define EEC1_STALE_MS    1000   // watchdog: re-check if EEC1 absent this long

// States
#define BC_IDLE        0
#define BC_WAIT_0A     1
#define BC_WAIT_07     2
#define BC_WAIT_05     3
#define BC_ENABLED     4
#define BC_CHECK_PROT  5

uint8_t           Cm848BroadcastController::_state = BC_IDLE;
elapsedMillis     Cm848BroadcastController::_sinceStep;
elapsedMillis     Cm848BroadcastController::_sinceEec1;
volatile bool     Cm848BroadcastController::_hasProtFlags = false;
volatile uint16_t Cm848BroadcastController::_pendingProtFlags = 0;

void Cm848BroadcastController::setup() {
    _state        = BC_IDLE;
    _hasProtFlags = false;
    _sinceStep    = 0;
    _sinceEec1    = 0;
}

void Cm848BroadcastController::enable() {
    _state = BC_IDLE;
}

void Cm848BroadcastController::loop() {
    switch (_state) {
        case BC_IDLE: {
            uint8_t svc[] = {0x0a};
            sendEF00Service(svc, 1);
            _sinceStep = 0;
            _state = BC_WAIT_0A;
            Serial.println("CM848BC: [1/3] 0x0a (initJ1939MessageBuffers)");
            break;
        }

        case BC_WAIT_0A:
            if (_sinceStep >= STEP_WAIT_MS) {
                uint8_t svc[] = {0x07};
                sendEF00Service(svc, 1);
                _sinceStep = 0;
                _state = BC_WAIT_07;
                Serial.println("CM848BC: [2/3] 0x07 (enableJ1939Output)");
            }
            break;

        case BC_WAIT_07:
            if (_sinceStep >= STEP_WAIT_MS) {
                uint8_t svc[] = {0x05, 0x01, 0x01};
                sendEF00Service(svc, 3);
                _sinceStep = 0;
                _state = BC_WAIT_05;
                Serial.println("CM848BC: [3/3] 0x05 01 01 (setProtectionConditionFlags)");
            }
            break;

        case BC_WAIT_05:
            if (_sinceStep >= STEP_WAIT_MS) {
                _sinceEec1 = 0;
                _state = BC_ENABLED;
                Serial.println("CM848BC: enabled — watching EEC1");
            }
            break;

        case BC_ENABLED:
            if (_sinceEec1 >= EEC1_STALE_MS) {
                sendProtFlagsRead();
                _sinceStep = 0;
                _hasProtFlags = false;
                _state = BC_CHECK_PROT;
            }
            break;

        case BC_CHECK_PROT:
            if (_hasProtFlags) {
                _hasProtFlags = false;
                uint16_t flags = _pendingProtFlags;
                if ((flags & 0x000C) == 0x000C) {
                    // Enable still in effect — engine off or not yet broadcasting
                    _sinceEec1 = 0;
                    _state = BC_ENABLED;
                    Serial.print("CM848BC: prot=0x");
                    Serial.print(flags, HEX);
                    Serial.println(" OK (engine off?)");
                } else {
                    // Enable was lost — restart sequence
                    _state = BC_IDLE;
                    Serial.print("CM848BC: prot=0x");
                    Serial.print(flags, HEX);
                    Serial.println(" — lost, re-enabling");
                }
            } else if (_sinceStep >= PROT_TIMEOUT_MS) {
                _state = BC_IDLE;
                Serial.println("CM848BC: prot read timeout — re-enabling");
            }
            break;
    }
}

// Called from CumminsBus::onReceive (ISR context) — volatile writes only.
void Cm848BroadcastController::onReceive(const CAN_message_t& msg) {
    if (msg.id == EEC1_CAN_ID) {
        _sinceEec1 = 0;
        return;
    }
    if (_state == BC_CHECK_PROT
            && msg.id == CLIP_RX_ID
            && msg.len >= 8
            && msg.buf[0] == 0x4B) {
        uint32_t addr = ((uint32_t)msg.buf[1] << 24) | ((uint32_t)msg.buf[2] << 16)
                      | ((uint32_t)msg.buf[3] << 8)  |  msg.buf[4];
        if (addr == PROT_FLAGS_ADDR && msg.buf[5] >= 2) {
            _pendingProtFlags = ((uint16_t)msg.buf[6] << 8) | msg.buf[7];
            _hasProtFlags = true;
        }
    }
}

void Cm848BroadcastController::sendEF00Service(const uint8_t* data, uint8_t len) {
    CAN_message_t msg;
    msg.id = CLIP_TX_ID;
    msg.flags.extended = true;
    msg.len = 8;
    memset(msg.buf, 0, 8);
    for (uint8_t i = 0; i < len && i < 8; i++) msg.buf[i] = data[i];
    CumminsBus::CumminsBusCan.write(msg);
}

void Cm848BroadcastController::sendProtFlagsRead() {
    CAN_message_t msg;
    msg.id = CLIP_TX_ID;
    msg.flags.extended = true;
    msg.len = 8;
    msg.buf[0] = 0x4A;
    msg.buf[1] = (PROT_FLAGS_ADDR >> 24) & 0xFF;
    msg.buf[2] = (PROT_FLAGS_ADDR >> 16) & 0xFF;
    msg.buf[3] = (PROT_FLAGS_ADDR >> 8)  & 0xFF;
    msg.buf[4] =  PROT_FLAGS_ADDR        & 0xFF;
    msg.buf[5] = 0x02;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    CumminsBus::CumminsBusCan.write(msg);
}
