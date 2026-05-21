#include "cummins-bus.h"

// CLIP Service 0x4A: Cummins proprietary memory-read protocol.
// Tool address 0xF9 (Calterm/Insite), no address claiming required on this bus.
#define CLIP_REQUEST_ID   0x18EF00F9  // priority 6, PGN 0xEF00, dest=ECU(0x00), src=tool(0xF9)
#define CLIP_RESPONSE_ID  0x18EFF900  // priority 6, PGN 0xEF00, dest=tool(0xF9), src=ECU(0x00)

#define CLIP_ADDR_APPS    0x0040a37a  // throttle_position_filtered (word)
#define CLIP_ADDR_LOAD    0x0040B1F0  // engine_load_percent (word, raw = %)

#define CLIP_POLL_MS      100         // send one request every 100ms

// Discovery addresses polled when raw log is active (includes APPS and LOAD)
static const uint32_t DISC_ADDRS[] = {
    0x0040BD86,  // throttle_position_demand (word)
    0x0040BD9E,  // throttle_position_secondary (word)
    0x003FA764,  // sensor_diag_accel_shifted_a (dword high word, verified-readable range)
    0x0040BD4E,  // engine_load_input_value (word)
    0x0040B1F0,  // engine_load_percent (word, reference)
};
static const uint8_t DISC_LENS[] = { 2, 2, 2, 2, 2 };
static const uint8_t DISC_COUNT = sizeof(DISC_ADDRS) / sizeof(DISC_ADDRS[0]);

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CumminsBus::CumminsBusCan;
volatile AppData* CumminsBus::_appData = nullptr;
volatile uint32_t CumminsBus::msgCount = 0;
elapsedMillis CumminsBus::sinceLastRx;
elapsedMillis CumminsBus::_sincePoll;
uint8_t CumminsBus::_pollIdx = 0;
bool CumminsBus::_rawLogActive = false;
elapsedMillis CumminsBus::_rawLogTimer;
uint32_t CumminsBus::_rawLogDurationMs = 0;

void CumminsBus::setup(AppData* appData) {
    _appData = appData;
    _pollIdx = 0;
    _sincePoll = 0;

    CumminsBusCan.begin();
    CumminsBusCan.setBaudRate(250000);
    CumminsBusCan.setMaxMB(16);
    CumminsBusCan.enableFIFO();
    CumminsBusCan.enableFIFOInterrupt();
    CumminsBusCan.onReceive(onReceive);
}

void CumminsBus::loop() {
    CumminsBusCan.events();

    if (_sincePoll >= CLIP_POLL_MS) {
        _sincePoll = 0;
        if (_rawLogActive) {
            sendClipRequest(DISC_ADDRS[_pollIdx], DISC_LENS[_pollIdx]);
            _pollIdx = (_pollIdx + 1) % DISC_COUNT;
        } else {
            sendClipRequest(_pollIdx == 0 ? CLIP_ADDR_APPS : CLIP_ADDR_LOAD, 2);
            _pollIdx = (_pollIdx + 1) % 2;
        }
    }
}

void CumminsBus::requestPgn(uint32_t pgn) {
    CAN_message_t msg;
    msg.id = 0x18EAFFFA;
    msg.flags.extended = true;
    msg.len = 3;
    msg.buf[0] = pgn & 0xFF;
    msg.buf[1] = (pgn >> 8) & 0xFF;
    msg.buf[2] = (pgn >> 16) & 0xFF;
    CumminsBusCan.write(msg);
}

void CumminsBus::startRawLog(uint32_t durationMs) {
    _rawLogDurationMs = durationMs;
    _rawLogTimer = 0;
    _rawLogActive = true;
    _pollIdx = 0;
    Serial.println("Cummins discovery log — polling 5 throttle candidates for 10s:");
    Serial.println("  [0] 0x0040BD86 throttle_position_demand (word)");
    Serial.println("  [1] 0x0040BD9E throttle_position_secondary (word)");
    Serial.println("  [2] 0x003FA764 sensor_diag_accel_shifted_a (dword high)");
    Serial.println("  [3] 0x0040BD4E engine_load_input_value (word)");
    Serial.println("  [4] 0x0040B1F0 engine_load_percent (word, reference)");
}

// CLIP request: [0x4A][addr BE, 4 bytes][len][0x00][0x00]
void CumminsBus::sendClipRequest(uint32_t addr, uint8_t len) {
    CAN_message_t msg;
    msg.id = CLIP_REQUEST_ID;
    msg.flags.extended = true;
    msg.len = 8;
    msg.buf[0] = 0x4A;
    msg.buf[1] = (addr >> 24) & 0xFF;
    msg.buf[2] = (addr >> 16) & 0xFF;
    msg.buf[3] = (addr >> 8)  & 0xFF;
    msg.buf[4] =  addr        & 0xFF;
    msg.buf[5] = len;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    CumminsBusCan.write(msg);
}

// CLIP response: [0x4B][addr echo, 4 bytes BE][len][data bytes...]
void CumminsBus::onReceive(const CAN_message_t &msg) {
    msgCount++;
    sinceLastRx = 0;

    if (_rawLogActive) {
        if (_rawLogTimer >= _rawLogDurationMs) {
            _rawLogActive = false;
            _pollIdx = 0;
            Serial.println("Cummins discovery log done");
        } else if (msg.id == CLIP_RESPONSE_ID && msg.len >= 7 && msg.buf[0] == 0x4B) {
            uint32_t addr = ((uint32_t)msg.buf[1] << 24) | ((uint32_t)msg.buf[2] << 16)
                          | ((uint32_t)msg.buf[3] << 8)  |  msg.buf[4];
            uint8_t dlen = msg.buf[5];
            Serial.print("CLIP 0x");
            if (addr < 0x10000000) Serial.print("0");
            Serial.print(addr, HEX);
            Serial.print(" len=");
            Serial.print(dlen);
            Serial.print(" data:");
            for (uint8_t i = 0; i < dlen && (6 + i) < msg.len; i++) {
                Serial.print(" ");
                if (msg.buf[6 + i] < 0x10) Serial.print("0");
                Serial.print(msg.buf[6 + i], HEX);
            }
            Serial.println();
        }
        return;
    }

    if (msg.id != CLIP_RESPONSE_ID || msg.len < 8 || msg.buf[0] != 0x4B) return;
    if (_appData == nullptr) return;

    uint32_t addr = ((uint32_t)msg.buf[1] << 24) | ((uint32_t)msg.buf[2] << 16)
                  | ((uint32_t)msg.buf[3] << 8)  |  msg.buf[4];
    uint8_t len = msg.buf[5];
    if (len < 1) return;

    if (addr == CLIP_ADDR_APPS && len >= 2) {
        uint16_t raw = ((uint16_t)msg.buf[6] << 8) | msg.buf[7];
        _appData->appsPercent.update((float)raw * 100.0f / 65535.0f);
    } else if (addr == CLIP_ADDR_LOAD && len >= 2) {
        uint16_t raw = ((uint16_t)msg.buf[6] << 8) | msg.buf[7];
        _appData->engineLoad.update((float)raw);
    }
}
