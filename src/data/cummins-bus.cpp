#include "cummins-bus.h"
#include "cm848-broadcast-controller.h"
#include "cm848-j1939-receiver.h"
#include "j1939-bus.h"

// CLIP Service 0x4A: Cummins proprietary memory-read protocol.
// Tool address 0xF9 (Calterm/Insite), no address claiming required on this bus.
#define CLIP_REQUEST_ID   0x18EF00F9  // priority 6, PGN 0xEF00, dest=ECU(0x00), src=tool(0xF9)
#define CLIP_RESPONSE_ID  0x18EFF900  // priority 6, PGN 0xEF00, dest=tool(0xF9), src=ECU(0x00)

#define CLIP_ADDR_APPS    0x0040a37a  // throttle_position_filtered (word)
#define CLIP_ADDR_LOAD    0x0040B1F0  // engine_load_percent (word, raw = %)

#define CLIP_POLL_MS      100         // send one request every 100ms
#define WAKEUP_INTERVAL_MS 1000
#define TP_TIMEOUT_MS      500

#define CLIP_ADDR_TIMER        0x0040A64C  // rolling security timer (dword)
#define TP_CM_TX_ID            0x18EC00F9  // J1939 TP CM outbound (RTS / CTS / EOM)
#define TP_DT_TX_ID            0x18EB00F9  // J1939 TP DT outbound (tool → ECU)
#define TP_CM_RX_ID            0x18ECF900  // J1939 TP CM from ECU
#define TP_DT_RX_ID            0x18EBF900  // J1939 TP DT from ECU (ECU → tool)

// Wakeup state machine states (ordered: IDLE < all intermediate < ARMED)
#define WS_IDLE                0
#define WS_READING_TIMER       1  // sent CLIP timer req, waiting for ECU RTS
#define WS_READING_TIMER_RX_DT 2  // sent CTS, collecting ECU DT frames
#define WS_WAITING_CTS         3  // sent our RTS, waiting for ECU CTS
#define WS_WAITING_EOM         4  // sent DTs, waiting for ECU EOM ACK
#define WS_ARMED               5  // handshake done — send 0x16 every second

#define WA_NONE           0
#define WA_ECU_RTS        1  // ECU sent RTS for timer TP transfer
#define WA_TIMER_RXED     2  // all ECU DT frames received, timer bytes captured
#define WA_CTS_RXED       3  // ECU acknowledged our RTS with CTS
#define WA_EOM_RXED       4  // ECU acknowledged our DTs with EOM

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
elapsedMillis CumminsBus::_sinceWakeup;
uint8_t CumminsBus::_wakeupState = WS_IDLE;
uint8_t CumminsBus::_timerBytes[3] = {0, 0, 0};
uint32_t CumminsBus::_wakeupStateDelay = 0;
volatile uint8_t CumminsBus::_pendingAction = WA_NONE;

void CumminsBus::setup(AppData* appData) {
    _appData = appData;
    _pollIdx = 0;
    _sincePoll = 0;
    _wakeupState = WS_IDLE;
    _pendingAction = WA_NONE;
    _sinceWakeup = 0;

    CumminsBusCan.begin();
    CumminsBusCan.setBaudRate(250000);
    CumminsBusCan.setMaxMB(16);
    CumminsBusCan.enableFIFO();
    CumminsBusCan.enableFIFOInterrupt();
    CumminsBusCan.onReceive(onReceive);
}

void CumminsBus::loop() {
    CumminsBusCan.events();

    if (_pendingAction != WA_NONE) {
        advanceWakeup();
    }

    if (_wakeupState == WS_IDLE && _sinceWakeup >= WAKEUP_INTERVAL_MS) {
        _sinceWakeup = 0;
        sendClipRequest(CLIP_ADDR_TIMER, 4);
        _wakeupState = WS_READING_TIMER;
    } else if (_wakeupState == WS_ARMED && _sinceWakeup >= WAKEUP_INTERVAL_MS) {
        _wakeupState = WS_IDLE;
        _sinceWakeup = 0;
    } else if (_wakeupState > WS_IDLE && _wakeupState < WS_ARMED
               && _sinceWakeup >= TP_TIMEOUT_MS) {
        _wakeupState = WS_IDLE;
        _sinceWakeup = 0;
        _pendingAction = WA_NONE;
        Serial.println("Cummins: handshake timeout, retrying");
    }

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

// CTS in reply to ECU's RTS: allow 2 packets starting at seq 1
void CumminsBus::sendCTS() {
    CAN_message_t msg;
    msg.id = TP_CM_TX_ID;
    msg.flags.extended = true;
    msg.len = 8;
    msg.buf[0] = 0x11;  // CTS
    msg.buf[1] = 0x02;  // allow 2 packets
    msg.buf[2] = 0x01;  // next expected sequence number
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0x00;  // PGN 0xEF00
    msg.buf[6] = 0xEF;
    msg.buf[7] = 0x00;
    CumminsBusCan.write(msg);
}

// EOM ACK after receiving all ECU DT frames (10 bytes, 2 packets)
void CumminsBus::sendEomAck() {
    CAN_message_t msg;
    msg.id = TP_CM_TX_ID;
    msg.flags.extended = true;
    msg.len = 8;
    msg.buf[0] = 0x13;  // EOM ACK
    msg.buf[1] = 0x0A;  // total bytes (10) LSB
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x02;  // total packets
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0x00;  // PGN 0xEF00
    msg.buf[6] = 0xEF;
    msg.buf[7] = 0x00;
    CumminsBusCan.write(msg);
}

void CumminsBus::sendRTS() {
    CAN_message_t msg;
    msg.id = TP_CM_TX_ID;
    msg.flags.extended = true;
    msg.len = 8;
    msg.buf[0] = 0x10;  // RTS
    msg.buf[1] = 0x0B;  // total bytes LSB (11)
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x02;  // 2 packets
    msg.buf[4] = 0xFF;  // max packets per CTS
    msg.buf[5] = 0x00;  // PGN 0xEF00 LSB
    msg.buf[6] = 0xEF;
    msg.buf[7] = 0x00;
    CumminsBusCan.write(msg);
}

// cm848_copyDataBuffer inverse cipher.
// The ECU transforms our 10-byte input before comparing against "ABCDEF" + timer.
// We invert the transform so the ECU sees exactly what it expects.
static void cm848InverseCipher(const uint8_t target[10], uint8_t inp[10]) {
    static const uint8_t SCATTER_IDX[10] = {4, 5, 9, 3, 7, 0, 2, 1, 8, 6};
    static const uint8_t XOR_IDX[10]     = {6, 8, 1, 2, 0, 7, 3, 9, 5, 4};
    static const uint8_t BIT_DEST_BYTE[80] = {
        0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3,
        3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6,
        6, 7, 7, 7, 7, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9,
        9, 9, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 5, 5, 5, 5,
        4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0
    };
    static const uint8_t BIT_DEST_BIT[80] = {
        7, 5, 4, 3, 0, 7, 5, 2, 1, 0, 6, 5, 2, 0, 7, 6,
        3, 2, 0, 7, 3, 2, 1, 0, 6, 5, 4, 3, 7, 6, 4, 3,
        2, 6, 5, 2, 1, 6, 5, 4, 3, 1, 7, 4, 3, 1, 0, 2,
        5, 6, 0, 2, 7, 0, 3, 4, 7, 0, 1, 5, 0, 1, 2, 7,
        4, 5, 6, 1, 4, 5, 1, 3, 4, 7, 3, 4, 6, 1, 2, 6
    };

    uint8_t local_18[10];
    for (int j = 0; j < 10; j++)
        local_18[j] = target[SCATTER_IDX[j]];
    for (int i = 9; i >= 0; i--)
        local_18[i] ^= local_18[XOR_IDX[i]];
    for (int i = 0; i < 10; i++) inp[i] = 0;
    for (int k = 0; k < 80; k++) {
        uint8_t byte_idx = k / 8;
        uint8_t bit_pos  = 7 - (k % 8);
        if (local_18[byte_idx] & (1 << bit_pos))
            inp[BIT_DEST_BYTE[k]] |= (1 << BIT_DEST_BIT[k]);
    }
}

void CumminsBus::sendDTs() {
    // Build target: ECU expects cipher(our_input) = [ABCDEF, T0, T1, T2, 0x00]
    uint8_t target[10] = {
        0x41, 0x42, 0x43, 0x44, 0x45, 0x46,
        _timerBytes[0], _timerBytes[1], _timerBytes[2], 0x00
    };
    uint8_t payload[10];
    cm848InverseCipher(target, payload);

    Serial.print("Cummins: sendDTs payload:");
    for (int i = 0; i < 10; i++) { Serial.print(" "); Serial.print(payload[i], HEX); }
    Serial.println();

    CAN_message_t dt1, dt2;

    dt1.id = TP_DT_TX_ID;
    dt1.flags.extended = true;
    dt1.len = 8;
    dt1.buf[0] = 0x01;
    dt1.buf[1] = 0x16;  // service byte
    dt1.buf[2] = payload[0]; dt1.buf[3] = payload[1]; dt1.buf[4] = payload[2];
    dt1.buf[5] = payload[3]; dt1.buf[6] = payload[4]; dt1.buf[7] = payload[5];
    CumminsBusCan.write(dt1);

    dt2.id = TP_DT_TX_ID;
    dt2.flags.extended = true;
    dt2.len = 8;
    dt2.buf[0] = 0x02;
    dt2.buf[1] = payload[6];
    dt2.buf[2] = payload[7];
    dt2.buf[3] = payload[8];
    dt2.buf[4] = payload[9];
    dt2.buf[5] = 0xFF; dt2.buf[6] = 0xFF; dt2.buf[7] = 0xFF;
    CumminsBusCan.write(dt2);
}

void CumminsBus::advanceWakeup() {
    uint8_t action = _pendingAction;
    _pendingAction = WA_NONE;

    switch (_wakeupState) {
        case WS_READING_TIMER:
            if (action == WA_ECU_RTS) {
                sendCTS();
                _wakeupState = WS_READING_TIMER_RX_DT;
                _sinceWakeup = 0;
            }
            break;
        case WS_READING_TIMER_RX_DT:
            if (action == WA_TIMER_RXED) {
                sendEomAck();
                sendRTS();
                _wakeupState = WS_WAITING_CTS;
                _sinceWakeup = 0;
            }
            break;
        case WS_WAITING_CTS:
            if (action == WA_CTS_RXED) {
                sendDTs();
                _wakeupState = WS_WAITING_EOM;
                _sinceWakeup = 0;
            }
            break;
        case WS_WAITING_EOM:
            if (action == WA_EOM_RXED) {
                _wakeupState = WS_ARMED;
                _sinceWakeup = 0;
                Serial.println("Cummins: security handshake OK — armed");
            }
            break;
        default:
            break;
    }
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

    Cm848BroadcastController::onReceive(msg);

    // Parse J1939 broadcasts from ECU and relay them to the J1939 bus (CAN3)
    if (Cm848J1939Receiver::onReceive(msg, _appData)) {
        J1939Bus::J1939BusCan.write(msg);
        return;
    }

    // Only log CLIP protocol frames — ECU J1939 broadcasts flood serial at 100ms intervals
    if (msg.id == CLIP_RESPONSE_ID || msg.id == TP_CM_RX_ID || msg.id == TP_DT_RX_ID) {
        Serial.print("CUMMINS RX id=0x");
        Serial.print(msg.id, HEX);
        Serial.print(" len=");
        Serial.print(msg.len);
        Serial.print(" data:");
        for (uint8_t i = 0; i < msg.len; i++) {
            Serial.print(" ");
            if (msg.buf[i] < 0x10) Serial.print("0");
            Serial.print(msg.buf[i], HEX);
        }
        Serial.println();
    }

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

    if (msg.id == TP_CM_RX_ID && msg.len >= 1) {
        if (msg.buf[0] == 0x10 && _wakeupState == WS_READING_TIMER) {
            _pendingAction = WA_ECU_RTS;
        } else if (msg.buf[0] == 0x11 && _wakeupState == WS_WAITING_CTS) {
            _pendingAction = WA_CTS_RXED;
        } else if (msg.buf[0] == 0x13 && _wakeupState == WS_WAITING_EOM) {
            _pendingAction = WA_EOM_RXED;
        }
        return;
    }

    // ECU DT frames: reassemble timer bytes
    // DT1: [seq=1, 0x4B, addr(4), len, T0]  — T0 is at buf[7]
    // DT2: [seq=2, T1, T2, T3, 0xFF×4]
    if (msg.id == TP_DT_RX_ID && _wakeupState == WS_READING_TIMER_RX_DT && msg.len >= 2) {
        if (msg.buf[0] == 0x01 && msg.len >= 8) {
            _timerBytes[0] = msg.buf[7];
        } else if (msg.buf[0] == 0x02) {
            _timerBytes[1] = msg.buf[1];
            _timerBytes[2] = msg.buf[2];
            _pendingAction = WA_TIMER_RXED;
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
