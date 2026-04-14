#include "j1939-source-address-handler.h"

AppData* J1939SourceAddressHandler::_appData = nullptr;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>* J1939SourceAddressHandler::_can = nullptr;

void J1939SourceAddressHandler::setup(AppData* appData, FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>* can) {
    _appData = appData;
    _can = can;
}

// PGN 60928 - Address Claim (our own)
void J1939SourceAddressHandler::sendOwnClaim() {
    if (_can == nullptr) return;

    CAN_message_t msg;
    msg.id = 0x18EEFF00;
    msg.flags.extended = true;
    msg.len = 8;
    msg.buf[0] = 0x01;
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x00;
    msg.buf[4] = 0x00;
    msg.buf[5] = 0x00;  // Function = Engine
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;

    int result = _can->write(msg);
    Serial.print("J1939 Address Claim write: ");
    Serial.println(result);
}

// PGN 59904 - Request for Address Claimed, targeted to destAddr
void J1939SourceAddressHandler::sendClaimRequest(uint8_t destAddr) {
    if (_can == nullptr) return;

    CAN_message_t msg;
    msg.id = 0x18EA0000 | ((uint32_t)destAddr << 8) | 0x00;
    msg.flags.extended = true;
    msg.len = 3;
    msg.buf[0] = 0x00;  // PGN 60928 = 0x00EE00
    msg.buf[1] = 0xEE;
    msg.buf[2] = 0x00;

    _can->write(msg);
    Serial.print("J1939 claim request -> 0x");
    Serial.println(destAddr, HEX);
}

void J1939SourceAddressHandler::logNodes() {
    if (_appData == nullptr) return;

    // Snapshot the table with interrupts disabled to avoid a torn read
    // while onReceive (ISR context) may be writing concurrently.
    J1939NodeTable snapshot;
    noInterrupts();
    snapshot = _appData->nodeTable;
    interrupts();

    uint8_t count = 0;
    for (uint8_t i = 0; i < J1939_MAX_NODES; i++) {
        if (snapshot.nodes[i].inUse) count++;
    }

    Serial.print("J1939 Nodes (");
    Serial.print(count);
    Serial.println(" known):");

    for (uint8_t i = 0; i < J1939_MAX_NODES; i++) {
        const J1939Node& node = snapshot.nodes[i];
        if (!node.inUse) continue;

        Serial.print("  src:");
        Serial.print(node.sourceAddress);

        Serial.print("  NAME:");
        if (node.claimed) {
            for (uint8_t b = 0; b < 8; b++) {
                Serial.print(node.name[b]);
                if (b < 7) Serial.print(" ");
            }
        } else {
            Serial.print("-- -- -- -- -- -- -- --");
        }

        Serial.print("  claimed:");
        Serial.print(node.claimed ? "YES" : "NO ");
        Serial.print("  requested:");
        Serial.println(node.requestSent ? "YES" : "NO");
    }
}

bool J1939SourceAddressHandler::onReceive(const CAN_message_t& msg) {
    if (_appData == nullptr) return false;

    uint8_t srcAddr   = msg.id & 0xFF;
    uint8_t pduFormat = (msg.id >> 16) & 0xFF;

    if (pduFormat == 0xEE) {
        // Incoming PGN 60928 — store the node's NAME
        J1939Node* node = _appData->nodeTable.findOrAllocate(srcAddr);
        if (node != nullptr) {
            memcpy(node->name, msg.buf, 8);
            node->claimed = true;
        }
        Serial.print("J1939 ADDRESS CLAIM src:0x");
        Serial.println(srcAddr, HEX);
        return true;
    }

    // For any other message, request a claim from sources we haven't heard from yet
    J1939Node* node = _appData->nodeTable.findOrAllocate(srcAddr);
    if (node != nullptr && !node->requestSent) {
        sendClaimRequest(srcAddr);
        node->requestSent = true;
    }

    return false;
}
