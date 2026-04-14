#include "serial-commands.h"
#include "odometer.h"
#include "engine-hours-since-rebuild.h"
#include "engine-hours-truck-lifetime.h"
#include "data/j1939-source-address-handler.h"
#include <string.h>
#include <stdlib.h>

char SerialCommands::_buf[64];
uint8_t SerialCommands::_len = 0;

void SerialCommands::setup() {
    _len = 0;
}

void SerialCommands::loop() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        // Accept \r, \n, or \r\n / \n\r as line terminators
        if (c == '\r' || c == '\n') {
            _buf[_len] = '\0';
            if (_len > 0) handleLine(_buf);
            _len = 0;
            continue;
        }
        if (_len < sizeof(_buf) - 1) {
            _buf[_len++] = c;
        }
    }
}

void SerialCommands::handleLine(const char* line) {
    Serial.print("CMD received: '");
    Serial.print(line);
    Serial.println("'");

    const char* eq = strchr(line, '=');
    if (eq == nullptr) {
        Serial.println("CMD: no '=' found");
        return;
    }

    size_t keyLen = (size_t)(eq - line);
    const char* value = eq + 1;

    if (*value == '\0') {
        Serial.println("CMD: empty value");
        return;
    }

    if (keyLen == 8 && strncmp(line, "odometer", 8) == 0) {
        float km = (float)atof(value);
        Odometer::setAndSave(km);
        Serial.print("CMD: odometer set to ");
        Serial.print(Odometer::km(), 3);
        Serial.println(" km");
        return;
    }

    if (keyLen == 23 && strncmp(line, "engineHoursSinceRebuild", 23) == 0) {
        float hours = (float)atof(value);
        EngineHoursSinceRebuild::setAndSave(hours);
        Serial.print("CMD: engineHoursSinceRebuild set to ");
        Serial.print(hours, 3);
        Serial.println(" hrs");
        return;
    }

    if (keyLen == 24 && strncmp(line, "engineHoursTruckLifetime", 24) == 0) {
        float hours = (float)atof(value);
        EngineHoursTruckLifetime::setAndSave(hours);
        Serial.print("CMD: engineHoursTruckLifetime set to ");
        Serial.print(hours, 3);
        Serial.println(" hrs");
        return;
    }

    if (keyLen == 3 && strncmp(line, "log", 3) == 0) {
        if (strcmp(value, "J1939Nodes") == 0) {
            J1939SourceAddressHandler::logNodes();
        } else if (strcmp(value, "help") == 0) {
            Serial.println("Serial commands:");
            Serial.println("  odometer=<km>                    — set total vehicle distance");
            Serial.println("  engineHoursSinceRebuild=<hours>  — set since-rebuild counter");
            Serial.println("  engineHoursTruckLifetime=<hours> — set lifetime counter");
            Serial.println("  log=J1939Nodes                   — print known J1939 nodes");
            Serial.println("  log=help                         — print this help");
        } else {
            Serial.print("CMD: unknown log target '");
            Serial.print(value);
            Serial.println("'");
        }
        return;
    }

    Serial.print("CMD: unknown key '");
    Serial.write(line, keyLen);
    Serial.print("' (len=");
    Serial.print(keyLen);
    Serial.println(")");
}
