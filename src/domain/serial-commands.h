#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H

#include <Arduino.h>

// Reads newline-terminated `key=value` commands from Serial and
// dispatches them to the owning domain class. Used for intentional
// one-time operations like seeding the odometer and engine hours.
//
// Commands:
//   odometer=<km>                     — set totalVehicleDistance (km)
//   engineHoursSinceRebuild=<hours>   — set since-rebuild counter
//   engineHoursTruckLifetime=<hours>  — set lifetime counter
class SerialCommands {
    public:
        static void setup();
        static void loop();

    private:
        static char _buf[64];
        static uint8_t _len;
        static void handleLine(const char* line);
};

#endif // SERIAL_COMMANDS_H
