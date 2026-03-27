# PCI Odometer + Free Data Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add PCM odometer reading via PCI diagnostic request and extract engine load, oil pressure, and intake air temp from PCI messages already being received.

**Architecture:** PciBus becomes read-write. New `requestOdometer()` method sends J1850 VPW diagnostic requests from `loop()`. Three new AppData fields populated from existing PCI message handlers. Odometer flows through AppData into J1939 PGN 65248 broadcast automatically.

**Tech Stack:** C++ / Arduino / Teensy MicroMod / J1850VPWCore / FlexCAN_T4

**Spec:** `docs/superpowers/specs/2026-03-27-pci-odometer-free-data-design.md`

---

### Task 1: Add new AppData fields

**Files:**
- Modify: `src/data/app-data.h:79-83`
- Modify: `src/domain/oct-domain.cpp:11-21` (initializer)

- [ ] **Step 1: Add three new FloatValue fields to AppData struct**

In `src/data/app-data.h`, add after the existing `ambientTemp` field:

```cpp
    // Additional PCI data
    FloatValue engineLoad;      // %
    FloatValue oilPressure;     // kPa
    FloatValue intakeAirTemp;   // degrees C
```

- [ ] **Step 2: Add initializers in OctDomain**

In `src/domain/oct-domain.cpp`, add to the `AppData OctDomain::appData` initializer after `.ambientTemp`:

```cpp
    .engineLoad = {0.0f, "%", 0},
    .oilPressure = {0.0f, "kPa", 0},
    .intakeAirTemp = {0.0f, "C", 0}
```

- [ ] **Step 3: Build to verify**

Run: `pio run`
Expected: SUCCESS with no errors

- [ ] **Step 4: Commit**

```bash
git add src/data/app-data.h src/domain/oct-domain.cpp
git commit -m "add AppData fields for engine load, oil pressure, intake air temp"
```

---

### Task 2: Extract free data from existing PCI messages

**Files:**
- Modify: `src/data/pci-bus.cpp:42-54` (0x10 handler)
- Modify: `src/data/pci-bus.cpp:58-69` (0xC0 handler)

- [ ] **Step 1: Add engine load decode to 0x10 handler**

In `src/data/pci-bus.cpp`, inside the `PCI_MSG_PCM_ENGINE` case, after the vehicleSpeed update (line 52) and before `break`:

```cpp
            // Byte 5: Engine load (raw 0-255 → 0-100%)
            float loadPct = (message[5] / 255.0f) * 100.0f;
            _appData->engineLoad.update(loadPct);
```

- [ ] **Step 2: Add oil pressure and intake air temp to 0xC0 handler**

In `src/data/pci-bus.cpp`, inside the `PCI_MSG_MULTI_SENSOR` case, after the batteryVoltage update (line 67) and before `break`:

```cpp
            // Byte 2: Oil pressure (0.5 PSI/bit → kPa)
            float oilKpa = message[2] * 0.5f * 6.895f;
            _appData->oilPressure.update(oilKpa);

            // Byte 4: Intake air temp (Celsius, offset -40)
            float intakeC = message[4] - 40.0f;
            _appData->intakeAirTemp.update(intakeC);
```

- [ ] **Step 3: Build to verify**

Run: `pio run`
Expected: SUCCESS

- [ ] **Step 4: Commit**

```bash
git add src/data/pci-bus.cpp
git commit -m "extract engine load, oil pressure, intake air temp from PCI"
```

---

### Task 3: Add odometer request, fix filter, and diagnostic response handling

**Files:**
- Modify: `src/data/pci-bus.h` (add defines, method, buffer)
- Modify: `src/data/pci-bus.cpp` (fix filter null-termination, add 0x64, request method, response handler)

- [ ] **Step 1: Update pci-bus.h**

Add diagnostic message ID define and new public method:

```cpp
#define PCI_MSG_DIAG_RESPONSE 0x64

class PciBus {
    public:
        static void setup(AppData* appData);
        static void requestOdometer();
        static volatile uint32_t _msgCount;
        static volatile uint32_t _errCount;

        // DEBUG: volatile buffer for logging diagnostic responses from loop()
        static volatile bool _diagResponseReady;
        static volatile uint8_t _diagResponseBuf[12];
        static volatile uint8_t _diagResponseLen;

    private:
        static AppData* _appData;
        static void onMessageReceived(uint8_t* message, uint8_t messageLength);
        static void onError(J1850VPW_Operations op, J1850VPW_Errors err);
};
```

- [ ] **Step 2: Add 0x64 to filter, add static variable definitions, implement requestOdometer()**

In `src/data/pci-bus.cpp`, add static definitions:

```cpp
volatile bool PciBus::_diagResponseReady = false;
volatile uint8_t PciBus::_diagResponseBuf[12] = {0};
volatile uint8_t PciBus::_diagResponseLen = 0;
```

Update the filter in `setup()` to include `PCI_MSG_DIAG_RESPONSE`:

```cpp
    uint8_t filter[] = { PCI_MSG_PCM_ENGINE, PCI_MSG_DIAG_RESPONSE, PCI_MSG_MULTI_SENSOR, PCI_MSG_AMBIENT_TEMP, PCI_MSG_VIN, 0x00 };
```

Add `requestOdometer()` method (called from loop, NOT from ISR):

```cpp
void PciBus::requestOdometer() {
    uint8_t request[] = { 0x24, 0x10, 0x3C, 0x01, 0x05, 0x00 };
    VPW.write(request, 6);
}
```

- [ ] **Step 3: Add 0x64 response handler to onMessageReceived**

In the switch statement, add before `default:`:

```cpp
        // 0x64: Diagnostic response (echo of 0x24 request)
        case PCI_MSG_DIAG_RESPONSE: {
            // Copy to volatile buffer for loop() to log/process
            _diagResponseLen = messageLength;
            for (uint8_t i = 0; i < messageLength && i < 12; i++) {
                _diagResponseBuf[i] = message[i];
            }
            _diagResponseReady = true;

            // Decode PCM odometer response: 64-10-3C-01-XX-XX-XX-XX
            if (messageLength >= 8 && message[1] == 0x10 && message[2] == 0x3C && message[3] == 0x01) {
                // 4-byte odometer at offset 4-7, big-endian (to be verified)
                uint32_t rawOdo = ((uint32_t)message[4] << 24) | ((uint32_t)message[5] << 16) |
                                  ((uint32_t)message[6] << 8) | (uint32_t)message[7];
                // DRB says raw is miles, convert to km
                float km = rawOdo * 1.609344f;
                _appData->totalVehicleDistance.update(km);
            }
            break;
        }
```

- [ ] **Step 4: Build to verify**

Run: `pio run`
Expected: SUCCESS

- [ ] **Step 5: Commit**

```bash
git add src/data/pci-bus.h src/data/pci-bus.cpp
git commit -m "add PCM odometer request and diagnostic response handler"
```

---

### Task 4: Wire odometer request into loop and update debug output

**Files:**
- Modify: `src/domain/oct-domain.cpp`

- [ ] **Step 1: Add odometer request timer and diagnostic response logging**

Add a new timer variable near the other timers:

```cpp
uint32_t lastOdometerRequest = 0;
```

In `loop()`, after the slow broadcast block and before the debug print, add:

```cpp
    // 1s: request PCM odometer via PCI diagnostic
    if (now - lastOdometerRequest >= 1000) {
        lastOdometerRequest = now;
        PciBus::requestOdometer();
    }

    // Log diagnostic responses from PCI (printed from loop, not ISR)
    if (PciBus::_diagResponseReady) {
        PciBus::_diagResponseReady = false;
        Serial.print("PCI DIAG [");
        Serial.print(PciBus::_diagResponseLen);
        Serial.print("]: ");
        for (uint8_t i = 0; i < PciBus::_diagResponseLen; i++) {
            if (PciBus::_diagResponseBuf[i] < 0x10) Serial.print("0");
            Serial.print(PciBus::_diagResponseBuf[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
```

- [ ] **Step 2: Add new fields to debug output**

In the debug print section, after the ambient temp line and before the VIN block, add:

```cpp
        float load, oilP, intakeT, odo;
        appData.engineLoad.read(load);
        appData.oilPressure.read(oilP);
        appData.intakeAirTemp.read(intakeT);
        appData.totalVehicleDistance.read(odo);

        Serial.print(" | Load:");
        Serial.print(load, 1);
        Serial.print("% | Oil:");
        Serial.print(oilP, 0);
        Serial.print("kPa | IAT:");
        Serial.print(intakeT, 1);
        Serial.print("C | Odo:");
        Serial.print(odo, 1);
        Serial.print("km");
```

- [ ] **Step 3: Build and flash**

Run: `pio run -e teensymm --target upload`
Expected: SUCCESS

- [ ] **Step 4: Verify in-truck**

Open serial monitor. Expected:
- Engine load, oil pressure, intake air temp should show real values (from existing PCI messages)
- `PCI DIAG [N]: 64 10 3C 01 ...` lines should appear if PCM responds to odometer request
- If PCM responds, `Odo:` should show a non-zero km value

- [ ] **Step 5: Commit and push**

```bash
git add src/domain/oct-domain.cpp
git commit -m "wire odometer request into loop, add new fields to debug output"
git push
```

---

### Task 5: Verify and adjust odometer byte order (in-truck)

**Files:**
- Possibly modify: `src/data/pci-bus.cpp` (odometer decode)

- [ ] **Step 1: Check PCI DIAG response**

Read the `PCI DIAG` output lines. Note the raw bytes after `64 10 3C 01`.

- [ ] **Step 2: Verify byte order**

The DRB says `data_offset=2` and `data_bytes=4`. The response format is `64-10-3C-01-XX-XX-XX-XX-CRC`. The odometer bytes are at positions 4-7. Check if the decoded km value matches the truck's dash odometer (convert km back to miles: `km / 1.609344`).

If the byte order is wrong (value way off), try little-endian:

```cpp
uint32_t rawOdo = (uint32_t)message[4] | ((uint32_t)message[5] << 8) |
                  ((uint32_t)message[6] << 16) | ((uint32_t)message[7] << 24);
```

- [ ] **Step 3: Commit fix if needed**

```bash
git add src/data/pci-bus.cpp
git commit -m "fix odometer byte order based on in-truck verification"
git push
```
