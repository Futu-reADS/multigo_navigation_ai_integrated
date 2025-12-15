# eHMI System Reference (External Human-Machine Interface)

**Source:** /home/pankaj/eHMI_all/
**Hardware:** ESP32-S3 DevKitC-1 (N32R8 - 32MB Flash, 8MB PSRAM)
**Framework:** PlatformIO + Arduino + Dezyne
**Date:** December 15, 2025

---

## 1. System Overview

The eHMI (External Human-Machine Interface) provides visual and audio feedback to communicate robot status to pedestrians and nearby people. It consists of three main components:

1. **LED Strips** (FastLED) - Dynamic animations around robot body
2. **LED Matrix Display** (HUB75 96×48) - Icons, text, and GIF animations
3. **Audio System** (I2S + SD card) - Multi-language voice announcements

---

## 2. Hardware Platform

### ESP32-S3 DevKitC-1 Specifications

**Microcontroller:** ESP32-S3-WROOM-1-N32R8
- **CPU:** Dual-core Xtensa LX7 @ 240 MHz
- **Flash:** 32MB (Octal SPI)
- **PSRAM:** 8MB (Octal SPI)
- **USB:** Native USB OTG (CDC for serial communication)
- **Peripherals:** I2S, SPI, I2C, GPIO

**Serial Communication:**
- Baud rate: 115200
- Protocol: Text-based (STATE:{int}\n, VOLUME:{int}\n, LANGUAGE:{string}\n)
- USB device path: `/dev/ttyACM0`

---

## 3. Component Details

### 3.1 LED Strips (FastLED)

**Library:** FastLED 3.6.0
**LED Type:** WS2812B / SK6812 (configurable)
**Configuration:**
- Dual independent strips (`leds` and `leds_2`)
- Configurable LED count (NUM_LEDS, NUM_LEDS_2)
- Primary and secondary color support

**Animation Patterns:**
```cpp
// Animation structure (from LedAnimationHandlerComponent.cpp)
struct AnimationData {
    int startLed;                      // Starting LED position
    int mainLength;                    // Solid color section length
    int frontTailLength;               // Fade-in tail length
    int rearTailLength;                // Fade-out tail length
    bool isFrontTail;                  // Enable front tail
    bool isRearTail;                   // Enable rear tail
    CRGB primaryColor;                 // Main strip color
    CRGB secondaryColor;               // Secondary strip color
    uint8_t maxBrightness;             // Max brightness (0-255)
    int numberOfCycles;                // Animation repeat count
    int numberOfRotationsInOneCycle;   // Rotations per cycle
    bool isAnimationHideAtEnd;         // Hide animation at end
    bool isLedStartFromFront;          // Direction flag
};
```

**Brightness Fading:**
- Quadratic easing for smooth fades
- Front tail: `brightness = maxBrightness × (position/length)²`
- Rear tail: `brightness = maxBrightness × (remaining/length)²`

**Use Cases:**
- **ACTIVE state:** Blue moving animation (forward motion indicator)
- **WAITING state:** Yellow pulsing (robot waiting)
- **ERROR state:** Red blinking (warning/error condition)
- **OBSTACLE_DETECTED:** Orange scanning pattern (obstacle awareness)

---

### 3.2 LED Matrix Display (HUB75)

**Library:** ESP32-HUB75-MatrixPanel-DMA 3.0.10
**Display Specifications:**
- Resolution: 96×48 pixels (2 panels of 96×24 chained)
- Interface: HUB75 (parallel RGB data)
- Panel configuration: 2 rows × 1 column
- Pixel pitch: P3 or P4 (configurable)

**Pin Mapping (from constants.h lines 29-42):**
```cpp
// RGB Data Lines
LED_R1_PIN = 4    // Red data (upper half)
LED_G1_PIN = 5    // Green data (upper half)
LED_B1_PIN = 6    // Blue data (upper half)
LED_R2_PIN = 7    // Red data (lower half)
LED_G2_PIN = 15   // Green data (lower half)
LED_B2_PIN = 16   // Blue data (lower half)

// Address Lines
LED_A_PIN = 17    // Row address A
LED_B_PIN = 18    // Row address B
LED_C_PIN = 8     // Row address C
LED_D_PIN = 3     // Row address D
LED_E_PIN = 46    // Row address E

// Control Lines
LED_CLK_PIN = 41  // Clock
LED_LAT_PIN = 40  // Latch
LED_OE_PIN = 39   // Output enable
```

**Display Modes:**
1. **Static Icons** - State-based icon display
2. **Text Display** - Multi-language text messages
3. **GIF Animation** - Animated content from SD card

**GIF Playback:**
- Source: SD card (/gifs/ directory)
- Format: Animated GIF files
- Playback delay: 10ms between frames (configurable)
- Automatic looping with retry logic (MAX_RETRIES = 3)

---

### 3.3 Audio System (I2S)

**Library:** ESP32-audioI2S (Schreibfaul1)
**Audio Output:** I2S DAC or MAX98357A amplifier
**Audio Source:** SD card (MP3 files)

**Pin Mapping:**
```cpp
I2S_BCLK_PIN    // Bit clock
I2S_LRC_PIN     // Left/Right clock (Word select)
I2S_DOUT_PIN    // Data out
```

**Audio Playback Features:**
- Multi-language support: EN (English), JP (Japanese), ZH (Chinese)
- Volume control: 0-10 (mapped to internal 0-20)
- Repeat playback with configurable delay and count
- FreeRTOS task on Core 1 (dedicated audio processing)
- SD card mutex protection (shared with GIF handler)

**Audio File Structure (SD card):**
```
/audio/
├── EN/
│   ├── active.mp3
│   ├── obstacle_detected.mp3
│   ├── waiting.mp3
│   ├── error.mp3
│   └── delivery_success.mp3
├── JP/
│   └── [same structure]
└── ZH/
    └── [same structure]
```

---

### 3.4 SD Card Interface

**SPI Configuration (from constants.h lines 5-8):**
```cpp
MOSI_PIN = 11    // SPI MOSI
MISO_PIN = 13    // SPI MISO
SCK_PIN = 12     // SPI clock
SS_PIN = 10      // Chip select
```

**Data Mode:** SPI_MODE0
**File System:** FAT32
**Mutex Protection:** FreeRTOS mutex for concurrent access

**SD Card Content:**
```
/
├── audio/
│   ├── EN/  (English audio files)
│   ├── JP/  (Japanese audio files)
│   └── ZH/  (Chinese audio files)
└── gifs/
    ├── active.gif
    ├── obstacle_detected.gif
    ├── waiting.gif
    └── error.gif
```

---

## 4. Communication Protocol

### Serial Interface (ROS 2 Node → ESP32)

**Protocol:** Text-based, newline-terminated
**Baud Rate:** 115200
**Format:**

```
STATE:{state_number}\n
VOLUME:{volume_level}\n
LANGUAGE:{language_code}\n
```

**Example Messages:**
```
STATE:1\n           // Set state to ACTIVE
VOLUME:7\n          // Set volume to 7 (out of 10)
LANGUAGE:EN\n       // Set language to English
```

**State Number Mapping (from main.cpp lines 58-76):**
```cpp
1  → ACTIVE               // Robot moving
4  → OBSTACLE_DETECTED    // Obstacle in path
5  → ON                   // System powered on
6  → OFF                  // System shutting down
7  → WAITING              // Waiting for action
8  → ERROR                // Error condition
9  → IDLE                 // Idle/standby
10 → DELIVERY_SUCCESS     // Task completed
14 → SET_GOAL             // Goal being set
15 → GOAL_SET             // Goal confirmed
17 → CHARGING_33          // Battery 33% (charging)
18 → CHARGING_67          // Battery 67% (charging)
19 → CHARGING_99          // Battery 99% (charging)
20 → CHARGING_100         // Battery 100% (fully charged)
999 → WARNING             // Warning condition
```

**Volume Mapping:**
```cpp
// ROS message: 0-10
// Internal ESP32: 0-20
int internal_volume = map(ros_volume, 0, 10, 0, 20);
```

**Language Codes:**
- `EN` - English
- `JP` - Japanese
- `ZH` - Chinese (default fallback)

---

## 5. Software Architecture

### 5.1 Dezyne Formal Modeling

**Purpose:** State machine verification and code generation
**Benefits:**
- Formally verified state transitions
- No deadlocks or race conditions
- Automatic C++ component generation

**Generated Components:**
- `EHmiSystem.hh` - Main system coordinator
- State machine logic with proven correctness

### 5.2 FreeRTOS Tasks

**Task 1: Audio Task (Core 1)**
```cpp
xTaskCreatePinnedToCore(
    AudioHandlerComponent::audioTask,
    "AudioTask",
    8192,        // Stack size
    this,        // Parameter
    2,           // Priority
    &audioTaskHandle,
    1            // Core 1
);
```

**Task 2: Update Loop (Main loop)**
- LED animation updates
- GIF playback
- Serial message processing
- State transitions

**Concurrency:**
- Audio runs on Core 1 (dedicated)
- All other tasks on Core 0
- SD card mutex prevents conflicts

---

## 6. State-Based Behavior Patterns

### State Definitions (SystemState.h)

```cpp
enum class State {
    ON,                   // System power on
    IDLE,                 // Standby mode
    SET_GOAL,             // Goal setting in progress
    GOAL_SET,             // Goal confirmed
    ACTIVE,               // Robot in motion
    OBSTACLE_DETECTED,    // Obstacle in path
    WAITING,              // Waiting for action
    ERROR,                // Error condition
    DELIVERY_SUCCESS,     // Task completed
    OFF,                  // System shutdown
    CHARGING_33,          // Charging (33%)
    CHARGING_67,          // Charging (67%)
    CHARGING_99,          // Charging (99%)
    CHARGING_100,         // Charging (100%)
    NONE                  // Undefined state
};
```

### Example State Behaviors (ParcelPal)

| State | LED Strip | LED Matrix | Audio |
|-------|-----------|------------|-------|
| **IDLE** | Green pulsing | Idle icon | None |
| **SET_GOAL** | Blue rotating | "Setting goal" text | Beep |
| **ACTIVE** | Blue moving forward | Arrow icon | None |
| **OBSTACLE_DETECTED** | Orange scanning | Warning icon | "Obstacle detected" |
| **WAITING** | Yellow pulsing | Clock icon | "Waiting" |
| **ERROR** | Red blinking | Error icon | "Error occurred" |
| **DELIVERY_SUCCESS** | Green chase | Success icon | "Delivery complete" |

---

## 7. Wheelchair Robot Adaptation

### New States Needed

**Add wheelchair-specific states:**
```cpp
enum class State {
    // ... (existing states)
    APPROACHING_WHEELCHAIR,   // Navigating to wheelchair
    SEARCHING_ARUCO,          // Searching for docking marker
    DOCKING,                  // Precision docking in progress
    WHEELCHAIR_ATTACHED,      // Wheelchair successfully attached
    TRANSPORTING,             // Transporting passenger
    ARRIVED_DESTINATION,      // Arrived at dropoff location
    UNDOCKING,                // Detaching from wheelchair
    WHEELCHAIR_DETACHED,      // Wheelchair successfully detached
};
```

**State Number Extensions:**
```cpp
// Add to convertIntToState() in main.cpp
21 → APPROACHING_WHEELCHAIR
22 → SEARCHING_ARUCO
23 → DOCKING
24 → WHEELCHAIR_ATTACHED
25 → TRANSPORTING
26 → ARRIVED_DESTINATION
27 → UNDOCKING
28 → WHEELCHAIR_DETACHED
```

### New Behavior Patterns

| State | LED Strip | LED Matrix | Audio (EN) |
|-------|-----------|------------|------------|
| **APPROACHING_WHEELCHAIR** | Blue slow pulse | "Approaching" + distance | "Approaching wheelchair" |
| **SEARCHING_ARUCO** | Cyan scanning | Camera icon + searching | None |
| **DOCKING** | Green moving pattern | Docking animation (GIF) | "Docking in progress, please wait" |
| **WHEELCHAIR_ATTACHED** | Green solid | Checkmark icon | "Wheelchair attached successfully" |
| **TRANSPORTING** | Smooth blue breathing | "Transporting" + ETA | None (quiet during transport) |
| **ARRIVED_DESTINATION** | Green chase | Arrival icon | "Arrived at destination" |
| **UNDOCKING** | Green reverse pattern | Undocking animation | "Undocking in progress" |
| **WHEELCHAIR_DETACHED** | Green pulse → Idle | "Ready" icon | "Wheelchair detached, ready" |

### Safety-Specific Audio Messages

**Critical Announcements:**
```
/audio/EN/safety/
├── stand_clear.mp3           - "Please stand clear, vehicle moving"
├── docking_warning.mp3       - "Docking in progress, please wait"
├── passenger_onboard.mp3     - "Passenger onboard, please keep distance"
├── emergency_stop.mp3        - "Emergency stop activated"
├── low_battery_warning.mp3   - "Low battery, returning to base"
```

**Repeat with higher priority during critical states:**
- DOCKING: "Please stand clear" every 5 seconds
- TRANSPORTING: "Passenger onboard" once at start
- EMERGENCY: "Emergency stop" continuous until cleared

---

## 8. ROS 2 Integration

### ROS 2 Node (ehmi_state_publisher)

**Published Topics:**
```
/ehmi/status_feedback    # Int32 - Confirmation of state change
```

**Subscribed Topics:**
```
/ehmi/status             # Int32 - State command
/ftd_master/interface/volume    # Int32 - Volume control
/ftd_master/interface/language  # String - Language selection
```

**Node Implementation (C++):**
```cpp
// From ParcelPal: ehmi_state_publisher.cpp
class ehmiState_Publisher : public rclcpp::Node {
    // Subscribes to /ehmi/status
    // Sends STATE:{msg->data}\n via serial to ESP32
    // Handles reconnection automatically
    // Resends last state every 1 second (reliability)
};
```

---

## 9. Development Workflow

### PlatformIO Build & Upload

**Build:**
```bash
cd /home/pankaj/eHMI_all
pio run
```

**Upload to ESP32:**
```bash
pio run --target upload --upload-port /dev/ttyUSB0
```

**Monitor Serial Output:**
```bash
pio device monitor --port /dev/ttyACM0 --baud 115200
```

### Dezyne Code Generation

**Generate components from models:**
```bash
./generate_code.sh
```

**Models location:** `/home/pankaj/eHMI_all/models/`

---

## 10. Audio File Preparation Guidelines

### File Format Specifications
- **Format:** MP3
- **Sample Rate:** 44.1 kHz or 48 kHz
- **Bit Rate:** 128 kbps (recommended)
- **Channels:** Mono or Stereo
- **Duration:** <5 seconds for announcements, <10 seconds for instructions

### File Naming Convention
```
{state_name}_{language}.mp3

Examples:
- active_en.mp3
- obstacle_detected_jp.mp3
- docking_warning_zh.mp3
```

### Recording Recommendations
- Professional voice talent (native speakers)
- Clear pronunciation, moderate speed
- Consistent volume levels across all files
- Background noise removal
- Gentle fade-in/fade-out (50ms)

---

## 11. LED Animation Design Patterns

### Pattern Types

**1. Pulsing (Breathing)**
- Use case: IDLE, WAITING states
- Implementation: Sine wave brightness modulation
- Period: 1-2 seconds

**2. Moving (Chase)**
- Use case: ACTIVE, APPROACHING states
- Implementation: LED position increment with tail
- Speed: 10-30 LEDs/second

**3. Scanning**
- Use case: SEARCHING_ARUCO, OBSTACLE_DETECTED
- Implementation: Back-and-forth LED movement
- Range: Full strip length

**4. Blinking**
- Use case: ERROR, WARNING states
- Implementation: On/off toggle
- Frequency: 2-5 Hz

**5. Solid**
- Use case: WHEELCHAIR_ATTACHED, success states
- Implementation: Static color, no animation
- Timeout: Auto-transition to IDLE after N seconds

---

## 12. Bill of Materials (eHMI System)

| Component | Specification | Quantity | Approx. Cost (USD) |
|-----------|--------------|----------|-------------------|
| ESP32-S3 DevKitC-1 | N32R8 variant | 1 | $10-15 |
| HUB75 LED Matrix Panel | 96×48 P3 or P4 | 2 | $40-60 |
| WS2812B LED Strip | 60 LEDs/m, 2m | 1 | $15-20 |
| I2S DAC/Amplifier | MAX98357A or similar | 1 | $5-8 |
| Speaker | 3W, 8Ω | 1 | $5-10 |
| MicroSD Card | 8GB Class 10 | 1 | $5-8 |
| Power Supply | 5V 10A (for LED matrix) | 1 | $15-20 |
| Cables & Connectors | Various | - | $10-15 |
| **Total** | | | **$105-156** |

---

## 13. Key Takeaways for Wheelchair Robot

### ✅ Reusable Components
1. **ESP32-S3 platform** - Proven hardware, sufficient resources
2. **Serial protocol** - Simple, reliable, easy to extend
3. **Multi-language framework** - SD card audio file structure
4. **State machine pattern** - Dezyne formal verification
5. **Dual LED system** - LED strips + LED matrix for redundancy

### 🔧 Required Modifications
1. **Add 8+ new states** for wheelchair docking and transport
2. **Record new audio files** for safety announcements
3. **Create new GIF animations** for docking visualization
4. **Adjust LED patterns** for wheelchair-specific states
5. **Update ROS node** to handle extended state set

### 💡 Design Recommendations
1. **Audio priority system** - Safety messages override others
2. **Redundant indicators** - LED + Audio for critical states
3. **Passenger comfort** - Quiet mode during TRANSPORTING
4. **Emergency override** - Instant red LED + alarm on e-stop
5. **Battery awareness** - Visual + audio warnings at multiple thresholds

---

## 14. File References

**Main Implementation:**
- `/home/pankaj/eHMI_all/src/main.cpp` (lines 1-236)
- `/home/pankaj/eHMI_all/platformio.ini` (lines 1-40)

**Component Implementations:**
- `/home/pankaj/eHMI_all/src/AudioHandlerComponent.cpp` (audio playback)
- `/home/pankaj/eHMI_all/src/LedAnimationHandlerComponent.cpp` (LED animations)
- `/home/pankaj/eHMI_all/src/GIFHandler.cpp` (matrix display GIFs)

**Configuration:**
- `/home/pankaj/eHMI_all/src/common/constants.h` (hardware pins)
- `/home/pankaj/eHMI_all/src/common/SystemState.h` (state definitions)

**Build System:**
- `/home/pankaj/eHMI_all/generate_code.sh` (Dezyne code generation)

---

**Document Status:** Complete
**Ready for:** Wheelchair robot eHMI specification
**Next Step:** Define complete state set and behavior table for wheelchair transport
