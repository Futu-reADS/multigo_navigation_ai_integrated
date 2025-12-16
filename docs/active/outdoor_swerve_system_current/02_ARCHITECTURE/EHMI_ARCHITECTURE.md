# eHMI Architecture (External Human-Machine Interface)

**Document ID:** ARCH-EHMI-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft

---

## 1. Overview

The **eHMI (External Human-Machine Interface) Subsystem** communicates robot intent and status to pedestrians and bystanders through visual and audio signals. It enhances safety and social acceptance of the autonomous robot.

**Key Capabilities:**
- LED strip for ambient robot state indication (360° visibility)
- LED matrix for directional intent display (arrows, icons, text)
- Audio announcements for key events (docking, arriving, warnings)
- Multi-language audio support (EN, JP, ZH)
- Formal state machine (Dezyne modeling) for deterministic behavior
- Serial communication with ROS 2 (ESP32-S3 microcontroller)

**Design Philosophy:**
- **Unambiguous communication:** Clear visual/audio signals
- **Predictable behavior:** Formal state machine (Dezyne)
- **Outdoor operation:** High brightness LEDs, weatherproof speakers
- **Social acceptance:** Friendly, non-threatening appearance

---

## 2. System Architecture

### 2.1 Component Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                    eHMI Subsystem                                │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                  ROS 2 eHMI Manager Node                   │ │
│  │                  (Python or C++)                           │ │
│  │                                                            │ │
│  │  - Subscribe to robot state topics                        │ │
│  │  - Map state to eHMI state (0-28)                         │ │
│  │  - Send commands to ESP32 via serial                      │ │
│  └──────────────────────┬─────────────────────────────────────┘ │
│                         │                                        │
│                         │ Serial (115200 baud)                   │
│                         ↓                                        │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                  ESP32-S3 Microcontroller                  │ │
│  │                  (Arduino/PlatformIO)                      │ │
│  │                                                            │ │
│  │  ┌──────────────────┐      ┌──────────────────┐          │ │
│  │  │  Dezyne State    │      │  LED Controller  │          │ │
│  │  │  Machine         │─────▶│                  │          │ │
│  │  │  (eHMI States    │      │  - WS2812B strip │          │ │
│  │  │   0-28)          │      │  - HUB75 matrix  │          │ │
│  │  └──────────────────┘      └──────────────────┘          │ │
│  │                                                            │ │
│  │  ┌──────────────────┐      ┌──────────────────┐          │ │
│  │  │  Audio Player    │      │  Serial          │          │ │
│  │  │                  │      │  Interface       │          │ │
│  │  │  - I2S DAC       │      │  (ROS commands)  │          │ │
│  │  │  - MP3 files     │      │                  │          │ │
│  │  └──────────────────┘      └──────────────────┘          │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                  Hardware                                  │ │
│  │                                                            │ │
│  │  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐│ │
│  │  │ WS2812B LED   │  │ HUB75 LED     │  │ I2S Speaker   ││ │
│  │  │ Strip         │  │ Matrix        │  │ (MAX98357A)   ││ │
│  │  │ (≥60 LEDs)    │  │ (64×32 pixels)│  │               ││ │
│  │  │               │  │               │  │               ││ │
│  │  │ - 360° view   │  │ - Directional │  │ - Announcements││ │
│  │  │ - Ambient     │  │   arrows      │  │ - Multi-lang  ││ │
│  │  │   status      │  │ - Icons, text │  │               ││ │
│  │  └───────────────┘  └───────────────┘  └───────────────┘│ │
│  └────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────┘
```

---

## 3. Hardware Architecture

### 3.1 ESP32-S3 Microcontroller

**Specifications:**
- **CPU:** Dual-core Xtensa LX7, 240 MHz
- **Memory:** 512 KB SRAM, 384 KB ROM
- **Flash:** 8 MB (for firmware + audio files)
- **GPIO:** 45 programmable pins
- **Peripherals:** I2S, SPI, UART, I2C
- **Power:** 3.3V, ~500mA (LEDs require separate 5V supply)

**Pinout:**
```
ESP32-S3 Pinout:
- GPIO2  → WS2812B LED strip data
- GPIO4-11 → HUB75 LED matrix (R1, G1, B1, R2, G2, B2, A, B, C, D, CLK, LAT, OE)
- GPIO12 → I2S BCLK (bit clock)
- GPIO13 → I2S LRCLK (left/right clock)
- GPIO14 → I2S DIN (data in)
- GPIO43 → UART TX (to ROS 2)
- GPIO44 → UART RX (from ROS 2)
```

---

### 3.2 LED Strip (WS2812B)

**Specifications:**
- **Type:** WS2812B individually addressable RGB LEDs
- **Count:** ≥60 LEDs (300mm strip, 5mm spacing)
- **Voltage:** 5V DC
- **Current:** ~60mA per LED (max brightness), ~3.6A total for 60 LEDs
- **Protocol:** Single-wire control (800 kHz data rate)

**Mounting:**
- 360° ring around robot base or top
- Weatherproof silicone coating (IP65+)
- Diffuser for softer light

**States:**
- **Idle:** Slow blue pulse
- **Navigating:** Green chase pattern
- **Docking:** Yellow rotating pattern
- **Passenger onboard:** Cyan solid
- **Error:** Red flashing
- **Emergency stop:** Red solid

---

### 3.3 LED Matrix (HUB75)

**Specifications:**
- **Resolution:** 64×32 pixels
- **Color:** RGB (full color)
- **Protocol:** HUB75 (parallel data + control)
- **Refresh rate:** 60 Hz minimum
- **Viewing angle:** 120° horizontal, 60° vertical
- **Brightness:** Adjustable (0-255)

**Content:**
- **Arrows:** Left, right, forward, backward, rotate
- **Icons:** Wheelchair, warning, checkmark, X
- **Text:** Short messages (4-8 characters, e.g., "STOP", "WAIT")

**Example Displays:**
- Approaching wheelchair: "→" (right arrow)
- Docking in progress: Wheelchair icon + rotating animation
- Destination reached: Checkmark icon
- Error: "X" + error code

---

### 3.4 Audio (I2S Speaker)

**Specifications:**
- **DAC:** MAX98357A I2S amplifier (3W mono)
- **Speaker:** 4Ω 3W waterproof speaker
- **Format:** MP3 audio files (stored on ESP32 flash)
- **Volume:** 0-10 (adjustable via ROS)
- **Sample rate:** 44.1 kHz

**Audio Files (Multi-Language):**
```
audio/
├── en/
│   ├── approaching.mp3           # "Approaching wheelchair"
│   ├── docking.mp3                # "Docking in progress"
│   ├── docked.mp3                 # "Docking complete"
│   ├── passenger_onboard.mp3      # "Passenger onboard"
│   ├── destination_reached.mp3    # "Destination reached"
│   ├── emergency_stop.mp3         # "Emergency stop activated"
│   └── error.mp3                  # "Error detected"
├── ja/
│   ├── approaching.mp3            # "車椅子に接近中"
│   ├── docking.mp3                # "ドッキング中"
│   ├── docked.mp3                 # "ドッキング完了"
│   └── ...
└── zh/
    ├── approaching.mp3            # "接近轮椅"
    └── ...
```

---

## 4. State Machine (Dezyne Formal Modeling)

### 4.1 State Definitions

**Standard Robot States (0-20):**
- 0: Off
- 1: Idle
- 2: Navigating
- 3: Paused
- 4: Charging
- 5: Error
- 6: Emergency Stop
- 7: Manual Control
- 8: Localization Lost
- 9: Low Battery
- 10: Obstacle Detected
- 11-20: Reserved

**Wheelchair-Specific States (21-28):**
- 21: Approaching Wheelchair
- 22: Docking in Progress
- 23: Docking Complete
- 24: Passenger Onboard
- 25: Transport in Progress
- 26: Destination Reached
- 27: Passenger Disembark
- 28: Docking Failed

---

### 4.2 Dezyne State Machine Model

**eHMI.dzn:**
```dzn
interface IeHMI {
  in void setState(int state);
  in void setLanguage(string lang);
  in void setVolume(int volume);
  
  behavior {
    enum State {
      OFF, IDLE, NAVIGATING, PAUSED, CHARGING, ERROR, EMERGENCY_STOP,
      MANUAL, LOC_LOST, LOW_BATTERY, OBSTACLE,
      APPROACHING_WHEELCHAIR, DOCKING, DOCKED, PASSENGER_ONBOARD,
      TRANSPORT, DESTINATION_REACHED, DISEMBARK, DOCKING_FAILED
    };
    
    State current_state = State.OFF;
    
    on setState(int s): {
      current_state = toState(s);
      updateLEDs(current_state);
      updateMatrix(current_state);
      playAudio(current_state);
    }
  }
}

component eHMI {
  provides IeHMI api;
  requires ILEDStrip led_strip;
  requires ILEDMatrix led_matrix;
  requires IAudio audio;
  
  behavior {
    on api.setState(state): {
      // State transition logic
      if (isValidTransition(current_state, state)) {
        current_state = state;
        
        // Update visual outputs
        led_strip.setPattern(getPattern(state));
        led_matrix.setContent(getContent(state));
        
        // Play audio announcement
        if (hasAudio(state)) {
          audio.play(getAudioFile(state, current_language));
        }
      } else {
        // Invalid transition, log error
        logError("Invalid state transition");
      }
    }
  }
}
```

---

### 4.3 State Transitions

**Allowed Transitions:**
```
IDLE → NAVIGATING                    (Start navigation)
NAVIGATING → APPROACHING_WHEELCHAIR  (Near docking station)
APPROACHING_WHEELCHAIR → DOCKING     (Start visual servoing)
DOCKING → DOCKED                     (Docking complete)
DOCKED → PASSENGER_ONBOARD           (Passenger attachment verified)
PASSENGER_ONBOARD → TRANSPORT        (Start transport mission)
TRANSPORT → DESTINATION_REACHED      (Arrive at destination)
DESTINATION_REACHED → DISEMBARK      (Passenger disembarking)
DISEMBARK → IDLE                     (Mission complete)

* → EMERGENCY_STOP                   (Any state → E-Stop)
* → ERROR                            (Any state → Error)
DOCKING → DOCKING_FAILED             (Docking timeout or failure)
```

**Invalid Transitions:**
- IDLE → PASSENGER_ONBOARD (cannot skip docking)
- DOCKING → TRANSPORT (cannot skip verification)

---

## 5. ROS 2 Integration

### 5.1 eHMI Manager Node

**Node Name:** `ehmi_manager`
**Language:** Python or C++
**Lifecycle:** Standard node

**Subscribed Topics:**
- `/robot_state` (std_msgs/UInt8): Overall robot state (0-20)
- `/docking/state` (std_msgs/String): Docking subsystem state
- `/route_status` (custom_msgs/RouteStatus): Mission progress
- `/passenger_attached` (std_msgs/Bool): Passenger onboard status
- `/emergency_stop` (std_msgs/Bool): E-Stop status

**Published Topics:**
- `/ehmi_state` (std_msgs/UInt8): Current eHMI state (0-28)

**Services:**
- `/ehmi/set_language` (custom_srvs/SetLanguage): Change audio language
- `/ehmi/set_volume` (custom_srvs/SetVolume): Adjust audio volume (0-10)
- `/ehmi/set_brightness` (custom_srvs/SetBrightness): Adjust LED brightness (0-255)

**Serial Interface:**
- **Port:** /dev/ttyUSB0 (or /dev/ttyACM0 for ESP32)
- **Baud rate:** 115200
- **Protocol:** Simple text-based commands

**Serial Commands:**
```
STATE <state_num>           # Set eHMI state (0-28)
LANG <language>             # Set language (en, ja, zh)
VOL <volume>                # Set volume (0-10)
BRIGHT <brightness>         # Set brightness (0-255)
TEST                        # Run test pattern (all LEDs + audio)
```

---

### 5.2 State Mapping Algorithm

**Python Implementation:**
```python
def map_robot_state_to_ehmi(robot_state, docking_state, passenger_attached, mission_active):
    # Emergency states (highest priority)
    if robot_state == RobotState.EMERGENCY_STOP:
        return eHMIState.EMERGENCY_STOP
    if robot_state == RobotState.ERROR:
        return eHMIState.ERROR
    
    # Docking states
    if docking_state == "APPROACH":
        return eHMIState.APPROACHING_WHEELCHAIR
    elif docking_state == "SERVOING":
        return eHMIState.DOCKING
    elif docking_state == "DOCKED":
        if passenger_attached:
            return eHMIState.PASSENGER_ONBOARD
        else:
            return eHMIState.DOCKED
    elif docking_state == "FAILED":
        return eHMIState.DOCKING_FAILED
    
    # Mission states
    if passenger_attached and mission_active:
        return eHMIState.TRANSPORT
    elif robot_state == RobotState.NAVIGATING:
        return eHMIState.NAVIGATING
    
    # Default states
    if robot_state == RobotState.IDLE:
        return eHMIState.IDLE
    elif robot_state == RobotState.CHARGING:
        return eHMIState.CHARGING
    
    # Fallback
    return eHMIState.IDLE
```

---

## 6. ESP32 Firmware

### 6.1 Main Loop

**main.cpp (Arduino):**
```cpp
#include <FastLED.h>
#include <Adafruit_GFX.h>
#include <RGBmatrixPanel.h>
#include <AudioFileSourceSPIFFS.h>
#include <AudioGeneratorMP3.h>
#include <AudioOutputI2S.h>

// LED Strip
#define LED_PIN 2
#define NUM_LEDS 60
CRGB leds[NUM_LEDS];

// LED Matrix
RGBmatrixPanel matrix(A, B, C, D, CLK, LAT, OE, false, 64);

// Audio
AudioGeneratorMP3 *mp3;
AudioFileSourceSPIFFS *file;
AudioOutputI2S *out;

// Current state
uint8_t current_state = 1;  // IDLE
String current_language = "en";
uint8_t current_volume = 5;

void setup() {
  Serial.begin(115200);
  
  // Initialize LED strip
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(128);
  
  // Initialize LED matrix
  matrix.begin();
  
  // Initialize audio
  out = new AudioOutputI2S();
  mp3 = new AudioGeneratorMP3();
  
  // Load initial state
  updateState(current_state);
}

void loop() {
  // Check for serial commands from ROS
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
  
  // Update LED animations
  updateLEDAnimation();
  
  // Update audio playback
  if (mp3->isRunning()) {
    if (!mp3->loop()) mp3->stop();
  }
  
  FastLED.show();
  delay(20);  // 50 Hz update rate
}

void processCommand(String cmd) {
  if (cmd.startsWith("STATE ")) {
    uint8_t state = cmd.substring(6).toInt();
    updateState(state);
  } else if (cmd.startsWith("LANG ")) {
    current_language = cmd.substring(5);
  } else if (cmd.startsWith("VOL ")) {
    current_volume = cmd.substring(4).toInt();
    out->SetGain(current_volume / 10.0);
  } else if (cmd.startsWith("BRIGHT ")) {
    uint8_t brightness = cmd.substring(7).toInt();
    FastLED.setBrightness(brightness);
  }
}

void updateState(uint8_t state) {
  current_state = state;
  
  // Update LED strip pattern
  setLEDPattern(state);
  
  // Update LED matrix content
  setMatrixContent(state);
  
  // Play audio announcement
  playAudio(state);
}

void setLEDPattern(uint8_t state) {
  switch (state) {
    case 1:  // IDLE
      // Slow blue pulse
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB::Blue;
      }
      break;
    case 2:  // NAVIGATING
      // Green chase pattern
      // ... (animation logic)
      break;
    case 21:  // APPROACHING_WHEELCHAIR
      // Yellow rotating pattern
      // ... (animation logic)
      break;
    case 24:  // PASSENGER_ONBOARD
      // Cyan solid
      fill_solid(leds, NUM_LEDS, CRGB::Cyan);
      break;
    case 6:  // EMERGENCY_STOP
      // Red solid
      fill_solid(leds, NUM_LEDS, CRGB::Red);
      break;
    // ... (other states)
  }
}

void setMatrixContent(uint8_t state) {
  matrix.fillScreen(0);  // Clear
  
  switch (state) {
    case 21:  // APPROACHING_WHEELCHAIR
      // Display right arrow
      matrix.drawLine(32, 16, 48, 16, matrix.Color333(7, 7, 0));  // Yellow arrow
      matrix.drawLine(48, 16, 44, 12, matrix.Color333(7, 7, 0));
      matrix.drawLine(48, 16, 44, 20, matrix.Color333(7, 7, 0));
      break;
    case 23:  // DOCKED
      // Display checkmark
      matrix.drawLine(24, 16, 28, 20, matrix.Color333(0, 7, 0));  // Green checkmark
      matrix.drawLine(28, 20, 40, 8, matrix.Color333(0, 7, 0));
      break;
    case 28:  // DOCKING_FAILED
      // Display X
      matrix.drawLine(24, 8, 40, 24, matrix.Color333(7, 0, 0));  // Red X
      matrix.drawLine(40, 8, 24, 24, matrix.Color333(7, 0, 0));
      break;
    // ... (other states)
  }
}

void playAudio(uint8_t state) {
  String filename = "/audio/" + current_language + "/";
  
  switch (state) {
    case 21:  // APPROACHING_WHEELCHAIR
      filename += "approaching.mp3";
      break;
    case 22:  // DOCKING
      filename += "docking.mp3";
      break;
    case 23:  // DOCKED
      filename += "docked.mp3";
      break;
    case 24:  // PASSENGER_ONBOARD
      filename += "passenger_onboard.mp3";
      break;
    case 26:  // DESTINATION_REACHED
      filename += "destination_reached.mp3";
      break;
    case 6:  // EMERGENCY_STOP
      filename += "emergency_stop.mp3";
      break;
    default:
      return;  // No audio for this state
  }
  
  // Play audio file
  file = new AudioFileSourceSPIFFS(filename.c_str());
  mp3->begin(file, out);
}
```

---

## 7. Testing Strategy

### 7.1 Unit Tests
- State machine transitions (Dezyne verification)
- LED pattern correctness
- Audio file playback
- Serial command parsing

### 7.2 Integration Tests
- ROS 2 → ESP32 communication
- State synchronization
- Multi-language audio switching
- Brightness/volume adjustment

### 7.3 Field Tests (Social Acceptance)
- Pedestrian reaction to eHMI signals
- Visibility in daylight (LED brightness)
- Audio audibility (outdoor noise)
- Cultural appropriateness (multi-language)

---

## 8. Performance Targets

| Metric | Target | Validation |
|--------|--------|------------|
| LED Update Rate | 50 Hz | Timestamp analysis |
| State Change Latency | <200ms | Integration test |
| LED Brightness (Daylight) | Visible at 10m | Field test |
| Audio Volume (Outdoor) | Audible at 5m | Field test |
| Serial Communication Rate | 115200 baud | Hardware spec |

---

## 9. Implementation Phases

### Phase 1: Basic eHMI (Sprints 1-2)
- ✅ ESP32 firmware setup
- ✅ LED strip control (WS2812B)
- ✅ Serial communication with ROS
- ✅ Basic states (0-10)

### Phase 2: Advanced eHMI (Sprints 3-4)
- ✅ LED matrix integration (HUB75)
- ✅ Audio playback (I2S)
- ✅ Wheelchair states (21-28)
- ✅ Multi-language support

### Phase 3: State Machine (Sprints 5-6)
- ✅ Dezyne formal model
- ✅ State transition validation
- ✅ ROS 2 eHMI manager node
- ✅ State mapping algorithm

### Phase 4: Field Validation (Sprints 7-8)
- ✅ Outdoor visibility testing
- ✅ Social acceptance testing
- ✅ Multi-language audio recording
- ✅ Production hardware assembly

---

**Document Status:** Draft
**Implementation Status:** Not Started
**Approvals Required:** eHMI Engineer, UX Designer, System Architect
