# eHMI Firmware Detailed Design

**Document ID:** DESIGN-EHMI-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft

---

## 1. Overview

This document provides detailed design specifications for the eHMI (External Human-Machine Interface) Firmware running on ESP32-S3, implementing visual and audio communication for wheelchair transport states.

**Hardware:**
- ESP32-S3 microcontroller (dual-core, WiFi/BLE)
- WS2812B LED strip (≥60 LEDs, addressable RGB)
- HUB75 LED matrix (64×32 pixels, RGB)
- I2S audio amplifier + speaker

**Software:**
- ESP-IDF (FreeRTOS)
- Dezyne formal state machine
- Serial communication with main computer (115200 baud)

---

## 2. State Machine

**Wheelchair-Specific States (21-28):**
```
21: Approaching Wheelchair    → Blue pulsing, "Approaching"
22: Docking in Progress       → Yellow rotating, "Docking"
23: Docking Complete          → Green solid, "Ready"
24: Passenger Onboard         → White solid, "Transporting"
25: Transport in Progress     → White breathing, "En Route"
26: Destination Reached       → Green pulsing, "Arrived"
27: Passenger Disembark       → Blue pulsing, "Please Exit"
28: Docking Failed            → Red flashing, "Error"
```

**Dezyne State Machine (eHMI.dzn):**
```dzn
interface IeHMI {
  in void setState(int state_num);
  in void setLanguage(string lang);  // "en", "ja", "zh"
  in void setVolume(int volume);     // 0-10

  out void updateLEDs();
  out void playAudio();

  behavior {
    enum State {
      Idle, Approaching, Docking, DockingComplete,
      PassengerOnboard, Transport, Arrived, Disembark, Failed
    };

    State current_state = State.Idle;

    on setState(state_num): {
      current_state = mapState(state_num);
      reply updateLEDs();
      reply playAudio();
    }
  }
}
```

---

## 3. LED Control

### WS2812B LED Strip Controller

```cpp
// led_strip_controller.cpp
#include <FastLED.h>

class LEDStripController {
public:
    LEDStripController(int num_leds, int data_pin)
        : num_leds_(num_leds), data_pin_(data_pin) {
        leds_ = new CRGB[num_leds];
        FastLED.addLeds<WS2812B, data_pin_, GRB>(leds_, num_leds_);
    }

    void setState(eHMIState state) {
        current_state_ = state;

        switch (state) {
            case eHMIState::APPROACHING:
                showPulsingBlue();
                break;

            case eHMIState::DOCKING:
                showRotatingYellow();
                break;

            case eHMIState::DOCKING_COMPLETE:
                showSolidGreen();
                break;

            case eHMIState::PASSENGER_ONBOARD:
                showSolidWhite();
                break;

            case eHMIState::TRANSPORT:
                showBreathingWhite();
                break;

            case eHMIState::ARRIVED:
                showPulsingGreen();
                break;

            case eHMIState::DOCKING_FAILED:
                showFlashingRed();
                break;
        }
    }

    void update() {
        // Called in main loop @ 30 Hz
        updateCurrentPattern();
        FastLED.show();
    }

private:
    void showPulsingBlue() {
        int brightness = (sin(millis() / 500.0) + 1.0) * 127;
        fill_solid(leds_, num_leds_, CRGB(0, 0, brightness));
    }

    void showRotatingYellow() {
        int offset = (millis() / 50) % num_leds_;
        for (int i = 0; i < num_leds_; i++) {
            int dist = abs(i - offset);
            if (dist < 10) {
                leds_[i] = CRGB(255, 255, 0);
            } else {
                leds_[i] = CRGB(0, 0, 0);
            }
        }
    }

    void showSolidGreen() {
        fill_solid(leds_, num_leds_, CRGB(0, 255, 0));
    }

    void showSolidWhite() {
        fill_solid(leds_, num_leds_, CRGB(255, 255, 255));
    }

    void showBreathingWhite() {
        int brightness = (sin(millis() / 1000.0) + 1.0) * 127;
        fill_solid(leds_, num_leds_, CRGB(brightness, brightness, brightness));
    }

    void showPulsingGreen() {
        int brightness = (sin(millis() / 500.0) + 1.0) * 127;
        fill_solid(leds_, num_leds_, CRGB(0, brightness, 0));
    }

    void showFlashingRed() {
        bool on = (millis() / 250) % 2;
        fill_solid(leds_, num_leds_, on ? CRGB(255, 0, 0) : CRGB(0, 0, 0));
    }

    int num_leds_;
    int data_pin_;
    CRGB* leds_;
    eHMIState current_state_;
};
```

---

### HUB75 LED Matrix Controller

```cpp
// led_matrix_controller.cpp
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

class LEDMatrixController {
public:
    LEDMatrixController() {
        HUB75_I2S_CFG mxconfig(
            PANEL_WIDTH,   // 64
            PANEL_HEIGHT,  // 32
            PANEL_CHAIN    // 1
        );

        matrix_ = new MatrixPanel_I2S_DMA(mxconfig);
        matrix_->begin();
        matrix_->setBrightness(128);
    }

    void displayMessage(const char* message, eHMIState state) {
        matrix_->clearScreen();

        // Set color based on state
        uint16_t color = getColorForState(state);

        // Draw text (scrolling if too long)
        matrix_->setTextColor(color);
        matrix_->setCursor(2, 12);
        matrix_->print(message);
    }

private:
    uint16_t getColorForState(eHMIState state) {
        switch (state) {
            case eHMIState::APPROACHING:
            case eHMIState::DISEMBARK:
                return matrix_->color565(0, 0, 255);  // Blue

            case eHMIState::DOCKING:
                return matrix_->color565(255, 255, 0);  // Yellow

            case eHMIState::DOCKING_COMPLETE:
            case eHMIState::ARRIVED:
                return matrix_->color565(0, 255, 0);  // Green

            case eHMIState::PASSENGER_ONBOARD:
            case eHMIState::TRANSPORT:
                return matrix_->color565(255, 255, 255);  // White

            case eHMIState::DOCKING_FAILED:
                return matrix_->color565(255, 0, 0);  // Red

            default:
                return matrix_->color565(128, 128, 128);  // Gray
        }
    }

    MatrixPanel_I2S_DMA* matrix_;
    static const int PANEL_WIDTH = 64;
    static const int PANEL_HEIGHT = 32;
    static const int PANEL_CHAIN = 1;
};
```

---

## 4. Audio Playback

**I2S Audio Controller:**
```cpp
// audio_controller.cpp
#include <driver/i2s.h>

class AudioController {
public:
    AudioController() {
        i2s_config_t i2s_config = {
            .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
            .sample_rate = 16000,  // 16kHz mono
            .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
            .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
            .communication_format = I2S_COMM_FORMAT_I2S_MSB,
            .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
            .dma_buf_count = 8,
            .dma_buf_len = 1024
        };

        i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
        i2s_set_pin(I2S_NUM_0, &pin_config_);
    }

    void playAudioForState(eHMIState state, const char* language) {
        // Load audio file based on state and language
        const char* audio_file = getAudioFile(state, language);

        // Play audio (non-blocking)
        playWAVFile(audio_file);
    }

    void setVolume(int volume) {
        // 0-10 scale → 0-100%
        volume_percent_ = volume * 10;
    }

private:
    const char* getAudioFile(eHMIState state, const char* language) {
        // Audio files stored in SPIFFS
        // Format: /audio/{language}/{state}.wav

        static char path[64];
        snprintf(path, sizeof(path), "/audio/%s/%d.wav", language, (int)state);
        return path;
    }

    void playWAVFile(const char* filename) {
        // Read WAV file from SPIFFS and write to I2S
        File audioFile = SPIFFS.open(filename, "r");
        if (!audioFile) {
            ESP_LOGE(TAG, "Failed to open audio file: %s", filename);
            return;
        }

        // Skip WAV header (44 bytes)
        audioFile.seek(44);

        // Stream audio to I2S
        uint8_t buffer[1024];
        size_t bytes_read;
        size_t bytes_written;

        while ((bytes_read = audioFile.read(buffer, sizeof(buffer))) > 0) {
            // Apply volume scaling
            applyVolume(buffer, bytes_read, volume_percent_);

            i2s_write(I2S_NUM_0, buffer, bytes_read, &bytes_written, portMAX_DELAY);
        }

        audioFile.close();
    }

    void applyVolume(uint8_t* buffer, size_t len, int volume_percent) {
        int16_t* samples = (int16_t*)buffer;
        size_t num_samples = len / 2;

        for (size_t i = 0; i < num_samples; i++) {
            samples[i] = (samples[i] * volume_percent) / 100;
        }
    }

    i2s_pin_config_t pin_config_;
    int volume_percent_ = 80;  // Default 80%
};
```

---

## 5. Serial Communication

**Protocol:**
```
Commands (from main computer → ESP32):
  STATE <state_num>    # Set eHMI state (0-28)
  LANG <language>      # Set language (en, ja, zh)
  VOL <volume>         # Set volume (0-10)

Responses (ESP32 → main computer):
  OK                   # Command successful
  ERROR <message>      # Command failed
```

**Implementation:**
```cpp
// serial_handler.cpp
class SerialHandler {
public:
    void processCommand(const String& command) {
        if (command.startsWith("STATE ")) {
            int state_num = command.substring(6).toInt();
            ehmi_controller_->setState(state_num);
            Serial.println("OK");
        }
        else if (command.startsWith("LANG ")) {
            String lang = command.substring(5);
            ehmi_controller_->setLanguage(lang.c_str());
            Serial.println("OK");
        }
        else if (command.startsWith("VOL ")) {
            int volume = command.substring(4).toInt();
            ehmi_controller_->setVolume(volume);
            Serial.println("OK");
        }
        else {
            Serial.println("ERROR Unknown command");
        }
    }

private:
    eHMIController* ehmi_controller_;
};
```

---

## 6. Main Firmware Loop

```cpp
// main.cpp
#include <Arduino.h>

LEDStripController led_strip(60, LED_DATA_PIN);
LEDMatrixController led_matrix;
AudioController audio;
SerialHandler serial_handler;

eHMIState current_state = eHMIState::IDLE;
String current_language = "en";

void setup() {
    Serial.begin(115200);

    led_strip.begin();
    led_matrix.begin();
    audio.begin();

    ESP_LOGI(TAG, "eHMI Firmware started");
}

void loop() {
    // Process serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        serial_handler.processCommand(command);
    }

    // Update LED patterns @ 30 Hz
    static unsigned long last_update = 0;
    if (millis() - last_update > 33) {  // ~30 Hz
        led_strip.update();
        last_update = millis();
    }

    delay(10);
}
```

---

## 7. Testing

**Serial Test Example:**
```bash
# Test state transitions
echo "STATE 21" > /dev/ttyUSB0  # Approaching
sleep 2
echo "STATE 22" > /dev/ttyUSB0  # Docking
sleep 2
echo "STATE 23" > /dev/ttyUSB0  # Docking Complete

# Test language switching
echo "LANG ja" > /dev/ttyUSB0

# Test volume
echo "VOL 5" > /dev/ttyUSB0
```

---

**Document Status:** Draft
**Implementation Status:** Ready for development
**Approvals Required:** eHMI Engineer, Embedded Systems Lead
