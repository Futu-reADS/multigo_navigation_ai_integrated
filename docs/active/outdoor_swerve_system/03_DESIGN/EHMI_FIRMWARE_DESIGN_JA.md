# eHMIファームウェア詳細設計

**ドキュメントID:** DESIGN-EHMI-001
**バージョン:** 1.0
**日付:** 2025-12-15
**ステータス:** ドラフト

---

## 1. 概要

本ドキュメントは、車椅子輸送状態のビジュアルおよびオーディオ通信を実装する、ESP32-S3上で動作するeHMI(外部ヒューマンマシンインターフェース)ファームウェアの詳細設計仕様を提供します。

**ハードウェア:**
- ESP32-S3マイクロコントローラ (デュアルコア、WiFi/BLE)
- WS2812B LEDストリップ (≥60 LED、アドレサブルRGB)
- HUB75 LEDマトリックス (64×32ピクセル、RGB)
- I2Sオーディオアンプ + スピーカー

**ソフトウェア:**
- ESP-IDF (FreeRTOS)
- Dezyne形式状態機械
- メインコンピュータとのシリアル通信 (115200 baud)

---

## 2. 状態機械

**車椅子固有状態 (21-28):**
```
21: 車椅子接近中          → 青点滅、「接近中」
22: ドッキング進行中      → 黄色回転、「ドッキング中」
23: ドッキング完了        → 緑点灯、「準備完了」
24: 乗客搭乗              → 白点灯、「輸送中」
25: 輸送進行中            → 白呼吸、「移動中」
26: 目的地到着            → 緑点滅、「到着」
27: 乗客降車              → 青点滅、「降車してください」
28: ドッキング失敗        → 赤点滅、「エラー」
```

**Dezyne状態機械 (eHMI.dzn):**
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

## 3. LED制御

### WS2812B LEDストリップコントローラ

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
                showPulsingBlue();  // 青点滅
                break;

            case eHMIState::DOCKING:
                showRotatingYellow();  // 黄色回転
                break;

            case eHMIState::DOCKING_COMPLETE:
                showSolidGreen();  // 緑点灯
                break;

            case eHMIState::PASSENGER_ONBOARD:
                showSolidWhite();  // 白点灯
                break;

            case eHMIState::TRANSPORT:
                showBreathingWhite();  // 白呼吸
                break;

            case eHMIState::ARRIVED:
                showPulsingGreen();  // 緑点滅
                break;

            case eHMIState::DOCKING_FAILED:
                showFlashingRed();  // 赤点滅
                break;
        }
    }

    void update() {
        // メインループで30 Hzで呼び出される
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

### HUB75 LEDマトリックスコントローラ

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

        // 状態に基づいて色を設定
        uint16_t color = getColorForState(state);

        // テキストを描画 (長すぎる場合はスクロール)
        matrix_->setTextColor(color);
        matrix_->setCursor(2, 12);
        matrix_->print(message);
    }

private:
    uint16_t getColorForState(eHMIState state) {
        switch (state) {
            case eHMIState::APPROACHING:
            case eHMIState::DISEMBARK:
                return matrix_->color565(0, 0, 255);  // 青

            case eHMIState::DOCKING:
                return matrix_->color565(255, 255, 0);  // 黄

            case eHMIState::DOCKING_COMPLETE:
            case eHMIState::ARRIVED:
                return matrix_->color565(0, 255, 0);  // 緑

            case eHMIState::PASSENGER_ONBOARD:
            case eHMIState::TRANSPORT:
                return matrix_->color565(255, 255, 255);  // 白

            case eHMIState::DOCKING_FAILED:
                return matrix_->color565(255, 0, 0);  // 赤

            default:
                return matrix_->color565(128, 128, 128);  // 灰
        }
    }

    MatrixPanel_I2S_DMA* matrix_;
    static const int PANEL_WIDTH = 64;
    static const int PANEL_HEIGHT = 32;
    static const int PANEL_CHAIN = 1;
};
```

---

## 4. オーディオ再生

**I2Sオーディオコントローラ:**
```cpp
// audio_controller.cpp
#include <driver/i2s.h>

class AudioController {
public:
    AudioController() {
        i2s_config_t i2s_config = {
            .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
            .sample_rate = 16000,  // 16kHzモノラル
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
        // 状態と言語に基づいてオーディオファイルをロード
        const char* audio_file = getAudioFile(state, language);

        // オーディオを再生 (ノンブロッキング)
        playWAVFile(audio_file);
    }

    void setVolume(int volume) {
        // 0-10スケール → 0-100%
        volume_percent_ = volume * 10;
    }

private:
    const char* getAudioFile(eHMIState state, const char* language) {
        // オーディオファイルはSPIFFSに保存
        // フォーマット: /audio/{language}/{state}.wav

        static char path[64];
        snprintf(path, sizeof(path), "/audio/%s/%d.wav", language, (int)state);
        return path;
    }

    void playWAVFile(const char* filename) {
        // SPIFFSからWAVファイルを読み込み、I2Sに書き込み
        File audioFile = SPIFFS.open(filename, "r");
        if (!audioFile) {
            ESP_LOGE(TAG, "オーディオファイルのオープン失敗: %s", filename);
            return;
        }

        // WAVヘッダーをスキップ (44バイト)
        audioFile.seek(44);

        // オーディオをI2Sにストリーム
        uint8_t buffer[1024];
        size_t bytes_read;
        size_t bytes_written;

        while ((bytes_read = audioFile.read(buffer, sizeof(buffer))) > 0) {
            // ボリュームスケーリングを適用
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
    int volume_percent_ = 80;  // デフォルト80%
};
```

---

## 5. シリアル通信

**プロトコル:**
```
コマンド (メインコンピュータ → ESP32):
  STATE <state_num>    # eHMI状態を設定 (0-28)
  LANG <language>      # 言語を設定 (en, ja, zh)
  VOL <volume>         # ボリュームを設定 (0-10)

応答 (ESP32 → メインコンピュータ):
  OK                   # コマンド成功
  ERROR <message>      # コマンド失敗
```

**実装:**
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
            Serial.println("ERROR 不明なコマンド");
        }
    }

private:
    eHMIController* ehmi_controller_;
};
```

---

## 6. メインファームウェアループ

```cpp
// main.cpp
#include <Arduino.h>

LEDStripController led_strip(60, LED_DATA_PIN);
LEDMatrixController led_matrix;
AudioController audio;
SerialHandler serial_handler;

eHMIState current_state = eHMIState::IDLE;
String current_language = "ja";  // デフォルト日本語

void setup() {
    Serial.begin(115200);

    led_strip.begin();
    led_matrix.begin();
    audio.begin();

    ESP_LOGI(TAG, "eHMIファームウェア開始");
}

void loop() {
    // シリアルコマンドを処理
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        serial_handler.processCommand(command);
    }

    // LEDパターンを30 Hzで更新
    static unsigned long last_update = 0;
    if (millis() - last_update > 33) {  // ~30 Hz
        led_strip.update();
        last_update = millis();
    }

    delay(10);
}
```

---

## 7. テスト

**シリアルテスト例:**
```bash
# 状態遷移テスト
echo "STATE 21" > /dev/ttyUSB0  # 接近中
sleep 2
echo "STATE 22" > /dev/ttyUSB0  # ドッキング中
sleep 2
echo "STATE 23" > /dev/ttyUSB0  # ドッキング完了

# 言語切り替えテスト
echo "LANG ja" > /dev/ttyUSB0

# ボリュームテスト
echo "VOL 5" > /dev/ttyUSB0
```

---

**ドキュメントステータス:** ドラフト
**実装ステータス:** 開発準備完了
**必要な承認:** eHMIエンジニア、組み込みシステムリード
