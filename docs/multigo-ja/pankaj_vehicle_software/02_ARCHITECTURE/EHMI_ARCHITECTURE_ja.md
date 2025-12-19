# eHMIアーキテクチャ（外部ヒューマンマシンインターフェース）

**文書ID:** ARCH-EHMI-001
**バージョン:** 1.0
**日付:** 2025年12月15日
**ステータス:** ドラフト

---

## 1. 概要

**eHMI（外部ヒューマンマシンインターフェース）サブシステム**は、視覚信号と音声信号を通じてロボットの意図とステータスを歩行者と傍観者に伝えます。自律ロボットの安全性と社会的受容性を向上させます。

**主な機能:**
- ロボット状態表示用LEDストリップ（360°視認性）
- 方向意図表示用LEDマトリックス（矢印、アイコン、テキスト）
- 主要イベント用音声アナウンス（ドッキング、到着、警告）
- 多言語音声サポート（EN、JP、ZH）
- 決定論的動作のための形式的状態機械（Dezyneモデリング）
- ROS 2とのシリアル通信（ESP32-S3マイクロコントローラー）

**設計哲学:**
- **明確な通信:** 明快な視覚/音声信号
- **予測可能な動作:** 形式的状態機械（Dezyne）
- **屋外動作:** 高輝度LED、防水スピーカー
- **社会的受容性:** フレンドリーで威圧的でない外観

---

## 2. システムアーキテクチャ

### 2.1 コンポーネント図

```
┌──────────────────────────────────────────────────────────────────┐
│                    eHMIサブシステム                               │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                  ROS 2 eHMIマネージャーノード                │ │
│  │                  (PythonまたはC++)                          │ │
│  │                                                            │ │
│  │  - ロボット状態トピックを購読                               │ │
│  │  - 状態をeHMI状態（0-28）にマップ                         │ │
│  │  - シリアル経由でESP32にコマンド送信                       │ │
│  └──────────────────────┬─────────────────────────────────────┘ │
│                         │                                        │
│                         │ シリアル（115200ボー）                  │
│                         ↓                                        │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                  ESP32-S3マイクロコントローラー               │ │
│  │                  (Arduino/PlatformIO)                      │ │
│  │                                                            │ │
│  │  ┌──────────────────┐      ┌──────────────────┐          │ │
│  │  │  Dezyne状態      │      │  LEDコントローラー│          │ │
│  │  │  マシン          │─────▶│                  │          │ │
│  │  │  (eHMI状態       │      │  - WS2812Bストリップ│          │ │
│  │  │   0-28)          │      │  - HUB75マトリックス│          │ │
│  │  └──────────────────┘      └──────────────────┘          │ │
│  │                                                            │ │
│  │  ┌──────────────────┐      ┌──────────────────┐          │ │
│  │  │  オーディオプレーヤー│      │  シリアル        │          │ │
│  │  │                  │      │  インターフェース │          │ │
│  │  │  - I2S DAC       │      │  (ROSコマンド)   │          │ │
│  │  │  - MP3ファイル   │      │                  │          │ │
│  │  └──────────────────┘      └──────────────────┘          │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                  ハードウェア                                │ │
│  │                                                            │ │
│  │  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐│ │
│  │  │ WS2812B LED   │  │ HUB75 LED     │  │ I2Sスピーカー  ││ │
│  │  │ ストリップ     │  │ マトリックス   │  │ (MAX98357A)   ││ │
│  │  │ (≥60 LED)     │  │ (64×32ピクセル)│  │               ││ │
│  │  │               │  │               │  │               ││ │
│  │  │ - 360°ビュー  │  │ - 方向        │  │ - アナウンス   ││ │
│  │  │ - アンビエント │  │   矢印        │  │ - 多言語      ││ │
│  │  │   ステータス   │  │ - アイコン、テキスト│  │               ││ │
│  │  └───────────────┘  └───────────────┘  └───────────────┘│ │
│  └────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────┘
```

---

## 3. ハードウェアアーキテクチャ

### 3.1 ESP32-S3マイクロコントローラー

**仕様:**
- **CPU:** デュアルコアXtensa LX7、240 MHz
- **メモリ:** 512 KB SRAM、384 KB ROM
- **フラッシュ:** 8 MB（ファームウェア + 音声ファイル用）
- **GPIO:** 45プログラマブルピン
- **ペリフェラル:** I2S、SPI、UART、I2C
- **電源:** 3.3V、~500mA（LEDは別途5V電源が必要）

**ピン配置:**
```
ESP32-S3ピン配置:
- GPIO2  → WS2812B LEDストリップデータ
- GPIO4-11 → HUB75 LEDマトリックス（R1、G1、B1、R2、G2、B2、A、B、C、D、CLK、LAT、OE）
- GPIO12 → I2S BCLK（ビットクロック）
- GPIO13 → I2S LRCLK（左/右クロック）
- GPIO14 → I2S DIN（データイン）
- GPIO43 → UART TX（ROS 2へ）
- GPIO44 → UART RX（ROS 2から）
```

---

### 3.2 LEDストリップ（WS2812B）

**仕様:**
- **タイプ:** WS2812B個別アドレス可能RGB LED
- **数:** ≥60 LED（300mmストリップ、5mm間隔）
- **電圧:** 5V DC
- **電流:** LEDあたり~60mA（最大輝度）、60 LEDで合計~3.6A
- **プロトコル:** シングルワイヤー制御（800 kHzデータレート）

**取り付け:**
- ロボットベースまたは上部に360°リング
- 防水シリコンコーティング（IP65+）
- より柔らかい光のためのディフューザー

**状態:**
- **アイドル:** 遅い青パルス
- **ナビゲート中:** 緑チェイスパターン
- **ドッキング:** 黄色回転パターン
- **乗客乗車:** シアン固定
- **エラー:** 赤点滅
- **緊急停止:** 赤固定

---

### 3.3 LEDマトリックス（HUB75）

**仕様:**
- **解像度:** 64×32ピクセル
- **色:** RGB（フルカラー）
- **プロトコル:** HUB75（パラレルデータ + 制御）
- **リフレッシュレート:** 60 Hz最小
- **視野角:** 水平120°、垂直60°
- **輝度:** 調整可能（0-255）

**コンテンツ:**
- **矢印:** 左、右、前、後、回転
- **アイコン:** 車椅子、警告、チェックマーク、X
- **テキスト:** 短いメッセージ（4-8文字、例:「STOP」、「WAIT」）

**表示例:**
- 車椅子に接近中:「→」（右矢印）
- ドッキング進行中: 車椅子アイコン + 回転アニメーション
- 目的地到着: チェックマークアイコン
- エラー:「X」 + エラーコード

---

### 3.4 オーディオ（I2Sスピーカー）

**仕様:**
- **DAC:** MAX98357A I2Sアンプ（3Wモノ）
- **スピーカー:** 4Ω 3W防水スピーカー
- **フォーマット:** MP3音声ファイル（ESP32フラッシュに保存）
- **音量:** 0-10（ROS経由で調整可能）
- **サンプルレート:** 44.1 kHz

**音声ファイル（多言語）:**
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

## 4. 状態機械（Dezyne形式的モデリング）

### 4.1 状態定義

**標準ロボット状態（0-20）:**
- 0: オフ
- 1: アイドル
- 2: ナビゲート中
- 3: 一時停止
- 4: 充電中
- 5: エラー
- 6: 緊急停止
- 7: 手動制御
- 8: ローカライゼーション喪失
- 9: 低バッテリー
- 10: 障害物検出
- 11-20: 予約済み

**車椅子専用状態（21-28）:**
- 21: 車椅子に接近中
- 22: ドッキング進行中
- 23: ドッキング完了
- 24: 乗客乗車
- 25: 輸送進行中
- 26: 目的地到着
- 27: 乗客下車
- 28: ドッキング失敗

---

### 4.2 Dezyne状態機械モデル

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
      // 状態遷移ロジック
      if (isValidTransition(current_state, state)) {
        current_state = state;

        // 視覚出力を更新
        led_strip.setPattern(getPattern(state));
        led_matrix.setContent(getContent(state));

        // 音声アナウンスを再生
        if (hasAudio(state)) {
          audio.play(getAudioFile(state, current_language));
        }
      } else {
        // 無効な遷移、エラーをログ
        logError("Invalid state transition");
      }
    }
  }
}
```

---

### 4.3 状態遷移

**許可される遷移:**
```
IDLE → NAVIGATING                    (ナビゲーション開始)
NAVIGATING → APPROACHING_WHEELCHAIR  (ドッキングステーション近く)
APPROACHING_WHEELCHAIR → DOCKING     (ビジュアルサーボイング開始)
DOCKING → DOCKED                     (ドッキング完了)
DOCKED → PASSENGER_ONBOARD           (乗客取り付け確認)
PASSENGER_ONBOARD → TRANSPORT        (輸送ミッション開始)
TRANSPORT → DESTINATION_REACHED      (目的地到着)
DESTINATION_REACHED → DISEMBARK      (乗客下車)
DISEMBARK → IDLE                     (ミッション完了)

* → EMERGENCY_STOP                   (任意の状態 → 緊急停止)
* → ERROR                            (任意の状態 → エラー)
DOCKING → DOCKING_FAILED             (ドッキングタイムアウトまたは失敗)
```

**無効な遷移:**
- IDLE → PASSENGER_ONBOARD（ドッキングをスキップできない）
- DOCKING → TRANSPORT（検証をスキップできない）

---

## 5. ROS 2統合

### 5.1 eHMIマネージャーノード

**ノード名:** `ehmi_manager`
**言語:** PythonまたはC++
**ライフサイクル:** 標準ノード

**購読トピック:**
- `/robot_state` (std_msgs/UInt8): 全体ロボット状態（0-20）
- `/docking/state` (std_msgs/String): ドッキングサブシステム状態
- `/route_status` (custom_msgs/RouteStatus): ミッション進捗
- `/passenger_attached` (std_msgs/Bool): 乗客乗車ステータス
- `/emergency_stop` (std_msgs/Bool): 緊急停止ステータス

**パブリッシュトピック:**
- `/ehmi_state` (std_msgs/UInt8): 現在のeHMI状態（0-28）

**サービス:**
- `/ehmi/set_language` (custom_srvs/SetLanguage): 音声言語変更
- `/ehmi/set_volume` (custom_srvs/SetVolume): 音声音量調整（0-10）
- `/ehmi/set_brightness` (custom_srvs/SetBrightness): LED輝度調整（0-255）

**シリアルインターフェース:**
- **ポート:** /dev/ttyUSB0（またはESP32用/dev/ttyACM0）
- **ボーレート:** 115200
- **プロトコル:** シンプルなテキストベースコマンド

**シリアルコマンド:**
```
STATE <state_num>           # eHMI状態設定（0-28）
LANG <language>             # 言語設定（en、ja、zh）
VOL <volume>                # 音量設定（0-10）
BRIGHT <brightness>         # 輝度設定（0-255）
TEST                        # テストパターン実行（全LED + 音声）
```

---

### 5.2 状態マッピングアルゴリズム

**Python実装:**
```python
def map_robot_state_to_ehmi(robot_state, docking_state, passenger_attached, mission_active):
    # 緊急状態（最高優先度）
    if robot_state == RobotState.EMERGENCY_STOP:
        return eHMIState.EMERGENCY_STOP
    if robot_state == RobotState.ERROR:
        return eHMIState.ERROR

    # ドッキング状態
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

    # ミッション状態
    if passenger_attached and mission_active:
        return eHMIState.TRANSPORT
    elif robot_state == RobotState.NAVIGATING:
        return eHMIState.NAVIGATING

    # デフォルト状態
    if robot_state == RobotState.IDLE:
        return eHMIState.IDLE
    elif robot_state == RobotState.CHARGING:
        return eHMIState.CHARGING

    # フォールバック
    return eHMIState.IDLE
```

---

## 6. ESP32ファームウェア

### 6.1 メインループ

**main.cpp（Arduino）:**
```cpp
#include <FastLED.h>
#include <Adafruit_GFX.h>
#include <RGBmatrixPanel.h>
#include <AudioFileSourceSPIFFS.h>
#include <AudioGeneratorMP3.h>
#include <AudioOutputI2S.h>

// LEDストリップ
#define LED_PIN 2
#define NUM_LEDS 60
CRGB leds[NUM_LEDS];

// LEDマトリックス
RGBmatrixPanel matrix(A, B, C, D, CLK, LAT, OE, false, 64);

// オーディオ
AudioGeneratorMP3 *mp3;
AudioFileSourceSPIFFS *file;
AudioOutputI2S *out;

// 現在の状態
uint8_t current_state = 1;  // IDLE
String current_language = "en";
uint8_t current_volume = 5;

void setup() {
  Serial.begin(115200);

  // LEDストリップ初期化
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(128);

  // LEDマトリックス初期化
  matrix.begin();

  // オーディオ初期化
  out = new AudioOutputI2S();
  mp3 = new AudioGeneratorMP3();

  // 初期状態読み込み
  updateState(current_state);
}

void loop() {
  // ROSからのシリアルコマンドをチェック
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }

  // LEDアニメーション更新
  updateLEDAnimation();

  // 音声再生更新
  if (mp3->isRunning()) {
    if (!mp3->loop()) mp3->stop();
  }

  FastLED.show();
  delay(20);  // 50 Hz更新レート
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

  // LEDストリップパターン更新
  setLEDPattern(state);

  // LEDマトリックスコンテンツ更新
  setMatrixContent(state);

  // 音声アナウンス再生
  playAudio(state);
}

void setLEDPattern(uint8_t state) {
  switch (state) {
    case 1:  // IDLE
      // 遅い青パルス
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB::Blue;
      }
      break;
    case 2:  // NAVIGATING
      // 緑チェイスパターン
      // ... (アニメーションロジック)
      break;
    case 21:  // APPROACHING_WHEELCHAIR
      // 黄色回転パターン
      // ... (アニメーションロジック)
      break;
    case 24:  // PASSENGER_ONBOARD
      // シアン固定
      fill_solid(leds, NUM_LEDS, CRGB::Cyan);
      break;
    case 6:  // EMERGENCY_STOP
      // 赤固定
      fill_solid(leds, NUM_LEDS, CRGB::Red);
      break;
    // ... (その他の状態)
  }
}

void setMatrixContent(uint8_t state) {
  matrix.fillScreen(0);  // クリア

  switch (state) {
    case 21:  // APPROACHING_WHEELCHAIR
      // 右矢印表示
      matrix.drawLine(32, 16, 48, 16, matrix.Color333(7, 7, 0));  // 黄色矢印
      matrix.drawLine(48, 16, 44, 12, matrix.Color333(7, 7, 0));
      matrix.drawLine(48, 16, 44, 20, matrix.Color333(7, 7, 0));
      break;
    case 23:  // DOCKED
      // チェックマーク表示
      matrix.drawLine(24, 16, 28, 20, matrix.Color333(0, 7, 0));  // 緑チェックマーク
      matrix.drawLine(28, 20, 40, 8, matrix.Color333(0, 7, 0));
      break;
    case 28:  // DOCKING_FAILED
      // X表示
      matrix.drawLine(24, 8, 40, 24, matrix.Color333(7, 0, 0));  // 赤X
      matrix.drawLine(40, 8, 24, 24, matrix.Color333(7, 0, 0));
      break;
    // ... (その他の状態)
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
      return;  // この状態には音声なし
  }

  // 音声ファイル再生
  file = new AudioFileSourceSPIFFS(filename.c_str());
  mp3->begin(file, out);
}
```

---

## 7. テスト戦略

### 7.1 単体テスト
- 状態機械遷移（Dezyne検証）
- LEDパターン正確性
- 音声ファイル再生
- シリアルコマンド解析

### 7.2 統合テスト
- ROS 2 → ESP32通信
- 状態同期
- 多言語音声切り替え
- 輝度/音量調整

### 7.3 フィールドテスト（社会的受容性）
- eHMI信号への歩行者反応
- 昼間の視認性（LED輝度）
- 音声可聴性（屋外ノイズ）
- 文化的適切性（多言語）

---

## 8. パフォーマンス目標

| メトリック | 目標 | 検証 |
|--------|--------|------------|
| LED更新レート | 50 Hz | タイムスタンプ分析 |
| 状態変更レイテンシ | <200ms | 統合テスト |
| LED輝度（昼間） | 10mで視認可能 | フィールドテスト |
| 音声音量（屋外） | 5mで可聴 | フィールドテスト |
| シリアル通信レート | 115200ボー | ハードウェア仕様 |

---

## 9. 実装フェーズ

### フェーズ1: 基本eHMI（スプリント1-2）
- ✅ ESP32ファームウェアセットアップ
- ✅ LEDストリップ制御（WS2812B）
- ✅ ROSとのシリアル通信
- ✅ 基本状態（0-10）

### フェーズ2: 高度なeHMI（スプリント3-4）
- ✅ LEDマトリックス統合（HUB75）
- ✅ 音声再生（I2S）
- ✅ 車椅子状態（21-28）
- ✅ 多言語サポート

### フェーズ3: 状態機械（スプリント5-6）
- ✅ Dezyne形式的モデル
- ✅ 状態遷移検証
- ✅ ROS 2 eHMIマネージャーノード
- ✅ 状態マッピングアルゴリズム

### フェーズ4: フィールド検証（スプリント7-8）
- ✅ 屋外視認性テスト
- ✅ 社会的受容性テスト
- ✅ 多言語音声録音
- ✅ 生産ハードウェア組み立て

---

**文書ステータス:** ドラフト
**実装ステータス:** 未開始
**必要な承認:** eHMIエンジニア、UXデザイナー、システムアーキテクト
