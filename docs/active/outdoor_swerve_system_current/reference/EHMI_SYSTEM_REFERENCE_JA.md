# eHMIシステムリファレンス（外部ヒューマンマシンインターフェース）

**ソース:** /home/pankaj/eHMI_all/
**ハードウェア:** ESP32-S3 DevKitC-1 (N32R8 - 32MB Flash, 8MB PSRAM)
**フレームワーク:** PlatformIO + Arduino + Dezyne
**日付:** 2025年12月15日

---

## 1. システム概要

eHMI（外部ヒューマンマシンインターフェース）は、ロボットのステータスを歩行者や近くの人々に伝えるための視覚的および音声的フィードバックを提供します。主要な3つのコンポーネントで構成されています:

1. **LED Strips** (FastLED) - ロボット本体周囲の動的アニメーション
2. **LEDマトリックスディスプレイ** (HUB75 96×48) - アイコン、テキスト、GIFアニメーション
3. **音声システム** (I2S + SDカード) - 多言語音声アナウンス

---

## 2. ハードウェアプラットフォーム

### ESP32-S3 DevKitC-1仕様

**マイクロコントローラー:** ESP32-S3-WROOM-1-N32R8
- **CPU:** デュアルコア Xtensa LX7 @ 240 MHz
- **Flash:** 32MB (Octal SPI)
- **PSRAM:** 8MB (Octal SPI)
- **USB:** ネイティブ USB OTG（シリアル通信用CDC）
- **ペリフェラル:** I2S, SPI, I2C, GPIO

**シリアル通信:**
- ボーレート: 115200
- プロトコル: テキストベース（STATE:{int}\n, VOLUME:{int}\n, LANGUAGE:{string}\n）
- USBデバイスパス: `/dev/ttyACM0`

---

## 3. コンポーネント詳細

### 3.1 LED Strips（FastLED）

**ライブラリ:** FastLED 3.6.0
**LEDタイプ:** WS2812B / SK6812（設定可能）
**設定:**
- デュアル独立ストリップ（`leds` および `leds_2`）
- 設定可能LED数（NUM_LEDS, NUM_LEDS_2）
- プライマリおよびセカンダリカラーサポート

**アニメーションパターン:**
```cpp
// アニメーション構造体
struct AnimationData {
    int startLed;                      // 開始LED位置
    int mainLength;                    // ソリッドカラーセクション長
    int frontTailLength;               // フェードイン尾部長
    int rearTailLength;                // フェードアウト尾部長
    bool isFrontTail;                  // 前部尾部有効化
    bool isRearTail;                   // 後部尾部有効化
    CRGB primaryColor;                 // メインストリップ色
    CRGB secondaryColor;               // セカンダリストリップ色
    uint8_t maxBrightness;             // 最大輝度（0-255）
    int numberOfCycles;                // アニメーション繰り返し回数
    int numberOfRotationsInOneCycle;   // サイクルあたり回転数
    bool isAnimationHideAtEnd;         // 終了時アニメーション非表示
    bool isLedStartFromFront;          // 方向フラグ
};
```

**輝度フェーディング:**
- スムーズフェード用二次緩和
- 前部尾部: `brightness = maxBrightness × (position/length)²`
- 後部尾部: `brightness = maxBrightness × (remaining/length)²`

**ユースケース:**
- **ACTIVE状態:** 青色移動アニメーション（前進動作インジケーター）
- **WAITING状態:** 黄色パルス（ロボット待機中）
- **ERROR状態:** 赤色点滅（警告/エラー状態）
- **OBSTACLE_DETECTED:** オレンジスキャンパターン（障害物認識）

---

### 3.2 LEDマトリックスディスプレイ（HUB75）

**ライブラリ:** ESP32-HUB75-MatrixPanel-DMA 3.0.10
**ディスプレイ仕様:**
- 解像度: 96×48ピクセル（96×24の2パネル連結）
- インターフェース: HUB75（並列RGBデータ）
- パネル構成: 2行 × 1列
- ピクセルピッチ: P3またはP4（設定可能）

**ピンマッピング:**
```cpp
// RGBデータライン
LED_R1_PIN = 4    // 赤データ（上半分）
LED_G1_PIN = 5    // 緑データ（上半分）
LED_B1_PIN = 6    // 青データ（上半分）
LED_R2_PIN = 7    // 赤データ（下半分）
LED_G2_PIN = 15   // 緑データ（下半分）
LED_B2_PIN = 16   // 青データ（下半分）

// 制御ライン
LED_A_PIN = 8     // 行アドレスA
LED_B_PIN = 9     // 行アドレスB
LED_C_PIN = 10    // 行アドレスC
LED_D_PIN = 11    // 行アドレスD
LED_E_PIN = -1    // 行アドレスE（未使用）
LED_LAT_PIN = 12  // ラッチ
LED_OE_PIN = 13   // 出力有効（アクティブLOW）
LED_CLK_PIN = 14  // クロック
```

**表示機能:**
- GIFアニメーション（SDカードから）
- 静的アイコン/テキスト
- スクロールメッセージ
- カスタムグラフィック

---

### 3.3 音声システム（I2S + SDカード）

**ライブラリ:** ESP8266Audio 1.9.7
**音声フォーマット:** WAV（推奨）、MP3サポート
**SDカード:** SPI接続
**音量:** 0-10（10レベル）

**ピンマッピング:**
```cpp
// I2S DAC
I2S_BCLK_PIN = 17   // ビットクロック
I2S_LRC_PIN = 18    // L/Rクロック（WS）
I2S_DOUT_PIN = 19   // データアウト

// SDカードSPI
SD_CS_PIN = 21      // チップセレクト
SD_MOSI_PIN = 23    // マスターアウトスレーブイン
SD_MISO_PIN = 22    // マスターインスレーブアウト
SD_SCK_PIN = 24     // シリアルクロック
```

**音声ファイル構造:**
```
/sd/
├── EN/                    # 英語音声ファイル
│   ├── active.wav
│   ├── waiting.wav
│   ├── obstacle.wav
│   └── ...
├── JP/                    # 日本語音声ファイル
│   ├── active.wav
│   ├── waiting.wav
│   └── ...
└── ZH/                    # 中国語音声ファイル
    ├── active.wav
    ├── waiting.wav
    └── ...
```

**音声再生ロジック:**
- 状態変更時トリガー
- 言語自動選択（LANGUAGE:設定から）
- 非ブロッキング再生（I2S DMA）

---

## 4. 通信プロトコル

### 4.1 ROS 2 → ESP32（シリアル）

**プロトコル:** 改行区切りテキストコマンド
**ボーレート:** 115200
**デバイス:** `/dev/ttyACM0`（USB CDC）

**サポートコマンド:**

#### STATE コマンド
```
STATE:{number}\n
```
**例:**
- `STATE:1\n` → ACTIVE状態
- `STATE:4\n` → OBSTACLE_DETECTED状態
- `STATE:21\n` → APPROACHING_WHEELCHAIR状態（車椅子ロボット）

#### VOLUME コマンド
```
VOLUME:{0-10}\n
```
**例:**
- `VOLUME:5\n` → 音量を50%に設定
- `VOLUME:10\n` → 音量を100%（最大）に設定

#### LANGUAGE コマンド
```
LANGUAGE:{EN|JP|ZH}\n
```
**例:**
- `LANGUAGE:EN\n` → 英語に切替
- `LANGUAGE:JP\n` → 日本語に切替
- `LANGUAGE:ZH\n` → 中国語に切替

---

## 5. 状態マッピング（ParcelPal既存）

### ParcelPal配送ロボット状態（1-20）

| 状態番号 | 状態名 | LED色 | 音声 |
|--------|--------|------|------|
| 1 | ACTIVE | 青色移動 | "システム準備完了" |
| 2 | NAVIGATING | 青色回転 | "移動中" |
| 3 | ARRIVED | 緑色点滅 | "目的地到着" |
| 4 | OBSTACLE_DETECTED | オレンジ点滅 | "障害物検出" |
| 5 | ERROR | 赤色点滅 | "システムエラー" |
| 6 | CHARGING | 緑色パルス | "充電中" |
| 7 | WAITING | 黄色パルス | "待機中" |
| 8 | LOADING | 青緑回転 | "荷物積載中" |
| 9 | UNLOADING | 青緑回転 | "荷物降ろし中" |
| 10 | EMERGENCY_STOP | 赤色点灯 | "緊急停止" |

---

## 6. 車椅子ロボット拡張状態（21-28）

### 新規状態（車椅子輸送用）

| 状態番号 | 状態名 | LED色 | 音声（EN） | 音声（JP） |
|--------|--------|------|----------|----------|
| 21 | APPROACHING_WHEELCHAIR | 黄色点滅 | "Approaching wheelchair" | "車椅子へ接近中" |
| 22 | SEARCHING_ARUCO | 青色回転 | "Searching marker" | "マーカー検索中" |
| 23 | DOCKING | 緑色パルス | "Docking in progress" | "ドッキング中" |
| 24 | WHEELCHAIR_ATTACHED | 緑色点灯 | "Wheelchair attached" | "車椅子接続済み" |
| 25 | TRANSPORTING_PASSENGER | オレンジ点灯 | "Passenger onboard" | "乗客輸送中" |
| 26 | ARRIVED_DESTINATION | 緑色点滅 | "Arrived at destination" | "目的地到着" |
| 27 | UNDOCKING | 黄色パルス | "Undocking wheelchair" | "切り離し中" |
| 28 | EMERGENCY_WHEELCHAIR | 赤色点滅 | "Emergency stop - wheelchair" | "緊急停止 - 車椅子" |

### 実装例（C++）

```cpp
// StateMachineComponent.cpp拡張
void StateMachineComponent::handleStateChange(int newState) {
    switch(newState) {
        // ParcelPal既存状態 (1-20)
        case STATE_ACTIVE:
            setLedAnimation(BLUE_MOVING);
            playAudio("active.wav");
            break;
        
        // 車椅子ロボット新規状態 (21-28)
        case STATE_APPROACHING_WHEELCHAIR:  // 21
            setLedAnimation(YELLOW_BLINKING);
            playAudio("approaching_wheelchair.wav");
            break;
        
        case STATE_SEARCHING_ARUCO:  // 22
            setLedAnimation(BLUE_ROTATING);
            playAudio("searching_marker.wav");
            break;
        
        case STATE_DOCKING:  // 23
            setLedAnimation(GREEN_PULSE);
            playAudio("docking.wav");
            break;
        
        case STATE_WHEELCHAIR_ATTACHED:  // 24
            setLedAnimation(GREEN_SOLID);
            playAudio("wheelchair_attached.wav");
            break;
        
        case STATE_TRANSPORTING_PASSENGER:  // 25
            setLedAnimation(ORANGE_SOLID);
            playAudio("transporting_passenger.wav");
            break;
        
        case STATE_ARRIVED_DESTINATION:  // 26
            setLedAnimation(GREEN_BLINKING);
            playAudio("arrived_destination.wav");
            break;
        
        case STATE_UNDOCKING:  // 27
            setLedAnimation(YELLOW_PULSE);
            playAudio("undocking.wav");
            break;
        
        case STATE_EMERGENCY_WHEELCHAIR:  // 28
            setLedAnimation(RED_BLINKING_FAST);
            playAudio("emergency_wheelchair.wav");
            break;
    }
}
```

---

## 7. ROS 2統合

### 7.1 eHMIドライバーノード（C++17）

**パッケージ:** `ehmi_driver_package`
**ノード名:** `ehmi_state_publisher`

**パブリッシャー:**
- なし（一方向制御のみ）

**サブスクライバー:**
```cpp
// /robot_state トピックをサブスクライブ
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;

void stateCallback(const std_msgs::msg::String::SharedPtr msg) {
    int state_number = parseStateString(msg->data);
    sendSerialCommand("STATE:" + std::to_string(state_number) + "\n");
}
```

**サービス:**
```cpp
// 音量設定サービス
rclcpp::Service<std_srvs::srv::SetInt>::SharedPtr volume_service_;

void setVolumeCallback(const std_srvs::srv::SetInt::Request::SharedPtr request) {
    int volume = std::clamp(request->data, 0, 10);
    sendSerialCommand("VOLUME:" + std::to_string(volume) + "\n");
}

// 言語設定サービス
rclcpp::Service<std_srvs::srv::SetString>::SharedPtr language_service_;

void setLanguageCallback(const std_srvs::srv::SetString::Request::SharedPtr request) {
    std::string lang = request->data;  // "EN", "JP", or "ZH"
    sendSerialCommand("LANGUAGE:" + lang + "\n");
}
```

### 7.2 使用例（Python）

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetInt, SetString

class WheelchairMaster(Node):
    def __init__(self):
        super().__init__('wheelchair_master')
        
        # eHMI状態パブリッシャー
        self.ehmi_pub = self.create_publisher(String, '/robot_state', 10)
        
        # eHMIサービスクライアント
        self.volume_client = self.create_client(SetInt, '/ehmi/set_volume')
        self.language_client = self.create_client(SetString, '/ehmi/set_language')
    
    def set_state(self, state_name):
        """ロボット状態を設定しeHMIを更新"""
        msg = String()
        msg.data = state_name
        self.ehmi_pub.publish(msg)
    
    def set_ehmi_volume(self, volume):
        """eHMI音量を設定（0-10）"""
        request = SetInt.Request()
        request.data = volume
        self.volume_client.call_async(request)
    
    def set_ehmi_language(self, language):
        """eHMI言語を設定（EN, JP, ZH）"""
        request = SetString.Request()
        request.data = language
        self.language_client.call_async(request)

# 使用例
master = WheelchairMaster()

# 状態遷移例
master.set_state("APPROACHING_WHEELCHAIR")  # STATE:21
time.sleep(5)
master.set_state("SEARCHING_ARUCO")         # STATE:22
time.sleep(3)
master.set_state("DOCKING")                 # STATE:23
```

---

## 8. 開発ワークフロー

### 8.1 ファームウェア開発（PlatformIO）

**1. プロジェクト構造:**
```
eHMI_all/
├── platformio.ini           # PlatformIO設定
├── src/
│   ├── main.cpp             # メインエントリーポイント
│   ├── StateMachineComponent.cpp
│   ├── LedAnimationHandlerComponent.cpp
│   ├── AudioComponent.cpp
│   └── SerialHandlerComponent.cpp
├── include/
│   ├── constants.h          # ピン定義、設定
│   └── dezyne/              # Dezyne形式モデル
└── data/                    # SDカードファイル（音声）
```

**2. ビルド & アップロード:**
```bash
# ビルド
pio run

# ESP32-S3へアップロード
pio run --target upload

# シリアルモニタ
pio device monitor
```

**3. テスト:**
```bash
# シリアルポートにコマンド送信
echo "STATE:21" > /dev/ttyACM0
echo "VOLUME:7" > /dev/ttyACM0
echo "LANGUAGE:JP" > /dev/ttyACM0
```

### 8.2 ROS 2ドライバー開発

**1. パッケージビルド:**
```bash
cd /path/to/ros2_ws
colcon build --packages-select ehmi_driver_package
source install/setup.bash
```

**2. ノード起動:**
```bash
ros2 run ehmi_driver_package ehmi_state_publisher
```

**3. テスト:**
```bash
# 状態パブリッシュ
ros2 topic pub /robot_state std_msgs/msg/String "{data: 'DOCKING'}"

# 音量設定
ros2 service call /ehmi/set_volume std_srvs/srv/SetInt "{data: 8}"

# 言語設定
ros2 service call /ehmi/set_language std_srvs/srv/SetString "{data: 'JP'}"
```

---

## 9. 車椅子ロボット統合チェックリスト

### ハードウェア
- [ ] ESP32-S3 DevKitC-1調達（N32R8バリアント）
- [ ] LED Strips配線（WS2812B、前後左右）
- [ ] LEDマトリックス配線（HUB75 96×48）
- [ ] I2S DAC配線（音声出力用）
- [ ] SDカード挿入（音声ファイル入り）
- [ ] USB接続テスト（/dev/ttyACM0認識確認）

### ファームウェア
- [ ] ParcelPal eHMIファームウェアクローン
- [ ] 車椅子状態21-28追加（StateMachineComponent.cpp）
- [ ] 新規LEDアニメーション定義（状態21-28用）
- [ ] 音声ファイル作成（EN/JP/ZH、状態21-28用）
- [ ] ESP32-S3へアップロード
- [ ] シリアルコマンドテスト

### ROS 2統合
- [ ] ehmi_driver_packageビルド
- [ ] /robot_stateトピック接続テスト
- [ ] Wheelchair Master → eHMI通信テスト
- [ ] 音量/言語サービステスト
- [ ] 起動ファイル作成（システム起動時自動開始）

### テスト
- [ ] 全状態1-28サイクルテスト
- [ ] 多言語切替テスト（EN/JP/ZH）
- [ ] 音量レベルテスト（0-10）
- [ ] 長時間動作テスト（12時間以上）
- [ ] フィールドテスト（屋外照明、ノイズ環境）

---

## 10. トラブルシューティング

### 問題1: ESP32が/dev/ttyACM0として認識されない
**解決策:**
```bash
# USB権限確認
ls -l /dev/ttyACM0

# ユーザーをdialoutグループに追加
sudo usermod -a -G dialout $USER

# 再ログイン必要
```

### 問題2: 音声再生されない
**チェック:**
- [ ] SDカード正しく挿入
- [ ] 音声ファイルSDカードに存在（/EN/, /JP/, /ZH/）
- [ ] I2S DACピン配線正しい
- [ ] 音量>0に設定（VOLUME:コマンド）
- [ ] スピーカー/ヘッドホン接続

### 問題3: LEDマトリックス表示異常
**チェック:**
- [ ] HUB75ケーブル正しく接続
- [ ] 電源十分（LEDマトリックスは最大2A消費）
- [ ] パネルチェーン設定正しい（2行×1列）
- [ ] ピンマッピング一致（constants.h確認）

### 問題4: シリアル通信タイムアウト
**解決策:**
```cpp
// シリアルタイムアウト増加
Serial.setTimeout(1000);  // 1秒

// 改行文字確認（\nまたは\r\n）
String cmd = Serial.readStringUntil('\n');
```

---

## 11. 性能仕様

### タイミング
- **シリアルコマンド応答時間:** <100ms
- **LED状態切替時間:** <200ms
- **音声再生開始時間:** <500ms
- **eHMI全体レイテンシ:** <1秒（ROS 2パブリッシュから可視出力まで）

### リソース使用率
- **CPU使用率:** ~30%（240MHzデュアルコア）
- **メモリ使用:** ~2MB RAM（8MB PSRAM利用可能）
- **電力消費:** 
  - アイドル: ~200mA @ 5V
  - LED最大輝度: ~3A @ 5V
  - 平均動作: ~1A @ 5V

---

## 12. 関連ドキュメント

**車椅子ロボットシステム:**
- `02_ARCHITECTURE/OVERALL_SYSTEM_ARCHITECTURE.md` - eHMI統合アーキテクチャ
- `04_INTERFACES/EHMI_PROTOCOL_SPECIFICATION.md` - 詳細プロトコル仕様

**ParcelPal参照:**
- `/home/pankaj/eHMI_all/` - 完全eHMIソースコード
- `/home/pankaj/autoware.FTD/src/future_packages/ehmi/` - ROS 2統合

---

**ドキュメントステータス:** 完了
**バージョン履歴:**
- v1.0 (2025-12-15): 車椅子ロボット拡張含むeHMI初版リファレンス
