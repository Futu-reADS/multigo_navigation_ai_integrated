# ハードウェア-ソフトウェアインターフェース仕様

**ドキュメントID:** INT-HW-SW-001
**バージョン:** 1.0
**日付:** 2025-12-16
**ステータス:** ドラフト - クリティカル（第1週優先度3）
**分類:** 内部

**責任チーム:**
- **ソフトウェア:** Pankaj（車両ソフトウェア - ROS 2統合）
- **ハードウェア:** Tsuchiya（機械、電気、センサー、電源）+ Kiril（外装、eHMI）

**目的:** 車両ソフトウェア（ROS 2）とハードウェアコンポーネント（センサー、アクチュエータ、電源システム、eHMI）間の正確なインターフェースを定義する。

---

## ドキュメント管理

| バージョン | 日付 | 著者 | 変更内容 |
|---------|------|---------|---------|
| 1.0 | 2025-12-16 | Pankaj + Tsuchiya + Kiril | 初期ハードウェア-ソフトウェアインターフェース仕様 |

---

## 目次

1. [はじめに](#1-はじめに)
2. [ROS 2トピック - センサーからソフトウェアへ](#2-ros-2トピック---センサーからソフトウェアへ)
3. [ROS 2トピック - ソフトウェアからアクチュエータへ](#3-ros-2トピック---ソフトウェアからアクチュエータへ)
4. [シリアルプロトコル](#4-シリアルプロトコル)
5. [電源管理インターフェース](#5-電源管理インターフェース)
6. [安全システムインターフェース](#6-安全システムインターフェース)
7. [座標フレーム（TF2）](#7-座標フレームtf2)
8. [テストと検証](#8-テストと検証)

---

## 1. はじめに

### 1.1 目的

本ドキュメントは、以下の間の**正確なインターフェース**を定義します:
- **ROS 2ソフトウェア**（ナビゲーション、ドッキング、知覚、安全）
- **ハードウェアコンポーネント**（センサー、モーター、バッテリー、eHMI）

**クリティカルな依存関係:**
- ソフトウェアは、センサーがこれらのトピックに公開しないと動作できない
- ハードウェアは、ソフトウェアがコマンドを公開しないと制御できない
- 両チームは仕様通りに正確に実装しなければならない

---

### 1.2 スコープ

**スコープ内:**
- ✅ ROS 2トピック仕様（メッセージタイプ、QoS、頻度）
- ✅ シリアルプロトコル（eHMI UART、モーターCANバス、BMS）
- ✅ 電源管理インターフェース（バッテリー監視、充電）
- ✅ 安全インターフェース（E-stop、バンパー、ウォッチドッグ）
- ✅ 座標フレーム（TF2変換）

**スコープ外:**
- ❌ 内部ソフトウェアアルゴリズム（ナビゲーション、ドッキング）
- ❌ ハードウェア機械設計（CADモデル）
- ❌ TVMサーバーインターフェース（TVM_API_SPECIFICATION.mdを参照）

---

### 1.3 関連ドキュメント

- **HARDWARE_REQUIREMENTS.md** - 完全なハードウェア仕様
- **MECHANICAL_REQUIREMENTS.md** - 機械組立詳細
- **ELECTRICAL_REQUIREMENTS.md** - 電気システム詳細
- **ROS2_TOPICS.md** - 完全なROS 2トピックリファレンス（既存）
- **TVM_API_SPECIFICATION.md** - 車両 ↔ TVMサーバーインターフェース

---

## 2. ROS 2トピック - センサーからソフトウェアへ

### 2.1 3D LiDAR点群

**トピック:** `/sensors/lidar/points`

**メッセージタイプ:** `sensor_msgs/msg/PointCloud2`

**パブリッシャー:** LiDARドライバーノード（ハードウェアインターフェース）

**サブスクライバー:**
- `/perception/obstacle_detection`
- `/perception/ground_removal`
- `/localization/ndt_matcher`

**頻度:** 10 Hz（最小）

**QoS:**
```
Reliability: RELIABLE
Durability: VOLATILE
History: KEEP_LAST (depth=1)
Deadline: 150ms
Liveliness: AUTOMATIC
```

**データ要件:**
- 点形式: XYZ（float32、メートル）
- 座標フレーム: `lidar_link`
- 最小点数: 100,000点/秒
- 範囲: 0.5m～100m（屋外）
- 整列: False（非順序点群）

**メッセージフィールド:**
```cpp
sensor_msgs::msg::PointCloud2 {
  std_msgs::msg::Header header;           // タイムスタンプ + frame_id
  uint32 height = 1;                      // 非整列点群
  uint32 width = [variable];              // 点の数
  sensor_msgs::msg::PointField[] fields;  // XYZフィールド
  bool is_bigendian = false;
  uint32 point_step = 12;                 // 3つのfloat（x,y,z）= 12バイト
  uint32 row_step = width * point_step;
  uint8[] data;                           // 生の点データ
  bool is_dense = false;                  // NaNを含む可能性あり
}
```

**ハードウェア要件（Tsuchiya）:**
- LiDARドライバーをインストール（ROS 2 Humble互換）
- LiDAR IP/ポートを構成（イーサネットベースの場合）
- LiDARを高さ0.6-0.8mに取り付け
- 360° FOVをクリアに確保（遮蔽なし）

---

### 2.2 前左カメラ（ArUco検出）

**トピック:** `/sensors/camera/front_left/image_raw`

**メッセージタイプ:** `sensor_msgs/msg/Image`

**パブリッシャー:** カメラドライバーノード

**サブスクライバー:** `/docking/aruco_detector`

**頻度:** 30 Hz（最小）

**QoS:**
```
Reliability: BEST_EFFORT
Durability: VOLATILE
History: KEEP_LAST (depth=1)
```

**データ要件:**
- 解像度: 1920×1080（フルHD最小）
- エンコーディング: `bgr8`または`rgb8`
- 座標フレーム: `camera_front_left_link`
- 露出: 自動露出有効（屋外照明は変動）

**コンパニオントピック - カメラ情報:**

**トピック:** `/sensors/camera/front_left/camera_info`

**メッセージタイプ:** `sensor_msgs/msg/CameraInfo`

**頻度:** 30 Hz（画像と同じ）

**データ要件:**
- 内部キャリブレーション行列（3×3）
- 歪み係数（5パラメータ: k1, k2, p1, p2, k3）
- チェッカーボードでキャリブレート（9×6または8×6）

**ハードウェア要件（Tsuchiya）:**
- カメラを高さ0.4-0.6mに取り付け
- 角度: 約15°下向き傾斜（地面マーカーを見るため）
- 防水: IP66+筐体
- ケーブル長: USB 3.0、最大3m（長い場合はアクティブケーブル）

---

### 2.3 前右カメラ（ArUco検出）

**トピック:** `/sensors/camera/front_right/image_raw`

**トピック:** `/sensors/camera/front_right/camera_info`

**仕様:** 前左カメラと同じ（2.2を参照）

**目的:** ドッキング精度向上のためのデュアルカメラフュージョン

---

### 2.4 IMU（9-DOF）

**トピック:** `/sensors/imu`

**メッセージタイプ:** `sensor_msgs/msg/Imu`

**パブリッシャー:** IMUドライバーノード

**サブスクライバー:**
- `/localization/ekf_localization`
- `/safety/tilt_monitor`

**頻度:** 50 Hz（最小）

**QoS:**
```
Reliability: RELIABLE
Durability: VOLATILE
History: KEEP_LAST (depth=10)
Deadline: 30ms
```

**データ要件:**
- 加速度計: ±4g範囲、0.01g分解能
- ジャイロスコープ: ±500°/s範囲、0.1°/s分解能
- 磁力計: ±4ガウス範囲（方位用）
- 座標フレーム: `imu_link`
- キャリブレーション: 工場キャリブレーションまたは実行時キャリブレーション

**メッセージフィールド:**
```cpp
sensor_msgs::msg::Imu {
  std_msgs::msg::Header header;
  geometry_msgs::msg::Quaternion orientation;         // 未使用（EKFが設定）
  double[9] orientation_covariance;                   // [-1, ...]に設定
  geometry_msgs::msg::Vector3 angular_velocity;       // rad/s（ジャイロ）
  double[9] angular_velocity_covariance;
  geometry_msgs::msg::Vector3 linear_acceleration;    // m/s²（加速度）
  double[9] linear_acceleration_covariance;
}
```

**ハードウェア要件（Tsuchiya）:**
- IMUをロボット重心近くに取り付け
- IMU軸をロボットフレームに整列（X=前方、Y=左、Z=上）
- 確実な取り付け（振動なし）
- ROS 2ドライバー利用可能（例: `imu_filter_madgwick`、`phidgets_imu`）

---

### 2.5 ホイールエンコーダー（スワーブドライブ）

**トピック（4トピック、ホイールごとに1つ）:**
```
/swerve/front_left/encoder
/swerve/front_right/encoder
/swerve/rear_left/encoder
/swerve/rear_right/encoder
```

**メッセージタイプ:** カスタムメッセージ`swerve_msgs/msg/WheelEncoder`

**パブリッシャー:** スワーブドライブコントローラーノード

**サブスクライバー:** `/swerve/odometry`、`/swerve/controller`

**頻度:** 100 Hz（最小）

**カスタムメッセージ定義:**
```
# swerve_msgs/msg/WheelEncoder.msg
std_msgs/Header header
float64 drive_position      # ホイール回転（ラジアン）
float64 drive_velocity      # ホイール速度（rad/s）
float64 steer_position      # ステアリング角度（ラジアン、-π～+π）
float64 steer_velocity      # ステアリング速度（rad/s）
```

**ハードウェア要件（Tsuchiya）:**
- エンコーダー分解能: ≥1000 CPR（回転あたりカウント）
- 直交エンコーダー（A/Bチャネル）
- インターフェース: モーターコントローラー経由（CANバス）
- ホーミング: ステアリングエンコーダーは絶対位置またはホーミングルーチンが必要

---

### 2.6 安全センサー

#### **2.6.1 バンパーセンサー**

**トピック:** `/safety/bumper`

**メッセージタイプ:** `std_msgs/msg/Bool`

**パブリッシャー:** 安全監視ノード

**頻度:** 50 Hz

**データ:** いずれかのバンパーが押された場合は`true`、それ以外は`false`

**ハードウェア要件（Tsuchiya）:**
- 周囲に物理バンパー（前部、後部、側面）
- ノーマルオープンスイッチ（押されると閉じる）
- インターフェース: GPIOまたはCANバス
- 応答時間: <50ms

---

#### **2.6.2 乗客検出（重量センサー）**

**トピック:** `/safety/passenger_detected`

**メッセージタイプ:** `std_msgs/msg/Bool`

**パブリッシャー:** 安全監視ノード

**頻度:** 10 Hz

**データ:** 乗客が乗車している場合（重量>20kg）は`true`

**ハードウェア要件（Tsuchiya）:**
- 車椅子ドッキングプラットフォーム下のロードセル
- しきい値: 最小20kg（車椅子+乗客を検出）
- インターフェース: アナログ入力 → ADC → CANバス

---

#### **2.6.3 緊急停止ボタン**

**トピック:** `/safety/estop`

**メッセージタイプ:** `std_msgs/msg/Bool`

**パブリッシャー:** 安全監視ノード

**頻度:** 50 Hz

**データ:** E-stopが押された場合は`true`、それ以外は`false`

**ハードウェア要件（Tsuchiya）:**
- 大型赤色マッシュルームボタン（IEC 60947-5-5カテゴリー0）
- ラッチ式（手動でリセットするまで押された状態を保持）
- モーターコントローラーに直接配線（即座に電源遮断）
- ソフトウェア認識のためROS 2にも公開

---

### 2.7 バッテリー管理システム（BMS）

**トピック:** `/power/battery_state`

**メッセージタイプ:** `sensor_msgs/msg/BatteryState`

**パブリッシャー:** BMSインターフェースノード

**サブスクライバー:** `/power/manager`、`/tvm_client`

**頻度:** 1 Hz（低頻度で許容）

**メッセージフィールド:**
```cpp
sensor_msgs::msg::BatteryState {
  std_msgs::msg::Header header;
  float voltage;                    // バッテリー電圧（V）
  float temperature;                // バッテリー温度（°C）
  float current;                    // 電流（A、負=放電）
  float charge;                     // 残容量（Ah）
  float capacity;                   // 満充電容量（Ah）
  float design_capacity;            // 設計容量（Ah）
  float percentage;                 // 充電状態（0.0～1.0）
  uint8 power_supply_status;        // CHARGING / DISCHARGING / FULL
  uint8 power_supply_health;        // GOOD / OVERHEAT / DEAD / など
  uint8 power_supply_technology;    // LION / LIPO / など
  bool present = true;
}
```

**ハードウェア要件（Tsuchiya）:**
- CANバスインターフェース付きBMS（CANopenまたはカスタムプロトコル）
- セルごとの電圧監視（48V = 13S Li-ion、13セル）
- 電流センシング: ±100A範囲、±0.5A精度
- 温度センサー: ≥2センサー（異なる位置）
- SoC推定: クーロンカウント+電圧カーブ

---

## 3. ROS 2トピック - ソフトウェアからアクチュエータへ

### 3.1 スワーブドライブコマンド

**トピック:** `/cmd_vel`

**メッセージタイプ:** `geometry_msgs/msg/Twist`

**パブリッシャー:** ナビゲーションコントローラー（`/nav2/controller`）

**サブスクライバー:** スワーブドライブコントローラー

**頻度:** 20 Hz（ナビゲーションコントローラーがコマンドを公開）

**QoS:**
```
Reliability: RELIABLE
Durability: VOLATILE
History: KEEP_LAST (depth=1)
Deadline: 100ms（>100ms間コマンドがない場合停止）
```

**メッセージフィールド:**
```cpp
geometry_msgs::msg::Twist {
  geometry_msgs::msg::Vector3 linear;   // linear.x = 前進速度（m/s）
  geometry_msgs::msg::Vector3 angular;  // angular.z = 回転速度（rad/s）
}
```

**座標フレーム:** `base_link`（ロボット中心）

**速度制限:**
- `linear.x`: -1.0～+1.5 m/s（前進/後退）
- `linear.y`: -1.0～+1.0 m/s（左右ストレーフ）- **スワーブドライブのみ**
- `angular.z`: -1.0～+1.0 rad/s（回転）

**ハードウェアアクション（Tsuchiya）:**
スワーブコントローラーがTwistを受信 → 逆運動学を計算 → CANバス経由でモーターコマンドを送信

---

### 3.2 スワーブドライブ個別ホイールコマンド

**トピック:** `/swerve/command`

**メッセージタイプ:** カスタムメッセージ`swerve_msgs/msg/SwerveDriveCommand`

**パブリッシャー:** スワーブドライブコントローラー

**サブスクライバー:** モーターコントローラーインターフェースノード

**頻度:** 50 Hz

**カスタムメッセージ定義:**
```
# swerve_msgs/msg/SwerveDriveCommand.msg
std_msgs/Header header
SwerveModuleCommand[] modules  # 4モジュール（FL、FR、RL、RR）

# SwerveModuleCommand.msg
string module_name              # "front_left", "front_right", "rear_left", "rear_right"
float64 drive_velocity          # ホイール速度（rad/s）
float64 steer_position          # ステアリング角度（rad、-π～+π）
```

**ハードウェアアクション（Tsuchiya）:**
モーターコントローラーがコマンドを受信 → CANバス経由で4つのホイールモーター+4つのステアリングモーターを駆動

---

### 3.3 eHMI状態コマンド

**トピック:** `/ehmi/state`

**メッセージタイプ:** `std_msgs/msg/UInt8`

**パブリッシャー:** 状態管理ノード（`/vehicle/state_manager`）

**サブスクライバー:** eHMIコントローラー（ESP32シリアルブリッジ）

**頻度:** イベント駆動（状態変更時）+ 1 Hzハートビート

**QoS:**
```
Reliability: RELIABLE
Durability: TRANSIENT_LOCAL（最後の状態を保持）
History: KEEP_LAST (depth=1)
```

**状態値（EHMI_SYSTEM_REFERENCE.mdを参照）:**
| 値 | 状態 | 説明 |
|-------|-------|-------------|
| 0 | OFF | システムオフ |
| 1 | IDLE | アイドル、ミッション待機 |
| 10 | NAVIGATING | 積極的に移動中 |
| 21 | DOCKING_APPROACH | 車椅子に接近中（新規） |
| 22 | DOCKING_VISUAL_SERVO | ドッキング用ビジュアルサーボ（新規） |
| 23 | DOCKING_ATTACHED | 車椅子接続完了（新規） |
| 24 | TRANSPORTING_PASSENGER | 乗客を乗せて移動中（新規） |
| 30 | CHARGING | ステーションで充電中 |
| 40 | ERROR | エラー状態（詳細は音声で） |
| 50 | EMERGENCY_STOP | E-stop作動 |

**ハードウェアアクション（Kiril）:**
ESP32が状態を受信 → LEDパターンを更新+音声アナウンスを再生

---

### 3.4 eHMI音声コマンド

**トピック:** `/ehmi/audio`

**メッセージタイプ:** `std_msgs/msg/String`

**パブリッシャー:** 各種ノード（ナビゲーション、ドッキング、安全）

**サブスクライバー:** eHMIコントローラー

**頻度:** イベント駆動

**データ:** 音声ファイル名（例: `"start_navigation_ja.mp3"`、`"docking_complete_en.mp3"`）

**ハードウェアアクション（Kiril）:**
ESP32がファイル名を受信 → I2S経由でSDカードから音声を再生

---

## 4. シリアルプロトコル

### 4.1 eHMIシリアルプロトコル（UART）

**インターフェース:** UART（USB-シリアルアダプター）

**パラメータ:**
- ボーレート: 115200
- データビット: 8
- ストップビット: 1
- パリティ: なし
- フロー制御: なし

**プロトコル:** ASCIIテキストコマンド（改行終端）

**コマンド（ROS → ESP32）:**

| コマンド | フォーマット | 例 | 説明 |
|---------|--------|---------|-------------|
| 状態 | `STATE:<value>` | `STATE:21` | eHMI状態を設定（0-50） |
| 音声 | `AUDIO:<filename>` | `AUDIO:docking_ja.mp3` | 音声ファイルを再生 |
| ボリューム | `VOLUME:<0-100>` | `VOLUME:80` | 音声ボリュームを設定（%） |
| 言語 | `LANG:<en/ja/zh>` | `LANG:ja` | 言語を設定 |
| LED輝度 | `BRIGHTNESS:<0-255>` | `BRIGHTNESS:200` | LED輝度を設定 |

**レスポンス（ESP32 → ROS）:**

| レスポンス | フォーマット | 例 | 説明 |
|----------|--------|---------|-------------|
| OK | `OK` | `OK` | コマンド確認 |
| エラー | `ERROR:<message>` | `ERROR:File not found` | コマンド失敗 |
| 状態 | `STATE:<value>` | `STATE:21` | 現在の状態 |

**交換例:**
```
ROS → ESP32: "STATE:21\n"
ESP32 → ROS: "OK\n"

ROS → ESP32: "AUDIO:docking_approach_ja.mp3\n"
ESP32 → ROS: "OK\n"
```

**ハードウェア要件（Kiril）:**
- USB-シリアルインターフェース付きESP32-S3
- 音声ファイル用SDカード（≥2GB）
- I2S音声アンプ（例: MAX98357A）
- WS2812B LEDストリップ（≥60 LED/メートル、2-3メートル）
- HUB75 LEDマトリクス（64×32または128×64、オプション）

**ソフトウェア要件（Pankaj）:**
- ROS 2シリアルブリッジノード（`/ehmi/serial_bridge`）
- `/ehmi/state`を公開 → `STATE`コマンドを送信
- `/ehmi/audio`をサブスクライブ → `AUDIO`コマンドを送信

---

### 4.2 モーターコントローラーCANバス

**インターフェース:** CANバス（Controller Area Network）

**パラメータ:**
- ボーレート: 500 kbit/s（標準）
- プロトコル: CANopen（CiA 402 - モーション制御）
- ノードID: 1-8（4駆動モーター+4ステアリングモーター）

**CAN ID:**
| ノードID | モーター | CAN ID（ベース） | 説明 |
|---------|-------|---------------|-------------|
| 1 | FL駆動 | 0x201 | 前左ホイール駆動 |
| 2 | FLステアリング | 0x202 | 前左ステアリング |
| 3 | FR駆動 | 0x203 | 前右ホイール駆動 |
| 4 | FRステアリング | 0x204 | 前右ステアリング |
| 5 | RL駆動 | 0x205 | 後左ホイール駆動 |
| 6 | RLステアリング | 0x206 | 後左ステアリング |
| 7 | RR駆動 | 0x207 | 後右ホイール駆動 |
| 8 | RRステアリング | 0x208 | 後右ステアリング |

**CANopen SDO（Service Data Object）:**
- 構成、パラメータ設定
- オブジェクトディクショナリ（OD）アクセス

**CANopen PDO（Process Data Object）:**
- リアルタイム制御（速度コマンド）
- ステータスフィードバック（位置、速度、電流）

**PDOマッピング例（速度モード）:**

**TPDO（Transmit PDO - モーター → ROS）:**
- 位置実測値（エンコーダーティック）
- 速度実測値（RPM）
- 電流実測値（mA）

**RPDO（Receive PDO - ROS → モーター）:**
- 目標速度（RPM）
- 制御ワード（有効化、無効化、リセット）

**ハードウェア要件（Tsuchiya）:**
- CANopenサポート付きモーターコントローラー（例: ODrive、VESC、またはカスタム）
- CANバス終端抵抗（各端に120Ω）
- CANバスシールド/絶縁（EMI保護）
- 電源: 48V入力、≥80W/モーター

**ソフトウェア要件（Pankaj）:**
- ROS 2 CANopenドライバー（`ros2_canopen`パッケージ）
- モーターコントローラー用EDSファイル（Electronic Data Sheet）
- スワーブコントローラーノードがPDO経由で速度コマンドを送信

---

### 4.3 バッテリー管理システムCANバス

**インターフェース:** CANバス（モーターコントローラーと共有または別個）

**パラメータ:**
- ボーレート: 250 kbit/s（低速、クリティカル度低い）
- プロトコル: カスタムまたはCANopen

**メッセージ（BMS → ROS）:**

| CAN ID | データ | 頻度 |
|--------|------|-----------|
| 0x300 | セル電圧（13セル、各2バイト） | 1 Hz |
| 0x301 | 総電圧、電流、SoC | 1 Hz |
| 0x302 | 温度（2センサー）、ステータス | 1 Hz |
| 0x303 | 警告、障害、充電状態 | イベント駆動 |

**CANフレーム例（総電圧、電流、SoC）:**
```
CAN ID: 0x301
Data[0-1]: 総電圧（uint16、単位: 0.1V、範囲: 0-100V）
Data[2-3]: 電流（int16、単位: 0.1A、範囲: ±1000A）
Data[4]:    SoC（uint8、単位: 1%、範囲: 0-100%）
Data[5]:    充電（uint8、0=放電、1=充電）
```

**ハードウェア要件（Tsuchiya）:**
- CANインターフェース付きBMS（例: Orion BMS、カスタムボード）
- CANバス終端（120Ω）
- 電圧監視: セルあたり0.1%精度
- 電流センシング: ホール効果センサー、±100A

**ソフトウェア要件（Pankaj）:**
- ROS 2 CANドライバー（`ros2_socketcan`）
- BMSインターフェースノードがCANフレームをデコード → `/power/battery_state`に公開

---

## 5. 電源管理インターフェース

### 5.1 低バッテリー動作

**トリガー:** SoC < 30%

**ソフトウェアアクション（Pankaj）:**
1. `/diagnostics`に警告を公開
2. ミッション中の場合: 現在のミッションを完了後、充電ステーションに戻る
3. アイドル中の場合: 充電ステーションへナビゲート
4. SoC > 40%まで新規ミッションを拒否

**ハードウェアアクション（Tsuchiya）:**
- BMSがCANバス経由で警告（警告フラグ設定）
- eHMIが低バッテリー警告を表示（LED色変更）

---

### 5.2 クリティカルバッテリー動作

**トリガー:** SoC < 10%

**ソフトウェアアクション（Pankaj）:**
1. 現在のミッションをキャンセル（安全停止）
2. 緊急基地帰還を作動（最短経路）
3. 最大速度を0.5 m/sに低減（エネルギー節約）
4. TVMサーバーにクリティカル警告を送信

**ハードウェアアクション（Tsuchiya）:**
- BMSがCANバス上でクリティカルフラグを設定
- eHMIがクリティカルバッテリー音声警告を再生

---

### 5.3 充電検出

**トリガー:** BMSが充電を検出（電流>0、電圧上昇）

**ソフトウェアアクション（Pankaj）:**
1. CHARGING状態に遷移
2. ナビゲーションを無効化（充電中は車両固定）
3. 充電進捗を監視（SoC更新）
4. SoC > 90%時: IDLE状態に遷移、ミッション準備完了

**ハードウェアアクション（Tsuchiya）:**
- 充電コネクタ: XT90またはAnderson Powerpole
- BMSが充電を有効化（内部リレー）
- 充電器: 48V 10A（≥500W）、満充電時自動遮断

---

## 6. 安全システムインターフェース

### 6.1 緊急停止動作

**トリガー:** `/safety/estop`トピック = `true`

**ソフトウェアアクション（Pankaj）:**
1. **即座:** すべてのモーターを停止（ゼロ速度コマンドを送信）
2. 状態をEMERGENCY_STOPに設定
3. すべてのミッションをキャンセル
4. 危険灯を作動（eHMI赤色点滅）
5. 緊急停止音声を再生
6. TVMサーバーに緊急警告を送信

**ハードウェアアクション（Tsuchiya）:**
- **直接配線:** E-stopボタンがモーターコントローラーへの電源を直接遮断（リレー）
- ラッチ式ボタン（手動リセット必要）
- ROS 2も`/safety/estop`トピック経由で通知（ソフトウェア認識）

**回復:**
1. オペレーターがE-stopボタンを手動リセット（ひねって解除）
2. ソフトウェアが`/safety/estop` = `false`を検出
3. オペレーターがTVMダッシュボードまたはローカルUI経由で再開コマンドを送信
4. 車両がIDLE状態に遷移

---

### 6.2 衝突検出（バンパー）

**トリガー:** `/safety/bumper` = `true`

**ソフトウェアアクション（Pankaj）:**
1. **即座:** すべてのモーターを停止（ゼロ速度）
2. 0.5メートル後退（衝突から脱出）
3. 2秒待機
4. バンパーがまだ押されている場合: 停止、オペレーターに警告
5. バンパーが解除された場合: 障害物を迂回するルート再計画を試行

**ハードウェアアクション（Tsuchiya）:**
- マイクロスイッチ付き物理バンパー（ノーマルオープン）
- 応答時間: <50ms（機械的）

---

### 6.3 傾斜検出（IMU）

**トリガー:** 傾斜角度>15°（ロールまたはピッチ）

**ソフトウェアアクション（Pankaj）:**
1. **即座:** すべてのモーターを停止
2. 状態をERROR（傾斜検出）に設定
3. 警告音声を再生
4. TVM経由でオペレーターに警告

**ハードウェアアクション:**
- IMUが傾斜を継続的に監視
- ソフトウェアがIMUデータからロール/ピッチを計算

---

## 7. 座標フレーム（TF2）

### 7.1 フレーム階層

```
map                          （グローバル参照フレーム、固定）
 └─ odom                     （オドメトリフレーム、時間とともにドリフト）
     └─ base_link            （ロボット中心、地面レベル）
         ├─ lidar_link       （LiDARセンサーフレーム）
         ├─ imu_link         （IMUセンサーフレーム）
         ├─ camera_front_left_link   （前左カメラ）
         ├─ camera_front_right_link  （前右カメラ）
         ├─ front_left_wheel_link    （スワーブモジュール）
         ├─ front_right_wheel_link
         ├─ rear_left_wheel_link
         └─ rear_right_wheel_link
```

### 7.2 フレーム定義

**`map` → `odom`:**
- 公開元: `/localization/ndt_matcher`（NDTローカライゼーション）
- 頻度: 10 Hz
- LiDARスキャンマッチングを使用してオドメトリのドリフトを補正

**`odom` → `base_link`:**
- 公開元: `/swerve/odometry`（ホイールエンコーダー）
- 頻度: 50 Hz
- ホイールエンコーダーからのデッドレコニング

**`base_link` → センサーフレーム:**
- 公開元: `/robot_state_publisher`（URDFからの静的変換）
- 頻度: 静的（起動時に一度公開）

---

### 7.3 静的変換要件（Tsuchiya + Pankaj）

**ハードウェアチーム（Tsuchiya）が提供すべき内容:**
- LiDAR取り付け位置（base_linkからのX、Y、Zオフセット）
- カメラ取り付け位置（X、Y、Z、ロール、ピッチ、ヨー）
- IMU取り付け位置と向き

**ソフトウェアチーム（Pankaj）が実施すべき内容:**
- 正確な変換を含むURDFロボット記述を作成
- `robot_state_publisher`経由で静的変換を公開
- カメラ外部パラメータをキャリブレート（ステレオフュージョン用のカメラ間変換）

**静的変換例:**
```yaml
# LiDAR取り付け（URDFスニペット）
<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.2 0 0.7" rpy="0 0 0"/>  # 前方20cm、上方70cm
</joint>

# 前左カメラ
<joint name="camera_fl_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_front_left_link"/>
  <origin xyz="0.3 0.15 0.5" rpy="0 0.26 0"/>  # 前方30cm、左15cm、上方50cm、下向き傾斜15°
</joint>
```

---

## 8. テストと検証

### 8.1 ハードウェアインターフェーステスト

**センサーテスト（Tsuchiya）:**
| テスト | 手順 | 合格基準 |
|------|-----------|---------------|
| LiDARデータ | `ros2 topic echo /sensors/lidar/points` | 10 Hz、>100k点 |
| カメラ画像 | `ros2 topic echo /sensors/camera/front_left/image_raw` | 30 Hz、1920×1080 |
| IMUデータ | `ros2 topic echo /sensors/imu` | 50 Hz、NaN値なし |
| エンコーダー | `ros2 topic echo /swerve/front_left/encoder` | 100 Hz、位置更新 |
| バッテリー | `ros2 topic echo /power/battery_state` | 1 Hz、有効なSoC（0-1.0） |

**アクチュエーターテスト（Tsuchiya）:**
| テスト | 手順 | 合格基準 |
|------|-----------|---------------|
| モーターコマンド | `/cmd_vel`に公開、ホイールを観察 | ホイール回転、方向正しい |
| eHMI状態 | `/ehmi/state`に公開、LEDを観察 | LEDパターン変化 |
| eHMI音声 | `/ehmi/audio`に公開、聴く | 音声が正しく再生 |

---

### 8.2 統合テスト（Pankaj + Tsuchiya）

**テスト1: センサーからソフトウェアへのパイプライン**
1. ハードウェアがセンサーデータを公開（LiDAR、カメラ、IMU）
2. ソフトウェアがデータを受信、処理、診断を公開
3. **合格:** すべてのセンサーが正しい頻度で公開、`/diagnostics`にエラーなし

**テスト2: ソフトウェアからアクチュエーターへのパイプライン**
1. ソフトウェアが`/cmd_vel`を公開（例: 前進0.5 m/s）
2. スワーブコントローラーがホイールコマンドを計算
3. モーターがコマンドを実行
4. エンコーダーがオドメトリにフィードバック
5. **合格:** ロボットが0.5 m/sで前進（±10%）

**テスト3: 緊急停止**
1. E-stopボタンを押す
2. モーターが即座に電源遮断（<100ms）
3. ソフトウェアが`/safety/estop` = trueを検出
4. eHMIが緊急状態を表示
5. **合格:** 車両が0.5秒以内に停止

**テスト4: 低バッテリー**
1. 低バッテリーをシミュレート（SoC = 25%）
2. ソフトウェアが低バッテリーを検出、充電ステーションへナビゲート
3. **合格:** 車両が基地帰還ミッションを完了

---

### 8.3 受入基準

**ハードウェア-ソフトウェアインターフェースは以下の場合に受入:**
- ✅ すべてのセンサーが正しい頻度で公開（LiDAR 10Hz、カメラ30Hz、IMU 50Hz、エンコーダー100Hz）
- ✅ すべてのアクチュエーターがコマンドに応答（モーター、eHMI）
- ✅ 緊急停止が動作（直接配線+ソフトウェア）
- ✅ バッテリー監視が機能（SoC、電圧、電流）
- ✅ TF2変換が正しく公開（robot_state_publisher）
- ✅ 統合テストが合格（>95%成功率）
- ✅ 通常動作下でデータ損失やタイミング違反なし

---

## ドキュメントステータス

**ステータス:** ドラフト - レビュー待ち
**次回レビュー:** 2025-12-17（ソフトウェア+ハードウェアチームとの共同レビュー）
**署名が必要:** Pankaj（ソフトウェア）+ Tsuchiya（ハードウェア）+ Kiril（eHMI）

---

## 付録

### 付録A: ROS 2パッケージ依存関係

**ソフトウェアチーム（Pankaj）がインストールすべき内容:**
```bash
# センサードライバー
sudo apt install ros-humble-urg-node              # LiDAR（例）
sudo apt install ros-humble-usb-cam               # USBカメラ
sudo apt install ros-humble-imu-filter-madgwick   # IMUフィルタリング

# CANバス
sudo apt install ros-humble-ros2-socketcan
sudo apt install ros-humble-ros2-canopen

# シリアル通信
sudo apt install ros-humble-serial-driver

# TF2
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-robot-state-publisher
```

---

### 付録B: ハードウェアチェックリスト（Tsuchiya + Kiril）

**統合テスト前:**
- ☐ LiDAR設置、電源投入、IP構成（イーサネットの場合）
- ☐ カメラ設置、キャリブレート済み（チェッカーボードキャリブレーション）
- ☐ IMU設置、軸をロボットフレームに整列
- ☐ スワーブドライブモーター設置、エンコーダー動作
- ☐ BMS設置、CANバス接続
- ☐ E-stopボタン設置、モーターコントローラーに直接配線
- ☐ バンパースイッチ設置、テスト済み
- ☐ eHMI（ESP32 + LED + 音声）設置、シリアル接続
- ☐ 配電完了、ヒューズ設置
- ☐ ケーブル管理完了、ストレインリリーフ
- ☐ すべてのコネクタにラベル付け（センサー名、ピン番号）

---

### 付録C: 起動ファイル例

**例: すべてのハードウェアドライバーを起動**

```python
# hardware_drivers.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # LiDAR
        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='lidar_driver',
            parameters=[{'ip_address': '192.168.1.10'}],
            remappings=[('scan', '/sensors/lidar/scan')]
        ),

        # カメラ
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera_front_left',
            parameters=[{'video_device': '/dev/video0'}],
            remappings=[('image_raw', '/sensors/camera/front_left/image_raw')]
        ),

        # IMU
        Node(
            package='phidgets_imu',
            executable='phidgets_imu_node',
            name='imu_driver',
            remappings=[('imu/data', '/sensors/imu')]
        ),

        # eHMIシリアルブリッジ
        Node(
            package='ehmi_interface',
            executable='serial_bridge',
            name='ehmi_serial_bridge',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'baud_rate': 115200}]
        ),

        # ロボット状態パブリッシャー（TF2）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(['cat ', robot_urdf])}]
        ),
    ])
```

---

**ドキュメント終了**

**クリティカルな次のステップ:**
1. ⚠️ **ハードウェアチーム（Tsuchiya + Kiril）:** センサー取り付け要件をレビュー
2. ⚠️ **ソフトウェアチーム（Pankaj）:** ROS 2トピック仕様をレビュー
3. ⚠️ **第1週5日目:** 両チームによる署名
4. ⚠️ **第2週:** ハードウェア組立開始（Tsuchiya）+ ソフトウェア統合（Pankaj）

**質問/明確化:** Pankaj（ソフトウェア）またはTsuchiya（ハードウェア）に連絡
