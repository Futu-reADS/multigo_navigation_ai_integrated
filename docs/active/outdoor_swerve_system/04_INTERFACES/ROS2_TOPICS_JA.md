# ROS 2 トピックインターフェース仕様

**ドキュメントID:** INTERFACE-TOPICS-001
**バージョン:** 1.0
**日付:** 2025-12-15
**ステータス:** ドラフト

---

## 1. 概要

本ドキュメントは、屋外車椅子搬送ロボットシステムにおけるサブシステム間通信に使用されるすべてのROS 2トピックを規定します。

**トピック命名規則:**
- システムレベル: `/robot_status`, `/emergency_stop`
- サブシステム固有: `/<サブシステム>/<機能>` (例: `/swerve/wheel_commands`)
- センサーデータ: `/sensors/<センサー種別>` (例: `/sensors/lidar`)

**QoSポリシー:**
- **RELIABLE**: 重要なコマンド、ミッションデータ（配信を保証）
- **BEST_EFFORT**: センサーデータ、ステータス更新（鮮度を優先）

---

## 2. システムレベルトピック

### 2.1 非常停止

**トピック:** `/emergency_stop`
**メッセージ型:** `std_msgs/msg/Bool`
**QoS:** `RELIABLE`, `TRANSIENT_LOCAL`, 履歴深度=1
**パブリッシャー:** UI、物理的E-Stopボタンハンドラー
**サブスクライバー:** 全サブシステム（必須）

**説明:** 非常停止信号。`data=true`の場合、すべてのサブシステムは即座に動作を停止し、安全状態に入る必要があります。

```yaml
# std_msgs/msg/Bool
bool data  # true=停止, false=リセット
```

---

### 2.2 ロボットステータス

**トピック:** `/robot_status`
**メッセージ型:** `custom_msgs/msg/RobotStatus`
**QoS:** `BEST_EFFORT`, `VOLATILE`, 履歴深度=1, 10 Hz
**パブリッシャー:** システムモニター
**サブスクライバー:** UI、ミッション管理、フリート管理（将来）

**メッセージ定義:**
```yaml
# custom_msgs/msg/RobotStatus.msg
Header header

# バッテリー
float32 battery_voltage      # V (例: 48.5)
float32 battery_percent      # % (0-100)
float32 battery_current      # A (負=放電中)

# 自己位置推定
float32 localization_quality # [0.0, 1.0] (NDT適合度スコア正規化)
geometry_msgs/Pose pose      # マップフレーム内の現在位置姿勢

# 安全性
string safety_state          # NORMAL, DEGRADED, EMERGENCY_STOP
string[] active_faults       # アクティブな障害コードのリスト

# ミッション
string mission_id            # 現在のミッションID（アイドル時は空）
string current_waypoint      # 現在のウェイポイント名
uint8 waypoints_completed    # 完了数
uint8 total_waypoints        # ルート内の合計数

# システムヘルス
float32 cpu_usage_percent    # %
float32 memory_usage_percent # %
float32 disk_usage_percent   # %
```

---

### 2.3 乗客接続状態

**トピック:** `/passenger_attached`
**メッセージ型:** `std_msgs/msg/Bool`
**QoS:** `RELIABLE`, `TRANSIENT_LOCAL`, 履歴深度=1
**パブリッシャー:** ドッキングコントローラー
**サブスクライバー:** スワーブコントローラー（速度制限）、安全監視、eHMI

**説明:** 車椅子の乗客が現在ドッキングされ、搭乗中であることを示します。

---

## 3. スワーブドライブトピック

### 3.1 ホイールコマンド

**トピック:** `/swerve/wheel_commands`
**メッセージ型:** `custom_msgs/msg/SwerveModuleCommandArray`
**QoS:** `RELIABLE`, 履歴深度=1, 20 Hz
**パブリッシャー:** スワーブドライブコントローラー
**サブスクライバー:** スワーブドライブハードウェアインターフェース

**メッセージ定義:**
```yaml
# custom_msgs/msg/SwerveModuleCommand.msg
uint8 module_id              # 0=FL, 1=FR, 2=RL, 3=RR
float64 drive_velocity       # m/s (符号付き、正=前進)
float64 steering_angle       # ラジアン [-π, π]

---

# custom_msgs/msg/SwerveModuleCommandArray.msg
Header header
SwerveModuleCommand[4] modules  # 固定サイズ配列
```

---

### 3.2 ジョイント状態

**トピック:** `/joint_states`
**メッセージ型:** `sensor_msgs/msg/JointState`
**QoS:** `BEST_EFFORT`, 履歴深度=1, 50 Hz
**パブリッシャー:** スワーブドライブハードウェアインターフェース
**サブスクライバー:** スワーブドライブコントローラー、オドメトリーパブリッシャー

**説明:** 全8関節（4駆動 + 4操舵）からのエンコーダーフィードバック。

```yaml
# sensor_msgs/msg/JointState
Header header
string[] name          # ["fl_drive", "fl_steer", "fr_drive", ...]
float64[] position     # ラジアン（操舵角）
float64[] velocity     # rad/s または m/s（駆動速度）
float64[] effort       # N⋅m（オプション）
```

---

### 3.3 オドメトリー

**トピック:** `/odom`
**メッセージ型:** `nav_msgs/msg/Odometry`
**QoS:** `BEST_EFFORT`, 履歴深度=1, 50 Hz
**パブリッシャー:** スワーブドライブコントローラー
**サブスクライバー:** ナビゲーション、自己位置推定（センサー融合）

**説明:** ホイールエンコーダーからのデッドレコニングオドメトリー（odom → base_link変換）。

---

## 4. ナビゲーショントピック

### 4.1 コマンド速度

**トピック:** `/cmd_vel`
**メッセージ型:** `geometry_msgs/msg/Twist`
**QoS:** `RELIABLE`, 履歴深度=1, 20 Hz
**パブリッシャー:** ナビゲーションコントローラー（Nav2）
**サブスクライバー:** スワーブドライブコントローラー、安全監視

**説明:** base_linkフレームでの指令速度。

```yaml
# geometry_msgs/msg/Twist
Vector3 linear   # [x, y, z] m/s (地上ロボットではz未使用)
Vector3 angular  # [roll, pitch, yaw] rad/s (yawのみ使用)
```

---

### 4.2 現在位置姿勢

**トピック:** `/current_pose`
**メッセージ型:** `geometry_msgs/msg/PoseStamped`
**QoS:** `BEST_EFFORT`, 履歴深度=1, 10 Hz
**パブリッシャー:** NDT自己位置推定ノード
**サブスクライバー:** ナビゲーションコントローラー、ミッション管理、UI

**説明:** マップフレーム内の現在のロボット位置姿勢（NDT自己位置推定による）。

---

### 4.3 ゴール位置姿勢

**トピック:** `/goal_pose`
**メッセージ型:** `geometry_msgs/msg/PoseStamped`
**QoS:** `RELIABLE`, 履歴深度=1
**パブリッシャー:** ミッション管理、UI（手動ゴール）
**サブスクライバー:** ナビゲーションコントローラー（Nav2）

**説明:** マップフレーム内の単一ナビゲーションゴール位置姿勢。

---

## 5. 認識トピック

### 5.1 点群（生データ）

**トピック:** `/sensors/lidar/points`
**メッセージ型:** `sensor_msgs/msg/PointCloud2`
**QoS:** `BEST_EFFORT`, 履歴深度=1, 10 Hz
**パブリッシャー:** LiDARドライバー
**サブスクライバー:** 認識パイプライン、NDT自己位置推定、可視化

**説明:** LiDARセンサーからの生3D点群（約150k点）。

---

### 5.2 障害物点群

**トピック:** `/perception/obstacles`
**メッセージ型:** `sensor_msgs/msg/PointCloud2`
**QoS:** `BEST_EFFORT`, 履歴深度=1, 20 Hz
**パブリッシャー:** 認識パイプライン（地面除去後）
**サブスクライバー:** コストマップレイヤー、衝突予測器

**説明:** フィルタリング済み障害物点（非地面、0.1m-2.0m高さ）。

---

### 5.3 コストマップ

**トピック:** `/local_costmap/costmap`
**メッセージ型:** `nav_msgs/msg/OccupancyGrid`
**QoS:** `BEST_EFFORT`, 履歴深度=1, 5 Hz
**パブリッシャー:** Nav2コストマップサーバー
**サブスクライバー:** ナビゲーションプランナー、UI（可視化）

**説明:** 障害物回避用の2Dローカルコストマップ。

---

## 6. ドッキングトピック

### 6.1 ArUco検出

**トピック:** `/docking/aruco_detections`
**メッセージ型:** `custom_msgs/msg/ArucoDetectionArray`
**QoS:** `BEST_EFFORT`, 履歴深度=1, 10 Hz
**パブリッシャー:** ArUco検出器
**サブスクライバー:** ドッキングコントローラー、UI（デバッグ可視化）

**メッセージ定義:**
```yaml
# custom_msgs/msg/ArucoDetection.msg
Header header
int32 marker_id              # ArUcoマーカーID (0-3)
geometry_msgs/Pose pose      # カメラフレーム内のマーカー位置姿勢
float32 quality_score        # 検出品質 [0.0, 1.0]
string camera_frame          # "front_camera" または "rear_camera"

---

# custom_msgs/msg/ArucoDetectionArray.msg
Header header
ArucoDetection[] detections
```

---

### 6.2 ドッキングステータス

**トピック:** `/docking/status`
**メッセージ型:** `custom_msgs/msg/DockingStatus`
**QoS:** `BEST_EFFORT`, 履歴深度=1, 10 Hz
**パブリッシャー:** ドッキングコントローラー
**サブスクライバー:** ミッション管理、UI、eHMI

**メッセージ定義:**
```yaml
# custom_msgs/msg/DockingStatus.msg
Header header
string state                 # IDLE, APPROACH, SEARCH, SERVOING, FINE_ALIGN, DOCKED, FAILED
float32 position_error       # メートル（目標までの距離）
float32 orientation_error    # ラジアン（ヨー誤差）
int32 visible_markers_count  # 現在見えているマーカー数
float32 convergence_progress # [0.0, 1.0]
```

---

## 7. 安全性トピック

### 7.1 安全ステータス

**トピック:** `/safety/status`
**メッセージ型:** `custom_msgs/msg/SafetyStatus`
**QoS:** `BEST_EFFORT`, `VOLATILE`, 履歴深度=1, 10 Hz
**パブリッシャー:** 安全監視
**サブスクライバー:** 全サブシステム、UI

**メッセージ定義:**
```yaml
# custom_msgs/msg/SafetyStatus.msg
Header header
string safety_state          # NORMAL, DEGRADED, COLLISION_AVOIDANCE, EMERGENCY_STOP
string[] active_faults       # 障害コード（例: "LIDAR_TIMEOUT", "LOCALIZATION_LOST"）
float32 min_obstacle_distance # メートル（最接近障害物）
bool emergency_stop_active   # ハードウェアE-Stopステータス
```

---

### 7.2 速度制限

**トピック:** `/safety/velocity_limit`
**メッセージ型:** `custom_msgs/msg/VelocityLimit`
**QoS:** `RELIABLE`, `TRANSIENT_LOCAL`, 履歴深度=1
**パブリッシャー:** 安全監視
**サブスクライバー:** スワーブドライブコントローラー

**メッセージ定義:**
```yaml
# custom_msgs/msg/VelocityLimit.msg
Header header
float32 max_linear_velocity  # m/s
float32 max_angular_velocity # rad/s
string reason                # 制限理由（例: "PASSENGER_ONBOARD", "SLOPE"）
```

---

## 8. UI/eHMIトピック

### 8.1 eHMI状態コマンド

**トピック:** `/ehmi/state_command`
**メッセージ型:** `custom_msgs/msg/eHMIState`
**QoS:** `RELIABLE`, 履歴深度=1
**パブリッシャー:** ミッション管理、ドッキングコントローラー
**サブスクライバー:** eHMIシリアルブリッジ

**メッセージ定義:**
```yaml
# custom_msgs/msg/eHMIState.msg
Header header
uint8 state_num              # 0-28 (車椅子状態21-28)
string language              # "en", "ja", "zh"
uint8 volume                 # 0-10
```

---

### 8.2 UIコマンド

**トピック:** `/ui/command`
**メッセージ型:** `custom_msgs/msg/UICommand`
**QoS:** `RELIABLE`, 履歴深度=10
**パブリッシャー:** タッチスクリーンUI
**サブスクライバー:** ミッション管理

**メッセージ定義:**
```yaml
# custom_msgs/msg/UICommand.msg
Header header
string command_type          # "START_MISSION", "PAUSE", "RESUME", "ABORT", "EMERGENCY_STOP"
string mission_id            # ミッションID（該当する場合）
```

---

## 9. センサートピック

### 9.1 IMUデータ

**トピック:** `/sensors/imu`
**メッセージ型:** `sensor_msgs/msg/Imu`
**QoS:** `BEST_EFFORT`, 履歴深度=1, 100 Hz
**パブリッシャー:** IMUドライバー
**サブスクライバー:** 認識（地面除去）、自己位置推定（センサー融合）

---

### 9.2 カメラ画像

**トピック:** `/sensors/camera/front/image_raw`
**メッセージ型:** `sensor_msgs/msg/Image`
**QoS:** `BEST_EFFORT`, 履歴深度=1, 10 Hz
**パブリッシャー:** カメラドライバー（前面）
**サブスクライバー:** ArUco検出器

**トピック:** `/sensors/camera/rear/image_raw`
**メッセージ型:** `sensor_msgs/msg/Image`
**QoS:** `BEST_EFFORT`, 履歴深度=1, 10 Hz
**パブリッシャー:** カメラドライバー（後面）
**サブスクライバー:** ArUco検出器

---

## 10. トピック一覧表

| トピック | 型 | QoS | レート | パブリッシャー | サブスクライバー |
|-------|------|-----|------|-----------|-------------|
| `/emergency_stop` | Bool | RELIABLE | イベント | UI, E-Stop | 全サブシステム |
| `/robot_status` | RobotStatus | BEST_EFFORT | 10 Hz | システム監視 | UI, ミッション管理 |
| `/passenger_attached` | Bool | RELIABLE | イベント | ドッキング制御 | スワーブ, 安全, eHMI |
| `/swerve/wheel_commands` | SwerveModuleCommandArray | RELIABLE | 20 Hz | スワーブ制御 | HWインターフェース |
| `/joint_states` | JointState | BEST_EFFORT | 50 Hz | HWインターフェース | スワーブ制御 |
| `/odom` | Odometry | BEST_EFFORT | 50 Hz | スワーブ制御 | ナビゲーション, 自己位置推定 |
| `/cmd_vel` | Twist | RELIABLE | 20 Hz | Nav2 | スワーブ制御, 安全 |
| `/current_pose` | PoseStamped | BEST_EFFORT | 10 Hz | NDT自己位置推定 | ナビゲーション, UI |
| `/sensors/lidar/points` | PointCloud2 | BEST_EFFORT | 10 Hz | LiDARドライバー | 認識, NDT |
| `/perception/obstacles` | PointCloud2 | BEST_EFFORT | 20 Hz | 認識 | コストマップ, 安全 |
| `/docking/aruco_detections` | ArucoDetectionArray | BEST_EFFORT | 10 Hz | ArUco検出器 | ドッキング制御 |
| `/docking/status` | DockingStatus | BEST_EFFORT | 10 Hz | ドッキング制御 | ミッション, UI, eHMI |
| `/safety/status` | SafetyStatus | BEST_EFFORT | 10 Hz | 安全監視 | 全て, UI |
| `/ehmi/state_command` | eHMIState | RELIABLE | イベント | ミッション, ドッキング | eHMIブリッジ |

**総トピック数:** 20以上のコアトピック（追加のデバッグ/可視化トピックは未記載）

---

**ドキュメントステータス:** ドラフト
**実装ステータス:** ROS 2実装準備完了
**承認要件:** システムアーキテクト、ROS統合リード
