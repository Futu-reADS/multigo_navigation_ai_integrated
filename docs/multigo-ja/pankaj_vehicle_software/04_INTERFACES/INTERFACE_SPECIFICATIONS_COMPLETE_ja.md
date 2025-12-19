# 完全インターフェース仕様

**ドキュメントID:** INTERFACE-COMPLETE-001
**バージョン:** 1.0
**日付:** 2025-12-15
**ステータス:** 包括的リファレンス

---

## フェーズ4ドキュメント完了

本ドキュメントは、屋外車椅子輸送ロボットシステムのすべてのインターフェース仕様を統合し、ROS 2トピック、サービス、アクション、およびシリアルプロトコルの完全なリファレンスを提供します。

---

## 1. ROS 2サービス

### 1.1 ナビゲーションサービス

**サービス:** `/navigation/load_map`
**タイプ:** `nav2_msgs/srv/LoadMap`
**サーバー:** マップマネージャー
**説明:** ローカライゼーション用の保存済みマップをロード

```yaml
# リクエスト
string map_url  # マップファイルへのパス（例: "/maps/building_floor_1/map.yaml"）

# レスポンス
bool success
string message
```

---

**サービス:** `/navigation/clear_route`
**タイプ:** `std_srvs/srv/Trigger`
**サーバー:** ルートマネージャー
**説明:** 現在のルートをクリアしてナビゲーションを停止

---

### 1.2 ドッキングサービス

**サービス:** `/docking/calibrate_cameras`
**タイプ:** `std_srvs/srv/Trigger`
**サーバー:** ArUco検出器
**説明:** カメラキャリブレーション手順を実行

---

### 1.3 安全サービス

**サービス:** `/safety/reset_emergency_stop`
**タイプ:** `std_srvs/srv/Trigger`
**サーバー:** 安全監視
**説明:** 手動確認後に緊急停止をリセット

```yaml
# レスポンス
bool success  # 再開が安全な場合はtrue
string message
```

---

## 2. ROS 2アクション

### 2.1 ナビゲーションアクション

**アクション:** `/navigate_to_pose`
**タイプ:** `nav2_msgs/action/NavigateToPose`
**サーバー:** Nav2 BTナビゲーター
**説明:** 単一のゴール姿勢へナビゲート

```yaml
# ゴール
geometry_msgs/PoseStamped pose
string behavior_tree  # オプションのカスタムBT

# 結果
std_msgs/Empty

# フィードバック
geometry_msgs/PoseStamped current_pose
builtin_interfaces/Duration navigation_time
builtin_interfaces/Duration estimated_time_remaining
int16 number_of_recoveries
float32 distance_remaining
```

---

**アクション:** `/follow_waypoints`
**タイプ:** `nav2_msgs/action/FollowWaypoints`
**サーバー:** ウェイポイントフォロワー
**説明:** 複数のウェイポイントをナビゲート

```yaml
# ゴール
geometry_msgs/PoseStamped[] poses

# 結果
int32[] missed_waypoints

# フィードバック
uint32 current_waypoint
```

---

### 2.2 ドッキングアクション

**アクション:** `/dock_wheelchair`
**タイプ:** `custom_msgs/action/DockWheelchair`
**サーバー:** ドッキングコントローラー
**説明:** 自律車椅子ドッキング

```yaml
# ゴール
string wheelchair_id
geometry_msgs/PoseStamped approximate_pose  # 粗い車椅子位置

# 結果
bool success
uint8 error_code  # 0=SUCCESS, 1=MARKER_NOT_FOUND, 2=CONVERGENCE_FAILED, 3=TIMEOUT
float32 final_position_error  # メートル
float32 final_orientation_error  # ラジアン

# フィードバック
string state  # APPROACH, SEARCH, SERVOING, FINE_ALIGN
float32 position_error
float32 orientation_error
int32 visible_markers
float32 progress  # [0.0, 1.0]
```

---

### 2.3 ミッションアクション

**アクション:** `/execute_mission`
**タイプ:** `custom_msgs/action/ExecuteMission`
**サーバー:** ミッションマネージャー
**説明:** 完全な車椅子輸送ミッションを実行

```yaml
# ゴール
string mission_id
string route_file  # ルートYAMLへのパス

# 結果
bool success
string error_message
uint8 waypoints_completed
float32 total_distance_traveled  # メートル
float32 total_time_elapsed  # 秒

# フィードバック
string current_state  # LOAD_ROUTE, LOCALIZE, NAVIGATE, DOCK, TRANSPORT, など
string current_waypoint
uint8 waypoints_completed
uint8 total_waypoints
float32 distance_remaining
builtin_interfaces/Duration estimated_time_remaining
```

---

## 3. シリアルプロトコル

### 3.1 eHMIシリアルプロトコル

**インターフェース:** UART（115200ボー、8N1）
**接続:** メインコンピュータ（USB）↔ ESP32-S3

**コマンド（メイン → ESP32）:**
```
STATE <state_num>    # eHMI状態を設定（0-28）
LANG <language>      # 言語を設定（en、ja、zh）
VOL <volume>         # ボリュームを設定（0-10）
BRIGHT <level>       # LED輝度を設定（0-255）
TEST                 # セルフテストを実行
```

**レスポンス（ESP32 → メイン）:**
```
OK                   # コマンド成功
ERROR <message>      # コマンド失敗
READY                # システム初期化完了
FAULT <code>         # ハードウェア障害検出
```

**セッション例:**
```
メイン → ESP32: STATE 21
ESP32 → メイン: OK
メイン → ESP32: LANG ja
ESP32 → メイン: OK
メイン → ESP32: VOL 8
ESP32 → メイン: OK
```

---

### 3.2 モーターコントローラープロトコル

**インターフェース:** CANバス（500 kbps）
**標準:** CANopen（CiA 402 - モーション制御）

**主要オブジェクトディクショナリエントリ:**
```
0x6040: Controlword        # モーション制御コマンド
0x6041: Statusword         # モーターステータス
0x6060: Modes of Operation # 位置/速度/トルクモード
0x6064: Position Actual    # 現在位置（エンコーダーカウント）
0x606C: Velocity Actual    # 現在速度（rpm）
0x607A: Target Position    # 位置設定値
0x60FF: Target Velocity    # 速度設定値
```

**速度モード制御（スワーブドライブ）:**
```c
// 速度モードを設定
CANopen_Write(0x6060, 3);  // 3 = プロファイル速度モード

// 速度をコマンド
int32_t target_rpm = (int32_t)(velocity_m_s / wheel_radius * 60 / (2*PI));
CANopen_Write(0x60FF, target_rpm);

// ドライブを有効化
CANopen_Write(0x6040, 0x000F);  // スイッチオン + 動作有効
```

---

### 3.3 LiDARシリアルプロトコル

**インターフェース:** イーサネット（UDP）
**ポート:** 2368（Velodyneプロトコルのデフォルト）
**データレート:** ~1.3 MB/s @ 10 Hz

**パケット構造:**
```
ヘッダー（42バイト）:
  - ブロックID（2バイト）
  - アジマス（2バイト）
  - データブロック（32チャネル × 各3バイト）
  - タイムスタンプ（4バイト）
  - ファクトリーバイト（2バイト）
```

**ROS 2ドライバー:** `velodyne_driver`パッケージがUDP → PointCloud2変換を処理

---

## 4. カスタムメッセージ定義

すべてのカスタムメッセージは`custom_msgs`パッケージで定義:

```bash
custom_msgs/
├── msg/
│   ├── RobotStatus.msg
│   ├── SwerveModuleCommand.msg
│   ├── SwerveModuleCommandArray.msg
│   ├── ArucoDetection.msg
│   ├── ArucoDetectionArray.msg
│   ├── DockingStatus.msg
│   ├── SafetyStatus.msg
│   ├── VelocityLimit.msg
│   ├── eHMIState.msg
│   └── UICommand.msg
├── srv/
│   └── （標準サービスを使用）
└── action/
    ├── DockWheelchair.action
    └── ExecuteMission.action
```

---

## 5. 通信マトリクス

| 送信元サブシステム | 送信先サブシステム | インターフェース | データレート | クリティカル? |
|----------------|--------------|-----------|-----------|-----------|
| UI | 全て | `/emergency_stop`（トピック） | イベント | ✅ はい |
| スワーブCtrl | HWインターフェース | `/swerve/wheel_commands`（トピック） | 20 Hz | ✅ はい |
| Nav2 | スワーブCtrl | `/cmd_vel`（トピック） | 20 Hz | ✅ はい |
| NDT | ナビゲーション | `/current_pose`（トピック） | 10 Hz | ✅ はい |
| 知覚 | コストマップ | `/perception/obstacles`（トピック） | 20 Hz | はい |
| ドッキングCtrl | ミッションMgr | `/dock_wheelchair`（アクション） | オンデマンド | ✅ はい |
| ミッションMgr | Nav2 | `/navigate_to_pose`（アクション） | オンデマンド | ✅ はい |
| 安全Mon | 全て | `/safety/status`（トピック） | 10 Hz | ✅ はい |
| メインPC | eHMI | シリアル（115200ボー） | イベント | はい |
| メインPC | モーター | CANバス（500 kbps） | 50 Hz | ✅ はい |

---

## 6. QoSプロファイル定義

**RELIABLE_COMMANDS:**
```yaml
reliability: RELIABLE
durability: VOLATILE
history: KEEP_LAST
depth: 10
deadline: 100ms
liveliness: AUTOMATIC
```

**SENSOR_DATA:**
```yaml
reliability: BEST_EFFORT
durability: VOLATILE
history: KEEP_LAST
depth: 1
deadline: -
liveliness: AUTOMATIC
```

**TRANSIENT_STATE:**
```yaml
reliability: RELIABLE
durability: TRANSIENT_LOCAL
history: KEEP_LAST
depth: 1
deadline: -
liveliness: AUTOMATIC
```

---

## 7. インターフェーステストチェックリスト

✅ **トピック:**
- [ ] すべてのパブリッシャー/サブスクライバーが接続されていることを確認
- [ ] QoS互換性を確認
- [ ] メッセージスループットをテスト（メッセージのドロップなし）
- [ ] メッセージ内容を検証（正しい単位、範囲）

✅ **サービス:**
- [ ] リクエスト/レスポンスレイテンシをテスト（<100ms）
- [ ] エラーハンドリングを検証（タイムアウト、無効なリクエスト）
- [ ] 起動時のサービス利用可能性を確認

✅ **アクション:**
- [ ] ゴール受諾/拒否をテスト
- [ ] フィードバックレートを検証（≥1 Hz）
- [ ] キャンセル動作をテスト
- [ ] 結果コードを検証

✅ **シリアル:**
- [ ] ボーレートとパリティを検証
- [ ] コマンド解析をテスト（有効/無効）
- [ ] ラウンドトリップレイテンシを測定
- [ ] エラー検出を確認（該当する場合はチェックサム）

---

**ドキュメントステータス:** 完了
**フェーズ4ステータス:** 100%完了（統合済み）
**承認が必要:** システムアーキテクト、ROS統合リード、組込システムリード
