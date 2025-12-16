# 全体システムアーキテクチャ - 車椅子輸送ロボット

**プロジェクト:** スワーブドライブ式屋外優先車椅子輸送ロボット
**ドキュメントタイプ:** システムアーキテクチャドキュメント
**ステータス:** ドラフト v1.0
**日付:** 2025年12月15日
**参照:** ParcelPal (Autoware) 実績アーキテクチャをベースに

---

## 1. アーキテクチャ概要

### 1.1 設計哲学

**屋外優先、屋内互換**
- すべてのコンポーネントは屋外堅牢性を考慮して選定
- 屋内動作は性能向上モードとして機能
- GPS依存なし（コスト削減 + 信頼性向上）

**関心の分離**
- 明確なインターフェースを持つ独立したサブシステム
- 並行開発可能
- モジュラーテストと統合

**実績あるアーキテクチャに基づく（ParcelPal）**
- ParcelPal Autowareスタックの60-70%を再利用
- 新規開発はスワーブドライブ + ドッキングに集中
- 低リスク、高速開発

---

## 2. システムアーキテクチャ図

```
┌────────────────────────────────────────────────────────────────────┐
│                    車椅子輸送ロボット                              │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │              アプリケーション層                               │ │
│  │                                                              │ │
│  │  ┌─────────────────────────────────────────────────────┐    │ │
│  │  │    Wheelchair Master （メインオーケストレーション） │    │ │
│  │  │    • ステートマシン（9状態 + 車椅子状態）          │    │ │
│  │  │    • ミッション計画 & キュー管理                   │    │ │
│  │  │    • 安全監視 & ジオフェンシング                   │    │ │
│  │  │    • バッテリー管理 & 自動充電                     │    │ │
│  │  └─────────────┬───────────────────────────────────────┘    │ │
│  │                │                                              │ │
│  └────────────────┼──────────────────────────────────────────────┘ │
│                   │                                                │
│  ┌────────────────┼──────────────────────────────────────────────┐ │
│  │                │         サブシステム層                       │ │
│  │    ┌───────────┴────┬──────────┬──────────┬──────────────┐   │ │
│  │    ▼                ▼          ▼          ▼              ▼   │ │
│  │ ┌────────┐  ┌──────────┐  ┌────────┐  ┌────────┐  ┌───────┐ │ │
│  │ │   UI   │  │ Autoware │  │ Swerve │  │Docking │  │ eHMI  │ │ │
│  │ │ (React)│  │   Nav    │  │ Drive  │  │ (ArUco)│  │(ESP32)│ │ │
│  │ │        │  │  Stack   │  │制御    │  │ Visual │  │LED+   │ │ │
│  │ │タッチ  │  │          │  │        │  │Servoing│  │Audio  │ │ │
│  │ │スク    │  │Nav2+NDT  │  │ROS2    │  │        │  │       │ │ │
│  │ │リーン  │  └────┬─────┘  └───┬────┘  └───┬────┘  └───────┘ │ │
│  │ └────────┘       │            │           │                  │ │
│  └──────────────────┼────────────┼───────────┼──────────────────┘ │
│                     │            │           │                    │
│  ┌──────────────────┼────────────┼───────────┼──────────────────┐ │
│  │                  │   ミドルウェア層 (ROS 2 Humble)            │ │
│  │                  │                                            │ │
│  │    Topics ───────┼────────────┼───────────┼───────────────┐  │ │
│  │    Services ─────┼────────────┼───────────┼───────────────┤  │ │
│  │    Actions ──────┼────────────┼───────────┼───────────────┤  │ │
│  │    TF2 ──────────┼────────────┼───────────┼───────────────┘  │ │
│  └──────────────────┼────────────┼───────────┼──────────────────┘ │
│                     │            │           │                    │
│  ┌──────────────────┼────────────┼───────────┼──────────────────┐ │
│  │         知覚 & アクチュエーション層                            │ │
│  │                  │            │           │                    │ │
│  │    ┌─────────────▼────────────▼───────────▼────────┐          │ │
│  │    │  センサー                    アクチュエーター  │          │ │
│  │    │  • 3D LiDAR (360°)          • 4× スワーブ    │          │ │
│  │    │  • 2× RGBカメラ                モジュール     │          │ │
│  │    │  • 9軸IMU                    • 結合機構       │          │ │
│  │    │  • ホイールエンコーダ        • 緊急停止       │          │ │
│  │    │  • 在席センサー              • ライト         │          │ │
│  │    │  • ArUcoマーカー（受動）                      │          │ │
│  │    └───────────────────────────────────────────────┘          │ │
│  └──────────────────────────────────────────────────────────────┘ │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │              コンピュートプラットフォーム                     │ │
│  │  GMKtec Nucbox K6 (Ryzen 7 7840HS, 32GB, Radeon 780M)       │ │
│  │  Ubuntu 22.04 LTS, ROS 2 Humble                             │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘
```

---

## 3. サブシステム説明

### 3.1 Wheelchair Master（メインオーケストレーション）

**言語:** Python 3.10
**目的:** システム全体の状態管理とミッション計画

**主要機能:**
1. **ステートマシン制御** - 9つの動作状態管理
2. **ミッション計画** - 複数停車点ナビゲーション
3. **安全監視** - 4層安全システム統合
4. **バッテリー管理** - 充電ステーション自動帰還（<30%）

**ステートマシン（9状態）:**
```
IDLE (待機)
  ↓ 乗客呼び出し
APPROACHING_WHEELCHAIR (車椅子接近中)
  ↓ 接近ゾーン到達
SEARCHING_ARUCO (ArUcoマーカー検索中)
  ↓ マーカー検出
DOCKING (ドッキング中)
  ↓ 機械的結合確認
WHEELCHAIR_ATTACHED (車椅子接続済み)
  ↓ 輸送開始
TRANSPORTING (輸送中)
  ↓ 目的地到着
ARRIVED_DESTINATION (目的地到着)
  ↓ 切り離し開始
UNDOCKING (切り離し中)
  ↓ 切り離し確認
WHEELCHAIR_DETACHED (車椅子切り離し済み)
  ↓ 次ミッションまたはIDLE
```

**ROS 2インターフェース:**
- パブリッシャー: `/robot_state` (std_msgs/String) - 20Hz
- サブスクライバー: `/mission_queue` (custom_msgs/Mission)
- サービスサーバー: `/emergency_stop` (std_srvs/Trigger)

---

### 3.2 Autoware ナビゲーションスタック（再利用 - ParcelPal）

**コンポーネント:**
- **ローカライゼーション:** NDTスキャンマッチャー（SLAM Toolbox）
- **経路計画:** Nav2 (Hybrid A* グローバル, DWA/TEB ローカル)
- **知覚:** PCLベースLiDAR処理

**主要機能:**
1. **手動SLAMマッピング** - 1回実行（オフライン、ループクロージャ付き）
2. **NDTローカライゼーション** - オンライン、ドリフトなし、±10cm精度
3. **障害物回避** - LiDAR + バイナリ検出（深層学習不要）
4. **経路計画** - 500m-1km範囲、GPS不要

**ParcelPalからの再利用率:** 100%（設定のみ調整）

**ROS 2インターフェース:**
- パブリッシャー: `/localization/pose` (PoseStamped) - 10Hz
- サブスクライバー: `/points_raw` (PointCloud2), `/goal_pose` (PoseStamped)
- アクションサーバー: `/navigate_to_pose` (nav2_msgs/NavigateToPose)

---

### 3.3 スワーブドライブコントローラ（新規 - C++17）

**目的:** 4輪独立ステアリング + ドライブ制御

**逆運動学:**
```cpp
// 入力: cmd_vel (vx, vy, ω)
// 出力: 各ホイール (speed, angle)

// ホイール位置（ベースリンク中心から）
wheel_positions = {
  FL: (+L/2, +W/2),  // 前左
  FR: (+L/2, -W/2),  // 前右
  RL: (-L/2, +W/2),  // 後左
  RR: (-L/2, -W/2)   // 後右
}

// 各ホイールの速度と角度計算
for each wheel:
  vx_wheel = vx - ω * wheel_y
  vy_wheel = vy + ω * wheel_x

  wheel_speed = sqrt(vx_wheel² + vy_wheel²)
  wheel_angle = atan2(vy_wheel, vx_wheel)
```

**設定パラメータ:**
```yaml
swerve_drive_controller:
  wheel_base: 0.6        # メートル（前後軸間距離）
  track_width: 0.5       # メートル（左右輪間距離）
  wheel_radius: 0.0825   # メートル (6.5インチ)
  max_linear_velocity: 1.5   # m/s
  max_angular_velocity: 1.0  # rad/s
  max_wheel_acceleration: 2.0  # m/s²
```

**ROS 2インターフェース:**
- サブスクライバー: `/cmd_vel` (Twist) - 20Hz
- パブリッシャー: `/wheel_states` (custom_msgs/SwerveModuleStates) - 50Hz
- サービス: `/reset_odometry` (std_srvs/Trigger)

---

### 3.4 車椅子ドッキングサブシステム（新規 - C++17 + OpenCV）

**目的:** ±2-5mm精度ドッキング（屋外）、±1mm目標（屋内）

**4段階ドッキングアプローチ:**

**段階1: 粗ナビゲーション（Autoware）**
- 目標: 車椅子位置の±1m以内へナビゲート
- 入力: 保存マップ上の車椅子座標
- 出力: ロボットを接近ゾーンへ移動

**段階2: ArUcoマーカー検出（OpenCV）**
- 検出範囲: 0.5-5m
- 辞書: DICT_4X4_50
- マーカーサイズ: 150mm × 150mm
- 検出レート: 10Hz最低
- カメラ: 2× RGB（ステレオまたは冗長性）

**段階3: ビジュアルサーボイング（IBVS）**
```cpp
// Image-Based Visual Servoing (IBVS)
// 目標: マーカー中心を画像中心に整列

error_x = marker_center_x - image_center_x
error_y = marker_center_y - image_center_y
error_z = desired_distance - current_distance

// PID制御
vx = Kp_x * error_x + Kd_x * derror_x
vy = Kp_y * error_y + Kd_y * derror_y
ω = Kp_θ * error_θ + Kd_θ * derror_θ

// スワーブコントローラへ送信
publish_cmd_vel(vx, vy, ω)
```

**段階4: 機械的結合**
- 結合確認センサー（接触センサー + 力センサー）
- 接続確認後のみ状態遷移許可
- 冗長確認（2つのセンサー）

**ArUco検出器設定:**
```yaml
aruco_detector:
  dictionary: DICT_4X4_50
  marker_size: 0.15      # メートル (150mm × 150mm)
  detection_rate: 10     # Hz
  min_marker_distance: 0.5  # メートル
  max_marker_distance: 5.0  # メートル
  adaptive_threshold: true  # 屋外照明変動対応
```

**ROS 2インターフェース:**
- サブスクライバー: `/camera/image_raw` (Image) - 15Hz
- パブリッシャー: `/aruco_poses` (PoseArray) - 10Hz, `/cmd_vel` (Twist) - 20Hz
- アクションサーバー: `/dock_wheelchair` (custom_msgs/DockWheelchair)

---

### 3.5 ユーザーインターフェース（適応 - ParcelPal）

**フロントエンド:** React 18 + Next.js 14, TypeScript
**バックエンド:** Node.js (ROS 2 WebSocketブリッジ)

**主要機能:**
1. **車椅子予約フロー** - ピックアップ/ドロップオフ選択
2. **リアルタイムステータス** - ロボット状態、位置、ETA表示
3. **多言語サポート** - 英語、日本語、中国語（i18n）
4. **オペレーターダッシュボード** - ミッション監視、診断

**UI画面:**
- ホーム画面: ロボット呼び出しボタン
- 予約画面: ピックアップ/ドロップオフ地点選択
- 輸送中画面: ETA、ルート表示、緊急停止ボタン
- 完了画面: 評価フィードバック

**ROS 2統合:**
- WebSocket経由でROS 2トピックをサブスクライブ
- rosbridge_server使用
- `/robot_state`, `/battery_level`, `/current_pose`をリアルタイム表示

---

### 3.6 eHMI（外部HMI）（拡張 - ParcelPal）

**ハードウェア:** ESP32-S3 + Dezyne形式モデリング
**コンポーネント:**
- LED strips (FastLED) - 前後左右
- LED matrix (HUB75 96×48) - 前面ディスプレイ
- Audio (I2S + SD card) - 多言語音声アナウンス

**車椅子固有状態（21-28）:**
```
STATE:21 → APPROACHING_WHEELCHAIR (黄色点滅 + "車椅子へ接近中")
STATE:22 → SEARCHING_ARUCO (青色回転 + "マーカー検索中")
STATE:23 → DOCKING (緑色パルス + "ドッキング中")
STATE:24 → WHEELCHAIR_ATTACHED (緑色点灯 + "車椅子接続済み")
STATE:25 → TRANSPORTING_PASSENGER (オレンジ点灯 + "乗客輸送中")
STATE:26 → ARRIVED_DESTINATION (緑色点滅 + "目的地到着")
STATE:27 → UNDOCKING (黄色パルス + "切り離し中")
STATE:28 → EMERGENCY_WHEELCHAIR (赤色点滅 + "緊急停止 - 車椅子")
```

**通信プロトコル（シリアル）:**
```
ROS 2 Node → Serial → ESP32-S3
  STATE:{num}\n          // 状態変更
  VOLUME:{0-10}\n        // 音量調整
  LANGUAGE:{EN|JP|ZH}\n  // 言語切替
```

**ParcelPalからの再利用:** 90%（新規状態21-28のみ追加）

---

## 4. データフロー図

### 4.1 通常動作フロー

```
┌──────────┐
│ユーザー  │ (タッチスクリーン予約)
└────┬─────┘
     │ /mission_queue
     ▼
┌────────────────┐
│Wheelchair      │ ステートマシン更新: IDLE → APPROACHING
│Master          │
└────┬───────────┘
     │ /goal_pose
     ▼
┌────────────────┐
│Autoware Nav    │ 経路計画 + NDTローカライゼーション
│Stack           │
└────┬───────────┘
     │ /cmd_vel
     ▼
┌────────────────┐
│Swerve Drive    │ 逆運動学 → 4×(速度, 角度)
│Controller      │
└────┬───────────┘
     │ /wheel_commands
     ▼
┌────────────────┐
│モーター        │ ホイール回転 + ステアリング
│ドライバー      │
└────────────────┘
```

### 4.2 ドッキングシーケンスフロー

```
┌────────────────┐
│Wheelchair      │ 状態: APPROACHING → SEARCHING_ARUCO
│Master          │
└────┬───────────┘
     │ /dock_wheelchair (アクション呼び出し)
     ▼
┌────────────────┐
│Docking         │ ArUco検出開始
│Subsystem       │
└────┬───────────┘
     │ /camera/image_raw
     ▼
┌────────────────┐
│カメラ          │ ArUcoマーカー画像
└────┬───────────┘
     │ 画像処理
     ▼
┌────────────────┐
│ArUco Detector  │ マーカー姿勢推定
│(OpenCV)        │
└────┬───────────┘
     │ /aruco_poses
     ▼
┌────────────────┐
│Visual Servoing│ IBVS制御ループ
│Controller      │
└────┬───────────┘
     │ /cmd_vel (ビジュアルサーボイング)
     ▼
┌────────────────┐
│Swerve Drive    │ 精密移動（±2-5mm）
│Controller      │
└────┬───────────┘
     │ 機械的接触
     ▼
┌────────────────┐
│結合機構        │ 接続確認センサー
└────┬───────────┘
     │ /coupling_status
     ▼
┌────────────────┐
│Wheelchair      │ 状態: DOCKING → WHEELCHAIR_ATTACHED
│Master          │
└────────────────┘
```

---

## 5. ROS 2トピックアーキテクチャ

### 5.1 主要トピック

| トピック | メッセージ型 | パブリッシャー | サブスクライバー | レート |
|---------|------------|-------------|----------------|-------|
| `/points_raw` | PointCloud2 | LiDAR Driver | Autoware Perception | 10 Hz |
| `/camera/image_raw` | Image | Camera Driver | Docking Subsystem | 15 Hz |
| `/cmd_vel` | Twist | Nav2 / Docking | Swerve Controller | 20 Hz |
| `/localization/pose` | PoseStamped | NDT Localizer | Wheelchair Master | 10 Hz |
| `/robot_state` | String | Wheelchair Master | UI, eHMI | 20 Hz |
| `/battery_level` | Float32 | Battery Manager | Wheelchair Master, UI | 1 Hz |
| `/aruco_poses` | PoseArray | Docking Subsystem | Wheelchair Master | 10 Hz |
| `/wheel_states` | SwerveModuleStates | Swerve Controller | Diagnostics | 50 Hz |
| `/emergency_stop` | Bool | E-Stop Button | Wheelchair Master | Event-driven |
| `/occupant_detected` | Bool | Occupant Sensor | Safety Monitor | 10 Hz |

### 5.2 サービス

| サービス | 型 | サーバー | クライアント | 目的 |
|---------|---|---------|-------------|------|
| `/emergency_stop` | Trigger | Wheelchair Master | UI, Hardware Button | 緊急停止実行 |
| `/reset_odometry` | Trigger | Swerve Controller | Wheelchair Master | オドメトリリセット |
| `/get_robot_status` | GetRobotStatus | Wheelchair Master | Operator Dashboard | ステータス取得 |

### 5.3 アクション

| アクション | 型 | サーバー | クライアント | 目的 |
|----------|---|---------|-------------|------|
| `/navigate_to_pose` | NavigateToPose | Nav2 | Wheelchair Master | 目標地点へナビゲート |
| `/dock_wheelchair` | DockWheelchair | Docking Subsystem | Wheelchair Master | ドッキング実行 |

---

## 6. TF2フレームツリー

```
map (固定世界フレーム)
  └─> odom (オドメトリフレーム、NDT更新)
        └─> base_link (ロボットベース)
              ├─> front_left_wheel
              ├─> front_right_wheel
              ├─> rear_left_wheel
              ├─> rear_right_wheel
              ├─> lidar_link
              ├─> camera_left_link
              ├─> camera_right_link
              ├─> imu_link
              └─> coupling_link (車椅子結合点)
```

**主要TF変換:**
- `map → odom`: NDTスキャンマッチャーが更新（10Hz）
- `odom → base_link`: オドメトリから計算（50Hz）
- `base_link → sensor_links`: 静的変換（キャリブレーション）

---

## 7. 安全アーキテクチャ（4層）

### レイヤー4: 運用安全
- ジオフェンシング（進入禁止エリア）
- 速度制限（乗客乗車時1.0 m/s）
- 勾配検出（ピックアップ/ドロップオフ時5°制限）

### レイヤー3: 衝突回避
- LiDAR障害物検出（0.1-30m範囲）
- 歩行者クリアランス（1.5m維持）
- 動的障害物回避（Nav2）

### レイヤー2: システム監視
- センサーヘルス監視（LiDAR、カメラ、IMU）
- バッテリーレベル監視（<30%で自動帰還）
- 傾斜角監視（15°超で停止）

### レイヤー1: 緊急停止（SIL 2）
- ハードウェア緊急停止ボタン（全側面アクセス可能）
- ソフトウェア緊急停止（/emergency_stopトピック）
- 応答時間: <100ms
- 停止距離: 1.0 m/s時1.5m以内

**冗長性:**
- ハードウェア緊急停止 → ソフトウェアバイパス不可
- ソフトウェア緊急停止 → トピック + サービス両対応
- 傾斜センサー → IMU + ホイールエンコーダ冗長

---

## 8. 性能バジェット

### 8.1 コンピュートリソース

**GMKtec Nucbox K6:**
- CPU: AMD Ryzen 7 7840HS (8コア/16スレッド, 最大5.1 GHz)
- RAM: 32GB DDR5
- GPU: Radeon 780M (統合GPU、深層学習不要）

**予測使用率:**

| コンポーネント | CPU使用率 | メモリ使用量 | 備考 |
|-------------|----------|------------|------|
| Autoware Nav Stack | 30-40% | 8GB | NDT + Nav2 |
| Swerve Drive Controller | 5-10% | 500MB | リアルタイム制御 |
| Docking Subsystem | 10-15% | 2GB | OpenCV画像処理 |
| Wheelchair Master | 2-5% | 500MB | Python状態管理 |
| UI (Node.js) | 3-5% | 1GB | WebSocket + React |
| eHMI (シリアル通信) | <1% | 100MB | 軽量通信 |
| ROS 2 DDS | 5-10% | 1GB | トピック通信 |
| **合計** | **60-70%** | **~13GB** | 30%マージン |

**ネットワーク帯域幅（ROS 2 DDS）:**
- `/points_raw`: 10Hz × 500KB = 5 MB/s
- `/camera/image_raw`: 15Hz × 2MB = 30 MB/s
- その他トピック: ~5 MB/s
- **合計**: ~40 MB/s（ローカルループバック、問題なし）

---

## 9. ParcelPalとの統合

### 9.1 再利用マトリクス

| サブシステム | ParcelPal再利用率 | 適応内容 | 新規開発 |
|-----------|----------------|---------|---------|
| **Autoware Nav Stack** | 100% | 設定調整のみ | なし |
| **NDT Localization** | 100% | パラメータチューニング | なし |
| **Wheelchair Master** | 80% | 状態追加（ドッキング、輸送） | 車椅子状態 |
| **UI Framework** | 70% | 車椅子予約フロー追加 | 予約UI |
| **eHMI Controller** | 90% | 新規状態21-28追加 | 車椅子状態 |
| **Swerve Drive** | 0% | - | 完全新規 |
| **Docking Subsystem** | 0% | - | 完全新規 |

### 9.2 適応コンポーネント

**Wheelchair Master（ParcelPalからの変更）:**
- ParcelPal: 配送ロボット状態（NAVIGATING, DELIVERING, RETURNING）
- 本システム: + 車椅子状態（DOCKING, WHEELCHAIR_ATTACHED, TRANSPORTING, UNDOCKING）
- コードベース再利用: ~80%（状態遷移ロジック共通）

**UI（ParcelPalからの変更）:**
- ParcelPal: 荷物配送追跡
- 本システム: 車椅子予約 + リアルタイム輸送ステータス
- フレームワーク再利用: 100%（React + Next.js）
- コンポーネント再利用: ~70%（新規予約フロー）

**eHMI（ParcelPalからの変更）:**
- ParcelPal: 配送ロボット状態（1-20）
- 本システム: + 車椅子状態（21-28）
- ファームウェア再利用: 90%（新規状態のみ追加）

---

## 10. 未解決事項 / TBD

| 項目 | ステータス | 担当 | 目標日 |
|-----|---------|-----|-------|
| 3D LiDARモデル選定（屋外グレード） | TBD | ハードウェアチーム | スプリント2 |
| カメラ解像度（720p vs 1080p vs 4K） | TBD | ドッキングチーム | スプリント5 |
| ローカルプランナープラグイン（DWA vs TEB） | TBD | ナビゲーションチーム | スプリント4 |
| 機械的結合機構設計 | TBD | ハードウェアチーム | スプリント8 |
| バッテリー容量（Ah）と充電仕様 | TBD | ハードウェアチーム | スプリント1 |
| 在席検知センサータイプ | TBD | 安全チーム | スプリント9 |

---

## 11. 関連ドキュメント

**要件:**
- `01_REQUIREMENTS/SYSTEM_REQUIREMENTS.md` - 完全システム要件（92要件）
- `01_REQUIREMENTS/SWERVE_DRIVE_REQUIREMENTS.md` - ドライブシステム詳細要件

**設計:**
- `03_DESIGN/SWERVE_DRIVE_CONTROLLER_DESIGN.md` - 逆運動学詳細設計
- `03_DESIGN/DOCKING_SUBSYSTEM_DESIGN.md` - ArUco + ビジュアルサーボイング設計
- `03_DESIGN/STATE_MACHINE_DESIGN.md` - Wheelchair Masterステートマシン詳細

**インターフェース:**
- `04_INTERFACES/ROS2_INTERFACE_SPECIFICATION.md` - 完全ROS 2インターフェース定義
- `04_INTERFACES/EHMI_PROTOCOL_SPECIFICATION.md` - eHMIシリアルプロトコル

**開発:**
- `05_DEVELOPMENT/AGILE_ROADMAP.md` - スプリント計画とタイムライン

**参照:**
- `reference/PARCELPAL_EXPLORATION_SUMMARY.md` - ParcelPalアーキテクチャ再利用ガイド

---

**ドキュメントステータス:** ドラフト v1.0
**レビューステータス:** ステークホルダーレビュー待ち
**次回レビュー:** フェーズ1スプリント2完了後
**承認者:** システムアーキテクト、技術リード、安全リード
**バージョン履歴:**
- v1.0 (2025-12-15): 初版システムアーキテクチャドキュメント
