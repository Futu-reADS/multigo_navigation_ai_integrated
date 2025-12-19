# 全体システムアーキテクチャ - 車椅子搬送ロボット

**プロジェクト:** スワーブドライブ付き屋外優先車椅子搬送ロボット
**文書タイプ:** システムアーキテクチャ文書
**ステータス:** ドラフトv1.0
**日付:** 2025年12月15日
**参照:** ParcelPal実証済みアーキテクチャにインスパイア（0%コード再利用、60%パターン）

---

## 1. アーキテクチャ概要

### 1.1 設計哲学

**屋外優先、屋内互換**
- 全コンポーネントは屋外堅牢性のために選択
- 屋内運用は強化性能モードとして
- GPS依存なし（コスト＋信頼性）

**関心の分離**
- 明確なインターフェースを持つ独立サブシステム
- 並行開発能力
- モジュラーテストと統合

**実証済みアーキテクチャにインスパイア（ParcelPal）**
- 0%コード再利用、60-70%アーキテクチャパターンをParcelPalから再利用
- ParcelPalはAutowareを使用; このシステムは標準ROS 2 + Nav2 + PCLを使用
- スワーブドライブ＋ドッキング＋NDT統合に新規開発を集中
- 実証済みアプローチによる低リスク、AI支援による高速開発

---

## 2. システムアーキテクチャ図

```
┌────────────────────────────────────────────────────────────────────┐
│                    車椅子搬送ロボット                                │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │              アプリケーション層                                │ │
│  │                                                              │ │
│  │  ┌─────────────────────────────────────────────────────┐    │ │
│  │  │    車椅子マスター（メインオーケストレーション）      │    │ │
│  │  │    • ステートマシン（9状態＋車椅子状態）             │    │ │
│  │  │    • ミッションプランニングとキュー管理              │    │ │
│  │  │    • 安全監視とジオフェンシング                      │    │ │
│  │  │    • バッテリー管理と自動充電                        │    │ │
│  │  └─────────────┬───────────────────────────────────────┘    │ │
│  │                │                                              │ │
│  └────────────────┼──────────────────────────────────────────────┘ │
│                   │                                                │
│  ┌────────────────┼──────────────────────────────────────────────┐ │
│  │                │         サブシステム層                        │ │
│  │    ┌───────────┴────┬──────────┬──────────┬──────────────┐   │ │
│  │    ▼                ▼          ▼          ▼              ▼   │ │
│  │ ┌────────┐  ┌──────────┐  ┌────────┐  ┌────────┐  ┌───────┐ │ │
│  │ │   UI   │  │   Nav    │  │ Swerve │  │Docking │  │ eHMI  │ │ │
│  │ │ (React)│  │  Stack   │  │ Drive  │  │ (ArUco)│  │(ESP32)│ │ │
│  │ │        │  │ (ROS 2)  │  │ Control│  │ Visual │  │LED+   │ │ │
│  │ │Touch   │  │          │  │        │  │Servoing│  │Audio  │ │ │
│  │ │Screen  │  │Nav2+NDT  │  │  IK/FK │  │  PBVS  │  │Serial │ │ │
│  │ └────────┘  └────┬─────┘  └───┬────┘  └───┬────┘  └───────┘ │ │
│  └──────────────────┼────────────┼───────────┼──────────────────┘ │
│                     │            │           │                    │
│  ┌──────────────────┼────────────┼───────────┼──────────────────┐ │
│  │                  │   ミドルウェア層（ROS 2 Humble）            │ │
│  │                  │                                            │ │
│  │    トピック ─────┼────────────┼───────────┼───────────────┐  │ │
│  │    サービス ─────┼────────────┼───────────┼───────────────┤  │ │
│  │    アクション ───┼────────────┼───────────┼───────────────┤  │ │
│  │    TF2 ──────────┼────────────┼───────────┼───────────────┘  │ │
│  └──────────────────┼────────────┼───────────┼──────────────────┘ │
│                     │            │           │                    │
│  ┌──────────────────┼────────────┼───────────┼──────────────────┐ │
│  │         認識とアクチュエーション層                              │ │
│  │                  │            │           │                    │ │
│  │    ┌─────────────▼────────────▼───────────▼────────┐          │ │
│  │    │  センサー                    アクチュエーター   │          │ │
│  │    │  • 3D LiDAR（360°）          • 4× スワーブ    │          │ │
│  │    │  • 2× RGBカメラ                 モジュール     │          │ │
│  │    │  • 9-DOF IMU                 • カップリング   │          │ │
│  │    │  • ホイールエンコーダー          機構          │          │ │
│  │    │  • 乗員センサー               • 緊急停止       │          │ │
│  │    │  • ArUcoマーカー（パッシブ）  • ライト         │          │ │
│  │    └───────────────────────────────────────────────┘          │ │
│  └──────────────────────────────────────────────────────────────┘ │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │              計算プラットフォーム                              │ │
│  │  GMKtec Nucbox K6（Ryzen 7 7840HS、32GB、Radeon 780M）      │ │
│  │  Ubuntu 22.04 LTS、ROS 2 Humble                             │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘
```

---

## 3. サブシステム説明

### 3.1 車椅子マスター（メインオーケストレーション）

**技術:** Python 3.10（Poetry）、ROS 2統合
**参照:** ParcelPal `ftd_master`パターンから適応

**責任:**
- ステートマシン調整（Idle → Approaching → Docking → Attached → Transporting → Arrived → Undocking → Detached）
- ミッションプランニング（単一搬送、マルチストップシャトル、スケジュールルート）
- 安全監視（乗客検出、傾斜角、ジオフェンシング）
- バッテリー管理（<30%で自動充電復帰）
- エラーリカバリーとフェイルセーフ動作

**ステートマシン:**
```
IDLE ──call──> APPROACHING_WHEELCHAIR ──marker_detected──> SEARCHING_ARUCO
  ▲                                                              │
  │                                                              ▼
  │                                                          DOCKING
  │                                                              │
  │                                                              ▼
WHEELCHAIR_DETACHED <──undock──< UNDOCKING <──arrive──< TRANSPORTING
                                      ▲                          │
                                      │                          ▼
                                 ARRIVED_DESTINATION <──attached─┘
                                                         (via WHEELCHAIR_ATTACHED)
```

**主要インターフェース:**
- 配信: `/mission/goal`、`/safety/status`、`/battery/level`
- サブスクライブ: `/docking/status`、`/nav/feedback`、`/occupant/detected`
- サービス: `/mission/start`、`/mission/cancel`、`/emergency_stop`

---

### 3.2 Autowareナビゲーションスタック（再利用 - ParcelPal）

**コンポーネント:**
- **ローカライゼーション:** NDTスキャンマッチャー（tier4_localization）
- **プランニング:** Nav2プランナープラグイン（グローバル: Hybrid A*、ローカル: DWA/TEB）
- **認識:** LiDAR障害物検出（PCLベース）
- **制御:** Nav2コントローラーサーバー

**アプローチ（ParcelPal実証済み）:**
1. **オフライン:** ループクロージャ付き手動SLAMマッピング（一度）
2. **オンライン:** 保存されたマップ上のNDTスキャンマッチャー（継続的SLAMなし）
3. **結果:** ドリフトフリーローカライゼーション、500m～1km信頼性範囲

**設定:**
```yaml
# tier4_localization_component.launch.xml
pose_source: ndt
ndt_scan_matcher:
  convergence_threshold: 0.01
  step_size: 0.1
  resolution: 1.0
```

**主要インターフェース:**
- 配信: `/localization/pose`、`/planning/path`、`/obstacles`
- サブスクライブ: `/points_raw`（LiDAR）、`/odometry/swerve`
- アクション: `/navigate_to_pose`

---

### 3.3 スワーブドライブコントローラー（新規）

**技術:** C++17、ROS 2コントローラープラグイン
**参照:** カスタム開発（ParcelPal相当なし - 差動駆動使用）

**責任:**
- 逆運動学（cmd_vel → 4車輪状態）
- 順運動学（ホイールエンコーダー → オドメトリ）
- Nav2コントローラープラグイン統合
- モーターコマンド生成（CANバス）

**運動学:**
```
入力: cmd_vel（vx、vy、ω）
出力: 4×（wheel_speed、wheel_angle）

各車輪iに対して:
  wheel_velocity[i] = f(vx、vy、ω、wheel_position[i])
  wheel_angle[i] = atan2(vy_wheel[i]、vx_wheel[i])
```

**設定:**
```yaml
swerve_drive_controller:
  wheel_base: 0.6  # メートル（前後距離）
  track_width: 0.5  # メートル（左右距離）
  wheel_radius: 0.0825  # メートル（6.5" = 165mm）
  max_linear_velocity: 1.5  # m/s
  max_angular_velocity: 1.0  # rad/s
```

**主要インターフェース:**
- サブスクライブ: `/cmd_vel`（Twist）
- 配信: `/odometry/swerve`（Odometry）、`/joint_states`
- ハードウェア: モーターコントローラーへのCANバス

---

### 3.4 車椅子ドッキングサブシステム（新規）

**技術:** C++17 + OpenCV、ROS 2ノード
**参照:** カスタム開発（ParcelPalはシンプルな配送ゾーンアプローチ）

**フェーズ:**
1. **粗ナビゲーション:** Autowareナビゲーションでドッキング接近ゾーンへ（±1m）
2. **マーカー検出:** ArUcoマーカー検出（OpenCV、0.5～5m範囲）
3. **ビジュアルサーボ:** IBVS（Image-Based Visual Servoing）で精密化（±2～5mm）
4. **機械的カップリング:** カップリング係合、取り付け確認

**ArUco設定:**
```yaml
aruco_detector:
  dictionary: DICT_4X4_50
  marker_size: 0.15  # メートル（150mm × 150mm）
  camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
  dist_coeffs: [k1, k2, p1, p2, k3]
  detection_rate: 10  # Hz
```

**ビジュアルサーボ制御:**
```
誤差: e = (marker_pose_desired - marker_pose_current)
制御: u = Kp × e + Kd × de/dt
出力: cmd_vel（微調整、最大0.1 m/s）
```

**主要インターフェース:**
- サブスクライブ: `/camera/image_raw`、`/camera/camera_info`
- 配信: `/docking/status`、`/docking/marker_pose`、`/cmd_vel`（オーバーライド）
- サービス: `/docking/start`、`/docking/cancel`
- アクション: `/dock_wheelchair`

---

### 3.5 ユーザーインターフェース（適応 - ParcelPal）

**技術:** React 18 + Next.js 14、TypeScript
**参照:** ParcelPal `ftd_parcelpal_ui`パターン（70%再利用）

**画面:**
- `/` - ホーム/アイドル画面
- `/booking` - 車椅子ピックアップ/ドロップオフ位置選択
- `/enroute` - アクティブミッションステータス、ETA、リアルタイム位置
- `/settings` - 言語、音量、システム診断

**バックエンド通信:**
- `wheelchair_master`へのWebSocketでリアルタイム更新
- ミッション送信、履歴クエリ用REST API

**主要機能:**
- 多言語（EN、JP、ZH）と音声アナウンス
- タッチ最適化（屋外手袋対応）
- バッテリーと充電ステータス表示
- 緊急停止ボタン（大きい、赤）

---

### 3.6 eHMI（外部HMI）（拡張 - ParcelPal）

**技術:** ESP32-S3、Arduino、Dezyne（フォーマルモデリング）
**参照:** ParcelPal `ehmi`パターン（90%再利用＋車椅子状態）

**コンポーネント:**
- **LEDストリップ:** FastLED（WS2812B）、動的アニメーション
- **LEDマトリクス:** HUB75 96×48パネル、アイコン＋テキスト＋GIF
- **オーディオ:** I2S DAC、SDカードからMP3再生

**車椅子用新状態:**
```cpp
21 = APPROACHING_WHEELCHAIR    // 青色ゆっくりパルス
22 = SEARCHING_ARUCO          // シアンスキャン
23 = DOCKING                  // 緑色移動パターン
24 = WHEELCHAIR_ATTACHED      // 緑色ソリッド
25 = TRANSPORTING             // 青色ブリージング（静かなオーディオ）
26 = ARRIVED_DESTINATION      // 緑色チェイス
27 = UNDOCKING                // 緑色リバース
28 = WHEELCHAIR_DETACHED      // 緑色パルス → アイドル
```

**通信:**
- シリアル（115200ボー）: `STATE:{num}\n`、`VOLUME:{0-10}\n`、`LANGUAGE:{EN|JP|ZH}\n`
- ROS 2ノード（`ehmi_state_publisher`）がROS ↔ ESP32をブリッジ

**詳細参照:** `../reference/EHMI_SYSTEM_REFERENCE.md`参照

---

## 4. データフロー

### 4.1 通常運用データフロー

```
┌─────────┐     ┌──────────┐     ┌─────────┐     ┌────────┐
│ LiDAR   │────>│ Autoware │────>│車椅子   │────>│ Swerve │
│         │     │   Nav    │     │マスター │     │ Drive  │
│カメラ   │────>│ (NDT +   │     │(State   │     │Control │
│         │     │  Nav2)   │     │ Machine)│     │        │
│ IMU     │────>│          │     │          │     │        │
└─────────┘     └────┬─────┘     └────┬─────┘     └────┬───┘
                     │                 │                 │
                     ▼                 ▼                 ▼
                ┌─────────┐       ┌────────┐       ┌────────┐
                │障害物   │       │ eHMI   │       │モーター│
                │検出     │       │        │       │ (CAN)  │
                └─────────┘       └────────┘       └────────┘
```

### 4.2 ドッキングシーケンスデータフロー

```
1. 車椅子マスター ──goal──> Autoware Nav ──navigate──> 接近ゾーン

2. ドッキングサブシステム ──marker_detected──> ArUco検出器（OpenCV）
                                              │
3. ビジュアルサーボ <─────pose_error──────────┘
        │
4. cmd_vel（微調整）──> スワーブドライブ ──> 精密ドッキング（±2～5mm）

5. カップリング機構 ──attached_confirmed──> 車椅子マスター ──state──> WHEELCHAIR_ATTACHED
```

---

## 5. ROS 2トピックアーキテクチャ

### 5.1 コアトピック

| トピック | タイプ | パブリッシャー | サブスクライバー | レート |
|-------|------|-----------|---------------|------|
| `/points_raw` | PointCloud2 | LiDARドライバー | Autowareローカライゼーション | 10 Hz |
| `/camera/image_raw` | Image | カメラドライバー | ドッキングサブシステム | 15 Hz |
| `/odometry/swerve` | Odometry | スワーブコントローラー | Autoware、マスター | 50 Hz |
| `/cmd_vel` | Twist | Nav2 / ドッキング | スワーブコントローラー | 20 Hz |
| `/localization/pose` | PoseStamped | NDTスキャンマッチャー | Nav2、マスター | 10 Hz |
| `/mission/goal` | PoseStamped | 車椅子マスター | Nav2プランナー | オンデマンド |
| `/docking/status` | String | ドッキングサブシステム | 車椅子マスター | 1 Hz |
| `/ehmi/status` | Int32 | 車椅子マスター | eHMIパブリッシャー | 変更時 |
| `/occupant/detected` | Bool | 乗員センサー | 車椅子マスター | 10 Hz |
| `/emergency_stop` | Bool | 緊急停止ボタン | 全ノード | 変更時 |

### 5.2 TF2フレームツリー

```
map
 └─> odom
      └─> base_link
           ├─> lidar_link
           ├─> camera_front_link
           ├─> camera_rear_link
           ├─> imu_link
           ├─> wheel_fl_link
           ├─> wheel_fr_link
           ├─> wheel_rl_link
           └─> wheel_rr_link
```

**主要変換:**
- `map → odom`: NDTスキャンマッチャー（ドリフト補正）
- `odom → base_link`: スワーブドライブオドメトリ
- `base_link → sensor_links`: 静的（URDF）

---

## 6. 安全アーキテクチャ

### 6.1 安全レイヤー

```
┌────────────────────────────────────────────────────────┐
│             レイヤー4: 運用安全性                        │
│  • ジオフェンシング（立入禁止ゾーン）                    │
│  • 速度制限（乗客搭乗時: 1.0 m/s）                      │
│  • 5°超勾配でのピックアップ/ドロップオフ禁止              │
└───────────────────┬────────────────────────────────────┘
                    │
┌───────────────────▼────────────────────────────────────┐
│             レイヤー3: 衝突回避                          │
│  • LiDAR障害物検出（0.1～30m）                          │
│  • 動的障害物追跡                                       │
│  • ソーシャルディスタンシング（歩行者から1.5m）          │
│  • 緊急ブレーキ（1.5m以内で停止）                       │
└───────────────────┬────────────────────────────────────┘
                    │
┌───────────────────▼────────────────────────────────────┐
│             レイヤー2: システム監視                      │
│  • センサー健全性チェック（10 Hz）                       │
│  • バッテリー監視（クリティカル<15%）                    │
│  • 傾斜角監視（>15°で停止）                             │
│  • 乗員存在（搬送のみ）                                 │
│  • フェイルセーフ: センサー喪失 → 安全停止               │
└───────────────────┬────────────────────────────────────┘
                    │
┌───────────────────▼────────────────────────────────────┐
│             レイヤー1: 緊急停止（SIL 2）                 │
│  • ハードウェア緊急停止ボタン（全側面アクセス可能）       │
│  • ソフトウェア緊急停止（ROSトピック、100ms応答）        │
│  • 即座のモーターカットオフ                              │
│  • 冗長ブレーキシステム                                  │
└────────────────────────────────────────────────────────┘
```

### 6.2 安全ステートマシン

```
NORMAL ──obstacle_detected──> CAUTION ──obstacle_close──> EMERGENCY_BRAKE
  │                              │                            │
  │                              └───obstacle_cleared────────┘
  │                                         │
  └──e_stop_pressed──────────────> E_STOP_ACTIVE
                                            │
                                    e_stop_released + reset
                                            │
                                            ▼
                                        NORMAL
```

---

## 7. 性能バジェット

### 7.1 計算リソース（GMKtec Nucbox K6）

| コンポーネント | CPU目標 | メモリ目標 | 注記 |
|-----------|-----------|---------------|-------|
| Autoware Navスタック | 30-40% | 8 GB | LiDAR処理、プランニング |
| スワーブドライブコントローラー | 5-10% | 512 MB | 運動学、モーター制御 |
| ドッキングサブシステム | 10-15% | 1 GB | OpenCV、ビジュアルサーボ |
| 車椅子マスター | 5-10% | 512 MB | ステートマシン、ミッションプランニング |
| UI + eHMI | 5% | 1 GB | Reactフロントエンド、Node.js |
| ROS 2ミドルウェア | 5-10% | 2 GB | DDS、TF2、ロギング |
| **合計** | **60-70%** | **~13 GB** | 30%マージン残し |

### 7.2 ネットワーク帯域幅（ROS 2 DDS）

| トピック | サイズ | レート | 帯域幅 |
|-------|------|------|-----------|
| PointCloud2（LiDAR） | ~500 KB | 10 Hz | 5 MB/s |
| Image（2×カメラ） | ~1 MB | 15 Hz | 30 MB/s |
| Odometry | 200 B | 50 Hz | 10 KB/s |
| その他 | - | - | ~5 MB/s |
| **合計** | | | **~40 MB/s** |

**軽減策:** DDS QoS設定を使用（クリティカルにRELIABLE、センサーにBEST_EFFORT）

---

## 8. デプロイアーキテクチャ

### 8.1 ディレクトリ構造

```
/opt/wheelchair_robot/
├── install/               # ROS 2ワークスペースインストール
│   ├── wheelchair_master/
│   ├── swerve_drive_controller/
│   ├── docking_subsystem/
│   └── autoware_*         # Autowareパッケージ
├── config/                # 設定ファイル
│   ├── params/            # ROS 2パラメータYAMLファイル
│   ├── maps/              # 保存されたSLAMマップ
│   └── calibration/       # センサー較正
├── logs/                  # ROS 2ログ、ミッションログ
├── ui/                    # React UIビルド
└── scripts/               # 起動スクリプト、メンテナンス
```

### 8.2 起動戦略

**メイン起動ファイル:** `/opt/wheelchair_robot/launch/wheelchair_robot.launch.py`

```python
# 疑似コード起動ファイル構造
LaunchDescription([
    # 1. ハードウェアドライバー
    Node(package='lidar_driver', ...),
    Node(package='camera_driver', ...),
    Node(package='imu_driver', ...),

    # 2. Autowareスタック
    IncludeLaunchDescription('autoware.launch.xml',
        launch_arguments={'map_path': map_path, ...}),

    # 3. カスタムサブシステム
    Node(package='swerve_drive_controller', ...),
    Node(package='docking_subsystem', ...),
    Node(package='wheelchair_master', ...),

    # 4. UIと監視
    Node(package='wheelchair_ui', ...),
    Node(package='ehmi_state_publisher', ...),
])
```

---

## 9. ParcelPalとの統合ポイント

### 9.1 再利用コンポーネント（変更なし/最小限）

| コンポーネント | 再利用% | ソース |
|-----------|---------|--------|
| NDTローカライゼーション | 100% | `tier4_localization_component.launch.xml` |
| LiDAR障害物検出 | 100% | Autoware認識 |
| グローバルパスプランニング | 100% | Nav2プランナーサーバー |
| バッテリー管理ロジック | 90% | `ftd_master`パターン |
| eHMIフレームワーク | 90% | `ehmi`パッケージ＋新状態 |

### 9.2 適応コンポーネント

| コンポーネント | 適応 | 理由 |
|-----------|-----------|--------|
| メインオーケストレーション | 80%再利用＋新ドッキング状態 | 車椅子特有ワークフロー |
| UI画面 | 70%再利用＋車椅子予約 | 異なるユーザーインタラクション |
| ローカルモーションプランニング | 80%再利用＋スワーブ制約 | スワーブドライブ対差動駆動 |

### 9.3 新規コンポーネント

| コンポーネント | 開発労力 | 根拠 |
|-----------|-------------------|-----------|
| スワーブドライブコントローラー | 高（ゼロから） | ParcelPalは差動駆動使用 |
| ドッキングサブシステム | 高（ゼロから） | ParcelPalはシンプルな配送ゾーン |
| 乗客安全監視 | 中 | 車椅子特有安全性 |

---

## 10. オープンアーキテクチャ質問

### 10.1 決定が必要

| 質問 | オプション | 推奨 | オーナー |
|----------|---------|----------------|-------|
| 3D LiDARモデル? | Ouster OS1-64、Velodyne VLP-16、Livox Mid-70 | TBD（Sprint 2） | ハードウェアチーム |
| カメラ解像度? | 720p、1080p、4K | 1080p（ArUco＋十分） | 認識チーム |
| ローカルプランナープラグイン? | DWA、TEB、MPC | TEB（滑らか、スワーブ対応） | Navチーム |
| カップリング機構? | 磁気、機械ラッチ、フック | TBD（設計レビュー） | Mechチーム |
| バッテリー容量? | 20Ah、30Ah、40Ah | TBD（電力バジェット） | ハードウェアチーム |

### 10.2 将来の拡張（MVP後）

- マルチロボットフリート調整（フェーズ2）
- 気象センサー統合（雨/風検出）
- 高度なHMI（ジェスチャー認識、音声制御）
- ディープラーニング認識（物体分類） - ディスクリートGPU必要
- V2X通信（vehicle-to-everything）

---

## 11. 関連文書

**参照:**
- `../reference/PARCELPAL_EXPLORATION_SUMMARY.md` - ParcelPal詳細分析
- `../reference/EHMI_SYSTEM_REFERENCE.md` - eHMI実装ガイド

**要件:**
- `../01_REQUIREMENTS/SYSTEM_REQUIREMENTS.md` - 92システム要件
- `../01_REQUIREMENTS/SWERVE_DRIVE_REQUIREMENTS.md` - 駆動システム仕様
- `../01_REQUIREMENTS/DOCKING_SYSTEM_REQUIREMENTS.md` - ドッキング精度

**詳細設計:**
- `../03_DESIGN/SWERVE_DRIVE_DETAILED_DESIGN.md` - 運動学、制御アルゴリズム
- `../03_DESIGN/DOCKING_MECHANISM_DESIGN.md` - ビジュアルサーボ、カップリング

**インターフェース:**
- `../04_INTERFACES/ROS_TOPICS_AND_SERVICES.md` - 完全なトピック/サービスリスト
- `../04_INTERFACES/HARDWARE_INTERFACES.md` - CAN、シリアル、I2C仕様

---

**文書ステータス:** ドラフトv1.0
**レビューステータス:** アーキテクチャレビューボード待ち
**次回レビュー:** フェーズ1スプリント3後
**承認者:** System Architect、リードエンジニア（Nav、認識、安全性）
**バージョン履歴:**
- v1.0（2025-12-15）: 初期アーキテクチャ文書
