# システムアーキテクチャ - 現在と提案

**最終更新日:** 2025-12-02
**ドキュメントバージョン:** 1.0
**ステータス:** 現在 (61% 完了) → 提案 (4フェーズで100%へ)

---

## 目次

### パート1: 現在のアーキテクチャ (既存のもの)
1. [システム概要](#1-システム概要)
2. [コンポーネントアーキテクチャ](#2-コンポーネントアーキテクチャ)
3. [通信パターン](#3-通信パターン)
4. [主要アルゴリズム](#4-主要アルゴリズム)
5. [データフロー](#5-データフロー)

### パート2: 提案されたアーキテクチャ (フェーズ1-4)
6. [セーフティアーキテクチャ](#6-提案されたセーフティアーキテクチャフェーズ-1)
7. [状態管理](#7-提案された状態管理フェーズ-1)
8. [ROS 2ベストプラクティス](#8-提案されたros-2改善フェーズ-3)
9. [テストインフラストラクチャ](#9-提案されたテストアーキテクチャフェーズ-2)
10. [デプロイメントアーキテクチャ](#10-提案されたデプロイメントアーキテクチャフェーズ-4)

---

# パート1: 現在のアーキテクチャ

## 1. システム概要

### 1.1 ハイレベルアーキテクチャ

```
┌─────────────────────────────────────────────────────────────┐
│                  MultiGo Navigation System                  │
│              (ROS 2 Humble on Ubuntu 22.04)                 │
└─────────────────────────────────────────────────────────────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
   ┌────▼────┐       ┌────▼────┐       ┌────▼────┐
   │ Master  │       │  Nav &  │       │Hardware │
   │ Control │       │ Docking │       │ Drivers │
   │multigo_ │       │multigo_ │       │multigo_ │
   │ master  │       │navigation│      │ launch  │
   └────┬────┘       └────┬────┘       └────┬────┘
        │                 │                  │
        └─────────────────┴──────────────────┘
                           │
                    ┌──────▼───────┐
                    │ Calibration  │
                    │MultiGoAruco  │
                    │     Test     │
                    └──────────────┘
```

### 1.2 技術スタック

| レイヤー | 技術 |
|-------|-----------|
| **フレームワーク** | ROS 2 Humble (LTS) |
| **OS** | Ubuntu 22.04 LTS |
| **言語** | C++17, Python 3.10 |
| **ナビゲーション** | Nav2 (業界標準) |
| **SLAM** | RTAB-Map (ビジュアルSLAM) |
| **ビジョン** | OpenCV 4.x (ArUcoマーカー) |
| **ビルド** | colcon |
| **ハードウェア** | Phidget22 (モーター制御) |

### 1.3 制御階層 (5レイヤー)

```
レイヤー1: ミッション制御 (multigo_master)
         ├─→ nav_master_node: ユーザーインターフェース
         ├─→ アクションオーケストレーション
         └─→ セーフティ確認

レイヤー2: タスク実行 (multigo_navigation)
         ├─→ nav_goal_node: アプローチサーバー
         ├─→ nav_docking_node: ドッキングサーバー
         └─→ 状態管理 (基本)

レイヤー3: モーションプランニング (Nav2 + nav_control)
         ├─→ Nav2 グローバルプランナー (NavFn)
         ├─→ Nav2 ローカルプランナー (DWB)
         ├─→ nav_control: キネマティクス
         └─→ コストマップ管理

レイヤー4: モーション実行 (mecanum_wheels)
         ├─→ 逆運動学
         ├─→ ホイール毎のPID
         └─→ Phidget22インターフェース

レイヤー5: ハードウェア
         └─→ 4× BLDCモーター、メカナムホイール、エンコーダー
```

---

## 2. コンポーネントアーキテクチャ

### 2.1 マスター制御 (`multigo_master`)

**目的:** ハイレベルミッションオーケストレーションとユーザーインタラクション

**主要ノード:** `nav_master_node`

**責任:**
- ユーザー確認ワークフロー
- アクションクライアントのオーケストレーション (アプローチ → ドッキング)
- ハイレベルエラーハンドリング

**アクションクライアント:**
1. `/approach` - ドッキングゾーンへのナビゲーションをトリガー
2. `/dock` - 精密ドッキングシーケンスをトリガー

**現在のワークフロー:**
```cpp
// 簡略化されたフロー
1. ユーザーがアプローチをトリガー
2. nav_master: "ドッキングステーションにアプローチしますか? (y/n)"
3. ユーザー確認 → /approach アクションを呼び出し
4. アプローチ完了を待つ
5. nav_master: "ドッキングを開始しますか? (y/n)"
6. ユーザー確認 → /dock アクションを呼び出し
7. ドッキング完了を待つ
8. 成功/失敗を報告
```

**問題点:**
- ❌ 明示的な状態マシンがない (ロボット状態の追跡が困難)
- ❌ シーケンシャルのみ (複雑なフローを処理できない)
- ❌ キャンセル以外のエラーリカバリーがない

---

### 2.2 ナビゲーション (`multigo_navigation`)

#### 2.2.1 アプローチゴール計算 (`nav_goal`)

**ノード:** `nav_goal_node`

**目的:** マーカーを検出し、アプローチ位置を計算し、Nav2にゴールを送信

**ファイル:** `src/nav_goal/nav_goal.cpp`

**アルゴリズム:**
```cpp
1. /aruco_detect/markers_left をサブスクライブ
2. 目的のマーカー (ID 20) を検出
3. カメラフレームでのマーカー姿勢を取得
4. TF2を使用してマップフレームに変換
5. オフセットゴールを計算 (marker_pos + offset * marker_direction)
   - offset = 0.305m (aruco_distance_offsetで設定可能)
6. /goal_pose (PoseStamped) をパブリッシュ
7. Nav2がゴールにナビゲート
8. Nav2完了時に成功を報告
```

**精度:** ±5cm (Nav2の粗いナビゲーションには十分)

**トピック:**
- **サブスクライブ:** `/aruco_detect/markers_left` (PoseArray)
- **パブリッシュ:** `/goal_pose` (PoseStamped)

**アクション:**
- **サーバー:** `/approach` (nav_interface::action::Approach)
- **クライアント:** `/navigate_to_pose` (nav2_msgs::action::NavigateToPose)

---

#### 2.2.2 精密ドッキング (`nav_docking`)

**ノード:** `nav_docking_node`

**目的:** PID制御を使用した±1mm精度の精密ドッキングのためのビジュアルサーボイング

**ファイル:** `src/nav_docking/nav_docking.cpp`

**2段階ドッキング:**

**ステージ1: 単一前方マーカー (距離 > 0.7m)**
```cpp
void frontMarkerCmdVelPublisher() {
    // 左マーカー (ID 20) のみ使用
    double error_dist = current_distance - target_distance;
    double error_y = marker_y_position;  // センタリング
    double error_yaw = marker_yaw;

    // 各軸のPID制御
    integral_dist_ += error_dist * dt;  // 積算 (フェーズ1でバグ修正済み)
    double derivative = (error_dist - prev_error_dist_) / dt;

    double vel_x = Kp_dist * error_dist
                 + Ki_dist * integral_dist_
                 + Kd_dist * derivative;

    // YとYawについても同様
    publishVelocity(vel_x, vel_y, vel_yaw);
}
```

**ステージ2: デュアルマーカー (距離 < 0.7m)**
```cpp
void dualMarkerCmdVelPublisher() {
    // 最大精度のために両方のマーカーを使用
    double center_x = (left_marker_x + right_marker_x) / 2;
    double center_y = (left_marker_y + right_marker_y) / 2;

    // 中心からの誤差を計算
    double error_dist = center_x - target_distance_dual;
    double error_y = center_y;
    double error_yaw = calculateYawFromTwoMarkers();

    // 同じPID制御、より高精度
    // ...
}
```

**ステージ遷移:** `distance < dual_aruco_distance_th (0.7m)` かつ 両マーカーが見える時

**ドッキング確認 (2段階検証):**
```cpp
void checkDockingCompletion() {
    if (distance < aruco_close_th && position_stable) {
        if (!first_confirmation_received_) {
            first_confirmation_received_ = true;
            wait(3.0);  // 安定性チェック
        } else if (!second_confirmation_received_) {
            if (still_within_threshold) {
                second_confirmation_received_ = true;
                reportSuccess();
            }
        }
    }
}
```

**PIDパラメータ:**
```yaml
# 距離制御
Kp_dist: 0.5
Ki_dist: 0.1
Kd_dist: 0.05

# Y軸センタリング
Kp_y: 0.8
Ki_y: 0.05
Kd_y: 0.1

# 回転制御
Kp_yaw: 0.6
Ki_yaw: 0.08
Kd_yaw: 0.12
```

**閾値:**
```yaml
aruco_distance_offset: 0.305          # ステージ1の目標距離
dual_aruco_distance_th: 0.700         # ステージ2への切り替え
aruco_distance_offset_dual: 0.430     # ステージ2の目標距離
aruco_close_th: 0.42                  # 最終アプローチ閾値
```

**トピック:**
- **サブスクライブ:**
  - `/aruco_detect/markers_left` (PoseArray)
  - `/aruco_detect/markers_right` (PoseArray)
- **パブリッシュ:** `/cmd_vel_final` (Twist)

**アクション:**
- **サーバー:** `/dock` (nav_interface::action::Dock)

**既知の問題 (フェーズ1で修正済み):**
- ⚠️ **CRIT-01:** PID積分が累積されない (197行目)
- ⚠️ **CRIT-02:** デュアルマーカー距離計算のバグ (387, 503行目)
- ⚠️ **HIGH-01:** 初期化されていない変数

---

#### 2.2.3 速度制御 (`nav_control`)

**ノード:** `nav_control_node`

**目的:** 動作モード (回転中心) に基づいて速度コマンドを調整

**ファイル:** `src/nav_control/nav_control.cpp`

**モード:**
1. **SOLO** - 中心回転 (通常ナビゲーション)
2. **DOCKING** - 前方回転 (精密ドッキング)
3. **COMBINE_CHAIR** - 遠方前方回転 (車椅子接続時)

**アルゴリズム:**
```cpp
void adjustVelocityForMode(Twist& cmd_vel, NavigationMode mode) {
    double rotation_center_offset;

    switch(mode) {
        case SOLO:
            rotation_center_offset = 0.0;   // 中心周りの回転
            break;
        case DOCKING:
            rotation_center_offset = 0.25;  // 25cm前方
            break;
        case COMBINE_CHAIR:
            rotation_center_offset = 0.50;  // 50cm前方
            break;
    }

    // 希望する回転中心を達成するために並進速度を調整
    cmd_vel.linear.x += cmd_vel.angular.z * rotation_center_offset;
}
```

**トピック:**
- **サブスクライブ:**
  - `/cmd_vel` (Nav2から)
  - `/cmd_vel_final` (nav_dockingから)
  - `/navigation_mode` (モード選択)
- **パブリッシュ:** `/cmd_vel_adjusted` (Twist)

---

#### 2.2.4 ArUcoマーカー検出 (`aruco_detect`)

**ノード:** `aruco_detect_node`

**目的:** ArUcoマーカーを検出し6DOF姿勢を推定

**ファイル:** `src/aruco_detect/aruco_detect.cpp`

**アルゴリズム:**
```cpp
void detectMarkers() {
    // 1. カメラから画像をキャプチャ
    cv::Mat image = camera_image_;

    // 2. マーカーを検出 (OpenCV ArUco)
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(
        image,
        dictionary,  // DICT_6X6_250
        marker_corners,
        marker_ids
    );

    // 3. 各マーカーの姿勢を推定
    for (size_t i = 0; i < marker_ids.size(); i++) {
        cv::Vec3d rvec, tvec;
        cv::aruco::estimatePoseSingleMarkers(
            marker_corners[i],
            marker_size,      // 0.15m (物理サイズ)
            camera_matrix,    // キャリブレーションから
            dist_coeffs,      // キャリブレーションから
            rvec, tvec
        );

        // 4. OpenCVフレームをROSフレームに変換
        // OpenCV: X-右, Y-下, Z-前
        // ROS:    X-前, Y-左, Z-上
        geometry_msgs::msg::Pose marker_pose = convertToROSFrame(rvec, tvec);

        // 5. 姿勢をパブリッシュ
        publishMarkerPose(marker_ids[i], marker_pose);
    }
}
```

**マーカー設定:**
- **左マーカー:** ID 20 (前方、アプローチに使用)
- **右マーカー:** ID 21 (デュアルマーカー精度に使用)
- **マーカーサイズ:** 0.15m (15cm)
- **辞書:** DICT_6X6_250

**トピック:**
- **サブスクライブ:**
  - `/camera/color/image_raw_left` (Image)
  - `/camera/color/image_raw_right` (Image)
- **パブリッシュ:**
  - `/aruco_detect/markers_left` (PoseArray)
  - `/aruco_detect/markers_right` (PoseArray)
  - TF変換: `camera → aruco_marker_20`, `aruco_marker_21`

---

### 2.3 サードパーティ統合

#### 2.3.1 Nav2 (Navigation2 Stack)

**目的:** 業界標準の自律ナビゲーションフレームワーク

**Nav2が提供するもの:**
- グローバル経路計画 (NavFnプランナー)
- ローカル軌道計画 (DWBローカルプランナー)
- コストマップ管理 (静的、障害物、膨張レイヤー)
- リカバリー動作 (スピン、バック、待機)
- ビヘイビアツリー実行

**統合ポイント:**
```
MultiGo → Nav2:
- 入力: /goal_pose (PoseStamped)
- 入力: /map (RTAB-MapからのOccupancyGrid)
- 入力: /scan (LiDARからのLaserScan)
- 入力: /tf (変換)

Nav2 → MultiGo:
- 出力: /cmd_vel (Twist) → nav_control
- アクション: /navigate_to_pose
```

**主要設定 (`nav2_params.yaml` - 357行):**

```yaml
controller_server:
  FollowPath:
    plugin: "dwb_core::DWBLocalPlanner"
    max_vel_x: 0.26        # m/s (ゆっくり歩く速度)
    max_vel_theta: 1.0     # rad/s
    max_vel_y: 0.0         # ⚠️ 無効 (ホロノミックには0.15であるべき)
    acc_lim_x: 2.5         # m/s²
    acc_lim_theta: 3.2     # rad/s²

planner_server:
  GridBased:
    plugin: "nav2_navfn_planner::NavfnPlanner"
    tolerance: 0.5         # ゴール許容誤差 (メートル)
    use_astar: false       # ダイクストラアルゴリズム
    allow_unknown: true    # 未探索エリアを通って計画可能

global_costmap:
  robot_radius: 0.28       # メートル
  resolution: 0.05         # 5cmグリッドセル
  inflation_radius: 0.55   # セーフティマージン
  plugins:
    - "static_layer"       # マップから
    - "obstacle_layer"     # LiDARから
    - "inflation_layer"    # セーフティバッファ

local_costmap:
  width: 3.0               # 3m × 3m プランニングホライズン
  height: 3.0
  update_frequency: 5.0    # Hz
```

**重大な問題:**
- ⚠️ **CRIT-09:** ホロノミック動作が無効 (`max_vel_y = 0`)
- **影響:** メカナムホイールが十分に活用されず、ナビゲーション中に横移動できない
- **修正 (フェーズ3):** ホロノミック動作用にDWBを設定

---

#### 2.3.2 RTAB-Map (Visual SLAM)

**目的:** リアルタイムアピアランスベースマッピングと自己位置推定

**RTAB-Mapが提供するもの:**
- ビジュアルSLAM (カメラベース)
- ループクロージャ検出 (ドリフト補正)
- 3Dポイントクラウドマッピング
- 長期メモリ (セッション間で環境を記憶)
- Nav2用の2D占有グリッド

**入力:**
- RGBカメラ画像 (左 + 右)
- LiDARスキャン (`/scan`)
- ホイールオドメトリ (`/odom`)

**出力:**
- `/map` (OccupancyGrid) → Nav2
- `/tf` 変換 `map → odom` (補正された自己位置推定)
- 3Dポイントクラウドマップ (ビジュアライゼーション)

**自己位置推定戦略:**
```
距離 > 5m    : RTAB-Mapのみ (マーカーが見えない)
1m < 距離 < 5m : RTAB-Map主、ArUco利用可能
距離 < 1m    : ArUcoビジュアルサーボイングに切り替え
```

**なぜデュアル自己位置推定?**
- **RTAB-Map:** グローバル一貫性、±5-10cm精度、無制限範囲
- **ArUco:** ローカル精度、±1mm精度、限定範囲 (0.5-5m)
- **両方の長所を活用**

---

### 2.4 モーション制御

#### 2.4.1 メカナムホイール (`mecanum_wheels`)

**ノード:** `mecanum_wheels_node`

**目的:** Twistコマンドをホイール速度に変換し、モーターを制御

**アルゴリズム:**
```cpp
// メカナムホイールの逆運動学
void twistToWheelVelocities(Twist cmd_vel) {
    double vx = cmd_vel.linear.x;
    double vy = cmd_vel.linear.y;
    double omega = cmd_vel.angular.z;

    // メカナムホイールのキネマティクス
    double wheel_fl = (1/r) * (vx - vy - (lx + ly) * omega);
    double wheel_fr = (1/r) * (vx + vy + (lx + ly) * omega);
    double wheel_rl = (1/r) * (vx + vy - (lx + ly) * omega);
    double wheel_rr = (1/r) * (vx - vy + (lx + ly) * omega);

    // ホイール毎のPID制御
    for (int i = 0; i < 4; i++) {
        motor_command[i] = wheelPID(target[i], current[i]);
    }

    // Phidgetモーターコントローラーに送信
    sendMotorCommands(motor_command);
}
```

**パラメータ:**
```yaml
WHEEL_BASE_LENGTH: 0.40m   # 前後距離
WHEEL_BASE_WIDTH: 0.30m    # 左右距離
WHEEL_DIAMETER: 0.0762m    # 3インチ
```

**トピック:**
- **サブスクライブ:** `/cmd_vel_adjusted` (Twist)
- **パブリッシュ:** `/odom` (Odometry)

---

## 3. 通信パターン

### 3.1 トピックマップ

| トピック | タイプ | パブリッシャー | サブスクライバー | 目的 |
|-------|------|-----------|---------------|---------|
| `/camera/color/image_raw_left` | Image | camera_driver | aruco_detect | 左カメラフィード |
| `/camera/color/image_raw_right` | Image | camera_driver | aruco_detect | 右カメラフィード |
| `/scan` | LaserScan | lidar_driver | Nav2, RTAB-Map | LiDARデータ |
| `/aruco_detect/markers_left` | PoseArray | aruco_detect | nav_goal, nav_docking | 検出されたマーカー |
| `/aruco_detect/markers_right` | PoseArray | aruco_detect | nav_docking | 検出されたマーカー |
| `/goal_pose` | PoseStamped | nav_goal | Nav2 | ナビゲーションゴール |
| `/cmd_vel` | Twist | Nav2 | nav_control | ナビゲーションコマンド |
| `/cmd_vel_final` | Twist | nav_docking | nav_control | ドッキングコマンド |
| `/cmd_vel_adjusted` | Twist | nav_control | mecanum_wheels | 最終コマンド |
| `/odom` | Odometry | mecanum_wheels | Nav2, RTAB-Map | ホイールオドメトリ |
| `/map` | OccupancyGrid | RTAB-Map | Nav2 | 2Dグリッドマップ |

### 3.2 アクションマップ

| アクション | タイプ | サーバー | クライアント | 目的 |
|--------|------|--------|--------|---------|
| `/approach` | Approach | nav_goal | nav_master | ドッキングゾーンへナビゲート |
| `/dock` | Dock | nav_docking | nav_master | 精密ドッキング |
| `/navigate_to_pose` | NavigateToPose | Nav2 bt_navigator | nav_goal | Nav2ナビゲーション |

### 3.3 TFツリー

```
map (RTAB-Map)
 └─→ odom (ホイールオドメトリ)
      └─→ base_link (ロボット中心)
           ├─→ base_footprint (地面投影)
           ├─→ camera_left_link
           │    └─→ aruco_marker_20
           ├─→ camera_right_link
           │    └─→ aruco_marker_21
           └─→ laser_link (LiDAR)
```

---

## 4. 主要アルゴリズム

### 4.1 PID制御 (ドッキング)

**目的:** 目標位置への滑らかで安定したアプローチ

**アルゴリズム:**
```cpp
double pidControl(double error, double Kp, double Ki, double Kd, double dt) {
    // 比例: 現在の誤差に反応
    double P = Kp * error;

    // 積分: 定常偏差を排除
    integral_ += error * dt;  // ⚠️ 累積必須! (CRIT-01)
    double I = Ki * integral_;

    // 微分: 振動を減衰
    double derivative = (error - prev_error_) / dt;
    double D = Kd * derivative;

    prev_error_ = error;

    return P + I + D;
}
```

**チューニング:**
- **Kp** (比例): 高い = 速い応答、オーバーシュートのリスク
- **Ki** (積分): 定常偏差を排除、ワインドアップのリスク
- **Kd** (微分): 振動を減衰、ノイズに敏感

**現在のチューニング (良好に動作):**
- 距離: Kp=0.5, Ki=0.1, Kd=0.05
- センタリング: Kp=0.8, Ki=0.05, Kd=0.1
- 回転: Kp=0.6, Ki=0.08, Kd=0.12

---

### 4.2 デュアルマーカーセンタリング

**目的:** 2つのマーカー間の中心線に対するロボット位置を計算

**アルゴリズム:**
```cpp
void calculateCenterPosition() {
    // 両方のマーカー位置を取得
    double left_x = left_marker.pose.position.x;
    double right_x = right_marker.pose.position.x;

    // 中心を計算 (平均)
    double center_x = (left_x + right_x) / 2;  // ⚠️ 括弧が重要! (CRIT-02)

    // YとYawについても同様
    double center_y = (left_y + right_y) / 2;
    double center_yaw = (left_yaw + right_yaw) / 2;

    // 誤差を計算
    double error_dist = center_x - target_distance;
    double error_y = center_y;  // 0であるべき (センタリング)
    double error_yaw = center_yaw;  // 0であるべき (整列)
}
```

**重大なバグ (CRIT-02):**
```cpp
// 誤り (演算子の優先順位の問題):
double center = (left_x) + (right_x) / 2;  // = left + (right/2)

// 正解:
double center = (left_x + right_x) / 2;    // = (left + right) / 2
```

**影響:** 修正なしでは、ロボットは約5-10cm中心からずれてドッキングする

---

## 5. データフロー

### 5.1 完全なドッキングワークフロー

```
[ユーザーコマンド]
    │
    ↓
┌───────────────────────────────────────┐
│ nav_master: "アプローチ? (y/n)"         │
└───────────────────────────────────────┘
    │ (ユーザー確認: y)
    ↓
┌───────────────────────────────────────┐
│ ステージ1: アプローチ (nav_goal)        │
│                                       │
│ 1. ArUcoマーカー (ID 20) を検出        │
│ 2. マップフレームに変換                │
│ 3. オフセットゴールを計算 (0.305m)     │
│ 4. /goal_pose をパブリッシュ           │
│ 5. Nav2がナビゲート (~30s)            │
│ 6. ロボットがマーカーから~30cm停止     │
│                                       │
│ 精度: ±5cm                            │
└───────────────────────────────────────┘
    │ (アプローチ完了)
    ↓
┌───────────────────────────────────────┐
│ nav_master: "ドッキング開始? (y/n)"     │
└───────────────────────────────────────┘
    │ (ユーザー確認: y)
    ↓
┌───────────────────────────────────────┐
│ ステージ2: アライメント (nav_docking)   │
│ 単一マーカーフェーズ                   │
│                                       │
│ 1. 前方マーカー (ID 20) を検出         │
│ 2. PID制御: X, Y, Yaw                 │
│ 3. 距離 < 0.7m までアプローチ          │
│ 4. 両マーカーが見える                  │
│                                       │
│ 精度: ±1cm                            │
│ 所要時間: ~15s                        │
└───────────────────────────────────────┘
    │ (両マーカーが見える)
    ↓
┌───────────────────────────────────────┐
│ ステージ3: 精密 (nav_docking)          │
│ デュアルマーカーフェーズ               │
│                                       │
│ 1. 両マーカー (ID 20, 21) を検出       │
│ 2. 中心位置を計算                      │
│ 3. センタリングのためのPID制御         │
│ 4. 距離 < 0.42m までアプローチ         │
│                                       │
│ 精度: ±1mm (目標)                     │
│ 所要時間: ~10s                        │
└───────────────────────────────────────┘
    │ (距離 < 0.42m)
    ↓
┌───────────────────────────────────────┐
│ ステージ4: 検証                        │
│                                       │
│ 1. 位置が安定しているか確認 (3秒)      │
│ 2. 最初の確認チェック                  │
│ 3. さらに3秒待機                       │
│ 4. 2回目の確認チェック                 │
│ 5. 成功を報告                          │
└───────────────────────────────────────┘
    │
    ↓
[ドッキング完了!]
```

**総所要時間:** ~60-70秒 (30sアプローチ + 15sアライメント + 15s精密 + 6s検証)

---

# パート2: 提案されたアーキテクチャ (フェーズ1-4)

## 6. 提案されたセーフティアーキテクチャ(フェーズ 1)

### 6.1 問題の説明

**現在の問題:**
- ❌ ドッキング中のLiDAR監視なし (障害物に対して盲目)
- ❌ 緊急停止メカニズムなし
- ❌ 上書き権限を持つセーフティレイヤーなし
- ❌ セーフティロジックがノード間に散在

**結果:** システムは本番展開に**安全ではない**

### 6.2 提案されたセーフティレイヤー

**設計原則:** すべての動作に対する上書き権限を持つ直交セーフティレイヤー

```
┌─────────────────────────────────────────────────────────┐
│           SAFETY SUPERVISOR (上書き権限)                │
│                                                         │
│  状態:                                                  │
│  ┌──────┐ ┌────────┐ ┌────────┐ ┌──────────────────┐  │
│  │ SAFE │→│CAUTION │→│ UNSAFE │→│ EMERGENCY_STOP   │  │
│  └──────┘ └────────┘ └────────┘ └──────────────────┘  │
│                                                         │
│  監視:                                                  │
│  • LiDARスキャン (障害物距離)                           │
│  • ArUcoマーカー (可視性)                               │
│  • ロボット状態 (位置、速度)                            │
│  • システムヘルス (ノードステータス)                     │
│  • 手動非常停止ボタン                                   │
│                                                         │
│  実行:                                                  │
│  • /multigo/safety/emergency_stop (Bool)                │
│  • /multigo/safety/speed_limit (Float32)                │
│  • /multigo/safety/state (String)                       │
└─────────────────────────────────────────────────────────┘
                         │
        ┌────────────────┼────────────────┐
        ↓                ↓                ↓
  [nav_docking]    [nav_control]   [mecanum_wheels]
     (チェック)       (チェック)       (チェック)
```

**実装:**

```cpp
class SafetySupervisor : public rclcpp::Node {
public:
    enum class SafetyState {
        SAFE,           // すべてクリア
        CAUTION,        // 警告、速度低減
        UNSAFE,         // 危険、動作停止
        EMERGENCY_STOP  // 手動非常停止有効
    };

private:
    SafetyState current_state_ = SafetyState::SAFE;

    // モニター
    rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<Bool>::SharedPtr estop_button_sub_;
    rclcpp::Subscription<PoseArray>::SharedPtr markers_sub_;

    // 実行者
    rclcpp::Publisher<Bool>::SharedPtr safety_stop_pub_;
    rclcpp::Publisher<Float32>::SharedPtr speed_limit_pub_;
    rclcpp::Publisher<String>::SharedPtr state_pub_;

public:
    void scanCallback(LaserScan::SharedPtr msg) {
        double min_dist = *std::min_element(msg->ranges.begin(), msg->ranges.end());

        if (min_dist < 0.15) {  // 15cm = 危険
            transitionTo(SafetyState::EMERGENCY_STOP);
        } else if (min_dist < 0.30) {  // 30cm = 注意
            transitionTo(SafetyState::CAUTION);
        } else if (min_dist < 0.50) {  // 50cm = 安全だが監視
            transitionTo(SafetyState::SAFE);
        }
    }

    void transitionTo(SafetyState new_state) {
        if (new_state == current_state_) return;

        RCLCPP_INFO(get_logger(), "Safety: %s -> %s",
                    stateToString(current_state_),
                    stateToString(new_state));

        current_state_ = new_state;

        switch (new_state) {
            case SafetyState::SAFE:
                publishSpeedLimit(1.0);      // 100% 速度
                publishSafetyStop(false);
                break;

            case SafetyState::CAUTION:
                publishSpeedLimit(0.5);      // 50% 速度
                RCLCPP_WARN(get_logger(), "CAUTION: 障害物を検出、速度を低減");
                break;

            case SafetyState::UNSAFE:
                publishSpeedLimit(0.2);      // 20% 速度 (徐行)
                RCLCPP_WARN(get_logger(), "UNSAFE: 障害物が非常に近い");
                break;

            case SafetyState::EMERGENCY_STOP:
                publishSpeedLimit(0.0);      // 停止
                publishSafetyStop(true);
                RCLCPP_ERROR(get_logger(), "緊急停止が有効化されました!");
                break;
        }

        publishState(stateToString(new_state));
    }
};
```

**モーションノードとの統合:**

```cpp
// nav_docking、nav_control、mecanum_wheelsで:
class NavDockingNode : public rclcpp::Node {
private:
    rclcpp::Subscription<Bool>::SharedPtr estop_sub_;
    rclcpp::Subscription<Float32>::SharedPtr speed_limit_sub_;

    bool emergency_stop_active_ = false;
    double speed_limit_multiplier_ = 1.0;

public:
    void estopCallback(Bool::SharedPtr msg) {
        emergency_stop_active_ = msg->data;
        if (emergency_stop_active_) {
            RCLCPP_ERROR(get_logger(), "非常停止! すべての動作を停止");
            publishZeroVelocity();
            cancelCurrentAction();
        }
    }

    void speedLimitCallback(Float32::SharedPtr msg) {
        speed_limit_multiplier_ = msg->data;
    }

    void publishVelocity(Twist cmd) {
        // セーフティチェック1: 非常停止が有効か?
        if (emergency_stop_active_) {
            RCLCPP_WARN(get_logger(), "非常停止により動作がブロックされました");
            return;  // パブリッシュしない
        }

        // セーフティチェック2: 速度制限を適用
        cmd.linear.x *= speed_limit_multiplier_;
        cmd.linear.y *= speed_limit_multiplier_;
        cmd.angular.z *= speed_limit_multiplier_;

        cmd_vel_pub_->publish(cmd);
    }
};
```

**セーフティトピック:**
- `/multigo/safety/emergency_stop` (Bool) - RELIABLE, TRANSIENT_LOCAL, KEEP_ALL
- `/multigo/safety/speed_limit` (Float32) - RELIABLE
- `/multigo/safety/state` (String) - 監視用

**テスト:**
```bash
# 非常停止をテスト
ros2 topic pub /multigo/safety/emergency_stop std_msgs/Bool "data: true"
# 確認: ロボットが即座に停止 (<100ms)

# 速度制限をテスト
ros2 topic pub /multigo/safety/speed_limit std_msgs/Float32 "data: 0.3"
# 確認: ロボットが30%速度に減速

# 障害物検出をテスト
# ドッキング中にロボットから20cmの場所に障害物を配置
# 確認: ロボットがEMERGENCY_STOPに遷移して停止
```

**工数:** 40時間 (設計 8h + 実装 24h + テスト 8h)

**優先度:** 🔴 **重要** - これなしでは展開できない

---

### 6.3 ジオフェンシング (立入禁止ゾーン)

**目的:** ロボットが越えられない仮想境界を定義

**設定:** `config/safety_zones.yaml`

```yaml
keep_out_zones:
  - id: "elevator_shaft"
    polygon: [[5.0, 10.0], [6.0, 10.0], [6.0, 11.0], [5.0, 11.0]]
    reason: "危険: エレベーターシャフト"

  - id: "stairs_north"
    polygon: [[2.0, 8.0], [3.0, 8.0], [3.0, 9.0], [2.0, 9.0]]
    reason: "崖の危険: 階段"

allowed_zones:
  - id: "hospital_floor_2"
    polygon: [[0, 0], [50, 0], [50, 30], [0, 30]]
    reason: "承認された稼働エリア"
```

**実装:**

```cpp
class SafetyZoneManager : public rclcpp::Node {
private:
    std::vector<Polygon> keep_out_zones_;
    std::vector<Polygon> allowed_zones_;

public:
    void loadZones(std::string yaml_file) {
        // YAMLを解析してポリゴンを読み込み
        YAML::Node config = YAML::LoadFile(yaml_file);
        for (auto zone : config["keep_out_zones"]) {
            keep_out_zones_.push_back(parsePolygon(zone));
        }
    }

    bool isPositionSafe(double x, double y) {
        // チェック1: 立入禁止ゾーン内にないか
        for (auto& zone : keep_out_zones_) {
            if (pointInPolygon(x, y, zone)) {
                RCLCPP_ERROR(get_logger(), "位置 (%.2f, %.2f) は立入禁止ゾーン内: %s",
                            x, y, zone.reason.c_str());
                return false;
            }
        }

        // チェック2: 少なくとも1つの許可ゾーン内にいるか
        bool in_allowed = false;
        for (auto& zone : allowed_zones_) {
            if (pointInPolygon(x, y, zone)) {
                in_allowed = true;
                break;
            }
        }

        if (!in_allowed) {
            RCLCPP_ERROR(get_logger(), "位置 (%.2f, %.2f) が許可ゾーンの外", x, y);
        }

        return in_allowed;
    }

private:
    bool pointInPolygon(double x, double y, const Polygon& poly) {
        // 点がポリゴン内にあるかのレイキャスティングアルゴリズム
        // 標準的な計算幾何学アルゴリズム
        // ...
    }
};
```

**統合:**
- セーフティスーパーバイザーが100msごとにロボット位置をチェック
- 違反が検出された場合 → EMERGENCY_STOPに遷移
- 立入禁止ゾーン内のナビゲーションゴールは拒否
- 経路計画は許可ゾーンに制約

**工数:** 16時間 (基本)、40時間 (ビジュアライゼーション付き完全版)

---

## 7. 提案された状態管理(フェーズ 1)

### 7.1 問題の説明

**現在の問題:**
- ❌ 明示的な状態追跡がない ("ロボットはどの状態にあるか?")
- ❌ デバッグが困難 ("どこで失敗したか?")
- ❌ 拡張が困難 (新しいステップの追加にはコード変更が必要)
- ❌ 動作のビジュアライゼーションがない

### 7.2 提案された状態マシン

**実装:**

```cpp
enum class MissionState {
    IDLE,
    WAITING_APPROACH_CONFIRMATION,
    NAVIGATING_TO_GOAL,
    WAITING_DOCK_CONFIRMATION,
    DOCKING,
    DOCKED,
    UNDOCKING,          // 将来
    TRANSPORTING,       // 将来
    ERROR
};

class MissionStateMachine : public rclcpp::Node {
private:
    MissionState current_state_ = MissionState::IDLE;

    // 状態変更パブリッシャー
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

    // アクションクライアント
    rclcpp_action::Client<Approach>::SharedPtr approach_client_;
    rclcpp_action::Client<Dock>::SharedPtr dock_client_;

public:
    MissionStateMachine() : Node("mission_state_machine") {
        // アクションクライアントをセットアップ
        approach_client_ = rclcpp_action::create_client<Approach>(this, "approach");
        dock_client_ = rclcpp_action::create_client<Dock>(this, "dock");

        // 状態パブリッシャーをセットアップ
        state_pub_ = create_publisher<std_msgs::msg::String>("/multigo/mission/state", 10);

        // メイン状態マシンループ (10 Hz)
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MissionStateMachine::update, this)
        );
    }

    void update() {
        switch (current_state_) {
            case MissionState::IDLE:
                // ユーザーがミッションを要求するのを待つ
                if (approach_requested_) {
                    transitionTo(MissionState::WAITING_APPROACH_CONFIRMATION);
                }
                break;

            case MissionState::WAITING_APPROACH_CONFIRMATION:
                // 確認プロンプトを表示
                displayPrompt("ドッキングステーションにアプローチしますか? (y/n)");

                if (user_confirmed_) {
                    transitionTo(MissionState::NAVIGATING_TO_GOAL);
                    sendApproachGoal();
                } else if (user_cancelled_) {
                    transitionTo(MissionState::IDLE);
                }
                break;

            case MissionState::NAVIGATING_TO_GOAL:
                // アプローチアクションのステータスをチェック
                if (approach_succeeded_) {
                    transitionTo(MissionState::WAITING_DOCK_CONFIRMATION);
                } else if (approach_failed_) {
                    transitionTo(MissionState::ERROR);
                    logError("アプローチ失敗: " + approach_error_message_);
                } else if (user_cancelled_) {
                    cancelApproachGoal();
                    transitionTo(MissionState::IDLE);
                }
                break;

            case MissionState::WAITING_DOCK_CONFIRMATION:
                displayPrompt("精密ドッキングを開始しますか? (y/n)");

                if (user_confirmed_) {
                    transitionTo(MissionState::DOCKING);
                    sendDockGoal();
                } else if (user_cancelled_) {
                    transitionTo(MissionState::IDLE);
                }
                break;

            case MissionState::DOCKING:
                // ドッキングアクションのステータスをチェック
                if (dock_succeeded_) {
                    transitionTo(MissionState::DOCKED);
                    displaySuccess("ドッキング完了!");
                } else if (dock_failed_) {
                    transitionTo(MissionState::ERROR);
                    logError("ドッキング失敗: " + dock_error_message_);
                } else if (user_cancelled_) {
                    cancelDockGoal();
                    transitionTo(MissionState::IDLE);
                }
                break;

            case MissionState::DOCKED:
                // 次のコマンドを待つ (アンドック、輸送など)
                if (undock_requested_) {
                    transitionTo(MissionState::UNDOCKING);  // 将来
                }
                break;

            case MissionState::ERROR:
                // エラーを表示、ユーザーの確認を待つ
                if (user_acknowledged_error_) {
                    transitionTo(MissionState::IDLE);
                }
                break;
        }
    }

    void transitionTo(MissionState new_state) {
        if (new_state == current_state_) return;

        RCLCPP_INFO(get_logger(), "状態遷移: %s -> %s",
                    stateToString(current_state_),
                    stateToString(new_state));

        // 旧状態の終了アクション
        onStateExit(current_state_);

        // 状態を更新
        current_state_ = new_state;

        // 新状態の開始アクション
        onStateEntry(new_state);

        // 監視用にパブリッシュ
        std_msgs::msg::String state_msg;
        state_msg.data = stateToString(new_state);
        state_pub_->publish(state_msg);
    }

    void onStateEntry(MissionState state) {
        // 状態に入る際の開始アクション
        switch (state) {
            case MissionState::NAVIGATING_TO_GOAL:
                RCLCPP_INFO(get_logger(), "ゴールへのナビゲーションを開始...");
                break;
            case MissionState::DOCKING:
                RCLCPP_INFO(get_logger(), "精密ドッキングを開始...");
                break;
            case MissionState::DOCKED:
                RCLCPP_INFO(get_logger(), "ドッキング完了、ロボットはドッキング済み");
                break;
            case MissionState::ERROR:
                RCLCPP_ERROR(get_logger(), "ERROR状態に入りました");
                break;
            default:
                break;
        }
    }

    void onStateExit(MissionState state) {
        // 状態を離れる際のクリーンアップアクション
        switch (state) {
            case MissionState::NAVIGATING_TO_GOAL:
                RCLCPP_INFO(get_logger(), "ナビゲーション完了");
                break;
            case MissionState::DOCKING:
                RCLCPP_INFO(get_logger(), "ドッキングシーケンス完了");
                break;
            default:
                break;
        }
    }

private:
    std::string stateToString(MissionState state) {
        switch (state) {
            case MissionState::IDLE: return "IDLE";
            case MissionState::WAITING_APPROACH_CONFIRMATION: return "WAITING_APPROACH_CONFIRMATION";
            case MissionState::NAVIGATING_TO_GOAL: return "NAVIGATING_TO_GOAL";
            case MissionState::WAITING_DOCK_CONFIRMATION: return "WAITING_DOCK_CONFIRMATION";
            case MissionState::DOCKING: return "DOCKING";
            case MissionState::DOCKED: return "DOCKED";
            case MissionState::UNDOCKING: return "UNDOCKING";
            case MissionState::TRANSPORTING: return "TRANSPORTING";
            case MissionState::ERROR: return "ERROR";
            default: return "UNKNOWN";
        }
    }
};
```

**メリット:**
- ✅ ロボット状態を常に把握: `ros2 topic echo /multigo/mission/state`
- ✅ デバッグが容易: "ロボットがDOCKING状態でスタック"
- ✅ より良いエラーハンドリング: 明確なエラー状態 + リカバリー
- ✅ 拡張可能: 新しい状態を追加 (UNDOCKING、CHARGING など)
- ✅ テスト可能: 状態遷移のユニットテスト

**監視:**
```bash
# 状態変化をリアルタイムで監視
ros2 topic echo /multigo/mission/state

# テスト用に状態遷移をトリガー
ros2 service call /mission/trigger_approach std_srvs/Trigger
```

**工数:** 24時間 (実装 + 統合 + テスト)

**優先度:** 🔴 **高** - デバッグ性の大幅な改善

---

## 8. 提案されたROS 2改善(フェーズ 3)

### 8.1 ライフサイクルノード

**現在の問題:** ノードが即座に起動、クリーンな起動/シャットダウンがない

**提案:** 重要なノードをライフサイクルノードに変換

**ライフサイクル状態:**
```
┌──────────────┐
│ Unconfigured │  (初期状態)
└──────┬───────┘
       │ configure()
       ↓
┌──────────────┐
│  Inactive    │  (設定読み込み済み、パブリッシュしていない)
└──────┬───────┘
       │ activate()
       ↓
┌──────────────┐
│   Active     │  (完全に動作中)
└──────┬───────┘
       │ deactivate()
       ↓
┌──────────────┐
│  Inactive    │  (素早く再有効化可能)
└──────┬───────┘
       │ cleanup()
       ↓
┌──────────────┐
│ Unconfigured │  (クリーンなシャットダウン)
└──────────────┘
```

**実装例 (ライフサイクルノードとしてのnav_docking):**

```cpp
#include <rclcpp_lifecycle/lifecycle_node.hpp>

class NavDockingLifecycle : public rclcpp_lifecycle::LifecycleNode {
public:
    NavDockingLifecycle() : LifecycleNode("nav_docking_node") {
        RCLCPP_INFO(get_logger(), "NavDocking ライフサイクルノードが作成されました");
    }

    // CONFIGURE: パラメータ読み込み、設定検証
    CallbackReturn on_configure(const State&) override {
        RCLCPP_INFO(get_logger(), "NavDockingノードを設定中...");

        // パラメータを読み込んで検証
        declare_parameter<double>("Kp_dist", 0.5);
        declare_parameter<double>("Ki_dist", 0.1);
        declare_parameter<double>("Kd_dist", 0.05);

        Kp_dist_ = get_parameter("Kp_dist").as_double();
        Ki_dist_ = get_parameter("Ki_dist").as_double();
        Kd_dist_ = get_parameter("Kd_dist").as_double();

        // 検証
        if (Kp_dist_ <= 0 || Ki_dist_ < 0 || Kd_dist_ < 0) {
            RCLCPP_ERROR(get_logger(), "無効なPIDパラメータ!");
            return CallbackReturn::FAILURE;
        }

        // パブリッシャー/サブスクライバーを作成 (まだ有効化しない)
        cmd_vel_pub_ = create_publisher<Twist>("/cmd_vel_final", 10);
        markers_sub_ = create_subscription<PoseArray>(
            "/aruco_detect/markers_left", 10,
            std::bind(&NavDockingLifecycle::markersCallback, this, std::placeholders::_1)
        );

        // アクションサーバーを作成 (まだ有効化しない)
        dock_action_server_ = rclcpp_action::create_server<Dock>(
            this, "dock",
            std::bind(&NavDockingLifecycle::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&NavDockingLifecycle::handleCancel, this, std::placeholders::_1),
            std::bind(&NavDockingLifecycle::handleAccepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(), "NavDockingが正常に設定されました");
        return CallbackReturn::SUCCESS;
    }

    // ACTIVATE: パブリッシュ開始、アクションサーバー有効化
    CallbackReturn on_activate(const State&) override {
        RCLCPP_INFO(get_logger(), "NavDockingノードを有効化中...");

        // パブリッシャーを有効化 (ライフサイクルパブリッシャーは有効時のみパブリッシュ)
        cmd_vel_pub_->on_activate();

        // 制御タイマー開始
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&NavDockingLifecycle::controlLoop, this)
        );

        RCLCPP_INFO(get_logger(), "NavDockingが有効、ドッキング準備完了");
        return CallbackReturn::SUCCESS;
    }

    // DEACTIVATE: パブリッシュ停止、アクションサーバー無効化
    CallbackReturn on_deactivate(const State&) override {
        RCLCPP_INFO(get_logger(), "NavDockingノードを無効化中...");

        // 制御ループ停止
        control_timer_->cancel();

        // コマンドのパブリッシュを停止 (安全!)
        cmd_vel_pub_->on_deactivate();
        publishZeroVelocity();

        // アクティブなドッキングアクションをキャンセル
        if (current_goal_handle_) {
            current_goal_handle_->abort();
        }

        RCLCPP_INFO(get_logger(), "NavDockingが無効化されました");
        return CallbackReturn::SUCCESS;
    }

    // CLEANUP: リソースを解放
    CallbackReturn on_cleanup(const State&) override {
        RCLCPP_INFO(get_logger(), "NavDockingノードをクリーンアップ中...");

        // 状態をリセット
        integral_dist_ = 0.0;
        integral_y_ = 0.0;
        integral_yaw_ = 0.0;

        // パブリッシャー/サブスクライバーをクリア
        cmd_vel_pub_.reset();
        markers_sub_.reset();
        dock_action_server_.reset();

        RCLCPP_INFO(get_logger(), "NavDockingがクリーンアップされました");
        return CallbackReturn::SUCCESS;
    }

    // SHUTDOWN: 終了前の最終クリーンアップ
    CallbackReturn on_shutdown(const State&) override {
        RCLCPP_INFO(get_logger(), "NavDockingノードをシャットダウン中...");

        // ロボットが停止していることを確認
        publishZeroVelocity();

        return CallbackReturn::SUCCESS;
    }
};
```

**ライフサイクル管理:**

```bash
# CLIでノードのライフサイクルを管理
ros2 lifecycle set /nav_docking_node configure
ros2 lifecycle set /nav_docking_node activate
ros2 lifecycle set /nav_docking_node deactivate
ros2 lifecycle set /nav_docking_node cleanup
```

**自動ライフサイクル管理:**

```yaml
# lifecycle_manager 設定
lifecycle_manager:
  ros__parameters:
    node_names:
      - nav_docking_node
      - nav_goal_node
      - nav_control_node
    autostart: true
    bond_timeout: 4.0
```

**変換するノード:**
1. nav_docking (12時間)
2. nav_control (12時間)
3. mecanum_wheels (12時間)
4. aruco_detect (12時間)

**総工数:** 48時間

**メリット:**
- ✅ クリーンな起動シーケンス (設定 → 有効化)
- ✅ 安全なシャットダウン (無効化 → クリーンアップ)
- ✅ システムを再起動せずにノードを再起動可能
- ✅ 標準化されたノード管理
- ✅ テストに適している (ユニットテスト用に無効化可能)

---

### 8.2 QoSポリシー (Quality of Service)

**現在の問題:** すべてのトピックがデフォルトQoSを使用 (メッセージ損失の可能性)

**提案:** メッセージの重要度に基づく明示的なQoSポリシー

**QoS設定:**

```cpp
// CRITICAL: 緊急停止、セーフティ信号
// これらのメッセージを決して失ってはならない
auto qos_critical = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)       // 配信保証
    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)  // 遅れて参加した者が最後のメッセージを取得
    .history(RMW_QOS_POLICY_HISTORY_KEEP_ALL);              // すべてのメッセージを保持

// CONTROL: 速度コマンド、ドッキング制御
// 最新値が必要、古いコマンドは陳腐化
auto qos_control = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)       // 配信保証
    .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1);          // 最新のみが重要

// SENSOR: カメラ画像、LiDARスキャン
// 高頻度、古いデータのドロップは許容
auto qos_sensor = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)    // 保証なし (高速!)
    .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1);          // 最新のみが重要

// STATUS: 診断、監視
// 遅れて参加した者のために履歴が必要
auto qos_status = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
    .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10);
```

**適用:**

```cpp
// 緊急停止パブリッシャー (CRITICAL QoS)
auto qos_critical = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
    .history(RMW_QOS_POLICY_HISTORY_KEEP_ALL);

estop_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/multigo/safety/emergency_stop",
    qos_critical  // ← 明示的なQoS
);

// カメラサブスクライバー (SENSOR QoS)
auto qos_sensor = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/camera/color/image_raw_left",
    qos_sensor,  // ← 明示的なQoS
    callback
);
```

**トピックQoS推奨事項:**

| トピック | QoSタイプ | 理由 |
|-------|----------|-----------|
| `/multigo/safety/emergency_stop` | CRITICAL | 非常停止信号を決して失ってはならない |
| `/multigo/safety/speed_limit` | CRITICAL | セーフティクリティカル |
| `/cmd_vel*` | CONTROL | 最新コマンドが必要、古いものは陳腐化 |
| `/camera/color/*` | SENSOR | 高頻度、ベストエフォートでOK |
| `/scan` | SENSOR | 高頻度、ベストエフォートでOK |
| `/aruco_detect/markers_*` | CONTROL | 信頼性のあるマーカー検出が必要 |
| `/goal_pose` | CONTROL | ナビゲーションゴールは到着必須 |
| `/multigo/mission/state` | STATUS | 監視用に履歴が必要 |

**工数:** 8時間 (すべてのトピックにQoSを適用 + テスト)

**メリット:**
- ✅ 非常停止信号の損失なし
- ✅ より良いリアルタイムパフォーマンス (センサー用BEST_EFFORT)
- ✅ 遅れて参加した者が重要な状態を取得 (TRANSIENT_LOCAL)
- ✅ 明示的な通信要件

---

### 8.3 強化されたアクション定義

**現在の問題:** アクション結果に `success: bool` のみ

**提案:** リッチなフィードバックと詳細な結果

**強化されたDock.action:**

```
# Goal
geometry_msgs/PoseStamped target_pose  # ドッキング先
float32 tolerance                      # 位置許容誤差 (デフォルト 0.002m)
---
# Result
bool success                           # 全体的な成功
string error_code                      # 失敗の場合: "MARKERS_LOST", "TIMEOUT", "OBSTACLE" など
string detailed_message                # 人間が読めるエラー説明

# 最終状態
geometry_msgs/Pose final_pose          # 実際の最終位置
float32 final_distance_error_x         # Xの誤差 (メートル)
float32 final_distance_error_y         # Yの誤差 (メートル)
float32 final_yaw_error                # Yawの誤差 (ラジアン)

# パフォーマンスメトリクス
float32 duration_seconds               # ドッキングにかかった時間
int32 attempts                         # 再試行回数
---
# Feedback (実行中にパブリッシュ)
string current_phase                   # "APPROACH", "ALIGN_SINGLE", "ALIGN_DUAL", "VERIFY"
float32 distance_to_target             # 現在の距離 (メートル)
float32 centering_error                # 横方向誤差 (メートル)
float32 yaw_error                      # 向き誤差 (ラジアン)
bool markers_visible                   # マーカーが検出されているか?
int32 marker_count                     # 見えるマーカー数 (1 または 2)
float32 progress_percentage            # 0-100%
```

**使用法:**

```cpp
// クライアント側 (nav_master)
void dockingFeedbackCallback(const Dock::Feedback::ConstSharedPtr feedback) {
    RCLCPP_INFO(get_logger(), "ドッキング進捗: %d%% | フェーズ: %s | 距離: %.3fm",
                (int)feedback->progress_percentage,
                feedback->current_phase.c_str(),
                feedback->distance_to_target);

    // ユーザーに表示
    displayProgress(feedback->progress_percentage, feedback->current_phase);
}

void dockingResultCallback(const Dock::Result::ConstSharedPtr result) {
    if (result->success) {
        RCLCPP_INFO(get_logger(),
                    "ドッキング成功!\n"
                    "  最終誤差: X=%.1fmm, Y=%.1fmm, Yaw=%.1f°\n"
                    "  所要時間: %.1fs, 試行回数: %d",
                    result->final_distance_error_x * 1000,
                    result->final_distance_error_y * 1000,
                    result->final_yaw_error * 180 / M_PI,
                    result->duration_seconds,
                    result->attempts);
    } else {
        RCLCPP_ERROR(get_logger(),
                     "ドッキング失敗: %s\n"
                     "  エラーコード: %s\n"
                     "  失敗までの時間: %.1fs",
                     result->detailed_message.c_str(),
                     result->error_code.c_str(),
                     result->duration_seconds);

        // 特定のエラーを処理
        if (result->error_code == "MARKERS_LOST") {
            // マーカー配置のチェックを提案
        } else if (result->error_code == "TIMEOUT") {
            // PIDゲインの低減またはタイムアウトの増加を提案
        }
    }
}
```

**エラーコード:**
```cpp
enum class DockingErrorCode {
    SUCCESS = 0,
    MARKERS_LOST,           // ドッキング中にマーカーが消えた
    TIMEOUT,                // 時間がかかりすぎた (>120s)
    OBSTACLE_DETECTED,      // セーフティシステムがドッキングを停止
    POSITION_UNSTABLE,      // 安定した位置を達成できなかった
    TOLERANCE_NOT_MET,      // 最終位置が許容誤差外
    ACTION_CANCELLED,       // ユーザーがキャンセル
    INTERNAL_ERROR          // 予期しないエラー
};
```

**工数:** 12時間 (アクション再設計 + 実装 + クライアント更新)

**メリット:**
- ✅ より良いエラーメッセージ ("マーカー消失" vs "失敗")
- ✅ 進捗のビジュアライゼーション (0-100%)
- ✅ パフォーマンスメトリクス (所要時間、精度)
- ✅ デバッグが容易 (正確な失敗モードがわかる)
- ✅ より良いユーザーエクスペリエンス

---

### 8.4 トピック命名の標準化

**現在の問題:** 一貫性のない命名 (`/cmd_vel`, `/cmd_vel_final`, `/aruco_detect/markers_left`)

**提案:** REP-144標準命名規則

**命名規則:**
```
/[robot_namespace]/[functionality]/[topic_name]

例:
/multigo/navigation/cmd_vel
/multigo/docking/cmd_vel
/multigo/perception/markers/left
/multigo/perception/markers/right
/multigo/motion/cmd_vel_final
/multigo/safety/emergency_stop
/multigo/mission/state
```

**移行マップ:**

| 旧名称 | 新名称 | 備考 |
|----------|----------|-------|
| `/cmd_vel` | `/multigo/navigation/cmd_vel` | Nav2出力 |
| `/cmd_vel_final` | `/multigo/docking/cmd_vel` | ドッキング出力 |
| `/cmd_vel_adjusted` | `/multigo/motion/cmd_vel` | モーターへの最終 |
| `/aruco_detect/markers_left` | `/multigo/perception/markers/left` | |
| `/aruco_detect/markers_right` | `/multigo/perception/markers/right` | |
| `/goal_pose` | `/multigo/navigation/goal_pose` | |
| `/scan` | `/multigo/sensors/scan` | LiDAR |
| `/camera/color/image_raw_left` | `/multigo/sensors/camera/left/image_raw` | |
| `/camera/color/image_raw_right` | `/multigo/sensors/camera/right/image_raw` | |

**メリット:**
- ✅ 明確な名前空間の整理
- ✅ マルチロボット対応 (robot_idプレフィックスを追加)
- ✅ トピック発見が容易 (`ros2 topic list | grep multigo/safety`)
- ✅ 業界標準 (REP-144)

**工数:** 16時間 (すべてのノード + ローンチファイルを更新 + テスト)

---

## 9. 提案されたテストアーキテクチャ(フェーズ 2)

**現在のステータス:** 0% テストカバレッジ ❌

**提案されたゴール:** 80%+ カバレッジ ✅

### 9.1 ユニットテスト

**フレームワーク:** gtest (C++)、pytest (Python)

**目標:** 40-50のユニットテスト

**テスト例:**

```cpp
// test/test_pid_control.cpp
#include <gtest/gtest.h>
#include "nav_docking/pid_controller.hpp"

TEST(PIDControlTest, IntegralAccumulates) {
    PIDController pid(0.5, 0.1, 0.05);  // Kp, Ki, Kd

    double error = 1.0;
    double dt = 0.1;

    // 5回の反復をシミュレート
    double output = 0;
    for (int i = 0; i < 5; i++) {
        output = pid.compute(error, dt);
    }

    // 積分は error * dt * iterations = 1.0 * 0.1 * 5 = 0.5 であるべき
    EXPECT_NEAR(pid.getIntegral(), 0.5, 0.001);

    // 出力には積分項が含まれるべき: 0.1 * 0.5 = 0.05
    EXPECT_GT(output, 0.05);  // 少なくとも積分の寄与
}

TEST(PIDControlTest, DerivativeReducesOscillation) {
    PIDController pid(0.5, 0.0, 0.5);  // 高いKd

    double dt = 0.1;

    // 大きな初期誤差
    double output1 = pid.compute(1.0, dt);

    // 急速に減少する誤差 (目標に近づく)
    double output2 = pid.compute(0.5, dt);

    // 微分項が出力を減少させるべき (減衰)
    EXPECT_LT(output2, output1);
}

TEST(DualMarkerTest, CenterCalculation) {
    double left_x = 1.0, right_x = 0.8;
    double center_x = (left_x + right_x) / 2;

    EXPECT_NEAR(center_x, 0.9, 0.001);  // 0.9であるべき、1.4ではない!
}

TEST(DualMarkerTest, OperatorPrecedenceBug) {
    double left_x = 1.0, right_x = 0.8;

    // 誤り (演算子の優先順位):
    double wrong_center = (left_x) + (right_x) / 2;  // = 1.0 + 0.4 = 1.4
    EXPECT_NEAR(wrong_center, 1.4, 0.001);

    // 正解:
    double correct_center = (left_x + right_x) / 2;  // = 1.8 / 2 = 0.9
    EXPECT_NEAR(correct_center, 0.9, 0.001);

    // 異なることを確認!
    EXPECT_NE(wrong_center, correct_center);
}

TEST(ArucoDetectTest, OpenCVToROSConversion) {
    cv::Vec3d opencv_tvec(1.0, 0.0, 2.0);  // OpenCVフレーム

    // ROSフレームに変換
    geometry_msgs::msg::Pose ros_pose = convertOpenCVToROS(opencv_tvec);

    // OpenCV X -> ROS -Y, OpenCV Y -> ROS -Z, OpenCV Z -> ROS X
    EXPECT_NEAR(ros_pose.position.x, 2.0, 0.001);   // Z -> X
    EXPECT_NEAR(ros_pose.position.y, -1.0, 0.001);  // X -> -Y
    EXPECT_NEAR(ros_pose.position.z, 0.0, 0.001);   // Y -> -Z
}

TEST(StateMachineTest, StateTransitions) {
    MissionStateMachine sm;

    EXPECT_EQ(sm.getState(), MissionState::IDLE);

    // アプローチをトリガー
    sm.handleEvent(Event::APPROACH_REQUESTED);
    EXPECT_EQ(sm.getState(), MissionState::WAITING_APPROACH_CONFIRMATION);

    // ユーザー確認
    sm.handleEvent(Event::USER_CONFIRMED);
    EXPECT_EQ(sm.getState(), MissionState::NAVIGATING_TO_GOAL);

    // ナビゲーション成功
    sm.handleEvent(Event::APPROACH_SUCCEEDED);
    EXPECT_EQ(sm.getState(), MissionState::WAITING_DOCK_CONFIRMATION);
}

TEST(SafetySupervisorTest, EmergencyStopTriggered) {
    SafetySupervisor supervisor;

    EXPECT_EQ(supervisor.getState(), SafetyState::SAFE);

    // 非常に近い障害物をシミュレート (10cm)
    auto scan = createLaserScan({0.10, 0.10, 0.10});  // すべての範囲が10cm
    supervisor.scanCallback(std::make_shared<LaserScan>(scan));

    EXPECT_EQ(supervisor.getState(), SafetyState::EMERGENCY_STOP);
}

TEST(SafetyZoneTest, KeepOutZoneDetection) {
    SafetyZoneManager zone_manager;

    // 立入禁止ゾーンを定義 (エレベーターシャフト)
    Polygon elevator = {{5.0, 10.0}, {6.0, 10.0}, {6.0, 11.0}, {5.0, 11.0}};
    zone_manager.addKeepOutZone("elevator", elevator);

    // ゾーン内をテスト (安全でない)
    EXPECT_FALSE(zone_manager.isPositionSafe(5.5, 10.5));

    // ゾーン外をテスト (安全)
    EXPECT_TRUE(zone_manager.isPositionSafe(7.0, 10.5));
}
```

**テスト実行:**
```bash
# テスト付きでビルド
colcon build --symlink-install

# すべてのユニットテストを実行
colcon test --packages-select nav_docking

# 特定のテストを実行
colcon test --packages-select nav_docking --ctest-args -R test_pid_control

# 結果を表示
colcon test-result --verbose
```

**工数:** 60時間 (48h開発 + 12hフィクスチャ/モック)

---

### 9.2 統合テスト

**目標:** 10-15の統合テスト

**テスト環境:** フェイクセンサー付きシミュレーション (Gazebo)

**テスト例:**

```python
# test/integration/test_approach_workflow.py
import pytest
import rclpy
from nav_interface.action import Approach

def test_approach_success(ros_node):
    """完全なアプローチワークフローをテスト"""

    # セットアップ: ロボットが原点、マーカーが (5, 0)
    publish_marker_pose(x=5.0, y=0.0, yaw=3.14159)

    # 実行: アプローチゴールを送信
    result = send_approach_goal(timeout_sec=60.0)

    # 検証: アプローチ成功
    assert result.success == True

    # 検証: ロボットがゴールに到達 (5cm以内)
    final_pose = get_robot_pose()
    assert abs(final_pose.x - 4.695) < 0.05  # 5.0 - 0.305
    assert abs(final_pose.y - 0.0) < 0.05

def test_docking_precision(ros_node):
    """精密ドッキングの精度をテスト"""

    # セットアップ: ロボットをアプローチ位置に配置
    position_robot_at_approach_pose()

    # 実行: ドッキングゴールを送信
    result = send_dock_goal(timeout_sec=60.0)

    # 検証: ドッキング成功
    assert result.success == True

    # 検証: 高精度 (<2mm)
    assert result.final_distance_error_x < 0.002  # 2mm
    assert result.final_distance_error_y < 0.002  # 2mm

    # 検証: 所要時間が妥当 (15-30s)
    assert 15.0 < result.duration_seconds < 30.0

def test_obstacle_stops_docking(ros_node):
    """安全性: 障害物検出がドッキングを停止"""

    # セットアップ: ドッキング開始
    dock_goal_handle = send_dock_goal_async()
    time.sleep(2.0)  # ドッキング開始を待つ

    # シミュレート: 経路に障害物が出現
    spawn_obstacle_in_path(distance=0.2)  # ロボットから20cm

    # 検証: ロボットが1秒以内に停止
    time.sleep(1.0)
    velocity = get_robot_velocity()
    assert velocity.linear.x < 0.01  # ほぼ停止

    # 検証: ドッキングアクションが失敗または一時停止を報告
    # (実装に依存: abort、pause、または wait)

def test_marker_loss_recovery(ros_node):
    """ドッキングがマーカー消失を処理することをテスト"""

    # セットアップ: ドッキング開始
    dock_goal_handle = send_dock_goal_async()
    time.sleep(2.0)

    # シミュレート: マーカーが消える (障害物でブロック)
    hide_markers(duration_sec=3.0)

    # 検証: ロボットが停止または減速
    time.sleep(0.5)
    velocity = get_robot_velocity()
    assert velocity.linear.x < 0.05

    # シミュレート: マーカーが再出現
    show_markers()

    # 検証: ドッキングが再開して完了
    result = wait_for_dock_result(timeout_sec=30.0)
    assert result.success == True

def test_full_workflow_approach_dock(ros_node):
    """完全なワークフローをテスト: idle → approach → dock → docked"""

    # 初期状態を検証
    assert get_mission_state() == "IDLE"

    # アプローチをリクエスト
    trigger_approach()
    time.sleep(1.0)
    assert get_mission_state() == "WAITING_APPROACH_CONFIRMATION"

    # ユーザーがアプローチを確認
    confirm_approach()
    assert get_mission_state() == "NAVIGATING_TO_GOAL"

    # アプローチ完了を待つ
    wait_for_state("WAITING_DOCK_CONFIRMATION", timeout_sec=60.0)

    # ユーザーがドッキングを確認
    confirm_dock()
    assert get_mission_state() == "DOCKING"

    # ドッキング完了を待つ
    wait_for_state("DOCKED", timeout_sec=60.0)

    # 最終状態と位置を検証
    assert get_mission_state() == "DOCKED"
    final_pose = get_robot_pose()
    target_pose = get_target_pose()
    assert pose_distance(final_pose, target_pose) < 0.002  # <2mm
```

**テストフィクスチャ:**

```python
@pytest.fixture
def ros_node():
    """テスト用のROS 2ノードを作成"""
    rclpy.init()
    node = rclpy.create_node('test_node')
    yield node
    node.destroy_node()
    rclpy.shutdown()

@pytest.fixture
def simulation():
    """Gazeboシミュレーションを起動"""
    sim_process = launch_simulation("test_world.sdf")
    time.sleep(10.0)  # シミュレーションの安定化を待つ
    yield sim_process
    sim_process.terminate()
    sim_process.wait()
```

**統合テスト実行:**
```bash
# テストシミュレーションを起動
ros2 launch multigo_testing test_simulation.launch.py

# 統合テストを実行
pytest test/integration/ -v

# 特定のテストを実行
pytest test/integration/test_approach_workflow.py::test_full_workflow_approach_dock -v -s
```

**工数:** 30時間 (20hテスト + 10hフィクスチャ/シミュレーション)

---

### 9.3 CI/CDパイプライン

**ツール:** GitHub Actions

**設定:** `.github/workflows/ci.yml`

```yaml
name: MultiGo CI Pipeline

on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main]

jobs:
  build-and-test:
    runs-on: ubuntu-22.04
    container:
      image: ros:humble

    steps:
      - name: コードをチェックアウト
        uses: actions/checkout@v3

      - name: 依存関係をインストール
        run: |
          apt update
          apt install -y python3-pip
          rosdep update
          rosdep install --from-paths src --ignore-src -r -y

      - name: ワークスペースをビルド
        run: |
          source /opt/ros/humble/setup.bash
          colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

      - name: ユニットテストを実行
        run: |
          source install/setup.bash
          colcon test --packages-select nav_docking nav_goal nav_control aruco_detect
          colcon test-result --verbose

      - name: 統合テストを実行
        run: |
          source install/setup.bash
          pytest test/integration/ -v

      - name: カバレッジレポートを生成
        run: |
          pip install coverage
          coverage report --fail-under=80

      - name: カバレッジをCodecovにアップロード
        uses: codecov/codecov-action@v3
        with:
          files: ./coverage.xml
          flags: unittests
          name: codecov-umbrella

      - name: コードをリント (C++)
        run: |
          apt install -y clang-format
          find src -name '*.cpp' -o -name '*.hpp' | xargs clang-format --dry-run --Werror

      - name: コードをリント (Python)
        run: |
          pip install flake8
          flake8 src --count --select=E9,F63,F7,F82 --show-source --statistics
```

**カバレッジ要件:**
- 最低80%のラインカバレッジ
- 最低70%のブランチカバレッジ
- すべてのテストが合格する必要がある

**工数:** 16時間 (CIセットアップ + 設定 + テスト)

---

## 10. 提案されたデプロイメントアーキテクチャ(フェーズ 4)

### 10.1 Dockerデプロイメント

**目的:** 簡単な複製のためのコンテナ化されたデプロイメント

**Docker Compose設定:**

```yaml
# docker-compose.yml
version: '3.8'

services:
  # ハードウェアインターフェース (デバイスアクセスが必要)
  hardware:
    image: multigo/hardware:${VERSION:-latest}
    container_name: multigo_hardware
    privileged: true  # デバイスアクセスのため
    devices:
      - /dev/video0:/dev/video0        # 左カメラ
      - /dev/video1:/dev/video1        # 右カメラ
      - /dev/ttyUSB0:/dev/ttyUSB0      # LiDAR
      - /dev/phidget:/dev/phidget      # モーターコントローラー
    environment:
      - ROS_DOMAIN_ID=42
    command: ros2 launch boot boot.launch.py
    networks:
      - ros_network

  # ナビゲーションとドッキング
  navigation:
    image: multigo/navigation:${VERSION:-latest}
    container_name: multigo_navigation
    depends_on:
      - hardware
    environment:
      - ROS_DOMAIN_ID=42
      - ROS_LOCALHOST_ONLY=0
    command: ros2 launch boot run.launch.py
    networks:
      - ros_network

  # セーフティスーパーバイザー (重要、常に再起動)
  safety:
    image: multigo/safety:${VERSION:-latest}
    container_name: multigo_safety
    depends_on:
      - hardware
      - navigation
    environment:
      - ROS_DOMAIN_ID=42
    command: ros2 run safety_supervisor safety_supervisor_node
    restart: always  # セーフティは重要!
    networks:
      - ros_network

  # Web UI (オプション)
  web_ui:
    image: multigo/web_ui:${VERSION:-latest}
    container_name: multigo_web_ui
    ports:
      - "8080:8080"
    environment:
      - ROS_DOMAIN_ID=42
    command: python3 -m multigo_web_ui
    networks:
      - ros_network

networks:
  ros_network:
    driver: bridge
```

**Dockerfiles:**

```dockerfile
# Dockerfile.hardware
FROM ros:humble-ros-base

# 依存関係をインストール
RUN apt-get update && apt-get install -y \
    ros-humble-camera-info-manager \
    ros-humble-cv-bridge \
    python3-pip \
    libphidget22 \
    && rm -rf /var/lib/apt/lists/*

# ソースをコピー
COPY src/camera_publisher /workspace/src/camera_publisher
COPY src/mecanum_wheels /workspace/src/mecanum_wheels

# ビルド
WORKDIR /workspace
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# エントリーポイント
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
```

**ビルドとデプロイ:**

```bash
# すべてのイメージをビルド
docker-compose build

# デプロイ
docker-compose up -d

# ログを表示
docker-compose logs -f

# 停止
docker-compose down
```

**工数:** 40時間 (Dockerfiles作成 + デプロイテスト + ドキュメント)

---

### 10.2 設定管理

**問題:** パラメータファイルが多すぎる、優先順位が不明確

**提案:** 階層的設定システム

**構造:**

```
config/
├── defaults/
│   ├── navigation.yaml       # システム全体のナビゲーションデフォルト
│   ├── docking.yaml          # システム全体のドッキングデフォルト
│   └── safety.yaml           # セーフティパラメータ
│
├── robots/
│   ├── multigo_001.yaml      # ロボット固有のオーバーライド
│   ├── multigo_002.yaml
│   └── multigo_test.yaml     # テストロボット
│
├── environments/
│   ├── hospital_floor2.yaml  # 環境固有 (マップ、ゾーン)
│   ├── hospital_floor3.yaml
│   └── simulation.yaml       # テスト用
│
└── missions/
    ├── wheelchair_dock.yaml  # ミッション固有パラメータ
    ├── bed_dock.yaml
    └── transport.yaml
```

**読み込み優先順位 (高から低):**
1. コマンドライン引数 (`--params-file`)
2. ミッション設定 (`wheelchair_dock.yaml`)
3. 環境設定 (`hospital_floor2.yaml`)
4. ロボット設定 (`multigo_001.yaml`)
5. システムデフォルト (`navigation.yaml`, `docking.yaml`)

**設定ローダー:**

```python
# config_loader.py
class HierarchicalConfigLoader:
    def __init__(self, config_dir: str):
        self.config_dir = Path(config_dir)

    def load_config(self,
                   robot_id: str,
                   environment: str,
                   mission: str) -> Dict:
        """設定階層を読み込んでマージ"""

        # デフォルトから開始
        config = self.load_yaml(self.config_dir / "defaults" / "navigation.yaml")
        config.update(self.load_yaml(self.config_dir / "defaults" / "docking.yaml"))
        config.update(self.load_yaml(self.config_dir / "defaults" / "safety.yaml"))

        # ロボット固有でオーバーライド
        robot_config = self.config_dir / "robots" / f"{robot_id}.yaml"
        if robot_config.exists():
            config.update(self.load_yaml(robot_config))

        # 環境固有でオーバーライド
        env_config = self.config_dir / "environments" / f"{environment}.yaml"
        if env_config.exists():
            config.update(self.load_yaml(env_config))

        # ミッション固有でオーバーライド
        mission_config = self.config_dir / "missions" / f"{mission}.yaml"
        if mission_config.exists():
            config.update(self.load_yaml(mission_config))

        return config
```

**使用法:**

```bash
# 階層的設定で起動
ros2 launch multigo_launch run.launch.py \
    robot_id:=multigo_001 \
    environment:=hospital_floor2 \
    mission:=wheelchair_dock
```

**工数:** 24時間 (ローダー実装 + 設定移行 + ドキュメント)

---

### 10.3 ティーチングモード (ウェイポイントシステム)

**目的:** 技術者でないユーザーがロボットに新しいルートとドッキング位置を教えられる

**実装:**

**1. ウェイポイントマネージャー:**

```cpp
class WaypointManager : public rclcpp::Node {
private:
    std::map<std::string, geometry_msgs::msg::Pose> waypoints_;
    std::string waypoints_file_;

public:
    WaypointManager() : Node("waypoint_manager") {
        // サービス
        save_service_ = create_service<multigo_msgs::srv::SaveWaypoint>(
            "/waypoint/save",
            std::bind(&WaypointManager::handleSave, this, _1, _2)
        );

        get_service_ = create_service<multigo_msgs::srv::GetWaypoint>(
            "/waypoint/get",
            std::bind(&WaypointManager::handleGet, this, _1, _2)
        );

        list_service_ = create_service<multigo_msgs::srv::ListWaypoints>(
            "/waypoint/list",
            std::bind(&WaypointManager::handleList, this, _1, _2)
        );

        // 既存のウェイポイントを読み込み
        loadWaypoints();
    }

    void handleSave(
        const std::shared_ptr<SaveWaypoint::Request> request,
        std::shared_ptr<SaveWaypoint::Response> response
    ) {
        // 現在のロボット姿勢を取得
        geometry_msgs::msg::PoseStamped current_pose;
        if (!getCurrentPose(current_pose)) {
            response->success = false;
            response->message = "現在の姿勢の取得に失敗";
            return;
        }

        // ウェイポイントを保存
        waypoints_[request->waypoint_id] = current_pose.pose;

        // ディスクに永続化
        saveWaypoints();

        response->success = true;
        response->message = "ウェイポイント保存: " + request->waypoint_id;

        RCLCPP_INFO(get_logger(), "ウェイポイント '%s' を (%.2f, %.2f) に保存",
                    request->waypoint_id.c_str(),
                    current_pose.pose.position.x,
                    current_pose.pose.position.y);
    }

    void saveWaypoints() {
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "waypoints";
        out << YAML::Value << YAML::BeginSeq;

        for (const auto& [id, pose] : waypoints_) {
            out << YAML::BeginMap;
            out << YAML::Key << "id" << YAML::Value << id;
            out << YAML::Key << "pose";
            out << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "x" << YAML::Value << pose.position.x;
            out << YAML::Key << "y" << YAML::Value << pose.position.y;
            out << YAML::Key << "yaw" << YAML::Value << getYawFromQuaternion(pose.orientation);
            out << YAML::EndMap;
            out << YAML::EndMap;
        }

        out << YAML::EndSeq;
        out << YAML::EndMap;

        std::ofstream fout(waypoints_file_);
        fout << out.c_str();
        fout.close();
    }
};
```

**2. ティーチングモード起動:**

```python
# teaching_mode.launch.py
def generate_launch_description():
    return LaunchDescription([
        # ハードウェアを起動
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('boot.launch.py')
        ),

        # RTAB-Mapをマッピングモードで起動
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            parameters=[{
                'Mem/IncrementalMemory': 'true',  # マッピングモード
                'Mem/InitWMWithAllNodes': 'false'
            }]
        ),

        # ウェイポイントマネージャーを起動
        Node(
            package='multigo_waypoints',
            executable='waypoint_manager',
            parameters=[{
                'waypoints_file': '/config/waypoints/hospital_floor2.yaml'
            }]
        ),

        # 手動制御用のテレオペレーションを起動
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            prefix='xterm -e',  # 別のターミナルで実行
            remappings=[
                ('/cmd_vel', '/multigo/teleop/cmd_vel')
            ]
        ),

        # ウェイポイントティーチングGUI
        Node(
            package='multigo_teaching',
            executable='teaching_gui',
            output='screen'
        )
    ])
```

**3. ティーチングワークフロー:**

```bash
# ステップ1: ティーチングモードで起動
ros2 launch multigo_teaching teaching_mode.launch.py

# ステップ2: ロボットを手動で場所まで運転 (キーボード/ジョイスティック使用)
# 矢印キーで移動、ロボットは移動しながらマップを構築

# ステップ3: 各重要な場所でウェイポイントを保存
ros2 service call /waypoint/save multigo_msgs/srv/SaveWaypoint "{waypoint_id: 'room_205_door'}"
ros2 service call /waypoint/save multigo_msgs/srv/SaveWaypoint "{waypoint_id: 'room_205_bed'}"

# ステップ4: マップを保存
ros2 service call /rtabmap/save_map std_srvs/srv/Empty

# ステップ5: すべての保存されたウェイポイントをリスト
ros2 service call /waypoint/list multigo_msgs/srv/ListWaypoints

# ステップ6: 保存されたマップで自律モードに切り替え
ros2 launch multigo_launch run.launch.py map:=hospital_floor2.db
```

**ティーチングGUI (RVizプラグインまたはWebインターフェース):**

```
┌─────────────────────────────────────────┐
│       ティーチングモードインターフェース  │
├─────────────────────────────────────────┤
│ 現在位置: (12.5, 8.3)                   │
│ マップカバレッジ: 85%                   │
│                                         │
│ 保存されたウェイポイント:                │
│  ☑ room_205_door     (10.2, 7.1)       │
│  ☑ room_205_bed      (12.5, 8.3)       │
│  ☑ charging_station  (5.0, 2.0)        │
│                                         │
│ [現在位置を保存]                         │
│ ウェイポイント名: _______________        │
│                                         │
│ [マップを保存してティーチングモードを終了] │
└─────────────────────────────────────────┘
```

**工数:** 40時間 (ウェイポイントマネージャー 20h + GUI 20h)

**メリット:**
- ✅ 技術者でないユーザーがルートを教えられる
- ✅ プログラミング不要
- ✅ 環境が変わった時に簡単に更新可能
- ✅ すべての保存されたウェイポイントを視覚化

---

## まとめ: 実装ロードマップ

### フェーズ1: 重大なバグとセーフティ (4週間) - 144時間
**優先度:** 🔴 重要

**成果物:**
- ✅ 3つの重大なバグを修正 (CRIT-01、CRIT-02、HIGH-01)
- ✅ セーフティスーパーバイザーレイヤー
- ✅ 緊急停止メカニズム
- ✅ nav_masterの状態マシン
- ✅ ドッキング中のLiDAR監視
- ✅ 基本的なジオフェンシング

**結果:** テスト準備が整った安全なシステム

---

### フェーズ2: テストインフラストラクチャ (4週間) - 96時間
**優先度:** 🔴 重要

**成果物:**
- ✅ 40-50のユニットテスト (80%カバレッジ)
- ✅ 10-15の統合テスト
- ✅ CI/CDパイプライン (GitHub Actions)
- ✅ 自動回帰テスト

**結果:** リグレッション保護付きでテスト・検証されたシステム

---

### フェーズ3: ROS 2ベストプラクティス (4週間) - 104時間
**優先度:** 🟡 高

**成果物:**
- ✅ 4つのノードをライフサイクルに変換
- ✅ 明示的なQoSポリシー
- ✅ 強化されたアクション定義
- ✅ 標準化されたトピック命名
- ✅ コマンド調停
- ✅ ホロノミックモーション有効化

**結果:** 本番品質のROS 2アーキテクチャ

---

### フェーズ4: デプロイメントと運用 (4週間) - 104時間
**優先度:** 🟡 高

**成果物:**
- ✅ ティーチングモード + ウェイポイントシステム
- ✅ Dockerデプロイメント
- ✅ 階層的設定
- ✅ 診断システム
- ✅ オペレータードキュメント

**結果:** デプロイ可能、運用可能なシステム

---

## 総工数: 616時間 (2人の開発者で16週間)

---

**次のステップ:**
1. このドキュメントをチームでレビュー
2. アーキテクチャ変更を承認
3. [IMPLEMENTATION-GUIDE.md](./IMPLEMENTATION-GUIDE.md) フェーズ1を開始
4. 週次で進捗を追跡

**質問がありますか?** ナビゲーションのヘルプについては [START-HERE.md](./START-HERE.md) を参照

---

**最終更新日:** 2025-12-02
**ドキュメントバージョン:** 1.0
