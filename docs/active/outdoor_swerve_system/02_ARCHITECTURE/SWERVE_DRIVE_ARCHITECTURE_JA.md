# スワーブドライブサブシステムアーキテクチャ

**文書ID:** ARCH-SWERVE-001
**バージョン:** 1.0
**日付:** 2025年12月15日
**ステータス:** ドラフト

---

## 1. 概要

**スワーブドライブサブシステム**は、車椅子搬送ロボットのための全方向移動制御を提供します。ParcelPalの差動駆動とは異なり、このサブシステムはホロノミック運動（独立したvx、vy、ω制御）を可能にし、狭いスペースでの精密な操縦を実現します。

**主要機能:**
- 4輪独立スワーブモジュール（各輪に駆動+操舵）
- 全方向移動（回転なしであらゆる方向に移動可能）
- Nav2コントローラー統合
- リアルタイム逆運動学
- 安全準拠速度制限

---

## 2. コンポーネントアーキテクチャ

### 2.1 システムコンポーネント

```
┌─────────────────────────────────────────────────────────┐
│              スワーブドライブサブシステム                  │
│                                                         │
│  ┌──────────────────┐      ┌──────────────────┐       │
│  │  Nav2コントローラー │─────▶│  スワーブドライブ │       │
│  │  (DWBプランナー)   │      │  コントローラーノード│       │
│  └──────────────────┘      │  (C++)           │       │
│         │                   │                  │       │
│         │ /cmd_vel          │  - IKソルバー     │       │
│         │ (Twist)           │  - ホイール調整   │       │
│         └──────────────────▶│  - 速度ガバナー   │       │
│                             └────────┬─────────┘       │
│                                      │                  │
│                                      │ 個別ホイール      │
│                                      │ コマンド         │
│                                      ↓                  │
│         ┌─────────────┬──────────────┴──────────┬──────────────┐
│         ▼             ▼                         ▼              ▼
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐      │
│  │ モジュール│  │ モジュール│  │ モジュール│  │ モジュール│      │
│  │ FL       │  │ FR       │  │ RL       │  │ RR       │      │
│  │ (ドライバ)│  │ (ドライバ)│  │ (ドライバ)│  │ (ドライバ)│      │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘      │
│       │             │              │             │             │
│       ↓             ↓              ↓             ↓             │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐      │
│  │ モーター  │  │ モーター  │  │ モーター  │  │ モーター  │      │
│  │ コントローラ│  │ コントローラ│  │ コントローラ│  │ コントローラ│     │
│  │ + エンコーダ│  │ + エンコーダ│  │ + エンコーダ│  │ + エンコーダ│      │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘      │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**コンポーネント:**
1. **スワーブドライブコントローラーノード** (C++): nav2_core::Controllerインターフェースを実装するメインコントローラー
2. **モジュールドライバーノード** (4×): ホイールごとの制御ノード（駆動モーター+操舵モーター）
3. **モーターコントローラー** (8×): ハードウェアコントローラー（駆動4+操舵4）
4. **エンコーダー** (8×): 位置フィードバック（ホイール速度4+操舵角度4）

---

## 3. ROS 2ノードアーキテクチャ

### 3.1 スワーブドライブコントローラーノード

**ノード名:** `swerve_drive_controller`
**言語:** C++（パフォーマンスクリティカル）
**ライフサイクル:** マネージドライフサイクルノード

**購読トピック:**
- `/cmd_vel` (geometry_msgs/Twist): 目標ボディ速度（vx、vy、ω）
- `/odom` (nav_msgs/Odometry): 現在のロボットオドメトリ
- `/joint_states` (sensor_msgs/JointState): ホイール位置と速度

**配信トピック:**
- `/wheel_commands` (custom_msgs/SwerveModuleCommand): ホイールごとの駆動/操舵コマンド
- `/swerve/status` (custom_msgs/SwerveStatus): コントローラーステータスと診断
- `/tf` (tf2_msgs/TFMessage): odom → base_link変換

**サービス:**
- `/swerve/reset_odometry` (std_srvs/Trigger): オドメトリを(0,0,0)にリセット

**パラメータ:**
```yaml
wheel_base: 0.6       # メートル（前後間距離）
track_width: 0.5      # メートル（左右間距離）
wheel_radius: 0.0825  # メートル（6.5インチホイール）
max_linear_velocity: 1.5   # m/s
max_angular_velocity: 2.0  # rad/s
max_linear_acceleration: 0.5  # m/s²
```

---

### 3.2 モジュールドライバーノード (×4)

**ノード名:** `swerve_module_fl`、`swerve_module_fr`、`swerve_module_rl`、`swerve_module_rr`
**言語:** C++またはPython
**ライフサイクル:** 標準ノード

**購読トピック:**
- `/wheel_commands` (custom_msgs/SwerveModuleCommand): 目標モジュール状態

**配信トピック:**
- `/joint_states` (sensor_msgs/JointState): 現在のホイール状態（位置、速度）

**ハードウェアインターフェース:**
- CANバスまたはシリアル接続でモーターコントローラーと通信
- エンコーダー値読み取り
- 速度コマンド送信

---

## 4. 逆運動学アルゴリズム

### 4.1 数学モデル

**入力:** ロボットフレームでのボディ速度コマンド `(vx, vy, ω)`

**出力:** 4モジュールの個別ホイール速度と操舵角

**方程式:**

各ホイール `i` の位置 `(xi, yi)` に対して:

```
# 並進と回転からのホイール速度寄与
vxi = vx - ω * yi
vyi = vy + ω * xi

# ホイール速度（大きさ）
vi = sqrt(vxi² + vyi²)

# 操舵角（方向）
θi = atan2(vyi, vxi)
```

**モジュール位置:**
```
FL: ( L/2,  W/2)
FR: ( L/2, -W/2)
RL: (-L/2,  W/2)
RR: (-L/2, -W/2)

ここで:
L = wheel_base = 0.6m
W = track_width = 0.5m
```

---

### 4.2 ホイール調整

**課題:** 操舵遷移中のモジュール間の「競合」を回避

**解決策:** すべてのモジュールが目標角度に達してから駆動力を適用するよう調整

**アルゴリズム:**
```cpp
void updateModules(SwerveCommand cmd) {
    // 1. すべてのモジュールの目標角度を計算
    for (int i = 0; i < 4; i++) {
        target_angles[i] = computeSteeringAngle(cmd, module_pos[i]);
    }

    // 2. 90°以上の回転が必要なモジュールをチェック
    //    必要な場合、角度を反転し駆動方向を逆転（最適化）
    for (int i = 0; i < 4; i++) {
        double angle_error = target_angles[i] - current_angles[i];
        if (abs(angle_error) > M_PI/2) {
            target_angles[i] += M_PI;  // 180°反転
            drive_directions[i] *= -1;  // 駆動逆転
        }
    }

    // 3. すべてのモジュールが目標角度に達するまで待機（許容範囲内）
    bool all_aligned = true;
    for (int i = 0; i < 4; i++) {
        if (abs(target_angles[i] - current_angles[i]) > angle_tolerance) {
            all_aligned = false;
            break;
        }
    }

    // 4. アライメント時のみ駆動速度を適用
    if (all_aligned) {
        for (int i = 0; i < 4; i++) {
            sendDriveCommand(i, target_velocities[i] * drive_directions[i]);
        }
    } else {
        // 操舵中は駆動をランプダウン
        for (int i = 0; i < 4; i++) {
            sendDriveCommand(i, 0.0);
        }
    }
}
```

---

## 5. Nav2統合

### 5.1 コントローラープラグイン

**インターフェース:** `nav2_core::Controller`

**実装:**
```cpp
class SwerveDriveController : public nav2_core::Controller {
public:
    void configure(/* ... */) override;
    void cleanup() override;
    void activate() override;
    void deactivate() override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;

    void setPlan(const nav_msgs::msg::Path & path) override;
};
```

**主要メソッド:**
- `computeVelocityCommands()`: パスに従うためのボディ速度を計算
- `setPlan()`: グローバルプランナーからパスを受信

---

### 5.2 設定 (nav2_params.yaml)

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "swerve_drive_controller::SwerveDriveController"
      # スワーブ固有パラメータ
      wheel_base: 0.6
      track_width: 0.5
      # DWB軌道パラメータ
      max_vel_x: 1.5
      max_vel_y: 1.5  # 全方向移動！
      max_vel_theta: 2.0
      min_vel_x: -1.5
      min_vel_y: -1.5
      min_vel_theta: -2.0
      # 軌道スコアリング
      goal_distance_bias: 20.0
      path_distance_bias: 32.0
      occdist_scale: 0.02
```

---

## 6. オドメトリ計算

### 6.1 順運動学

**入力:** 個別ホイール速度と操舵角

**出力:** ロボットボディ速度 `(vx, vy, ω)`

**方法:** 最小二乗解（4輪の過決定システム）

```cpp
Eigen::Vector3d computeOdometry(
    const std::array<double, 4>& wheel_speeds,
    const std::array<double, 4>& steering_angles) {

    // ヤコビ行列を構築（4輪×3自由度）
    Eigen::Matrix<double, 4, 3> J;
    for (int i = 0; i < 4; i++) {
        double xi = module_positions[i].x;
        double yi = module_positions[i].y;
        double theta = steering_angles[i];

        J(i, 0) = cos(theta);           // vx寄与
        J(i, 1) = sin(theta);           // vy寄与
        J(i, 2) = -yi*cos(theta) + xi*sin(theta);  // ω寄与
    }

    // 測定ホイール速度
    Eigen::Vector4d v_wheels(wheel_speeds.data());

    // 最小二乗解: v_body = (J^T J)^-1 J^T v_wheels
    Eigen::Vector3d v_body = (J.transpose() * J).inverse() * J.transpose() * v_wheels;

    return v_body;  // (vx, vy, ω)
}
```

---

### 6.2 オドメトリ積分

**更新レート:** 50 Hz（ホイールエンコーダーから）

**積分:**
```cpp
void updateOdometry(double dt) {
    Eigen::Vector3d v_body = computeOdometry(wheel_speeds, steering_angles);

    double vx = v_body(0);
    double vy = v_body(1);
    double omega = v_body(2);

    // 姿勢更新（odomフレーム内）
    theta += omega * dt;
    x += (vx * cos(theta) - vy * sin(theta)) * dt;
    y += (vx * sin(theta) + vy * cos(theta)) * dt;

    // オドメトリ配信
    publishOdometry(x, y, theta, vx, vy, omega);
}
```

---

## 7. 安全統合

### 7.1 速度ガバナー

**目的:** コマンド速度に安全制限を適用

**制限:**
- 最大直線速度: 1.5 m/s（乗客なし）、1.0 m/s（乗客あり）
- 最大角速度: 2.0 rad/s
- 最大加速度: 0.5 m/s²

**実装:**
```cpp
geometry_msgs::msg::Twist limitVelocity(
    const geometry_msgs::msg::Twist& cmd_vel,
    bool passenger_onboard) {

    geometry_msgs::msg::Twist limited = cmd_vel;

    // 速度制限
    double max_linear = passenger_onboard ? 1.0 : 1.5;
    double linear_mag = sqrt(cmd_vel.linear.x * cmd_vel.linear.x +
                             cmd_vel.linear.y * cmd_vel.linear.y);
    if (linear_mag > max_linear) {
        double scale = max_linear / linear_mag;
        limited.linear.x *= scale;
        limited.linear.y *= scale;
    }

    // 角速度制限
    limited.angular.z = std::clamp(cmd_vel.angular.z, -2.0, 2.0);

    // 加速度制限（前回コマンドと比較）
    double accel_dt = 0.05;  // 20 Hzコントローラー
    double max_delta_v = max_accel * accel_dt;
    // ... （前回コマンドからのデルタを制限）

    return limited;
}
```

---

### 7.2 緊急停止

**動作:** すべてのホイール動作を即座に停止

**実装:**
```cpp
void emergencyStop() {
    // すべてのモジュールにゼロ速度を送信
    for (int i = 0; i < 4; i++) {
        sendDriveCommand(i, 0.0);
        // 現在の操舵角を維持（急激な動作を避けるためリセットしない）
    }

    // ブレーキ作動
    engageBrakes();
}
```

---

## 8. テスト戦略

### 8.1 ユニットテスト
- 逆運動学の正確性（既知入力→期待出力）
- 順運動学（ホイール速度→ボディ速度）
- 速度制限ロジック
- モジュール調整アルゴリズム

### 8.2 統合テスト
- Nav2コントローラーインターフェース
- ROS 2トピック通信
- 緊急停止応答時間
- オドメトリ精度（正方形走行、誤差測定）

### 8.3 フィールドテスト
- 全方向移動（左右ストレイフ、対角線）
- 精密測位（±10cm精度）
- スムーズな遷移（操舵変更中にジャーク動作なし）
- 荷重下での性能（100kgペイロード）

---

## 9. 性能目標

| メトリック | 目標 | 検証 |
|--------|--------|------------|
| 制御ループレート | 20 Hz | タイムスタンプ解析 |
| IK計算時間 | <5ms | プロファイリング |
| モジュール調整時間 | <200ms | モーションキャプチャ |
| オドメトリドリフト | <2% 距離 | 正方形パステスト |
| 最大直線速度 | 1.5 m/s | 速度テスト |
| 最大角速度 | 2.0 rad/s | 回転テスト |
| 加速度 | 0.5 m/s² | 加速度テスト |

---

## 10. 実装フェーズ

### フェーズ1: 基本コントローラー（スプリント1-2）
- ✅ 逆運動学実装
- ✅ 基本モジュールドライバー（シミュレーションハードウェア）
- ✅ オドメトリ計算
- ✅ ユニットテスト

### フェーズ2: Nav2統合（スプリント3-4）
- ✅ コントローラープラグインインターフェース
- ✅ DWBプランナー統合
- ✅ パスフォローイングテスト
- ✅ ローカライゼーションとの統合

### フェーズ3: 安全&ポリッシュ（スプリント5-6）
- ✅ 速度ガバナー
- ✅ 緊急停止
- ✅ モジュール調整改善
- ✅ フィールドテストとチューニング

---

**文書ステータス:** ドラフト
**実装ステータス:** 未開始
**承認必要:** ナビゲーションリード、スワーブドライブエンジニア、安全エンジニア
