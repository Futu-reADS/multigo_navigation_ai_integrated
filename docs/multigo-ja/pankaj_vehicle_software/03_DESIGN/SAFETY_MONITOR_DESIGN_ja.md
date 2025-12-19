# 安全監視詳細設計

**ドキュメントID:** DESIGN-SAFETY-001
**バージョン:** 1.0
**日付:** 2025-12-15
**ステータス:** ドラフト

---

## 1. 概要

本ドキュメントは、4つの独立したレイヤーによる多層防御安全性を実装する、安全監視の詳細設計仕様を提供します。

**主要コンポーネント:**
- EmergencyStopController（ハードウェアリレー + ソフトウェアモニター）
- VelocityGovernor（コンテキストベースの速度制限）
- HealthMonitor（すべてのサブシステムのウォッチドッグ）
- CollisionPredictor（予測的安全性）

**安全アーキテクチャ:** 4つの独立したレイヤー
1. **緊急停止（ハードウェア）** - <100msリレー遮断
2. **システム監視（ウォッチドッグ）** - ソフトウェア健康チェック
3. **衝突回避（知覚 + 計画）** - 予測的安全性
4. **動作安全（制約）** - 速度/斜面/天候制限

---

## 2. 安全レイヤー

### レイヤー1: 緊急停止（ハードウェア）

```
物理E-Stopボタン → ハードウェアリレー → モーター電源遮断
        │                                      (<100ms)
        ├→ ソフトウェアE-Stopトピック（/emergency_stop）
        └→ 安全ステータスLED
```

**実装:**
```cpp
class EmergencyStopController {
public:
    void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            // GPIO経由でハードウェアリレーをトリガー
            gpio_->setPin(ESTOP_RELAY_PIN, GPIO_HIGH);

            // ゼロ速度を公開
            publishZeroVelocity();

            // 安全状態を設定
            safety_state_ = SafetyState::EMERGENCY_STOP;

            RCLCPP_ERROR(logger_, "EMERGENCY STOP ACTIVATED");
        }
    }

private:
    void publishZeroVelocity() {
        geometry_msgs::msg::Twist zero_cmd;
        // すべての速度 = 0
        cmd_vel_pub_->publish(zero_cmd);
    }
};
```

---

### レイヤー2: ウォッチドッグ監視

```cpp
class HealthMonitor {
public:
    struct SubsystemHealth {
        std::string name;
        rclcpp::Time last_heartbeat;
        double timeout_threshold;  // 秒
        HealthStatus status;
    };

    void monitorLoop() {
        auto now = node_->now();

        for (auto& subsystem : monitored_subsystems_) {
            double elapsed = (now - subsystem.last_heartbeat).seconds();

            if (elapsed > subsystem.timeout_threshold) {
                RCLCPP_ERROR(logger_, "Subsystem %s timeout (%.1fs)",
                             subsystem.name.c_str(), elapsed);

                handleSubsystemFailure(subsystem.name);
            }
        }
    }

private:
    void handleSubsystemFailure(const std::string& subsystem_name) {
        // クリティカルサブシステムは安全停止をトリガー
        if (isCriticalSubsystem(subsystem_name)) {
            triggerSafeStop("Critical subsystem failure: " + subsystem_name);
        } else {
            // 非クリティカル: 警告をログ、縮退モードに入る
            enterDegradedMode(subsystem_name);
        }
    }

    std::vector<SubsystemHealth> monitored_subsystems_ = {
        {"swerve_controller", now(), 1.0, HealthStatus::OK},
        {"ndt_localization", now(), 2.0, HealthStatus::OK},
        {"perception", now(), 1.0, HealthStatus::OK},
        {"navigation", now(), 5.0, HealthStatus::OK}
    };
};
```

---

### レイヤー3: 衝突回避

**予測的衝突検出:**
```cpp
class CollisionPredictor {
public:
    bool predictCollision(
        const geometry_msgs::msg::Twist& cmd_vel,
        const sensor_msgs::msg::PointCloud2& obstacle_cloud,
        double prediction_horizon)  // 秒
    {
        // ロボットの軌道をシミュレート
        auto trajectory = simulateTrajectory(cmd_vel, prediction_horizon);

        // 軌道に沿って衝突をチェック
        for (const auto& pose : trajectory) {
            if (checkCollision(pose, obstacle_cloud)) {
                return true;  // 衝突予測
            }
        }

        return false;  // 安全
    }

private:
    std::vector<Pose> simulateTrajectory(
        const geometry_msgs::msg::Twist& cmd_vel,
        double horizon)
    {
        std::vector<Pose> trajectory;
        Pose current_pose = getCurrentPose();

        double dt = 0.1;  // 100msタイムステップ
        for (double t = 0; t < horizon; t += dt) {
            current_pose = integrateMotion(current_pose, cmd_vel, dt);
            trajectory.push_back(current_pose);
        }

        return trajectory;
    }
};
```

---

### レイヤー4: 速度ガバナー

**コンテキスト認識速度制限:**
```cpp
class VelocityGovernor {
public:
    geometry_msgs::msg::Twist limitVelocity(
        const geometry_msgs::msg::Twist& cmd_vel,
        const SafetyContext& context)
    {
        geometry_msgs::msg::Twist limited_cmd = cmd_vel;

        // 1. 乗客乗車チェック
        double max_linear_vel = context.passenger_onboard ?
            params_.max_vel_with_passenger : params_.max_vel_empty;

        // 2. 斜面制限
        if (std::abs(context.slope_angle) > params_.max_slope) {
            RCLCPP_WARN(logger_, "Slope %.1f° exceeds max %.1f° - reducing speed",
                        context.slope_angle * 180.0 / M_PI,
                        params_.max_slope * 180.0 / M_PI);
            max_linear_vel *= 0.5;  // 急斜面では50%速度
        }

        // 3. 天候条件
        if (context.weather == Weather::LIGHT_RAIN) {
            max_linear_vel *= 0.7;  // 雨天時は70%速度
        }

        // 4. 障害物との近接
        if (context.min_obstacle_distance < 2.0) {
            double proximity_factor = context.min_obstacle_distance / 2.0;
            max_linear_vel *= proximity_factor;
        }

        // 制限を適用
        double linear_vel = std::sqrt(
            cmd_vel.linear.x * cmd_vel.linear.x +
            cmd_vel.linear.y * cmd_vel.linear.y);

        if (linear_vel > max_linear_vel) {
            double scale = max_linear_vel / linear_vel;
            limited_cmd.linear.x *= scale;
            limited_cmd.linear.y *= scale;
        }

        // 角速度制限
        limited_cmd.angular.z = std::clamp(
            cmd_vel.angular.z,
            -params_.max_angular_vel,
            params_.max_angular_vel);

        return limited_cmd;
    }

private:
    struct Parameters {
        double max_vel_empty = 1.5;          // m/s
        double max_vel_with_passenger = 1.0; // m/s
        double max_angular_vel = 2.0;        // rad/s
        double max_slope = 0.175;            // ラジアン（10°）
    };

    Parameters params_;
};
```

---

## 3. 安全状態機械

```
┌────────────────────────────────────────────────────────────┐
│              安全状態機械                                  │
└────────────────────────────────────────────────────────────┘

    ┌──────────┐
    │  正常    │ ← すべてのシステムが動作中
    └────┬─────┘
         │
         ├─→ サブシステム障害 → ┌─────────────┐
         │                      │ 縮退        │
         │                      └─────────────┘
         │
         ├─→ 障害物検出 → ┌──────────────┐
         │               │ 衝突         │
         │               │ 回避         │
         │               └──────────────┘
         │
         └─→ E-STOP押下 → ┌────────────┐
                         │ 緊急       │
                         │ 停止       │
                         └────────────┘

回復:
    緊急停止 → (手動リセット) → 正常
    縮退 → (サブシステム回復) → 正常
    衝突回避 → (障害物除去) → 正常
```

---

## 4. 構成

**safety_params.yaml:**
```yaml
safety_monitor:
  ros__parameters:
    # 速度制限
    max_linear_velocity_empty: 1.5        # m/s
    max_linear_velocity_passenger: 1.0    # m/s
    max_angular_velocity: 2.0             # rad/s
    max_acceleration: 0.5                 # m/s²

    # 斜面制限
    max_slope_angle: 0.175                # ラジアン（10°）
    max_slope_angle_passenger: 0.087      # ラジアン（5°）

    # 衝突予測
    prediction_horizon: 3.0               # 秒
    min_obstacle_distance: 0.5            # メートル（緊急停止）
    warning_obstacle_distance: 2.0        # メートル（減速）

    # ウォッチドッグタイムアウト
    subsystem_timeouts:
      swerve_controller: 1.0              # 秒
      ndt_localization: 2.0
      perception: 1.0
      navigation: 5.0

    # 天候動作
    allow_light_rain: true
    allow_heavy_rain: false
    max_wind_speed: 10.0                  # m/s
```

---

## 5. テスト戦略

**安全テスト例:**
```cpp
TEST(SafetyMonitorTest, EmergencyStopActivation) {
    SafetyMonitor monitor;

    // 緊急停止をシミュレート
    std_msgs::msg::Bool estop_msg;
    estop_msg.data = true;
    monitor.emergencyStopCallback(
        std::make_shared<std_msgs::msg::Bool>(estop_msg));

    // 状態を確認
    EXPECT_EQ(monitor.getSafetyState(), SafetyState::EMERGENCY_STOP);

    // ゼロ速度が公開されたことを確認
    auto cmd_vel = monitor.getLastCommandVelocity();
    EXPECT_DOUBLE_EQ(cmd_vel.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(cmd_vel.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(cmd_vel.angular.z, 0.0);
}

TEST(VelocityGovernorTest, PassengerSpeedLimit) {
    VelocityGovernor governor;

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 2.0;  // 2.0 m/sを要求

    SafetyContext context;
    context.passenger_onboard = true;

    auto limited_cmd = governor.limitVelocity(cmd_vel, context);

    // 乗客がいる場合は1.0 m/sに制限される
    EXPECT_LE(limited_cmd.linear.x, 1.0);
}
```

---

**ドキュメントステータス:** ドラフト
**実装ステータス:** 開発準備完了
**承認が必要:** 安全リード、システムエンジニア
