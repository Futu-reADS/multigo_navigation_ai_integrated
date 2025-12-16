# スワーブドライブコントローラ詳細設計

**ドキュメントID:** DESIGN-SWERVE-001
**バージョン:** 1.0
**日付:** 2025-12-15
**ステータス:** ドラフト

---

## 1. 概要

本ドキュメントは、スワーブドライブコントローラの詳細設計仕様を提供します。クラス構造、アルゴリズム、データフロー、実装詳細を含みます。

**主要コンポーネント:**
- SwerveDriveController (Nav2プラグイン)
- SwerveModule (FL, FR, RL, RR用に4インスタンス)
- InverseKinematics ソルバー
- VelocityGovernor
- OdometryPublisher

---

## 2. クラス図

```
┌─────────────────────────────────────────────────────────────┐
│                  nav2_core::Controller                      │
│                  (インターフェース)                          │
└───────────────────────┬─────────────────────────────────────┘
                        │ 継承
                        ↓
┌─────────────────────────────────────────────────────────────┐
│              SwerveDriveController                          │
├─────────────────────────────────────────────────────────────┤
│ - wheel_base_: double                                       │
│ - track_width_: double                                      │
│ - wheel_radius_: double                                     │
│ - max_linear_velocity_: double                              │
│ - max_angular_velocity_: double                             │
│ - modules_: std::array<SwerveModule, 4>                     │
│ - ik_solver_: std::unique_ptr<InverseKinematics>           │
│ - velocity_governor_: std::unique_ptr<VelocityGovernor>    │
│ - odom_publisher_: std::unique_ptr<OdometryPublisher>      │
├─────────────────────────────────────────────────────────────┤
│ + configure(...)                                            │
│ + activate()                                                │
│ + deactivate()                                              │
│ + computeVelocityCommands(...): TwistStamped               │
│ + setPlan(const Path&)                                      │
│ - publishWheelCommands(...)                                 │
│ - updateOdometry(...)                                       │
└─────────────────────────────────────────────────────────────┘
                        │ has-a (4×)
                        ↓
┌─────────────────────────────────────────────────────────────┐
│              SwerveModule                                   │
├─────────────────────────────────────────────────────────────┤
│ - module_id_: ModuleID (FL, FR, RL, RR)                    │
│ - position_: Eigen::Vector2d                                │
│ - current_drive_velocity_: double                           │
│ - current_steering_angle_: double                           │
│ - target_drive_velocity_: double                            │
│ - target_steering_angle_: double                            │
│ - aligned_: bool                                            │
├─────────────────────────────────────────────────────────────┤
│ + setTarget(double drive_vel, double steer_angle)          │
│ + updateState(double drive_vel, double steer_angle)        │
│ + isAligned(double tolerance): bool                        │
│ + optimizeSteeringAngle(): void                             │
│ + getModuleState(): ModuleState                             │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│              InverseKinematics                              │
├─────────────────────────────────────────────────────────────┤
│ - wheel_base_: double                                       │
│ - track_width_: double                                      │
│ - module_positions_: std::array<Eigen::Vector2d, 4>        │
├─────────────────────────────────────────────────────────────┤
│ + computeModuleStates(Twist): array<ModuleState, 4>        │
│ + computeOdometry(array<ModuleState, 4>): Twist            │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│              VelocityGovernor                               │
├─────────────────────────────────────────────────────────────┤
│ - max_linear_velocity_: double                              │
│ - max_angular_velocity_: double                             │
│ - max_acceleration_: double                                 │
│ - passenger_onboard_: bool                                  │
│ - previous_command_: Twist                                  │
├─────────────────────────────────────────────────────────────┤
│ + limitVelocity(Twist): Twist                               │
│ + setPassengerOnboard(bool)                                 │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. 詳細クラス仕様

### 3.1 SwerveDriveController

**ヘッダー (swerve_drive_controller.hpp):**
```cpp
#ifndef SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_
#define SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <array>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "custom_msgs/msg/swerve_module_command.hpp"

namespace swerve_drive_controller
{

enum class ModuleID {
  FRONT_LEFT = 0,
  FRONT_RIGHT = 1,
  REAR_LEFT = 2,
  REAR_RIGHT = 3
};

struct ModuleState {
  double drive_velocity;      // m/s
  double steering_angle;      // radians
  double drive_direction;     // +1 or -1 (optimization)
};

class SwerveModule {
public:
  SwerveModule(ModuleID id, const Eigen::Vector2d& position);

  void setTarget(double drive_vel, double steer_angle);
  void updateState(double drive_vel, double steer_angle);
  bool isAligned(double tolerance) const;
  void optimizeSteeringAngle();
  ModuleState getModuleState() const;

private:
  ModuleID module_id_;
  Eigen::Vector2d position_;
  double current_drive_velocity_{0.0};
  double current_steering_angle_{0.0};
  double target_drive_velocity_{0.0};
  double target_steering_angle_{0.0};
  double drive_direction_{1.0};
  bool aligned_{true};
};

class InverseKinematics {
public:
  InverseKinematics(double wheel_base, double track_width);

  std::array<ModuleState, 4> computeModuleStates(
    const geometry_msgs::msg::Twist& body_velocity) const;

  geometry_msgs::msg::Twist computeOdometry(
    const std::array<ModuleState, 4>& module_states) const;

private:
  double wheel_base_;
  double track_width_;
  std::array<Eigen::Vector2d, 4> module_positions_;
};

class VelocityGovernor {
public:
  VelocityGovernor(
    double max_linear_vel,
    double max_angular_vel,
    double max_accel);

  geometry_msgs::msg::Twist limitVelocity(
    const geometry_msgs::msg::Twist& cmd_vel);

  void setPassengerOnboard(bool onboard);

private:
  double max_linear_velocity_;
  double max_angular_velocity_;
  double max_acceleration_;
  bool passenger_onboard_{false};
  geometry_msgs::msg::Twist previous_command_;
  rclcpp::Time previous_time_;
};

class SwerveDriveController : public nav2_core::Controller {
public:
  SwerveDriveController() = default;
  ~SwerveDriveController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& velocity,
    nav2_core::GoalChecker* goal_checker) override;

  void setPlan(const nav_msgs::msg::Path& path) override;
  void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

protected:
  void publishWheelCommands(const std::array<ModuleState, 4>& module_states);
  void updateOdometry(const std::array<ModuleState, 4>& module_states);
  bool areModulesAligned(double tolerance) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // 設定パラメータ
  double wheel_base_;
  double track_width_;
  double wheel_radius_;
  double max_linear_velocity_;
  double max_angular_velocity_;
  double max_acceleration_;
  double angle_alignment_tolerance_;

  // コンポーネント
  std::array<SwerveModule, 4> modules_;
  std::unique_ptr<InverseKinematics> ik_solver_;
  std::unique_ptr<VelocityGovernor> velocity_governor_;

  // ROSパブリッシャー/サブスクライバー
  rclcpp::Publisher<custom_msgs::msg::SwerveModuleCommand>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr passenger_sub_;

  // 現在のパス
  nav_msgs::msg::Path current_path_;
};

}  // namespace swerve_drive_controller

#endif  // SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_
```

---

### 3.2 主要メソッド実装

**computeVelocityCommands (DWBスタイルローカルプランナー統合):**
```cpp
geometry_msgs::msg::TwistStamped SwerveDriveController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped& pose,
  const geometry_msgs::msg::Twist& velocity,
  nav2_core::GoalChecker* goal_checker)
{
  auto node = node_.lock();

  // 1. DWB軌道プランナー出力を取得 (DWBから提供される)
  geometry_msgs::msg::Twist raw_cmd_vel = computeLocalTrajectory(pose, velocity);

  // 2. 速度制限を適用 (安全性 + 乗客快適性)
  geometry_msgs::msg::Twist limited_cmd_vel = velocity_governor_->limitVelocity(raw_cmd_vel);

  // 3. 逆運動学を計算 (車体速度 → モジュール状態)
  std::array<ModuleState, 4> target_module_states =
    ik_solver_->computeModuleStates(limited_cmd_vel);

  // 4. 全モジュールの目標状態を設定
  for (size_t i = 0; i < 4; ++i) {
    modules_[i].setTarget(
      target_module_states[i].drive_velocity,
      target_module_states[i].steering_angle
    );
    modules_[i].optimizeSteeringAngle();  // 90°以上の回転の場合は反転
  }

  // 5. 全モジュールがアライメントされているか確認 (許容範囲内)
  bool all_aligned = areModulesAligned(angle_alignment_tolerance_);

  // 6. ホイールコマンドをパブリッシュ (アライメントされていない場合は駆動速度=0)
  std::array<ModuleState, 4> command_states;
  for (size_t i = 0; i < 4; ++i) {
    auto state = modules_[i].getModuleState();
    if (!all_aligned) {
      state.drive_velocity = 0.0;  // ステアリング中は駆動を減速
    }
    command_states[i] = state;
  }
  publishWheelCommands(command_states);

  // 7. オドメトリを更新 (順運動学)
  updateOdometry(command_states);

  // 8. Nav2用のコマンド速度を返す (可視化用)
  geometry_msgs::msg::TwistStamped stamped_cmd_vel;
  stamped_cmd_vel.header.stamp = node->now();
  stamped_cmd_vel.header.frame_id = "base_link";
  stamped_cmd_vel.twist = limited_cmd_vel;

  return stamped_cmd_vel;
}
```

**InverseKinematics::computeModuleStates:**
```cpp
std::array<ModuleState, 4> InverseKinematics::computeModuleStates(
  const geometry_msgs::msg::Twist& body_velocity) const
{
  std::array<ModuleState, 4> module_states;

  double vx = body_velocity.linear.x;
  double vy = body_velocity.linear.y;
  double omega = body_velocity.angular.z;

  for (size_t i = 0; i < 4; ++i) {
    double xi = module_positions_[i].x();
    double yi = module_positions_[i].y();

    // 並進と回転からのホイール速度寄与
    double vxi = vx - omega * yi;
    double vyi = vy + omega * xi;

    // モジュール速度 (大きさ) と方向
    double speed = std::sqrt(vxi * vxi + vyi * vyi);
    double angle = std::atan2(vyi, vxi);

    module_states[i].drive_velocity = speed;
    module_states[i].steering_angle = angle;
    module_states[i].drive_direction = 1.0;
  }

  return module_states;
}
```

**SwerveModule::optimizeSteeringAngle (反転最適化):**
```cpp
void SwerveModule::optimizeSteeringAngle() {
  // 目標角度への最短回転を計算
  double angle_error = target_steering_angle_ - current_steering_angle_;

  // [-π, π]に正規化
  while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
  while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

  // 回転が90°を超える場合、ステアリングを反転して駆動を逆転
  if (std::abs(angle_error) > M_PI / 2.0) {
    target_steering_angle_ += M_PI;  // 180°反転

    // 再度正規化
    while (target_steering_angle_ > M_PI) target_steering_angle_ -= 2.0 * M_PI;
    while (target_steering_angle_ < -M_PI) target_steering_angle_ += 2.0 * M_PI;

    drive_direction_ = -1.0;  // 駆動方向を逆転
  } else {
    drive_direction_ = 1.0;
  }
}
```

---

## 4. シーケンス図

### 4.1 通常動作シーケンス

```
Nav2 BT                SwerveDrive           IK Solver         Modules (×4)       Hardware
Navigator              Controller                                                 Controllers
   │                       │                     │                 │                  │
   │ computeVelocityCommands(pose, vel, goal)   │                 │                  │
   │──────────────────────>│                     │                 │                  │
   │                       │                     │                 │                  │
   │                       │ computeModuleStates(twist)            │                  │
   │                       │────────────────────>│                 │                  │
   │                       │                     │                 │                  │
   │                       │<────────────────────│                 │                  │
   │                       │ module_states[4]    │                 │                  │
   │                       │                     │                 │                  │
   │                       │ setTarget(vel, angle)                 │                  │
   │                       │─────────────────────────────────────>│                  │
   │                       │                     │                 │                  │
   │                       │ optimizeSteeringAngle()               │                  │
   │                       │─────────────────────────────────────>│                  │
   │                       │                     │                 │                  │
   │                       │ areModulesAligned()│                 │                  │
   │                       │─────────────────────────────────────>│                  │
   │                       │<─────────────────────────────────────│                  │
   │                       │ all_aligned=true    │                 │                  │
   │                       │                     │                 │                  │
   │                       │ publishWheelCommands()                │                  │
   │                       │────────────────────────────────────────────────────────>│
   │                       │                     │                 │                  │
   │<──────────────────────│                     │                 │                  │
   │ TwistStamped          │                     │                 │                  │
   │                       │                     │                 │                  │
```

### 4.2 モジュールアライメント待機シーケンス

```
SwerveDrive            Modules (×4)          Hardware
Controller                                   Controllers
   │                       │                     │
   │ setTarget(vel, angle) │                     │
   │──────────────────────>│                     │
   │                       │                     │
   │ areModulesAligned()   │                     │
   │──────────────────────>│                     │
   │<──────────────────────│                     │
   │ all_aligned=false     │                     │
   │                       │                     │
   │ publishWheelCommands()│                     │
   │ (drive_vel = 0)       │                     │
   │────────────────────────────────────────────>│
   │                       │                     │
   │ [wait 50ms]           │                     │
   │                       │                     │
   │ areModulesAligned()   │                     │
   │──────────────────────>│                     │
   │<──────────────────────│                     │
   │ all_aligned=true      │                     │
   │                       │                     │
   │ publishWheelCommands()│                     │
   │ (drive_vel = target)  │                     │
   │────────────────────────────────────────────>│
   │                       │                     │
```

---

## 5. 状態機械 (モジュールアライメント)

```
┌─────────────────────────────────────────────────────────────┐
│           モジュールアライメント状態機械                    │
└─────────────────────────────────────────────────────────────┘

   ┌──────────┐
   │  IDLE    │
   └────┬─────┘
        │ 新規目標受信
        ↓
   ┌──────────┐
   │ALIGNING  │ ← ステアリングモーターが目標角度に移動中
   └────┬─────┘   駆動速度 = 0
        │ 全モジュールが許容範囲内
        ↓
   ┌──────────┐
   │ ALIGNED  │ ← 全モジュールが目標角度
   └────┬─────┘   駆動速度 = 目標値
        │ 新規目標受信
        ↓
   ┌──────────┐
   │ALIGNING  │
   └──────────┘
```

**アライメントチェックロジック:**
```cpp
bool SwerveDriveController::areModulesAligned(double tolerance) const {
  for (const auto& module : modules_) {
    if (!module.isAligned(tolerance)) {
      return false;
    }
  }
  return true;
}

bool SwerveModule::isAligned(double tolerance) const {
  double angle_error = std::abs(target_steering_angle_ - current_steering_angle_);
  return angle_error < tolerance;
}
```

---

## 6. 設定パラメータ

**nav2_params.yaml (抜粋):**
```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "swerve_drive_controller::SwerveDriveController"

      # ロボット幾何学
      wheel_base: 0.6                 # メートル (L)
      track_width: 0.5                # メートル (W)
      wheel_radius: 0.0825            # メートル (6.5インチホイール)

      # 速度制限
      max_linear_velocity: 1.5        # m/s (乗客なし)
      max_linear_velocity_passenger: 1.0  # m/s (乗客あり)
      max_angular_velocity: 2.0       # rad/s
      max_linear_acceleration: 0.5    # m/s²
      max_angular_acceleration: 1.0   # rad/s²

      # モジュールアライメント
      angle_alignment_tolerance: 0.05 # ラジアン (~3°)

      # DWB軌道パラメータ
      sim_time: 1.7                   # 秒
      vx_samples: 20
      vy_samples: 20                  # 全方向移動!
      vtheta_samples: 40

      # 軌道スコアリング
      path_distance_bias: 32.0
      goal_distance_bias: 24.0
      occdist_scale: 0.01

      # トピック
      wheel_command_topic: "/wheel_commands"
      odom_topic: "/odom"
      joint_states_topic: "/joint_states"
      passenger_topic: "/passenger_attached"
```

---

## 7. データ構造

**custom_msgs/SwerveModuleCommand.msg:**
```
# 単一スワーブモジュールのコマンド
uint8 module_id           # 0=FL, 1=FR, 2=RL, 3=RR
float64 drive_velocity    # m/s (符号付き、正=前進)
float64 steering_angle    # ラジアン [-π, π]
---
# フィードバック (このメッセージタイプでは未使用)
---
# 結果 (このメッセージタイプでは未使用)
```

**モジュール位置初期化:**
```cpp
void SwerveDriveController::configure(...) {
  // パラメータをロード
  wheel_base_ = 0.6;
  track_width_ = 0.5;

  // モジュール位置を初期化
  double L = wheel_base_;
  double W = track_width_;

  modules_[static_cast<size_t>(ModuleID::FRONT_LEFT)] =
    SwerveModule(ModuleID::FRONT_LEFT, Eigen::Vector2d(L/2, W/2));

  modules_[static_cast<size_t>(ModuleID::FRONT_RIGHT)] =
    SwerveModule(ModuleID::FRONT_RIGHT, Eigen::Vector2d(L/2, -W/2));

  modules_[static_cast<size_t>(ModuleID::REAR_LEFT)] =
    SwerveModule(ModuleID::REAR_LEFT, Eigen::Vector2d(-L/2, W/2));

  modules_[static_cast<size_t>(ModuleID::REAR_RIGHT)] =
    SwerveModule(ModuleID::REAR_RIGHT, Eigen::Vector2d(-L/2, -W/2));

  // IKソルバーを初期化
  ik_solver_ = std::make_unique<InverseKinematics>(wheel_base_, track_width_);

  // 速度ガバナーを初期化
  velocity_governor_ = std::make_unique<VelocityGovernor>(
    max_linear_velocity_,
    max_angular_velocity_,
    max_acceleration_
  );
}
```

---

## 8. エラーハンドリング

**モジュールエンコーダ故障:**
```cpp
void SwerveDriveController::jointStateCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // エンコーダフィードバックからモジュール状態を更新
  for (size_t i = 0; i < 4; ++i) {
    // エンコーダデータが有効か確認
    if (msg->position.size() < i*2 + 2 || msg->velocity.size() < i*2 + 2) {
      RCLCPP_ERROR(logger_, "Invalid joint state message (missing encoder data)");
      // 安全停止をトリガー
      triggerSafeStop("Encoder failure");
      return;
    }

    double drive_vel = msg->velocity[i*2];
    double steer_angle = msg->position[i*2 + 1];

    // 妥当性チェック
    if (std::abs(drive_vel) > max_linear_velocity_ * 2.0) {
      RCLCPP_WARN(logger_, "Module %zu: Unrealistic drive velocity %.2f", i, drive_vel);
      drive_vel = 0.0;  // 不正な読み取りを無視
    }

    modules_[i].updateState(drive_vel, steer_angle);
  }
}
```

**モジュールタイムアウト (エンコーダ更新を受信していない):**
```cpp
void SwerveDriveController::checkModuleTimeout() {
  auto now = node_->now();

  for (size_t i = 0; i < 4; ++i) {
    auto elapsed = (now - modules_[i].last_update_time).seconds();

    if (elapsed > module_timeout_threshold_) {
      RCLCPP_ERROR(logger_, "Module %zu timeout (%.2fs)", i, elapsed);
      triggerSafeStop("Module timeout");
      return;
    }
  }
}
```

---

## 9. テスト戦略

### 9.1 単体テスト

**テスト: 逆運動学の正確性**
```cpp
TEST(InverseKinematicsTest, PureTranslationX) {
  InverseKinematics ik(0.6, 0.5);

  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1.0;  // 前進1 m/s
  twist.linear.y = 0.0;
  twist.angular.z = 0.0;

  auto states = ik.computeModuleStates(twist);

  // 全モジュールが同じ速度と角度を持つべき
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(states[i].drive_velocity, 1.0, 1e-6);
    EXPECT_NEAR(states[i].steering_angle, 0.0, 1e-6);  // 前進 = 0°
  }
}

TEST(InverseKinematicsTest, PureRotation) {
  InverseKinematics ik(0.6, 0.5);

  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.angular.z = 1.0;  // 回転1 rad/s

  auto states = ik.computeModuleStates(twist);

  // 全モジュールがロボット中心周りを回転すべき
  // 前左と後右は同じ方向を向く (円の接線)
  // 前右と後左は反対方向を向く
  EXPECT_NEAR(states[0].steering_angle, M_PI/4, 0.01);    // FL: 45°
  EXPECT_NEAR(states[1].steering_angle, 3*M_PI/4, 0.01);  // FR: 135°
  EXPECT_NEAR(states[2].steering_angle, -3*M_PI/4, 0.01); // RL: -135°
  EXPECT_NEAR(states[3].steering_angle, -M_PI/4, 0.01);   // RR: -45°
}
```

**テスト: モジュール反転最適化**
```cpp
TEST(SwerveModuleTest, FlipOptimization) {
  SwerveModule module(ModuleID::FRONT_LEFT, Eigen::Vector2d(0.3, 0.25));

  // 現在の角度: 0° (前進)
  module.updateState(1.0, 0.0);

  // 目標角度: 170° (ほぼ後退)
  module.setTarget(1.0, 170.0 * M_PI / 180.0);
  module.optimizeSteeringAngle();

  // -10°に反転して駆動を逆転すべき
  auto state = module.getModuleState();
  EXPECT_NEAR(state.steering_angle, -10.0 * M_PI / 180.0, 0.01);
  EXPECT_NEAR(state.drive_direction, -1.0, 1e-6);
}
```

---

### 9.2 統合テスト

**テスト: 完全制御ループ (Gazeboシミュレーション)**
```cpp
TEST_F(SwerveDriveControllerIntegrationTest, StraightLineMotion) {
  // コントローラを初期化
  auto controller = std::make_shared<SwerveDriveController>();
  controller->configure(node_, "test_controller", tf_, costmap_);
  controller->activate();

  // 直線パスを設定
  nav_msgs::msg::Path path;
  // ... (10m直線パスを作成)
  controller->setPlan(path);

  // 10秒間制御ループをシミュレート
  rclcpp::Rate rate(20);  // 20 Hz
  for (int i = 0; i < 200; ++i) {
    auto pose = getCurrentPose();
    auto velocity = getCurrentVelocity();

    auto cmd_vel = controller->computeVelocityCommands(pose, velocity, goal_checker_);

    // コマンド速度が前進であることを確認
    EXPECT_GT(cmd_vel.twist.linear.x, 0.0);
    EXPECT_NEAR(cmd_vel.twist.linear.y, 0.0, 0.1);
    EXPECT_NEAR(cmd_vel.twist.angular.z, 0.0, 0.1);

    rate.sleep();
  }

  // ロボットがゴールに到達したことを確認
  auto final_pose = getCurrentPose();
  EXPECT_NEAR(final_pose.pose.position.x, 10.0, 0.1);
}
```

---

## 10. パフォーマンス最適化

**計算複雑度:**
- 逆運動学: O(1) - 4モジュール、固定計算
- モジュールアライメントチェック: O(1) - 4比較
- 全体制御ループ: GMKtec Nucbox K6で20 Hz @ <5ms

**メモリ使用量:**
- ModuleState: 3 double × 4 modules = 96 bytes
- 設定パラメータ: ~200 bytes
- 合計: ~1 KB (無視できる)

**最適化のヒント:**
- ベクトル化数学演算にEigenを使用
- configure()でモジュール位置を事前計算
- 可能な限り三角関数をキャッシュ
- 小さな関数をインライン化 (isAligned など)

---

**ドキュメントステータス:** ドラフト
**実装ステータス:** 開発準備完了
**必要な承認:** ナビゲーションリード、スワーブドライブエンジニア、ソフトウェアアーキテクト
