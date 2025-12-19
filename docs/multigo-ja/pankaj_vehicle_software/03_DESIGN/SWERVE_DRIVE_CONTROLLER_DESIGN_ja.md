# スワーブドライブコントローラー詳細設計

**文書ID:** DESIGN-SWERVE-001
**バージョン:** 1.0
**日付:** 2025-12-15
**ステータス:** ドラフト

---

## 1. 概要

この文書は、クラス構造、アルゴリズム、データフロー、実装詳細を含むスワーブドライブコントローラーの詳細設計仕様を提供します。

**主要コンポーネント:**
- SwerveDriveController（Nav2プラグイン）
- SwerveModule（FL、FR、RL、RR用の4インスタンス）
- InverseKinematicsソルバー
- VelocityGovernor
- OdometryPublisher

---

## 2. クラス図

```
┌─────────────────────────────────────────────────────────────┐
│                  nav2_core::Controller                      │
│                  (Interface)                                │
└───────────────────────┬─────────────────────────────────────┘
                        │ inherits
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

  // Configuration parameters
  double wheel_base_;
  double track_width_;
  double wheel_radius_;
  double max_linear_velocity_;
  double max_angular_velocity_;
  double max_acceleration_;
  double angle_alignment_tolerance_;

  // Components
  std::array<SwerveModule, 4> modules_;
  std::unique_ptr<InverseKinematics> ik_solver_;
  std::unique_ptr<VelocityGovernor> velocity_governor_;

  // ROS publishers/subscribers
  rclcpp::Publisher<custom_msgs::msg::SwerveModuleCommand>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr passenger_sub_;

  // Current path
  nav_msgs::msg::Path current_path_;
};

}  // namespace swerve_drive_controller

#endif  // SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_
```

---

### 3.2 主要メソッド実装

**computeVelocityCommands（DWBスタイルローカルプランナー統合）:**
```cpp
geometry_msgs::msg::TwistStamped SwerveDriveController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped& pose,
  const geometry_msgs::msg::Twist& velocity,
  nav2_core::GoalChecker* goal_checker)
{
  auto node = node_.lock();

  // 1. Get DWB trajectory planner output (this would come from DWB)
  geometry_msgs::msg::Twist raw_cmd_vel = computeLocalTrajectory(pose, velocity);

  // 2. Apply velocity limiting (safety + passenger comfort)
  geometry_msgs::msg::Twist limited_cmd_vel = velocity_governor_->limitVelocity(raw_cmd_vel);

  // 3. Compute inverse kinematics (body velocity → module states)
  std::array<ModuleState, 4> target_module_states =
    ik_solver_->computeModuleStates(limited_cmd_vel);

  // 4. Set target states for all modules
  for (size_t i = 0; i < 4; ++i) {
    modules_[i].setTarget(
      target_module_states[i].drive_velocity,
      target_module_states[i].steering_angle
    );
    modules_[i].optimizeSteeringAngle();  // Flip if >90° rotation
  }

  // 5. Check if all modules are aligned (within tolerance)
  bool all_aligned = areModulesAligned(angle_alignment_tolerance_);

  // 6. Publish wheel commands (drive velocity = 0 if not aligned)
  std::array<ModuleState, 4> command_states;
  for (size_t i = 0; i < 4; ++i) {
    auto state = modules_[i].getModuleState();
    if (!all_aligned) {
      state.drive_velocity = 0.0;  // Ramp down drive while steering
    }
    command_states[i] = state;
  }
  publishWheelCommands(command_states);

  // 7. Update odometry (forward kinematics)
  updateOdometry(command_states);

  // 8. Return commanded velocity for Nav2 (for visualization)
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

    // Wheel velocity contribution from translation and rotation
    double vxi = vx - omega * yi;
    double vyi = vy + omega * xi;

    // Module speed (magnitude) and direction
    double speed = std::sqrt(vxi * vxi + vyi * vyi);
    double angle = std::atan2(vyi, vxi);

    module_states[i].drive_velocity = speed;
    module_states[i].steering_angle = angle;
    module_states[i].drive_direction = 1.0;
  }

  return module_states;
}
```

**SwerveModule::optimizeSteeringAngle（フリップ最適化）:**
```cpp
void SwerveModule::optimizeSteeringAngle() {
  // Compute shortest rotation to target angle
  double angle_error = target_steering_angle_ - current_steering_angle_;

  // Normalize to [-π, π]
  while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
  while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

  // If rotation >90°, flip steering and reverse drive
  if (std::abs(angle_error) > M_PI / 2.0) {
    target_steering_angle_ += M_PI;  // Flip 180°

    // Normalize again
    while (target_steering_angle_ > M_PI) target_steering_angle_ -= 2.0 * M_PI;
    while (target_steering_angle_ < -M_PI) target_steering_angle_ += 2.0 * M_PI;

    drive_direction_ = -1.0;  // Reverse drive direction
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

## 5. ステートマシン（モジュールアライメント）

```
┌─────────────────────────────────────────────────────────────┐
│             Module Alignment State Machine                  │
└─────────────────────────────────────────────────────────────┘

   ┌──────────┐
   │  IDLE    │
   └────┬─────┘
        │ new target received
        ↓
   ┌──────────┐
   │ALIGNING  │ ← Steering motors moving to target angle
   └────┬─────┘   Drive velocity = 0
        │ all modules within tolerance
        ↓
   ┌──────────┐
   │ ALIGNED  │ ← All modules at target angle
   └────┬─────┘   Drive velocity = target
        │ new target received
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

## 6. 設定パラメーター

**nav2_params.yaml（抜粋）:**
```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "swerve_drive_controller::SwerveDriveController"

      # Robot geometry
      wheel_base: 0.6                 # meters (L)
      track_width: 0.5                # meters (W)
      wheel_radius: 0.0825            # meters (6.5" wheels)

      # Velocity limits
      max_linear_velocity: 1.5        # m/s (no passenger)
      max_linear_velocity_passenger: 1.0  # m/s (with passenger)
      max_angular_velocity: 2.0       # rad/s
      max_linear_acceleration: 0.5    # m/s²
      max_angular_acceleration: 1.0   # rad/s²

      # Module alignment
      angle_alignment_tolerance: 0.05 # radians (~3°)

      # DWB trajectory parameters
      sim_time: 1.7                   # seconds
      vx_samples: 20
      vy_samples: 20                  # Omnidirectional!
      vtheta_samples: 40

      # Trajectory scoring
      path_distance_bias: 32.0
      goal_distance_bias: 24.0
      occdist_scale: 0.01

      # Topics
      wheel_command_topic: "/wheel_commands"
      odom_topic: "/odom"
      joint_states_topic: "/joint_states"
      passenger_topic: "/passenger_attached"
```

---

## 7. データ構造

**custom_msgs/SwerveModuleCommand.msg:**
```
# Command for a single swerve module
uint8 module_id           # 0=FL, 1=FR, 2=RL, 3=RR
float64 drive_velocity    # m/s (signed, positive=forward)
float64 steering_angle    # radians [-π, π]
---
# Feedback (not used in this message type)
---
# Result (not used in this message type)
```

**モジュール位置の初期化:**
```cpp
void SwerveDriveController::configure(...) {
  // Load parameters
  wheel_base_ = 0.6;
  track_width_ = 0.5;

  // Initialize module positions
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

  // Initialize IK solver
  ik_solver_ = std::make_unique<InverseKinematics>(wheel_base_, track_width_);

  // Initialize velocity governor
  velocity_governor_ = std::make_unique<VelocityGovernor>(
    max_linear_velocity_,
    max_angular_velocity_,
    max_acceleration_
  );
}
```

---

## 8. エラーハンドリング

**モジュールエンコーダー障害:**
```cpp
void SwerveDriveController::jointStateCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Update module states from encoder feedback
  for (size_t i = 0; i < 4; ++i) {
    // Check if encoder data is valid
    if (msg->position.size() < i*2 + 2 || msg->velocity.size() < i*2 + 2) {
      RCLCPP_ERROR(logger_, "Invalid joint state message (missing encoder data)");
      // Trigger safe stop
      triggerSafeStop("Encoder failure");
      return;
    }

    double drive_vel = msg->velocity[i*2];
    double steer_angle = msg->position[i*2 + 1];

    // Sanity check
    if (std::abs(drive_vel) > max_linear_velocity_ * 2.0) {
      RCLCPP_WARN(logger_, "Module %zu: Unrealistic drive velocity %.2f", i, drive_vel);
      drive_vel = 0.0;  // Ignore bad reading
    }

    modules_[i].updateState(drive_vel, steer_angle);
  }
}
```

**モジュールタイムアウト（エンコーダー更新を受信していない）:**
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

### 9.1 ユニットテスト

**テスト: 逆運動学の正しさ**
```cpp
TEST(InverseKinematicsTest, PureTranslationX) {
  InverseKinematics ik(0.6, 0.5);

  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1.0;  // 1 m/s forward
  twist.linear.y = 0.0;
  twist.angular.z = 0.0;

  auto states = ik.computeModuleStates(twist);

  // All modules should have same velocity and angle
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(states[i].drive_velocity, 1.0, 1e-6);
    EXPECT_NEAR(states[i].steering_angle, 0.0, 1e-6);  // Forward = 0°
  }
}

TEST(InverseKinematicsTest, PureRotation) {
  InverseKinematics ik(0.6, 0.5);

  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.angular.z = 1.0;  // 1 rad/s rotation

  auto states = ik.computeModuleStates(twist);

  // All modules should rotate around robot center
  // Front-left and rear-right should point same direction (tangent to circle)
  // Front-right and rear-left should point opposite direction
  EXPECT_NEAR(states[0].steering_angle, M_PI/4, 0.01);    // FL: 45°
  EXPECT_NEAR(states[1].steering_angle, 3*M_PI/4, 0.01);  // FR: 135°
  EXPECT_NEAR(states[2].steering_angle, -3*M_PI/4, 0.01); // RL: -135°
  EXPECT_NEAR(states[3].steering_angle, -M_PI/4, 0.01);   // RR: -45°
}
```

**テスト: モジュールフリップ最適化**
```cpp
TEST(SwerveModuleTest, FlipOptimization) {
  SwerveModule module(ModuleID::FRONT_LEFT, Eigen::Vector2d(0.3, 0.25));

  // Current angle: 0° (forward)
  module.updateState(1.0, 0.0);

  // Target angle: 170° (almost backward)
  module.setTarget(1.0, 170.0 * M_PI / 180.0);
  module.optimizeSteeringAngle();

  // Should flip to -10° and reverse drive
  auto state = module.getModuleState();
  EXPECT_NEAR(state.steering_angle, -10.0 * M_PI / 180.0, 0.01);
  EXPECT_NEAR(state.drive_direction, -1.0, 1e-6);
}
```

---

### 9.2 統合テスト

**テスト: 完全な制御ループ（Gazeboシミュレーション）**
```cpp
TEST_F(SwerveDriveControllerIntegrationTest, StraightLineMotion) {
  // Initialize controller
  auto controller = std::make_shared<SwerveDriveController>();
  controller->configure(node_, "test_controller", tf_, costmap_);
  controller->activate();

  // Set straight path
  nav_msgs::msg::Path path;
  // ... (create 10m straight path)
  controller->setPlan(path);

  // Simulate control loop for 10 seconds
  rclcpp::Rate rate(20);  // 20 Hz
  for (int i = 0; i < 200; ++i) {
    auto pose = getCurrentPose();
    auto velocity = getCurrentVelocity();

    auto cmd_vel = controller->computeVelocityCommands(pose, velocity, goal_checker_);

    // Verify commanded velocity is forward
    EXPECT_GT(cmd_vel.twist.linear.x, 0.0);
    EXPECT_NEAR(cmd_vel.twist.linear.y, 0.0, 0.1);
    EXPECT_NEAR(cmd_vel.twist.angular.z, 0.0, 0.1);

    rate.sleep();
  }

  // Verify robot reached goal
  auto final_pose = getCurrentPose();
  EXPECT_NEAR(final_pose.pose.position.x, 10.0, 0.1);
}
```

---

## 10. パフォーマンス最適化

**計算複雑性:**
- 逆運動学: O(1) - 4モジュール、固定計算
- モジュールアライメントチェック: O(1) - 4回比較
- 全体の制御ループ: GMKtec Nucbox K6で<5ms @ 20 Hz

**メモリ使用量:**
- ModuleState: 3 double × 4モジュール = 96バイト
- 設定パラメーター: ~200バイト
- 合計: ~1 KB（無視できる程度）

**最適化のヒント:**
- ベクトル化された数学演算にEigenを使用
- configure()でモジュール位置を事前計算
- 可能な場合は三角関数をキャッシュ
- 小さな関数をインライン化（isAlignedなど）

---

**文書ステータス:** ドラフト
**実装ステータス:** 開発準備完了
**必要な承認:** ナビゲーションリード、スワーブドライブエンジニア、ソフトウェアアーキテクト
