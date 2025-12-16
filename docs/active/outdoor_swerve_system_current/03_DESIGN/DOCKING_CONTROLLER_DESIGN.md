# Docking Controller Detailed Design

**Document ID:** DESIGN-DOCKING-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft

---

## 1. Overview

This document provides detailed design specifications for the Autonomous Docking Controller, which enables precision wheelchair attachment using ArUco marker-based visual servoing.

**Key Components:**
- DockingController (ROS 2 action server)
- ArucoDetector (dual camera fusion)
- VisualServoController (PBVS)
- DockingStateMachine (Dezyne formal model)
- SafetyMonitor (collision detection during docking)

**Docking Precision Target:**
- Outdoor: ±2-5mm position, ±2° orientation
- Indoor (future): ±1mm position, ±0.5° orientation

---

## 2. Class Diagram

```
┌─────────────────────────────────────────────────────────────┐
│              rclcpp_action::ServerBase                      │
│              (ROS 2 Action Interface)                       │
└───────────────────────┬─────────────────────────────────────┘
                        │ inherits
                        ↓
┌─────────────────────────────────────────────────────────────┐
│              DockingController                              │
│              (Main Coordinator)                             │
├─────────────────────────────────────────────────────────────┤
│ - aruco_detector_: unique_ptr<ArucoDetector>               │
│ - servo_controller_: unique_ptr<VisualServoController>     │
│ - state_machine_: unique_ptr<DockingStateMachine>          │
│ - safety_monitor_: unique_ptr<SafetyMonitor>               │
│ - docking_params_: DockingParameters                        │
├─────────────────────────────────────────────────────────────┤
│ + configure(...)                                            │
│ + activate()                                                │
│ + handleGoal(...): GoalResponse                             │
│ + handleCancel(...): CancelResponse                         │
│ + handleAccepted(...)                                       │
│ - executeDocking(goal_handle)                               │
│ - publishFeedback(...)                                      │
└─────────────────────────────────────────────────────────────┘
                        │ has-a
                        ↓
┌─────────────────────────────────────────────────────────────┐
│              ArucoDetector                                  │
│              (Dual Camera Fusion)                           │
├─────────────────────────────────────────────────────────────┤
│ - front_camera_sub_: Subscription<Image>                   │
│ - rear_camera_sub_: Subscription<Image>                    │
│ - aruco_dict_: cv::aruco::Dictionary                       │
│ - detector_params_: cv::aruco::DetectorParameters          │
│ - camera_matrix_: cv::Mat                                   │
│ - dist_coeffs_: cv::Mat                                     │
│ - marker_detections_: map<int, MarkerPose>                 │
├─────────────────────────────────────────────────────────────┤
│ + detectMarkers(image): vector<MarkerDetection>            │
│ + estimateMarkerPose(corners, marker_id): Pose             │
│ + fuseDetections(): DockingPose                             │
│ + getMarkerQuality(detection): double                      │
│ - applyOutdoorOptimizations(image): Mat                    │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│              VisualServoController                          │
│              (PBVS - Position-Based Visual Servoing)        │
├─────────────────────────────────────────────────────────────┤
│ - pid_x_: PIDController                                     │
│ - pid_y_: PIDController                                     │
│ - pid_theta_: PIDController                                 │
│ - current_marker_pose_: Pose                                │
│ - target_offset_: Pose                                      │
├─────────────────────────────────────────────────────────────┤
│ + computeVelocityCommand(marker_pose): Twist               │
│ + setTargetOffset(offset): void                            │
│ + isConverged(tolerance): bool                             │
│ + reset(): void                                             │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│              DockingStateMachine                            │
│              (Dezyne Formal Model)                          │
├─────────────────────────────────────────────────────────────┤
│ - current_state_: State                                     │
│ - retry_count_: int                                         │
├─────────────────────────────────────────────────────────────┤
│ + handleEvent(Event): State                                 │
│ + getCurrentState(): State                                  │
│ + reset()                                                   │
└─────────────────────────────────────────────────────────────┘

States:
  IDLE → APPROACH → SEARCH → SERVOING → FINE_ALIGN → DOCKED
                ↓             ↓           ↓
              FAILED      FAILED      FAILED

┌─────────────────────────────────────────────────────────────┐
│              SafetyMonitor                                  │
├─────────────────────────────────────────────────────────────┤
│ - collision_threshold_: double                              │
│ - max_approach_velocity_: double                            │
├─────────────────────────────────────────────────────────────┤
│ + checkSafety(cmd_vel, marker_pose): SafetyStatus         │
│ + limitVelocity(cmd_vel): Twist                            │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. Detailed Class Specifications

### 3.1 DockingController

**Header (docking_controller.hpp):**
```cpp
#ifndef DOCKING_CONTROLLER__DOCKING_CONTROLLER_HPP_
#define DOCKING_CONTROLLER__DOCKING_CONTROLLER_HPP_

#include <memory>
#include <map>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "custom_msgs/action/dock_wheelchair.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace docking_controller
{

enum class State {
  IDLE,
  APPROACH,      // Nav2 coarse approach to marker vicinity
  SEARCH,        // Rotate to find ArUco markers
  SERVOING,      // Visual servoing (fast approach)
  FINE_ALIGN,    // Precision alignment (<10cm)
  DOCKED,        // Docking complete
  FAILED         // Docking failed
};

enum class Event {
  START_DOCKING,
  APPROACH_COMPLETE,
  MARKER_DETECTED,
  MARKER_LOST,
  CONVERGENCE_ACHIEVED,
  TIMEOUT,
  COLLISION_DETECTED,
  MANUAL_ABORT
};

struct DockingParameters {
  // Marker configuration
  double marker_size;              // meters (e.g., 0.15m)
  std::vector<int> marker_ids;     // Expected marker IDs [0, 1, 2, 3]

  // Approach parameters
  double coarse_approach_distance; // meters (e.g., 2.0m)
  double search_angular_velocity;  // rad/s (e.g., 0.3)

  // Visual servoing parameters
  double servo_kp_x, servo_kp_y, servo_kp_theta;
  double servo_ki_x, servo_ki_y, servo_ki_theta;
  double servo_kd_x, servo_kd_y, servo_kd_theta;
  double max_servo_velocity;       // m/s (e.g., 0.1)

  // Convergence criteria
  double position_tolerance;       // meters (e.g., 0.005 = 5mm)
  double orientation_tolerance;    // radians (e.g., 0.035 = 2°)
  double convergence_time;         // seconds (e.g., 1.0)

  // Timeouts
  double search_timeout;           // seconds (e.g., 30.0)
  double servoing_timeout;         // seconds (e.g., 60.0)

  // Safety
  double min_marker_distance;      // meters (e.g., 0.05)
  int max_retry_attempts;          // (e.g., 3)
};

struct MarkerPose {
  int marker_id;
  Eigen::Vector3d position;        // [x, y, z] in camera frame
  Eigen::Quaterniond orientation;
  double quality_score;            // [0.0, 1.0]
  rclcpp::Time timestamp;
};

struct DockingPose {
  Eigen::Vector3d position;        // Robot position relative to docking target
  Eigen::Quaterniond orientation;
  double confidence;               // [0.0, 1.0]
  std::vector<int> visible_markers;
};

class DockingController : public rclcpp::Node {
public:
  using DockAction = custom_msgs::action::DockWheelchair;
  using GoalHandle = rclcpp_action::ServerGoalHandle<DockAction>;

  explicit DockingController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~DockingController() = default;

private:
  void configure();

  // Action server callbacks
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const DockAction::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

  // Main docking execution
  void executeDocking(const std::shared_ptr<GoalHandle> goal_handle);

  // State-specific handlers
  void stateApproach(const std::shared_ptr<GoalHandle> goal_handle);
  void stateSearch(const std::shared_ptr<GoalHandle> goal_handle);
  void stateServoing(const std::shared_ptr<GoalHandle> goal_handle);
  void stateFineAlign(const std::shared_ptr<GoalHandle> goal_handle);

  // Helper methods
  bool triggerNav2Approach(const geometry_msgs::msg::PoseStamped& target);
  void publishVelocityCommand(const geometry_msgs::msg::Twist& cmd_vel);
  void publishFeedback(const std::shared_ptr<GoalHandle>& goal_handle);
  void transitionState(State new_state);

  // Components
  std::unique_ptr<ArucoDetector> aruco_detector_;
  std::unique_ptr<VisualServoController> servo_controller_;
  std::unique_ptr<DockingStateMachine> state_machine_;
  std::unique_ptr<SafetyMonitor> safety_monitor_;

  // Parameters
  DockingParameters params_;

  // ROS interfaces
  rclcpp_action::Server<DockAction>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;

  // State
  State current_state_{State::IDLE};
  rclcpp::Time state_entry_time_;
  int retry_count_{0};
};

}  // namespace docking_controller

#endif  // DOCKING_CONTROLLER__DOCKING_CONTROLLER_HPP_
```

---

### 3.2 ArucoDetector

**Key Methods:**

**detectMarkers (with outdoor optimizations):**
```cpp
std::vector<MarkerDetection> ArucoDetector::detectMarkers(const cv::Mat& image) {
  cv::Mat processed = applyOutdoorOptimizations(image);

  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<int> marker_ids;

  // Detect ArUco markers
  cv::aruco::detectMarkers(
    processed,
    aruco_dict_,
    marker_corners,
    marker_ids,
    detector_params_
  );

  std::vector<MarkerDetection> detections;

  for (size_t i = 0; i < marker_ids.size(); ++i) {
    // Estimate 3D pose from corners
    cv::Vec3d rvec, tvec;
    cv::aruco::estimatePoseSingleMarkers(
      std::vector<std::vector<cv::Point2f>>{marker_corners[i]},
      params_.marker_size,
      camera_matrix_,
      dist_coeffs_,
      rvec,
      tvec
    );

    MarkerDetection detection;
    detection.marker_id = marker_ids[i];
    detection.position = Eigen::Vector3d(tvec[0], tvec[1], tvec[2]);

    // Convert rotation vector to quaternion
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    detection.orientation = rotationMatrixToQuaternion(rotation_matrix);

    // Quality score based on corner sharpness and reprojection error
    detection.quality_score = computeMarkerQuality(marker_corners[i], rvec, tvec);

    detections.push_back(detection);
  }

  return detections;
}

cv::Mat ArucoDetector::applyOutdoorOptimizations(const cv::Mat& image) {
  cv::Mat gray, enhanced;

  // Convert to grayscale
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

  // Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
  // CRITICAL for outdoor lighting variations
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(2.0);
  clahe->setTilesGridSize(cv::Size(8, 8));
  clahe->apply(gray, enhanced);

  // Optional: Gaussian blur to reduce noise
  cv::GaussianBlur(enhanced, enhanced, cv::Size(5, 5), 0);

  return enhanced;
}
```

**fuseDetections (dual camera fusion):**
```cpp
DockingPose ArucoDetector::fuseDetections() {
  // Collect all recent detections from both cameras
  std::vector<MarkerPose> all_detections;

  for (const auto& [marker_id, pose] : front_camera_detections_) {
    if ((node_->now() - pose.timestamp).seconds() < detection_timeout_) {
      all_detections.push_back(pose);
    }
  }

  for (const auto& [marker_id, pose] : rear_camera_detections_) {
    if ((node_->now() - pose.timestamp).seconds() < detection_timeout_) {
      all_detections.push_back(pose);
    }
  }

  if (all_detections.empty()) {
    return DockingPose{};  // No valid detections
  }

  // Weighted average based on quality scores
  Eigen::Vector3d weighted_position = Eigen::Vector3d::Zero();
  double total_weight = 0.0;

  for (const auto& detection : all_detections) {
    // Transform marker pose from camera frame to base_link
    geometry_msgs::msg::PoseStamped marker_in_camera;
    marker_in_camera.header.frame_id = detection.camera_frame;
    marker_in_camera.pose = eigenToRosPose(detection.position, detection.orientation);

    geometry_msgs::msg::PoseStamped marker_in_base;
    tf_buffer_->transform(marker_in_camera, marker_in_base, "base_link");

    Eigen::Vector3d pos = rosPoseToEigen(marker_in_base.pose).position;
    double weight = detection.quality_score;

    weighted_position += weight * pos;
    total_weight += weight;
  }

  DockingPose fused_pose;
  fused_pose.position = weighted_position / total_weight;
  fused_pose.confidence = std::min(1.0, total_weight / all_detections.size());

  // Compute docking target (offset from markers)
  fused_pose.position -= docking_offset_;

  return fused_pose;
}
```

---

### 3.3 VisualServoController

**PBVS Controller Implementation:**
```cpp
geometry_msgs::msg::Twist VisualServoController::computeVelocityCommand(
  const DockingPose& marker_pose)
{
  // Compute position error in base_link frame
  Eigen::Vector3d position_error = target_offset_ - marker_pose.position;

  // Compute orientation error
  Eigen::Quaterniond orientation_error =
    target_orientation_.inverse() * marker_pose.orientation;

  // Extract yaw error
  double yaw_error = extractYaw(orientation_error);

  // PID control for x, y, theta
  double cmd_vx = pid_x_.compute(position_error.x(), dt_);
  double cmd_vy = pid_y_.compute(position_error.y(), dt_);
  double cmd_omega = pid_theta_.compute(yaw_error, dt_);

  // Safety limits (reduce speed as approaching target)
  double distance = position_error.norm();
  double speed_scale = computeSpeedScale(distance);

  cmd_vx *= speed_scale;
  cmd_vy *= speed_scale;
  cmd_omega *= speed_scale;

  // Velocity limiting
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = std::clamp(cmd_vx, -max_velocity_, max_velocity_);
  cmd_vel.linear.y = std::clamp(cmd_vy, -max_velocity_, max_velocity_);
  cmd_vel.angular.z = std::clamp(cmd_omega, -max_angular_velocity_, max_angular_velocity_);

  return cmd_vel;
}

double VisualServoController::computeSpeedScale(double distance) {
  // Linear ramp from max speed at 0.5m to min speed at 0.05m
  const double far_distance = 0.5;   // meters
  const double near_distance = 0.05; // meters
  const double min_scale = 0.1;      // 10% of max speed when very close

  if (distance > far_distance) {
    return 1.0;
  } else if (distance < near_distance) {
    return min_scale;
  } else {
    // Linear interpolation
    return min_scale + (1.0 - min_scale) *
           (distance - near_distance) / (far_distance - near_distance);
  }
}

bool VisualServoController::isConverged(double pos_tol, double orient_tol) {
  // Check if position and orientation errors are within tolerance
  Eigen::Vector3d position_error = target_offset_ - current_marker_pose_.position;
  double position_error_norm = position_error.norm();

  Eigen::Quaterniond orientation_error =
    target_orientation_.inverse() * current_marker_pose_.orientation;
  double yaw_error = std::abs(extractYaw(orientation_error));

  bool converged = (position_error_norm < pos_tol) && (yaw_error < orient_tol);

  if (converged) {
    // Require sustained convergence
    if (!convergence_timer_started_) {
      convergence_start_time_ = node_->now();
      convergence_timer_started_ = true;
    }

    double convergence_duration = (node_->now() - convergence_start_time_).seconds();
    return convergence_duration >= required_convergence_time_;
  } else {
    convergence_timer_started_ = false;
    return false;
  }
}
```

---

## 4. State Machine Design

### 4.1 Dezyne State Machine (Formal Model)

**States and Transitions:**
```
┌──────────────────────────────────────────────────────────────┐
│               Docking State Machine (Dezyne)                 │
└──────────────────────────────────────────────────────────────┘

    ┌────────┐
    │  IDLE  │
    └───┬────┘
        │ START_DOCKING
        ↓
    ┌──────────┐
    │ APPROACH │ ← Nav2 coarse navigation to marker vicinity
    └────┬─────┘   (within ~2m of docking station)
         │ APPROACH_COMPLETE
         ↓
    ┌──────────┐
    │  SEARCH  │ ← Rotate in place to find ArUco markers
    └────┬─────┘   (search timeout: 30s)
         │ MARKER_DETECTED
         ↓
    ┌──────────┐
    │SERVOING  │ ← Visual servoing (fast approach)
    └────┬─────┘   (distance: 2m → 0.1m)
         │ CLOSE_ENOUGH (<10cm)
         ↓
    ┌──────────┐
    │FINE_ALIGN│ ← Precision alignment (slow, careful)
    └────┬─────┘   (target: ±2-5mm)
         │ CONVERGENCE_ACHIEVED
         ↓
    ┌──────────┐
    │  DOCKED  │ ← Success! Robot at docking position
    └──────────┘

    Error Handling:
    ┌─────────┐
    │ FAILED  │ ← Timeout / Collision / Marker Lost
    └────┬────┘
         │ retry_count < max_retries
         ↓
    ┌──────────┐
    │ APPROACH │ (retry)
    └──────────┘
```

**Dezyne Interface (docking.dzn):**
```dzn
interface IDocking {
  in void startDocking();
  in void abort();
  out void approachComplete();
  out void markerDetected();
  out void markerLost();
  out void convergenceAchieved();
  out void timeout();
  out void collisionDetected();
  out void dockingComplete();
  out void dockingFailed();

  behavior {
    enum State { Idle, Approach, Search, Servoing, FineAlign, Docked, Failed };
    State state = State.Idle;
    int retryCount = 0;

    on startDocking: {
      state = State.Approach;
      retryCount = 0;
    }

    on approachComplete: [state.Approach] {
      state = State.Search;
    }

    on markerDetected: [state.Search] {
      state = State.Servoing;
    }

    on convergenceAchieved: [state.Servoing] {
      if (distance < 0.1m) state = State.FineAlign;
    }

    on convergenceAchieved: [state.FineAlign] {
      state = State.Docked;
      reply dockingComplete();
    }

    on timeout: [state.Search || state.Servoing] {
      retryCount++;
      if (retryCount < MAX_RETRIES) {
        state = State.Approach;
      } else {
        state = State.Failed;
        reply dockingFailed();
      }
    }

    on abort: {
      state = State.Failed;
      reply dockingFailed();
    }
  }
}
```

---

## 5. Sequence Diagrams

### 5.1 Successful Docking Sequence

```
User/Mission    Docking         ArUco         Visual Servo    Nav2      Robot
Controller      Controller      Detector      Controller      Client    Platform
    │               │               │                │           │           │
    │ DockWheelchair()               │                │           │           │
    │──────────────>│               │                │           │           │
    │               │ [STATE: APPROACH]              │           │           │
    │               │                                │           │           │
    │               │ NavigateToPose(coarse_target)  │           │           │
    │               │────────────────────────────────────────────>│           │
    │               │                                │           │           │
    │               │<────────────────────────────────────────────│           │
    │               │ Goal reached                   │           │           │
    │               │                                │           │           │
    │               │ [STATE: SEARCH]                │           │           │
    │               │ Rotate to find markers         │           │           │
    │               │────────────────────────────────────────────────────────>│
    │               │                                │           │           │
    │               │ Camera images                  │           │           │
    │               │───────────────>│               │           │           │
    │               │                │ detectMarkers()│           │           │
    │               │                │───────────────>│           │           │
    │               │<───────────────│               │           │           │
    │               │ Marker detected (4 markers)    │           │           │
    │               │                                │           │           │
    │               │ [STATE: SERVOING]              │           │           │
    │               │ fuseDetections()               │           │           │
    │               │───────────────>│               │           │           │
    │               │<───────────────│               │           │           │
    │               │ docking_pose                   │           │           │
    │               │                                │           │           │
    │               │ computeVelocityCommand(pose)   │           │           │
    │               │────────────────────────────────>│           │           │
    │               │<────────────────────────────────│           │           │
    │               │ cmd_vel                        │           │           │
    │               │────────────────────────────────────────────────────────>│
    │               │                                │           │           │
    │               │ [distance < 10cm]              │           │           │
    │               │ [STATE: FINE_ALIGN]            │           │           │
    │               │ (reduced speed, high precision)│           │           │
    │               │────────────────────────────────────────────────────────>│
    │               │                                │           │           │
    │               │ isConverged() == true (1s sustained)      │           │
    │               │                                │           │           │
    │               │ [STATE: DOCKED]                │           │           │
    │<──────────────│                                │           │           │
    │ Result: SUCCESS (position error: 3mm, angle: 1.5°)        │           │
    │               │                                │           │           │
```

### 5.2 Marker Lost Recovery Sequence

```
Docking         ArUco         Visual Servo    State
Controller      Detector      Controller      Machine
    │               │               │             │
    │ [STATE: SERVOING]             │             │
    │               │               │             │
    │ fuseDetections()              │             │
    │──────────────>│               │             │
    │<──────────────│               │             │
    │ NO MARKERS    │               │             │
    │               │               │             │
    │ MARKER_LOST event             │             │
    │───────────────────────────────────────────>│
    │               │               │             │
    │ [STATE: SEARCH]               │             │
    │ Stop motion   │               │             │
    │ Rotate to re-acquire markers  │             │
    │               │               │             │
    │ [Timeout: 5s] │               │             │
    │ Markers re-detected           │             │
    │───────────────────────────────────────────>│
    │               │               │             │
    │ [STATE: SERVOING] (resume)    │             │
    │               │               │             │
```

---

## 6. Configuration Parameters

**docking_params.yaml:**
```yaml
docking_controller:
  ros__parameters:
    # Marker configuration
    marker_size: 0.15                   # meters (15cm ArUco markers)
    marker_ids: [0, 1, 2, 3]           # Expected marker IDs at docking station
    aruco_dictionary: "DICT_4X4_100"   # OpenCV ArUco dictionary

    # Camera calibration (loaded from file)
    front_camera:
      frame_id: "front_camera_optical"
      calibration_file: "config/front_camera.yaml"

    rear_camera:
      frame_id: "rear_camera_optical"
      calibration_file: "config/rear_camera.yaml"

    # Approach parameters
    coarse_approach_distance: 2.0      # meters (Nav2 gets robot within 2m)
    search_angular_velocity: 0.3       # rad/s (rotation during marker search)

    # Visual servoing PID gains
    servo_pid_x:
      kp: 0.5
      ki: 0.0
      kd: 0.1

    servo_pid_y:
      kp: 0.5
      ki: 0.0
      kd: 0.1

    servo_pid_theta:
      kp: 1.0
      ki: 0.0
      kd: 0.2

    # Velocity limits
    max_servo_velocity: 0.15           # m/s (servoing phase)
    max_fine_align_velocity: 0.03      # m/s (fine alignment phase)
    max_angular_velocity: 0.3          # rad/s

    # Convergence criteria
    position_tolerance: 0.005          # meters (5mm)
    orientation_tolerance: 0.035       # radians (2°)
    convergence_time: 1.0              # seconds (sustained convergence)

    # Timeouts
    search_timeout: 30.0               # seconds
    servoing_timeout: 60.0             # seconds
    fine_align_timeout: 30.0           # seconds

    # Safety
    min_marker_distance: 0.05          # meters (emergency stop if closer)
    max_retry_attempts: 3

    # Docking offset (robot position relative to markers)
    docking_offset_x: 0.0              # meters
    docking_offset_y: 0.0
    docking_offset_theta: 0.0          # radians
```

---

## 7. Error Handling

**Marker Detection Failures:**
```cpp
void DockingController::stateSearch(const std::shared_ptr<GoalHandle> goal_handle) {
  auto elapsed = (node_->now() - state_entry_time_).seconds();

  if (elapsed > params_.search_timeout) {
    RCLCPP_ERROR(logger_, "Marker search timeout (%.1fs)", elapsed);

    retry_count_++;
    if (retry_count_ < params_.max_retry_attempts) {
      RCLCPP_WARN(logger_, "Retrying docking (attempt %d/%d)",
                  retry_count_, params_.max_retry_attempts);
      transitionState(State::APPROACH);
    } else {
      RCLCPP_ERROR(logger_, "Docking failed: max retries exceeded");
      auto result = std::make_shared<DockAction::Result>();
      result->success = false;
      result->error_code = DockAction::Result::ERROR_MARKER_NOT_FOUND;
      goal_handle->abort(result);
      transitionState(State::FAILED);
    }
    return;
  }

  // Rotate slowly to search for markers
  geometry_msgs::msg::Twist search_cmd;
  search_cmd.angular.z = params_.search_angular_velocity;
  publishVelocityCommand(search_cmd);

  // Check for marker detections
  auto docking_pose = aruco_detector_->fuseDetections();
  if (docking_pose.confidence > 0.5) {
    RCLCPP_INFO(logger_, "Markers detected! Transitioning to SERVOING");
    transitionState(State::SERVOING);
  }
}
```

**Collision Detection:**
```cpp
SafetyStatus SafetyMonitor::checkSafety(
  const geometry_msgs::msg::Twist& cmd_vel,
  const DockingPose& marker_pose)
{
  // Check if too close to markers (emergency stop distance)
  double distance = marker_pose.position.norm();
  if (distance < min_marker_distance_) {
    RCLCPP_ERROR(logger_, "COLLISION IMMINENT: distance %.3fm < min %.3fm",
                 distance, min_marker_distance_);
    return SafetyStatus::EMERGENCY_STOP;
  }

  // Check if approaching too fast
  double approach_velocity = std::sqrt(
    cmd_vel.linear.x * cmd_vel.linear.x +
    cmd_vel.linear.y * cmd_vel.linear.y
  );

  double safe_velocity = computeSafeVelocity(distance);
  if (approach_velocity > safe_velocity) {
    RCLCPP_WARN(logger_, "Velocity %.2f m/s exceeds safe velocity %.2f m/s at distance %.2fm",
                approach_velocity, safe_velocity, distance);
    return SafetyStatus::VELOCITY_LIMIT;
  }

  return SafetyStatus::SAFE;
}
```

---

## 8. Testing Strategy

### 8.1 Unit Tests

**Test: ArUco Detection Accuracy**
```cpp
TEST(ArucoDetectorTest, SingleMarkerDetection) {
  ArucoDetector detector(camera_matrix, dist_coeffs, 0.15);

  // Load test image with known marker pose
  cv::Mat test_image = cv::imread("test_data/marker_1m_0deg.png");

  auto detections = detector.detectMarkers(test_image);

  ASSERT_EQ(detections.size(), 1);
  EXPECT_EQ(detections[0].marker_id, 0);

  // Verify position (should be ~1m in front of camera)
  EXPECT_NEAR(detections[0].position.z(), 1.0, 0.05);  // ±5cm
  EXPECT_NEAR(detections[0].position.x(), 0.0, 0.02);  // ±2cm
}

TEST(ArucoDetectorTest, OutdoorLightingRobustness) {
  ArucoDetector detector(camera_matrix, dist_coeffs, 0.15);

  // Test with various lighting conditions
  std::vector<std::string> test_images = {
    "test_data/bright_sunlight.png",
    "test_data/overcast.png",
    "test_data/shadow.png"
  };

  for (const auto& image_path : test_images) {
    cv::Mat test_image = cv::imread(image_path);
    auto detections = detector.detectMarkers(test_image);

    // Should detect marker in all lighting conditions
    ASSERT_GE(detections.size(), 1) << "Failed for: " << image_path;
  }
}
```

**Test: Visual Servo Convergence**
```cpp
TEST(VisualServoControllerTest, Convergence) {
  VisualServoController controller(0.5, 0.0, 0.1);  // PID gains

  // Initial position: 0.5m forward, 0.1m left, 5° rotation
  DockingPose initial_pose;
  initial_pose.position = Eigen::Vector3d(0.5, 0.1, 0.0);
  initial_pose.orientation = Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ());

  // Target: origin
  controller.setTargetOffset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());

  // Simulate servo loop
  DockingPose current_pose = initial_pose;
  for (int i = 0; i < 1000; ++i) {
    auto cmd_vel = controller.computeVelocityCommand(current_pose);

    // Simulate robot motion (simplified)
    current_pose.position.x() -= cmd_vel.linear.x * 0.05;
    current_pose.position.y() -= cmd_vel.linear.y * 0.05;

    if (controller.isConverged(0.005, 0.035)) {
      break;
    }
  }

  // Verify convergence
  EXPECT_LT(current_pose.position.norm(), 0.005);  // <5mm
}
```

---

### 8.2 Integration Tests

**Test: Full Docking Sequence (Gazebo)**
```cpp
TEST_F(DockingIntegrationTest, SuccessfulDocking) {
  // Spawn wheelchair with ArUco markers in Gazebo
  spawnWheelchairWithMarkers({0, 1, 2, 3}, Pose(2.0, 0.0, 0.0));

  // Start docking action
  auto goal = DockAction::Goal();
  goal.wheelchair_id = "test_wheelchair";

  auto result_future = docking_client_->async_send_goal(goal);

  // Wait for result (timeout: 120s)
  auto result = result_future.get();

  // Verify success
  ASSERT_TRUE(result->success);
  EXPECT_EQ(result->error_code, DockAction::Result::SUCCESS);

  // Verify final position
  auto robot_pose = getRobotPose();
  auto target_pose = getWheelchairDockingPose();

  double position_error = (robot_pose.position - target_pose.position).norm();
  EXPECT_LT(position_error, 0.01);  // <10mm (outdoor target: 5mm)
}
```

---

## 9. Performance Optimization

**Computation Budget:**
- ArUco detection: <50ms @ 10 Hz (per camera)
- Visual servo computation: <5ms @ 20 Hz
- Total CPU usage: <15% on GMKtec Nucbox K6

**Optimization Techniques:**
- Use ROI (region of interest) to limit ArUco search area
- Adaptive detector parameters based on distance
- Multi-threading for dual camera processing
- Caching of transformation matrices

---

**Document Status:** Draft
**Implementation Status:** Ready for development
**Approvals Required:** Navigation Lead, Vision Engineer, Safety Lead
