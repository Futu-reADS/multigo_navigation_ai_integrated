# ドッキングコントローラ詳細設計

**ドキュメントID:** DESIGN-DOCKING-001
**バージョン:** 1.0
**日付:** 2025-12-15
**ステータス:** ドラフト

---

## 1. 概要

本ドキュメントは、ArUcoマーカーベースのビジュアルサーボを使用した精密車椅子接続を可能にする自律ドッキングコントローラの詳細設計仕様を提供します。

**主要コンポーネント:**
- DockingController (ROS 2アクションサーバー)
- ArucoDetector (デュアルカメラフュージョン)
- VisualServoController (PBVS)
- DockingStateMachine (Dezyne形式モデル)
- SafetyMonitor (ドッキング中の衝突検出)

**ドッキング精度目標:**
- 屋外: ±2-5mm位置、±2°姿勢
- 屋内 (将来): ±1mm位置、±0.5°姿勢

---

## 2. クラス図

```
┌─────────────────────────────────────────────────────────────┐
│              rclcpp_action::ServerBase                      │
│              (ROS 2アクションインターフェース)               │
└───────────────────────┬─────────────────────────────────────┘
                        │ 継承
                        ↓
┌─────────────────────────────────────────────────────────────┐
│              DockingController                              │
│              (メインコーディネーター)                        │
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
│              (デュアルカメラフュージョン)                    │
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
│              (PBVS - 位置ベースビジュアルサーボ)             │
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
│              (Dezyne形式モデル)                             │
├─────────────────────────────────────────────────────────────┤
│ - current_state_: State                                     │
│ - retry_count_: int                                         │
├─────────────────────────────────────────────────────────────┤
│ + handleEvent(Event): State                                 │
│ + getCurrentState(): State                                  │
│ + reset()                                                   │
└─────────────────────────────────────────────────────────────┘

状態:
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

## 3. 詳細クラス仕様

### 3.1 DockingController

**ヘッダー (docking_controller.hpp):**
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
  APPROACH,      // Nav2粗接近でマーカー付近へ
  SEARCH,        // ArUcoマーカーを見つけるために回転
  SERVOING,      // ビジュアルサーボ (高速接近)
  FINE_ALIGN,    // 精密アライメント (<10cm)
  DOCKED,        // ドッキング完了
  FAILED         // ドッキング失敗
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
  // マーカー設定
  double marker_size;              // メートル (例: 0.15m)
  std::vector<int> marker_ids;     // 期待されるマーカーID [0, 1, 2, 3]

  // 接近パラメータ
  double coarse_approach_distance; // メートル (例: 2.0m)
  double search_angular_velocity;  // rad/s (例: 0.3)

  // ビジュアルサーボパラメータ
  double servo_kp_x, servo_kp_y, servo_kp_theta;
  double servo_ki_x, servo_ki_y, servo_ki_theta;
  double servo_kd_x, servo_kd_y, servo_kd_theta;
  double max_servo_velocity;       // m/s (例: 0.1)

  // 収束基準
  double position_tolerance;       // メートル (例: 0.005 = 5mm)
  double orientation_tolerance;    // ラジアン (例: 0.035 = 2°)
  double convergence_time;         // 秒 (例: 1.0)

  // タイムアウト
  double search_timeout;           // 秒 (例: 30.0)
  double servoing_timeout;         // 秒 (例: 60.0)

  // 安全性
  double min_marker_distance;      // メートル (例: 0.05)
  int max_retry_attempts;          // (例: 3)
};

struct MarkerPose {
  int marker_id;
  Eigen::Vector3d position;        // カメラフレームでの[x, y, z]
  Eigen::Quaterniond orientation;
  double quality_score;            // [0.0, 1.0]
  rclcpp::Time timestamp;
};

struct DockingPose {
  Eigen::Vector3d position;        // ドッキング目標に対するロボット位置
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

  // アクションサーバーコールバック
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const DockAction::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

  // メインドッキング実行
  void executeDocking(const std::shared_ptr<GoalHandle> goal_handle);

  // 状態固有ハンドラー
  void stateApproach(const std::shared_ptr<GoalHandle> goal_handle);
  void stateSearch(const std::shared_ptr<GoalHandle> goal_handle);
  void stateServoing(const std::shared_ptr<GoalHandle> goal_handle);
  void stateFineAlign(const std::shared_ptr<GoalHandle> goal_handle);

  // ヘルパーメソッド
  bool triggerNav2Approach(const geometry_msgs::msg::PoseStamped& target);
  void publishVelocityCommand(const geometry_msgs::msg::Twist& cmd_vel);
  void publishFeedback(const std::shared_ptr<GoalHandle>& goal_handle);
  void transitionState(State new_state);

  // コンポーネント
  std::unique_ptr<ArucoDetector> aruco_detector_;
  std::unique_ptr<VisualServoController> servo_controller_;
  std::unique_ptr<DockingStateMachine> state_machine_;
  std::unique_ptr<SafetyMonitor> safety_monitor_;

  // パラメータ
  DockingParameters params_;

  // ROSインターフェース
  rclcpp_action::Server<DockAction>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;

  // 状態
  State current_state_{State::IDLE};
  rclcpp::Time state_entry_time_;
  int retry_count_{0};
};

}  // namespace docking_controller

#endif  // DOCKING_CONTROLLER__DOCKING_CONTROLLER_HPP_
```

---

### 3.2 ArucoDetector

**主要メソッド:**

**detectMarkers (屋外最適化付き):**
```cpp
std::vector<MarkerDetection> ArucoDetector::detectMarkers(const cv::Mat& image) {
  cv::Mat processed = applyOutdoorOptimizations(image);

  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<int> marker_ids;

  // ArUcoマーカーを検出
  cv::aruco::detectMarkers(
    processed,
    aruco_dict_,
    marker_corners,
    marker_ids,
    detector_params_
  );

  std::vector<MarkerDetection> detections;

  for (size_t i = 0; i < marker_ids.size(); ++i) {
    // コーナーから3D姿勢を推定
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

    // 回転ベクトルをクォータニオンに変換
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    detection.orientation = rotationMatrixToQuaternion(rotation_matrix);

    // コーナーの鮮明さと再投影誤差に基づく品質スコア
    detection.quality_score = computeMarkerQuality(marker_corners[i], rvec, tvec);

    detections.push_back(detection);
  }

  return detections;
}

cv::Mat ArucoDetector::applyOutdoorOptimizations(const cv::Mat& image) {
  cv::Mat gray, enhanced;

  // グレースケールに変換
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

  // CLAHE (コントラスト制限適応ヒストグラム平坦化) を適用
  // 屋外照明変動に対して重要
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(2.0);
  clahe->setTilesGridSize(cv::Size(8, 8));
  clahe->apply(gray, enhanced);

  // オプション: ノイズ低減のためのガウシアンぼかし
  cv::GaussianBlur(enhanced, enhanced, cv::Size(5, 5), 0);

  return enhanced;
}
```

**fuseDetections (デュアルカメラフュージョン):**
```cpp
DockingPose ArucoDetector::fuseDetections() {
  // 両方のカメラからの最近の検出を収集
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
    return DockingPose{};  // 有効な検出なし
  }

  // 品質スコアに基づく加重平均
  Eigen::Vector3d weighted_position = Eigen::Vector3d::Zero();
  double total_weight = 0.0;

  for (const auto& detection : all_detections) {
    // マーカー姿勢をカメラフレームからbase_linkに変換
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

  // ドッキング目標を計算 (マーカーからのオフセット)
  fused_pose.position -= docking_offset_;

  return fused_pose;
}
```

---

### 3.3 VisualServoController

**PBVSコントローラ実装:**
```cpp
geometry_msgs::msg::Twist VisualServoController::computeVelocityCommand(
  const DockingPose& marker_pose)
{
  // base_linkフレームでの位置誤差を計算
  Eigen::Vector3d position_error = target_offset_ - marker_pose.position;

  // 姿勢誤差を計算
  Eigen::Quaterniond orientation_error =
    target_orientation_.inverse() * marker_pose.orientation;

  // ヨー誤差を抽出
  double yaw_error = extractYaw(orientation_error);

  // x, y, theta のPID制御
  double cmd_vx = pid_x_.compute(position_error.x(), dt_);
  double cmd_vy = pid_y_.compute(position_error.y(), dt_);
  double cmd_omega = pid_theta_.compute(yaw_error, dt_);

  // 安全制限 (目標に接近するにつれて速度を低減)
  double distance = position_error.norm();
  double speed_scale = computeSpeedScale(distance);

  cmd_vx *= speed_scale;
  cmd_vy *= speed_scale;
  cmd_omega *= speed_scale;

  // 速度制限
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = std::clamp(cmd_vx, -max_velocity_, max_velocity_);
  cmd_vel.linear.y = std::clamp(cmd_vy, -max_velocity_, max_velocity_);
  cmd_vel.angular.z = std::clamp(cmd_omega, -max_angular_velocity_, max_angular_velocity_);

  return cmd_vel;
}

double VisualServoController::computeSpeedScale(double distance) {
  // 0.5mで最大速度から0.05mで最小速度への線形ランプ
  const double far_distance = 0.5;   // メートル
  const double near_distance = 0.05; // メートル
  const double min_scale = 0.1;      // 非常に近い時は最大速度の10%

  if (distance > far_distance) {
    return 1.0;
  } else if (distance < near_distance) {
    return min_scale;
  } else {
    // 線形補間
    return min_scale + (1.0 - min_scale) *
           (distance - near_distance) / (far_distance - near_distance);
  }
}

bool VisualServoController::isConverged(double pos_tol, double orient_tol) {
  // 位置と姿勢の誤差が許容範囲内かチェック
  Eigen::Vector3d position_error = target_offset_ - current_marker_pose_.position;
  double position_error_norm = position_error.norm();

  Eigen::Quaterniond orientation_error =
    target_orientation_.inverse() * current_marker_pose_.orientation;
  double yaw_error = std::abs(extractYaw(orientation_error));

  bool converged = (position_error_norm < pos_tol) && (yaw_error < orient_tol);

  if (converged) {
    // 持続的な収束を要求
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

## 4. 状態機械設計

### 4.1 Dezyne状態機械 (形式モデル)

**状態と遷移:**
```
┌──────────────────────────────────────────────────────────────┐
│             ドッキング状態機械 (Dezyne)                      │
└──────────────────────────────────────────────────────────────┘

    ┌────────┐
    │  IDLE  │
    └───┬────┘
        │ START_DOCKING
        ↓
    ┌──────────┐
    │ APPROACH │ ← Nav2粗ナビゲーションでマーカー付近へ
    └────┬─────┘   (ドッキングステーションから約2m以内)
         │ APPROACH_COMPLETE
         ↓
    ┌──────────┐
    │  SEARCH  │ ← その場で回転してArUcoマーカーを見つける
    └────┬─────┘   (検索タイムアウト: 30秒)
         │ MARKER_DETECTED
         ↓
    ┌──────────┐
    │SERVOING  │ ← ビジュアルサーボ (高速接近)
    └────┬─────┘   (距離: 2m → 0.1m)
         │ CLOSE_ENOUGH (<10cm)
         ↓
    ┌──────────┐
    │FINE_ALIGN│ ← 精密アライメント (低速、慎重)
    └────┬─────┘   (目標: ±2-5mm)
         │ CONVERGENCE_ACHIEVED
         ↓
    ┌──────────┐
    │  DOCKED  │ ← 成功! ロボットはドッキング位置
    └──────────┘

    エラーハンドリング:
    ┌─────────┐
    │ FAILED  │ ← タイムアウト / 衝突 / マーカー喪失
    └────┬────┘
         │ retry_count < max_retries
         ↓
    ┌──────────┐
    │ APPROACH │ (再試行)
    └──────────┘
```

**Dezyneインターフェース (docking.dzn):**
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

## 5. シーケンス図

### 5.1 成功ドッキングシーケンス

```
ユーザー/ミッション ドッキング      ArUco         ビジュアルサーボ Nav2      ロボット
コントローラ         コントローラ    検出器        コントローラ      クライアント プラットフォーム
    │               │               │                │           │           │
    │ DockWheelchair()               │                │           │           │
    │──────────────>│               │                │           │           │
    │               │ [状態: APPROACH]               │           │           │
    │               │                                │           │           │
    │               │ NavigateToPose(粗目標)         │           │           │
    │               │────────────────────────────────────────────>│           │
    │               │                                │           │           │
    │               │<────────────────────────────────────────────│           │
    │               │ ゴール到達                     │           │           │
    │               │                                │           │           │
    │               │ [状態: SEARCH]                 │           │           │
    │               │ マーカーを見つけるために回転   │           │           │
    │               │────────────────────────────────────────────────────────>│
    │               │                                │           │           │
    │               │ カメラ画像                     │           │           │
    │               │───────────────>│               │           │           │
    │               │                │ detectMarkers()│           │           │
    │               │                │───────────────>│           │           │
    │               │<───────────────│               │           │           │
    │               │ マーカー検出 (4マーカー)       │           │           │
    │               │                                │           │           │
    │               │ [状態: SERVOING]               │           │           │
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
    │               │ [距離 < 10cm]                  │           │           │
    │               │ [状態: FINE_ALIGN]             │           │           │
    │               │ (低速度、高精度)               │           │           │
    │               │────────────────────────────────────────────────────────>│
    │               │                                │           │           │
    │               │ isConverged() == true (1秒間持続)         │           │
    │               │                                │           │           │
    │               │ [状態: DOCKED]                 │           │           │
    │<──────────────│                                │           │           │
    │ 結果: 成功 (位置誤差: 3mm、角度: 1.5°)         │           │           │
    │               │                                │           │           │
```

### 5.2 マーカー喪失回復シーケンス

```
ドッキング         ArUco         ビジュアルサーボ 状態
コントローラ       検出器        コントローラ      機械
    │               │               │             │
    │ [状態: SERVOING]              │             │
    │               │               │             │
    │ fuseDetections()              │             │
    │──────────────>│               │             │
    │<──────────────│               │             │
    │ マーカーなし  │               │             │
    │               │               │             │
    │ MARKER_LOSTイベント           │             │
    │───────────────────────────────────────────>│
    │               │               │             │
    │ [状態: SEARCH]                │             │
    │ 動作停止      │               │             │
    │ マーカー再取得のために回転    │             │
    │               │               │             │
    │ [タイムアウト: 5秒]           │             │
    │ マーカー再検出                │             │
    │───────────────────────────────────────────>│
    │               │               │             │
    │ [状態: SERVOING] (再開)       │             │
    │               │               │             │
```

---

## 6. 設定パラメータ

**docking_params.yaml:**
```yaml
docking_controller:
  ros__parameters:
    # マーカー設定
    marker_size: 0.15                   # メートル (15cm ArUcoマーカー)
    marker_ids: [0, 1, 2, 3]           # ドッキングステーションで期待されるマーカーID
    aruco_dictionary: "DICT_4X4_100"   # OpenCV ArUco辞書

    # カメラキャリブレーション (ファイルからロード)
    front_camera:
      frame_id: "front_camera_optical"
      calibration_file: "config/front_camera.yaml"

    rear_camera:
      frame_id: "rear_camera_optical"
      calibration_file: "config/rear_camera.yaml"

    # 接近パラメータ
    coarse_approach_distance: 2.0      # メートル (Nav2がロボットを2m以内に取得)
    search_angular_velocity: 0.3       # rad/s (マーカー検索中の回転)

    # ビジュアルサーボPIDゲイン
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

    # 速度制限
    max_servo_velocity: 0.15           # m/s (サーボフェーズ)
    max_fine_align_velocity: 0.03      # m/s (微調整フェーズ)
    max_angular_velocity: 0.3          # rad/s

    # 収束基準
    position_tolerance: 0.005          # メートル (5mm)
    orientation_tolerance: 0.035       # ラジアン (2°)
    convergence_time: 1.0              # 秒 (持続的な収束)

    # タイムアウト
    search_timeout: 30.0               # 秒
    servoing_timeout: 60.0             # 秒
    fine_align_timeout: 30.0           # 秒

    # 安全性
    min_marker_distance: 0.05          # メートル (これより近い場合は緊急停止)
    max_retry_attempts: 3

    # ドッキングオフセット (マーカーに対するロボット位置)
    docking_offset_x: 0.0              # メートル
    docking_offset_y: 0.0
    docking_offset_theta: 0.0          # ラジアン
```

---

## 7. エラーハンドリング

**マーカー検出失敗:**
```cpp
void DockingController::stateSearch(const std::shared_ptr<GoalHandle> goal_handle) {
  auto elapsed = (node_->now() - state_entry_time_).seconds();

  if (elapsed > params_.search_timeout) {
    RCLCPP_ERROR(logger_, "マーカー検索タイムアウト (%.1f秒)", elapsed);

    retry_count_++;
    if (retry_count_ < params_.max_retry_attempts) {
      RCLCPP_WARN(logger_, "ドッキング再試行 (試行 %d/%d)",
                  retry_count_, params_.max_retry_attempts);
      transitionState(State::APPROACH);
    } else {
      RCLCPP_ERROR(logger_, "ドッキング失敗: 最大再試行回数を超えました");
      auto result = std::make_shared<DockAction::Result>();
      result->success = false;
      result->error_code = DockAction::Result::ERROR_MARKER_NOT_FOUND;
      goal_handle->abort(result);
      transitionState(State::FAILED);
    }
    return;
  }

  // マーカーを検索するためにゆっくり回転
  geometry_msgs::msg::Twist search_cmd;
  search_cmd.angular.z = params_.search_angular_velocity;
  publishVelocityCommand(search_cmd);

  // マーカー検出をチェック
  auto docking_pose = aruco_detector_->fuseDetections();
  if (docking_pose.confidence > 0.5) {
    RCLCPP_INFO(logger_, "マーカー検出! SERVOINGに遷移");
    transitionState(State::SERVOING);
  }
}
```

**衝突検出:**
```cpp
SafetyStatus SafetyMonitor::checkSafety(
  const geometry_msgs::msg::Twist& cmd_vel,
  const DockingPose& marker_pose)
{
  // マーカーに近すぎるかチェック (緊急停止距離)
  double distance = marker_pose.position.norm();
  if (distance < min_marker_distance_) {
    RCLCPP_ERROR(logger_, "衝突の危険: 距離 %.3fm < 最小 %.3fm",
                 distance, min_marker_distance_);
    return SafetyStatus::EMERGENCY_STOP;
  }

  // 接近速度が速すぎるかチェック
  double approach_velocity = std::sqrt(
    cmd_vel.linear.x * cmd_vel.linear.x +
    cmd_vel.linear.y * cmd_vel.linear.y
  );

  double safe_velocity = computeSafeVelocity(distance);
  if (approach_velocity > safe_velocity) {
    RCLCPP_WARN(logger_, "速度 %.2f m/s が距離 %.2fm での安全速度 %.2f m/s を超えています",
                approach_velocity, safe_velocity, distance);
    return SafetyStatus::VELOCITY_LIMIT;
  }

  return SafetyStatus::SAFE;
}
```

---

## 8. テスト戦略

### 8.1 単体テスト

**テスト: ArUco検出精度**
```cpp
TEST(ArucoDetectorTest, SingleMarkerDetection) {
  ArucoDetector detector(camera_matrix, dist_coeffs, 0.15);

  // 既知のマーカー姿勢のテスト画像をロード
  cv::Mat test_image = cv::imread("test_data/marker_1m_0deg.png");

  auto detections = detector.detectMarkers(test_image);

  ASSERT_EQ(detections.size(), 1);
  EXPECT_EQ(detections[0].marker_id, 0);

  // 位置を検証 (カメラの約1m前方であるべき)
  EXPECT_NEAR(detections[0].position.z(), 1.0, 0.05);  // ±5cm
  EXPECT_NEAR(detections[0].position.x(), 0.0, 0.02);  // ±2cm
}

TEST(ArucoDetectorTest, OutdoorLightingRobustness) {
  ArucoDetector detector(camera_matrix, dist_coeffs, 0.15);

  // さまざまな照明条件でテスト
  std::vector<std::string> test_images = {
    "test_data/bright_sunlight.png",
    "test_data/overcast.png",
    "test_data/shadow.png"
  };

  for (const auto& image_path : test_images) {
    cv::Mat test_image = cv::imread(image_path);
    auto detections = detector.detectMarkers(test_image);

    // すべての照明条件でマーカーを検出すべき
    ASSERT_GE(detections.size(), 1) << "失敗: " << image_path;
  }
}
```

**テスト: ビジュアルサーボ収束**
```cpp
TEST(VisualServoControllerTest, Convergence) {
  VisualServoController controller(0.5, 0.0, 0.1);  // PIDゲイン

  // 初期位置: 前方0.5m、左0.1m、回転5°
  DockingPose initial_pose;
  initial_pose.position = Eigen::Vector3d(0.5, 0.1, 0.0);
  initial_pose.orientation = Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ());

  // 目標: 原点
  controller.setTargetOffset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());

  // サーボループをシミュレート
  DockingPose current_pose = initial_pose;
  for (int i = 0; i < 1000; ++i) {
    auto cmd_vel = controller.computeVelocityCommand(current_pose);

    // ロボットの動きをシミュレート (簡略化)
    current_pose.position.x() -= cmd_vel.linear.x * 0.05;
    current_pose.position.y() -= cmd_vel.linear.y * 0.05;

    if (controller.isConverged(0.005, 0.035)) {
      break;
    }
  }

  // 収束を検証
  EXPECT_LT(current_pose.position.norm(), 0.005);  // <5mm
}
```

---

### 8.2 統合テスト

**テスト: 完全ドッキングシーケンス (Gazebo)**
```cpp
TEST_F(DockingIntegrationTest, SuccessfulDocking) {
  // GazeboでArUcoマーカー付き車椅子を生成
  spawnWheelchairWithMarkers({0, 1, 2, 3}, Pose(2.0, 0.0, 0.0));

  // ドッキングアクション開始
  auto goal = DockAction::Goal();
  goal.wheelchair_id = "test_wheelchair";

  auto result_future = docking_client_->async_send_goal(goal);

  // 結果を待つ (タイムアウト: 120秒)
  auto result = result_future.get();

  // 成功を検証
  ASSERT_TRUE(result->success);
  EXPECT_EQ(result->error_code, DockAction::Result::SUCCESS);

  // 最終位置を検証
  auto robot_pose = getRobotPose();
  auto target_pose = getWheelchairDockingPose();

  double position_error = (robot_pose.position - target_pose.position).norm();
  EXPECT_LT(position_error, 0.01);  // <10mm (屋外目標: 5mm)
}
```

---

## 9. パフォーマンス最適化

**計算予算:**
- ArUco検出: <50ms @ 10 Hz (カメラあたり)
- ビジュアルサーボ計算: <5ms @ 20 Hz
- 合計CPU使用率: GMKtec Nucbox K6で<15%

**最適化技術:**
- ROI (関心領域) を使用してArUco検索領域を制限
- 距離に基づく適応検出器パラメータ
- デュアルカメラ処理用のマルチスレッディング
- 変換行列のキャッシング

---

**ドキュメントステータス:** ドラフト
**実装ステータス:** 開発準備完了
**必要な承認:** ナビゲーションリード、ビジョンエンジニア、安全性リード
