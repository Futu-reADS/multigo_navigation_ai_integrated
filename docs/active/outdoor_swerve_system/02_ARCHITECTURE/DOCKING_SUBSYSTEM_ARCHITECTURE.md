# Docking Subsystem Architecture

**Document ID:** ARCH-DOCK-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft

---

## 1. Overview

The **Docking Subsystem** enables precision attachment to wheelchair bases using visual servoing with ArUco markers. This is a **NEW subsystem** (not in ParcelPal) designed specifically for the wheelchair transport use case.

**Key Capabilities:**
- Two-phase docking (coarse navigation + fine visual servoing)
- ArUco marker-based precision positioning (±2-5mm outdoor)
- Dual camera fusion for reliability
- State machine-driven docking sequence
- Safety monitoring and failure recovery
- Outdoor operation capability (light rain, varying lighting)

**Docking Target:** ±2-5mm position accuracy, >95% success rate outdoor

---

## 2. System Architecture

### 2.1 Component Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    Docking Subsystem                            │
│                                                                 │
│  ┌──────────────────┐      ┌──────────────────────────────┐   │
│  │  Nav2            │─────▶│  Docking Coordinator         │   │
│  │  (Coarse Approach│      │  (State Machine)             │   │
│  │   to zone)       │      │                              │   │
│  └──────────────────┘      │  - IDLE → APPROACH → SEARCH  │   │
│                             │  - SERVOING → DOCKED         │   │
│                             │  - Failure handling          │   │
│                             └──────────┬───────────────────┘   │
│                                        │                        │
│                                        │ marker pose            │
│                                        ↓                        │
│                      ┌─────────────────────────────┐           │
│                      │   ArUco Detector            │           │
│                      │   (OpenCV cv::aruco)        │           │
│                      │                             │           │
│  ┌─────────────┐    │  - Dual camera fusion       │           │
│  │ Camera 1    │───▶│  - Pose estimation (PnP)    │           │
│  │ (Front)     │    │  - Outlier rejection        │           │
│  └─────────────┘    └─────────────┬───────────────┘           │
│                                    │                            │
│  ┌─────────────┐                   │ marker pose                │
│  │ Camera 2    │───────────────────┘                            │
│  │ (Front-Alt) │                                                │
│  └─────────────┘                                                │
│                                        │                        │
│                                        │ target velocity        │
│                                        ↓                        │
│                      ┌─────────────────────────────┐           │
│                      │   Visual Servoing           │           │
│                      │   Controller (PBVS)         │           │
│                      │                             │           │
│                      │  - Position-based VS        │           │
│                      │  - PID control (x,y,θ)      │           │
│                      │  - Velocity limiting        │           │
│                      └──────────┬──────────────────┘           │
│                                 │                               │
│                                 │ /cmd_vel                      │
│                                 ↓                               │
│                      ┌─────────────────────────────┐           │
│                      │   Swerve Drive Controller   │           │
│                      │   (Omnidirectional motion)  │           │
│                      └─────────────────────────────┘           │
└─────────────────────────────────────────────────────────────────┘
```

**Key Components:**
1. **Docking Coordinator**: State machine managing docking sequence
2. **ArUco Detector**: Marker detection and pose estimation from dual cameras
3. **Visual Servoing Controller**: Position-based visual servoing (PBVS) with PID control
4. **Swerve Drive Controller**: Executes precision motion commands

---

## 3. ROS 2 Node Architecture

### 3.1 Docking Coordinator Node

**Node Name:** `docking_coordinator`
**Language:** C++
**Lifecycle:** Managed lifecycle node

**Subscribed Topics:**
- `/aruco_poses` (geometry_msgs/PoseArray): Detected ArUco marker poses
- `/odom` (nav_msgs/Odometry): Current robot odometry
- `/camera_1/image_raw` (sensor_msgs/Image): Front camera image
- `/camera_2/image_raw` (sensor_msgs/Image): Front-alt camera image

**Published Topics:**
- `/docking/state` (std_msgs/String): Current docking state
- `/docking/status` (custom_msgs/DockingStatus): Detailed status and diagnostics
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands during visual servoing

**Action Servers:**
- `/dock` (custom_msgs/action/Dock): Execute docking sequence
- `/undock` (custom_msgs/action/Undock): Execute undocking sequence

**Action Clients:**
- `/navigate_to_pose` (nav2_msgs/action/NavigateToPose): Coarse approach navigation

**Parameters:**
```yaml
# Approach zone configuration
approach_distance: 1.5          # meters from docking station
approach_tolerance: 0.3         # meters (Nav2 xy_goal_tolerance)

# Visual servoing configuration
servoing_gains:
  kp_x: 0.8                     # Proportional gain for x
  kp_y: 0.8                     # Proportional gain for y
  kp_theta: 1.2                 # Proportional gain for θ
  ki_x: 0.1                     # Integral gain for x
  ki_y: 0.1                     # Integral gain for y

servoing_limits:
  max_linear_vel: 0.2           # m/s during servoing
  max_angular_vel: 0.3          # rad/s during servoing

# Docking success criteria
docking_tolerance:
  position_xy: 0.005            # meters (±5mm)
  orientation: 0.087            # radians (±5°)
  stable_duration: 2.0          # seconds

# Failure thresholds
max_servoing_time: 60.0         # seconds
marker_lost_timeout: 5.0        # seconds
```

---

### 3.2 ArUco Detector Node

**Node Name:** `aruco_detector`
**Language:** C++
**Lifecycle:** Standard node

**Subscribed Topics:**
- `/camera_1/image_raw` (sensor_msgs/Image): Front camera image
- `/camera_1/camera_info` (sensor_msgs/CameraInfo): Camera calibration
- `/camera_2/image_raw` (sensor_msgs/Image): Front-alt camera image
- `/camera_2/camera_info` (sensor_msgs/CameraInfo): Camera calibration

**Published Topics:**
- `/aruco_poses` (geometry_msgs/PoseArray): Detected marker poses in robot frame
- `/aruco_debug` (sensor_msgs/Image): Debug image with marker overlays
- `/aruco_detections` (custom_msgs/ArucoDetectionArray): Per-camera detections

**Parameters:**
```yaml
# ArUco configuration
dictionary: "DICT_4X4_50"       # ArUco dictionary
marker_size: 0.15               # meters (15cm physical size)
detection_rate: 10              # Hz

# Dual camera fusion
fusion_mode: "weighted_average" # or "best_camera"
min_detection_confidence: 0.7   # Reject low-confidence detections
max_reprojection_error: 2.0     # pixels

# Camera intrinsics (loaded from camera_info)
camera_1_frame: "camera_1_optical_frame"
camera_2_frame: "camera_2_optical_frame"
base_frame: "base_link"
```

---

### 3.3 Visual Servoing Controller Node

**Node Name:** `visual_servoing_controller`
**Language:** C++
**Lifecycle:** Standard node

**Subscribed Topics:**
- `/aruco_poses` (geometry_msgs/PoseArray): Target marker pose
- `/docking/state` (std_msgs/String): Docking state (enable/disable control)

**Published Topics:**
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for swerve drive

**Parameters:**
```yaml
# Control method
control_method: "PBVS"          # Position-Based Visual Servoing

# PID gains (tuned for outdoor operation)
pid_gains:
  kp: [0.8, 0.8, 1.2]           # [x, y, θ]
  ki: [0.1, 0.1, 0.0]           # [x, y, θ]
  kd: [0.05, 0.05, 0.1]         # [x, y, θ]

# Velocity limits
max_vel:
  linear_x: 0.2                 # m/s
  linear_y: 0.2                 # m/s
  angular_z: 0.3                # rad/s

# Deadband (avoid oscillation)
deadband:
  position_xy: 0.002            # meters (±2mm)
  orientation: 0.017            # radians (±1°)
```

---

## 4. Docking State Machine

### 4.1 State Transition Diagram

```
      ┌─────────────┐
      │    IDLE     │
      └──────┬──────┘
             │ /dock action called
             ↓
      ┌─────────────┐
      │  APPROACH   │ ← Nav2 navigation to approach zone (±30cm)
      └──────┬──────┘
             │ reached approach zone
             ↓
      ┌─────────────┐
      │   SEARCH    │ ← Rotate slowly, scan for ArUco marker
      └──────┬──────┘
             │ marker detected
             ↓
      ┌─────────────┐
      │  SERVOING   │ ← Visual servoing to target pose (±5mm)
      └──────┬──────┘
             │ position stable (±5mm for 2s)
             ↓
      ┌─────────────┐
      │   DOCKED    │ ← Send locking command to wheelchair
      └──────┬──────┘
             │ /undock action called
             ↓
      ┌─────────────┐
      │ UNDOCKING   │ ← Unlock, back away 0.5m
      └──────┬──────┘
             │ undocking complete
             ↓
      ┌─────────────┐
      │    IDLE     │
      └─────────────┘

Failure Transitions (any state):
- Marker lost >5s → SEARCH
- Timeout → FAILED → IDLE
- Emergency stop → IDLE
```

---

### 4.2 State Implementation

**IDLE:**
- No active control
- Waiting for `/dock` action goal
- Wheelchair lock disengaged

**APPROACH:**
- Call Nav2 `/navigate_to_pose` action with approach pose
- Approach pose = docking station pose + offset (1.5m in front)
- Tolerance: ±30cm (Nav2 xy_goal_tolerance)
- **Transition:** Reached approach zone → SEARCH

**SEARCH:**
- Rotate slowly (0.2 rad/s) to scan for ArUco marker
- Subscribe to `/aruco_poses` topic
- **Transition:** Marker detected for 3 consecutive frames → SERVOING
- **Timeout:** 30 seconds → FAILED

**SERVOING:**
- Enable visual servoing controller
- Compute target velocity from marker pose error
- Publish `/cmd_vel` at 10 Hz
- Monitor position error: `error = sqrt((x-x_target)^2 + (y-y_target)^2)`
- **Transition:** Error <5mm and stable for 2s → DOCKED
- **Marker lost:** >5s → SEARCH
- **Timeout:** 60s → FAILED

**DOCKED:**
- Stop all motion (`/cmd_vel` = 0)
- Send locking command to wheelchair (future: CAN/serial interface)
- Publish success feedback on `/dock` action
- **Transition:** `/undock` action called → UNDOCKING

**UNDOCKING:**
- Unlock wheelchair (future: CAN/serial interface)
- Back away 0.5m (open-loop motion for 2.5s at -0.2 m/s)
- **Transition:** Motion complete → IDLE

**FAILED:**
- Stop all motion
- Publish failure feedback on `/dock` action
- Log failure reason (marker lost, timeout, safety violation)
- **Transition:** Automatic → IDLE

---

## 5. ArUco Detection Pipeline

### 5.1 Marker Detection (OpenCV)

**Algorithm:**
```cpp
void detectMarkers(const cv::Mat& image) {
    // 1. Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // 2. Adaptive histogram equalization (outdoor lighting robustness)
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(gray, gray);

    // 3. ArUco marker detection
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::Ptr<cv::aruco::Dictionary> dictionary = 
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::DetectorParameters> params = 
        cv::aruco::DetectorParameters::create();
    
    // Tuned for outdoor detection
    params->adaptiveThreshWinSizeMin = 3;
    params->adaptiveThreshWinSizeMax = 23;
    params->adaptiveThreshWinSizeStep = 10;
    
    cv::aruco::detectMarkers(gray, dictionary, marker_corners, marker_ids, params);

    // 4. Reject detections with poor corner quality
    filterLowQualityDetections(marker_corners, marker_ids);
}
```

---

### 5.2 Pose Estimation (PnP)

**Algorithm:**
```cpp
geometry_msgs::msg::Pose estimateMarkerPose(
    const std::vector<cv::Point2f>& corners,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    double marker_size) {

    // 3D marker corners (marker frame, z=0 plane)
    std::vector<cv::Point3f> object_points = {
        {-marker_size/2,  marker_size/2, 0},  // top-left
        { marker_size/2,  marker_size/2, 0},  // top-right
        { marker_size/2, -marker_size/2, 0},  // bottom-right
        {-marker_size/2, -marker_size/2, 0}   // bottom-left
    };

    // Solve PnP (Perspective-n-Point)
    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(
        object_points,
        corners,
        camera_matrix,
        dist_coeffs,
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_IPPE_SQUARE  // Best for planar markers
    );

    if (!success) {
        return {};  // Invalid pose
    }

    // Check reprojection error
    std::vector<cv::Point2f> reprojected;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, reprojected);
    double error = computeReprojectionError(corners, reprojected);
    if (error > max_reprojection_error) {
        return {};  // Poor pose quality
    }

    // Convert to geometry_msgs::Pose
    return toPose(rvec, tvec);
}
```

---

### 5.3 Dual Camera Fusion

**Weighted Average Fusion:**
```cpp
geometry_msgs::msg::Pose fuseCameraPoses(
    const geometry_msgs::msg::Pose& pose_cam1,
    const geometry_msgs::msg::Pose& pose_cam2,
    double confidence_cam1,
    double confidence_cam2) {

    // Normalize confidences
    double total_confidence = confidence_cam1 + confidence_cam2;
    double w1 = confidence_cam1 / total_confidence;
    double w2 = confidence_cam2 / total_confidence;

    // Weighted average of positions
    geometry_msgs::msg::Pose fused;
    fused.position.x = w1 * pose_cam1.position.x + w2 * pose_cam2.position.x;
    fused.position.y = w1 * pose_cam1.position.y + w2 * pose_cam2.position.y;
    fused.position.z = w1 * pose_cam1.position.z + w2 * pose_cam2.position.z;

    // SLERP (Spherical Linear Interpolation) for orientations
    fused.orientation = slerp(
        pose_cam1.orientation,
        pose_cam2.orientation,
        w2  // interpolation factor
    );

    return fused;
}

// Confidence metric: function of reprojection error and marker size
double computeConfidence(double reprojection_error, double marker_area_pixels) {
    // Lower error → higher confidence
    double error_factor = 1.0 / (1.0 + reprojection_error);
    
    // Larger marker → higher confidence (closer to camera)
    double size_factor = std::clamp(marker_area_pixels / 10000.0, 0.1, 1.0);
    
    return error_factor * size_factor;
}
```

---

## 6. Visual Servoing Control

### 6.1 Position-Based Visual Servoing (PBVS)

**Control Law:**
```cpp
geometry_msgs::msg::Twist computeServoingCommand(
    const geometry_msgs::msg::Pose& current_pose,
    const geometry_msgs::msg::Pose& target_pose) {

    // 1. Compute pose error in robot frame
    Eigen::Vector3d position_error;
    position_error.x() = target_pose.position.x - current_pose.position.x;
    position_error.y() = target_pose.position.y - current_pose.position.y;
    position_error.z() = 0.0;  // Ignore z (assume planar motion)

    // 2. Compute orientation error (angle between quaternions)
    tf2::Quaternion q_current, q_target;
    tf2::fromMsg(current_pose.orientation, q_current);
    tf2::fromMsg(target_pose.orientation, q_target);
    tf2::Quaternion q_error = q_target * q_current.inverse();
    double theta_error = q_error.getAngle();

    // 3. PID control
    Eigen::Vector3d integral_error;  // accumulated over time
    Eigen::Vector3d derivative_error;  // rate of change

    // Proportional term
    double vx = kp_x * position_error.x() + ki_x * integral_error.x();
    double vy = kp_y * position_error.y() + ki_y * integral_error.y();
    double omega = kp_theta * theta_error;

    // 4. Apply deadband (avoid oscillation near target)
    if (position_error.norm() < deadband_position) {
        vx = 0.0;
        vy = 0.0;
    }
    if (std::abs(theta_error) < deadband_orientation) {
        omega = 0.0;
    }

    // 5. Velocity limiting
    double v_mag = std::sqrt(vx*vx + vy*vy);
    if (v_mag > max_linear_vel) {
        vx *= max_linear_vel / v_mag;
        vy *= max_linear_vel / v_mag;
    }
    omega = std::clamp(omega, -max_angular_vel, max_angular_vel);

    // 6. Construct Twist message
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.angular.z = omega;

    return cmd_vel;
}
```

---

### 6.2 Stability Verification

**Docking Completion Check:**
```cpp
bool isDockingStable(
    const std::vector<geometry_msgs::msg::Pose>& pose_history,
    double duration_threshold,
    double position_tolerance,
    double orientation_tolerance) {

    // Require at least N samples (e.g., 20 samples @ 10 Hz = 2 seconds)
    int required_samples = static_cast<int>(duration_threshold * control_rate);
    if (pose_history.size() < required_samples) {
        return false;
    }

    // Check last N samples are all within tolerance
    geometry_msgs::msg::Pose target = pose_history.back();
    for (int i = pose_history.size() - required_samples; i < pose_history.size(); i++) {
        double position_error = computePositionError(pose_history[i], target);
        double orientation_error = computeOrientationError(pose_history[i], target);

        if (position_error > position_tolerance ||
            orientation_error > orientation_tolerance) {
            return false;  // Not stable
        }
    }

    return true;  // Stable for required duration
}
```

---

## 7. Safety Features

### 7.1 Collision Avoidance During Docking

**Approach:**
- Use 3D LiDAR during APPROACH and SEARCH states (normal Nav2 costmap)
- **Disable** obstacle avoidance during SERVOING (trust visual servoing path)
- Monitor wheelchair base presence (expected obstacle at docking location)

**Implementation:**
```cpp
void updateCostmapConfiguration(DockingState state) {
    if (state == DockingState::SERVOING) {
        // Disable costmap inflation during servoing
        costmap_client->setParameter("inflation_radius", 0.0);
    } else {
        // Normal inflation during approach/search
        costmap_client->setParameter("inflation_radius", 0.3);
    }
}
```

---

### 7.2 Failure Detection and Recovery

**Failure Modes:**
1. **Marker Lost During Servoing:** Return to SEARCH state, re-acquire marker
2. **Timeout in SERVOING:** After 60s, declare FAILED, return to IDLE
3. **Unstable Marker Pose:** If pose jumps >10cm between frames, reject outlier
4. **Camera Failure:** If both cameras fail, abort docking
5. **Collision During Approach:** Nav2 detects obstacle, abort docking

**Recovery Strategy:**
```cpp
void handleFailure(FailureReason reason) {
    switch (reason) {
        case FailureReason::MARKER_LOST:
            if (marker_lost_duration < marker_lost_timeout) {
                // Short loss: continue servoing with last known pose
                continue_with_last_pose = true;
            } else {
                // Long loss: return to SEARCH
                transitionTo(DockingState::SEARCH);
            }
            break;

        case FailureReason::TIMEOUT:
            // Abort docking, return to IDLE
            transitionTo(DockingState::FAILED);
            publishActionResult(false, "Docking timeout");
            break;

        case FailureReason::CAMERA_FAILURE:
            // Critical failure, cannot recover
            transitionTo(DockingState::FAILED);
            publishActionResult(false, "Camera failure");
            break;

        case FailureReason::COLLISION:
            // Nav2 detected collision during approach
            transitionTo(DockingState::FAILED);
            publishActionResult(false, "Collision during approach");
            break;
    }
}
```

---

## 8. Testing Strategy

### 8.1 Unit Tests
- ArUco detection accuracy (known marker → expected pose)
- PnP pose estimation correctness
- Dual camera fusion algorithm
- Visual servoing control law (error → velocity)
- State machine transitions

### 8.2 Integration Tests
- ArUco detector node with recorded camera images
- Visual servoing controller with simulated marker poses
- Full docking sequence in Gazebo simulation
- Failure recovery scenarios

### 8.3 Field Tests (Outdoor)
- Docking success rate (target: >95%)
- Docking precision (target: ±2-5mm)
- Robustness to lighting (cloudy, sunny, twilight)
- Robustness to light rain (IP65+ cameras)
- Marker detection range (0.5m to 3m)
- Docking time (target: <30s from approach zone)

---

## 9. Performance Targets

| Metric | Target | Validation |
|--------|--------|------------|
| Docking Precision (Outdoor) | ±2-5mm (x, y) | Motion capture or manual measurement |
| Docking Precision (Indoor) | ±1mm (x, y) | Motion capture or manual measurement |
| Docking Success Rate | >95% | Field tests (100 trials) |
| Marker Detection Range | 0.5m - 3m | Detection tests |
| Marker Detection Rate | 10 Hz | Timestamp analysis |
| Visual Servoing Control Rate | 10 Hz | Timestamp analysis |
| Docking Time (APPROACH→DOCKED) | <30s | Field tests |
| Marker Lost Recovery Time | <5s | Failure recovery tests |

---

## 10. Implementation Phases

### Phase 1: ArUco Detection (Sprints 1-2)
- ✅ Single camera ArUco detection
- ✅ Pose estimation (PnP)
- ✅ Dual camera fusion
- ✅ Unit tests

### Phase 2: Visual Servoing (Sprints 3-4)
- ✅ PBVS controller implementation
- ✅ PID tuning (simulation)
- ✅ Stability verification
- ✅ Integration with swerve drive

### Phase 3: State Machine (Sprints 5-6)
- ✅ Docking coordinator node
- ✅ State transitions
- ✅ Nav2 integration (approach phase)
- ✅ Failure handling

### Phase 4: Field Validation (Sprints 7-8)
- ✅ Outdoor docking tests
- ✅ Precision tuning
- ✅ Robustness testing (lighting, weather)
- ✅ Success rate validation (100 trials)

---

**Document Status:** Draft
**Implementation Status:** Not Started
**Approvals Required:** Docking Engineer, Vision Engineer, Navigation Lead, Safety Engineer
