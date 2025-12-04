# System Architecture - Current & Proposed

**Last Updated:** December 4, 2025
**Document Version:** 1.1
**Status:** Current (52% complete - 47/91 requirements) → Proposed (4 phases to 100%)

**📋 Related Documents:**
- [REQUIREMENTS.md](./REQUIREMENTS.md) - All 91 requirements
- [REQUIREMENTS-TRACEABILITY.md](./REQUIREMENTS-TRACEABILITY.md) - Requirement-to-phase mapping
- [IMPLEMENTATION-GUIDE.md](./IMPLEMENTATION-GUIDE.md) - Implementation plan
- [ISSUES-AND-FIXES.md](./ISSUES-AND-FIXES.md) - Known issues

---

## Table of Contents

### Part 1: Current Architecture (What Exists)
1. [System Overview](#1-system-overview)
2. [Component Architecture](#2-component-architecture)
3. [Communication Patterns](#3-communication-patterns)
4. [Key Algorithms](#4-key-algorithms)
5. [Data Flow](#5-data-flow)

### Part 2: Proposed Architecture (Phases 1-4)
6. [Safety Architecture](#6-proposed-safety-architecture-phase-1)
7. [State Management](#7-proposed-state-management-phase-1)
8. [ROS 2 Best Practices](#8-proposed-ros-2-improvements-phase-3)
9. [Testing Infrastructure](#9-proposed-testing-architecture-phase-2)
10. [Deployment Architecture](#10-proposed-deployment-architecture-phase-4)

---

# Part 1: Current Architecture

## 1. System Overview

### 1.1 High-Level Architecture

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

### 1.2 Technology Stack

| Layer | Technology |
|-------|-----------|
| **Framework** | ROS 2 Humble (LTS) |
| **OS** | Ubuntu 22.04 LTS |
| **Languages** | C++17, Python 3.10 |
| **Navigation** | Nav2 (industry standard) |
| **SLAM** | RTAB-Map (visual SLAM) |
| **Vision** | OpenCV 4.x (ArUco markers) |
| **Build** | colcon |
| **Hardware** | Phidget22 (motor control) |

### 1.3 Control Hierarchy (5 Layers)

```
Layer 1: Mission Control (multigo_master)
         ├─→ nav_master_node: User interface
         ├─→ Action orchestration
         └─→ Safety confirmations

Layer 2: Task Execution (multigo_navigation)
         ├─→ nav_goal_node: Approach server
         ├─→ nav_docking_node: Dock server
         └─→ State management (BASIC)

Layer 3: Motion Planning (Nav2 + nav_control)
         ├─→ Nav2 global planner (NavFn)
         ├─→ Nav2 local planner (DWB)
         ├─→ nav_control: Kinematics
         └─→ Costmap management

Layer 4: Motion Execution (mecanum_wheels)
         ├─→ Inverse kinematics
         ├─→ Per-wheel PID
         └─→ Phidget22 interface

Layer 5: Hardware
         └─→ 4× BLDC motors, mecanum wheels, encoders
```

---

## 2. Component Architecture

### 2.1 Master Control (`multigo_master`)

**Purpose:** High-level mission orchestration and user interaction

**Key Node:** `nav_master_node`

**Responsibilities:**
- User confirmation workflow
- Action client orchestration (Approach → Dock)
- High-level error handling

**Action Clients:**
1. `/approach` - Triggers navigation to docking zone
2. `/dock` - Triggers precision docking sequence

**Current Workflow:**
```cpp
// Simplified flow
1. User triggers approach
2. nav_master: "Approach docking station? (y/n)"
3. User confirms → call /approach action
4. Wait for approach completion
5. nav_master: "Begin docking? (y/n)"
6. User confirms → call /dock action
7. Wait for dock completion
8. Report success/failure
```

**Issues:**
- ❌ No explicit state machine (hard to track robot state)
- ❌ Sequential only (cannot handle complex flows)
- ❌ No error recovery beyond cancellation

---

### 2.2 Navigation (`multigo_navigation`)

#### 2.2.1 Approach Goal Calculation (`nav_goal`)

**Node:** `nav_goal_node`

**Purpose:** Detect markers, calculate approach position, send goal to Nav2

**File:** `src/nav_goal/nav_goal.cpp`

**Algorithm:**
```cpp
1. Subscribe to /aruco_detect/markers_left
2. Detect desired marker (ID 20)
3. Get marker pose in camera frame
4. Transform to map frame using TF2
5. Calculate offset goal (marker_pos + offset * marker_direction)
   - offset = 0.305m (configurable via aruco_distance_offset)
6. Publish /goal_pose (PoseStamped)
7. Nav2 navigates to goal
8. Report success when Nav2 completes
```

**Accuracy:** ±5cm (sufficient for Nav2 coarse navigation)

**Topics:**
- **Subscribes:** `/aruco_detect/markers_left` (PoseArray)
- **Publishes:** `/goal_pose` (PoseStamped)

**Actions:**
- **Server:** `/approach` (nav_interface::action::Approach)
- **Client:** `/navigate_to_pose` (nav2_msgs::action::NavigateToPose)

---

#### 2.2.2 Precision Docking (`nav_docking`)

**Node:** `nav_docking_node`

**Purpose:** Visual servoing for ±1mm precision docking using PID control

**File:** `src/nav_docking/nav_docking.cpp`

**Two-Stage Docking:**

**Stage 1: Single Front Marker (distance > 0.7m)**
```cpp
void frontMarkerCmdVelPublisher() {
    // Use left marker (ID 20) only
    double error_dist = current_distance - target_distance;
    double error_y = marker_y_position;  // Centering
    double error_yaw = marker_yaw;

    // PID control for each axis
    integral_dist_ += error_dist * dt;  // Accumulate (BUG FIXED in Phase 1)
    double derivative = (error_dist - prev_error_dist_) / dt;

    double vel_x = Kp_dist * error_dist
                 + Ki_dist * integral_dist_
                 + Kd_dist * derivative;

    // Similar for Y and Yaw
    publishVelocity(vel_x, vel_y, vel_yaw);
}
```

**Stage 2: Dual Markers (distance < 0.7m)**
```cpp
void dualMarkerCmdVelPublisher() {
    // Use both markers for maximum precision
    double center_x = (left_marker_x + right_marker_x) / 2;
    double center_y = (left_marker_y + right_marker_y) / 2;

    // Calculate errors from center
    double error_dist = center_x - target_distance_dual;
    double error_y = center_y;
    double error_yaw = calculateYawFromTwoMarkers();

    // Same PID control, higher precision
    // ...
}
```

**Stage Transition:** When `distance < dual_aruco_distance_th (0.7m)` AND both markers visible

**Docking Confirmation (Two-Step Verification):**
```cpp
void checkDockingCompletion() {
    if (distance < aruco_close_th && position_stable) {
        if (!first_confirmation_received_) {
            first_confirmation_received_ = true;
            wait(3.0);  // Stability check
        } else if (!second_confirmation_received_) {
            if (still_within_threshold) {
                second_confirmation_received_ = true;
                reportSuccess();
            }
        }
    }
}
```

**PID Parameters:**
```yaml
# Distance control
Kp_dist: 0.5
Ki_dist: 0.1
Kd_dist: 0.05

# Y-axis centering
Kp_y: 0.8
Ki_y: 0.05
Kd_y: 0.1

# Rotation control
Kp_yaw: 0.6
Ki_yaw: 0.08
Kd_yaw: 0.12
```

**Thresholds:**
```yaml
aruco_distance_offset: 0.305          # Stage 1 target distance
dual_aruco_distance_th: 0.700         # Switch to Stage 2
aruco_distance_offset_dual: 0.430     # Stage 2 target distance
aruco_close_th: 0.42                  # Final approach threshold
```

**Topics:**
- **Subscribes:**
  - `/aruco_detect/markers_left` (PoseArray)
  - `/aruco_detect/markers_right` (PoseArray)
- **Publishes:** `/cmd_vel_final` (Twist)

**Actions:**
- **Server:** `/dock` (nav_interface::action::Dock)

**Known Issues (Fixed in Phase 1):**
- ⚠️ **CRIT-01:** PID integral not accumulating (line 197)
- ⚠️ **CRIT-02:** Dual marker distance calculation bug (line 387, 503)
- ⚠️ **HIGH-01:** Uninitialized variables

---

#### 2.2.3 Velocity Control (`nav_control`)

**Node:** `nav_control_node`

**Purpose:** Adjust velocity commands based on operation mode (rotation center)

**File:** `src/nav_control/nav_control.cpp`

**Modes:**
1. **SOLO** - Center rotation (normal navigation)
2. **DOCKING** - Forward rotation (precision docking)
3. **COMBINE_CHAIR** - Far forward rotation (wheelchair attached)

**Algorithm:**
```cpp
void adjustVelocityForMode(Twist& cmd_vel, NavigationMode mode) {
    double rotation_center_offset;

    switch(mode) {
        case SOLO:
            rotation_center_offset = 0.0;   // Rotate around center
            break;
        case DOCKING:
            rotation_center_offset = 0.25;  // 25cm forward
            break;
        case COMBINE_CHAIR:
            rotation_center_offset = 0.50;  // 50cm forward
            break;
    }

    // Adjust linear velocity to achieve desired rotation center
    cmd_vel.linear.x += cmd_vel.angular.z * rotation_center_offset;
}
```

**Topics:**
- **Subscribes:**
  - `/cmd_vel` (from Nav2)
  - `/cmd_vel_final` (from nav_docking)
  - `/navigation_mode` (mode selection)
- **Publishes:** `/cmd_vel_adjusted` (Twist)

---

#### 2.2.4 ArUco Marker Detection (`aruco_detect`)

**Node:** `aruco_detect_node`

**Purpose:** Detect ArUco markers and estimate 6DOF pose

**File:** `src/aruco_detect/aruco_detect.cpp`

**Algorithm:**
```cpp
void detectMarkers() {
    // 1. Capture image from camera
    cv::Mat image = camera_image_;

    // 2. Detect markers (OpenCV ArUco)
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(
        image,
        dictionary,  // DICT_6X6_250
        marker_corners,
        marker_ids
    );

    // 3. Estimate pose for each marker
    for (size_t i = 0; i < marker_ids.size(); i++) {
        cv::Vec3d rvec, tvec;
        cv::aruco::estimatePoseSingleMarkers(
            marker_corners[i],
            marker_size,      // 0.15m (physical size)
            camera_matrix,    // From calibration
            dist_coeffs,      // From calibration
            rvec, tvec
        );

        // 4. Convert OpenCV frame to ROS frame
        // OpenCV: X-right, Y-down, Z-forward
        // ROS:    X-forward, Y-left, Z-up
        geometry_msgs::msg::Pose marker_pose = convertToROSFrame(rvec, tvec);

        // 5. Publish pose
        publishMarkerPose(marker_ids[i], marker_pose);
    }
}
```

**Marker Configuration:**
- **Left marker:** ID 20 (front, used for approach)
- **Right marker:** ID 21 (used for dual-marker precision)
- **Marker size:** 0.15m (15cm)
- **Dictionary:** DICT_6X6_250

**Topics:**
- **Subscribes:**
  - `/camera/color/image_raw_left` (Image)
  - `/camera/color/image_raw_right` (Image)
- **Publishes:**
  - `/aruco_detect/markers_left` (PoseArray)
  - `/aruco_detect/markers_right` (PoseArray)
  - TF transforms: `camera → aruco_marker_20`, `aruco_marker_21`

---

### 2.3 Third-Party Integration

#### 2.3.1 Nav2 (Navigation2 Stack)

**Purpose:** Industry-standard autonomous navigation framework

**What Nav2 Provides:**
- Global path planning (NavFn planner)
- Local trajectory planning (DWB local planner)
- Costmap management (static, obstacle, inflation layers)
- Recovery behaviors (spin, backup, wait)
- Behavior tree execution

**Integration Points:**
```
MultiGo → Nav2:
- Input: /goal_pose (PoseStamped)
- Input: /map (OccupancyGrid from RTAB-Map)
- Input: /scan (LaserScan from LiDAR)
- Input: /tf (transforms)

Nav2 → MultiGo:
- Output: /cmd_vel (Twist) → nav_control
- Action: /navigate_to_pose
```

**Key Configuration (`nav2_params.yaml` - 357 lines):**

```yaml
controller_server:
  FollowPath:
    plugin: "dwb_core::DWBLocalPlanner"
    max_vel_x: 0.26        # m/s (slow walk)
    max_vel_theta: 1.0     # rad/s
    max_vel_y: 0.0         # ⚠️ DISABLED (should be 0.15 for holonomic)
    acc_lim_x: 2.5         # m/s²
    acc_lim_theta: 3.2     # rad/s²

planner_server:
  GridBased:
    plugin: "nav2_navfn_planner::NavfnPlanner"
    tolerance: 0.5         # Goal tolerance (meters)
    use_astar: false       # Dijkstra algorithm
    allow_unknown: true    # Can plan through unexplored areas

global_costmap:
  robot_radius: 0.28       # meters
  resolution: 0.05         # 5cm grid cells
  inflation_radius: 0.55   # Safety margin
  plugins:
    - "static_layer"       # From map
    - "obstacle_layer"     # From LiDAR
    - "inflation_layer"    # Safety buffer

local_costmap:
  width: 3.0               # 3m × 3m planning horizon
  height: 3.0
  update_frequency: 5.0    # Hz
```

**Critical Issue:**
- ⚠️ **CRIT-09:** Holonomic motion disabled (`max_vel_y = 0`)
- **Impact:** Mecanum wheels underutilized, cannot move sideways during navigation
- **Fix (Phase 3):** Configure DWB for holonomic motion

---

#### 2.3.2 RTAB-Map (Visual SLAM)

**Purpose:** Real-Time Appearance-Based Mapping and Localization

**What RTAB-Map Provides:**
- Visual SLAM (camera-based)
- Loop closure detection (corrects drift)
- 3D point cloud mapping
- Long-term memory (remembers environment across sessions)
- 2D occupancy grid for Nav2

**Inputs:**
- RGB camera images (left + right)
- LiDAR scans (`/scan`)
- Wheel odometry (`/odom`)

**Outputs:**
- `/map` (OccupancyGrid) → Nav2
- `/tf` transform `map → odom` (corrected localization)
- 3D point cloud map (visualization)

**Localization Strategy:**
```
Distance > 5m    : RTAB-Map only (markers not visible)
1m < Distance < 5m : RTAB-Map primary, ArUco available
Distance < 1m    : Switch to ArUco visual servoing
```

**Why Dual Localization?**
- **RTAB-Map:** Global consistency, ±5-10cm accuracy, unlimited range
- **ArUco:** Local precision, ±1mm accuracy, limited range (0.5-5m)
- **Best of both worlds**

---

### 2.4 Motion Control

#### 2.4.1 Mecanum Wheels (`mecanum_wheels`)

**Node:** `mecanum_wheels_node`

**Purpose:** Convert Twist commands to wheel velocities and control motors

**Algorithm:**
```cpp
// Inverse kinematics for mecanum wheels
void twistToWheelVelocities(Twist cmd_vel) {
    double vx = cmd_vel.linear.x;
    double vy = cmd_vel.linear.y;
    double omega = cmd_vel.angular.z;

    // Mecanum wheel kinematics
    double wheel_fl = (1/r) * (vx - vy - (lx + ly) * omega);
    double wheel_fr = (1/r) * (vx + vy + (lx + ly) * omega);
    double wheel_rl = (1/r) * (vx + vy - (lx + ly) * omega);
    double wheel_rr = (1/r) * (vx - vy + (lx + ly) * omega);

    // Per-wheel PID control
    for (int i = 0; i < 4; i++) {
        motor_command[i] = wheelPID(target[i], current[i]);
    }

    // Send to Phidget motor controllers
    sendMotorCommands(motor_command);
}
```

**Parameters:**
```yaml
WHEEL_BASE_LENGTH: 0.40m   # Front-to-rear distance
WHEEL_BASE_WIDTH: 0.30m    # Left-to-right distance
WHEEL_DIAMETER: 0.0762m    # 3 inches
```

**Topics:**
- **Subscribes:** `/cmd_vel_adjusted` (Twist)
- **Publishes:** `/odom` (Odometry)

---

## 3. Communication Patterns

### 3.1 Topic Map

| Topic | Type | Publisher | Subscriber(s) | Purpose |
|-------|------|-----------|---------------|---------|
| `/camera/color/image_raw_left` | Image | camera_driver | aruco_detect | Left camera feed |
| `/camera/color/image_raw_right` | Image | camera_driver | aruco_detect | Right camera feed |
| `/scan` | LaserScan | lidar_driver | Nav2, RTAB-Map | LiDAR data |
| `/aruco_detect/markers_left` | PoseArray | aruco_detect | nav_goal, nav_docking | Detected markers |
| `/aruco_detect/markers_right` | PoseArray | aruco_detect | nav_docking | Detected markers |
| `/goal_pose` | PoseStamped | nav_goal | Nav2 | Navigation goal |
| `/cmd_vel` | Twist | Nav2 | nav_control | Navigation commands |
| `/cmd_vel_final` | Twist | nav_docking | nav_control | Docking commands |
| `/cmd_vel_adjusted` | Twist | nav_control | mecanum_wheels | Final commands |
| `/odom` | Odometry | mecanum_wheels | Nav2, RTAB-Map | Wheel odometry |
| `/map` | OccupancyGrid | RTAB-Map | Nav2 | 2D grid map |

### 3.2 Action Map

| Action | Type | Server | Client | Purpose |
|--------|------|--------|--------|---------|
| `/approach` | Approach | nav_goal | nav_master | Navigate to docking zone |
| `/dock` | Dock | nav_docking | nav_master | Precision docking |
| `/navigate_to_pose` | NavigateToPose | Nav2 bt_navigator | nav_goal | Nav2 navigation |

### 3.3 TF Tree

```
map (RTAB-Map)
 └─→ odom (wheel odometry)
      └─→ base_link (robot center)
           ├─→ base_footprint (ground projection)
           ├─→ camera_left_link
           │    └─→ aruco_marker_20
           ├─→ camera_right_link
           │    └─→ aruco_marker_21
           └─→ laser_link (LiDAR)
```

---

## 4. Key Algorithms

### 4.1 PID Control (Docking)

**Purpose:** Smooth, stable approach to target position

**Algorithm:**
```cpp
double pidControl(double error, double Kp, double Ki, double Kd, double dt) {
    // Proportional: React to current error
    double P = Kp * error;

    // Integral: Eliminate steady-state error
    integral_ += error * dt;  // ⚠️ Must accumulate! (CRIT-01)
    double I = Ki * integral_;

    // Derivative: Dampen oscillations
    double derivative = (error - prev_error_) / dt;
    double D = Kd * derivative;

    prev_error_ = error;

    return P + I + D;
}
```

**Tuning:**
- **Kp** (proportional): Higher = faster response, risk of overshoot
- **Ki** (integral): Eliminates steady-state error, risk of windup
- **Kd** (derivative): Dampens oscillations, sensitive to noise

**Current Tuning (works well):**
- Distance: Kp=0.5, Ki=0.1, Kd=0.05
- Centering: Kp=0.8, Ki=0.05, Kd=0.1
- Rotation: Kp=0.6, Ki=0.08, Kd=0.12

---

### 4.2 Dual Marker Centering

**Purpose:** Calculate robot position relative to centerline between two markers

**Algorithm:**
```cpp
void calculateCenterPosition() {
    // Get both marker positions
    double left_x = left_marker.pose.position.x;
    double right_x = right_marker.pose.position.x;

    // Calculate center (average)
    double center_x = (left_x + right_x) / 2;  // ⚠️ Parentheses critical! (CRIT-02)

    // Same for Y and Yaw
    double center_y = (left_y + right_y) / 2;
    double center_yaw = (left_yaw + right_yaw) / 2;

    // Calculate errors
    double error_dist = center_x - target_distance;
    double error_y = center_y;  // Should be 0 (centered)
    double error_yaw = center_yaw;  // Should be 0 (aligned)
}
```

**Critical Bug (CRIT-02):**
```cpp
// WRONG (operator precedence issue):
double center = (left_x) + (right_x) / 2;  // = left + (right/2)

// CORRECT:
double center = (left_x + right_x) / 2;    // = (left + right) / 2
```

**Impact:** Without fix, robot docks off-center by ~5-10cm

---

## 5. Data Flow

### 5.1 Complete Docking Workflow

```
[User Command]
    │
    ↓
┌───────────────────────────────────────┐
│ nav_master: "Approach? (y/n)"         │
└───────────────────────────────────────┘
    │ (User confirms: y)
    ↓
┌───────────────────────────────────────┐
│ STAGE 1: APPROACH (nav_goal)          │
│                                       │
│ 1. Detect ArUco marker (ID 20)        │
│ 2. Transform to map frame             │
│ 3. Calculate offset goal (0.305m)     │
│ 4. Publish /goal_pose                 │
│ 5. Nav2 navigates (~30s)              │
│ 6. Robot stops ~30cm from marker      │
│                                       │
│ Accuracy: ±5cm                        │
└───────────────────────────────────────┘
    │ (Approach complete)
    ↓
┌───────────────────────────────────────┐
│ nav_master: "Begin docking? (y/n)"    │
└───────────────────────────────────────┘
    │ (User confirms: y)
    ↓
┌───────────────────────────────────────┐
│ STAGE 2: ALIGNMENT (nav_docking)      │
│ Single Marker Phase                   │
│                                       │
│ 1. Detect front marker (ID 20)        │
│ 2. PID control: X, Y, Yaw             │
│ 3. Approach until distance < 0.7m     │
│ 4. Both markers visible               │
│                                       │
│ Accuracy: ±1cm                        │
│ Duration: ~15s                        │
└───────────────────────────────────────┘
    │ (Both markers visible)
    ↓
┌───────────────────────────────────────┐
│ STAGE 3: PRECISION (nav_docking)      │
│ Dual Marker Phase                     │
│                                       │
│ 1. Detect both markers (ID 20, 21)    │
│ 2. Calculate center position          │
│ 3. PID control for centering          │
│ 4. Approach until distance < 0.42m    │
│                                       │
│ Accuracy: ±1mm (target)               │
│ Duration: ~10s                        │
└───────────────────────────────────────┘
    │ (Distance < 0.42m)
    ↓
┌───────────────────────────────────────┐
│ STAGE 4: VERIFICATION                 │
│                                       │
│ 1. Check position stable (3 seconds)  │
│ 2. First confirmation check           │
│ 3. Wait 3 more seconds                │
│ 4. Second confirmation check          │
│ 5. Report success                     │
└───────────────────────────────────────┘
    │
    ↓
[Docking Complete!]
```

**Total Duration:** ~60-70 seconds (30s approach + 15s alignment + 15s precision + 6s verification)

---

# Part 2: Proposed Architecture (Phases 1-4)

## 6. Proposed Safety Architecture (Phase 1)

**Requirements Addressed:** SR-1.1, SR-1.2, SR-2.2, SR-5.1, SR-5.2, SR-6.1 ([REQUIREMENTS.md](./REQUIREMENTS.md))
**Implementation:** [IMPLEMENTATION-GUIDE.md Phase 1 Weeks 2-4](./IMPLEMENTATION-GUIDE.md#phase-1-critical-bugs--safety)
**Issues Fixed:** CRIT-03 (LiDAR), CRIT-05 (E-Stop), CRIT-08 (Geofencing)

### 6.1 Problem Statement

**Current Issues:**
- ❌ No LiDAR monitoring during docking (blind to obstacles) - **SR-2.2, DR-5.3**
- ❌ No emergency stop mechanism - **SR-1.2**
- ❌ No safety layer with override authority - **SR-5.1**
- ❌ Safety logic scattered across nodes
- ❌ No geofencing/keep-out zones - **SR-6.1**

**Result:** System is **NOT** safe for production deployment

### 6.2 Proposed Safety Layer

**Requirements:** SR-5.1, SR-5.2 ([REQUIREMENTS.md](./REQUIREMENTS.md))
**Design Principle:** Orthogonal safety layer with override authority over ALL motion

```
┌─────────────────────────────────────────────────────────┐
│           SAFETY SUPERVISOR (Override Authority)        │
│                                                         │
│  States:                                                │
│  ┌──────┐ ┌────────┐ ┌────────┐ ┌──────────────────┐  │
│  │ SAFE │→│CAUTION │→│ UNSAFE │→│ EMERGENCY_STOP   │  │
│  └──────┘ └────────┘ └────────┘ └──────────────────┘  │
│                                                         │
│  Monitors:                                              │
│  • LiDAR scan (obstacle distance)                       │
│  • ArUco markers (visibility)                           │
│  • Robot state (position, velocity)                     │
│  • System health (node status)                          │
│  • Manual e-stop button                                 │
│                                                         │
│  Enforcers:                                             │
│  • /multigo/safety/emergency_stop (Bool)                │
│  • /multigo/safety/speed_limit (Float32)                │
│  • /multigo/safety/state (String)                       │
└─────────────────────────────────────────────────────────┘
                         │
        ┌────────────────┼────────────────┐
        ↓                ↓                ↓
  [nav_docking]    [nav_control]   [mecanum_wheels]
     (checks)         (checks)         (checks)
```

**Implementation:**

```cpp
class SafetySupervisor : public rclcpp::Node {
public:
    enum class SafetyState {
        SAFE,           // All clear
        CAUTION,        // Warning, reduce speed
        UNSAFE,         // Danger, stop motion
        EMERGENCY_STOP  // Manual e-stop active
    };

private:
    SafetyState current_state_ = SafetyState::SAFE;

    // Monitors
    rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<Bool>::SharedPtr estop_button_sub_;
    rclcpp::Subscription<PoseArray>::SharedPtr markers_sub_;

    // Enforcers
    rclcpp::Publisher<Bool>::SharedPtr safety_stop_pub_;
    rclcpp::Publisher<Float32>::SharedPtr speed_limit_pub_;
    rclcpp::Publisher<String>::SharedPtr state_pub_;

public:
    void scanCallback(LaserScan::SharedPtr msg) {
        double min_dist = *std::min_element(msg->ranges.begin(), msg->ranges.end());

        if (min_dist < 0.15) {  // 15cm = DANGER
            transitionTo(SafetyState::EMERGENCY_STOP);
        } else if (min_dist < 0.30) {  // 30cm = CAUTION
            transitionTo(SafetyState::CAUTION);
        } else if (min_dist < 0.50) {  // 50cm = SAFE but monitored
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
                publishSpeedLimit(1.0);      // 100% speed
                publishSafetyStop(false);
                break;

            case SafetyState::CAUTION:
                publishSpeedLimit(0.5);      // 50% speed
                RCLCPP_WARN(get_logger(), "CAUTION: Obstacle detected, reducing speed");
                break;

            case SafetyState::UNSAFE:
                publishSpeedLimit(0.2);      // 20% speed (crawl)
                RCLCPP_WARN(get_logger(), "UNSAFE: Obstacle very close");
                break;

            case SafetyState::EMERGENCY_STOP:
                publishSpeedLimit(0.0);      // STOP
                publishSafetyStop(true);
                RCLCPP_ERROR(get_logger(), "EMERGENCY STOP ACTIVATED!");
                break;
        }

        publishState(stateToString(new_state));
    }
};
```

**Integration with Motion Nodes:**

```cpp
// In nav_docking, nav_control, mecanum_wheels:
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
            RCLCPP_ERROR(get_logger(), "E-STOP! Halting all motion");
            publishZeroVelocity();
            cancelCurrentAction();
        }
    }

    void speedLimitCallback(Float32::SharedPtr msg) {
        speed_limit_multiplier_ = msg->data;
    }

    void publishVelocity(Twist cmd) {
        // Safety check 1: E-stop active?
        if (emergency_stop_active_) {
            RCLCPP_WARN(get_logger(), "Motion blocked by e-stop");
            return;  // Don't publish
        }

        // Safety check 2: Apply speed limit
        cmd.linear.x *= speed_limit_multiplier_;
        cmd.linear.y *= speed_limit_multiplier_;
        cmd.angular.z *= speed_limit_multiplier_;

        cmd_vel_pub_->publish(cmd);
    }
};
```

**Safety Topics:**
- `/multigo/safety/emergency_stop` (Bool) - RELIABLE, TRANSIENT_LOCAL, KEEP_ALL
- `/multigo/safety/speed_limit` (Float32) - RELIABLE
- `/multigo/safety/state` (String) - For monitoring

**Testing:**
```bash
# Test e-stop
ros2 topic pub /multigo/safety/emergency_stop std_msgs/Bool "data: true"
# Verify: Robot stops immediately (<100ms)

# Test speed limit
ros2 topic pub /multigo/safety/speed_limit std_msgs/Float32 "data: 0.3"
# Verify: Robot slows to 30% speed

# Test obstacle detection
# Place obstacle 20cm from robot during docking
# Verify: Robot transitions to EMERGENCY_STOP and halts
```

**Effort:** 40 hours (design 8h + implement 24h + test 8h)

**Priority:** 🔴 **CRITICAL** - Cannot deploy without this

---

### 6.3 Geofencing (Keep-Out Zones)

**Purpose:** Define virtual boundaries robot cannot cross

**Configuration:** `config/safety_zones.yaml`

```yaml
keep_out_zones:
  - id: "elevator_shaft"
    polygon: [[5.0, 10.0], [6.0, 10.0], [6.0, 11.0], [5.0, 11.0]]
    reason: "Dangerous: elevator shaft"

  - id: "stairs_north"
    polygon: [[2.0, 8.0], [3.0, 8.0], [3.0, 9.0], [2.0, 9.0]]
    reason: "Cliff hazard: stairs"

allowed_zones:
  - id: "hospital_floor_2"
    polygon: [[0, 0], [50, 0], [50, 30], [0, 30]]
    reason: "Approved operating area"
```

**Implementation:**

```cpp
class SafetyZoneManager : public rclcpp::Node {
private:
    std::vector<Polygon> keep_out_zones_;
    std::vector<Polygon> allowed_zones_;

public:
    void loadZones(std::string yaml_file) {
        // Parse YAML and load polygons
        YAML::Node config = YAML::LoadFile(yaml_file);
        for (auto zone : config["keep_out_zones"]) {
            keep_out_zones_.push_back(parsePolygon(zone));
        }
    }

    bool isPositionSafe(double x, double y) {
        // Check 1: Not in any keep-out zone
        for (auto& zone : keep_out_zones_) {
            if (pointInPolygon(x, y, zone)) {
                RCLCPP_ERROR(get_logger(), "Position (%.2f, %.2f) in keep-out zone: %s",
                            x, y, zone.reason.c_str());
                return false;
            }
        }

        // Check 2: Inside at least one allowed zone
        bool in_allowed = false;
        for (auto& zone : allowed_zones_) {
            if (pointInPolygon(x, y, zone)) {
                in_allowed = true;
                break;
            }
        }

        if (!in_allowed) {
            RCLCPP_ERROR(get_logger(), "Position (%.2f, %.2f) outside allowed zones", x, y);
        }

        return in_allowed;
    }

private:
    bool pointInPolygon(double x, double y, const Polygon& poly) {
        // Ray casting algorithm for point-in-polygon test
        // Standard computational geometry algorithm
        // ...
    }
};
```

**Integration:**
- Safety supervisor checks robot position every 100ms
- If violation detected → transition to EMERGENCY_STOP
- Navigation goals rejected if inside keep-out zone
- Path planning constrained to allowed zones

**Effort:** 16 hours (basic), 40 hours (full with visualization)

---

## 7. Proposed State Management (Phase 1)

**Requirements Addressed:** MCR-3.1 ([REQUIREMENTS.md](./REQUIREMENTS.md))
**Implementation:** [IMPLEMENTATION-GUIDE.md Phase 1 Week 2](./IMPLEMENTATION-GUIDE.md#week-2-safety-architecture-design)

### 7.1 Problem Statement

**Current Issues:**
- ❌ No explicit state tracking ("What state is the robot in?") - **MCR-3.1**
- ❌ Hard to debug ("Where did it fail?")
- ❌ Hard to extend (adding new steps requires code changes)
- ❌ No visualization of behavior

### 7.2 Proposed State Machine

**Implementation:**

```cpp
enum class MissionState {
    IDLE,
    WAITING_APPROACH_CONFIRMATION,
    NAVIGATING_TO_GOAL,
    WAITING_DOCK_CONFIRMATION,
    DOCKING,
    DOCKED,
    UNDOCKING,          // Future
    TRANSPORTING,       // Future
    ERROR
};

class MissionStateMachine : public rclcpp::Node {
private:
    MissionState current_state_ = MissionState::IDLE;

    // State change publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

    // Action clients
    rclcpp_action::Client<Approach>::SharedPtr approach_client_;
    rclcpp_action::Client<Dock>::SharedPtr dock_client_;

public:
    MissionStateMachine() : Node("mission_state_machine") {
        // Setup action clients
        approach_client_ = rclcpp_action::create_client<Approach>(this, "approach");
        dock_client_ = rclcpp_action::create_client<Dock>(this, "dock");

        // Setup state publisher
        state_pub_ = create_publisher<std_msgs::msg::String>("/multigo/mission/state", 10);

        // Main state machine loop (10 Hz)
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MissionStateMachine::update, this)
        );
    }

    void update() {
        switch (current_state_) {
            case MissionState::IDLE:
                // Wait for user to request mission
                if (approach_requested_) {
                    transitionTo(MissionState::WAITING_APPROACH_CONFIRMATION);
                }
                break;

            case MissionState::WAITING_APPROACH_CONFIRMATION:
                // Display confirmation prompt
                displayPrompt("Approach docking station? (y/n)");

                if (user_confirmed_) {
                    transitionTo(MissionState::NAVIGATING_TO_GOAL);
                    sendApproachGoal();
                } else if (user_cancelled_) {
                    transitionTo(MissionState::IDLE);
                }
                break;

            case MissionState::NAVIGATING_TO_GOAL:
                // Check approach action status
                if (approach_succeeded_) {
                    transitionTo(MissionState::WAITING_DOCK_CONFIRMATION);
                } else if (approach_failed_) {
                    transitionTo(MissionState::ERROR);
                    logError("Approach failed: " + approach_error_message_);
                } else if (user_cancelled_) {
                    cancelApproachGoal();
                    transitionTo(MissionState::IDLE);
                }
                break;

            case MissionState::WAITING_DOCK_CONFIRMATION:
                displayPrompt("Begin precision docking? (y/n)");

                if (user_confirmed_) {
                    transitionTo(MissionState::DOCKING);
                    sendDockGoal();
                } else if (user_cancelled_) {
                    transitionTo(MissionState::IDLE);
                }
                break;

            case MissionState::DOCKING:
                // Check dock action status
                if (dock_succeeded_) {
                    transitionTo(MissionState::DOCKED);
                    displaySuccess("Docking complete!");
                } else if (dock_failed_) {
                    transitionTo(MissionState::ERROR);
                    logError("Docking failed: " + dock_error_message_);
                } else if (user_cancelled_) {
                    cancelDockGoal();
                    transitionTo(MissionState::IDLE);
                }
                break;

            case MissionState::DOCKED:
                // Wait for next command (undock, transport, etc.)
                if (undock_requested_) {
                    transitionTo(MissionState::UNDOCKING);  // Future
                }
                break;

            case MissionState::ERROR:
                // Display error, wait for user to acknowledge
                if (user_acknowledged_error_) {
                    transitionTo(MissionState::IDLE);
                }
                break;
        }
    }

    void transitionTo(MissionState new_state) {
        if (new_state == current_state_) return;

        RCLCPP_INFO(get_logger(), "State transition: %s -> %s",
                    stateToString(current_state_),
                    stateToString(new_state));

        // Exit actions for old state
        onStateExit(current_state_);

        // Update state
        current_state_ = new_state;

        // Entry actions for new state
        onStateEntry(new_state);

        // Publish for monitoring
        std_msgs::msg::String state_msg;
        state_msg.data = stateToString(new_state);
        state_pub_->publish(state_msg);
    }

    void onStateEntry(MissionState state) {
        // Entry actions when entering a state
        switch (state) {
            case MissionState::NAVIGATING_TO_GOAL:
                RCLCPP_INFO(get_logger(), "Starting navigation to goal...");
                break;
            case MissionState::DOCKING:
                RCLCPP_INFO(get_logger(), "Starting precision docking...");
                break;
            case MissionState::DOCKED:
                RCLCPP_INFO(get_logger(), "Docking complete, robot is docked");
                break;
            case MissionState::ERROR:
                RCLCPP_ERROR(get_logger(), "Entered ERROR state");
                break;
            default:
                break;
        }
    }

    void onStateExit(MissionState state) {
        // Cleanup actions when leaving a state
        switch (state) {
            case MissionState::NAVIGATING_TO_GOAL:
                RCLCPP_INFO(get_logger(), "Navigation complete");
                break;
            case MissionState::DOCKING:
                RCLCPP_INFO(get_logger(), "Docking sequence complete");
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

**Benefits:**
- ✅ Always know robot state: `ros2 topic echo /multigo/mission/state`
- ✅ Easy debugging: "Robot stuck in DOCKING state"
- ✅ Better error handling: Clear error state + recovery
- ✅ Extensible: Add new states (UNDOCKING, CHARGING, etc.)
- ✅ Testable: Unit test state transitions

**Monitoring:**
```bash
# Watch state changes in real-time
ros2 topic echo /multigo/mission/state

# Trigger state transitions for testing
ros2 service call /mission/trigger_approach std_srvs/Trigger
```

**Effort:** 24 hours (implement + integrate + test)

**Priority:** 🔴 **HIGH** - Major improvement to debuggability

---

## 8. Proposed ROS 2 Improvements (Phase 3)

**Requirements Addressed:** QR-4.1, QR-5.1, QR-7.1, MR-4.1, MCR-2.3, SR-3.1, NR-2.2 ([REQUIREMENTS.md](./REQUIREMENTS.md))
**Implementation:** [IMPLEMENTATION-GUIDE.md Phase 3 Weeks 9-12](./IMPLEMENTATION-GUIDE.md#phase-3-ros-2-best-practices)
**Issues Fixed:** CRIT-09 (Holonomic), CRIT-10 (Timeouts)

### 8.1 Lifecycle Nodes

**Requirements:** QR-4.1 ([REQUIREMENTS.md](./REQUIREMENTS.md))
**Current Issue:** Nodes start immediately, no clean startup/shutdown

**Proposed:** Convert critical nodes to lifecycle nodes

**Lifecycle States:**
```
┌──────────────┐
│ Unconfigured │  (Initial state)
└──────┬───────┘
       │ configure()
       ↓
┌──────────────┐
│  Inactive    │  (Loaded config, not publishing)
└──────┬───────┘
       │ activate()
       ↓
┌──────────────┐
│   Active     │  (Fully operational)
└──────┬───────┘
       │ deactivate()
       ↓
┌──────────────┐
│  Inactive    │  (Can re-activate quickly)
└──────┬───────┘
       │ cleanup()
       ↓
┌──────────────┐
│ Unconfigured │  (Clean shutdown)
└──────────────┘
```

**Example Implementation (nav_docking as lifecycle node):**

```cpp
#include <rclcpp_lifecycle/lifecycle_node.hpp>

class NavDockingLifecycle : public rclcpp_lifecycle::LifecycleNode {
public:
    NavDockingLifecycle() : LifecycleNode("nav_docking_node") {
        RCLCPP_INFO(get_logger(), "NavDocking Lifecycle Node created");
    }

    // CONFIGURE: Load parameters, validate config
    CallbackReturn on_configure(const State&) override {
        RCLCPP_INFO(get_logger(), "Configuring NavDocking node...");

        // Load and validate parameters
        declare_parameter<double>("Kp_dist", 0.5);
        declare_parameter<double>("Ki_dist", 0.1);
        declare_parameter<double>("Kd_dist", 0.05);

        Kp_dist_ = get_parameter("Kp_dist").as_double();
        Ki_dist_ = get_parameter("Ki_dist").as_double();
        Kd_dist_ = get_parameter("Kd_dist").as_double();

        // Validate
        if (Kp_dist_ <= 0 || Ki_dist_ < 0 || Kd_dist_ < 0) {
            RCLCPP_ERROR(get_logger(), "Invalid PID parameters!");
            return CallbackReturn::FAILURE;
        }

        // Create publishers/subscribers (but don't activate yet)
        cmd_vel_pub_ = create_publisher<Twist>("/cmd_vel_final", 10);
        markers_sub_ = create_subscription<PoseArray>(
            "/aruco_detect/markers_left", 10,
            std::bind(&NavDockingLifecycle::markersCallback, this, std::placeholders::_1)
        );

        // Create action server (but don't activate)
        dock_action_server_ = rclcpp_action::create_server<Dock>(
            this, "dock",
            std::bind(&NavDockingLifecycle::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&NavDockingLifecycle::handleCancel, this, std::placeholders::_1),
            std::bind(&NavDockingLifecycle::handleAccepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(), "NavDocking configured successfully");
        return CallbackReturn::SUCCESS;
    }

    // ACTIVATE: Start publishing, enable action server
    CallbackReturn on_activate(const State&) override {
        RCLCPP_INFO(get_logger(), "Activating NavDocking node...");

        // Activate publishers (lifecycle publishers only publish when active)
        cmd_vel_pub_->on_activate();

        // Start control timer
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&NavDockingLifecycle::controlLoop, this)
        );

        RCLCPP_INFO(get_logger(), "NavDocking active, ready to dock");
        return CallbackReturn::SUCCESS;
    }

    // DEACTIVATE: Stop publishing, disable action server
    CallbackReturn on_deactivate(const State&) override {
        RCLCPP_INFO(get_logger(), "Deactivating NavDocking node...");

        // Stop control loop
        control_timer_->cancel();

        // Stop publishing commands (safety!)
        cmd_vel_pub_->on_deactivate();
        publishZeroVelocity();

        // Cancel any active docking actions
        if (current_goal_handle_) {
            current_goal_handle_->abort();
        }

        RCLCPP_INFO(get_logger(), "NavDocking deactivated");
        return CallbackReturn::SUCCESS;
    }

    // CLEANUP: Release resources
    CallbackReturn on_cleanup(const State&) override {
        RCLCPP_INFO(get_logger(), "Cleaning up NavDocking node...");

        // Reset state
        integral_dist_ = 0.0;
        integral_y_ = 0.0;
        integral_yaw_ = 0.0;

        // Clear publishers/subscribers
        cmd_vel_pub_.reset();
        markers_sub_.reset();
        dock_action_server_.reset();

        RCLCPP_INFO(get_logger(), "NavDocking cleaned up");
        return CallbackReturn::SUCCESS;
    }

    // SHUTDOWN: Final cleanup before exit
    CallbackReturn on_shutdown(const State&) override {
        RCLCPP_INFO(get_logger(), "Shutting down NavDocking node...");

        // Ensure robot stopped
        publishZeroVelocity();

        return CallbackReturn::SUCCESS;
    }
};
```

**Lifecycle Management:**

```bash
# Manage node lifecycle via CLI
ros2 lifecycle set /nav_docking_node configure
ros2 lifecycle set /nav_docking_node activate
ros2 lifecycle set /nav_docking_node deactivate
ros2 lifecycle set /nav_docking_node cleanup
```

**Automatic Lifecycle Management:**

```yaml
# lifecycle_manager configuration
lifecycle_manager:
  ros__parameters:
    node_names:
      - nav_docking_node
      - nav_goal_node
      - nav_control_node
    autostart: true
    bond_timeout: 4.0
```

**Nodes to Convert:**
1. nav_docking (12 hours)
2. nav_control (12 hours)
3. mecanum_wheels (12 hours)
4. aruco_detect (12 hours)

**Total Effort:** 48 hours

**Benefits:**
- ✅ Clean startup sequence (config → activate)
- ✅ Safe shutdown (deactivate → cleanup)
- ✅ Can restart nodes without restarting system
- ✅ Standardized node management
- ✅ Better for testing (can deactivate for unit tests)

---

### 8.2 QoS Policies (Quality of Service)

**Current Issue:** All topics use default QoS (may cause message loss)

**Proposed:** Explicit QoS policies based on message criticality

**QoS Configuration:**

```cpp
// CRITICAL: Emergency stop, safety signals
// Must NEVER lose these messages
auto qos_critical = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)       // Guaranteed delivery
    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)  // Late joiners get last message
    .history(RMW_QOS_POLICY_HISTORY_KEEP_ALL);              // Keep all messages

// CONTROL: Velocity commands, docking control
// Need latest value, old commands are stale
auto qos_control = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)       // Guaranteed delivery
    .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1);          // Only latest matters

// SENSOR: Camera images, LiDAR scans
// High frequency, okay to drop old data
auto qos_sensor = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)    // No guarantees (fast!)
    .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1);          // Only latest matters

// STATUS: Diagnostics, monitoring
// Want history for late joiners
auto qos_status = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
    .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10);
```

**Application:**

```cpp
// Emergency stop publisher (CRITICAL QoS)
auto qos_critical = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
    .history(RMW_QOS_POLICY_HISTORY_KEEP_ALL);

estop_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/multigo/safety/emergency_stop",
    qos_critical  // ← Explicit QoS
);

// Camera subscriber (SENSOR QoS)
auto qos_sensor = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/camera/color/image_raw_left",
    qos_sensor,  // ← Explicit QoS
    callback
);
```

**Topic QoS Recommendations:**

| Topic | QoS Type | Rationale |
|-------|----------|-----------|
| `/multigo/safety/emergency_stop` | CRITICAL | Must never lose e-stop signal |
| `/multigo/safety/speed_limit` | CRITICAL | Safety-critical |
| `/cmd_vel*` | CONTROL | Need latest command, old is stale |
| `/camera/color/*` | SENSOR | High frequency, best effort OK |
| `/scan` | SENSOR | High frequency, best effort OK |
| `/aruco_detect/markers_*` | CONTROL | Need reliable marker detection |
| `/goal_pose` | CONTROL | Navigation goal must arrive |
| `/multigo/mission/state` | STATUS | Want history for monitoring |

**Effort:** 8 hours (apply QoS to all topics + test)

**Benefits:**
- ✅ No lost e-stop signals
- ✅ Better real-time performance (BEST_EFFORT for sensors)
- ✅ Late joiners get critical state (TRANSIENT_LOCAL)
- ✅ Explicit communication requirements

---

### 8.3 Enhanced Action Definitions

**Current Issue:** Action results only have `success: bool`

**Proposed:** Rich feedback and detailed results

**Enhanced Dock.action:**

```
# Goal
geometry_msgs/PoseStamped target_pose  # Where to dock
float32 tolerance                      # Position tolerance (default 0.002m)
---
# Result
bool success                           # Overall success
string error_code                      # If failed: "MARKERS_LOST", "TIMEOUT", "OBSTACLE", etc.
string detailed_message                # Human-readable error description

# Final state
geometry_msgs/Pose final_pose          # Actual final position
float32 final_distance_error_x         # Error in X (meters)
float32 final_distance_error_y         # Error in Y (meters)
float32 final_yaw_error                # Error in Yaw (radians)

# Performance metrics
float32 duration_seconds               # How long docking took
int32 attempts                         # Number of retry attempts
---
# Feedback (published during execution)
string current_phase                   # "APPROACH", "ALIGN_SINGLE", "ALIGN_DUAL", "VERIFY"
float32 distance_to_target             # Current distance (meters)
float32 centering_error                # Lateral error (meters)
float32 yaw_error                      # Orientation error (radians)
bool markers_visible                   # Are markers detected?
int32 marker_count                     # How many markers visible (1 or 2)
float32 progress_percentage            # 0-100%
```

**Usage:**

```cpp
// Client side (nav_master)
void dockingFeedbackCallback(const Dock::Feedback::ConstSharedPtr feedback) {
    RCLCPP_INFO(get_logger(), "Docking progress: %d%% | Phase: %s | Distance: %.3fm",
                (int)feedback->progress_percentage,
                feedback->current_phase.c_str(),
                feedback->distance_to_target);

    // Display to user
    displayProgress(feedback->progress_percentage, feedback->current_phase);
}

void dockingResultCallback(const Dock::Result::ConstSharedPtr result) {
    if (result->success) {
        RCLCPP_INFO(get_logger(),
                    "Docking SUCCESS!\n"
                    "  Final errors: X=%.1fmm, Y=%.1fmm, Yaw=%.1f°\n"
                    "  Duration: %.1fs, Attempts: %d",
                    result->final_distance_error_x * 1000,
                    result->final_distance_error_y * 1000,
                    result->final_yaw_error * 180 / M_PI,
                    result->duration_seconds,
                    result->attempts);
    } else {
        RCLCPP_ERROR(get_logger(),
                     "Docking FAILED: %s\n"
                     "  Error code: %s\n"
                     "  Duration before failure: %.1fs",
                     result->detailed_message.c_str(),
                     result->error_code.c_str(),
                     result->duration_seconds);

        // Handle specific errors
        if (result->error_code == "MARKERS_LOST") {
            // Suggest marker placement check
        } else if (result->error_code == "TIMEOUT") {
            // Suggest reducing PID gains or increasing timeout
        }
    }
}
```

**Error Codes:**
```cpp
enum class DockingErrorCode {
    SUCCESS = 0,
    MARKERS_LOST,           // Markers disappeared during docking
    TIMEOUT,                // Took too long (>120s)
    OBSTACLE_DETECTED,      // Safety system halted docking
    POSITION_UNSTABLE,      // Could not achieve stable position
    TOLERANCE_NOT_MET,      // Final position outside tolerance
    ACTION_CANCELLED,       // User cancelled
    INTERNAL_ERROR          // Unexpected error
};
```

**Effort:** 12 hours (redesign actions + implement + update clients)

**Benefits:**
- ✅ Better error messages ("Markers lost" vs "Failed")
- ✅ Progress visualization (0-100%)
- ✅ Performance metrics (duration, accuracy)
- ✅ Easier debugging (know exact failure mode)
- ✅ Better user experience

---

### 8.4 Topic Naming Standardization

**Current Issue:** Inconsistent naming (`/cmd_vel`, `/cmd_vel_final`, `/aruco_detect/markers_left`)

**Proposed:** REP-144 standard naming convention

**Naming Convention:**
```
/[robot_namespace]/[functionality]/[topic_name]

Examples:
/multigo/navigation/cmd_vel
/multigo/docking/cmd_vel
/multigo/perception/markers/left
/multigo/perception/markers/right
/multigo/motion/cmd_vel_final
/multigo/safety/emergency_stop
/multigo/mission/state
```

**Migration Map:**

| Old Name | New Name | Notes |
|----------|----------|-------|
| `/cmd_vel` | `/multigo/navigation/cmd_vel` | Nav2 output |
| `/cmd_vel_final` | `/multigo/docking/cmd_vel` | Docking output |
| `/cmd_vel_adjusted` | `/multigo/motion/cmd_vel` | Final to motors |
| `/aruco_detect/markers_left` | `/multigo/perception/markers/left` | |
| `/aruco_detect/markers_right` | `/multigo/perception/markers/right` | |
| `/goal_pose` | `/multigo/navigation/goal_pose` | |
| `/scan` | `/multigo/sensors/scan` | LiDAR |
| `/camera/color/image_raw_left` | `/multigo/sensors/camera/left/image_raw` | |
| `/camera/color/image_raw_right` | `/multigo/sensors/camera/right/image_raw` | |

**Benefits:**
- ✅ Clear namespace organization
- ✅ Multi-robot ready (add robot_id prefix)
- ✅ Easier topic discovery (`ros2 topic list | grep multigo/safety`)
- ✅ Industry standard (REP-144)

**Effort:** 16 hours (update all nodes + launch files + test)

---

## 9. Proposed Testing Architecture (Phase 2)

**Requirements Addressed:** QR-1.1, QR-1.2, QR-1.3, QR-3.2 ([REQUIREMENTS.md](./REQUIREMENTS.md))
**Implementation:** [IMPLEMENTATION-GUIDE.md Phase 2 Weeks 5-8](./IMPLEMENTATION-GUIDE.md#phase-2-testing-infrastructure)
**Issue Fixed:** CRIT-04 (Zero test coverage)

**Current Status:** 0% test coverage ❌ - **QR-1.1**

**Proposed Goal:** 80%+ coverage ✅

### 9.1 Unit Testing

**Framework:** gtest (C++), pytest (Python)

**Target:** 40-50 unit tests

**Example Tests:**

```cpp
// test/test_pid_control.cpp
#include <gtest/gtest.h>
#include "nav_docking/pid_controller.hpp"

TEST(PIDControlTest, IntegralAccumulates) {
    PIDController pid(0.5, 0.1, 0.05);  // Kp, Ki, Kd

    double error = 1.0;
    double dt = 0.1;

    // Simulate 5 iterations
    double output = 0;
    for (int i = 0; i < 5; i++) {
        output = pid.compute(error, dt);
    }

    // Integral should be error * dt * iterations = 1.0 * 0.1 * 5 = 0.5
    EXPECT_NEAR(pid.getIntegral(), 0.5, 0.001);

    // Output should include integral term: 0.1 * 0.5 = 0.05
    EXPECT_GT(output, 0.05);  // At least the integral contribution
}

TEST(PIDControlTest, DerivativeReducesOscillation) {
    PIDController pid(0.5, 0.0, 0.5);  // High Kd

    double dt = 0.1;

    // Large initial error
    double output1 = pid.compute(1.0, dt);

    // Rapidly decreasing error (approaching target)
    double output2 = pid.compute(0.5, dt);

    // Derivative term should reduce output (damping)
    EXPECT_LT(output2, output1);
}

TEST(DualMarkerTest, CenterCalculation) {
    double left_x = 1.0, right_x = 0.8;
    double center_x = (left_x + right_x) / 2;

    EXPECT_NEAR(center_x, 0.9, 0.001);  // Should be 0.9, NOT 1.4!
}

TEST(DualMarkerTest, OperatorPrecedenceBug) {
    double left_x = 1.0, right_x = 0.8;

    // WRONG (operator precedence):
    double wrong_center = (left_x) + (right_x) / 2;  // = 1.0 + 0.4 = 1.4
    EXPECT_NEAR(wrong_center, 1.4, 0.001);

    // CORRECT:
    double correct_center = (left_x + right_x) / 2;  // = 1.8 / 2 = 0.9
    EXPECT_NEAR(correct_center, 0.9, 0.001);

    // Verify they're different!
    EXPECT_NE(wrong_center, correct_center);
}

TEST(ArucoDetectTest, OpenCVToROSConversion) {
    cv::Vec3d opencv_tvec(1.0, 0.0, 2.0);  // OpenCV frame

    // Convert to ROS frame
    geometry_msgs::msg::Pose ros_pose = convertOpenCVToROS(opencv_tvec);

    // OpenCV X -> ROS -Y, OpenCV Y -> ROS -Z, OpenCV Z -> ROS X
    EXPECT_NEAR(ros_pose.position.x, 2.0, 0.001);   // Z -> X
    EXPECT_NEAR(ros_pose.position.y, -1.0, 0.001);  // X -> -Y
    EXPECT_NEAR(ros_pose.position.z, 0.0, 0.001);   // Y -> -Z
}

TEST(StateMachineTest, StateTransitions) {
    MissionStateMachine sm;

    EXPECT_EQ(sm.getState(), MissionState::IDLE);

    // Trigger approach
    sm.handleEvent(Event::APPROACH_REQUESTED);
    EXPECT_EQ(sm.getState(), MissionState::WAITING_APPROACH_CONFIRMATION);

    // User confirms
    sm.handleEvent(Event::USER_CONFIRMED);
    EXPECT_EQ(sm.getState(), MissionState::NAVIGATING_TO_GOAL);

    // Navigation succeeds
    sm.handleEvent(Event::APPROACH_SUCCEEDED);
    EXPECT_EQ(sm.getState(), MissionState::WAITING_DOCK_CONFIRMATION);
}

TEST(SafetySupervisorTest, EmergencyStopTriggered) {
    SafetySupervisor supervisor;

    EXPECT_EQ(supervisor.getState(), SafetyState::SAFE);

    // Simulate obstacle very close (10cm)
    auto scan = createLaserScan({0.10, 0.10, 0.10});  // All ranges 10cm
    supervisor.scanCallback(std::make_shared<LaserScan>(scan));

    EXPECT_EQ(supervisor.getState(), SafetyState::EMERGENCY_STOP);
}

TEST(SafetyZoneTest, KeepOutZoneDetection) {
    SafetyZoneManager zone_manager;

    // Define keep-out zone (elevator shaft)
    Polygon elevator = {{5.0, 10.0}, {6.0, 10.0}, {6.0, 11.0}, {5.0, 11.0}};
    zone_manager.addKeepOutZone("elevator", elevator);

    // Test inside zone (unsafe)
    EXPECT_FALSE(zone_manager.isPositionSafe(5.5, 10.5));

    // Test outside zone (safe)
    EXPECT_TRUE(zone_manager.isPositionSafe(7.0, 10.5));
}
```

**Run Tests:**
```bash
# Build with tests
colcon build --symlink-install

# Run all unit tests
colcon test --packages-select nav_docking

# Run specific test
colcon test --packages-select nav_docking --ctest-args -R test_pid_control

# View results
colcon test-result --verbose
```

**Effort:** 60 hours (48h development + 12h fixtures/mocks)

---

### 9.2 Integration Testing

**Target:** 10-15 integration tests

**Test Environment:** Simulation (Gazebo) with fake sensors

**Example Tests:**

```python
# test/integration/test_approach_workflow.py
import pytest
import rclpy
from nav_interface.action import Approach

def test_approach_success(ros_node):
    """Test complete approach workflow"""

    # Setup: Robot at origin, marker at (5, 0)
    publish_marker_pose(x=5.0, y=0.0, yaw=3.14159)

    # Execute: Send approach goal
    result = send_approach_goal(timeout_sec=60.0)

    # Verify: Approach succeeded
    assert result.success == True

    # Verify: Robot reached goal (within 5cm)
    final_pose = get_robot_pose()
    assert abs(final_pose.x - 4.695) < 0.05  # 5.0 - 0.305
    assert abs(final_pose.y - 0.0) < 0.05

def test_docking_precision(ros_node):
    """Test precision docking accuracy"""

    # Setup: Robot at approach position
    position_robot_at_approach_pose()

    # Execute: Send dock goal
    result = send_dock_goal(timeout_sec=60.0)

    # Verify: Docking succeeded
    assert result.success == True

    # Verify: High precision (<2mm)
    assert result.final_distance_error_x < 0.002  # 2mm
    assert result.final_distance_error_y < 0.002  # 2mm

    # Verify: Duration reasonable (15-30s)
    assert 15.0 < result.duration_seconds < 30.0

def test_obstacle_stops_docking(ros_node):
    """Test safety: obstacle detection halts docking"""

    # Setup: Start docking
    dock_goal_handle = send_dock_goal_async()
    time.sleep(2.0)  # Let docking start

    # Simulate: Obstacle appears in path
    spawn_obstacle_in_path(distance=0.2)  # 20cm from robot

    # Verify: Robot stops within 1 second
    time.sleep(1.0)
    velocity = get_robot_velocity()
    assert velocity.linear.x < 0.01  # Nearly stopped

    # Verify: Docking action reports failure or pause
    # (depends on implementation: abort, pause, or wait)

def test_marker_loss_recovery(ros_node):
    """Test docking handles marker loss"""

    # Setup: Start docking
    dock_goal_handle = send_dock_goal_async()
    time.sleep(2.0)

    # Simulate: Markers disappear (blocked by obstacle)
    hide_markers(duration_sec=3.0)

    # Verify: Robot stops or slows
    time.sleep(0.5)
    velocity = get_robot_velocity()
    assert velocity.linear.x < 0.05

    # Simulate: Markers reappear
    show_markers()

    # Verify: Docking resumes and completes
    result = wait_for_dock_result(timeout_sec=30.0)
    assert result.success == True

def test_full_workflow_approach_dock(ros_node):
    """Test complete workflow: idle → approach → dock → docked"""

    # Verify initial state
    assert get_mission_state() == "IDLE"

    # Request approach
    trigger_approach()
    time.sleep(1.0)
    assert get_mission_state() == "WAITING_APPROACH_CONFIRMATION"

    # User confirms approach
    confirm_approach()
    assert get_mission_state() == "NAVIGATING_TO_GOAL"

    # Wait for approach to complete
    wait_for_state("WAITING_DOCK_CONFIRMATION", timeout_sec=60.0)

    # User confirms docking
    confirm_dock()
    assert get_mission_state() == "DOCKING"

    # Wait for docking to complete
    wait_for_state("DOCKED", timeout_sec=60.0)

    # Verify final state and position
    assert get_mission_state() == "DOCKED"
    final_pose = get_robot_pose()
    target_pose = get_target_pose()
    assert pose_distance(final_pose, target_pose) < 0.002  # <2mm
```

**Test Fixtures:**

```python
@pytest.fixture
def ros_node():
    """Create ROS 2 node for testing"""
    rclpy.init()
    node = rclpy.create_node('test_node')
    yield node
    node.destroy_node()
    rclpy.shutdown()

@pytest.fixture
def simulation():
    """Launch Gazebo simulation"""
    sim_process = launch_simulation("test_world.sdf")
    time.sleep(10.0)  # Wait for simulation to stabilize
    yield sim_process
    sim_process.terminate()
    sim_process.wait()
```

**Run Integration Tests:**
```bash
# Launch test simulation
ros2 launch multigo_testing test_simulation.launch.py

# Run integration tests
pytest test/integration/ -v

# Run specific test
pytest test/integration/test_approach_workflow.py::test_full_workflow_approach_dock -v -s
```

**Effort:** 30 hours (20h tests + 10h fixtures/simulation)

---

### 9.3 CI/CD Pipeline

**Tool:** GitHub Actions

**Configuration:** `.github/workflows/ci.yml`

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
      - name: Checkout code
        uses: actions/checkout@v3

      - name: Install dependencies
        run: |
          apt update
          apt install -y python3-pip
          rosdep update
          rosdep install --from-paths src --ignore-src -r -y

      - name: Build workspace
        run: |
          source /opt/ros/humble/setup.bash
          colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

      - name: Run unit tests
        run: |
          source install/setup.bash
          colcon test --packages-select nav_docking nav_goal nav_control aruco_detect
          colcon test-result --verbose

      - name: Run integration tests
        run: |
          source install/setup.bash
          pytest test/integration/ -v

      - name: Generate coverage report
        run: |
          pip install coverage
          coverage report --fail-under=80

      - name: Upload coverage to Codecov
        uses: codecov/codecov-action@v3
        with:
          files: ./coverage.xml
          flags: unittests
          name: codecov-umbrella

      - name: Lint code (C++)
        run: |
          apt install -y clang-format
          find src -name '*.cpp' -o -name '*.hpp' | xargs clang-format --dry-run --Werror

      - name: Lint code (Python)
        run: |
          pip install flake8
          flake8 src --count --select=E9,F63,F7,F82 --show-source --statistics
```

**Coverage Requirements:**
- Minimum 80% line coverage
- Minimum 70% branch coverage
- All tests must pass

**Effort:** 16 hours (setup CI + configure + test)

---

## 10. Proposed Deployment Architecture (Phase 4)

**Requirements Addressed:** DPR-1.1, DPR-1.2, DPR-2.1, LIR-2.3, QR-6.1, MCR-5.1, MCR-5.2, DOCR-1.1 ([REQUIREMENTS.md](./REQUIREMENTS.md))
**Implementation:** [IMPLEMENTATION-GUIDE.md Phase 4 Weeks 13-16](./IMPLEMENTATION-GUIDE.md#phase-4-deployment--operations)
**Issue Fixed:** CRIT-06 (Teaching mode)

### 10.1 Docker Deployment

**Requirements:** DPR-1.1, DPR-1.2, QR-6.1 ([REQUIREMENTS.md](./REQUIREMENTS.md))
**Purpose:** Containerized deployment for easy replication

**Docker Compose Configuration:**

```yaml
# docker-compose.yml
version: '3.8'

services:
  # Hardware interface (needs device access)
  hardware:
    image: multigo/hardware:${VERSION:-latest}
    container_name: multigo_hardware
    privileged: true  # For device access
    devices:
      - /dev/video0:/dev/video0        # Left camera
      - /dev/video1:/dev/video1        # Right camera
      - /dev/ttyUSB0:/dev/ttyUSB0      # LiDAR
      - /dev/phidget:/dev/phidget      # Motor controllers
    environment:
      - ROS_DOMAIN_ID=42
    command: ros2 launch boot boot.launch.py
    networks:
      - ros_network

  # Navigation and docking
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

  # Safety supervisor (critical, always restart)
  safety:
    image: multigo/safety:${VERSION:-latest}
    container_name: multigo_safety
    depends_on:
      - hardware
      - navigation
    environment:
      - ROS_DOMAIN_ID=42
    command: ros2 run safety_supervisor safety_supervisor_node
    restart: always  # Safety is critical!
    networks:
      - ros_network

  # Web UI (optional)
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

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-camera-info-manager \
    ros-humble-cv-bridge \
    python3-pip \
    libphidget22 \
    && rm -rf /var/lib/apt/lists/*

# Copy source
COPY src/camera_publisher /workspace/src/camera_publisher
COPY src/mecanum_wheels /workspace/src/mecanum_wheels

# Build
WORKDIR /workspace
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
```

**Build and Deploy:**

```bash
# Build all images
docker-compose build

# Deploy
docker-compose up -d

# View logs
docker-compose logs -f

# Stop
docker-compose down
```

**Effort:** 40 hours (create Dockerfiles + test deployment + documentation)

---

### 10.2 Configuration Management

**Problem:** Too many parameter files, unclear precedence

**Proposed:** Hierarchical configuration system

**Structure:**

```
config/
├── defaults/
│   ├── navigation.yaml       # System-wide navigation defaults
│   ├── docking.yaml          # System-wide docking defaults
│   └── safety.yaml           # Safety parameters
│
├── robots/
│   ├── multigo_001.yaml      # Robot-specific overrides
│   ├── multigo_002.yaml
│   └── multigo_test.yaml     # Test robot
│
├── environments/
│   ├── hospital_floor2.yaml  # Environment-specific (map, zones)
│   ├── hospital_floor3.yaml
│   └── simulation.yaml       # For testing
│
└── missions/
    ├── wheelchair_dock.yaml  # Mission-specific parameters
    ├── bed_dock.yaml
    └── transport.yaml
```

**Loading Priority (highest to lowest):**
1. Command-line arguments (`--params-file`)
2. Mission config (`wheelchair_dock.yaml`)
3. Environment config (`hospital_floor2.yaml`)
4. Robot config (`multigo_001.yaml`)
5. System defaults (`navigation.yaml`, `docking.yaml`)

**Configuration Loader:**

```python
# config_loader.py
class HierarchicalConfigLoader:
    def __init__(self, config_dir: str):
        self.config_dir = Path(config_dir)

    def load_config(self,
                   robot_id: str,
                   environment: str,
                   mission: str) -> Dict:
        """Load and merge configuration hierarchy"""

        # Start with defaults
        config = self.load_yaml(self.config_dir / "defaults" / "navigation.yaml")
        config.update(self.load_yaml(self.config_dir / "defaults" / "docking.yaml"))
        config.update(self.load_yaml(self.config_dir / "defaults" / "safety.yaml"))

        # Override with robot-specific
        robot_config = self.config_dir / "robots" / f"{robot_id}.yaml"
        if robot_config.exists():
            config.update(self.load_yaml(robot_config))

        # Override with environment-specific
        env_config = self.config_dir / "environments" / f"{environment}.yaml"
        if env_config.exists():
            config.update(self.load_yaml(env_config))

        # Override with mission-specific
        mission_config = self.config_dir / "missions" / f"{mission}.yaml"
        if mission_config.exists():
            config.update(self.load_yaml(mission_config))

        return config
```

**Usage:**

```bash
# Launch with hierarchical config
ros2 launch multigo_launch run.launch.py \
    robot_id:=multigo_001 \
    environment:=hospital_floor2 \
    mission:=wheelchair_dock
```

**Effort:** 24 hours (implement loader + migrate configs + document)

---

### 10.3 Teaching Mode (Waypoint System)

**Purpose:** Non-technical users can teach robot new routes and docking locations

**Implementation:**

**1. Waypoint Manager:**

```cpp
class WaypointManager : public rclcpp::Node {
private:
    std::map<std::string, geometry_msgs::msg::Pose> waypoints_;
    std::string waypoints_file_;

public:
    WaypointManager() : Node("waypoint_manager") {
        // Services
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

        // Load existing waypoints
        loadWaypoints();
    }

    void handleSave(
        const std::shared_ptr<SaveWaypoint::Request> request,
        std::shared_ptr<SaveWaypoint::Response> response
    ) {
        // Get current robot pose
        geometry_msgs::msg::PoseStamped current_pose;
        if (!getCurrentPose(current_pose)) {
            response->success = false;
            response->message = "Failed to get current pose";
            return;
        }

        // Save waypoint
        waypoints_[request->waypoint_id] = current_pose.pose;

        // Persist to disk
        saveWaypoints();

        response->success = true;
        response->message = "Waypoint saved: " + request->waypoint_id;

        RCLCPP_INFO(get_logger(), "Saved waypoint '%s' at (%.2f, %.2f)",
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

**2. Teaching Mode Launch:**

```python
# teaching_mode.launch.py
def generate_launch_description():
    return LaunchDescription([
        # Start hardware
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('boot.launch.py')
        ),

        # Start RTAB-Map in mapping mode
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            parameters=[{
                'Mem/IncrementalMemory': 'true',  # Mapping mode
                'Mem/InitWMWithAllNodes': 'false'
            }]
        ),

        # Start waypoint manager
        Node(
            package='multigo_waypoints',
            executable='waypoint_manager',
            parameters=[{
                'waypoints_file': '/config/waypoints/hospital_floor2.yaml'
            }]
        ),

        # Start teleoperation for manual control
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            prefix='xterm -e',  # Run in separate terminal
            remappings=[
                ('/cmd_vel', '/multigo/teleop/cmd_vel')
            ]
        ),

        # Waypoint teaching GUI
        Node(
            package='multigo_teaching',
            executable='teaching_gui',
            output='screen'
        )
    ])
```

**3. Teaching Workflow:**

```bash
# Step 1: Launch in teaching mode
ros2 launch multigo_teaching teaching_mode.launch.py

# Step 2: Manually drive robot to location (use keyboard/joystick)
# Use arrow keys to move, robot builds map as it moves

# Step 3: At each important location, save waypoint
ros2 service call /waypoint/save multigo_msgs/srv/SaveWaypoint "{waypoint_id: 'room_205_door'}"
ros2 service call /waypoint/save multigo_msgs/srv/SaveWaypoint "{waypoint_id: 'room_205_bed'}"

# Step 4: Save map
ros2 service call /rtabmap/save_map std_srvs/srv/Empty

# Step 5: List all saved waypoints
ros2 service call /waypoint/list multigo_msgs/srv/ListWaypoints

# Step 6: Switch to autonomous mode with saved map
ros2 launch multigo_launch run.launch.py map:=hospital_floor2.db
```

**Teaching GUI (RViz plugin or web interface):**

```
┌─────────────────────────────────────────┐
│         Teaching Mode Interface         │
├─────────────────────────────────────────┤
│ Current Position: (12.5, 8.3)           │
│ Map Coverage: 85%                       │
│                                         │
│ Saved Waypoints:                        │
│  ☑ room_205_door     (10.2, 7.1)       │
│  ☑ room_205_bed      (12.5, 8.3)       │
│  ☑ charging_station  (5.0, 2.0)        │
│                                         │
│ [Save Current Position]                 │
│ Waypoint Name: _______________          │
│                                         │
│ [Save Map and Exit Teaching Mode]      │
└─────────────────────────────────────────┘
```

**Effort:** 40 hours (waypoint manager 20h + GUI 20h)

**Benefits:**
- ✅ Non-technical users can teach routes
- ✅ No programming required
- ✅ Easy to update when environment changes
- ✅ Visualize all saved waypoints

---

## Summary: Implementation Roadmap

### Phase 1: Critical Bugs & Safety (4 weeks) - 144 hours
**Priority:** 🔴 CRITICAL

**Deliverables:**
- ✅ Fix 3 critical bugs (CRIT-01, CRIT-02, HIGH-01)
- ✅ Safety supervisor layer
- ✅ Emergency stop mechanism
- ✅ State machine in nav_master
- ✅ LiDAR monitoring during docking
- ✅ Basic geofencing

**Result:** Safe system ready for testing

---

### Phase 2: Testing Infrastructure (4 weeks) - 96 hours
**Priority:** 🔴 CRITICAL

**Deliverables:**
- ✅ 40-50 unit tests (80% coverage)
- ✅ 10-15 integration tests
- ✅ CI/CD pipeline (GitHub Actions)
- ✅ Automated regression testing

**Result:** Tested, validated system with regression protection

---

### Phase 3: ROS 2 Best Practices (4 weeks) - 104 hours
**Priority:** 🟡 HIGH

**Deliverables:**
- ✅ 4 nodes converted to lifecycle
- ✅ Explicit QoS policies
- ✅ Enhanced action definitions
- ✅ Standardized topic naming
- ✅ Command arbitration
- ✅ Holonomic motion enabled

**Result:** Production-quality ROS 2 architecture

---

### Phase 4: Deployment & Operations (4 weeks) - 104 hours
**Priority:** 🟡 HIGH

**Deliverables:**
- ✅ Teaching mode + waypoint system
- ✅ Docker deployment
- ✅ Hierarchical configuration
- ✅ Diagnostics system
- ✅ Operator documentation

**Result:** Deployable, operational system

---

## Total Effort: 504 hours (16 weeks with 2 developers)

*Note: Updated from 616h after requirement analysis and effort refinement*

---

**Next Steps:**
1. Review this document with team
2. Approve architecture changes
3. Review [REQUIREMENTS-TRACEABILITY.md](./REQUIREMENTS-TRACEABILITY.md) for requirement mapping
4. Start [IMPLEMENTATION-GUIDE.md](./IMPLEMENTATION-GUIDE.md) Phase 1
5. Track progress weekly using requirement IDs

**Questions?** See [START-HERE.md](./START-HERE.md) for navigation help

---

## Document History

### Version 1.1 (December 4, 2025)
**Changes:**
- ✅ Added requirement cross-references to all sections
- ✅ Updated completion percentage: 61% → 52% (91 requirements vs 77)
- ✅ Added links to REQUIREMENTS-TRACEABILITY.md
- ✅ Updated total effort: 616h → 504h (corrected calculation)
- ✅ Added requirement IDs to all proposed architecture sections:
  - Section 6: Safety (SR-1.1, SR-1.2, SR-2.2, SR-5.1, SR-5.2, SR-6.1)
  - Section 7: State Management (MCR-3.1)
  - Section 8: ROS 2 Improvements (QR-4.1, QR-5.1, QR-7.1, MR-4.1, MCR-2.3, SR-3.1, NR-2.2)
  - Section 9: Testing (QR-1.1, QR-1.2, QR-1.3, QR-3.2)
  - Section 10: Deployment (DPR-1.1, DPR-1.2, DPR-2.1, LIR-2.3, QR-6.1, MCR-5.1, MCR-5.2, DOCR-1.1)

### Version 1.0 (December 2, 2025)
- Initial comprehensive architecture analysis
- Current + Proposed architecture documented
- 4-phase implementation plan outlined

---

**Last Updated:** December 4, 2025
**Document Version:** 1.1
