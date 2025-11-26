# Docking System - Architecture Overview

**Branch:** feature/localization | **Analyst:** Claude AI | **Date:** Nov 25, 2025

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Component Architecture](#2-component-architecture)
3. [Data Flow](#3-data-flow)
4. [State Machine](#4-state-machine)
5. [Control Architecture](#5-control-architecture)
6. [Coordinate Frames](#6-coordinate-frames)
7. [Communication Topology](#7-communication-topology)
8. [Deployment Architecture](#8-deployment-architecture)

---

## 1. System Overview

### 1.1 Purpose

The Multi Go docking system enables **autonomous precision docking** of a mecanum-wheeled mobile robot to a stationary charging/loading station using **visual servoing** with **dual ArUco markers**.

### 1.2 Key Capabilities

- **Multi-stage approach:** Global navigation → Visual approach → Precision docking
- **Dual marker accuracy:** ±1mm final positioning using two markers
- **Fallback resilience:** Single marker operation if one marker lost
- **Action-based interface:** ROS2 action server for asynchronous control
- **Omnidirectional control:** Full X/Y/Yaw control using mecanum wheels

### 1.3 System Boundaries

```
┌─────────────────────────────────────────────────────────────┐
│                    DOCKING SYSTEM                           │
│                                                              │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │ Vision      │  │ Navigation   │  │ Motion       │       │
│  │ Perception  │→ │ Control      │→ │ Execution    │       │
│  └─────────────┘  └──────────────┘  └──────────────┘       │
│         ↑                                    ↓               │
└─────────┼────────────────────────────────────┼──────────────┘
          │                                    │
     [Cameras]                             [Motors]
          │                                    │
     [Dock Markers]                       [Robot Base]
```

**External Interfaces:**
- **Input:** Camera images, dock marker poses (via ArUco detection)
- **Output:** Velocity commands (Twist), action feedback/results
- **Dependencies:** TF2 transforms, NAV2 navigation stack

---

## 2. Component Architecture

### 2.1 Component Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                        DOCKING SYSTEM                            │
├──────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌────────────────┐                                              │
│  │  aruco_detect  │  Vision Layer                                │
│  │   (x2 nodes)   │                                              │
│  │                │                                              │
│  │ • Left camera  │                                              │
│  │ • Right camera │                                              │
│  └────────┬───────┘                                              │
│           │ PoseArray                                            │
│           ├─────────────────────┐                                │
│           ↓                     ↓                                │
│  ┌────────────────┐    ┌──────────────────┐                     │
│  │   nav_goal     │    │   nav_docking    │  Control Layer      │
│  │                │    │                  │                      │
│  │ • Stage 3      │    │ • Stage 4 & 5    │                      │
│  │ • Approach     │    │ • Visual servo   │                      │
│  │ • Goal gen     │    │ • PID control    │                      │
│  └────────┬───────┘    └────────┬─────────┘                     │
│           │ goal_pose           │ cmd_vel_final                  │
│           ↓                     ↓                                │
│  ┌────────────────┐    ┌──────────────────┐                     │
│  │     NAV2       │    │   nav_control    │  Motion Layer       │
│  │                │    │                  │                      │
│  │ • Global plan  │    │ • Rotation       │                      │
│  │ • Local plan   │    │   center adj.    │                      │
│  └────────────────┘    └────────┬─────────┘                     │
│                                 │ cmd_vel                        │
│                                 ↓                                │
│                        ┌──────────────────┐                      │
│                        │ mecanum_wheels   │  Execution Layer     │
│                        │                  │                      │
│                        │ • Inverse kinem. │                      │
│                        │ • Motor PID      │                      │
│                        │ • Phidget driver │                      │
│                        └────────┬─────────┘                      │
│                                 │ Motor commands                 │
└─────────────────────────────────┼────────────────────────────────┘
                                  ↓
                          [ Hardware Motors ]
```

---

### 2.2 Component Responsibilities

#### 2.2.1 aruco_detect (Vision Perception)

**Purpose:** Detect and localize ArUco markers from camera images

**Responsibilities:**
- Subscribe to camera images (`sensor_msgs/Image`)
- Subscribe to camera info (`sensor_msgs/CameraInfo`)
- Detect ArUco markers (DICT_6X6_250)
- Estimate 6-DOF pose of markers
- Transform from OpenCV to ROS coordinate system
- Publish marker poses (`geometry_msgs/PoseArray`)
- Broadcast TF transforms (`camera_frame` → `aruco_marker_XX`)
- Display detections in OpenCV window (optional)

**Configuration:**
- `desired_aruco_marker_id`: Which marker to detect
- `marker_width`: Physical marker size (m)
- `camera_topic`: Image source
- `camera_info`: Camera calibration

**Implementation:** C++ ROS2 node, OpenCV 4.x

---

#### 2.2.2 nav_goal (Approach Navigation)

**Purpose:** Generate navigation goals to approach dock (Stage 3)

**Responsibilities:**
- Subscribe to marker poses from both cameras
- Transform marker poses to map frame
- Calculate approach goal position (offset from marker)
- Publish navigation goals to NAV2 (`goal_pose`)
- Monitor distance to marker
- Transition to Stage 4 when distance < 0.05m
- Provide `approach` action server interface
- Publish feedback (wheelchair_distance)

**Configuration:**
- `desired_aruco_marker_id_left/right`: Target marker IDs
- `aruco_distance_offset`: Desired stopping distance
- `goal_distance_threshold`: When to transition (0.05m)

**Implementation:** C++ ROS2 node with action server

---

#### 2.2.3 nav_docking (Visual Servo Control)

**Purpose:** Precision docking using visual servoing (Stage 4 & 5)

**Responsibilities:**
- Subscribe to marker poses from both cameras
- Transform to base_link frame
- **Stage 4 (Alignment):**
  - Use single or dual markers based on availability
  - Calculate errors in X, Y, Yaw
  - PID control to min_error threshold (1cm)
  - Prioritize Y-axis alignment before forward motion
- **Stage 5 (Final Docking):**
  - Require dual markers
  - High-precision PID control
  - Achieve min_docking_error threshold (1mm)
  - Confirmation step before completion
- Publish velocity commands (`cmd_vel_final`)
- Provide `dock` action server interface
- Publish feedback (distance)
- Handle marker timeouts and fallback

**Configuration:**
- PID gains: `kp_x/y/z`, `ki_x/y/z`, `kd_x/y/z`
- Marker offsets for single/dual mode
- Error thresholds: `min_error`, `min_docking_error`
- Timeouts: `marker_delay_threshold_sec`, `docking_reset_threshold_sec`

**Implementation:** C++ ROS2 node with action server, dual timers (⚠️ race condition)

---

#### 2.2.4 nav_control (Kinematic Transform)

**Purpose:** Transform velocity commands for mecanum kinematics with rotation center offset

**Responsibilities:**
- Subscribe to `cmd_vel_final` (Twist)
- Apply rotation center offset based on drive mode:
  - SOLO: 0.0m (pivot at center)
  - DOCKING: 0.25m (pivot forward, for precision)
  - COMBINE_CHAIR: 0.5m (pivot far forward, for wheelchair)
- Transform lateral velocity: `y' = y + z * rotation_offset`
- Clamp velocities to safe limits
- Publish transformed `cmd_vel` (Twist)

**Configuration:**
- `mode_drive`: Current operating mode
- `LENGTH_ROTATION_CENTER_*`: Offset for each mode

**Implementation:** C++ ROS2 node

---

#### 2.2.5 mecanum_wheels (Motor Control)

**Purpose:** Convert velocity commands to motor speeds and execute with closed-loop control

**Responsibilities:**
- Subscribe to `cmd_vel` (Twist)
- **Inverse kinematics:** Convert X, Y, Yaw → wheel speeds
- **Forward kinematics:** Calculate actual velocity from encoders
- **Closed-loop PID:** Correct for wheel slippage
- **Motor interface:** Command Phidget BLDC motors via USB
- Publish real velocity (`real_vel`) for odometry

**Configuration:**
- Wheel geometry: `WHEEL_SEPARATION_WIDTH/LENGTH`
- Wheel radius: `WHEEL_RADIUS`
- PID gains: `kp`, `ki`, `kd`
- Motor ports: Hub ports 1-4

**Implementation:** Python ROS2 node, Phidget22 library

---

### 2.3 Dependency Graph

```
aruco_detect (left)  ──┐
aruco_detect (right) ──┼──→ nav_goal ──→ NAV2
                       └──→ nav_docking ──→ nav_control ──→ mecanum_wheels
                                                                    ↓
                       TF2 ←──────────────────────────────────────┘
```

**Critical Dependencies:**
- **TF2 transforms:** All nodes depend on TF tree
- **Camera feeds:** Vision layer depends on camera nodes
- **Phidget hardware:** Motor control depends on USB connection
- **nav_interface:** Action definitions from external package

---

## 3. Data Flow

### 3.1 End-to-End Data Flow

```
[Dock with Markers]
       ↓ Reflection
[Camera Images] ─────────────┐
                             ↓
                    [aruco_detect]
                      • Detect markers
                      • Estimate pose
                      • Transform coords
                             ↓
            [PoseArray: aruco_detect/markers_*]
                             ↓
             ┌───────────────┴─────────────┐
             ↓                             ↓
        [nav_goal]                    [nav_docking]
        Stage 3: Approach             Stage 4 & 5: Docking
             ↓                             ↓
    [PoseStamped: goal_pose]      [Twist: cmd_vel_final]
             ↓                             ↓
         [NAV2] ──────────────────→  [nav_control]
                                          ↓
                               [Twist: cmd_vel]
                                          ↓
                               [mecanum_wheels]
                                  • Inverse kinematics
                                  • PID control
                                          ↓
                               [Motor Commands: USB]
                                          ↓
                                  [BLDC Motors]
                                          ↓
                                   [Robot Motion]
```

---

### 3.2 Message Types & Topics

| Topic | Message Type | Publisher | Subscriber | Rate | Purpose |
|-------|-------------|-----------|------------|------|---------|
| `/camera/color/image_raw_left` | `sensor_msgs/Image` | camera_left | aruco_detect_left | ~30 Hz | Left camera feed |
| `/camera/color/image_raw_right` | `sensor_msgs/Image` | camera_right | aruco_detect_right | ~30 Hz | Right camera feed |
| `/aruco_detect/markers_left` | `geometry_msgs/PoseArray` | aruco_detect_left | nav_goal, nav_docking | Variable | Left marker poses |
| `/aruco_detect/markers_right` | `geometry_msgs/PoseArray` | aruco_detect_right | nav_goal, nav_docking | Variable | Right marker poses |
| `/goal_pose` | `geometry_msgs/PoseStamped` | nav_goal | NAV2 | 30 Hz | Navigation goals |
| `/cmd_vel_final` | `geometry_msgs/Twist` | nav_docking | nav_control | 30 Hz | Docking commands |
| `/cmd_vel` | `geometry_msgs/Twist` | nav_control | mecanum_wheels | Variable | Motor commands |
| `/real_vel` | `geometry_msgs/Twist` | mecanum_wheels | odometry | 30 Hz | Actual velocity |

---

### 3.3 Action Interfaces

#### Approach Action (`/approach`)

**Type:** `nav_interface/action/Approach`

```
# Goal
bool approach_request

---
# Result
bool approach_success

---
# Feedback
float64 wheelchair_distance  # Distance to target
```

**Lifecycle:**
1. Client sends goal (`approach_request = true`)
2. `nav_goal` accepts and starts Stage 3
3. Publishes goals to NAV2
4. Monitors distance, publishes feedback
5. When distance < 0.05m, succeeds and transitions to Stage 4

---

#### Dock Action (`/dock`)

**Type:** `nav_interface/action/Dock`

```
# Goal
bool dock_request

---
# Result
bool success

---
# Feedback
float64 distance  # Current error distance
```

**Lifecycle:**
1. Client sends goal (`dock_request = true`)
2. `nav_docking` accepts and starts Stage 4
3. Visual servoing control, publishes `cmd_vel_final`
4. Publishes feedback with current error
5. Transitions Stage 4 → Stage 5 when aligned (error < 1cm)
6. Stage 5 precision docking (error < 1mm)
7. Confirmation step (0.5s delay)
8. Succeeds with `success = true`

---

## 4. State Machine

### 4.1 Docking State Diagram

```
                    START
                      ↓
              [Action: approach]
                      ↓
          ┌───────────────────────┐
          │   STAGE 3: APPROACH   │
          │   (nav_goal)          │
          │                       │
          │ • Markers detected    │
          │ • Generate goals      │
          │ • NAV2 navigation     │
          └───────────┬───────────┘
                      │
               distance < 0.05m
                      ↓
              [Action: dock]
                      ↓
          ┌───────────────────────┐
          │  STAGE 4: ALIGNMENT   │
          │  (nav_docking)        │
          │                       │
          │ • Visual servoing     │
          │ • Single/dual markers │
          │ • PID control         │
          │ • Y-axis priority     │
          └───────────┬───────────┘
                      │
              error < 0.01m (1cm)
                      ↓
          ┌───────────────────────┐
          │ STAGE 5: FINAL DOCK   │
          │ (nav_docking)         │
          │                       │
          │ • Dual markers req.   │
          │ • Precision PID       │
          │ • High accuracy       │
          └───────────┬───────────┘
                      │
              error < 0.001m (1mm)
                      ↓
          ┌───────────────────────┐
          │  CONFIRMATION         │
          │ (0.5s delay)          │
          │                       │
          │ • Re-check errors     │
          │ • Verify stability    │
          └───────────┬───────────┘
                      │
              Still within tolerance
                      ↓
                  COMPLETE
                  (success!)

         [Failure/Cancel paths not shown]
```

---

### 4.2 State Variables (Current Implementation)

**⚠️ Note:** Current implementation uses boolean flags instead of proper state enum.

| Variable | Type | Purpose | Location |
|----------|------|---------|----------|
| `stage_3_docking_status` | bool | Approach complete | nav_goal.cpp |
| `stage_4_docking_status` | bool | Alignment complete | nav_docking.cpp |
| `stage_5_docking_status` | bool | Final docking complete | nav_docking.cpp |
| `stage_6_docking_status` | bool | ⚠️ **Unused!** | nav_docking.h:73 |
| `confirmed_docking_status` | bool | Confirmation flag | nav_docking.cpp |
| `enable_callback` | bool | Action active | nav_docking.cpp |

**Issue:** No formal state machine, hard to maintain, not thread-safe

---

### 4.3 State Transitions

| From | To | Condition | Timeout | Reset Condition |
|------|----|-----------| --------|-----------------|
| IDLE | STAGE 3 | `approach` action received | None | N/A |
| STAGE 3 | STAGE 4 | `marker_tx < 0.05m` | None | Distance increases |
| STAGE 4 | STAGE 5 | `error_x/y/yaw < 0.01m` | ❌ None | Error exceeds threshold |
| STAGE 5 | CONFIRM | `error_x/y/yaw < 0.001m` | ❌ None | Error exceeds threshold |
| CONFIRM | COMPLETE | Still within tolerance | 0.5s | Error exceeds threshold |
| STAGE 5 | STAGE 4 | Marker lost > 3s | 3.0s | N/A |

**⚠️ Gap:** No maximum stage timeout! Can loop forever.

---

## 5. Control Architecture

### 5.1 Control Hierarchy

```
    [High-Level Mission Planner]
                ↓
         [Behavior Layer]
                ↓
      ┌─────────────────┐
      │  DOCKING CONTROL │
      └─────────┬────────┘
                ↓
    ┌───────────┴──────────┐
    ↓                      ↓
[Global Navigation]   [Visual Servoing]
(NAV2 - Stage 3)      (nav_docking - Stage 4&5)
    ↓                      ↓
    └───────────┬──────────┘
                ↓
         [Velocity Arbiter]
         (nav_control)
                ↓
        [Motor Controller]
        (mecanum_wheels)
                ↓
          [Hardware]
```

---

### 5.2 PID Control Loops

#### Stage 4 & 5 Control

```
┌─────────────────────────────────────────────────────────────┐
│                   PID CONTROL LOOP                          │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  Marker Pose ──→ Transform ──→ Error Calculation           │
│  (camera)         (TF2)          error_x                     │
│                                  error_y                     │
│                                  error_yaw                   │
│                                      ↓                       │
│                               ┌──────────────┐              │
│                               │  PID (X-axis)│              │
│                               │ kp=1.0       │              │
│                               │ ki=1.0 🐛    │              │
│                               │ kd=0.2       │              │
│                               └──────┬───────┘              │
│                                      ↓                       │
│                               ┌──────────────┐              │
│                               │  PID (Y-axis)│              │
│                               │ kp=1.0       │              │
│                               │ ki=0.7 🐛    │              │
│                               │ kd=0.03      │              │
│                               └──────┬───────┘              │
│                                      ↓                       │
│                               ┌──────────────┐              │
│                               │  PID (Yaw)   │              │
│                               │ kp=1.0       │              │
│                               │ ki=2.6 🐛    │              │
│                               │ kd=0.05      │              │
│                               └──────┬───────┘              │
│                                      ↓                       │
│                         [Velocity Commands]                 │
│                            cmd_vel_final                     │
│                                      ↓                       │
│                            [Velocity Clamping]               │
│                            max: 0.1 m/s                      │
│                            min: 0.005 m/s                    │
│                                      ↓                       │
│                             [Robot Motion]                   │
│                                      ↓                       │
│                        [Actual Position] ───┐               │
│                                             │               │
│                                    Feedback │               │
│                                             │               │
│                        [Marker Detection] ←─┘               │
└─────────────────────────────────────────────────────────────┘
```

**🐛 Bug:** Integral term not accumulating! (See code-analysis.md)

---

### 5.3 Motor Control Loop

```
┌──────────────────────────────────────────────────────────┐
│               MOTOR CONTROL (mecanum_wheels)             │
├──────────────────────────────────────────────────────────┤
│                                                           │
│  cmd_vel ──→ Inverse Kinematics ──→ Wheel Speeds        │
│  (Twist)        (mecanum)            [fl, fr, bl, br]    │
│                                           ↓               │
│                                     [Motor PID]          │
│                                      kp=0.2              │
│                                      ki=4.2              │
│                                      kd=0.1              │
│                                           ↓               │
│                                    [Phidget Motors]      │
│                                      (4x BLDC)            │
│                                           ↓               │
│                                   [Encoder Feedback]      │
│                                           ↓               │
│                                  Forward Kinematics       │
│                                           ↓               │
│                                      real_vel ───┐       │
│                                                   │       │
│                                          Feedback │       │
│                                                   │       │
│                                    [Motor PID] ←─┘       │
└──────────────────────────────────────────────────────────┘
```

**Note:** This PID implementation is **correct** (unlike nav_docking PID).

---

## 6. Coordinate Frames

### 6.1 Transform Tree

```
              map
               │
               ├─ odom
               │   │
               │   └─ base_link ────────────┐
               │       │                     │
               │       ├─ camera_left_frame  │
               │       │   │                  │
               │       │   └─ aruco_marker_20 │ (detected)
               │       │                      │
               │       └─ camera_right_frame  │
               │           │                  │
               │           └─ aruco_marker_21 │ (detected)
               │
               └─ [Other robot frames]
```

---

### 6.2 Frame Definitions

| Frame | Parent | Description | Source |
|-------|--------|-------------|--------|
| `map` | (world) | Global reference frame | RTAB-Map |
| `odom` | `map` | Odometry frame (drift-free in map) | Robot odometry |
| `base_link` | `odom` | Robot center | Robot state publisher |
| `camera_left_frame` | `base_link` | Left camera optical frame | Static TF |
| `camera_right_frame` | `base_link` | Right camera optical frame | Static TF |
| `aruco_marker_20` | `camera_left_frame` | Detected left marker | aruco_detect (dynamic) |
| `aruco_marker_21` | `camera_right_frame` | Detected right marker | aruco_detect (dynamic) |

---

### 6.3 Coordinate Transform Chain

**Stage 3 (Approach):**
```
aruco_marker_20 → camera_left_frame → base_link → odom → map
     (detected)      (static TF)      (dynamic)  (dynamic)
```

**Purpose:** Generate goal pose in map frame for NAV2

---

**Stage 4 & 5 (Docking):**
```
aruco_marker_20/21 → camera_{left/right}_frame → base_link
      (detected)           (static TF)            (control frame)
```

**Purpose:** Calculate errors in robot-centric frame for visual servoing

---

### 6.4 Coordinate System Conventions

**ROS Convention (REP 103):**
- X: Forward
- Y: Left
- Z: Up

**OpenCV Convention (aruco_detect converts):**
- X: Right
- Y: Down
- Z: Forward (into image)

**Conversion Matrix:**
```
ROS_coords = cv_to_ros * OpenCV_coords * cv_to_ros^T

cv_to_ros = | 0   0  1|
            |-1   0  0|
            | 0  -1  0|
```

---

## 7. Communication Topology

### 7.1 Node Graph

```
                       ┌───────────────┐
                       │  camera_left  │
                       └───────┬───────┘
                               │ /camera/left/image_raw
                               ↓
                     ┌────────────────────┐
                     │ aruco_detect_left  │
                     └──────────┬─────────┘
                                │ /aruco_detect/markers_left
                                ├─────────────────────┐
                                ↓                     ↓
                      ┌─────────────────┐   ┌──────────────────┐
                      │   nav_goal      │   │   nav_docking    │
                      │  [ACTION]       │   │   [ACTION]       │
                      └────────┬────────┘   └────────┬─────────┘
                               │ /goal_pose          │ /cmd_vel_final
                               ↓                     ↓
                      ┌─────────────────┐   ┌──────────────────┐
                      │     NAV2        │   │   nav_control    │
                      └─────────────────┘   └────────┬─────────┘
                                                     │ /cmd_vel
                                                     ↓
                                            ┌──────────────────┐
                                            │ mecanum_wheels   │
                                            └────────┬─────────┘
                                                     │ USB
                                                     ↓
                                             [Phidget Hub]
                                                     │
                                            ┌────────┴────────┐
                                            ↓                 ↓
                                      [Motor 1-2]       [Motor 3-4]

[Similar branch for camera_right → aruco_detect_right]
```

---

### 7.2 QoS Profiles

**Recommended QoS:**

| Topic | Reliability | Durability | History Depth | Use Case |
|-------|-------------|------------|---------------|----------|
| `/camera/*/image_raw` | BEST_EFFORT | VOLATILE | 1 | High-rate sensor data |
| `/aruco_detect/markers_*` | RELIABLE | VOLATILE | 10 | Critical perception data |
| `/goal_pose` | RELIABLE | TRANSIENT_LOCAL | 1 | Goals must arrive |
| `/cmd_vel_final` | RELIABLE | VOLATILE | 1 | Real-time control |
| `/cmd_vel` | RELIABLE | VOLATILE | 1 | Motor commands |
| **Actions** | RELIABLE | TRANSIENT_LOCAL | 10 | Goal/Result delivery |

**Current:** Not explicitly configured (using defaults)

---

## 8. Deployment Architecture

### 8.1 Process Deployment

**Single Robot Deployment:**

```
┌───────────────────────────────────────────────────────┐
│              ROBOT ONBOARD COMPUTER                   │
│                 (Ubuntu 22.04)                        │
├───────────────────────────────────────────────────────┤
│                                                        │
│  ROS2 Humble Workspace                                │
│                                                        │
│  ┌──────────────────────────────────────────┐        │
│  │  Vision Nodes (2 processes)              │        │
│  │  • aruco_detect_left                     │        │
│  │  • aruco_detect_right                    │        │
│  └──────────────────────────────────────────┘        │
│                                                        │
│  ┌──────────────────────────────────────────┐        │
│  │  Control Nodes (3 processes)             │        │
│  │  • nav_goal                              │        │
│  │  • nav_docking                           │        │
│  │  • nav_control                           │        │
│  └──────────────────────────────────────────┘        │
│                                                        │
│  ┌──────────────────────────────────────────┐        │
│  │  Motor Node (1 process)                  │        │
│  │  • mecanum_wheels (Python)               │        │
│  └──────────────────────────────────────────┘        │
│                                                        │
│  ┌──────────────────────────────────────────┐        │
│  │  Navigation Stack                        │        │
│  │  • NAV2 (multiple nodes)                 │        │
│  │  • RTAB-Map                              │        │
│  └──────────────────────────────────────────┘        │
│                                                        │
│  ┌──────────────────────────────────────────┐        │
│  │  Hardware Interfaces                     │        │
│  │  • Camera drivers (2)                    │        │
│  │  • Phidget USB interface                 │        │
│  │  • LiDAR driver                          │        │
│  └──────────────────────────────────────────┘        │
│                                                        │
└───────────────────────────────────────────────────────┘
```

---

### 8.2 Hardware Architecture

```
┌────────────────────────────────────────────┐
│         ROBOT HARDWARE PLATFORM            │
├────────────────────────────────────────────┤
│                                             │
│  ┌──────────────────────────────────────┐ │
│  │  Onboard Computer                    │ │
│  │  • Intel NUC or similar              │ │
│  │  • Ubuntu 22.04                      │ │
│  │  • ROS2 Humble                       │ │
│  └──────────────────────────────────────┘ │
│          │                                  │
│          ├─ USB ──→ Cameras (2x)          │
│          ├─ USB ──→ Phidget Hub           │
│          ├─ USB ──→ LiDAR                 │
│          └─ Ethernet ──→ Remote access    │
│                                             │
│  ┌──────────────────────────────────────┐ │
│  │  Phidget VINT Hub                    │ │
│  │  • Motor controllers (4 ports)       │ │
│  └──────────────────────────────────────┘ │
│          │                                  │
│          ├─ Port 1 ──→ Motor LR           │
│          ├─ Port 2 ──→ Motor RR           │
│          ├─ Port 3 ──→ Motor LF           │
│          └─ Port 4 ──→ Motor RF           │
│                                             │
│  ┌──────────────────────────────────────┐ │
│  │  Mecanum Wheel Base                  │ │
│  │  • 4x BLDC Motors                    │ │
│  │  • 4x Mecanum wheels (3" dia)        │ │
│  │  • Wheelbase: 0.40m x 0.30m          │ │
│  └──────────────────────────────────────┘ │
└────────────────────────────────────────────┘
```

---

### 8.3 Network Architecture

**Single Robot (Standalone):**
```
Robot
  │
  └─ ROS_DOMAIN_ID=30
     All nodes communicate via DDS
```

**Multi-Robot (Future):**
```
Robot 1 (ROS_DOMAIN_ID=30)  ←─ WiFi ─→  Robot 2 (ROS_DOMAIN_ID=31)
                                ↓
                            Base Station
                         (Monitoring/Control)
```

---

## 9. Key Design Decisions

### 9.1 Why Dual Markers?

**Decision:** Use two markers instead of one

**Rationale:**
- **Redundancy:** System continues if one marker occluded
- **Accuracy:** Averaging reduces noise
- **Rotation:** Distance between markers gives rotation estimate
- **Robustness:** Less sensitive to single marker detection errors

**Trade-off:** More complex logic, potential for race conditions

---

### 9.2 Why Three Stages?

**Decision:** Stage 3 (NAV2) → Stage 4 (Alignment) → Stage 5 (Precision)

**Rationale:**
- **Stage 3:** Global navigation efficient for long distances
- **Stage 4:** Visual servoing accurate for medium range (5cm → 1cm)
- **Stage 5:** High-precision control for final millimeter accuracy
- **Separation:** Different control strategies for different scales

**Trade-off:** Complex state management, potential for transition failures

---

### 9.3 Why Separate nav_goal and nav_docking?

**Decision:** Two separate nodes instead of one combined node

**Rationale:**
- **Modularity:** Clear separation of concerns
- **Testing:** Can test approach and docking independently
- **Flexibility:** Can use different markers for approach vs. docking
- **Reusability:** nav_goal could be reused for other visual features

**Trade-off:** More processes, inter-node communication overhead

---

### 9.4 Why Action Servers?

**Decision:** Use ROS2 actions instead of services or topics

**Rationale:**
- **Asynchronous:** Long-running operations (docking takes 30+ seconds)
- **Feedback:** Real-time progress updates during execution
- **Cancellation:** Ability to abort mid-operation
- **Standard:** ROS2 pattern for goal-oriented behaviors

**Trade-off:** More complex than simple service calls

---

## 10. Performance Characteristics

### 10.1 Timing Analysis

| Operation | Expected Time | Actual | Notes |
|-----------|---------------|--------|-------|
| **Stage 3: Approach** | 10-60 seconds | ❓ | Depends on distance and NAV2 speed |
| **Stage 4: Alignment** | 5-15 seconds | ❓ | Depends on initial misalignment |
| **Stage 5: Final Dock** | 3-10 seconds | ❓ | High-precision movements |
| **Total Docking Time** | 20-90 seconds | ❓ | Varies by scenario |
| **Marker Detection** | <33ms (30 fps) | ❓ | Camera-dependent |
| **Control Loop** | 33ms (30 Hz) | ✅ | Configured |
| **Transform Lookup** | <10ms | ❓ | TF2 cached |

**Gap:** Need actual timing measurements from field tests

---

### 10.2 Accuracy Specifications

| Metric | Requirement | Achieved | Status |
|--------|-------------|----------|--------|
| **Final Position (X)** | ±1mm | ❓ | 🐛 Bug affects this |
| **Final Position (Y)** | ±3mm | ❓ | Higher tolerance |
| **Final Orientation** | ±1 degree | ❓ | Needs verification |
| **Stage 4 Accuracy** | ±1cm | ❓ | Intermediate |
| **Repeatability** | ±2mm | ❓ | Needs testing |

**Gap:** Need accuracy validation testing

---

## 11. Failure Modes & Recovery

### 11.1 Known Failure Modes

| Failure Mode | Detection | Recovery | Status |
|--------------|-----------|----------|--------|
| **One marker lost** | Delay > 0.2s | Fall back to single marker | ✅ Implemented |
| **Both markers lost** | Delay > 0.2s | Stop, wait for markers | 🟡 Partial (no timeout) |
| **TF lookup fail** | Exception | Skip cycle, log warning | ✅ Implemented |
| **Action cancelled** | `is_canceling()` | Stop motion, return | ✅ Implemented |
| **Marker lost > 3s (Stage 5)** | Duration check | Reset to Stage 4 | ✅ Implemented |
| **Infinite loop** | None | ❌ Not handled | ⚠️ Critical gap |

---

### 11.2 Unhandled Edge Cases

⚠️ **Gaps:**
- No maximum docking time (could loop forever)
- No handling of persistent failures (max retries)
- No detection of marker damage/incorrectness
- No handling of camera failure
- No handling of motor failure
- No handling of excessive drift during docking

---

## 12. Future Architecture Considerations

### 12.1 Scalability

**Multi-Robot Docking:**
- Need coordination to avoid marker conflicts
- Marker ID assignment strategy
- Queue management for shared docks

**Multiple Docks:**
- Dock database with positions
- Dock selection algorithm
- Dynamic marker ID discovery

---

### 12.2 Extensibility

**Potential Extensions:**
- **Undocking action:** Reverse procedure
- **Dock selection:** Choose best available dock
- **Auto-charging:** Integrate with battery monitoring
- **Diagnostics:** Health monitoring and reporting
- **Auto-calibration:** Learn offsets from successful docks

---

## Summary

The Multi Go docking system implements a sophisticated **three-stage visual servoing approach** using **dual ArUco markers** for precision autonomous docking. The architecture demonstrates good modularity with clear separation of concerns across vision, control, and motion execution layers.

**Strengths:**
- ✅ Modular component design
- ✅ Redundancy through dual markers
- ✅ Robust fallback mechanisms
- ✅ Standard ROS2 patterns (actions, transforms)

**Critical Issues:**
- 🐛 PID integral calculation bug
- 🐛 Dual marker distance formula bug
- ⚠️ Thread safety concerns (race conditions)
- ⚠️ No formal state machine
- ⚠️ Missing timeouts and retry logic

**Recommendations:**
1. Fix critical bugs immediately
2. Implement proper state machine with enum states
3. Add mutex protection for shared state
4. Implement timeout and retry mechanisms
5. Expand testing coverage

---

*For detailed bug analysis, see `code-analysis.md`. For requirements status, see `requirements-docking.md`.*
