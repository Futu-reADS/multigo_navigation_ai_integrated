# Docking System - Quick Reference

**Branch:** feature/localization | **Analyst:** Claude AI | **Date:** Nov 25, 2025

---

## System Overview

The Multi Go docking system enables **autonomous precision docking** using **dual ArUco markers** for visual servoing. The system achieves **1mm positioning accuracy** through a multi-stage approach combining NAV2 navigation with vision-based control.

---

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                      DOCKING SYSTEM ARCHITECTURE                     │
└─────────────────────────────────────────────────────────────────────┘

┌──────────────┐
│   Cameras    │  Left & Right Front Cameras
│  (2x RGB)    │  1280x720, Calibrated
└──────┬───────┘
       │ Image Stream
       ↓
┌──────────────────────┐
│  aruco_detect_node   │  ArUco Detection (DICT_6X6_250)
│   (2 instances)      │  • Marker ID 20 (left)
└──────┬───────────────┘  • Marker ID 21 (right)
       │ PoseArray
       │ (markers_left / markers_right)
       │
       ├─────────────────┐
       ↓                 ↓
┌─────────────┐   ┌──────────────────┐
│ nav_goal    │   │  nav_docking     │  **Main Docking Controller**
│   (Stage 3) │   │  (Stage 4 & 5)   │
└──────┬──────┘   └──────┬───────────┘
       │                 │
       │ goal_pose       │ cmd_vel_final
       ↓                 ↓
┌──────────────┐   ┌──────────────────┐
│    NAV2      │   │  nav_control     │  Rotation Center Adjustment
│  Navigator   │   │    (Kinematic)   │  Mode: DOCKING
└──────────────┘   └──────┬───────────┘
                          │ cmd_vel
                          ↓
                   ┌──────────────────┐
                   │ mecanum_wheels   │  Motor Control (PID)
                   │  (Phidget22)     │  4x BLDC Motors
                   └──────────────────┘
                          ↓
                   ┌──────────────────┐
                   │  Robot Platform  │  Mecanum Drive
                   │   (Hardware)     │  0.40m x 0.30m
                   └──────────────────┘
```

---

## Three-Stage Docking Process

| Stage | Node | Function | Precision | Status Check |
|-------|------|----------|-----------|--------------|
| **Stage 3** | `nav_goal` | **Approach** - Publishes goals to NAV2 | ~5cm | `distance < 0.05m` |
| **Stage 4** | `nav_docking` | **Alignment** - Visual servoing approach | ~1cm | `error < 0.01m` |
| **Stage 5** | `nav_docking` | **Final Dock** - Precision positioning | ~1mm | `error < 0.001m` |

```
    [Stage 3: NAV2 Approach]
            │
            │ Distance < 0.05m
            ↓
    [Stage 4: Visual Alignment]
    • Single/Dual marker switching
    • PID control (X, Y, Yaw)
    • Error threshold: 1cm
            │
            │ Aligned & error < 1cm
            ↓
    [Stage 5: Precision Docking]
    • Dual markers required
    • High-precision PID
    • Error threshold: 1mm
    • Confirmation step (0.5s)
            │
            ↓
    [✓ DOCKING COMPLETE]
```

---

## Key Components

### 1. **nav_docking** (Main Controller)
- **File:** `src/nav_docking/src/nav_docking.cpp`
- **Action:** `dock` (bool request → bool success, float distance feedback)
- **Control:** Dual PID loops (Stage 4 & 5)
- **Publish Rate:** 30 Hz
- **Output:** `cmd_vel_final` (Twist)

### 2. **aruco_detect** (Vision)
- **File:** `src/aruco_detect/src/aruco_detect.cpp`
- **Dictionary:** DICT_6X6_250
- **Markers:** ID 20 (left), ID 21 (right)
- **Output:** `aruco_detect/markers_{left,right}` (PoseArray)
- **Transform:** Broadcasts TF: `camera_frame → aruco_marker_XX`

### 3. **nav_goal** (Approach)
- **File:** `src/nav_goal/src/nav_goal.cpp`
- **Action:** `approach` (bool request → bool success)
- **Function:** Transforms marker → map → goal pose
- **Output:** `goal_pose` (PoseStamped)
- **Threshold:** 0.05m (5cm)

### 4. **nav_control** (Kinematics)
- **File:** `src/nav_control/src/nav_control.cpp`
- **Function:** Rotation center offset for docking mode
- **Modes:** SOLO (0m), DOCKING (0.25m), COMBINE_CHAIR (0.5m)
- **Transform:** `cmd_vel_final → cmd_vel`

---

## Control Parameters

### PID Gains (Tuned for Docking)
```yaml
kp_x: 1.0,  ki_x: 1.0,  kd_x: 0.2   # Forward/backward
kp_y: 1.0,  ki_y: 0.7,  kd_y: 0.03  # Left/right (mecanum)
kp_z: 1.0,  ki_z: 2.6,  kd_z: 0.05  # Rotation
```

### Marker Offsets
```
desired_aruco_marker_id_left:   20
desired_aruco_marker_id_right:  21
aruco_distance_offset:          0.31 m    (single marker)
aruco_left_right_offset:        0.17 m
aruco_distance_offset_dual:     0.430 m   (dual marker)
aruco_center_offset_dual:       0.0 m
aruco_rotation_offset_dual:     0.0 m
```

### Thresholds
```
Stage 4 error threshold:        0.01 m (1 cm)
Stage 5 error threshold:        0.001 m (1 mm)
Marker delay threshold:         0.2 sec
Docking reset threshold:        3.0 sec
```

---

## Data Flow

```
┌─────────────┐
│ ArUco       │  Markers on Dock
│ Markers     │  ID: 20, 21
└──────┬──────┘
       │
       ↓ (Detection)
┌─────────────────────────────────────────┐
│  aruco_detect (2 instances)             │
│  • Pose estimation                      │
│  • TF broadcast                         │
└──────┬──────────────────────────────────┘
       │
       │ PoseArray (marker poses in camera frame)
       ├───────────────────────┐
       ↓                       ↓
┌──────────────┐       ┌─────────────────────┐
│  nav_goal    │       │  nav_docking        │
│              │       │                     │
│  Transform:  │       │  Transform:         │
│  camera →    │       │  camera → base_link │
│  map frame   │       │                     │
│              │       │  Compute Errors:    │
│  Output:     │       │  • error_x          │
│  goal_pose   │       │  • error_y          │
│              │       │  • error_yaw        │
│              │       │                     │
│              │       │  PID Control →      │
│              │       │  cmd_vel_final      │
└──────┬───────┘       └──────┬──────────────┘
       │                      │
       ↓                      ↓
  ┌────────┐         ┌──────────────┐
  │  NAV2  │         │ nav_control  │
  │        │         │ (kinematic   │
  │        │         │  transform)  │
  └────────┘         └──────┬───────┘
                            │
                            ↓
                     ┌──────────────┐
                     │ cmd_vel      │
                     │              │
                     │ mecanum_     │
                     │ wheels       │
                     └──────────────┘
```

---

## Marker Detection Strategy

### Dual Marker Mode (Preferred)
- **When:** Both markers visible (delay < 0.2s)
- **Calculation:**
  ```cpp
  distance = (left_x + right_x) / 2
  rotation = (right_x - left_x)
  center = (left_y - (-right_y))
  ```

### Single Marker Fallback
- **When:** One marker lost or delayed (> 0.2s)
- **Selection:** Use marker with shortest delay
- **Offset:** Apply `aruco_left_right_offset_single` (±0.17m)

---

## Safety Features

| Feature | Implementation | Location |
|---------|---------------|----------|
| **Marker Timeout** | Stop if marker lost > 0.2s | `nav_docking.cpp:427` |
| **Stage Reset** | Reset to Stage 4 if markers lost > 3s | `nav_docking.cpp:493` |
| **Velocity Clamping** | Max speed: 0.1 m/s, Min: 0.005 m/s | `nav_docking.h:108-109` |
| **Alignment Check** | No forward motion until Y-aligned | `nav_docking.cpp:430-446` |
| **Confirmation** | Requires 2 consecutive success readings | `nav_docking.cpp:540-551` |
| **Action Cancellation** | Graceful cancel support | `nav_docking.cpp:131-136` |

---

## Action Interface

### Dock Action
```cpp
# Goal
bool dock_request

---
# Result
bool success

---
# Feedback
float64 distance   # Current error distance
```

**Usage:**
```bash
ros2 action send_goal /dock nav_interface/action/Dock "{dock_request: true}"
```

---

## Configuration Files

| File | Purpose | Location |
|------|---------|----------|
| `docking_pid_params.yaml` | PID tuning | `src/nav_docking/config/` |
| `nav_docking.launch.py` | Node launch | `src/nav_docking/launch/` |
| `calib.yaml` | Camera calibration | `src/camera_publisher/config/` |

---

## Performance Metrics

| Metric | Value | Notes |
|--------|-------|-------|
| **Control Frequency** | 30 Hz | Timer-based publishing |
| **Position Accuracy** | ±1 mm | Stage 5 final docking |
| **Approach Accuracy** | ±1 cm | Stage 4 alignment |
| **Marker Detection Rate** | Variable | Depends on lighting/distance |
| **Marker Delay Threshold** | 0.2 sec | Dual → Single fallback |
| **Max Linear Speed** | 0.1 m/s | Safety limit |
| **Min Linear Speed** | 0.005 m/s | PID output minimum |

---

## Quick Diagnostic Commands

```bash
# Check docking node status
ros2 node info /nav_docking_node

# Monitor marker detection
ros2 topic echo /aruco_detect/markers_left
ros2 topic echo /aruco_detect/markers_right

# Monitor docking commands
ros2 topic echo /cmd_vel_final

# Check action server
ros2 action list
ros2 action info /dock

# View transforms
ros2 run tf2_ros tf2_echo base_link aruco_marker_20
ros2 run tf2_ros tf2_echo base_link aruco_marker_21
```

---

## Coordinate Frames

```
        map
         │
         ├─ base_link (robot center)
         │   │
         │   ├─ camera_front_left_frame
         │   │   └─ aruco_marker_20 (left)
         │   │
         │   └─ camera_front_right_frame
         │       └─ aruco_marker_21 (right)
         │
         └─ ... (other frames)
```

---

## Common Issues & Solutions

| Issue | Likely Cause | Solution |
|-------|--------------|----------|
| Docking never completes | Marker not visible | Check camera placement |
| Oscillation at final stage | PID gains too high | Reduce Ki gains |
| Robot stops mid-approach | Marker delay timeout | Check lighting, marker size |
| Incorrect final position | Wrong offset params | Calibrate `aruco_distance_offset_dual` |
| Stage 4 → Stage 5 no transition | Alignment threshold not met | Check Y-axis error |

---

## Key Files Reference

| Component | File Path | Lines of Code |
|-----------|-----------|---------------|
| Main Controller | `src/nav_docking/src/nav_docking.cpp` | 573 |
| Header | `src/nav_docking/include/nav_docking/nav_docking.h` | 139 |
| PID Config | `src/nav_docking/config/docking_pid_params.yaml` | 15 |
| Launch | `src/nav_docking/launch/nav_docking.launch.py` | ~50 |

---

## Dependencies

- **ROS2 Humble**
- **OpenCV 4.x** (with ArUco)
- **TF2** (transforms)
- **nav_interface** (action definitions)
- **geometry_msgs, sensor_msgs**

---

*For detailed analysis, see `architecture-overview.md` and `requirements-docking.md`*
