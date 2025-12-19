# ROS 2 Topics Interface Specification

**Document ID:** INTERFACE-TOPICS-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft

---

## 1. Overview

This document specifies all ROS 2 topics used for inter-subsystem communication in the outdoor wheelchair transport robot system.

**Topic Naming Convention:**
- System-level: `/robot_status`, `/emergency_stop`
- Subsystem-specific: `/<subsystem>/<function>` (e.g., `/swerve/wheel_commands`)
- Sensor data: `/sensors/<sensor_type>` (e.g., `/sensors/lidar`)

**QoS Policies:**
- **RELIABLE**: Critical commands, mission data (ensures delivery)
- **BEST_EFFORT**: Sensor data, status updates (prioritizes freshness)

---

## 2. System-Level Topics

### 2.1 Emergency Stop

**Topic:** `/emergency_stop`
**Message Type:** `std_msgs/msg/Bool`
**QoS:** `RELIABLE`, `TRANSIENT_LOCAL`, History depth=1
**Publisher:** UI, Physical E-Stop Button Handler
**Subscribers:** All subsystems (mandatory)

**Description:** Emergency stop signal. When `data=true`, all subsystems must immediately halt motion and enter safe state.

```yaml
# std_msgs/msg/Bool
bool data  # true=STOP, false=RESET
```

---

### 2.2 Robot Status

**Topic:** `/robot_status`
**Message Type:** `custom_msgs/msg/RobotStatus`
**QoS:** `BEST_EFFORT`, `VOLATILE`, History depth=1, 10 Hz
**Publisher:** System Monitor
**Subscribers:** UI, Mission Manager, Fleet Management (future)

**Message Definition:**
```yaml
# custom_msgs/msg/RobotStatus.msg
Header header

# Battery
float32 battery_voltage      # V (e.g., 48.5)
float32 battery_percent      # % (0-100)
float32 battery_current      # A (negative=discharging)

# Localization
float32 localization_quality # [0.0, 1.0] (NDT fitness score normalized)
geometry_msgs/Pose pose      # Current pose in map frame

# Safety
string safety_state          # NORMAL, DEGRADED, EMERGENCY_STOP
string[] active_faults       # List of active fault codes

# Mission
string mission_id            # Current mission ID (empty if idle)
string current_waypoint      # Current waypoint name
uint8 waypoints_completed    # Number completed
uint8 total_waypoints        # Total in route

# System health
float32 cpu_usage_percent    # %
float32 memory_usage_percent # %
float32 disk_usage_percent   # %
```

---

### 2.3 Passenger Attached

**Topic:** `/passenger_attached`
**Message Type:** `std_msgs/msg/Bool`
**QoS:** `RELIABLE`, `TRANSIENT_LOCAL`, History depth=1
**Publisher:** Docking Controller
**Subscribers:** Swerve Controller (velocity limiting), Safety Monitor, eHMI

**Description:** Indicates if wheelchair passenger is currently docked and onboard.

---

## 3. Swerve Drive Topics

### 3.1 Wheel Commands

**Topic:** `/swerve/wheel_commands`
**Message Type:** `custom_msgs/msg/SwerveModuleCommandArray`
**QoS:** `RELIABLE`, History depth=1, 20 Hz
**Publisher:** Swerve Drive Controller
**Subscribers:** Swerve Drive Hardware Interface

**Message Definition:**
```yaml
# custom_msgs/msg/SwerveModuleCommand.msg
uint8 module_id              # 0=FL, 1=FR, 2=RL, 3=RR
float64 drive_velocity       # m/s (signed, positive=forward)
float64 steering_angle       # radians [-π, π]

---

# custom_msgs/msg/SwerveModuleCommandArray.msg
Header header
SwerveModuleCommand[4] modules  # Fixed size array
```

---

### 3.2 Joint States

**Topic:** `/joint_states`
**Message Type:** `sensor_msgs/msg/JointState`
**QoS:** `BEST_EFFORT`, History depth=1, 50 Hz
**Publisher:** Swerve Drive Hardware Interface
**Subscribers:** Swerve Drive Controller, Odometry Publisher

**Description:** Encoder feedback from all 8 joints (4 drive + 4 steering).

```yaml
# sensor_msgs/msg/JointState
Header header
string[] name          # ["fl_drive", "fl_steer", "fr_drive", ...]
float64[] position     # radians (steering angles)
float64[] velocity     # rad/s or m/s (drive velocities)
float64[] effort       # N⋅m (optional)
```

---

### 3.3 Odometry

**Topic:** `/odom`
**Message Type:** `nav_msgs/msg/Odometry`
**QoS:** `BEST_EFFORT`, History depth=1, 50 Hz
**Publisher:** Swerve Drive Controller
**Subscribers:** Navigation, Localization (sensor fusion)

**Description:** Dead-reckoning odometry from wheel encoders (odom → base_link transform).

---

## 4. Navigation Topics

### 4.1 Command Velocity

**Topic:** `/cmd_vel`
**Message Type:** `geometry_msgs/msg/Twist`
**QoS:** `RELIABLE`, History depth=1, 20 Hz
**Publisher:** Navigation Controller (Nav2)
**Subscribers:** Swerve Drive Controller, Safety Monitor

**Description:** Commanded velocity in base_link frame.

```yaml
# geometry_msgs/msg/Twist
Vector3 linear   # [x, y, z] m/s (z unused for ground robot)
Vector3 angular  # [roll, pitch, yaw] rad/s (only yaw used)
```

---

### 4.2 Current Pose

**Topic:** `/current_pose`
**Message Type:** `geometry_msgs/msg/PoseStamped`
**QoS:** `BEST_EFFORT`, History depth=1, 10 Hz
**Publisher:** NDT Localization Node
**Subscribers:** Navigation Controller, Mission Manager, UI

**Description:** Current robot pose in map frame (from NDT localization).

---

### 4.3 Goal Pose

**Topic:** `/goal_pose`
**Message Type:** `geometry_msgs/msg/PoseStamped`
**QoS:** `RELIABLE`, History depth=1
**Publisher:** Mission Manager, UI (manual goal)
**Subscribers:** Navigation Controller (Nav2)

**Description:** Single navigation goal pose in map frame.

---

## 5. Perception Topics

### 5.1 Point Cloud (Raw)

**Topic:** `/sensors/lidar/points`
**Message Type:** `sensor_msgs/msg/PointCloud2`
**QoS:** `BEST_EFFORT`, History depth=1, 10 Hz
**Publisher:** LiDAR Driver
**Subscribers:** Perception Pipeline, NDT Localization, Visualization

**Description:** Raw 3D point cloud from LiDAR sensor (~150k points).

---

### 5.2 Obstacle Cloud

**Topic:** `/perception/obstacles`
**Message Type:** `sensor_msgs/msg/PointCloud2`
**QoS:** `BEST_EFFORT`, History depth=1, 20 Hz
**Publisher:** Perception Pipeline (after ground removal)
**Subscribers:** Costmap Layer, Collision Predictor

**Description:** Filtered obstacle points (non-ground, 0.1m-2.0m height).

---

### 5.3 Costmap

**Topic:** `/local_costmap/costmap`
**Message Type:** `nav_msgs/msg/OccupancyGrid`
**QoS:** `BEST_EFFORT`, History depth=1, 5 Hz
**Publisher:** Nav2 Costmap Server
**Subscribers:** Navigation Planner, UI (visualization)

**Description:** 2D local costmap for obstacle avoidance.

---

## 6. Docking Topics

### 6.1 ArUco Detections

**Topic:** `/docking/aruco_detections`
**Message Type:** `custom_msgs/msg/ArucoDetectionArray`
**QoS:** `BEST_EFFORT`, History depth=1, 10 Hz
**Publisher:** ArUco Detector
**Subscribers:** Docking Controller, UI (debug visualization)

**Message Definition:**
```yaml
# custom_msgs/msg/ArucoDetection.msg
Header header
int32 marker_id              # ArUco marker ID (0-3)
geometry_msgs/Pose pose      # Marker pose in camera frame
float32 quality_score        # Detection quality [0.0, 1.0]
string camera_frame          # "front_camera" or "rear_camera"

---

# custom_msgs/msg/ArucoDetectionArray.msg
Header header
ArucoDetection[] detections
```

---

### 6.2 Docking Status

**Topic:** `/docking/status`
**Message Type:** `custom_msgs/msg/DockingStatus`
**QoS:** `BEST_EFFORT`, History depth=1, 10 Hz
**Publisher:** Docking Controller
**Subscribers:** Mission Manager, UI, eHMI

**Message Definition:**
```yaml
# custom_msgs/msg/DockingStatus.msg
Header header
string state                 # IDLE, APPROACH, SEARCH, SERVOING, FINE_ALIGN, DOCKED, FAILED
float32 position_error       # meters (distance to target)
float32 orientation_error    # radians (yaw error)
int32 visible_markers_count  # Number of markers currently visible
float32 convergence_progress # [0.0, 1.0]
```

---

## 7. Safety Topics

### 7.1 Safety Status

**Topic:** `/safety/status`
**Message Type:** `custom_msgs/msg/SafetyStatus`
**QoS:** `BEST_EFFORT`, `VOLATILE`, History depth=1, 10 Hz
**Publisher:** Safety Monitor
**Subscribers:** All subsystems, UI

**Message Definition:**
```yaml
# custom_msgs/msg/SafetyStatus.msg
Header header
string safety_state          # NORMAL, DEGRADED, COLLISION_AVOIDANCE, EMERGENCY_STOP
string[] active_faults       # Fault codes (e.g., "LIDAR_TIMEOUT", "LOCALIZATION_LOST")
float32 min_obstacle_distance # meters (closest obstacle)
bool emergency_stop_active   # Hardware E-Stop status
```

---

### 7.2 Velocity Limit

**Topic:** `/safety/velocity_limit`
**Message Type:** `custom_msgs/msg/VelocityLimit`
**QoS:** `RELIABLE`, `TRANSIENT_LOCAL`, History depth=1
**Publisher:** Safety Monitor
**Subscribers:** Swerve Drive Controller

**Message Definition:**
```yaml
# custom_msgs/msg/VelocityLimit.msg
Header header
float32 max_linear_velocity  # m/s
float32 max_angular_velocity # rad/s
string reason                # Reason for limit (e.g., "PASSENGER_ONBOARD", "SLOPE")
```

---

## 8. UI/eHMI Topics

### 8.1 eHMI State Command

**Topic:** `/ehmi/state_command`
**Message Type:** `custom_msgs/msg/eHMIState`
**QoS:** `RELIABLE`, History depth=1
**Publisher:** Mission Manager, Docking Controller
**Subscribers:** eHMI Serial Bridge

**Message Definition:**
```yaml
# custom_msgs/msg/eHMIState.msg
Header header
uint8 state_num              # 0-28 (wheelchair states 21-28)
string language              # "en", "ja", "zh"
uint8 volume                 # 0-10
```

---

### 8.2 UI Commands

**Topic:** `/ui/command`
**Message Type:** `custom_msgs/msg/UICommand`
**QoS:** `RELIABLE`, History depth=10
**Publisher:** Touch Screen UI
**Subscribers:** Mission Manager

**Message Definition:**
```yaml
# custom_msgs/msg/UICommand.msg
Header header
string command_type          # "START_MISSION", "PAUSE", "RESUME", "ABORT", "EMERGENCY_STOP"
string mission_id            # Mission ID (if applicable)
```

---

## 9. Sensor Topics

### 9.1 IMU Data

**Topic:** `/sensors/imu`
**Message Type:** `sensor_msgs/msg/Imu`
**QoS:** `BEST_EFFORT`, History depth=1, 100 Hz
**Publisher:** IMU Driver
**Subscribers:** Perception (ground removal), Localization (sensor fusion)

---

### 9.2 Camera Images

**Topic:** `/sensors/camera/front/image_raw`
**Message Type:** `sensor_msgs/msg/Image`
**QoS:** `BEST_EFFORT`, History depth=1, 10 Hz
**Publisher:** Camera Driver (front)
**Subscribers:** ArUco Detector

**Topic:** `/sensors/camera/rear/image_raw`
**Message Type:** `sensor_msgs/msg/Image`
**QoS:** `BEST_EFFORT`, History depth=1, 10 Hz
**Publisher:** Camera Driver (rear)
**Subscribers:** ArUco Detector

---

## 10. Topic Summary Table

| Topic | Type | QoS | Rate | Publisher | Subscribers |
|-------|------|-----|------|-----------|-------------|
| `/emergency_stop` | Bool | RELIABLE | Event | UI, E-Stop | All subsystems |
| `/robot_status` | RobotStatus | BEST_EFFORT | 10 Hz | System Monitor | UI, Mission Mgr |
| `/passenger_attached` | Bool | RELIABLE | Event | Docking Ctrl | Swerve, Safety, eHMI |
| `/swerve/wheel_commands` | SwerveModuleCommandArray | RELIABLE | 20 Hz | Swerve Ctrl | HW Interface |
| `/joint_states` | JointState | BEST_EFFORT | 50 Hz | HW Interface | Swerve Ctrl |
| `/odom` | Odometry | BEST_EFFORT | 50 Hz | Swerve Ctrl | Navigation, Localization |
| `/cmd_vel` | Twist | RELIABLE | 20 Hz | Nav2 | Swerve Ctrl, Safety |
| `/current_pose` | PoseStamped | BEST_EFFORT | 10 Hz | NDT Localization | Navigation, UI |
| `/sensors/lidar/points` | PointCloud2 | BEST_EFFORT | 10 Hz | LiDAR Driver | Perception, NDT |
| `/perception/obstacles` | PointCloud2 | BEST_EFFORT | 20 Hz | Perception | Costmap, Safety |
| `/docking/aruco_detections` | ArucoDetectionArray | BEST_EFFORT | 10 Hz | ArUco Detector | Docking Ctrl |
| `/docking/status` | DockingStatus | BEST_EFFORT | 10 Hz | Docking Ctrl | Mission, UI, eHMI |
| `/safety/status` | SafetyStatus | BEST_EFFORT | 10 Hz | Safety Monitor | All, UI |
| `/ehmi/state_command` | eHMIState | RELIABLE | Event | Mission, Docking | eHMI Bridge |

**Total Topics:** 20+ core topics (additional debug/visualization topics not listed)

---

**Document Status:** Draft
**Implementation Status:** Ready for ROS 2 implementation
**Approvals Required:** Systems Architect, ROS Integration Lead
