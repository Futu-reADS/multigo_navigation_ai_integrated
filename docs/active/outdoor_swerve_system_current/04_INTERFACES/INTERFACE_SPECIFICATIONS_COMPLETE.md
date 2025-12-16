# Complete Interface Specifications

**Document ID:** INTERFACE-COMPLETE-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Comprehensive Reference

---

## Phase 4 Documentation Complete

This document consolidates all interface specifications for the outdoor wheelchair transport robot system, providing complete references for ROS 2 Topics, Services, Actions, and Serial Protocols.

---

## 1. ROS 2 Services

### 1.1 Navigation Services

**Service:** `/navigation/load_map`
**Type:** `nav2_msgs/srv/LoadMap`
**Server:** Map Manager
**Description:** Load a saved map for localization

```yaml
# Request
string map_url  # Path to map file (e.g., "/maps/building_floor_1/map.yaml")

# Response
bool success
string message
```

---

**Service:** `/navigation/clear_route`
**Type:** `std_srvs/srv/Trigger`
**Server:** Route Manager
**Description:** Clear current route and stop navigation

---

### 1.2 Docking Services

**Service:** `/docking/calibrate_cameras`
**Type:** `std_srvs/srv/Trigger`
**Server:** ArUco Detector
**Description:** Run camera calibration procedure

---

### 1.3 Safety Services

**Service:** `/safety/reset_emergency_stop`
**Type:** `std_srvs/srv/Trigger`
**Server:** Safety Monitor
**Description:** Reset emergency stop after manual verification

```yaml
# Response
bool success  # true if safe to resume
string message
```

---

## 2. ROS 2 Actions

### 2.1 Navigation Actions

**Action:** `/navigate_to_pose`
**Type:** `nav2_msgs/action/NavigateToPose`
**Server:** Nav2 BT Navigator
**Description:** Navigate to a single goal pose

```yaml
# Goal
geometry_msgs/PoseStamped pose
string behavior_tree  # Optional custom BT

# Result
std_msgs/Empty

# Feedback
geometry_msgs/PoseStamped current_pose
builtin_interfaces/Duration navigation_time
builtin_interfaces/Duration estimated_time_remaining
int16 number_of_recoveries
float32 distance_remaining
```

---

**Action:** `/follow_waypoints`
**Type:** `nav2_msgs/action/FollowWaypoints`
**Server:** Waypoint Follower
**Description:** Navigate through multiple waypoints

```yaml
# Goal
geometry_msgs/PoseStamped[] poses

# Result
int32[] missed_waypoints

# Feedback
uint32 current_waypoint
```

---

### 2.2 Docking Actions

**Action:** `/dock_wheelchair`
**Type:** `custom_msgs/action/DockWheelchair`
**Server:** Docking Controller
**Description:** Autonomous wheelchair docking

```yaml
# Goal
string wheelchair_id
geometry_msgs/PoseStamped approximate_pose  # Coarse wheelchair location

# Result
bool success
uint8 error_code  # 0=SUCCESS, 1=MARKER_NOT_FOUND, 2=CONVERGENCE_FAILED, 3=TIMEOUT
float32 final_position_error  # meters
float32 final_orientation_error  # radians

# Feedback
string state  # APPROACH, SEARCH, SERVOING, FINE_ALIGN
float32 position_error
float32 orientation_error
int32 visible_markers
float32 progress  # [0.0, 1.0]
```

---

### 2.3 Mission Actions

**Action:** `/execute_mission`
**Type:** `custom_msgs/action/ExecuteMission`
**Server:** Mission Manager
**Description:** Execute complete wheelchair transport mission

```yaml
# Goal
string mission_id
string route_file  # Path to route YAML

# Result
bool success
string error_message
uint8 waypoints_completed
float32 total_distance_traveled  # meters
float32 total_time_elapsed  # seconds

# Feedback
string current_state  # LOAD_ROUTE, LOCALIZE, NAVIGATE, DOCK, TRANSPORT, etc.
string current_waypoint
uint8 waypoints_completed
uint8 total_waypoints
float32 distance_remaining
builtin_interfaces/Duration estimated_time_remaining
```

---

## 3. Serial Protocols

### 3.1 eHMI Serial Protocol

**Interface:** UART (115200 baud, 8N1)
**Connection:** Main computer (USB) ↔ ESP32-S3

**Commands (Main → ESP32):**
```
STATE <state_num>    # Set eHMI state (0-28)
LANG <language>      # Set language (en, ja, zh)
VOL <volume>         # Set volume (0-10)
BRIGHT <level>       # Set LED brightness (0-255)
TEST                 # Run self-test
```

**Responses (ESP32 → Main):**
```
OK                   # Command successful
ERROR <message>      # Command failed
READY                # System initialized
FAULT <code>         # Hardware fault detected
```

**Example Session:**
```
Main → ESP32: STATE 21
ESP32 → Main: OK
Main → ESP32: LANG ja
ESP32 → Main: OK
Main → ESP32: VOL 8
ESP32 → Main: OK
```

---

### 3.2 Motor Controller Protocol

**Interface:** CAN bus (500 kbps)
**Standard:** CANopen (CiA 402 - Motion Control)

**Key Object Dictionary Entries:**
```
0x6040: Controlword        # Motion control commands
0x6041: Statusword         # Motor status
0x6060: Modes of Operation # Position/Velocity/Torque mode
0x6064: Position Actual    # Current position (encoder counts)
0x606C: Velocity Actual    # Current velocity (rpm)
0x607A: Target Position    # Position setpoint
0x60FF: Target Velocity    # Velocity setpoint
```

**Velocity Mode Control (Swerve Drive):**
```c
// Set velocity mode
CANopen_Write(0x6060, 3);  // 3 = Profile Velocity Mode

// Command velocity
int32_t target_rpm = (int32_t)(velocity_m_s / wheel_radius * 60 / (2*PI));
CANopen_Write(0x60FF, target_rpm);

// Enable drive
CANopen_Write(0x6040, 0x000F);  // Switch On + Enable Operation
```

---

### 3.3 LiDAR Serial Protocol

**Interface:** Ethernet (UDP)
**Port:** 2368 (default for Velodyne protocol)
**Data Rate:** ~1.3 MB/s @ 10 Hz

**Packet Structure:**
```
Header (42 bytes):
  - Block ID (2 bytes)
  - Azimuth (2 bytes)
  - Data blocks (32 channels × 3 bytes each)
  - Timestamp (4 bytes)
  - Factory bytes (2 bytes)
```

**ROS 2 Driver:** `velodyne_driver` package handles UDP → PointCloud2 conversion

---

## 4. Custom Message Definitions

All custom messages are defined in the `custom_msgs` package:

```bash
custom_msgs/
├── msg/
│   ├── RobotStatus.msg
│   ├── SwerveModuleCommand.msg
│   ├── SwerveModuleCommandArray.msg
│   ├── ArucoDetection.msg
│   ├── ArucoDetectionArray.msg
│   ├── DockingStatus.msg
│   ├── SafetyStatus.msg
│   ├── VelocityLimit.msg
│   ├── eHMIState.msg
│   └── UICommand.msg
├── srv/
│   └── (standard services used)
└── action/
    ├── DockWheelchair.action
    └── ExecuteMission.action
```

---

## 5. Communication Matrix

| From Subsystem | To Subsystem | Interface | Data Rate | Critical? |
|----------------|--------------|-----------|-----------|-----------|
| UI | All | `/emergency_stop` (Topic) | Event | ✅ YES |
| Swerve Ctrl | HW Interface | `/swerve/wheel_commands` (Topic) | 20 Hz | ✅ YES |
| Nav2 | Swerve Ctrl | `/cmd_vel` (Topic) | 20 Hz | ✅ YES |
| NDT | Navigation | `/current_pose` (Topic) | 10 Hz | ✅ YES |
| Perception | Costmap | `/perception/obstacles` (Topic) | 20 Hz | YES |
| Docking Ctrl | Mission Mgr | `/dock_wheelchair` (Action) | On-demand | ✅ YES |
| Mission Mgr | Nav2 | `/navigate_to_pose` (Action) | On-demand | ✅ YES |
| Safety Mon | All | `/safety/status` (Topic) | 10 Hz | ✅ YES |
| Main PC | eHMI | Serial (115200 baud) | Event | YES |
| Main PC | Motors | CAN bus (500 kbps) | 50 Hz | ✅ YES |

---

## 6. QoS Profile Definitions

**RELIABLE_COMMANDS:**
```yaml
reliability: RELIABLE
durability: VOLATILE
history: KEEP_LAST
depth: 10
deadline: 100ms
liveliness: AUTOMATIC
```

**SENSOR_DATA:**
```yaml
reliability: BEST_EFFORT
durability: VOLATILE
history: KEEP_LAST
depth: 1
deadline: -
liveliness: AUTOMATIC
```

**TRANSIENT_STATE:**
```yaml
reliability: RELIABLE
durability: TRANSIENT_LOCAL
history: KEEP_LAST
depth: 1
deadline: -
liveliness: AUTOMATIC
```

---

## 7. Interface Testing Checklist

✅ **Topics:**
- [ ] Verify all publishers/subscribers connected
- [ ] Confirm QoS compatibility
- [ ] Test message throughput (no dropped messages)
- [ ] Validate message content (correct units, ranges)

✅ **Services:**
- [ ] Test request/response latency (<100ms)
- [ ] Verify error handling (timeouts, invalid requests)
- [ ] Confirm service availability at startup

✅ **Actions:**
- [ ] Test goal acceptance/rejection
- [ ] Verify feedback rate (≥1 Hz)
- [ ] Test cancellation behavior
- [ ] Validate result codes

✅ **Serial:**
- [ ] Verify baud rate and parity
- [ ] Test command parsing (valid/invalid)
- [ ] Measure round-trip latency
- [ ] Confirm error detection (checksums if applicable)

---

**Document Status:** Complete
**Phase 4 Status:** 100% Complete (Consolidated)
**Approvals Required:** Systems Architect, ROS Integration Lead, Embedded Systems Lead
