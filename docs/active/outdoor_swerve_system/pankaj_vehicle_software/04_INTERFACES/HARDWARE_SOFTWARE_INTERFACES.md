# Hardware-Software Interface Specification

**Document ID:** INT-HW-SW-001
**Version:** 1.0
**Date:** 2025-12-16
**Status:** Draft - CRITICAL (Week 1 Priority 3)
**Classification:** Internal

**Responsible Teams:**
- **Software:** Pankaj (Vehicle Software - ROS 2 Integration)
- **Hardware:** Tsuchiya (Mechanical, Electrical, Sensors, Power) + Kiril (Exterior, eHMI)

**Purpose:** Define exact interfaces between vehicle software (ROS 2) and hardware components (sensors, actuators, power system, eHMI).

---

## Document Control

| Version | Date | Authors | Changes |
|---------|------|---------|---------|
| 1.0 | 2025-12-16 | Pankaj + Tsuchiya + Kiril | Initial hardware-software interface specification |

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [ROS 2 Topics - Sensors to Software](#2-ros-2-topics---sensors-to-software)
3. [ROS 2 Topics - Software to Actuators](#3-ros-2-topics---software-to-actuators)
4. [Serial Protocols](#4-serial-protocols)
5. [Power Management Interface](#5-power-management-interface)
6. [Safety System Interface](#6-safety-system-interface)
7. [Coordinate Frames (TF2)](#7-coordinate-frames-tf2)
8. [Testing & Validation](#8-testing--validation)

---

## 1. Introduction

### 1.1 Purpose

This document defines **exact interfaces** between:
- **ROS 2 Software** (navigation, docking, perception, safety)
- **Hardware Components** (sensors, motors, battery, eHMI)

**Critical Dependencies:**
- Software CANNOT operate without sensors publishing to these topics
- Hardware CANNOT be controlled without software publishing commands
- Both teams MUST implement exactly as specified

---

### 1.2 Scope

**In Scope:**
- ✅ ROS 2 topic specifications (message types, QoS, frequency)
- ✅ Serial protocols (eHMI UART, motor CAN bus, BMS)
- ✅ Power management interfaces (battery monitoring, charging)
- ✅ Safety interfaces (E-stop, bumpers, watchdog)
- ✅ Coordinate frames (TF2 transformations)

**Out of Scope:**
- ❌ Internal software algorithms (navigation, docking)
- ❌ Hardware mechanical design (CAD models)
- ❌ TVM server interface (see TVM_API_SPECIFICATION.md)

---

### 1.3 Related Documents

- **HARDWARE_REQUIREMENTS.md** - Complete hardware specifications
- **MECHANICAL_REQUIREMENTS.md** - Mechanical assembly details
- **ELECTRICAL_REQUIREMENTS.md** - Electrical system details
- **ROS2_TOPICS.md** - Complete ROS 2 topic reference (existing)
- **TVM_API_SPECIFICATION.md** - Vehicle ↔ TVM server interface

---

## 2. ROS 2 Topics - Sensors to Software

### 2.1 3D LiDAR Point Cloud

**Topic:** `/sensors/lidar/points`

**Message Type:** `sensor_msgs/msg/PointCloud2`

**Publisher:** LiDAR driver node (hardware interface)

**Subscribers:**
- `/perception/obstacle_detection`
- `/perception/ground_removal`
- `/localization/ndt_matcher`

**Frequency:** 10 Hz (minimum)

**QoS:**
```
Reliability: RELIABLE
Durability: VOLATILE
History: KEEP_LAST (depth=1)
Deadline: 150ms
Liveliness: AUTOMATIC
```

**Data Requirements:**
- Point format: XYZ (float32, meters)
- Coordinate frame: `lidar_link`
- Minimum points: 100,000 points/second
- Range: 0.5m to 100m (outdoor)
- Organized: False (unordered point cloud)

**Message Fields:**
```cpp
sensor_msgs::msg::PointCloud2 {
  std_msgs::msg::Header header;           // Timestamp + frame_id
  uint32 height = 1;                      // Unorganized cloud
  uint32 width = [variable];              // Number of points
  sensor_msgs::msg::PointField[] fields;  // XYZ fields
  bool is_bigendian = false;
  uint32 point_step = 12;                 // 3 floats (x,y,z) = 12 bytes
  uint32 row_step = width * point_step;
  uint8[] data;                           // Raw point data
  bool is_dense = false;                  // May contain NaN
}
```

**Hardware Requirements (Tsuchiya):**
- Install LiDAR driver (ROS 2 Humble compatible)
- Configure LiDAR IP/port (if Ethernet-based)
- Mount LiDAR at height: 0.6-0.8m
- Ensure clear 360° FOV (no occlusions)

---

### 2.2 Front-Left Camera (ArUco Detection)

**Topic:** `/sensors/camera/front_left/image_raw`

**Message Type:** `sensor_msgs/msg/Image`

**Publisher:** Camera driver node

**Subscribers:** `/docking/aruco_detector`

**Frequency:** 30 Hz (minimum)

**QoS:**
```
Reliability: BEST_EFFORT
Durability: VOLATILE
History: KEEP_LAST (depth=1)
```

**Data Requirements:**
- Resolution: 1920×1080 (Full HD minimum)
- Encoding: `bgr8` or `rgb8`
- Coordinate frame: `camera_front_left_link`
- Exposure: Auto-exposure enabled (outdoor lighting varies)

**Companion Topic - Camera Info:**

**Topic:** `/sensors/camera/front_left/camera_info`

**Message Type:** `sensor_msgs/msg/CameraInfo`

**Frequency:** 30 Hz (same as image)

**Data Requirements:**
- Intrinsic calibration matrix (3×3)
- Distortion coefficients (5 parameters: k1, k2, p1, p2, k3)
- Calibrated with checkerboard (9×6 or 8×6)

**Hardware Requirements (Tsuchiya):**
- Mount camera at height: 0.4-0.6m
- Angle: ~15° downward tilt (to see ground markers)
- Weatherproofing: IP66+ housing
- Cable length: USB 3.0, max 3m (active cable if longer)

---

### 2.3 Front-Right Camera (ArUco Detection)

**Topic:** `/sensors/camera/front_right/image_raw`

**Topic:** `/sensors/camera/front_right/camera_info`

**Specifications:** Same as front-left camera (see 2.2)

**Purpose:** Dual-camera fusion for improved docking accuracy

---

### 2.4 IMU (9-DOF)

**Topic:** `/sensors/imu`

**Message Type:** `sensor_msgs/msg/Imu`

**Publisher:** IMU driver node

**Subscribers:**
- `/localization/ekf_localization`
- `/safety/tilt_monitor`

**Frequency:** 50 Hz (minimum)

**QoS:**
```
Reliability: RELIABLE
Durability: VOLATILE
History: KEEP_LAST (depth=10)
Deadline: 30ms
```

**Data Requirements:**
- Accelerometer: ±4g range, 0.01g resolution
- Gyroscope: ±500°/s range, 0.1°/s resolution
- Magnetometer: ±4 gauss range (for heading)
- Coordinate frame: `imu_link`
- Calibration: Factory calibrated or runtime calibration

**Message Fields:**
```cpp
sensor_msgs::msg::Imu {
  std_msgs::msg::Header header;
  geometry_msgs::msg::Quaternion orientation;         // Not used (populated by EKF)
  double[9] orientation_covariance;                   // Set to [-1, ...]
  geometry_msgs::msg::Vector3 angular_velocity;       // rad/s (gyro)
  double[9] angular_velocity_covariance;
  geometry_msgs::msg::Vector3 linear_acceleration;    // m/s² (accel)
  double[9] linear_acceleration_covariance;
}
```

**Hardware Requirements (Tsuchiya):**
- Mount IMU near robot center of mass
- Align IMU axes with robot frame (X=forward, Y=left, Z=up)
- Secure mounting (no vibration)
- ROS 2 driver available (e.g., `imu_filter_madgwick`, `phidgets_imu`)

---

### 2.5 Wheel Encoders (Swerve Drive)

**Topics (4 topics, one per wheel):**
```
/swerve/front_left/encoder
/swerve/front_right/encoder
/swerve/rear_left/encoder
/swerve/rear_right/encoder
```

**Message Type:** Custom message `swerve_msgs/msg/WheelEncoder`

**Publisher:** Swerve drive controller node

**Subscribers:** `/swerve/odometry`, `/swerve/controller`

**Frequency:** 100 Hz (minimum)

**Custom Message Definition:**
```
# swerve_msgs/msg/WheelEncoder.msg
std_msgs/Header header
float64 drive_position      # Wheel rotation (radians)
float64 drive_velocity      # Wheel speed (rad/s)
float64 steer_position      # Steering angle (radians, -π to +π)
float64 steer_velocity      # Steering rate (rad/s)
```

**Hardware Requirements (Tsuchiya):**
- Encoder resolution: ≥1000 CPR (counts per revolution)
- Quadrature encoders (A/B channels)
- Interface: Via motor controller (CAN bus)
- Homing: Steering encoders must have absolute position or homing routine

---

### 2.6 Safety Sensors

#### **2.6.1 Bumper Sensors**

**Topic:** `/safety/bumper`

**Message Type:** `std_msgs/msg/Bool`

**Publisher:** Safety monitor node

**Frequency:** 50 Hz

**Data:** `true` if any bumper pressed, `false` otherwise

**Hardware Requirements (Tsuchiya):**
- Physical bumpers around perimeter (front, rear, sides)
- Normally-open switches (closed when pressed)
- Interface: GPIO or CAN bus
- Response time: <50ms

---

#### **2.6.2 Passenger Detection (Weight Sensor)**

**Topic:** `/safety/passenger_detected`

**Message Type:** `std_msgs/msg/Bool`

**Publisher:** Safety monitor node

**Frequency:** 10 Hz

**Data:** `true` if passenger onboard (weight >20kg)

**Hardware Requirements (Tsuchiya):**
- Load cells under wheelchair docking platform
- Threshold: 20kg minimum (to detect wheelchair + passenger)
- Interface: Analog input → ADC → CAN bus

---

#### **2.6.3 Emergency Stop Button**

**Topic:** `/safety/estop`

**Message Type:** `std_msgs/msg/Bool`

**Publisher:** Safety monitor node

**Frequency:** 50 Hz

**Data:** `true` if E-stop pressed, `false` otherwise

**Hardware Requirements (Tsuchiya):**
- Large red mushroom button (IEC 60947-5-5 Category 0)
- Latching (stays pressed until manually reset)
- Hardwired to motor controllers (cuts power immediately)
- Also publishes to ROS 2 for software awareness

---

### 2.7 Battery Management System (BMS)

**Topic:** `/power/battery_state`

**Message Type:** `sensor_msgs/msg/BatteryState`

**Publisher:** BMS interface node

**Subscribers:** `/power/manager`, `/tvm_client`

**Frequency:** 1 Hz (lower frequency acceptable)

**Message Fields:**
```cpp
sensor_msgs::msg::BatteryState {
  std_msgs::msg::Header header;
  float voltage;                    // Battery voltage (V)
  float temperature;                // Battery temp (°C)
  float current;                    // Current (A, negative = discharging)
  float charge;                     // Remaining charge (Ah)
  float capacity;                   // Full capacity (Ah)
  float design_capacity;            // Design capacity (Ah)
  float percentage;                 // State of Charge (0.0 to 1.0)
  uint8 power_supply_status;        // CHARGING / DISCHARGING / FULL
  uint8 power_supply_health;        // GOOD / OVERHEAT / DEAD / etc.
  uint8 power_supply_technology;    // LION / LIPO / etc.
  bool present = true;
}
```

**Hardware Requirements (Tsuchiya):**
- BMS with CAN bus interface (CANopen or custom protocol)
- Voltage monitoring per cell (48V = 13S Li-ion, 13 cells)
- Current sensing: ±100A range, ±0.5A accuracy
- Temperature sensors: ≥2 sensors (different locations)
- SoC estimation: Coulomb counting + voltage curve

---

## 3. ROS 2 Topics - Software to Actuators

### 3.1 Swerve Drive Command

**Topic:** `/cmd_vel`

**Message Type:** `geometry_msgs/msg/Twist`

**Publisher:** Navigation controller (`/nav2/controller`)

**Subscriber:** Swerve drive controller

**Frequency:** 20 Hz (navigation controller publishes commands)

**QoS:**
```
Reliability: RELIABLE
Durability: VOLATILE
History: KEEP_LAST (depth=1)
Deadline: 100ms (stop if no command for >100ms)
```

**Message Fields:**
```cpp
geometry_msgs::msg::Twist {
  geometry_msgs::msg::Vector3 linear;   // linear.x = forward vel (m/s)
  geometry_msgs::msg::Vector3 angular;  // angular.z = rotation vel (rad/s)
}
```

**Coordinate Frame:** `base_link` (robot-centric)

**Velocity Limits:**
- `linear.x`: -1.0 to +1.5 m/s (forward/backward)
- `linear.y`: -1.0 to +1.0 m/s (strafe left/right) - **swerve drive only**
- `angular.z`: -1.0 to +1.0 rad/s (rotation)

**Hardware Action (Tsuchiya):**
Swerve controller receives Twist → computes inverse kinematics → sends motor commands via CAN bus

---

### 3.2 Swerve Drive Individual Wheel Commands

**Topic:** `/swerve/command`

**Message Type:** Custom message `swerve_msgs/msg/SwerveDriveCommand`

**Publisher:** Swerve drive controller

**Subscriber:** Motor controller interface node

**Frequency:** 50 Hz

**Custom Message Definition:**
```
# swerve_msgs/msg/SwerveDriveCommand.msg
std_msgs/Header header
SwerveModuleCommand[] modules  # 4 modules (FL, FR, RL, RR)

# SwerveModuleCommand.msg
string module_name              # "front_left", "front_right", "rear_left", "rear_right"
float64 drive_velocity          # Wheel speed (rad/s)
float64 steer_position          # Steering angle (rad, -π to +π)
```

**Hardware Action (Tsuchiya):**
Motor controller receives commands → drives 4 wheel motors + 4 steering motors via CAN bus

---

### 3.3 eHMI State Command

**Topic:** `/ehmi/state`

**Message Type:** `std_msgs/msg/UInt8`

**Publisher:** State manager node (`/vehicle/state_manager`)

**Subscriber:** eHMI controller (ESP32 serial bridge)

**Frequency:** Event-driven (on state change) + 1 Hz heartbeat

**QoS:**
```
Reliability: RELIABLE
Durability: TRANSIENT_LOCAL (last state retained)
History: KEEP_LAST (depth=1)
```

**State Values (see EHMI_SYSTEM_REFERENCE.md):**
| Value | State | Description |
|-------|-------|-------------|
| 0 | OFF | System off |
| 1 | IDLE | Idle, waiting for mission |
| 10 | NAVIGATING | Actively moving |
| 21 | DOCKING_APPROACH | Approaching wheelchair (NEW) |
| 22 | DOCKING_VISUAL_SERVO | Visual servoing for docking (NEW) |
| 23 | DOCKING_ATTACHED | Wheelchair attached (NEW) |
| 24 | TRANSPORTING_PASSENGER | Moving with passenger onboard (NEW) |
| 30 | CHARGING | Charging at station |
| 40 | ERROR | Error state (details via audio) |
| 50 | EMERGENCY_STOP | E-stop activated |

**Hardware Action (Kiril):**
ESP32 receives state → updates LED patterns + plays audio announcement

---

### 3.4 eHMI Audio Command

**Topic:** `/ehmi/audio`

**Message Type:** `std_msgs/msg/String`

**Publisher:** Various nodes (navigation, docking, safety)

**Subscriber:** eHMI controller

**Frequency:** Event-driven

**Data:** Audio file name (e.g., `"start_navigation_ja.mp3"`, `"docking_complete_en.mp3"`)

**Hardware Action (Kiril):**
ESP32 receives filename → plays audio from SD card via I2S

---

## 4. Serial Protocols

### 4.1 eHMI Serial Protocol (UART)

**Interface:** UART (USB-Serial adapter)

**Parameters:**
- Baud rate: 115200
- Data bits: 8
- Stop bits: 1
- Parity: None
- Flow control: None

**Protocol:** ASCII text commands (newline-terminated)

**Commands (ROS → ESP32):**

| Command | Format | Example | Description |
|---------|--------|---------|-------------|
| State | `STATE:<value>` | `STATE:21` | Set eHMI state (0-50) |
| Audio | `AUDIO:<filename>` | `AUDIO:docking_ja.mp3` | Play audio file |
| Volume | `VOLUME:<0-100>` | `VOLUME:80` | Set audio volume (%) |
| Language | `LANG:<en/ja/zh>` | `LANG:ja` | Set language |
| LED brightness | `BRIGHTNESS:<0-255>` | `BRIGHTNESS:200` | Set LED brightness |

**Responses (ESP32 → ROS):**

| Response | Format | Example | Description |
|----------|--------|---------|-------------|
| OK | `OK` | `OK` | Command acknowledged |
| Error | `ERROR:<message>` | `ERROR:File not found` | Command failed |
| State | `STATE:<value>` | `STATE:21` | Current state |

**Example Exchange:**
```
ROS → ESP32: "STATE:21\n"
ESP32 → ROS: "OK\n"

ROS → ESP32: "AUDIO:docking_approach_ja.mp3\n"
ESP32 → ROS: "OK\n"
```

**Hardware Requirements (Kiril):**
- ESP32-S3 with USB-Serial interface
- SD card for audio files (≥2GB)
- I2S audio amplifier (e.g., MAX98357A)
- WS2812B LED strip (≥60 LEDs/meter, 2-3 meters)
- HUB75 LED matrix (64×32 or 128×64, optional)

**Software Requirements (Pankaj):**
- ROS 2 serial bridge node (`/ehmi/serial_bridge`)
- Publishes `/ehmi/state` → sends `STATE` command
- Subscribes `/ehmi/audio` → sends `AUDIO` command

---

### 4.2 Motor Controller CAN Bus

**Interface:** CAN bus (Controller Area Network)

**Parameters:**
- Baud rate: 500 kbit/s (standard)
- Protocol: CANopen (CiA 402 - motion control)
- Node IDs: 1-8 (4 drive motors + 4 steering motors)

**CAN IDs:**
| Node ID | Motor | CAN ID (Base) | Description |
|---------|-------|---------------|-------------|
| 1 | FL Drive | 0x201 | Front-left wheel drive |
| 2 | FL Steer | 0x202 | Front-left steering |
| 3 | FR Drive | 0x203 | Front-right wheel drive |
| 4 | FR Steer | 0x204 | Front-right steering |
| 5 | RL Drive | 0x205 | Rear-left wheel drive |
| 6 | RL Steer | 0x206 | Rear-left steering |
| 7 | RR Drive | 0x207 | Rear-right wheel drive |
| 8 | RR Steer | 0x208 | Rear-right steering |

**CANopen SDO (Service Data Object):**
- Configuration, parameter setting
- Object dictionary (OD) access

**CANopen PDO (Process Data Object):**
- Real-time control (velocity commands)
- Status feedback (position, velocity, current)

**Example PDO Mapping (Velocity Mode):**

**TPDO (Transmit PDO - Motor → ROS):**
- Position actual value (encoder ticks)
- Velocity actual value (RPM)
- Current actual value (mA)

**RPDO (Receive PDO - ROS → Motor):**
- Target velocity (RPM)
- Control word (enable, disable, reset)

**Hardware Requirements (Tsuchiya):**
- Motor controllers with CANopen support (e.g., ODrive, VESC, or custom)
- CAN bus termination resistors (120Ω at each end)
- CAN bus shield/isolation (EMI protection)
- Power: 48V input, ≥80W per motor

**Software Requirements (Pankaj):**
- ROS 2 CANopen driver (`ros2_canopen` package)
- EDS files for motor controllers (Electronic Data Sheet)
- Swerve controller node sends velocity commands via PDO

---

### 4.3 Battery Management System CAN Bus

**Interface:** CAN bus (shared with motor controllers or separate)

**Parameters:**
- Baud rate: 250 kbit/s (lower speed, less critical)
- Protocol: Custom or CANopen

**Messages (BMS → ROS):**

| CAN ID | Data | Frequency |
|--------|------|-----------|
| 0x300 | Cell voltages (13 cells, 2 bytes each) | 1 Hz |
| 0x301 | Total voltage, current, SoC | 1 Hz |
| 0x302 | Temperature (2 sensors), status | 1 Hz |
| 0x303 | Warnings, faults, charging state | Event-driven |

**Example CAN Frame (Total Voltage, Current, SoC):**
```
CAN ID: 0x301
Data[0-1]: Total voltage (uint16, units: 0.1V, range: 0-100V)
Data[2-3]: Current (int16, units: 0.1A, range: ±1000A)
Data[4]:    SoC (uint8, units: 1%, range: 0-100%)
Data[5]:    Charging (uint8, 0=discharging, 1=charging)
```

**Hardware Requirements (Tsuchiya):**
- BMS with CAN interface (e.g., Orion BMS, custom board)
- CAN bus termination (120Ω)
- Voltage monitoring: 0.1% accuracy per cell
- Current sensing: Hall effect sensor, ±100A

**Software Requirements (Pankaj):**
- ROS 2 CAN driver (`ros2_socketcan`)
- BMS interface node decodes CAN frames → publishes `/power/battery_state`

---

## 5. Power Management Interface

### 5.1 Low Battery Behavior

**Trigger:** SoC < 30%

**Software Action (Pankaj):**
1. Publish warning on `/diagnostics`
2. If on mission: Complete current mission, then return to charging station
3. If idle: Navigate to charging station
4. Reject new missions until SoC > 40%

**Hardware Action (Tsuchiya):**
- BMS alerts via CAN bus (warning flag set)
- eHMI displays low battery warning (LED color change)

---

### 5.2 Critical Battery Behavior

**Trigger:** SoC < 10%

**Software Action (Pankaj):**
1. Cancel current mission (safe stop)
2. Activate emergency return to base (shortest path)
3. Reduce max speed to 0.5 m/s (conserve energy)
4. Send critical alert to TVM server

**Hardware Action (Tsuchiya):**
- BMS sets critical flag on CAN bus
- eHMI plays critical battery audio warning

---

### 5.3 Charging Detection

**Trigger:** BMS detects charging (current > 0, voltage rising)

**Software Action (Pankaj):**
1. Transition to CHARGING state
2. Disable navigation (vehicle immobilized during charging)
3. Monitor charging progress (SoC updates)
4. When SoC > 90%: Transition to IDLE, ready for missions

**Hardware Action (Tsuchiya):**
- Charging connector: XT90 or Anderson Powerpole
- BMS enables charging (internal relay)
- Charger: 48V 10A (≥500W), automatic cutoff at full

---

## 6. Safety System Interface

### 6.1 Emergency Stop Behavior

**Trigger:** `/safety/estop` topic = `true`

**Software Action (Pankaj):**
1. **Immediate:** Stop all motors (send zero velocity command)
2. Set state to EMERGENCY_STOP
3. Cancel all missions
4. Activate hazard lights (eHMI red flashing)
5. Play emergency stop audio
6. Send emergency alert to TVM server

**Hardware Action (Tsuchiya):**
- **Hardwired:** E-stop button directly cuts power to motor controllers (relay)
- Latching button (requires manual reset)
- ROS 2 also notified via `/safety/estop` topic (software awareness)

**Recovery:**
1. Operator manually resets E-stop button (twist to unlock)
2. Software detects `/safety/estop` = `false`
3. Operator must send resume command via TVM dashboard or local UI
4. Vehicle transitions to IDLE state

---

### 6.2 Collision Detection (Bumper)

**Trigger:** `/safety/bumper` = `true`

**Software Action (Pankaj):**
1. **Immediate:** Stop all motors (zero velocity)
2. Reverse 0.5 meters (escape collision)
3. Wait 2 seconds
4. If bumper still pressed: Stop, alert operator
5. If bumper released: Attempt to reroute around obstacle

**Hardware Action (Tsuchiya):**
- Physical bumpers with microswitches (normally open)
- Response time: <50ms (mechanical)

---

### 6.3 Tilt Detection (IMU)

**Trigger:** Tilt angle > 15° (roll or pitch)

**Software Action (Pankaj):**
1. **Immediate:** Stop all motors
2. Set state to ERROR (tilt detected)
3. Play warning audio
4. Alert operator via TVM

**Hardware Action:**
- IMU monitors tilt continuously
- Software calculates roll/pitch from IMU data

---

## 7. Coordinate Frames (TF2)

### 7.1 Frame Hierarchy

```
map                          (Global reference frame, fixed)
 └─ odom                     (Odometry frame, drifts over time)
     └─ base_link            (Robot center, at ground level)
         ├─ lidar_link       (LiDAR sensor frame)
         ├─ imu_link         (IMU sensor frame)
         ├─ camera_front_left_link   (Front-left camera)
         ├─ camera_front_right_link  (Front-right camera)
         ├─ front_left_wheel_link    (Swerve module)
         ├─ front_right_wheel_link
         ├─ rear_left_wheel_link
         └─ rear_right_wheel_link
```

### 7.2 Frame Definitions

**`map` → `odom`:**
- Published by: `/localization/ndt_matcher` (NDT localization)
- Frequency: 10 Hz
- Corrects drift in odometry using LiDAR scan matching

**`odom` → `base_link`:**
- Published by: `/swerve/odometry` (wheel encoders)
- Frequency: 50 Hz
- Dead reckoning from wheel encoders

**`base_link` → sensor frames:**
- Published by: `/robot_state_publisher` (static transforms from URDF)
- Frequency: Static (published once at startup)

---

### 7.3 Static Transform Requirements (Tsuchiya + Pankaj)

**Hardware Team (Tsuchiya) SHALL provide:**
- LiDAR mounting position (X, Y, Z offset from base_link)
- Camera mounting positions (X, Y, Z, roll, pitch, yaw)
- IMU mounting position and orientation

**Software Team (Pankaj) SHALL:**
- Create URDF robot description with exact transforms
- Publish static transforms via `robot_state_publisher`
- Calibrate camera extrinsics (camera-to-camera transformation for stereo fusion)

**Example Static Transforms:**
```yaml
# LiDAR mounting (URDF snippet)
<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.2 0 0.7" rpy="0 0 0"/>  # 20cm forward, 70cm up
</joint>

# Front-left camera
<joint name="camera_fl_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_front_left_link"/>
  <origin xyz="0.3 0.15 0.5" rpy="0 0.26 0"/>  # 30cm forward, 15cm left, 50cm up, 15° down tilt
</joint>
```

---

## 8. Testing & Validation

### 8.1 Hardware Interface Tests

**Sensor Tests (Tsuchiya):**
| Test | Procedure | Pass Criteria |
|------|-----------|---------------|
| LiDAR data | `ros2 topic echo /sensors/lidar/points` | 10 Hz, >100k points |
| Camera image | `ros2 topic echo /sensors/camera/front_left/image_raw` | 30 Hz, 1920×1080 |
| IMU data | `ros2 topic echo /sensors/imu` | 50 Hz, no NaN values |
| Encoders | `ros2 topic echo /swerve/front_left/encoder` | 100 Hz, position updates |
| Battery | `ros2 topic echo /power/battery_state` | 1 Hz, valid SoC (0-1.0) |

**Actuator Tests (Tsuchiya):**
| Test | Procedure | Pass Criteria |
|------|-----------|---------------|
| Motor command | Publish to `/cmd_vel`, observe wheels | Wheels rotate, direction correct |
| eHMI state | Publish to `/ehmi/state`, observe LEDs | LED pattern changes |
| eHMI audio | Publish to `/ehmi/audio`, listen | Audio plays correctly |

---

### 8.2 Integration Tests (Pankaj + Tsuchiya)

**Test 1: Sensor-to-Software Pipeline**
1. Hardware publishes sensor data (LiDAR, cameras, IMU)
2. Software receives data, processes, and publishes diagnostics
3. **Pass:** All sensors publishing at correct frequency, no errors in `/diagnostics`

**Test 2: Software-to-Actuator Pipeline**
1. Software publishes `/cmd_vel` (e.g., 0.5 m/s forward)
2. Swerve controller computes wheel commands
3. Motors execute commands
4. Encoders feedback to odometry
5. **Pass:** Robot moves forward at 0.5 m/s (±10%)

**Test 3: Emergency Stop**
1. Press E-stop button
2. Motors cut power immediately (<100ms)
3. Software detects `/safety/estop` = true
4. eHMI shows emergency state
5. **Pass:** Vehicle stops within 0.5 seconds

**Test 4: Low Battery**
1. Simulate low battery (SoC = 25%)
2. Software detects low battery, navigates to charging station
3. **Pass:** Vehicle completes return-to-base mission

---

### 8.3 Acceptance Criteria

**Hardware-Software Interface is ACCEPTED when:**
- ✅ All sensors publishing at correct frequency (LiDAR 10Hz, Camera 30Hz, IMU 50Hz, Encoders 100Hz)
- ✅ All actuators responding to commands (motors, eHMI)
- ✅ Emergency stop working (hardwired + software)
- ✅ Battery monitoring functional (SoC, voltage, current)
- ✅ TF2 transforms published correctly (robot_state_publisher)
- ✅ Integration tests passing (>95% success rate)
- ✅ No data loss or timing violations under normal operation

---

## Document Status

**Status:** Draft - Pending Review
**Next Review:** 2025-12-17 (Joint review with software + hardware teams)
**Sign-Off Required:** Pankaj (Software) + Tsuchiya (Hardware) + Kiril (eHMI)

---

## Appendices

### Appendix A: ROS 2 Package Dependencies

**Software Team (Pankaj) SHALL install:**
```bash
# Sensor drivers
sudo apt install ros-humble-urg-node              # LiDAR (example)
sudo apt install ros-humble-usb-cam               # USB cameras
sudo apt install ros-humble-imu-filter-madgwick   # IMU filtering

# CAN bus
sudo apt install ros-humble-ros2-socketcan
sudo apt install ros-humble-ros2-canopen

# Serial communication
sudo apt install ros-humble-serial-driver

# TF2
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-robot-state-publisher
```

---

### Appendix B: Hardware Checklist (Tsuchiya + Kiril)

**Before Integration Testing:**
- ☐ LiDAR installed, powered, IP configured (if Ethernet)
- ☐ Cameras installed, calibrated (checkerboard calibration)
- ☐ IMU installed, axes aligned with robot frame
- ☐ Swerve drive motors installed, encoders working
- ☐ BMS installed, CAN bus connected
- ☐ E-stop button installed, hardwired to motor controllers
- ☐ Bumper switches installed, tested
- ☐ eHMI (ESP32 + LEDs + audio) installed, serial connected
- ☐ Power distribution complete, fuses installed
- ☐ Cable management complete, strain relief
- ☐ All connectors labeled (sensor names, pin numbers)

---

### Appendix C: Example Launch File

**Example: Launch all hardware drivers**

```python
# hardware_drivers.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # LiDAR
        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='lidar_driver',
            parameters=[{'ip_address': '192.168.1.10'}],
            remappings=[('scan', '/sensors/lidar/scan')]
        ),

        # Cameras
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera_front_left',
            parameters=[{'video_device': '/dev/video0'}],
            remappings=[('image_raw', '/sensors/camera/front_left/image_raw')]
        ),

        # IMU
        Node(
            package='phidgets_imu',
            executable='phidgets_imu_node',
            name='imu_driver',
            remappings=[('imu/data', '/sensors/imu')]
        ),

        # eHMI Serial Bridge
        Node(
            package='ehmi_interface',
            executable='serial_bridge',
            name='ehmi_serial_bridge',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'baud_rate': 115200}]
        ),

        # Robot State Publisher (TF2)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(['cat ', robot_urdf])}]
        ),
    ])
```

---

**END OF DOCUMENT**

**Critical Next Steps:**
1. ⚠️ **Hardware Team (Tsuchiya + Kiril):** Review sensor mounting requirements
2. ⚠️ **Software Team (Pankaj):** Review ROS 2 topic specifications
3. ⚠️ **Week 1 Day 5:** Sign-off by both teams
4. ⚠️ **Week 2:** Begin hardware assembly (Tsuchiya) + software integration (Pankaj)

**Questions/Clarifications:** Contact Pankaj (Software) or Tsuchiya (Hardware)
