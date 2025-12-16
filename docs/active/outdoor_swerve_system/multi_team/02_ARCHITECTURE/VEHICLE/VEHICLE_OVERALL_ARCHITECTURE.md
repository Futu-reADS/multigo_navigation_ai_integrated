# Overall System Architecture - Wheelchair Transport Robot

**Project:** Outdoor-First Wheelchair Transport Robot with Swerve Drive
**Document Type:** System Architecture Document
**Status:** Draft v1.0
**Date:** December 15, 2025
**Reference:** Inspired by ParcelPal proven architecture (0% code reuse, 60% patterns)

---

## 1. Architecture Overview

### 1.1 Design Philosophy

**Outdoor-First, Indoor-Compatible**
- All components selected for outdoor robustness
- Indoor operation as enhanced performance mode
- No GPS dependency (cost + reliability)

**Separation of Concerns**
- Independent subsystems with clear interfaces
- Parallel development capability
- Modular testing and integration

**Inspired by Proven Architecture (ParcelPal)**
- 0% code reuse, 60-70% architectural pattern reuse from ParcelPal
- ParcelPal uses Autoware; this system uses standard ROS 2 + Nav2 + PCL
- Focus new development on swerve drive + docking + NDT integration
- Lower risk through proven approach, faster development via AI assistance

---

## 2. System Architecture Diagram

```
┌────────────────────────────────────────────────────────────────────┐
│                    WHEELCHAIR TRANSPORT ROBOT                      │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │              APPLICATION LAYER                                │ │
│  │                                                              │ │
│  │  ┌─────────────────────────────────────────────────────┐    │ │
│  │  │    Wheelchair Master (Main Orchestration)           │    │ │
│  │  │    • State machine (9 states + wheelchair states)   │    │ │
│  │  │    • Mission planning & queue management            │    │ │
│  │  │    • Safety monitoring & geofencing                 │    │ │
│  │  │    • Battery management & auto-charging             │    │ │
│  │  └─────────────┬───────────────────────────────────────┘    │ │
│  │                │                                              │ │
│  └────────────────┼──────────────────────────────────────────────┘ │
│                   │                                                │
│  ┌────────────────┼──────────────────────────────────────────────┐ │
│  │                │         SUBSYSTEM LAYER                      │ │
│  │    ┌───────────┴────┬──────────┬──────────┬──────────────┐   │ │
│  │    ▼                ▼          ▼          ▼              ▼   │ │
│  │ ┌────────┐  ┌──────────┐  ┌────────┐  ┌────────┐  ┌───────┐ │ │
│  │ │   UI   │  │   Nav    │  │ Swerve │  │Docking │  │ eHMI  │ │ │
│  │ │ (React)│  │  Stack   │  │ Drive  │  │ (ArUco)│  │(ESP32)│ │ │
│  │ │        │  │ (ROS 2)  │  │ Control│  │ Visual │  │LED+   │ │ │
│  │ │Touch   │  │          │  │        │  │Servoing│  │Audio  │ │ │
│  │ │Screen  │  │Nav2+NDT  │  │  IK/FK │  │  PBVS  │  │Serial │ │ │
│  │ └────────┘  └────┬─────┘  └───┬────┘  └───┬────┘  └───────┘ │ │
│  └──────────────────┼────────────┼───────────┼──────────────────┘ │
│                     │            │           │                    │
│  ┌──────────────────┼────────────┼───────────┼──────────────────┐ │
│  │                  │   MIDDLEWARE LAYER (ROS 2 Humble)          │ │
│  │                  │                                            │ │
│  │    Topics ───────┼────────────┼───────────┼───────────────┐  │ │
│  │    Services ─────┼────────────┼───────────┼───────────────┤  │ │
│  │    Actions ──────┼────────────┼───────────┼───────────────┤  │ │
│  │    TF2 ──────────┼────────────┼───────────┼───────────────┘  │ │
│  └──────────────────┼────────────┼───────────┼──────────────────┘ │
│                     │            │           │                    │
│  ┌──────────────────┼────────────┼───────────┼──────────────────┐ │
│  │         PERCEPTION & ACTUATION LAYER                          │ │
│  │                  │            │           │                    │ │
│  │    ┌─────────────▼────────────▼───────────▼────────┐          │ │
│  │    │  SENSORS                        ACTUATORS      │          │ │
│  │    │  • 3D LiDAR (360°)              • 4× Swerve   │          │ │
│  │    │  • 2× RGB Cameras                  Modules    │          │ │
│  │    │  • 9-DOF IMU                     • Coupling   │          │ │
│  │    │  • Wheel Encoders                  Mechanism  │          │ │
│  │    │  • Occupant Sensor               • E-Stop     │          │ │
│  │    │  • ArUco Markers (passive)       • Lights     │          │ │
│  │    └───────────────────────────────────────────────┘          │ │
│  └──────────────────────────────────────────────────────────────┘ │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │              COMPUTE PLATFORM                                 │ │
│  │  GMKtec Nucbox K6 (Ryzen 7 7840HS, 32GB, Radeon 780M)       │ │
│  │  Ubuntu 22.04 LTS, ROS 2 Humble                             │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘
```

---

## 3. Subsystem Descriptions

### 3.1 Wheelchair Master (Main Orchestration)

**Technology:** Python 3.10 (Poetry), ROS 2 integration
**Reference:** Adapted from ParcelPal `ftd_master` pattern

**Responsibilities:**
- State machine coordination (Idle → Approaching → Docking → Attached → Transporting → Arrived → Undocking → Detached)
- Mission planning (single transport, multi-stop shuttle, scheduled routes)
- Safety monitoring (passenger detection, tilt angle, geofencing)
- Battery management (auto-return to charge <30%)
- Error recovery and failsafe behaviors

**State Machine:**
```
IDLE ──call──> APPROACHING_WHEELCHAIR ──marker_detected──> SEARCHING_ARUCO
  ▲                                                              │
  │                                                              ▼
  │                                                          DOCKING
  │                                                              │
  │                                                              ▼
WHEELCHAIR_DETACHED <──undock──< UNDOCKING <──arrive──< TRANSPORTING
                                      ▲                          │
                                      │                          ▼
                                 ARRIVED_DESTINATION <──attached─┘
                                                         (via WHEELCHAIR_ATTACHED)
```

**Key Interfaces:**
- Publishes: `/mission/goal`, `/safety/status`, `/battery/level`
- Subscribes: `/docking/status`, `/nav/feedback`, `/occupant/detected`
- Services: `/mission/start`, `/mission/cancel`, `/emergency_stop`

---

### 3.2 Autoware Navigation Stack (REUSED - ParcelPal)

**Components:**
- **Localization:** NDT scan matcher (tier4_localization)
- **Planning:** Nav2 planner plugins (global: Hybrid A*, local: DWA/TEB)
- **Perception:** LiDAR obstacle detection (PCL-based)
- **Control:** Nav2 controller server

**Approach (ParcelPal Proven):**
1. **Offline:** Manual SLAM mapping once with loop closure
2. **Online:** NDT scan matcher on saved map (NO continuous SLAM)
3. **Result:** Drift-free localization, 500m-1km reliable range

**Configuration:**
```yaml
# tier4_localization_component.launch.xml
pose_source: ndt
ndt_scan_matcher:
  convergence_threshold: 0.01
  step_size: 0.1
  resolution: 1.0
```

**Key Interfaces:**
- Publishes: `/localization/pose`, `/planning/path`, `/obstacles`
- Subscribes: `/points_raw` (LiDAR), `/odometry/swerve`
- Actions: `/navigate_to_pose`

---

### 3.3 Swerve Drive Controller (NEW)

**Technology:** C++17, ROS 2 controller plugin
**Reference:** Custom development (no ParcelPal equivalent - they use differential drive)

**Responsibilities:**
- Inverse kinematics (cmd_vel → 4 wheel states)
- Forward kinematics (wheel encoders → odometry)
- Nav2 controller plugin integration
- Motor command generation (CAN bus)

**Kinematics:**
```
Input: cmd_vel (vx, vy, ω)
Output: 4× (wheel_speed, wheel_angle)

For each wheel i:
  wheel_velocity[i] = f(vx, vy, ω, wheel_position[i])
  wheel_angle[i] = atan2(vy_wheel[i], vx_wheel[i])
```

**Configuration:**
```yaml
swerve_drive_controller:
  wheel_base: 0.6  # meters (front-rear distance)
  track_width: 0.5  # meters (left-right distance)
  wheel_radius: 0.0825  # meters (6.5" = 165mm)
  max_linear_velocity: 1.5  # m/s
  max_angular_velocity: 1.0  # rad/s
```

**Key Interfaces:**
- Subscribes: `/cmd_vel` (Twist)
- Publishes: `/odometry/swerve` (Odometry), `/joint_states`
- Hardware: CAN bus to motor controllers

---

### 3.4 Wheelchair Docking Subsystem (NEW)

**Technology:** C++17 + OpenCV, ROS 2 node
**Reference:** Custom development (ParcelPal has simple delivery zone approach)

**Phases:**
1. **Coarse Navigation:** Autoware nav to docking approach zone (±1m)
2. **Marker Detection:** ArUco marker detection (OpenCV, 0.5-5m range)
3. **Visual Servoing:** IBVS (Image-Based Visual Servoing) for precision (±2-5mm)
4. **Mechanical Coupling:** Engage coupling, verify attachment

**ArUco Configuration:**
```yaml
aruco_detector:
  dictionary: DICT_4X4_50
  marker_size: 0.15  # meters (150mm × 150mm)
  camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
  dist_coeffs: [k1, k2, p1, p2, k3]
  detection_rate: 10  # Hz
```

**Visual Servoing Control:**
```
Error: e = (marker_pose_desired - marker_pose_current)
Control: u = Kp × e + Kd × de/dt
Output: cmd_vel (fine adjustments, max 0.1 m/s)
```

**Key Interfaces:**
- Subscribes: `/camera/image_raw`, `/camera/camera_info`
- Publishes: `/docking/status`, `/docking/marker_pose`, `/cmd_vel` (override)
- Services: `/docking/start`, `/docking/cancel`
- Actions: `/dock_wheelchair`

---

### 3.5 User Interface (ADAPTED - ParcelPal)

**Technology:** React 18 + Next.js 14, TypeScript
**Reference:** ParcelPal `ftd_parcelpal_ui` pattern (70% reuse)

**Screens:**
- `/` - Home/Idle screen
- `/booking` - Wheelchair pickup/dropoff location selection
- `/enroute` - Active mission status, ETA, real-time position
- `/settings` - Language, volume, system diagnostics

**Backend Communication:**
- WebSocket to `wheelchair_master` for real-time updates
- REST API for mission submission, history query

**Key Features:**
- Multi-language (EN, JP, ZH) with audio announcements
- Touch-optimized (outdoor glove-friendly)
- Battery and charging status display
- Emergency stop button (large, red)

---

### 3.6 eHMI (External HMI) (EXTENDED - ParcelPal)

**Technology:** ESP32-S3, Arduino, Dezyne (formal modeling)
**Reference:** ParcelPal `ehmi` pattern (90% reuse + wheelchair states)

**Components:**
- **LED Strips:** FastLED (WS2812B), dynamic animations
- **LED Matrix:** HUB75 96×48 panel, icons + text + GIF
- **Audio:** I2S DAC, MP3 playback from SD card

**New States for Wheelchair:**
```cpp
21 = APPROACHING_WHEELCHAIR    // Blue slow pulse
22 = SEARCHING_ARUCO          // Cyan scanning
23 = DOCKING                  // Green moving pattern
24 = WHEELCHAIR_ATTACHED      // Green solid
25 = TRANSPORTING             // Blue breathing (quiet audio)
26 = ARRIVED_DESTINATION      // Green chase
27 = UNDOCKING                // Green reverse
28 = WHEELCHAIR_DETACHED      // Green pulse → Idle
```

**Communication:**
- Serial (115200 baud): `STATE:{num}\n`, `VOLUME:{0-10}\n`, `LANGUAGE:{EN|JP|ZH}\n`
- ROS 2 node (`ehmi_state_publisher`) bridges ROS ↔ ESP32

**Detailed Reference:** See `../reference/EHMI_SYSTEM_REFERENCE.md`

---

## 4. Data Flow

### 4.1 Normal Operation Data Flow

```
┌─────────┐     ┌──────────┐     ┌─────────┐     ┌────────┐
│ LiDAR   │────>│ Autoware │────>│Wheelchair│────>│ Swerve │
│         │     │   Nav    │     │  Master  │     │ Drive  │
│ Cameras │────>│ (NDT +   │     │ (State   │     │Control │
│         │     │  Nav2)   │     │ Machine) │     │        │
│ IMU     │────>│          │     │          │     │        │
└─────────┘     └────┬─────┘     └────┬─────┘     └────┬───┘
                     │                 │                 │
                     ▼                 ▼                 ▼
                ┌─────────┐       ┌────────┐       ┌────────┐
                │Obstacle │       │ eHMI   │       │ Motors │
                │Detection│       │        │       │ (CAN)  │
                └─────────┘       └────────┘       └────────┘
```

### 4.2 Docking Sequence Data Flow

```
1. Wheelchair Master ──goal──> Autoware Nav ──navigate──> Approach Zone

2. Docking Subsystem ──marker_detected──> ArUco Detector (OpenCV)
                                              │
3. Visual Servoing <─────pose_error──────────┘
        │
4. cmd_vel (fine) ──> Swerve Drive ──> Precision Docking (±2-5mm)

5. Coupling Mechanism ──attached_confirmed──> Wheelchair Master ──state──> WHEELCHAIR_ATTACHED
```

---

## 5. ROS 2 Topic Architecture

### 5.1 Core Topics

| Topic | Type | Publisher | Subscriber(s) | Rate |
|-------|------|-----------|---------------|------|
| `/points_raw` | PointCloud2 | LiDAR driver | Autoware localization | 10 Hz |
| `/camera/image_raw` | Image | Camera driver | Docking subsystem | 15 Hz |
| `/odometry/swerve` | Odometry | Swerve controller | Autoware, Master | 50 Hz |
| `/cmd_vel` | Twist | Nav2 / Docking | Swerve controller | 20 Hz |
| `/localization/pose` | PoseStamped | NDT scan matcher | Nav2, Master | 10 Hz |
| `/mission/goal` | PoseStamped | Wheelchair Master | Nav2 planner | On demand |
| `/docking/status` | String | Docking subsystem | Wheelchair Master | 1 Hz |
| `/ehmi/status` | Int32 | Wheelchair Master | eHMI publisher | On change |
| `/occupant/detected` | Bool | Occupant sensor | Wheelchair Master | 10 Hz |
| `/emergency_stop` | Bool | E-stop button | All nodes | On change |

### 5.2 TF2 Frame Tree

```
map
 └─> odom
      └─> base_link
           ├─> lidar_link
           ├─> camera_front_link
           ├─> camera_rear_link
           ├─> imu_link
           ├─> wheel_fl_link
           ├─> wheel_fr_link
           ├─> wheel_rl_link
           └─> wheel_rr_link
```

**Key Transforms:**
- `map → odom`: NDT scan matcher (corrects drift)
- `odom → base_link`: Swerve drive odometry
- `base_link → sensor_links`: Static (URDF)

---

## 6. Safety Architecture

### 6.1 Safety Layers

```
┌────────────────────────────────────────────────────────┐
│             LAYER 4: OPERATIONAL SAFETY                │
│  • Geofencing (keep-out zones)                         │
│  • Speed limits (passenger onboard: 1.0 m/s)           │
│  • No pickup/dropoff on slopes >5°                     │
└───────────────────┬────────────────────────────────────┘
                    │
┌───────────────────▼────────────────────────────────────┐
│             LAYER 3: COLLISION AVOIDANCE               │
│  • LiDAR obstacle detection (0.1-30m)                  │
│  • Dynamic obstacle tracking                           │
│  • Social distancing (1.5m from pedestrians)           │
│  • Emergency braking (stop within 1.5m)                │
└───────────────────┬────────────────────────────────────┘
                    │
┌───────────────────▼────────────────────────────────────┐
│             LAYER 2: SYSTEM MONITORING                 │
│  • Sensor health checks (10 Hz)                        │
│  • Battery monitoring (critical <15%)                  │
│  • Tilt angle monitoring (stop if >15°)                │
│  • Occupant presence (transport only)                  │
│  • Failsafe: sensor loss → safe stop                   │
└───────────────────┬────────────────────────────────────┘
                    │
┌───────────────────▼────────────────────────────────────┐
│             LAYER 1: EMERGENCY STOP (SIL 2)            │
│  • Hardware E-stop button (accessible all sides)       │
│  • Software E-stop (ROS topic, 100ms response)         │
│  • Immediate motor cutoff                              │
│  • Redundant braking system                            │
└────────────────────────────────────────────────────────┘
```

### 6.2 Safety State Machine

```
NORMAL ──obstacle_detected──> CAUTION ──obstacle_close──> EMERGENCY_BRAKE
  │                              │                            │
  │                              └───obstacle_cleared────────┘
  │                                         │
  └──e_stop_pressed──────────────> E_STOP_ACTIVE
                                            │
                                    e_stop_released + reset
                                            │
                                            ▼
                                        NORMAL
```

---

## 7. Performance Budget

### 7.1 Compute Resources (GMKtec Nucbox K6)

| Component | CPU Target | Memory Target | Notes |
|-----------|-----------|---------------|-------|
| Autoware Nav Stack | 30-40% | 8 GB | LiDAR processing, planning |
| Swerve Drive Controller | 5-10% | 512 MB | Kinematics, motor control |
| Docking Subsystem | 10-15% | 1 GB | OpenCV, visual servoing |
| Wheelchair Master | 5-10% | 512 MB | State machine, mission planning |
| UI + eHMI | 5% | 1 GB | React frontend, Node.js |
| ROS 2 Middleware | 5-10% | 2 GB | DDS, TF2, logging |
| **Total** | **60-70%** | **~13 GB** | Leaves 30% margin |

### 7.2 Network Bandwidth (ROS 2 DDS)

| Topic | Size | Rate | Bandwidth |
|-------|------|------|-----------|
| PointCloud2 (LiDAR) | ~500 KB | 10 Hz | 5 MB/s |
| Image (2× cameras) | ~1 MB | 15 Hz | 30 MB/s |
| Odometry | 200 B | 50 Hz | 10 KB/s |
| Others | - | - | ~5 MB/s |
| **Total** | | | **~40 MB/s** |

**Mitigation:** Use DDS QoS settings (RELIABLE for critical, BEST_EFFORT for sensors)

---

## 8. Deployment Architecture

### 8.1 Directory Structure

```
/opt/wheelchair_robot/
├── install/               # ROS 2 workspace install
│   ├── wheelchair_master/
│   ├── swerve_drive_controller/
│   ├── docking_subsystem/
│   └── autoware_*         # Autoware packages
├── config/                # Configuration files
│   ├── params/            # ROS 2 parameter YAML files
│   ├── maps/              # Saved SLAM maps
│   └── calibration/       # Sensor calibration
├── logs/                  # ROS 2 logs, mission logs
├── ui/                    # React UI build
└── scripts/               # Launch scripts, maintenance
```

### 8.2 Launch Strategy

**Main Launch File:** `/opt/wheelchair_robot/launch/wheelchair_robot.launch.py`

```python
# Pseudo-code launch file structure
LaunchDescription([
    # 1. Hardware drivers
    Node(package='lidar_driver', ...),
    Node(package='camera_driver', ...),
    Node(package='imu_driver', ...),
    
    # 2. Autoware stack
    IncludeLaunchDescription('autoware.launch.xml',
        launch_arguments={'map_path': map_path, ...}),
    
    # 3. Custom subsystems
    Node(package='swerve_drive_controller', ...),
    Node(package='docking_subsystem', ...),
    Node(package='wheelchair_master', ...),
    
    # 4. UI and monitoring
    Node(package='wheelchair_ui', ...),
    Node(package='ehmi_state_publisher', ...),
])
```

---

## 9. Integration Points with ParcelPal

### 9.1 Reused Components (No/Minimal Changes)

| Component | Reuse % | Source |
|-----------|---------|--------|
| NDT Localization | 100% | `tier4_localization_component.launch.xml` |
| LiDAR Obstacle Detection | 100% | Autoware perception |
| Global Path Planning | 100% | Nav2 planner server |
| Battery Management Logic | 90% | `ftd_master` pattern |
| eHMI Framework | 90% | `ehmi` package + new states |

### 9.2 Adapted Components

| Component | Adaptation | Reason |
|-----------|-----------|--------|
| Main Orchestration | 80% reuse + new docking states | Wheelchair-specific workflow |
| UI Screens | 70% reuse + wheelchair booking | Different user interaction |
| Local Motion Planning | 80% reuse + swerve constraints | Swerve drive vs. differential |

### 9.3 New Components

| Component | Development Effort | Rationale |
|-----------|-------------------|-----------|
| Swerve Drive Controller | High (from scratch) | ParcelPal uses differential drive |
| Docking Subsystem | High (from scratch) | ParcelPal has simple delivery zone |
| Passenger Safety Monitoring | Medium | Wheelchair-specific safety |

---

## 10. Open Architecture Questions

### 10.1 Decisions Needed

| Question | Options | Recommendation | Owner |
|----------|---------|----------------|-------|
| 3D LiDAR model? | Ouster OS1-64, Velodyne VLP-16, Livox Mid-70 | TBD (Sprint 2) | Hardware team |
| Camera resolution? | 720p, 1080p, 4K | 1080p (ArUco + sufficient) | Perception team |
| Local planner plugin? | DWA, TEB, MPC | TEB (smooth, swerve-friendly) | Nav team |
| Coupling mechanism? | Magnetic, mechanical latch, hook | TBD (design review) | Mech team |
| Battery capacity? | 20Ah, 30Ah, 40Ah | TBD (power budget) | Hardware team |

### 10.2 Future Enhancements (Post-MVP)

- Multi-robot fleet coordination (Phase 2)
- Weather sensor integration (rain/wind detection)
- Advanced HMI (gesture recognition, voice control)
- Deep learning perception (object classification) - requires discrete GPU
- V2X communication (vehicle-to-everything)

---

## 11. Related Documents

**Reference:**
- `../reference/PARCELPAL_EXPLORATION_SUMMARY.md` - ParcelPal detailed analysis
- `../reference/EHMI_SYSTEM_REFERENCE.md` - eHMI implementation guide

**Requirements:**
- `../01_REQUIREMENTS/SYSTEM_REQUIREMENTS.md` - 92 system requirements
- `../01_REQUIREMENTS/SWERVE_DRIVE_REQUIREMENTS.md` - Drive system specs
- `../01_REQUIREMENTS/DOCKING_SYSTEM_REQUIREMENTS.md` - Docking precision

**Detailed Design:**
- `../03_DESIGN/SWERVE_DRIVE_DETAILED_DESIGN.md` - Kinematics, control algorithms
- `../03_DESIGN/DOCKING_MECHANISM_DESIGN.md` - Visual servoing, coupling

**Interfaces:**
- `../04_INTERFACES/ROS_TOPICS_AND_SERVICES.md` - Complete topic/service list
- `../04_INTERFACES/HARDWARE_INTERFACES.md` - CAN, serial, I2C specifications

---

**Document Status:** Draft v1.0
**Review Status:** Pending architecture review board
**Next Review:** After Phase 1 Sprint 3
**Approvers:** System Architect, Lead Engineers (Nav, Perception, Safety)
**Version History:**
- v1.0 (2025-12-15): Initial architecture document
