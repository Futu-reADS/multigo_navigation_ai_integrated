# Multi Go Navigation System - Quick Reference

**Branch:** feature/localization | **Analyst:** Claude AI | **Date:** Nov 25, 2025

---

## System Overview

**Multi Go** is an autonomous mobile robot system for wheelchair and cart mobility using mecanum wheels, visual localization, and precision docking capabilities.

**Key Features:**
- 🗺️ **SLAM & Localization** - RTAB-Map visual SLAM
- 🧭 **Autonomous Navigation** - NAV2 stack with dynamic obstacle avoidance
- 🎯 **Precision Docking** - ArUco marker-based visual servoing (±1mm accuracy)
- 🔄 **Omnidirectional Motion** - Mecanum wheels for holonomic control
- 📹 **Multi-Camera Perception** - Dual cameras for marker detection
- 🛡️ **Safety** - LiDAR-based obstacle detection

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    MULTI GO NAVIGATION SYSTEM                   │
└─────────────────────────────────────────────────────────────────┘

┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│  PERCEPTION      │  │  LOCALIZATION    │  │  PLANNING        │
│                  │  │                  │  │                  │
│ • Cameras (2x)   │─→│ • RTAB-Map SLAM  │─→│ • NAV2 Global    │
│ • LiDAR          │  │ • Visual odom    │  │ • NAV2 Local     │
│ • ArUco detect   │  │ • TF transforms  │  │ • Costmaps       │
└──────────────────┘  └──────────────────┘  └────────┬─────────┘
                                                      ↓
┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│  DOCKING         │  │  MOTION CONTROL  │  │  EXECUTION       │
│                  │  │                  │  │                  │
│ • nav_goal       │─→│ • nav_control    │─→│ • mecanum_wheels │
│ • nav_docking    │  │ • Mode switching │  │ • Motor PID      │
│ • Visual servo   │  │ • Kinematics     │  │ • Phidget driver │
└──────────────────┘  └──────────────────┘  └────────┬─────────┘
                                                      ↓
                                              ┌──────────────────┐
                                              │   HARDWARE       │
                                              │ • 4x BLDC motors │
                                              │ • Mecanum wheels │
                                              └──────────────────┘
```

---

## Core Components

### Perception Layer

| Component | Function | Technology | Output |
|-----------|----------|------------|--------|
| **Cameras** | Visual input | 2x RGB 1280x720 | Image streams |
| **aruco_detect** | Marker detection | OpenCV ArUco | Marker poses |
| **LiDAR** | Obstacle detection | Hesai LiDAR | Point clouds |
| **ego_pcl_filter** | Self-filtering | PCL | Filtered clouds |
| **pcl_merge** | Sensor fusion | PCL | Merged cloud |
| **laserscan_to_pcl** | Format conversion | ROS2 | Point cloud |

### Localization & Mapping

| Component | Function | Technology | Output |
|-----------|----------|------------|--------|
| **RTAB-Map** | Visual SLAM | RTABMap ROS2 | Map, poses |
| **TF2** | Transform tree | ROS2 TF2 | Coordinate frames |
| **Odometry** | Dead reckoning | Wheel encoders | Odom estimates |

### Navigation & Planning

| Component | Function | Technology | Output |
|-----------|----------|------------|--------|
| **NAV2** | Path planning | Nav2 stack | Planned paths |
| **Global Planner** | High-level planning | A*, etc. | Global path |
| **Local Planner** | Reactive control | DWB/TEB | Local velocities |
| **Costmaps** | Obstacle representation | Nav2 | Occupancy grids |

### Docking System

| Component | Function | Technology | Output |
|-----------|----------|------------|--------|
| **nav_goal** | Approach planning | Custom C++ | Navigation goals |
| **nav_docking** | Visual servoing | Custom C++ | Velocity commands |
| **nav_control** | Kinematic transform | Custom C++ | Adjusted velocities |

### Motion Execution

| Component | Function | Technology | Output |
|-----------|----------|------------|--------|
| **mecanum_wheels** | Motor control | Python + Phidget22 | Motor commands |
| **Inverse Kinematics** | Wheel speed calc | Mecanum equations | Wheel velocities |
| **Motor PID** | Closed-loop control | PID controller | PWM signals |

---

## Operating Modes

### 1. SOLO Mode
- **Description:** Mobile base alone
- **Rotation Center:** 0.0m (center pivot)
- **Use Case:** General navigation

### 2. DOCKING Mode
- **Description:** Precision docking operations
- **Rotation Center:** 0.25m (forward pivot)
- **Use Case:** Docking to charging station

### 3. COMBINE_CHAIR Mode
- **Description:** Wheelchair attached
- **Rotation Center:** 0.5m (far forward pivot)
- **Use Case:** Transporting wheelchair/patient

---

## Data Flow - High Level

```
[Environment]
     ↓
[Sensors: Cameras, LiDAR]
     ↓
[Perception: ArUco, PCL Processing]
     ↓
[Localization: RTAB-Map, TF2]
     ↓
[Planning: NAV2 or Docking Controller]
     ↓
[Motion Control: nav_control]
     ↓
[Execution: mecanum_wheels]
     ↓
[Hardware: Motors]
     ↓
[Robot Motion]
```

---

## Key Topics

| Topic | Type | Publisher | Subscriber | Purpose |
|-------|------|-----------|------------|---------|
| `/camera/*/image_raw` | Image | camera_node | aruco_detect | Visual input |
| `/scan` | LaserScan | lidar_node | nav2, pcl | Obstacle detection |
| `/aruco_detect/markers_*` | PoseArray | aruco_detect | nav_goal, nav_docking | Marker poses |
| `/goal_pose` | PoseStamped | nav_goal | NAV2 | Navigation goals |
| `/cmd_vel_final` | Twist | nav_docking | nav_control | Docking commands |
| `/cmd_vel` | Twist | NAV2/nav_control | mecanum_wheels | Final velocities |
| `/map` | OccupancyGrid | rtabmap | nav2 | SLAM map |
| `/tf` | TFMessage | multiple | all | Transforms |

---

## Action Servers

| Action | Server Node | Type | Purpose |
|--------|-------------|------|---------|
| `/approach` | nav_goal | Approach | Navigate to docking zone |
| `/dock` | nav_docking | Dock | Precision docking |
| `/navigate_to_pose` | NAV2 | NavigateToPose | General navigation |
| `/compute_path_to_pose` | NAV2 | ComputePathToPose | Path planning |

---

## Coordinate Frames

```
map (world)
 │
 ├─ odom (odometry)
 │   │
 │   └─ base_link (robot center)
 │       │
 │       ├─ camera_left_frame
 │       │   └─ aruco_marker_20 (detected)
 │       │
 │       ├─ camera_right_frame
 │       │   └─ aruco_marker_21 (detected)
 │       │
 │       ├─ lidar_frame
 │       │
 │       └─ [wheel frames]
```

---

## Launch Commands

### Simulation
```bash
# Terminal 1: Launch simulation environment
ros2 launch boot simulation.launch.py

# Terminal 2: Launch navigation stack
ros2 launch boot run.launch.py
```

### Real Robot
```bash
# Terminal 1: Launch hardware drivers
ros2 launch boot boot.launch.py

# Terminal 2: Launch navigation stack
ros2 launch boot run.launch.py
```

### Remote Visualization
```bash
ssh -X user@robot_ip
source install/setup.bash
ros2 launch boot rviz_launch.py
```

---

## Configuration Files

| File | Location | Purpose |
|------|----------|---------|
| **PID Parameters** | `src/nav_docking/config/docking_pid_params.yaml` | Docking control gains |
| **Camera Calibration** | `src/camera_publisher/config/calib.yaml` | Camera intrinsics |
| **NAV2 Params** | `boot` package (external) | Navigation tuning |
| **RTAB-Map Config** | `boot` package (external) | SLAM parameters |

---

## Robot Specifications

### Physical
- **Dimensions:** ~0.5m x 0.5m (base)
- **Wheelbase:** 0.40m (width) x 0.30m (length)
- **Wheel Diameter:** 0.0762m (3 inches)
- **Drive:** 4-wheel mecanum (omnidirectional)

### Performance
- **Max Speed:** 0.1 m/s (during docking), higher during navigation
- **Rotation:** In-place rotation capable
- **Docking Accuracy:** ±1mm (target)
- **Payload:** Unknown (see questions-and-gaps.md)

### Sensors
- **Cameras:** 2x RGB (1280x720)
- **LiDAR:** Hesai (model unknown)
- **Encoders:** 4x motor encoders
- **IMU:** Unknown (likely present)

### Compute
- **Platform:** Ubuntu 22.04
- **ROS:** ROS2 Humble
- **Hardware:** Unknown (likely Intel NUC or similar)

---

## Software Stack

### Core Dependencies
```
ROS2 Humble
├─ NAV2 (navigation)
├─ RTAB-Map (SLAM)
├─ OpenCV 4.x (vision)
├─ PCL (point clouds)
├─ TF2 (transforms)
└─ Phidget22 (motors)
```

### Custom Packages
```
multigo_navigation/
├─ aruco_detect          # Marker detection
├─ nav_goal              # Approach control
├─ nav_docking           # Docking control
├─ nav_control           # Kinematic transform
├─ mecanum_wheels        # Motor control
├─ ego_pcl_filter        # Cloud filtering
├─ laserscan_to_pcl      # Format conversion
├─ pcl_merge             # Cloud merging
└─ camera_publisher      # Camera interface
```

### External Packages (Imported)
```
multigo_master           # Action interfaces
multigo_launch (boot)    # Launch files
mecanum_drive            # Drive kinematics
HesaiLidar_ROS_2.0      # LiDAR driver
```

---

## Development Workflow

### Build
```bash
cd multigo_navigation
colcon build --symlink-install --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
source install/setup.bash
```

### Test
```bash
colcon test
colcon test-result --verbose
```

### Clean Build
```bash
rm -rf build install log
colcon build --symlink-install
```

---

## Common Operations

### Send Docking Command
```bash
# Approach
ros2 action send_goal /approach nav_interface/action/Approach "{approach_request: true}"

# Dock
ros2 action send_goal /dock nav_interface/action/Dock "{dock_request: true}"
```

### Send Navigation Goal
```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 1.0, y: 2.0, z: 0.0},
         orientation: {w: 1.0}}
}"
```

### Monitor Topics
```bash
# List topics
ros2 topic list

# Echo marker detections
ros2 topic echo /aruco_detect/markers_left

# Monitor velocity commands
ros2 topic echo /cmd_vel
```

### Check Transforms
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo base_link aruco_marker_20
```

### Node Information
```bash
# List nodes
ros2 node list

# Node info
ros2 node info /nav_docking_node
```

---

## Troubleshooting

### Issue: Markers Not Detected
**Check:**
- Camera topics publishing: `ros2 topic hz /camera/color/image_raw_left`
- ArUco node running: `ros2 node list | grep aruco`
- Lighting conditions
- Marker visibility and quality

### Issue: Docking Fails
**Check:**
- Both markers visible: `ros2 topic echo /aruco_detect/markers_left`
- Transforms available: `ros2 run tf2_ros tf2_echo base_link aruco_marker_20`
- PID parameters loaded: Check logs
- Action server active: `ros2 action list`

### Issue: Robot Not Moving
**Check:**
- Motor node running: `ros2 node list | grep mecanum`
- Velocity commands published: `ros2 topic echo /cmd_vel`
- Phidget connection: Check USB devices
- Emergency stop status

### Issue: Navigation Fails
**Check:**
- Map available: `ros2 topic echo /map --once`
- Localization working: Check /tf for odom→base_link
- Costmap clear: Check in RViz
- Goal reachable: Check planned path

---

## Performance Monitoring

```bash
# CPU usage
top

# ROS2 node performance
ros2 topic hz <topic_name>
ros2 topic bw <topic_name>

# Transform delays
ros2 run tf2_ros tf2_monitor

# Bag recording
ros2 bag record -a
```

---

## Safety Features

| Feature | Implementation | Status |
|---------|----------------|--------|
| **Emergency Stop** | Unknown | ❓ |
| **Velocity Limiting** | nav_control, mecanum_wheels | ✅ |
| **Obstacle Avoidance** | NAV2 costmaps | ✅ (nav only) |
| **Marker Timeout** | nav_docking | ✅ |
| **Action Cancellation** | All action servers | ✅ |
| **Collision Detection (Docking)** | None | ❌ |

---

## System Status

### Working ✅
- Basic navigation with NAV2
- SLAM with RTAB-Map
- Marker detection (ArUco)
- Motor control (mecanum wheels)
- Docking approach (Stage 3)
- Visual servoing (Stage 4 & 5)

### Issues 🐛
- PID integral calculation bug
- Dual marker distance formula bug
- Thread safety (race conditions)
- Parameter assignment bugs

### Missing ❌
- Undocking capability
- Comprehensive error recovery
- Automated testing
- Diagnostics system
- Safety scanner integration during docking
- Documentation (specs, calibration, testing)

---

## Quick Diagnostic Commands

```bash
# System health check
ros2 node list                    # All nodes running?
ros2 topic list                   # Topics available?
ros2 action list                  # Actions available?

# Sensor check
ros2 topic hz /camera/color/image_raw_left
ros2 topic hz /scan
ros2 topic echo /aruco_detect/markers_left --once

# Transform check
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_link

# Motion check
ros2 topic echo /cmd_vel
ros2 topic echo /real_vel

# Logs
ros2 node info /nav_docking_node
colcon test --packages-select nav_docking
```

---

## File Structure

```
multigo_navigation/
├── src/
│   ├── aruco_detect/
│   ├── camera_publisher/
│   ├── ego_pcl_filter/
│   ├── laserscan_to_pcl/
│   ├── mecanum_wheels/
│   ├── nav_control/
│   ├── nav_docking/        ⭐ Main docking controller
│   ├── nav_goal/
│   ├── pcl_merge/
│   └── third_party/
│       ├── perception_pcl/
│       ├── rtabmap/
│       └── rtabmap_ros/
├── docs/                    📚 This analysis
│   ├── docking-system-analysis/
│   └── overall-system-analysis/
├── multigo.repos           🔗 External dependencies
└── README.md
```

---

## Key Contacts

- **Developer:** Thomas Vines (thomas.vines.gc@futu-re.co.jp)
- **Organization:** Futu-reADS
- **Field Test Location:** China

---

## Next Steps

### Immediate (Critical Bugs)
1. Fix PID integral calculation
2. Fix dual marker distance formula
3. Add thread safety (mutexes)
4. Fix parameter assignment bugs

### Short Term (Safety & Robustness)
1. Add action timeouts
2. Implement error recovery
3. Add diagnostics
4. Create test suite

### Medium Term (Features)
1. Implement undocking
2. Add dynamic reconfiguration
3. Improve documentation
4. Field validation

---

*For detailed analysis, see respective documents in `/docs/` folders*

**Key Documents:**
- Docking: `docking-system-analysis/requirements-docking.md`
- Code Quality: `docking-system-analysis/code-analysis.md`
- Questions: `docking-system-analysis/questions-and-gaps.md`
- Architecture: `docking-system-analysis/architecture-overview.md`
- Testing: `docking-system-analysis/test-coverage-analysis.md`
