# MultiGo System - Developer Guide & Architecture

**Document Version:** 1.0
**Last Updated:** 2025-12-01
**Audience:** Developers, system architects, integration engineers
**Reading Time:** 30 minutes

---

## Table of Contents

1. [System Architecture Overview](#system-architecture-overview)
2. [Component Deep Dive](#component-deep-dive)
3. [Data Flow & Communication](#data-flow--communication)
4. [Key Algorithms](#key-algorithms)
5. [ROS 2 Integration](#ros-2-integration)
6. [Hardware Interfaces](#hardware-interfaces)
7. [Development Environment](#development-environment)
8. [Code Structure](#code-structure)

---

## System Architecture Overview

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    MultiGo Navigation System                │
│                     (ROS 2 Humble on Ubuntu 22.04)          │
└─────────────────────────────────────────────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
   ┌────▼────┐          ┌────▼────┐          ┌────▼────┐
   │ Master  │          │  Nav &  │          │Hardware │
   │ Control │          │ Docking │          │ Drivers │
   │multigo_ │          │multigo_ │          │multigo_ │
   │ master  │          │navigation│         │ launch  │
   └────┬────┘          └────┬────┘          └────┬────┘
        │                    │                     │
        └────────────────────┴─────────────────────┘
                              │
                    ┌─────────▼──────────┐
                    │  Calibration Tools │
                    │ MultiGoArucoTest   │
                    └────────────────────┘
```

### Repository Structure

| Repository | Purpose | Key Components |
|-----------|---------|----------------|
| **multigo_master** | High-level control and user interface | `nav_master` (action orchestration) |
| **multigo_navigation** | Core navigation and docking algorithms | `nav_goal`, `nav_docking`, `nav_control`, perception |
| **multigo_launch** | Integration, configuration, launch files | `boot.launch.py`, `run.launch.py`, configs |
| **MultiGoArucoTest** | Camera calibration and marker testing | `CamCalibration.py`, `ArucoTest.py` |

### Technology Stack

| Layer | Technologies |
|-------|-------------|
| **Framework** | ROS 2 Humble |
| **OS** | Ubuntu 22.04 LTS |
| **Languages** | C++ 17, Python 3.10 |
| **Navigation** | Nav2 (Navigation2 stack) |
| **SLAM** | RTAB-Map (Real-Time Appearance-Based Mapping) |
| **Vision** | OpenCV 4.x (ArUco module) |
| **Build System** | colcon (ROS 2 standard) |
| **Hardware Control** | Phidget22 (motor controllers) |

---

## Component Deep Dive

### 1. Master Control (`multigo_master`)

**Package:** `nav_master`
**Language:** C++
**Node:** `nav_master_node`

#### Responsibilities
- User interaction and confirmation workflow
- High-level action orchestration
- State management for docking sequence
- Error handling and reporting

#### Key Files
```
multigo_master/
├── nav_master/
│   ├── src/
│   │   └── nav_master.cpp          # Main control logic
│   └── action/
│       ├── Approach.action          # Approach action definition
│       └── Dock.action              # Dock action definition
```

#### Action Clients
1. **Approach Action Client**
   - Action: `/approach`
   - Type: `nav_interface::action::Approach`
   - Goal: `approach_request = true`
   - Triggers: Navigation to docking position

2. **Dock Action Client**
   - Action: `/dock`
   - Type: `nav_interface::action::Dock`
   - Goal: `dock_request = true`
   - Triggers: Precision docking sequence

#### Workflow Implementation
```cpp
// Simplified workflow
void NavMaster::run() {
    // 1. User confirmation for approach
    if (getUserConfirmation("Approach docking station? (y/n)")) {
        sendApproachGoal();
        waitForApproachCompletion();

        // 2. User confirmation for docking
        if (getUserConfirmation("Begin docking? (y/n)")) {
            sendDockGoal();
            waitForDockCompletion();

            reportSuccess();
        }
    }
}
```

**Reference:** See [multigo_master repository](https://github.com/your-org/multigo_master) for full implementation

---

### 2. Navigation & Docking (`multigo_navigation`)

#### 2.1 Navigation Goal (`nav_goal`)

**Language:** C++
**Node:** `nav_goal_node`

**Purpose:** Calculate approach goal from detected markers and interface with Nav2

**Key Functions:**
```cpp
// Calculate goal position offset from marker
geometry_msgs::msg::PoseStamped calculateApproachGoal(
    const geometry_msgs::msg::Pose& marker_pose,
    double offset_distance  // From parameter: aruco_distance_offset
);

// Publish goal to Nav2
void publishGoal(const geometry_msgs::msg::PoseStamped& goal);
```

**Topics:**
- Subscribes: `/aruco_detect/markers_left` (PoseArray)
- Publishes: `/goal_pose` (PoseStamped)

**Actions:**
- Server: `/approach`
- Client: `/navigate_to_pose` (Nav2)

**Implementation Details:**

File: `nav_goal.cpp` (location: `multigo_navigation/src/nav_goal/`)

**Approach calculation:**
```cpp
// Transform marker pose from camera frame to map frame
tf2::doTransform(marker_pose_camera, marker_pose_map, transform);

// Calculate goal offset
goal_pose.position.x = marker_pose_map.x + offset * cos(marker_yaw);
goal_pose.position.y = marker_pose_map.y + offset * sin(marker_yaw);
goal_pose.orientation = marker_orientation; // Face the marker
```

---

#### 2.2 Docking Control (`nav_docking`)

**Language:** C++
**Node:** `nav_docking_node`

**Purpose:** Precision docking using visual servoing with PID control

**Key Features:**
- Two-stage docking: single marker → dual markers
- PID control for 3 axes (X distance, Y centering, Yaw rotation)
- Two-step verification for docking completion

**File:** `nav_docking.cpp` (location: `multigo_navigation/src/nav_docking/`)

**Control Stages:**

**Stage 1: Single Front Marker (Distance > 0.7m)**
```cpp
void frontMarkerCmdVelPublisher() {
    // Use left marker (ID 20) for guidance
    double error_dist = current_distance - target_distance;
    double error_y = marker_y_position; // Centering
    double error_yaw = marker_yaw;

    // PID control for each axis
    double vel_x = pidControl(error_dist, Kp_dist, Ki_dist, Kd_dist);
    double vel_y = pidControl(error_y, Kp_y, Ki_y, Kd_y);
    double vel_yaw = pidControl(error_yaw, Kp_yaw, Ki_yaw, Kd_yaw);

    publishVelocity(vel_x, vel_y, vel_yaw);
}
```

**Stage 2: Dual Markers (Distance < 0.7m)**
```cpp
void dualMarkerCmdVelPublisher() {
    // Use both markers for maximum precision
    double center_x = (left_marker_x + right_marker_x) / 2;
    double center_y = (left_marker_y + right_marker_y) / 2;

    double error_dist = center_x - target_distance_dual;
    double error_y = center_y;
    double error_yaw = calculateYawFromTwoMarkers();

    // Same PID control, higher precision
    // ...
}
```

**PID Parameters** (loaded from `run.launch.py`):
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

**Docking Confirmation:**
```cpp
// Two-step verification
void checkDockingCompletion() {
    if (distance < close_threshold && position_stable) {
        if (!first_confirmation_received) {
            first_confirmation_received = true;
            wait(3.0); // Stability check
        } else if (!second_confirmation_received) {
            // Re-check after 3 seconds
            if (still_within_threshold) {
                second_confirmation_received = true;
                reportDockingSuccess();
            }
        }
    }
}
```

**Topics:**
- Subscribes:
  - `/aruco_detect/markers_left` (PoseArray)
  - `/aruco_detect/markers_right` (PoseArray)
- Publishes: `/cmd_vel_final` (Twist)

**Actions:**
- Server: `/dock`

**Known Issues (as of analysis):**
⚠️ **3 Critical Bugs Found** - See [Requirements Document](./03-REQUIREMENTS-DOCUMENT.md) Section DR-2.1 for details

---

#### 2.3 Velocity Control (`nav_control`)

**Language:** C++
**Node:** `nav_control_node`

**Purpose:** Adjust velocity commands based on operating mode (rotation center adjustment)

**Modes:**
1. **SOLO** - Normal navigation, center rotation
2. **DOCKING** - Forward rotation center for precision
3. **COMBINE_CHAIR** - Far forward rotation (wheelchair attached)

**Rotation Center Adjustment:**
```cpp
void adjustVelocityForMode(Twist& cmd_vel, NavigationMode mode) {
    double rotation_center_offset;

    switch(mode) {
        case SOLO:
            rotation_center_offset = 0.0;  // Rotate around center
            break;
        case DOCKING:
            rotation_center_offset = 0.25; // 25cm forward
            break;
        case COMBINE_CHAIR:
            rotation_center_offset = 0.50; // 50cm forward (wheelchair)
            break;
    }

    // Adjust linear velocities to achieve desired rotation center
    cmd_vel.linear.x += cmd_vel.angular.z * rotation_center_offset;
}
```

**Topics:**
- Subscribes:
  - `/cmd_vel` (from Nav2)
  - `/cmd_vel_final` (from nav_docking)
  - `/navigation_mode` (mode selection)
- Publishes: `/cmd_vel_adjusted` (Twist)

**Parameters:**
- `LENGTH_ROTATION_CENTER_DOCKING`: 0.25
- `LENGTH_ROTATION_CENTER_COMBINE_CHAIR`: 0.50

---

#### 2.4 Perception - ArUco Detection (`aruco_detect`)

**Language:** C++
**Node:** `aruco_detect_node`

**Purpose:** Detect and estimate pose of ArUco markers

**Algorithm:**
```cpp
void detectMarkers() {
    // 1. Capture image from camera
    cv::Mat image = camera_image_;

    // 2. Detect markers
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(
        image,
        dictionary,  // DICT_6X6_250
        marker_corners,
        marker_ids
    );

    // 3. Estimate pose for each detected marker
    for (size_t i = 0; i < marker_ids.size(); i++) {
        cv::Vec3d rvec, tvec;
        cv::aruco::estimatePoseSingleMarkers(
            marker_corners[i],
            marker_size,      // Physical marker size (meters)
            camera_matrix,    // From calibration
            dist_coeffs,      // From calibration
            rvec, tvec
        );

        // 4. Convert OpenCV frame to ROS frame
        geometry_msgs::msg::Pose marker_pose = convertToROSFrame(rvec, tvec);

        // 5. Publish
        publishMarkerPose(marker_ids[i], marker_pose);
    }
}
```

**Coordinate Frame Conversion:**
```
OpenCV:  X-right, Y-down, Z-forward
    ↓ (rotation matrix)
ROS:     X-forward, Y-left, Z-up
```

**Topics:**
- Subscribes:
  - `/camera/color/image_raw_left` (Image)
  - `/camera/color/image_raw_right` (Image)
- Publishes:
  - `/aruco_detect/markers_left` (PoseArray)
  - `/aruco_detect/markers_right` (PoseArray)
  - TF transforms: `camera_frame` → `aruco_marker_20`, `aruco_marker_21`

**Parameters:**
- `desired_aruco_marker_id_left`: 20
- `desired_aruco_marker_id_right`: 21
- `marker_size`: 0.15 (meters)

**Reference:** `aruco_detect.cpp` in `multigo_navigation/src/aruco_detect/`

---

#### 2.5 Point Cloud Processing

**Pipeline:**
```
LiDAR → LaserScan → PCL Conversion → Ego Filter → PCL Merge → Costmap
```

**Components:**

**a) laserscan_to_pcl**
- Converts LaserScan to PointCloud2
- File: `multigo_navigation/src/laserscan_to_pcl/`

**b) ego_pcl_filter**
- Removes robot self-points from cloud
- Filter radius: Configurable (robot footprint)
- File: `multigo_navigation/src/ego_pcl_filter/`

**c) pcl_merge**
- Merges multiple point cloud sources
- Inputs: Filtered LiDAR, optional depth camera
- Output: `/merged_cloud`
- File: `multigo_navigation/src/pcl_merge/`

**Data Flow:**
```
/scan (LaserScan)
    → /scan_pcl (PointCloud2)
    → /filtered_pcl (PointCloud2, ego removed)
    → /merged_cloud (PointCloud2, final)
    → Nav2 Costmap (obstacle_layer)
```

---

### 3. Launch & Configuration (`multigo_launch`)

**Package:** `boot`

#### 3.1 Launch Files

**File: `boot.launch.py`**
- Purpose: Hardware driver initialization
- Launches:
  - Camera drivers (left, right)
  - LiDAR driver (Hesai)
  - Motor controller (Phidgets)
  - Static transforms (URDF)

**File: `run.launch.py`** (487 lines - **Configuration Hub**)
- Purpose: Navigation stack and docking system
- Launches:
  - Nav2 stack (planner, controller, costmaps)
  - RTAB-Map SLAM
  - All docking nodes (nav_goal, nav_docking, nav_control)
  - Perception nodes (aruco_detect)
  - Point cloud processing
- Loads:
  - All parameters for all subsystems
  - Nav2 configuration from `nav2_params.yaml`
  - Robot mode settings

**File: `simulation.launch.py`**
- Purpose: Gazebo simulation
- Spawns: Robot model, ArUco markers in world

#### 3.2 Configuration Files

**nav2_params.yaml** (357 lines)
```yaml
# Global Planner (NavFn)
planner_server:
  ros__parameters:
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5          # Goal tolerance (meters)
      use_astar: true         # A* algorithm
      allow_unknown: true     # Navigate through unknown space

# Local Planner (DWB)
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.26         # Max forward velocity (m/s)
      max_vel_theta: 1.0      # Max rotation (rad/s)
      acc_lim_x: 2.5          # Acceleration limit
      acc_lim_theta: 3.2

# Global Costmap
global_costmap:
  global_costmap:
    ros__parameters:
      robot_radius: 0.28      # Robot size
      inflation_radius: 0.55  # Safety buffer
      resolution: 0.05        # Map resolution (5cm)

# Local Costmap
local_costmap:
  local_costmap:
    ros__parameters:
      width: 3.0              # 3m × 3m local window
      height: 3.0
      update_frequency: 5.0   # Hz
      obstacle_layer:
        observation_sources: scan_cloud
        scan_cloud:
          topic: /merged_cloud
          data_type: "PointCloud2"
```

**Reference:** Full configuration at `multigo_launch/config/nav2_params.yaml`

---

## Data Flow & Communication

### Topic Graph

```
┌─────────────┐
│   Cameras   │
└──────┬──────┘
       │ image_raw_left, image_raw_right
       ▼
┌─────────────┐      markers_left/right
│aruco_detect │ ────────────────────┐
└─────────────┘                     │
                                    ▼
┌─────────────┐                ┌─────────┐      goal_pose
│   LiDAR     │──scan──┐       │nav_goal │ ────────────┐
└─────────────┘        │       └─────────┘             │
                       │                               ▼
┌─────────────┐        │                          ┌─────────┐
│laserscan2pcl│◄───────┘                          │  Nav2   │
└──────┬──────┘                                   └────┬────┘
       │ scan_pcl                                      │
       ▼                                               │ cmd_vel
┌─────────────┐      filtered_pcl                     │
│ego_pcl_filter│──────────────┐                       │
└─────────────┘               │                       │
                              ▼                       │
                        ┌──────────┐                  │
                        │pcl_merge │                  │
                        └────┬─────┘                  │
                             │ merged_cloud           │
                             ├────────────────────────┤
                             │                        │
                             ▼                        │
                    ┌──────────────┐                  │
                    │Nav2 Costmaps │                  │
                    └──────────────┘                  │
                                                      │
markers_left/right                                    │
       ┌───────────────────────────┐                  │
       ▼                           │                  │
┌─────────────┐      cmd_vel_final │                  │
│nav_docking  │────────────────────┤                  │
└─────────────┘                    │                  │
                                   ▼                  ▼
                             ┌──────────────────────────┐
                             │     nav_control          │
                             │  (mode-based routing)    │
                             └───────────┬──────────────┘
                                        │ cmd_vel_adjusted
                                        ▼
                                ┌───────────────┐
                                │mecanum_wheels │
                                └───────┬───────┘
                                        │ wheel velocities
                                        ▼
                                ┌───────────────┐
                                │Motor Hardware │
                                └───────────────┘
```

### Action Flow

```
┌─────────────┐
│ nav_master  │ (User presses "Approach")
└──────┬──────┘
       │ /approach goal
       ▼
┌─────────────┐
│  nav_goal   │ (Calculates approach position)
└──────┬──────┘
       │ /navigate_to_pose goal
       ▼
┌─────────────┐
│    Nav2     │ (Executes navigation)
└──────┬──────┘
       │ Result: Success
       ▼
┌─────────────┐
│ nav_master  │ (User presses "Dock")
└──────┬──────┘
       │ /dock goal
       ▼
┌─────────────┐
│ nav_docking │ (Visual servoing docking)
└──────┬──────┘
       │ Result: Success (verified)
       ▼
┌─────────────┐
│ nav_master  │ (Reports completion)
└─────────────┘
```

---

## Key Algorithms

### 1. RTAB-Map SLAM (Localization & Mapping)

**Type:** Online SLAM (learns while navigating)

**How it works:**
```
1. Robot explores environment
2. Cameras capture visual features
3. RTAB-Map creates "nodes" (key locations) with features
4. Loop closure: Recognizes when returning to known places
5. Graph optimization: Corrects accumulated drift
6. Outputs: Map + robot position continuously
```

**Configuration:**
```yaml
rtabmap:
  Mem/IncrementalMemory: true    # Continuous learning
  Mem/InitWMWithAllNodes: false  # Start fresh or load previous
  database_path: /path/to/rtabmap.db  # Persistent memory
```

**Performance:**
- Position accuracy: ±5-10cm
- Loop closure: Corrects drift when revisiting areas
- Memory: Grows with environment size (database pruning available)

**Reference:** See [CEA Layman Guide Section 23](./CEA_PRESENTATION_LAYMAN_GUIDE.md#23-how-rtab-map-navigation-works) for detailed explanation

### 2. Nav2 Path Planning

**Global Planner: NavFn (A*)**
```
Algorithm:
1. Start position: Current robot pose
2. Goal position: From nav_goal
3. Costmap: Static map + inflation
4. Search: A* finds lowest-cost path
5. Output: Sequence of waypoints
```

**Local Planner: DWB (Dynamic Window Approach)**
```
Algorithm:
1. Sample velocity space: (vx, vy, ω) combinations
2. Forward simulate: Predict trajectory for each sample
3. Score trajectories:
   - Path alignment: Follow global plan
   - Obstacle avoidance: Stay clear of obstacles
   - Goal approach: Make progress toward goal
4. Select: Highest-scoring trajectory
5. Execute: Send velocity commands
6. Repeat: 10 Hz (every 0.1 seconds)
```

**Costmap Layers:**
1. Static Layer: Map from RTAB-Map
2. Obstacle Layer: Real-time LiDAR data
3. Inflation Layer: Safety buffer around obstacles

### 3. Visual Servoing (Docking)

**PID Control (Per Axis):**
```
Error = Target - Current
Proportional = Kp * Error
Integral = Ki * Σ(Error * dt)
Derivative = Kd * (Error - PrevError) / dt

Output = Proportional + Integral + Derivative
```

**For Docking:**
```python
# X-axis (distance control)
error_dist = target_distance - current_distance
vel_x = PID(error_dist, Kp_dist, Ki_dist, Kd_dist)

# Y-axis (centering)
error_y = 0 - marker_y_position  # Target is center (y=0)
vel_y = PID(error_y, Kp_y, Ki_y, Kd_y)

# Yaw (rotation)
error_yaw = 0 - marker_yaw  # Target is aligned (yaw=0)
vel_yaw = PID(error_yaw, Kp_yaw, Ki_yaw, Kd_yaw)

# Publish combined velocity
cmd_vel = Twist(linear_x=vel_x, linear_y=vel_y, angular_z=vel_yaw)
```

**Transition Logic:**
```
if distance > 0.7m:
    use_single_marker_control()  # Front marker only
else:
    use_dual_marker_control()    # Both markers, precision mode
```

---

## ROS 2 Integration

### Node Communication Patterns

**1. Topics (Continuous Data Streams)**
```cpp
// Example: Publishing velocity commands
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

// In timer callback (10 Hz)
void publishVelocity() {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.2;
    pub_->publish(msg);
}
```

**2. Actions (Long-Running Tasks)**
```cpp
// Example: Dock action server
rclcpp_action::Server<nav_interface::action::Dock>::SharedPtr action_server_;

action_server_ = rclcpp_action::create_server<nav_interface::action::Dock>(
    this,
    "/dock",
    std::bind(&NavDocking::handleGoal, this, _1, _2),
    std::bind(&NavDocking::handleCancel, this, _1),
    std::bind(&NavDocking::handleAccepted, this, _1)
);

// Provide feedback during execution
auto feedback = std::make_shared<Dock::Feedback>();
feedback->current_distance = calculateDistance();
goal_handle->publish_feedback(feedback);

// Report result when done
auto result = std::make_shared<Dock::Result>();
result->success = true;
goal_handle->succeed(result);
```

**3. TF Transforms (Coordinate Frames)**
```cpp
// Publish transform: camera → marker
geometry_msgs::msg::TransformStamped transform;
transform.header.stamp = now();
transform.header.frame_id = "camera_left";
transform.child_frame_id = "aruco_marker_20";
transform.transform = markerPose;

tf_broadcaster_->sendTransform(transform);

// Lookup transform: map → marker
geometry_msgs::msg::TransformStamped transform;
try {
    transform = tf_buffer_->lookupTransform(
        "map", "aruco_marker_20",
        tf2::TimePointZero
    );
} catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(get_logger(), "TF lookup failed: %s", ex.what());
}
```

### Parameter Management

**Declare and use parameters:**
```cpp
// In constructor
declare_parameter<double>("Kp_dist", 0.5);
declare_parameter<double>("aruco_distance_offset", 0.305);

// Get parameter value
double Kp_dist = get_parameter("Kp_dist").as_double();

// Dynamic reconfiguration (optional)
auto param_callback = [this](const std::vector<rclcpp::Parameter>& params) {
    for (const auto& param : params) {
        if (param.get_name() == "Kp_dist") {
            Kp_dist_ = param.as_double();
        }
    }
};
param_subscriber_ = add_on_set_parameters_callback(param_callback);
```

---

## Hardware Interfaces

### 1. Cameras (RGB)

**Hardware:** 2× RGB cameras (USB or CSI)
**Driver:** Standard ROS 2 camera driver (e.g., `usb_cam`, `v4l2_camera`)
**Resolution:** 1280×720 @ 30 fps
**Calibration:** Required (see `MultiGoArucoTest/CamCalibration.py`)

**Topics:**
- `/camera/color/image_raw_left` (sensor_msgs/Image)
- `/camera/color/image_raw_right` (sensor_msgs/Image)
- `/camera/camera_info_left` (sensor_msgs/CameraInfo)
- `/camera/camera_info_right` (sensor_msgs/CameraInfo)

### 2. LiDAR (Hesai)

**Hardware:** Hesai 3D LiDAR
**Driver:** `HesaiLidar_ROS_2.0` (third-party package)
**Range:** 0.1m - 30m
**Update Rate:** 10 Hz

**Topics:**
- `/scan` (sensor_msgs/LaserScan) OR
- `/points` (sensor_msgs/PointCloud2)

### 3. Motor Controllers (Phidgets BLDC)

**Hardware:** 4× Phidget22 BLDC motor controllers
**Driver:** Phidget22 Python API
**Interface:** USB
**Control:** Velocity mode with encoder feedback

**Communication:**
```python
from Phidget22.Devices.DCMotor import DCMotor

# Initialize motor
motor = DCMotor()
motor.setChannel(0)  # Motor channel (0-3)
motor.openWaitForAttachment(5000)

# Set velocity (-1.0 to +1.0)
motor.setTargetVelocity(0.5)

# Read encoder
position = motor.getPosition()
velocity = motor.getVelocity()
```

**Node:** `mecanum_wheels/phidgets_control.py`

---

## Development Environment

### System Requirements

```yaml
OS: Ubuntu 22.04 LTS
ROS: ROS 2 Humble
Python: 3.10+
C++: C++17 standard
Build: colcon
```

### Installation

**1. Install ROS 2 Humble:**
```bash
# Follow official ROS 2 Humble installation guide
# https://docs.ros.org/en/humble/Installation.html
```

**2. Clone repositories:**
```bash
mkdir -p ~/multigo_ws/src
cd ~/multigo_ws/src

# Clone main navigation repository
git clone <multigo_navigation_url>

# Import other repositories
vcs import < multigo_navigation/multigo.repos
```

**3. Install dependencies:**
```bash
cd ~/multigo_ws
rosdep install --from-paths src --ignore-src -r -y
```

**4. Build:**
```bash
colcon build --symlink-install
source install/setup.bash
```

### Running the System

**Hardware launch:**
```bash
ros2 launch boot boot.launch.py
```

**Navigation & docking:**
```bash
ros2 launch boot run.launch.py
```

**Simulation (Gazebo):**
```bash
ros2 launch boot simulation.launch.py
```

**Master control:**
```bash
ros2 run nav_master nav_master_node
```

---

## Code Structure

### Package Layout (Typical)

```
package_name/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package manifest (dependencies)
├── include/
│   └── package_name/
│       └── header.hpp          # Public headers
├── src/
│   └── node.cpp                # Implementation
├── launch/
│   └── package.launch.py       # Launch file
├── config/
│   └── params.yaml             # Parameters
└── test/
    └── test_node.cpp           # Unit tests
```

### Coding Standards

**C++ Style:**
- Follow ROS 2 C++ style guide
- Use `rclcpp::Node` as base class
- RAII for resource management
- Smart pointers (`std::shared_ptr`, `std::unique_ptr`)

**Python Style:**
- PEP 8 compliance
- Type hints where applicable
- Docstrings for functions/classes

**Naming Conventions:**
```cpp
// Classes: PascalCase
class NavDockingNode : public rclcpp::Node {};

// Variables: snake_case
double target_distance_;

// Constants: UPPER_SNAKE_CASE
const double MAX_VELOCITY = 0.26;

// Functions: camelCase (ROS 2) or snake_case (Python)
void publishVelocity();
```

---

## Next Steps for Developers

**1. Understanding the System:**
→ Read: [System Overview & User Guide](./01-SYSTEM-OVERVIEW-USER-GUIDE.md)
→ Review: This document thoroughly
→ Study: [Requirements Document](./03-REQUIREMENTS-DOCUMENT.md)

**2. Setting Up Development:**
→ Follow: [Getting Started Guide](./04-GETTING-STARTED-GUIDE.md)
→ Build: Clone and compile all repositories
→ Test: Run simulation first

**3. Contributing:**
→ Review: Existing code in target repository
→ Check: [Known issues in Requirements Doc](./03-REQUIREMENTS-DOCUMENT.md)
→ Fix: Start with critical bugs (see DR-2.1, DR-3.2)

**4. Testing:**
→ Write: Unit tests for new code
→ Run: Simulation tests before hardware
→ Validate: Field testing with data collection

---

## Related Documents

- **[System Overview & User Guide](./01-SYSTEM-OVERVIEW-USER-GUIDE.md)** - User perspective
- **[Requirements Document](./03-REQUIREMENTS-DOCUMENT.md)** - Detailed requirements & bugs
- **[Getting Started Guide](./04-GETTING-STARTED-GUIDE.md)** - Setup instructions
- **[Complete Requirements](./claude_code_analysis/complete-system-analysis/requirements-complete.md)** - Technical requirements with status

---

**Document maintained by:** MultiGo Development Team
**Architecture Analysis:** Based on 4-repository code review (Nov 2025)
**For technical support:** See project documentation or contact lead developer
