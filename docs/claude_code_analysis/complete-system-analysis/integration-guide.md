# Multi Go System Integration Guide

**Version:** 1.0
**Date:** November 25, 2025
**Target Audience:** Developers, System Integrators, Field Engineers

---

## Document Purpose

This guide provides **step-by-step instructions** for integrating, configuring, deploying, and operating the complete Multi Go autonomous robot system. It covers all four repositories and explains how they work together.

**Covered Repositories:**
- `multigo_navigation` - Core navigation and docking algorithms
- `multigo_launch` - System integration and configuration
- `multigo_master` - Master control
- `MultiGoArucoTest` - Calibration and testing tools

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [System Setup](#system-setup)
3. [Camera Calibration Workflow](#camera-calibration-workflow)
4. [Build and Installation](#build-and-installation)
5. [Hardware Connection](#hardware-connection)
6. [System Startup](#system-startup)
7. [Verification and Testing](#verification-and-testing)
8. [Operating the Docking System](#operating-the-docking-system)
9. [Parameter Tuning](#parameter-tuning)
10. [Troubleshooting](#troubleshooting)
11. [Integration Points Reference](#integration-points-reference)

---

## Prerequisites

### Hardware Requirements

| Component | Specification | Notes |
|-----------|---------------|-------|
| **Compute** | Intel NUC or equivalent | Ubuntu 22.04, 8GB+ RAM |
| **Cameras** | 2x RGB cameras, 1280x720 @ 30fps | USB or Ethernet interface |
| **LiDAR** | Hesai LiDAR (model TBD) | ROS2 driver available |
| **Motor Controller** | Phidget22 BLDC controllers | USB connection |
| **Motors** | 4x BLDC motors with encoders | Mecanum wheel configuration |
| **Wheels** | 4x Mecanum wheels, 0.0762m diameter | 3-inch wheels |
| **Power** | Battery or AC power | Sufficient for all components |
| **Network** | WiFi or Ethernet (optional) | For remote operation/visualization |

### Software Requirements

| Software | Version | Installation |
|----------|---------|--------------|
| **OS** | Ubuntu 22.04 LTS | `sudo apt install ubuntu-desktop` |
| **ROS2** | Humble Hawksbill | [ROS2 Humble Install](https://docs.ros.org/en/humble/Installation.html) |
| **Python** | 3.10+ | Included with Ubuntu 22.04 |
| **OpenCV** | 4.x | `sudo apt install libopencv-dev` |
| **Phidget22** | Latest | `pip3 install Phidget22` |
| **PCL** | 1.12+ | `sudo apt install ros-humble-pcl-ros` |
| **Nav2** | Humble release | `sudo apt install ros-humble-navigation2` |
| **RTAB-Map** | Humble release | `sudo apt install ros-humble-rtabmap-ros` |

### Development Tools

```bash
sudo apt update
sudo apt install -y \
  git \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  build-essential \
  cmake \
  wget
```

---

## System Setup

### Step 1: Clone Main Repository

```bash
# Create workspace directory
mkdir -p ~/multigo_ws
cd ~/multigo_ws

# Clone main repository
git clone <multigo_navigation_url> src/multigo_navigation
cd src/multigo_navigation

# Switch to feature/localization branch (if not default)
git checkout feature/localization
```

### Step 2: Import External Dependencies

```bash
cd ~/multigo_ws/src

# Import all external repositories using vcs tool
vcs import < multigo_navigation/multigo.repos
```

**This will clone:**
- `multigo_launch` (boot package)
- `multigo_master` (nav_interface, nav_master)
- `mecanum_drive` (kinematics library)
- `HesaiLidar_ROS_2.0` (LiDAR driver)

**Verify all repositories cloned:**
```bash
ls ~/multigo_ws/src
# Expected output:
# multigo_navigation/
# multigo_launch/
# multigo_master/
# mecanum_drive/
# HesaiLidar_ROS_2.0/
```

### Step 3: Clone Calibration Tools (Separate)

```bash
cd ~
git clone <MultiGoArucoTest_url>

# This is a separate repository for offline calibration
```

### Step 4: Install ROS Dependencies

```bash
cd ~/multigo_ws

# Initialize rosdep (first time only)
sudo rosdep init
rosdep update

# Install all dependencies
rosdep install --from-paths src --ignore-src -r -y
```

---

## Camera Calibration Workflow

**⚠️ Important:** Camera calibration must be done **before** first use and **after any camera hardware changes**.

### Calibration Materials Needed

- **Chessboard pattern:** 9x6 internal corners (10x7 squares)
  - Print on flat, rigid surface (foam board recommended)
  - Square size: 20-30mm (larger is better for far-distance calibration)
- **Good lighting:** Uniform, diffuse lighting
- **Workspace:** 2m x 2m clear area

### Step 1: Prepare Calibration Environment

```bash
cd ~/MultiGoArucoTest/ArucoTest

# Install Python dependencies
pip3 install opencv-python numpy pickle

# Connect left camera (USB or network)
# Verify camera accessible
ls /dev/video*
# Expected: /dev/video0 (or similar)
```

### Step 2: Run Calibration Script

```bash
python3 CamCalibration.py
```

**Calibration Process:**

1. **Script will open camera feed window**
2. **Position chessboard in view:**
   - Cover different areas of image (center, corners, edges)
   - Vary distance (0.3m - 2m)
   - Vary angles (tilted, rotated)
3. **Capture 20-30 images:**
   - Press `SPACE` to capture when chessboard detected (green overlay)
   - Ensure variety in positions and orientations
4. **Script will process images:**
   - Detects corners
   - Calculates camera matrix and distortion coefficients
   - Reports reprojection error (target: <1 pixel)
5. **Output files:**
   - `calib.pckl` - Python pickle format
   - Console output with calibration results

### Step 3: Convert Calibration to ROS Format

```bash
# Convert pickle to YAML for ROS2
python3 convert_calib_to_yaml.py calib.pckl calib.yaml

# If convert script not available, manually create calib.yaml:
# Format:
# camera_matrix:
#   rows: 3
#   cols: 3
#   data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
# distortion_coefficients:
#   rows: 1
#   cols: 5
#   data: [k1, k2, p1, p2, k3]
```

### Step 4: Deploy Calibration to Robot

```bash
# Copy calibration file to camera_publisher config directory
cp calib.yaml ~/multigo_ws/src/multigo_navigation/src/camera_publisher/config/

# Repeat for right camera
# (Connect right camera, run CamCalibration.py again, save as calib_right.yaml)
```

### Step 5: Validate Calibration

```bash
# Test ArUco detection with calibrated camera
python3 ArucoTest.py

# Verify:
# - Marker detection reliable at various distances
# - Distance estimation accurate (compare to measured distance)
# - No significant distortion in image
```

**Calibration Quality Metrics:**
- ✅ **Good:** Reprojection error <0.5 pixel
- 🟡 **Acceptable:** 0.5-1.0 pixel
- ❌ **Poor:** >1.0 pixel (re-calibrate)

---

## Build and Installation

### Step 1: Build Workspace

```bash
cd ~/multigo_ws

# Build all packages with symlink install
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5

# Build time: 5-10 minutes on typical hardware
```

**Expected output:**
```
Summary: 15 packages finished [5min 32s]
  1 package had stderr output: [list of warnings, if any]
```

**Troubleshooting Build Errors:**
- **Missing dependencies:** Re-run `rosdep install`
- **OpenCV not found:** `sudo apt install libopencv-dev`
- **PCL errors:** `sudo apt install ros-humble-pcl-ros`

### Step 2: Source Workspace

```bash
source ~/multigo_ws/install/setup.bash

# Add to ~/.bashrc for automatic sourcing
echo "source ~/multigo_ws/install/setup.bash" >> ~/.bashrc
```

### Step 3: Verify Build

```bash
# Check that all packages are available
ros2 pkg list | grep -E "(nav_goal|nav_docking|aruco_detect|boot)"

# Expected output:
# nav_goal
# nav_docking
# aruco_detect
# nav_control
# mecanum_wheels
# boot (multigo_launch)
# ... (and others)
```

---

## Hardware Connection

### Camera Connection

**Left Camera:**
```bash
# Connect to USB port
# Verify device node
ls -l /dev/video*

# Expected: /dev/video0

# Test camera feed
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0

# Verify image topic
ros2 topic echo /image_raw --once
```

**Right Camera:**
```bash
# Connect to second USB port
# Expected: /dev/video1

# Cameras should be:
# - Left camera: /dev/video0
# - Right camera: /dev/video1
```

**Camera Mounting:**
- **Spacing:** 0.30-0.40m apart (baseline for stereo)
- **Orientation:** Both facing forward, parallel
- **Height:** Appropriate for marker detection range
- **Field of view:** Markers visible from 0.5m - 5m

### LiDAR Connection

```bash
# Connect Hesai LiDAR via Ethernet or USB
# Configure static IP if Ethernet (e.g., 192.168.1.201)

# Test LiDAR driver
ros2 launch hesai_ros_driver hesai_ros_driver.launch.py

# Verify point cloud topic
ros2 topic echo /scan --once
```

### Motor Controller Connection

```bash
# Connect Phidget controllers via USB
# Verify Phidget devices
lsusb | grep Phidget

# Expected: Phidget devices listed

# Test Phidget connection
python3 -c "from Phidget22.Devices.DCMotor import DCMotor; print('Phidget22 OK')"
```

### Complete Hardware Checklist

- [ ] Left camera connected (/dev/video0)
- [ ] Right camera connected (/dev/video1)
- [ ] LiDAR connected and powered
- [ ] Phidget motor controllers connected
- [ ] All motors connected to controllers
- [ ] Motor encoders connected
- [ ] Power supply connected and ON
- [ ] Emergency stop accessible (if available)

---

## System Startup

### Boot Sequence Overview

```
┌─────────────────────────────────────────────────┐
│ Phase 1: Hardware Drivers (boot.launch.py)     │
│ • Cameras, LiDAR, Motors, Static Transforms     │
└───────────────────┬─────────────────────────────┘
                    │ (Wait for all hardware ready)
                    ↓
┌─────────────────────────────────────────────────┐
│ Phase 2: Navigation Stack (run.launch.py)      │
│ • Nav2, RTAB-Map, Docking, Control             │
└───────────────────┬─────────────────────────────┘
                    │ (System ready)
                    ↓
┌─────────────────────────────────────────────────┐
│ Phase 3: Localization Initialization           │
│ • Build/load RTAB-Map, wait for localization   │
└───────────────────┬─────────────────────────────┘
                    │
                    ↓
              [READY FOR OPERATION]
```

### Phase 1: Launch Hardware Drivers

**Terminal 1:**
```bash
cd ~/multigo_ws
source install/setup.bash

# Launch hardware drivers
ros2 launch boot boot.launch.py
```

**Expected output:**
```
[INFO] [camera_publisher]: Left camera opened: /dev/video0
[INFO] [camera_publisher]: Right camera opened: /dev/video1
[INFO] [hesai_driver]: LiDAR connected
[INFO] [mecanum_wheels]: Phidget motors initialized
[INFO] [static_transform_publisher]: Publishing static transforms
```

**Verify hardware launch:**
```bash
# In a new terminal (Terminal 2)
source ~/multigo_ws/install/setup.bash

# Check nodes
ros2 node list
# Expected: camera_publisher, hesai_driver, mecanum_wheels, ...

# Check topics
ros2 topic list | grep -E "(camera|scan|cmd_vel)"
# Expected: /camera/color/image_raw_left, /scan, /cmd_vel, ...
```

**⚠️ Do not proceed to Phase 2 until all hardware nodes are running.**

### Phase 2: Launch Navigation Stack

**Terminal 3:**
```bash
cd ~/multigo_ws
source install/setup.bash

# Launch navigation and docking stack
ros2 launch boot run.launch.py
```

**Expected output:**
```
[INFO] [aruco_detect]: ArUco detector initialized (DICT_6X6_250)
[INFO] [nav_goal]: Approach action server started
[INFO] [nav_docking]: Dock action server started
[INFO] [nav_control]: Kinematic transform node ready
[INFO] [controller_server]: DWB local planner loaded
[INFO] [planner_server]: NavFn global planner loaded
[INFO] [rtabmap]: RTAB-Map node started
[INFO] [bt_navigator]: Behavior tree ready
```

**Verify navigation launch:**
```bash
# Terminal 4
source ~/multigo_ws/install/setup.bash

# Check action servers
ros2 action list
# Expected: /approach, /dock, /navigate_to_pose, /compute_path_to_pose

# Check nodes
ros2 node list | grep -E "(nav_goal|nav_docking|rtabmap|controller_server)"
```

### Phase 3: Localization Initialization

**Option A: Build New Map (First Time)**
```bash
# Move robot around manually or with joystick
# RTAB-Map will build map automatically as robot explores

# Monitor map building
ros2 topic echo /rtabmap/info

# Wait until loop closures detected (indicates map quality)
```

**Option B: Load Existing Map**
```bash
# If you have a saved RTAB-Map database
# Stop run.launch.py (Ctrl+C in Terminal 3)

# Re-launch with map file
ros2 launch boot run.launch.py \
  rtabmap_args:="--delete_db_on_start false" \
  database_path:=/path/to/rtabmap.db
```

**Verify localization:**
```bash
# Check localization pose published
ros2 topic hz /rtabmap/localization_pose
# Expected: ~10-30 Hz

# Check TF tree includes map -> odom -> base_link
ros2 run tf2_ros tf2_echo map base_link
# Expected: Transform with recent timestamp
```

**System Status: ✅ READY FOR OPERATION**

---

## Verification and Testing

### Quick System Health Check

```bash
#!/bin/bash
# save as: check_system_health.sh

echo "=== Multi Go System Health Check ==="

echo -e "\n[1/7] Checking nodes..."
ros2 node list | grep -E "(camera|aruco|nav_goal|nav_docking|nav_control|mecanum|rtabmap|controller_server)" || echo "⚠️ Some nodes missing"

echo -e "\n[2/7] Checking action servers..."
ros2 action list | grep -E "(approach|dock|navigate_to_pose)" || echo "⚠️ Action servers not ready"

echo -e "\n[3/7] Checking camera feeds..."
timeout 2 ros2 topic echo /camera/color/image_raw_left --once > /dev/null && echo "✅ Left camera OK" || echo "❌ Left camera fail"
timeout 2 ros2 topic echo /camera/color/image_raw_right --once > /dev/null && echo "✅ Right camera OK" || echo "❌ Right camera fail"

echo -e "\n[4/7] Checking LiDAR..."
timeout 2 ros2 topic echo /scan --once > /dev/null && echo "✅ LiDAR OK" || echo "❌ LiDAR fail"

echo -e "\n[5/7] Checking ArUco detection..."
ros2 topic hz /aruco_detect/markers_left --window 10 || echo "⚠️ No markers detected (this is OK if markers not in view)"

echo -e "\n[6/7] Checking TF tree..."
ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -q "At time" && echo "✅ Localization OK" || echo "❌ Localization fail"

echo -e "\n[7/7] Checking parameters loaded..."
ros2 param list /nav_docking_node | grep -q Kp_dist && echo "✅ Docking params OK" || echo "❌ Docking params missing"

echo -e "\n=== Health Check Complete ==="
```

**Run health check:**
```bash
chmod +x check_system_health.sh
./check_system_health.sh
```

### Functional Tests

#### Test 1: Camera and ArUco Detection

**Setup:**
- Print ArUco markers ID 20 and 21 (DICT_6X6_250)
- Marker size: 150mm x 150mm recommended
- Mount markers on docking station or test board

**Test Procedure:**
```bash
# Terminal 1: Monitor left marker detections
ros2 topic echo /aruco_detect/markers_left

# Terminal 2: Monitor right marker detections
ros2 topic echo /aruco_detect/markers_right

# Place marker ID 20 in front of left camera
# Expected: Pose published in /aruco_detect/markers_left

# Place marker ID 21 in front of right camera
# Expected: Pose published in /aruco_detect/markers_right

# Verify pose accuracy by measuring distance and comparing to reported Z value
```

**Acceptance:**
- ✅ Both markers detected reliably
- ✅ Distance estimation within ±5cm of measured distance
- ✅ Detection rate >20 Hz

#### Test 2: Motor Control

**Setup:**
- Clear area around robot (2m radius)
- Emergency stop ready

**Test Procedure:**
```bash
# Publish test velocity command (SLOWLY)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

# Expected: Robot moves forward slowly

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

# Test lateral movement (mecanum)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.05, z: 0.0}, angular: {z: 0.0}}"

# Expected: Robot moves sideways (left or right depending on sign)

# Test rotation
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.1}}"

# Expected: Robot rotates in place
```

**Acceptance:**
- ✅ Robot responds to all velocity commands
- ✅ Motion direction correct (forward = +X, left = +Y, CCW = +Z)
- ✅ Motor encoders feedback functioning (check /real_vel topic)

#### Test 3: Navigation

**Setup:**
- RTAB-Map localized
- Clear path to goal

**Test Procedure:**
```bash
# Send navigation goal (adjust coordinates for your map)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, \
    pose: {position: {x: 1.0, y: 0.5, z: 0.0}, \
           orientation: {w: 1.0}}}"

# Monitor in RViz (optional, Terminal 5)
ros2 run rviz2 rviz2

# Expected: Robot plans path and navigates to goal

# Check status
ros2 topic echo /navigate_to_pose/_action/status
```

**Acceptance:**
- ✅ Path planned without errors
- ✅ Robot avoids obstacles
- ✅ Goal reached within tolerance (±0.5m default)

#### Test 4: Approach Action

**Setup:**
- Marker ID 20 visible to left camera
- Clear path to marker

**Test Procedure:**
```bash
# Send approach action
ros2 action send_goal /approach nav_interface/action/Approach \
  "{approach_request: true}" \
  --feedback

# Expected output:
# - Feedback: approaching...
# - Result: success = true

# Measure final distance to marker
# Expected: ~30cm (aruco_distance_offset = 0.305m)
```

**Acceptance:**
- ✅ Action completes successfully
- ✅ Final position within 30 ± 5cm of marker
- ✅ Robot aligned facing marker

#### Test 5: Dock Action (Requires Markers in Position)

**Setup:**
- Both markers (ID 20, 21) mounted on docking station
- Markers spaced as per docking station design
- Robot positioned within ~1m of markers (after approach)

**Test Procedure:**
```bash
# Send dock action
ros2 action send_goal /dock nav_interface/action/Dock \
  "{dock_request: true}" \
  --feedback

# Expected:
# Stage 4 (front marker): Aligns using ID 20
# Stage 5 (dual marker): Precision docking using both markers
# Feedback: Distance decreasing
# Result: success = true

# Measure final position accuracy
# Use precision measurement tools (calipers, laser rangefinder)
```

**Acceptance:**
- ✅ Action completes successfully
- ✅ Final position within tolerance (target: ±1mm, but bugs may prevent this)
- ✅ Robot stable at final position
- ✅ Confirmation checks passed

**⚠️ Known Issue:** PID bugs may affect docking accuracy. See troubleshooting section.

---

## Operating the Docking System

### Using Master Control (Recommended)

**If `nav_master` is configured:**

```bash
# Launch nav_master (if not already running)
ros2 run nav_master nav_master_node

# Follow prompts:
# 1. "Approach docking station? (y/n)" → Type 'y' and press Enter
# 2. Wait for approach to complete
# 3. "Begin docking? (y/n)" → Type 'y' and press Enter
# 4. Wait for docking to complete
# 5. "Docking complete!" message displayed
```

**User Confirmation Workflow:**
```
[IDLE]
   ↓ User initiates
[Prompt: Approach?]
   ↓ User confirms (y)
[Approaching...] (Feedback displayed)
   ↓ Approach success
[Prompt: Begin docking?]
   ↓ User confirms (y)
[Docking...] (Distance feedback displayed)
   ↓ Dock success
[DOCKED - Complete!]
```

### Manual Operation (Without Master Control)

**If operating nodes directly:**

```bash
# Step 1: Send approach action
ros2 action send_goal /approach nav_interface/action/Approach \
  "{approach_request: true}"

# Wait for result: success = true

# Step 2: Send dock action
ros2 action send_goal /dock nav_interface/action/Dock \
  "{dock_request: true}"

# Monitor feedback
ros2 topic echo /dock/_action/feedback

# Wait for result: success = true
```

### Canceling Docking

**If docking needs to be aborted:**

```bash
# Cancel dock action
ros2 action send_goal /dock nav_interface/action/Dock \
  "{dock_request: false}" \
  --cancel

# Or, if using master control, send interrupt signal (Ctrl+C)
```

**Robot will:**
1. Stop motion immediately
2. Cancel action
3. Return "canceled" status

### Emergency Stop

**Hardware E-Stop (if available):**
- Press physical emergency stop button
- All motors cut immediately (hardware level)
- Requires manual reset

**Software Stop:**
```bash
# Publish zero velocity
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

# If emergency stop topic implemented (recommended addition):
ros2 topic pub /emergency_stop std_msgs/msg/Bool "data: true"
```

---

## Parameter Tuning

### Docking PID Tuning

**⚠️ Important:** Fix PID integral bug before tuning (see bugs in requirements-complete.md).

**PID Parameters Location:**
- File: `multigo_launch/launch/run.launch.py`
- Can also be set via command line:

```bash
ros2 launch boot run.launch.py \
  Kp_dist:=0.6 \
  Ki_dist:=0.15 \
  Kd_dist:=0.06
```

**Tuning Procedure:**

1. **Start with P-only (set Ki=0, Kd=0):**
   ```bash
   Kp_dist:=0.5, Ki_dist:=0.0, Kd_dist:=0.0
   ```
   - Test docking
   - If oscillates: Decrease Kp
   - If too slow: Increase Kp
   - If steady-state error: Add integral

2. **Add Integral (after fixing bug):**
   ```bash
   Kp_dist:=0.5, Ki_dist:=0.1, Kd_dist:=0.0
   ```
   - Eliminates steady-state error
   - If oscillates or overshoots: Decrease Ki

3. **Add Derivative:**
   ```bash
   Kp_dist:=0.5, Ki_dist:=0.1, Kd_dist:=0.05
   ```
   - Reduces overshoot
   - Dampens oscillations
   - If too slow: Decrease Kd

**Recommended Starting Values:**

| Parameter | Distance Control | Center Alignment | Rotation Control |
|-----------|------------------|------------------|------------------|
| **Kp** | 0.5 | 0.8 | 1.0 |
| **Ki** | 0.1 | 0.2 | 0.15 |
| **Kd** | 0.05 | 0.1 | 0.08 |

**Tuning for Different Robot Masses:**
- Heavier robot: Increase Kp, Kd
- Lighter robot: Decrease Kp, may need less Kd

**Field Tuning Process:**
1. Make small adjustments (10-20%)
2. Test 5+ docking attempts per configuration
3. Measure: Success rate, accuracy, time
4. Document best configuration
5. Re-test after robot modifications (load changes, etc.)

### Navigation Tuning

**Nav2 parameters location:**
- File: `multigo_launch/config/nav2_params.yaml`

**Key parameters to tune:**

```yaml
# Local planner (DWB)
max_vel_x: 0.26  # Increase for faster navigation
max_vel_theta: 1.0  # Rotation speed
acc_lim_x: 2.5  # Acceleration (increase if motors can handle)
acc_lim_theta: 3.2

# Costmaps
inflation_radius: 0.55  # Safety margin (decrease if too conservative)
robot_radius: 0.28  # Physical robot radius

# Global planner
tolerance: 0.5  # Goal tolerance (decrease for precision)
```

**After modifying `nav2_params.yaml`:**
```bash
# Rebuild to apply changes
cd ~/multigo_ws
colcon build --packages-select boot
source install/setup.bash

# Restart run.launch.py
```

---

## Troubleshooting

### Issue 1: Markers Not Detected

**Symptoms:**
- `/aruco_detect/markers_left` or `/markers_right` empty
- No pose feedback during docking

**Diagnostic Steps:**
```bash
# Check camera feeds
ros2 run rqt_image_view rqt_image_view
# Select /camera/color/image_raw_left

# Is image clear? Focused? Adequate lighting?
```

**Solutions:**
1. **Lighting:**
   - Increase ambient light (avoid direct sunlight on markers)
   - Use diffuse lighting

2. **Focus:**
   - Auto-focus cameras may struggle at close range
   - Use fixed-focus if possible
   - Manually adjust focus for docking distance range

3. **Marker Quality:**
   - Print markers at high resolution (300+ DPI)
   - Ensure markers flat, not wrinkled
   - Clean markers (no glare or reflections)

4. **Calibration:**
   - Re-run camera calibration if poor detection
   - Check reprojection error <1 pixel

5. **Range:**
   - Markers must be within 0.5m - 5m range
   - Larger markers for far detection
   - Smaller markers risk pixel aliasing

### Issue 2: Docking Inaccurate or Fails

**Symptoms:**
- Robot stops short or overshoots
- Oscillates around target
- Action times out or fails

**Known Bugs (Must Fix First):**
1. **PID Integral not accumulating** → Fix in nav_docking.cpp:197
2. **Dual marker distance calculation** → Fix in nav_docking.cpp:387, 503
3. **Uninitialized variables** → Initialize prev_error_* in nav_docking.h

**After Fixing Bugs:**

**Re-tune PID gains:**
- Start conservative (lower Kp)
- Increase gradually
- Document optimal values

**Check marker stability:**
```bash
ros2 topic echo /aruco_detect/markers_left
# Pose should be stable (not jittering excessively)
```

**Verify transforms:**
```bash
ros2 run tf2_ros tf2_echo base_link aruco_marker_20
# Should update at camera frame rate (~30 Hz)
# Check for large jumps in position (indicates detection instability)
```

### Issue 3: Robot Not Moving

**Symptoms:**
- `cmd_vel` commands published but no motion
- Motors silent

**Diagnostic Steps:**
```bash
# Check if mecanum_wheels node running
ros2 node list | grep mecanum

# Check cmd_vel published
ros2 topic echo /cmd_vel

# Check Phidget connection
lsusb | grep Phidget
```

**Solutions:**
1. **Phidget not connected:**
   ```bash
   # Reconnect USB
   # Restart mecanum_wheels node
   ros2 run mecanum_wheels mecanum_wheels_node
   ```

2. **Motor controllers not initialized:**
   - Check mecanum_wheels logs for Phidget errors
   - Verify motor power supply ON

3. **cmd_vel topic not subscribed:**
   - Verify mecanum_wheels subscribes to /cmd_vel
   ```bash
   ros2 topic info /cmd_vel
   # Check subscription count > 0
   ```

4. **E-stop engaged:**
   - Check if hardware e-stop button pressed
   - Reset e-stop

### Issue 4: Navigation Fails / Path Not Found

**Symptoms:**
- Nav2 reports "No path found"
- Robot stuck or rotates in place

**Diagnostic Steps:**
```bash
# Check localization
ros2 topic echo /rtabmap/localization_pose

# Check map
ros2 topic echo /map --once

# Visualize in RViz
ros2 run rviz2 rviz2
# Add: Map, Costmap, Robot model, Path
```

**Solutions:**
1. **Poor localization:**
   - Robot position in map incorrect
   - Re-localize: Move robot to known position, wait for RTAB-Map convergence
   - Or reset RTAB-Map and rebuild map

2. **Goal unreachable:**
   - Goal in obstacle or unknown space
   - Costmap inflated too much (decrease inflation_radius)
   - Try closer goal

3. **Costmap issues:**
   - Clear costmap:
   ```bash
   ros2 service call /global_costmap/clear_entirely_global_costmap std_srvs/srv/Empty
   ros2 service call /local_costmap/clear_entirely_local_costmap std_srvs/srv/Empty
   ```

### Issue 5: System Slow / High CPU Usage

**Symptoms:**
- High CPU usage (>80%)
- Delayed responses
- Low topic rates

**Diagnostic Steps:**
```bash
# Check CPU usage per process
top
# Look for high CPU processes (rtabmap, controller_server common culprits)

# Check topic rates
ros2 topic hz /camera/color/image_raw_left
# Expected: ~30 Hz
```

**Solutions:**
1. **Reduce RTAB-Map computational load:**
   - Decrease feature extraction rate
   - Reduce image resolution
   - Adjust RTAB-Map parameters

2. **Optimize costmap update rates:**
   - In `nav2_params.yaml`:
   ```yaml
   local_costmap:
     update_frequency: 3.0  # Decrease from 5.0
   ```

3. **Reduce camera resolution:**
   - If 1280x720 too high, use 640x480
   - Trade-off: Lower resolution → less accurate detection

4. **Use more powerful compute:**
   - Upgrade to higher-spec onboard computer

### Issue 6: TF Errors / Transform Timeout

**Symptoms:**
- Errors: "Could not transform..."
- Actions fail with transform exceptions

**Diagnostic Steps:**
```bash
# Check TF tree
ros2 run tf2_tools view_frames
# Generates frames.pdf

# Check specific transform
ros2 run tf2_ros tf2_echo map base_link
```

**Solutions:**
1. **Missing transform:**
   - Ensure RTAB-Map publishing map → odom
   - Ensure robot_state_publisher running (for base_link → sensor frames)

2. **Transform timeout:**
   - Increase transform tolerance in code
   - Check if transforms publishing at adequate rate:
   ```bash
   ros2 topic hz /tf
   # Expected: >10 Hz for dynamic transforms
   ```

3. **Timestamp issues:**
   - Ensure all nodes use synchronized time (use_sim_time parameter)
   - Check for clock jumps (system time changes)

---

## Integration Points Reference

### Repository Integration Map

```
multigo_launch (Configuration Hub)
    ├─→ Imports nav_interface from multigo_master
    ├─→ Launches nodes from multigo_navigation
    ├─→ Configures Nav2, RTAB-Map (third_party)
    └─→ Loads all parameters (357-line nav2_params.yaml)

multigo_master (Control Layer)
    ├─→ Defines action interfaces (Approach, Dock)
    └─→ Provides nav_master orchestration node

multigo_navigation (Execution Layer)
    ├─→ Implements action servers (nav_goal, nav_docking)
    ├─→ ArUco detection (aruco_detect)
    ├─→ Motion control (nav_control, mecanum_wheels)
    └─→ Point cloud processing (ego_pcl_filter, pcl_merge)

MultiGoArucoTest (Offline Tools)
    └─→ Generates calibration files (calib.yaml)
        └─→ Deployed to camera_publisher/config/
```

### Critical Integration Points

| Integration Point | Repository A | Repository B | Mechanism | Notes |
|-------------------|-------------|--------------|-----------|-------|
| **Action Interfaces** | multigo_master | multigo_navigation | nav_interface package | Approach, Dock actions |
| **Launch Files** | multigo_launch | multigo_navigation | ros2 launch | boot.launch.py, run.launch.py |
| **Parameters** | multigo_launch | multigo_navigation | YAML + launch args | nav2_params.yaml, run.launch.py |
| **Calibration** | MultiGoArucoTest | multigo_navigation | File copy | calib.yaml → camera_publisher/config/ |
| **Nav2 Integration** | multigo_launch | Nav2 (third_party) | Configuration | nav2_params.yaml |
| **RTAB-Map Integration** | multigo_launch | RTAB-Map (third_party) | Launch parameters | run.launch.py |

### Topic Flow Integration

```
Cameras (boot.launch.py)
    ↓ /camera/color/image_raw_left, _right
aruco_detect (run.launch.py)
    ↓ /aruco_detect/markers_left, _right
nav_goal (run.launch.py)
    ↓ /goal_pose
Nav2 bt_navigator (run.launch.py)
    ↓ /navigate_to_pose (action)
Nav2 controller_server (run.launch.py)
    ↓ /cmd_vel

[Parallel docking path:]
aruco_detect
    ↓ /aruco_detect/markers_*
nav_docking (run.launch.py)
    ↓ /cmd_vel_final
nav_control (run.launch.py)
    ↓ /cmd_vel

[Final motion:]
mecanum_wheels (boot.launch.py)
    ← /cmd_vel
    ↓ Motor commands (Phidget API)
    ↓ Robot motion
```

---

## Advanced Integration

### Custom Launch Configuration

**Create custom launch file:**

```python
# ~/multigo_ws/src/custom_multigo.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    boot_dir = get_package_share_directory('boot')

    # Include hardware launch
    boot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(boot_dir, 'launch', 'boot.launch.py')
        )
    )

    # Include navigation launch with custom parameters
    run_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(boot_dir, 'launch', 'run.launch.py')
        ),
        launch_arguments={
            'Kp_dist': '0.6',  # Custom PID gains
            'Ki_dist': '0.15',
            'Kd_dist': '0.06',
        }.items()
    )

    # Add custom monitoring node
    monitor_node = Node(
        package='custom_package',
        executable='system_monitor',
        name='system_monitor'
    )

    return LaunchDescription([
        boot_launch,
        run_launch,
        monitor_node
    ])
```

**Launch custom configuration:**
```bash
ros2 launch custom_multigo.launch.py
```

---

## Conclusion

This integration guide provides complete instructions for setting up, configuring, and operating the Multi Go autonomous robot system. The system integrates four repositories working together to provide autonomous navigation and precision docking capabilities.

**Key Success Factors:**
1. ✅ Proper camera calibration (reprojection error <1 pixel)
2. ✅ All hardware connections verified before software launch
3. ✅ Systematic verification at each integration stage
4. ✅ PID tuning after fixing known bugs
5. ✅ Field testing with data collection

**Support Resources:**
- Architecture: `/docs/complete-system-analysis/complete-architecture.md`
- Requirements: `/docs/complete-system-analysis/requirements-complete.md`
- Comparison: `/docs/complete-system-analysis/COMPARISON_BEFORE_AFTER.md`

**For Issues:**
- Check troubleshooting section in this guide
- Review bug list in requirements-complete.md
- Consult original analysis in `/docs/docking-system-analysis/`

---

**Document Version:** 1.0
**Last Updated:** November 25, 2025
**Maintainer:** Multi Go Development Team
