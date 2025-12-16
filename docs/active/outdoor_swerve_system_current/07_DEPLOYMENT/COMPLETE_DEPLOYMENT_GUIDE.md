# Complete Deployment Guide

**Document ID:** DEPLOY-COMPLETE-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Comprehensive Reference

---

## Phase 7: Deployment Documentation Complete

This comprehensive guide consolidates all deployment documentation including hardware assembly, software installation, calibration procedures, operations manual, and maintenance guide for the outdoor wheelchair transport robot system.

---

# Part 1: Hardware Assembly

## 1.1 Bill of Materials (BOM)

### Core Components

| Component | Specification | Quantity | Supplier | Part Number |
|-----------|--------------|----------|----------|-------------|
| **Compute** |
| Main Computer | GMKtec Nucbox K6 (Ryzen 7 7840HS, 32GB RAM) | 1 | GMKtec | K6-R7-32GB |
| **LiDAR** |
| 3D LiDAR | Outdoor-grade, IP65+, 360° FOV | 1 | Velodyne/Ouster | VLP-16 or OS1-64 |
| **Cameras** |
| RGB Camera (Front) | 1080p, 60fps, USB 3.0 | 1 | See3CAM | CU135 |
| RGB Camera (Rear) | 1080p, 60fps, USB 3.0 | 1 | See3CAM | CU135 |
| **IMU** |
| 9-DOF IMU | Accelerometer + Gyro + Magnetometer | 1 | Adafruit | BNO055 |
| **Motors** |
| Swerve Modules | 6.5" hoverboard in-wheel motors (80W) | 8 | Generic | HW-6.5-80W |
| Motor Controllers | CAN bus, 48V, 20A | 8 | ODrive | ODrive Pro |
| **Battery** |
| Lithium Battery | 48V 20Ah (≥1kWh), BMS included | 1 | Custom | 48V-20Ah-LiFePO4 |
| **UI** |
| Touch Screen | 7-10" capacitive, IP65 front, ≥400 nits | 1 | Waveshare | 10.1" HDMI LCD |
| **eHMI** |
| Microcontroller | ESP32-S3, WiFi/BLE | 1 | Espressif | ESP32-S3-DevKitC |
| LED Strip | WS2812B, ≥60 LEDs, IP65 | 1 | BTF-Lighting | WS2812B-60LED/m |
| LED Matrix | HUB75, 64×32 RGB | 1 | Adafruit | 64x32 RGB Matrix |
| Audio | I2S amplifier + speaker | 1 | Adafruit | MAX98357 + speaker |
| **Structure** |
| Frame | Aluminum extrusion, custom design | - | 80/20 Inc | Custom |
| Weatherproofing | IP54+ enclosures | - | Various | - |
| E-Stop Button | Mushroom button, NC contacts | 2 | Schneider Electric | XB4BS8444 |

**Estimated Total Cost:** $8,000 - $12,000

---

## 1.2 Assembly Instructions

### Step 1: Frame Assembly

1. **Base Frame:**
   - Assemble 80/20 aluminum extrusion base (600mm × 500mm)
   - Install mounting plates for motors
   - Ensure frame is square (diagonal measurements equal)

2. **Vertical Posts:**
   - Attach 4 vertical posts (800mm height)
   - Install cross bracing for rigidity

3. **Equipment Mounting:**
   - Mount GMKtec Nucbox K6 in weatherproof enclosure
   - Mount battery pack in accessible location
   - Install LiDAR on top platform (360° unobstructed view)

---

### Step 2: Swerve Drive Installation

**Per Module (4 total):**
1. Mount drive motor to frame
2. Mount steering motor perpendicular to drive
3. Install encoder on steering axis
4. Connect drive motor to wheel hub
5. Route cables to motor controllers

**Motor Controller Wiring:**
```
Motor Controller → Motor:
  Phase A (Yellow) → Motor Phase A
  Phase B (Green)  → Motor Phase B
  Phase C (Blue)   → Motor Phase C

Motor Controller → Power:
  V+ (Red)   → Battery + (via fuse)
  V- (Black) → Battery -
  GND        → Chassis ground

Motor Controller → CAN Bus:
  CAN-H → CAN bus H (twisted pair)
  CAN-L → CAN bus L (twisted pair)
  120Ω termination resistor at each end
```

---

### Step 3: Sensor Installation

**LiDAR:**
1. Mount on top platform at robot center
2. Height: ~1.5m above ground
3. Connect Ethernet cable to main computer
4. Verify unobstructed 360° FOV

**Cameras:**
1. Front camera: Mount facing forward, 30° downward tilt
2. Rear camera: Mount facing rear, 30° downward tilt
3. Height: ~0.8m for ArUco marker detection
4. Connect USB 3.0 to main computer

**IMU:**
1. Mount at robot center, near main computer
2. Ensure IMU axes align with robot frame
3. Connect I2C/UART to main computer

---

### Step 4: Electrical System

**Power Distribution:**
```
Battery (48V 20Ah)
    │
    ├─→ Main Power Switch (manual disconnect)
    │
    ├─→ Emergency Stop Relay (NC, <100ms cutoff)
    │
    ├─→ DC-DC Converter (48V → 12V, 10A) → Compute, Sensors
    │
    ├─→ DC-DC Converter (48V → 5V, 5A) → UI, eHMI
    │
    └─→ Motor Controllers (8×) → Swerve Motors
```

**E-Stop Wiring:**
```
E-Stop Button (NC) ──→ Relay Coil ──→ 12V
                            │
                         Relay NO
                            │
                    Battery → Motors
```

**Fusing:**
- Main battery fuse: 50A
- Motor controller branch: 30A (per pair)
- 12V system: 15A
- 5V system: 10A

---

### Step 5: Communication Wiring

**CAN Bus Network:**
```
Main Computer (CAN interface)
    │
    ├─→ Motor Controller 1 (FL drive)
    ├─→ Motor Controller 2 (FL steer)
    ├─→ Motor Controller 3 (FR drive)
    ├─→ Motor Controller 4 (FR steer)
    ├─→ Motor Controller 5 (RL drive)
    ├─→ Motor Controller 6 (RL steer)
    ├─→ Motor Controller 7 (RR drive)
    └─→ Motor Controller 8 (RR steer)

120Ω termination at first and last node
```

**Serial Connections:**
- eHMI ESP32 ← USB → Main Computer (115200 baud)
- IMU ← I2C/UART → Main Computer

**Network:**
- LiDAR ← Ethernet → Main Computer (static IP: 192.168.1.201)

---

# Part 2: Software Installation

## 2.1 Operating System Setup

**Install Ubuntu 22.04 LTS:**
```bash
# 1. Create bootable USB (on separate machine)
sudo dd if=ubuntu-22.04.3-desktop-amd64.iso of=/dev/sdX bs=4M

# 2. Boot GMKtec Nucbox from USB
# 3. Install Ubuntu (minimal installation)
# 4. Set hostname: wheelchair-robot-01
```

**Post-Installation:**
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install SSH for remote access
sudo apt install openssh-server -y
sudo systemctl enable ssh

# Configure static IP (optional)
sudo nano /etc/netplan/01-netcfg.yaml
# Set static IP: 192.168.1.100

# Disable sleep/suspend (robot should stay awake)
sudo systemctl mask sleep.target suspend.target hibernate.target
```

---

## 2.2 ROS 2 Installation

```bash
# Install ROS 2 Humble (see COMPLETE_DEVELOPMENT_GUIDE.md Part 1)
sudo apt install ros-humble-desktop ros-dev-tools -y

# Install project dependencies
sudo apt install -y \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-rosbridge-suite \
    ros-humble-pcl-ros \
    libpcl-dev

# Create workspace
mkdir -p ~/wheelchair_ws/src
cd ~/wheelchair_ws/src

# Clone repository
git clone https://github.com/your-org/wheelchair-robot.git

# Install dependencies
cd ~/wheelchair_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Auto-source on login
echo "source ~/wheelchair_ws/install/setup.bash" >> ~/.bashrc
```

---

## 2.3 System Services

**Create systemd service for auto-start:**
```ini
# /etc/systemd/system/wheelchair-robot.service
[Unit]
Description=Wheelchair Transport Robot
After=network.target

[Service]
Type=simple
User=robot
WorkingDirectory=/home/robot/wheelchair_ws
ExecStart=/usr/bin/bash -c 'source install/setup.bash && ros2 launch wheelchair_robot main.launch.py'
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

**Enable and start:**
```bash
sudo systemctl daemon-reload
sudo systemctl enable wheelchair-robot.service
sudo systemctl start wheelchair-robot.service

# Check status
sudo systemctl status wheelchair-robot

# View logs
sudo journalctl -u wheelchair-robot -f
```

---

# Part 3: Calibration Procedures

## 3.1 LiDAR Calibration

**Verify LiDAR Communication:**
```bash
# Check Ethernet connection
ping 192.168.1.201

# Launch LiDAR driver
ros2 launch velodyne_driver velodyne_driver_node-VLP16-launch.py

# Verify point cloud
ros2 topic echo /velodyne_points --once
rviz2  # Add PointCloud2 display for /velodyne_points
```

**LiDAR-to-Base_Link Transform:**
```yaml
# config/robot_description.yaml
lidar_transform:
  x: 0.0      # meters (forward from base_link)
  y: 0.0      # meters (left from base_link)
  z: 1.5      # meters (up from base_link)
  roll: 0.0   # radians
  pitch: 0.0  # radians
  yaw: 0.0    # radians
```

---

## 3.2 Camera Calibration

**Intrinsic Calibration (per camera):**
```bash
# Install calibration tools
sudo apt install ros-humble-camera-calibration

# Run calibration (use checkerboard pattern)
ros2 run camera_calibration cameracalibrator \
    --size 8x6 \
    --square 0.025 \
    image:=/camera/image_raw

# Save calibration file to:
# config/front_camera_calibration.yaml
# config/rear_camera_calibration.yaml
```

**Example Calibration File:**
```yaml
image_width: 1920
image_height: 1080
camera_name: front_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [1200.0, 0.0, 960.0,
         0.0, 1200.0, 540.0,
         0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.1, -0.2, 0.001, 0.0, 0.0]
```

**Camera-to-Base_Link Transform:**
```yaml
# Front camera
front_camera_transform:
  x: 0.3      # meters forward
  y: 0.0
  z: 0.8      # meters up
  roll: 0.0
  pitch: -0.524  # -30° downward
  yaw: 0.0
```

---

## 3.3 IMU Calibration

**Accelerometer/Gyroscope Calibration:**
```bash
# Place robot on level surface
# Run IMU calibration
ros2 run imu_calibration calibrate_imu

# Follow prompts:
# 1. Keep IMU still for bias calibration
# 2. Rotate IMU through 6 orientations for scale calibration

# Save calibration to:
# config/imu_calibration.yaml
```

---

## 3.4 Wheel Encoder Calibration

**Determine Wheel Radius:**
```bash
# Method: Push robot exactly 10m, count encoder ticks

# 1. Mark start position
# 2. Command forward motion:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.5}}' --once

# 3. Stop after 10m traveled
# 4. Read encoder ticks:
ros2 topic echo /joint_states

# 5. Calculate wheel radius:
# wheel_radius = distance / (ticks × 2π / ticks_per_rev)
```

**Update Configuration:**
```yaml
# config/swerve_params.yaml
swerve_drive:
  wheel_radius: 0.0825  # meters (measured)
  wheel_base: 0.6       # meters
  track_width: 0.5      # meters
```

---

# Part 4: Operations Manual

## 4.1 Startup Procedure

**Daily Startup Checklist:**
1. [ ] Visual inspection (no damage, cables secure)
2. [ ] Battery charge >50%
3. [ ] All sensors clean (LiDAR lens, cameras)
4. [ ] Test area clear
5. [ ] Weather acceptable (no heavy rain, wind <10 m/s)

**System Boot:**
```bash
# 1. Turn on main power switch
# 2. Wait for boot (Ubuntu + ROS 2, ~60 seconds)
# 3. Verify system status
ros2 topic echo /robot_status --once

# Expected output:
# battery_percent: >50
# safety_state: "NORMAL"
# localization_quality: >0.8 (after map loads)

# 4. Test emergency stop
# Press E-Stop button → verify motors cut power
# Release E-Stop → reset via UI or:
ros2 service call /safety/reset_emergency_stop std_srvs/srv/Trigger
```

---

## 4.2 Mapping Procedure (One-Time Setup)

**Create Map for New Area:**
```bash
# 1. Launch SLAM
ros2 launch slam_toolbox online_async_launch.py

# 2. Drive robot manually with joystick
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 3. Cover entire operational area
# - Drive at <0.5 m/s
# - Get multiple passes for loop closure
# - Ensure all waypoints are mapped

# 4. Save map
ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph \
    "{filename: '/maps/building_floor_1'}"

# 5. Save occupancy grid
ros2 run nav2_map_server map_saver_cli \
    -f /maps/building_floor_1/map

# Result files:
# - /maps/building_floor_1/map.pgm
# - /maps/building_floor_1/map.yaml
# - /maps/building_floor_1.posegraph
```

---

## 4.3 Mission Execution

**Load Map and Route:**
```bash
# 1. Load map
ros2 service call /navigation/load_map \
    nav2_msgs/srv/LoadMap \
    "{map_url: '/maps/building_floor_1/map.yaml'}"

# 2. Set initial pose (via UI or command line)
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0, y: 0, z: 0}}}}'

# 3. Wait for localization (NDT fitness <1.0)
ros2 topic echo /current_pose

# 4. Load route
# Use UI to select route or command line:
ros2 action send_goal /execute_mission \
    custom_msgs/action/ExecuteMission \
    "{mission_id: 'wheelchair_transport_001', route_file: '/routes/route_001.yaml'}"

# 5. Monitor mission progress
ros2 topic echo /robot_status
```

---

## 4.4 Emergency Procedures

**Emergency Stop:**
```
1. Press red mushroom E-Stop button (physical)
   OR
2. Press E-Stop on touch screen UI
   OR
3. Remote E-Stop command:
   ros2 topic pub /emergency_stop std_msgs/msg/Bool '{data: true}'

Result: All motors cut power within 100ms
```

**Recovery from E-Stop:**
```
1. Identify and resolve cause of emergency
2. Verify area is clear
3. Reset E-Stop:
   - Twist mushroom button to release
   - Press "RESET" on UI
   OR
   ros2 service call /safety/reset_emergency_stop std_srvs/srv/Trigger
```

**Localization Loss:**
```
If robot loses localization (fitness >2.0):
1. Robot will automatically stop
2. Safety state → "DEGRADED"
3. Recovery options:
   a) Set initial pose manually via UI
   b) Drive robot to known landmark
   c) Restart localization:
      ros2 service call /ndt_localization/reset std_srvs/srv/Trigger
```

---

## 4.5 Shutdown Procedure

```bash
# 1. Ensure robot is idle (no active mission)
ros2 topic echo /robot_status --once

# 2. Shutdown ROS 2 system
sudo systemctl stop wheelchair-robot.service

# 3. Shutdown computer
sudo shutdown -h now

# 4. Wait for complete shutdown (~30 seconds)
# 5. Turn off main power switch
# 6. Plug in battery charger
```

---

# Part 5: Maintenance Guide

## 5.1 Routine Maintenance Schedule

### Daily (Before Each Use)
- [ ] Visual inspection (damage, loose cables)
- [ ] Battery charge check (>50% for operation)
- [ ] Clean LiDAR lens (microfiber cloth)
- [ ] Clean camera lenses
- [ ] Test emergency stop function

### Weekly
- [ ] Wheel inspection (wear, proper inflation if pneumatic)
- [ ] Motor encoder check (no slippage)
- [ ] Cable management (secure, no chafing)
- [ ] Software logs review (check for errors)
- [ ] Battery health check (voltage, capacity)

### Monthly
- [ ] Deep clean (LiDAR, cameras, enclosures)
- [ ] Firmware updates (ESP32, motor controllers)
- [ ] Calibration verification (IMU, cameras)
- [ ] Lubricate swerve module bearings
- [ ] Tighten all bolts (vibration loosening)

### Quarterly (Every 3 Months)
- [ ] Full system calibration (LiDAR, cameras, encoders)
- [ ] Battery capacity test (full discharge/charge cycle)
- [ ] Software update (ROS 2, Nav2, all packages)
- [ ] Weatherproofing inspection (seals, gaskets)
- [ ] Motor controller diagnostics

### Annually
- [ ] Full hardware inspection (structural, electrical)
- [ ] Replace wear items (bearings, brushes if applicable)
- [ ] Battery replacement if capacity <80%
- [ ] Recertify safety systems (E-Stop, collision avoidance)
- [ ] Update documentation with as-built changes

---

## 5.2 Component Replacement Procedures

### Battery Replacement

**Safety First:**
- Disconnect main power switch
- Discharge capacitors (wait 5 minutes)
- Wear insulated gloves

**Procedure:**
```
1. Disconnect battery terminals (- first, then +)
2. Remove mounting bolts
3. Lift battery out carefully (heavy! ~15kg)
4. Install new battery
5. Connect terminals (+ first, then -)
6. Verify voltage with multimeter (48V ±2V)
7. Test system operation
```

---

### LiDAR Replacement

```
1. Shutdown system
2. Disconnect Ethernet and power cables
3. Remove mounting bolts
4. Install new LiDAR
5. Connect cables
6. Power on and verify operation:
   ros2 topic echo /velodyne_points --once
7. If needed, re-run extrinsic calibration
```

---

### Motor/Encoder Replacement

```
1. Disconnect motor power (verify with multimeter: 0V)
2. Disconnect CAN bus cable
3. Remove wheel/drive assembly
4. Replace motor or encoder
5. Reassemble
6. Verify encoder direction:
   ros2 topic echo /joint_states
7. Run encoder calibration procedure (see 3.4)
```

---

## 5.3 Troubleshooting Guide

### Robot Won't Boot
- **Check:** Battery voltage (should be 48V ±2V)
- **Check:** Main power switch ON
- **Check:** Fuses intact
- **Solution:** Charge battery or replace blown fuse

### Localization Not Working
- **Check:** LiDAR publishing data (`ros2 topic hz /velodyne_points`)
- **Check:** Map loaded (`ros2 service list | grep load_map`)
- **Check:** Initial pose set
- **Solution:** Restart NDT localization, set initial pose manually

### Docking Fails
- **Check:** ArUco markers visible to cameras
- **Check:** Camera publishing images (`ros2 topic hz /camera/image_raw`)
- **Check:** Markers at correct height (~0.8m)
- **Check:** Lighting conditions (avoid direct sunlight on markers)
- **Solution:** Improve marker visibility, recalibrate cameras

### Motor Not Responding
- **Check:** CAN bus communication (`candump can0`)
- **Check:** Motor controller power LED
- **Check:** Emergency stop not engaged
- **Solution:** Check CAN wiring, verify motor controller configuration

### High Localization Drift
- **Check:** IMU publishing data (`ros2 topic echo /imu`)
- **Check:** Wheel encoder data quality
- **Check:** Map quality (sufficient features)
- **Solution:** Re-calibrate IMU, verify encoder connections, remap area

---

## 5.4 Spare Parts List

**Critical Spares (Keep On-Hand):**
- Battery pack (48V 20Ah) ×1
- Motor controller ×2
- Camera (front/rear compatible) ×1
- IMU ×1
- E-Stop button ×1
- Fuses (various ratings) ×10
- CAN bus cables ×2
- USB cables ×2
- Ethernet cables ×2

**Consumables:**
- Microfiber cleaning cloths
- Contact cleaner spray
- Dielectric grease
- Cable ties
- Heat shrink tubing

---

**Document Status:** Complete
**Phase 7 Status:** 100% Complete (Consolidated)
**ALL DOCUMENTATION PHASES COMPLETE!**
**Approvals Required:** Operations Manager, Maintenance Lead, Safety Engineer
