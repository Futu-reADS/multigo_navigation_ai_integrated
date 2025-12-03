# MultiGo System - Getting Started Guide

**Document Version:** 1.0
**Last Updated:** 2025-12-01
**Audience:** New users, operators, developers
**Reading Time:** 15 minutes

---

## Table of Contents

1. [Quick Start for Users](#quick-start-for-users)
2. [Quick Start for Developers](#quick-start-for-developers)
3. [Camera Calibration Guide](#camera-calibration-guide)
4. [First Docking Attempt](#first-docking-attempt)
5. [Common Issues & Solutions](#common-issues--solutions)
6. [Cheat Sheet](#cheat-sheet)

---

## Quick Start for Users

### Prerequisites

✅ **Hardware Ready:**
- MultiGo robot powered on
- Cameras connected and working
- LiDAR spinning (green light)
- Motors initialized (no error beeps)

✅ **Environment Setup:**
- ArUco markers (ID 20, 21) printed and mounted on wheelchair
- Clear path between robot and wheelchair
- Adequate lighting (bright enough to read a book comfortably)

✅ **Software Running:**
- All ROS 2 nodes started (see [Launch System](#step-3-launch-the-system))

---

### Step 1: Power On the System

**Power up sequence:**
```bash
# 1. Turn on main power
# 2. Wait for boot screen (Ubuntu 22.04 login)
# 3. Login to system
# 4. Check battery level (should be >30%)
```

**Visual indicators:**
- Green LED: System ready
- Blinking yellow: Initializing
- Red: Error (check logs)

---

### Step 2: Verify Markers

**Check markers are visible:**
1. Stand ~2 meters in front of wheelchair
2. Hold up phone camera - can you clearly see both markers?
3. If blurry or dark → improve lighting
4. Markers should be flat, not wrinkled or damaged

**Marker specs:**
- **Left marker:** ID 20
- **Right marker:** ID 21
- **Size:** ~15cm × 15cm (printed on letter/A4 paper)
- **Separation:** ~40cm apart (measure center to center)
- **Height:** Eye level for robot cameras (~1m from ground)

**Get markers:**
- Print files: `MultiGoArucoTest/markers/aruco_id20.png`, `aruco_id21.png`
- Print quality: 300 DPI minimum, high-quality printer
- Paper: Plain white paper, matte finish (avoid glossy)
- Mounting: Tape flat to rigid surface (cardboard backing OK)

---

### Step 3: Launch the System

**Open terminal and run:**

```bash
# Terminal 1: Hardware drivers
cd ~/multigo_ws
source install/setup.bash
ros2 launch boot boot.launch.py

# Wait for "All drivers initialized" message
```

```bash
# Terminal 2: Navigation & docking
cd ~/multigo_ws
source install/setup.bash
ros2 launch boot run.launch.py

# Wait for "Navigation stack ready" message
```

```bash
# Terminal 3: Master control (user interface)
cd ~/multigo_ws
source install/setup.bash
ros2 run nav_master nav_master_node

# You'll see: "MultiGo Master Control Ready"
```

**Startup time:** ~2 minutes total

---

### Step 4: Your First Transport

**Scenario:** Transport person from current location to 5 meters away

**Steps:**

1. **Position wheelchair with markers**
   - Place wheelchair 5 meters in front of robot
   - Markers clearly visible from robot
   - Brakes locked (wheelchair stable)

2. **Initiate approach**
   ```
   [Master Control displays]
   "Approach wheelchair? (y/n)"

   Type: y [ENTER]
   ```

3. **Watch robot navigate**
   - Robot moves toward wheelchair
   - Avoids any obstacles automatically
   - Stops ~30cm in front
   - Time: ~30 seconds

4. **Confirm docking**
   ```
   [Master Control displays]
   "Approach complete! Begin docking? (y/n)"

   Type: y [ENTER]
   ```

5. **Watch precision docking**
   - Robot makes slow, careful movements
   - Adjusts position (forward, sideways, rotating)
   - Final approach very slow
   - Time: ~25 seconds

   ```
   [Master Control displays]
   "Docking complete! Position verified."
   ```

6. **Load passenger** (Manual)
   - Help person into wheelchair
   - Secure wheelchair to robot platform
   - Double-check stability

7. **Transport**
   ```
   [Master Control displays]
   "Passenger loaded? Ready to transport? (y/n)"

   Type: y [ENTER]

   [Enter destination]
   "Where to? (room number or coordinates)"

   Type: Room 205 [ENTER]
   ```

   - Robot navigates smoothly
   - Very gentle turns
   - Stops automatically for obstacles

8. **Arrival and unloading**
   - Robot arrives at destination
   - Help person out of wheelchair
   - Unsecure from platform

   ```
   [Master Control displays]
   "Undock? (y/n)"

   Type: y [ENTER]
   ```

**Congratulations!** You've completed your first transport! 🎉

---

## Quick Start for Developers

### Prerequisites

✅ **Development Environment:**
- Ubuntu 22.04 LTS
- ROS 2 Humble installed
- ~10 GB free disk space
- Internet connection (for dependencies)

✅ **Skills:**
- Familiarity with ROS 2 concepts (nodes, topics, actions)
- C++ or Python programming
- Basic Linux terminal usage

---

### Step 1: Install ROS 2 Humble

```bash
# Follow official guide
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# Quick install (Ubuntu 22.04):
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### Step 2: Clone Repositories

```bash
# Create workspace
mkdir -p ~/multigo_ws/src
cd ~/multigo_ws/src

# Clone main navigation repository
git clone <multigo_navigation_url>
# Replace <multigo_navigation_url> with actual URL

# Import dependencies using .repos file
vcs import < multigo_navigation/multigo.repos

# Directory structure after import:
# ~/multigo_ws/src/
#   ├── multigo_navigation/    (main repo)
#   ├── multigo_launch/         (configuration & launch)
#   ├── multigo_master/         (master control)
#   └── MultiGoArucoTest/       (calibration tools)
```

**`.repos` file contents:**
```yaml
# multigo_navigation/multigo.repos
repositories:
  multigo_launch:
    type: git
    url: <multigo_launch_url>
    version: main
  multigo_master:
    type: git
    url: <multigo_master_url>
    version: main
  MultiGoArucoTest:
    type: git
    url: <MultiGoArucoTest_url>
    version: main
  third_party/HesaiLidar_ROS_2.0:
    type: git
    url: https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0
    version: humble
```

---

### Step 3: Install Dependencies

```bash
cd ~/multigo_ws

# Install ROS dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Install Nav2 (if not already installed)
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Install RTAB-Map
sudo apt install ros-humble-rtabmap-ros

# Install OpenCV (for ArUco detection)
sudo apt install python3-opencv
sudo apt install ros-humble-cv-bridge

# Install Phidgets (motor control)
sudo apt install libphidget22-dev
pip3 install Phidget22

# Install additional tools
sudo apt install ros-humble-tf-transformations
sudo apt install python3-transforms3d
```

---

### Step 4: Build the Workspace

```bash
cd ~/multigo_ws

# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Add to bashrc for convenience
echo "source ~/multigo_ws/install/setup.bash" >> ~/.bashrc
```

**Build time:** 5-10 minutes (first build)

**Common build errors:**
- Missing dependencies → Run `rosdep install` again
- Compilation errors → Check for C++17 support
- Python errors → Verify Python 3.10+ installed

---

### Step 5: Verify Installation

```bash
# Check package list
ros2 pkg list | grep multigo
# Expected output:
# nav_control
# nav_docking
# nav_goal
# aruco_detect
# camera_publisher
# ... (more packages)

# Check launch files
ros2 launch boot --show-args boot.launch.py
# Should display launch file arguments

# Run a simple test (without hardware)
ros2 run nav_master nav_master_node
# Should start without errors (will wait for hardware)
```

---

### Step 6: Run Simulation (Optional)

**Without hardware, test in Gazebo:**

```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch boot simulation.launch.py

# Terminal 2: Launch navigation stack
ros2 launch boot run.launch.py use_sim_time:=true

# Terminal 3: Send test goal
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
'{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}}}'
```

**Expected:** Robot navigates to (2, 0) in simulation

---

### Step 7: Code Exploration

**Start with these files:**

1. **Master control flow:**
   ```bash
   code ~/multigo_ws/src/multigo_master/nav_master/src/nav_master.cpp
   ```
   - Understand user confirmation workflow
   - See action client usage

2. **Docking algorithm:**
   ```bash
   code ~/multigo_ws/src/multigo_navigation/src/nav_docking/nav_docking.cpp
   ```
   - Study PID control implementation
   - **Note:** Contains known bugs (see Requirements Doc)

3. **Configuration:**
   ```bash
   code ~/multigo_ws/src/multigo_launch/launch/run.launch.py
   ```
   - See all parameters
   - Understand system integration

4. **ArUco detection:**
   ```bash
   code ~/multigo_ws/src/multigo_navigation/src/aruco_detect/aruco_detect.cpp
   ```
   - Study marker detection logic
   - See pose estimation

---

## Camera Calibration Guide

### Why Calibrate?

Camera lenses distort images (fisheye effect). Calibration computes correction parameters for accurate pose estimation.

**Without calibration:** ±5cm error at 1m distance
**With calibration:** ±5mm error at 1m distance (10× better!)

---

### Step 1: Print Chessboard Pattern

**Requirements:**
- Pattern: 9×6 internal corners (10×7 squares total)
- Square size: ~30mm (measure with ruler)
- Print quality: 300 DPI, high-quality printer
- Paper: A4 or Letter, matte white
- Mount: Tape to flat, rigid board (cardboard OK)

**Download pattern:**
```bash
# Pattern included in OpenCV, or generate:
python3 -c "import cv2; cv2.imwrite('chessboard.png', cv2.resize(cv2.imread(cv2.samples.findFile('chessboard.png')), (900, 1200)))"
```

**Or use online generator:** https://calib.io/pages/camera-calibration-pattern-generator

---

### Step 2: Run Calibration Script

```bash
cd ~/multigo_ws/src/MultiGoArucoTest/ArucoTest

# Run calibration tool
python3 CamCalibration.py

# Follow on-screen instructions:
# 1. Hold chessboard in front of camera
# 2. Press 's' to save image when corners detected (green overlay)
# 3. Move chessboard to different position/angle
# 4. Repeat until 20+ images captured
# 5. Press 'q' to quit and compute calibration
```

**Tips for good calibration:**
1. **Vary positions:** Top, bottom, left, right, center
2. **Vary angles:** Tilt forward, backward, sideways
3. **Vary distances:** Close (30cm), medium (1m), far (2m)
4. **Keep stable:** Hold steady when pressing 's'
5. **Good lighting:** Bright, even illumination

**Image sequence example:**
```
Image 1: Center, straight on
Image 2: Top-left corner, tilted left
Image 3: Top-right corner, tilted right
Image 4: Bottom-center, tilted down
Image 5: Close-up, center
... (15 more varied positions)
```

---

### Step 3: Verify Calibration

**Check output:**
```bash
# Calibration files created:
ls -lh calib.*

# Expected files:
# calib.pckl   (Python pickle format - intermediate)
# calib.yaml   (ROS-compatible format - final)

# Check reprojection error in console:
# "Reprojection error: 0.42 pixels"  ← Should be <1.0
```

**If error >1.0 pixel:**
- Recalibrate with more images
- Ensure chessboard is truly flat
- Check for good lighting (no glare)

---

### Step 4: Deploy Calibration

```bash
# Copy to camera config directory
cp calib.yaml ~/multigo_ws/src/multigo_launch/config/camera_calib_left.yaml

# For right camera, repeat calibration:
python3 CamCalibration.py --camera right
cp calib.yaml ~/multigo_ws/src/multigo_launch/config/camera_calib_right.yaml

# Rebuild to update config
cd ~/multigo_ws
colcon build --packages-select boot
source install/setup.bash
```

---

## First Docking Attempt

### Pre-Flight Checklist

Before attempting first dock:

✅ **System:**
- [ ] All nodes running (boot.launch.py + run.launch.py)
- [ ] Cameras calibrated (reprojection error <1.0)
- [ ] LiDAR operational (check `/scan` topic)
- [ ] Motors initialized (no error messages)

✅ **Environment:**
- [ ] Markers printed clearly (300 DPI)
- [ ] Markers mounted flat on wheelchair
- [ ] Markers separated by ~40cm
- [ ] Good lighting (can read text easily)
- [ ] Clear space (3m radius, no obstacles)

✅ **Safety:**
- [ ] Emergency stop accessible
- [ ] Human ready to intervene
- [ ] Wheelchair brakes locked
- [ ] No people in robot's path

---

### Docking Test Procedure

**Test 1: Close Range (1 meter)**

```bash
# Position robot 1 meter in front of markers
# Markers clearly visible

# Terminal: Master control
ros2 run nav_master nav_master_node

# When prompted:
Approach? → y
# Watch robot approach

Dock? → y
# Watch docking sequence

# Expected: Success within 30-40 seconds
```

**Observe:**
- ✅ Smooth velocity changes
- ✅ Centering behavior (Y-axis correction)
- ✅ Slowing down as it gets closer
- ✅ Final millimeter adjustments
- ❌ Jerky movements → PID tuning needed
- ❌ Drifting sideways → Calibration issue

---

**Test 2: Medium Range (3 meters)**

Same procedure, starting 3 meters away.

**Expected:**
- Approach time: ~15 seconds
- Docking time: ~25 seconds
- Total: ~40 seconds

---

**Test 3: Long Range (10 meters)**

Robot navigates through room to marker.

**Expected:**
- Navigation time: ~60 seconds
- Approach + dock: ~40 seconds
- Total: ~100 seconds

---

### Measuring Accuracy

**Setup:**
1. Mark robot's final position with tape
2. Undock robot
3. Re-dock 10 times
4. Measure position variation

**Tools:**
- Ruler (±1mm precision)
- Dial indicator (for ±0.1mm precision)
- Overhead camera (visual verification)

**Expected results (after bug fixes):**
- X-axis: ±1mm
- Y-axis: ±1mm
- Yaw: ±1°

**If accuracy worse:**
1. Check calibration quality
2. Verify marker flatness
3. Re-tune PID gains (see [Developer Guide](./02-DEVELOPER-GUIDE-ARCHITECTURE.md))

---

## Common Issues & Solutions

### Issue: "Cannot detect markers"

**Symptoms:**
- No markers detected
- `aruco_detect` node shows no output

**Solutions:**

1. **Check camera topics:**
   ```bash
   ros2 topic list | grep camera
   # Should see: /camera/color/image_raw_left, /camera/color/image_raw_right

   ros2 topic hz /camera/color/image_raw_left
   # Should see: ~30 Hz
   ```

2. **Check lighting:**
   - Too dark: Add more light
   - Too bright: Reduce glare, adjust camera exposure

3. **Check marker quality:**
   - Re-print at 300 DPI
   - Ensure flat, not wrinkled
   - Correct marker ID (20, 21)

4. **Check camera calibration:**
   ```bash
   # Verify calib.yaml exists
   ls ~/multigo_ws/src/multigo_launch/config/camera_calib_left.yaml
   ```

---

### Issue: "Robot doesn't move during navigation"

**Symptoms:**
- Nav2 goal sent, but robot stationary
- No velocity commands

**Solutions:**

1. **Check Nav2 status:**
   ```bash
   ros2 topic echo /cmd_vel
   # Should see velocity commands
   ```

2. **Check motor status:**
   ```bash
   # Look for Phidget errors in logs
   ros2 run mecanum_wheels phidgets_control.py
   ```

3. **Check costmaps:**
   ```bash
   ros2 run nav2_costmap_2d nav2_costmap_2d
   # Verify robot is not inside obstacle
   ```

4. **Check tf tree:**
   ```bash
   ros2 run tf2_tools view_frames
   # Verify all frames connected: map → odom → base_link → camera_left/right
   ```

---

### Issue: "Docking fails to complete"

**Symptoms:**
- Robot approaches but doesn't confirm docking
- Timeout or endless adjustment

**Solutions:**

1. **Check marker visibility:**
   ```bash
   ros2 topic echo /aruco_detect/markers_left
   # Should continuously output marker poses
   ```

2. **Check thresholds:**
   - `aruco_close_th`: 0.42m (is robot within this?)
   - Verify robot is actually reaching target distance

3. **Check for known bugs:**
   - See [Requirements Doc Section FR-4.1](./03-REQUIREMENTS-DOCUMENT.md#fr-41-single-marker-control-)
   - If bugs present, apply fixes first

4. **PID tuning:**
   - May need adjustment for your specific setup
   - See [Developer Guide](./02-DEVELOPER-GUIDE-ARCHITECTURE.md) for PID tuning

---

### Issue: "RTAB-Map error: cannot localize"

**Symptoms:**
- "Lost localization" warning
- Robot thinks it's in wrong position

**Solutions:**

1. **Ensure sufficient visual features:**
   - Blank walls → Add posters/markers
   - Poor lighting → Increase brightness

2. **Reset RTAB-Map database:**
   ```bash
   # Delete old map
   rm ~/.ros/rtabmap.db

   # Restart RTAB-Map
   ros2 launch boot run.launch.py
   ```

3. **Perform loop closure:**
   - Drive robot around
   - Return to starting position
   - RTAB-Map should recognize and correct drift

---

## Cheat Sheet

### Essential Commands

**Launch system:**
```bash
# Terminal 1: Hardware
ros2 launch boot boot.launch.py

# Terminal 2: Navigation
ros2 launch boot run.launch.py

# Terminal 3: Master control
ros2 run nav_master nav_master_node
```

**Check system status:**
```bash
# All nodes running?
ros2 node list

# Topics active?
ros2 topic list

# Camera working?
ros2 topic hz /camera/color/image_raw_left

# LiDAR working?
ros2 topic hz /scan

# Markers detected?
ros2 topic echo /aruco_detect/markers_left
```

**Debugging:**
```bash
# View logs
ros2 topic echo /rosout

# Specific node logs
ros2 run rqt_console rqt_console

# TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Visualize (RViz)
ros2 run rviz2 rviz2
```

**Restart specific node:**
```bash
# Kill node
ros2 lifecycle set /nav_docking_node shutdown

# Restart
ros2 run nav_docking nav_docking_node
```

---

### Key Parameters (Quick Reference)

**Marker IDs:**
- Left: 20
- Right: 21

**Docking Distances:**
- Approach offset: 0.305m
- Dual marker transition: 0.700m
- Docking complete: <0.42m

**Velocities:**
- Nav max: 0.26 m/s
- Docking max: 0.1 m/s

**Configuration Files:**
- Nav2: `multigo_launch/config/nav2_params.yaml`
- Docking: Parameters in `run.launch.py`

---

### Quick Diagnostics

**Problem: Nothing working**
```bash
# Check ROS 2 installation
ros2 --version  # Should be Humble

# Source workspace
source ~/multigo_ws/install/setup.bash

# Rebuild
cd ~/multigo_ws
colcon build
```

**Problem: Slow performance**
```bash
# Check CPU usage
htop

# Check topics lagging
ros2 topic hz /merged_cloud  # Should be ~10 Hz
ros2 topic hz /cmd_vel       # Should be ~5-10 Hz
```

**Problem: Compilation errors**
```bash
# Clean build
cd ~/multigo_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

---

## Next Steps

**For Users:**
1. Complete first successful docking (1 meter test)
2. Practice at different distances
3. Learn emergency procedures
4. Read: [User Guide](./01-SYSTEM-OVERVIEW-USER-GUIDE.md) for full details

**For Developers:**
1. Explore codebase (start with `nav_master.cpp`)
2. Run simulation tests
3. Review known bugs: [Requirements Doc](./03-REQUIREMENTS-DOCUMENT.md)
4. Consider contributing fixes!

**For System Integrators:**
1. Review architecture: [Developer Guide](./02-DEVELOPER-GUIDE-ARCHITECTURE.md)
2. Plan deployment (environment prep, marker placement)
3. Conduct risk assessment
4. Develop operating procedures

---

## Support & Resources

**Documentation:**
- [System Overview](./01-SYSTEM-OVERVIEW-USER-GUIDE.md) - User perspective
- [Developer Guide](./02-DEVELOPER-GUIDE-ARCHITECTURE.md) - Technical details
- [Requirements](./03-REQUIREMENTS-DOCUMENT.md) - System requirements & status

**External Resources:**
- ROS 2 Documentation: https://docs.ros.org/en/humble/
- Nav2 Documentation: https://navigation.ros.org/
- RTAB-Map Wiki: https://github.com/introlab/rtabmap_ros/wiki
- OpenCV ArUco: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html

**Community:**
- ROS Discourse: https://discourse.ros.org/
- Nav2 GitHub: https://github.com/ros-planning/navigation2

---

**Good luck with MultiGo! 🤖**

**Document maintained by:** MultiGo Development Team
**Feedback:** Report issues to project maintainers
**Last updated:** 2025-12-01
