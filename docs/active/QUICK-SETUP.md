# Quick Setup Guide

**Time Required:** 30 minutes
**Prerequisites:** Ubuntu 22.04, basic ROS 2 knowledge

---

## 1. Install ROS 2 Humble (15 min)

```bash
# Add ROS 2 apt repository
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup

# Install dependencies
sudo apt install python3-colcon-common-extensions python3-rosdep
```

---

## 2. Clone Repository (2 min)

```bash
mkdir -p ~/multigo_ws/src
cd ~/multigo_ws/src
git clone <repository_url> multigo_navigation_ai_integrated
cd ..
```

---

## 3. Install Dependencies (5 min)

```bash
cd ~/multigo_ws

# Initialize rosdep
sudo rosdep init  # Skip if already done
rosdep update

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
```

---

## 4. Build (5 min)

```bash
cd ~/multigo_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

---

## 5. Test in Simulation (3 min)

```bash
# Launch simulation
ros2 launch boot simulation.launch.py

# In another terminal:
source ~/multigo_ws/install/setup.bash

# Send test goal
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 1.0, y: 0.0, z: 0.0}}
}"
```

---

## Troubleshooting

**Build errors?**
- Check ROS 2 sourced: `echo $ROS_DISTRO` (should show "humble")
- Missing deps: `rosdep install --from-paths src --ignore-src -r -y`

**Simulation won't start?**
- Install Gazebo: `sudo apt install ros-humble-gazebo-ros-pkgs`

**Tests failing?**
- Expected! 0% coverage currently. See IMPLEMENTATION-GUIDE for Phase 2.

---

**Next Steps:**
- Read: [START-HERE.md](./START-HERE.md)
- Fix bugs: [IMPLEMENTATION-GUIDE.md - Phase 1](./IMPLEMENTATION-GUIDE.md#phase-1-critical-bugs--safety)
- Understand system: [SYSTEM-ARCHITECTURE.md](./SYSTEM-ARCHITECTURE.md)

**Last Updated:** 2025-12-02
