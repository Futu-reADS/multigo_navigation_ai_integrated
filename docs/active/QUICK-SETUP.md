# Quick Setup Guide

**Time Required:** 30 minutes
**Prerequisites:** Ubuntu 22.04, basic ROS 2 knowledge
**Last Updated:** December 4, 2025

**⚠️ Important:** This gets you running in simulation. For production deployment, see [IMPLEMENTATION-GUIDE.md](./IMPLEMENTATION-GUIDE.md) - the system needs bug fixes and safety features first!

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
- Expected! 0% coverage currently. See [IMPLEMENTATION-GUIDE.md Phase 2](./IMPLEMENTATION-GUIDE.md#phase-2-testing-infrastructure) for testing plan.

---

## Known Issues ⚠️

**This system has critical bugs and is NOT production-ready!**

See [ISSUES-AND-FIXES.md](./ISSUES-AND-FIXES.md) for details:
- 🔴 **CRIT-01:** PID integral bug (docking won't be accurate)
- 🔴 **CRIT-02:** Distance calculation bug (dual marker centering wrong)
- 🔴 **CRIT-03:** No LiDAR during docking (blind to obstacles - UNSAFE!)
- 🔴 **CRIT-04:** Zero test coverage (no regression protection)
- 🔴 **CRIT-05:** No emergency stop (SAFETY ISSUE!)

**Status:** 52% complete (47/91 requirements) - See [REQUIREMENTS.md](./REQUIREMENTS.md)

---

## Next Steps

**1. Understand the System (30 min):**
- Read: [START-HERE.md](./START-HERE.md) - Overview and navigation
- Check: [REQUIREMENTS.md](./REQUIREMENTS.md) - What the system should do (91 requirements)
- Review: [SYSTEM-ARCHITECTURE.md](./SYSTEM-ARCHITECTURE.md) - How it works

**2. See What's Missing (15 min):**
- Check: [REQUIREMENTS-TRACEABILITY.md](./REQUIREMENTS-TRACEABILITY.md) - Full requirement mapping
- Review: [ISSUES-AND-FIXES.md](./ISSUES-AND-FIXES.md) - 28 known issues

**3. Plan Implementation (1 hour):**
- Read: [IMPLEMENTATION-GUIDE.md](./IMPLEMENTATION-GUIDE.md) - 16-week plan to production
- Start with: [Phase 1 - Critical Bugs & Safety](./IMPLEMENTATION-GUIDE.md#phase-1-critical-bugs--safety)

**4. Get Coding:**
- Fix: [CRIT-01 PID Bug](./IMPLEMENTATION-GUIDE.md#task-11-fix-pid-integral-bug-4-hours) (4 hours)
- Fix: [CRIT-02 Distance Bug](./IMPLEMENTATION-GUIDE.md#task-12-fix-dual-marker-distance-calculation-1-hour) (1 hour)
- Then follow Phase 1 Week 2-4 for safety features

---

## Documentation Map

```
📂 docs/active/
├── START-HERE.md                  ← Start here for overview
├── QUICK-SETUP.md                 ← You are here (get it running)
├── REQUIREMENTS.md                ← What needs to be built (91 requirements)
├── REQUIREMENTS-TRACEABILITY.md   ← Requirement → Implementation mapping
├── IMPLEMENTATION-GUIDE.md        ← How to fix everything (16 weeks)
├── SYSTEM-ARCHITECTURE.md         ← How it works (current + proposed)
└── ISSUES-AND-FIXES.md            ← What's broken (28 issues)
```

---

**Last Updated:** December 4, 2025
**System Status:** 🟡 Beta (52% complete) - Safe for supervised testing only after Phase 1 fixes
