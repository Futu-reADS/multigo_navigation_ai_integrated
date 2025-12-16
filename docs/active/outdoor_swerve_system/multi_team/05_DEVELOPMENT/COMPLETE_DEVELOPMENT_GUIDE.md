# Complete Development Guide

**Document ID:** DEV-COMPLETE-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Comprehensive Reference

---

## Phase 5: Development Documentation Complete

This comprehensive guide consolidates all development documentation including environment setup, build instructions, coding standards, git workflow, and debugging techniques for the outdoor wheelchair transport robot system.

---

# Part 1: Development Environment Setup

## 1.1 System Requirements

**Hardware:**
- CPU: 4+ cores (8+ recommended)
- RAM: 16GB minimum (32GB recommended)
- Disk: 100GB free space (SSD recommended)
- Network: Internet connection for package downloads

**Software:**
- OS: Ubuntu 22.04 LTS (Jammy Jellyfish)
- ROS: ROS 2 Humble Hawksbill
- Build System: colcon
- Version Control: Git 2.34+

---

## 1.2 Ubuntu 22.04 Installation

```bash
# 1. Download Ubuntu 22.04 LTS Desktop
wget https://releases.ubuntu.com/22.04/ubuntu-22.04.3-desktop-amd64.iso

# 2. Create bootable USB (replace /dev/sdX with your USB device)
sudo dd if=ubuntu-22.04.3-desktop-amd64.iso of=/dev/sdX bs=4M status=progress

# 3. Boot from USB and follow installation wizard
# - Minimal installation (faster)
# - Download updates during installation
# - Install third-party software for graphics
```

**Post-Installation:**
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install essential tools
sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    terminator \
    htop
```

---

## 1.3 ROS 2 Humble Installation

```bash
# Set locale
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install -y ros-humble-desktop

# Install development tools
sudo apt install -y \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update
```

---

## 1.4 Project Dependencies

```bash
# Install ROS 2 packages
sudo apt install -y \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-rosbridge-suite \
    ros-humble-tf2-tools \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins

# Install perception libraries
sudo apt install -y \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    libpcl-dev \
    ros-humble-laser-geometry

# Install vision libraries
sudo apt install -y \
    ros-humble-vision-opencv \
    ros-humble-cv-bridge \
    libopencv-dev \
    python3-opencv

# Install Eigen and math libraries
sudo apt install -y \
    libeigen3-dev \
    libboost-all-dev

# Install hardware interface dependencies
sudo apt install -y \
    ros-humble-hardware-interface \
    ros-humble-controller-manager \
    ros-humble-ros2-control
```

---

## 1.5 Workspace Setup

```bash
# Create workspace
mkdir -p ~/wheelchair_ws/src
cd ~/wheelchair_ws

# Clone repository
cd src
git clone https://github.com/your-org/wheelchair-robot.git

# Install dependencies using rosdep
cd ~/wheelchair_ws
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
cd ~/wheelchair_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace
echo "source ~/wheelchair_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 1.6 IDE Configuration (VS Code)

```bash
# Install VS Code
sudo snap install code --classic

# Install recommended extensions
code --install-extension ms-vscode.cpptools
code --install-extension ms-python.python
code --install-extension ms-vscode.cmake-tools
code --install-extension twxs.cmake
code --install-extension ms-iot.vscode-ros

# Create workspace settings (.vscode/settings.json)
{
    "cmake.configureOnOpen": false,
    "python.autoComplete.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages"
    ],
    "python.analysis.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages"
    ],
    "C_Cpp.default.includePath": [
        "/opt/ros/humble/include/**",
        "${workspaceFolder}/src/**",
        "${workspaceFolder}/install/**"
    ],
    "C_Cpp.default.compilerPath": "/usr/bin/g++",
    "C_Cpp.default.cppStandard": "c++17"
}
```

---

# Part 2: Build Instructions

## 2.1 Building the Project

**Basic Build:**
```bash
cd ~/wheelchair_ws
colcon build
```

**Optimized Build (Release mode):**
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**Build Specific Packages:**
```bash
colcon build --packages-select swerve_drive_controller
```

**Build with Symlinks (for Python/Launch files):**
```bash
colcon build --symlink-install
```

**Parallel Build (faster):**
```bash
colcon build --parallel-workers 8
```

---

## 2.2 Package Structure

**ROS 2 Package Template:**
```
package_name/
├── package.xml              # Package manifest
├── CMakeLists.txt           # Build configuration
├── include/
│   └── package_name/
│       └── header.hpp       # C++ headers
├── src/
│   ├── node.cpp             # C++ implementation
│   └── __init__.py          # Python package (if using Python)
├── launch/
│   └── launch_file.py       # Launch files
├── config/
│   └── params.yaml          # Parameter files
├── test/
│   └── test_node.cpp        # Unit tests
└── README.md
```

---

## 2.3 CMakeLists.txt Template

```cmake
cmake_minimum_required(VERSION 3.8)
project(swerve_drive_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Add library
add_library(${PROJECT_NAME} SHARED
  src/swerve_drive_controller.cpp
  src/swerve_module.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  nav_msgs
  Eigen3
)

# Add executable
add_executable(swerve_node src/swerve_node.cpp)
target_link_libraries(swerve_node ${PROJECT_NAME})

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS swerve_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/test_swerve.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

ament_package()
```

---

## 2.4 package.xml Template

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>swerve_drive_controller</name>
  <version>1.0.0</version>
  <description>Swerve drive controller for wheelchair robot</description>

  <maintainer email="dev@example.com">Development Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>eigen</depend>

  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

# Part 3: Coding Standards

## 3.1 C++ Style Guide

**General Principles:**
- Follow [ROS 2 C++ Style Guide](https://docs.ros.org/en/humble/Contributing/Code-Style-Language-Versions.html)
- Use C++17 features
- Prefer `std::` over raw pointers
- Use `auto` for complex types
- RAII for resource management

**Naming Conventions:**
```cpp
// Classes: PascalCase
class SwerveDriveController {};

// Functions/Methods: camelCase
void computeVelocity();

// Variables: snake_case
double wheel_radius_;

// Constants: UPPER_SNAKE_CASE
const double MAX_VELOCITY = 2.0;

// Private members: trailing underscore
private:
  double velocity_;
  rclcpp::Node::SharedPtr node_;
```

**File Organization:**
```cpp
// header.hpp
#ifndef PACKAGE_NAME__HEADER_HPP_
#define PACKAGE_NAME__HEADER_HPP_

// System includes
#include <memory>
#include <string>

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Local includes
#include "package_name/other_header.hpp"

namespace package_name
{

class ClassName
{
public:
  ClassName();
  ~ClassName();

  void publicMethod();

private:
  void privateMethod();

  double member_variable_;
};

}  // namespace package_name

#endif  // PACKAGE_NAME__HEADER_HPP_
```

**Logging:**
```cpp
// Use ROS 2 logging (NOT std::cout)
RCLCPP_DEBUG(logger_, "Detailed debug information");
RCLCPP_INFO(logger_, "General information");
RCLCPP_WARN(logger_, "Warning: %s", warning_msg.c_str());
RCLCPP_ERROR(logger_, "Error: %d", error_code);
RCLCPP_FATAL(logger_, "Fatal error - shutting down");
```

---

## 3.2 Python Style Guide

**Follow PEP 8:**
```python
#!/usr/bin/env python3
"""
Module docstring describing the file purpose.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MyNode(Node):
    """Node class docstring."""

    def __init__(self):
        super().__init__('my_node')

        # Use snake_case for variables
        self.max_velocity = 2.0

        # Create publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

    def compute_velocity(self, target_pose):
        """
        Compute velocity command.

        Args:
            target_pose: Target pose as Pose message

        Returns:
            Twist message with velocity command
        """
        cmd_vel = Twist()
        # Implementation
        return cmd_vel
```

---

## 3.3 Documentation Standards

**Header Comments:**
```cpp
/**
 * @file swerve_drive_controller.hpp
 * @brief Swerve drive controller for omnidirectional motion
 * @author Development Team
 * @date 2025-12-15
 */
```

**Function Documentation:**
```cpp
/**
 * @brief Compute velocity commands from current pose and target
 *
 * @param current_pose Robot's current pose in map frame
 * @param target_pose Desired target pose
 * @return geometry_msgs::msg::Twist Velocity command
 *
 * @throws std::runtime_error if poses are in different frames
 */
geometry_msgs::msg::Twist computeVelocity(
    const geometry_msgs::msg::Pose& current_pose,
    const geometry_msgs::msg::Pose& target_pose);
```

---

# Part 4: Git Workflow

## 4.1 Branching Strategy

**Main Branches:**
- `main` - Production-ready code
- `develop` - Integration branch for features

**Feature Branches:**
- `feature/issue-123-swerve-controller`
- `feature/aruco-docking`
- `bugfix/navigation-crash`
- `hotfix/emergency-stop-fix`

**Branch Naming Convention:**
```
<type>/<issue-number>-<short-description>

Examples:
feature/42-add-docking-controller
bugfix/138-fix-ndt-crash
hotfix/emergency-stop-timing
```

---

## 4.2 Commit Message Format

**Format:**
```
<type>(<scope>): <subject>

<body>

<footer>
```

**Types:**
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code style changes (formatting, no logic change)
- `refactor`: Code refactoring
- `test`: Adding/updating tests
- `chore`: Build/tooling changes

**Examples:**
```
feat(swerve): Add module alignment wait logic

Implement state machine to wait for all swerve modules to align
before applying drive velocity. Prevents wheel slip during rapid
direction changes.

Closes #42

---

fix(navigation): Fix NDT localization crash on map load

NDT was crashing when map point cloud was empty. Added validation
check before setting target cloud.

Fixes #138

---

docs(readme): Update build instructions for ROS 2 Humble

Updated README with correct package names and build commands for
ROS 2 Humble. Previous instructions were for Foxy.
```

---

## 4.3 Pull Request Process

**1. Create Feature Branch:**
```bash
git checkout -b feature/42-swerve-controller
```

**2. Make Changes and Commit:**
```bash
git add src/swerve_drive_controller.cpp
git commit -m "feat(swerve): Add inverse kinematics solver"
```

**3. Push to Remote:**
```bash
git push origin feature/42-swerve-controller
```

**4. Create Pull Request:**
- Title: Clear description of changes
- Description: Link to issue, explain implementation
- Reviewers: Assign relevant team members

**5. Code Review Checklist:**
- [ ] Code follows style guide
- [ ] All tests pass (`colcon test`)
- [ ] New features have tests
- [ ] Documentation updated
- [ ] No compiler warnings
- [ ] Commit messages follow format

**6. Merge:**
- Squash commits if needed
- Delete feature branch after merge

---

## 4.4 Common Git Commands

```bash
# Clone repository
git clone https://github.com/your-org/wheelchair-robot.git

# Create and checkout new branch
git checkout -b feature/new-feature

# View status
git status

# Stage changes
git add file.cpp
git add .  # Add all changes

# Commit
git commit -m "feat: Add new feature"

# Push to remote
git push origin feature/new-feature

# Pull latest changes
git pull origin main

# Rebase on main
git fetch origin
git rebase origin/main

# Resolve merge conflicts
# Edit conflicted files, then:
git add file.cpp
git rebase --continue

# Stash changes
git stash
git stash pop

# View log
git log --oneline --graph --all
```

---

# Part 5: Debugging Guide

## 5.1 ROS 2 Command Line Tools

**Node Introspection:**
```bash
# List all nodes
ros2 node list

# Get node info
ros2 node info /swerve_controller

# List topics
ros2 topic list

# Echo topic data
ros2 topic echo /cmd_vel

# Show topic info (publishers/subscribers/QoS)
ros2 topic info /cmd_vel

# Publish test message
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'

# List services
ros2 service list

# Call service
ros2 service call /reset_emergency_stop std_srvs/srv/Trigger

# List parameters
ros2 param list

# Get parameter value
ros2 param get /swerve_controller wheel_base

# Set parameter
ros2 param set /swerve_controller max_velocity 1.0
```

---

## 5.2 TF2 Debugging

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Monitor specific transform
ros2 run tf2_ros tf2_echo map base_link

# Check for TF warnings
ros2 topic echo /tf_static
```

---

## 5.3 RQt Tools

```bash
# Launch rqt
rqt

# Useful plugins:
# - Topic Monitor: /plugins/Topics/Topic Monitor
# - TF Tree: /plugins/Visualization/TF Tree
# - Node Graph: /plugins/Introspection/Node Graph
# - Parameter Reconfigure: /plugins/Configuration/Dynamic Reconfigure
# - Plot: /plugins/Visualization/Plot
```

---

## 5.4 RViz Visualization

```bash
# Launch RViz
rviz2

# Useful displays:
# - RobotModel: Visualize URDF
# - TF: Show coordinate frames
# - LaserScan/PointCloud2: Visualize sensor data
# - Map: Show occupancy grid
# - Path: Show planned path
# - Marker: Visualize custom markers
```

---

## 5.5 GDB Debugging

**Debug Build:**
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

**Launch with GDB:**
```bash
ros2 run --prefix 'gdb -ex run --args' swerve_drive_controller swerve_node
```

**GDB Commands:**
```
(gdb) break swerve_drive_controller.cpp:42  # Set breakpoint
(gdb) run                                     # Start execution
(gdb) next                                    # Step over
(gdb) step                                    # Step into
(gdb) continue                                # Continue execution
(gdb) print variable_name                     # Print variable
(gdb) backtrace                               # Show call stack
```

---

## 5.6 Logging Best Practices

**Set Log Level:**
```bash
# Terminal-wide
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"

# Per-node
ros2 run swerve_drive_controller swerve_node --ros-args --log-level DEBUG

# In launch file
Node(
    package='swerve_drive_controller',
    executable='swerve_node',
    arguments=['--ros-args', '--log-level', 'DEBUG']
)
```

**Log Levels:**
- `DEBUG`: Detailed diagnostic information
- `INFO`: General information
- `WARN`: Warning messages
- `ERROR`: Error messages
- `FATAL`: Fatal errors (node shutdown)

---

## 5.7 Performance Profiling

**CPU Profiling with `perf`:**
```bash
# Install perf
sudo apt install linux-tools-generic

# Profile node
sudo perf record -g ros2 run package_name node_name

# View results
sudo perf report
```

**Memory Profiling with `valgrind`:**
```bash
# Install valgrind
sudo apt install valgrind

# Check for memory leaks
valgrind --leak-check=full ros2 run package_name node_name
```

---

## 5.8 Common Issues and Solutions

**Issue: "Could not find a package configuration file"**
```bash
# Solution: Source workspace
source ~/wheelchair_ws/install/setup.bash
```

**Issue: "No executable found"**
```bash
# Solution: Rebuild and install
colcon build --packages-select package_name
source install/setup.bash
```

**Issue: "Transform timeout"**
```bash
# Solution: Check TF tree
ros2 run tf2_tools view_frames
# Verify all required transforms are being published
```

**Issue: "Topic not receiving data"**
```bash
# Solution: Check QoS compatibility
ros2 topic info /topic_name --verbose
# Publisher and subscriber QoS must be compatible
```

---

**Document Status:** Complete
**Phase 5 Status:** 100% Complete (Consolidated)
**Approvals Required:** Development Lead, DevOps Lead
