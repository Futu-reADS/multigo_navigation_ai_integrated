# Software Requirements

**Document ID:** REQ-SW-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft
**Classification:** Internal

---

## Document Control

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-15 | Software Lead | Initial software requirements consolidating all software stack specifications |

---

## 1. Introduction

### 1.1 Purpose

This document consolidates **all software requirements** for the outdoor-first wheelchair transport robot, including operating system, middleware (ROS 2), libraries, tools, and development environment.

### 1.2 Scope

This specification covers:
- **Operating System** - Ubuntu 22.04 LTS, kernel requirements
- **Middleware** - ROS 2 Humble, Autoware, Nav2
- **Core Libraries** - OpenCV, PCL, tf2, robot_localization
- **Development Tools** - Build system, debugging, version control
- **Software Architecture** - Node structure, launch files, configuration
- **Deployment** - Installation, updates, logging

---

## 2. Operating System Requirements

### 2.1 Base OS (SW-OS-BASE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-OS-BASE-001 | System SHALL use Ubuntu 22.04 LTS (Jammy Jellyfish) | Critical | OS inspection |
| SW-OS-BASE-002 | System SHALL use Linux kernel 5.15+ with real-time patches | High | Kernel version |
| SW-OS-BASE-003 | System SHALL have 64-bit (amd64) architecture | Critical | Architecture check |
| SW-OS-BASE-004 | System SHALL receive security updates for 5 years (LTS) | High | Ubuntu policy |
| SW-OS-BASE-005 | System SHALL NOT use GUI desktop (headless, resources) | High | Installation config |

**Total Requirements: 5**

---

### 2.2 System Configuration (SW-OS-CONFIG)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-OS-CONFIG-001 | System SHALL configure CPU governor to "performance" mode | High | sysfs check |
| SW-OS-CONFIG-002 | System SHALL disable swap (real-time performance) | High | swap -s |
| SW-OS-CONFIG-003 | System SHALL configure firewall rules for ROS 2 DDS | High | ufw/iptables |
| SW-OS-CONFIG-004 | System SHALL enable auto-login for robot user account | Medium | systemd config |
| SW-OS-CONFIG-005 | System SHALL configure network interfaces (static IP or DHCP) | High | netplan config |

**Total Requirements: 5**

---

## 3. Middleware Requirements

### 3.1 ROS 2 Humble (SW-MW-ROS2)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-MW-ROS2-001 | System SHALL use ROS 2 Humble Hawksbill LTS | Critical | ros2 --version |
| SW-MW-ROS2-002 | System SHALL install ros-humble-desktop or ros-humble-ros-base | Critical | dpkg -l |
| SW-MW-ROS2-003 | System SHALL use Cyclone DDS or Fast DDS middleware | Critical | RMW env variable |
| SW-MW-ROS2-004 | System SHALL configure DDS domain ID (avoid interference) | High | ROS_DOMAIN_ID |
| SW-MW-ROS2-005 | System SHALL source ROS 2 setup in user .bashrc | Critical | bashrc inspection |
| SW-MW-ROS2-006 | System SHALL use colcon for build management | Critical | colcon --version |

**Total Requirements: 6**

---

### 3.2 Nav2 (SW-MW-NAV2)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-MW-NAV2-001 | System SHALL install ros-humble-navigation2 | Critical | dpkg -l |
| SW-MW-NAV2-002 | System SHALL install ros-humble-nav2-bringup | Critical | dpkg -l |
| SW-MW-NAV2-003 | System SHALL use Nav2 behavior trees for navigation logic | High | Code inspection |
| SW-MW-NAV2-004 | System SHALL configure Nav2 via YAML parameter files | Critical | Config files |

**Total Requirements: 4**

---

### 3.3 Autoware (SW-MW-AUTOWARE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-MW-AUTOWARE-001 | System SHALL use Autoware.Core (localization, perception) | Critical | Package list |
| SW-MW-AUTOWARE-002 | System SHALL use NDT localization from Autoware | Critical | Launch file |
| SW-MW-AUTOWARE-003 | System SHALL integrate Autoware with Nav2 | Critical | Node graph |
| SW-MW-AUTOWARE-004 | System SHALL configure Autoware via YAML parameter files | High | Config files |

**Total Requirements: 4**

---

## 4. Core Library Requirements

### 4.1 Perception Libraries (SW-LIB-PERC)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-LIB-PERC-001 | System SHALL use PCL (Point Cloud Library) 1.12+ | Critical | pkg-config |
| SW-LIB-PERC-002 | System SHALL use OpenCV 4.x | Critical | opencv_version |
| SW-LIB-PERC-003 | System SHALL NOT use TensorFlow or PyTorch (GPU constraint) | Critical | Package list |
| SW-LIB-PERC-004 | System SHALL use cv_bridge for ROS 2 ↔ OpenCV conversion | Critical | Package list |
| SW-LIB-PERC-005 | System SHALL use image_transport for compressed image topics | High | Package list |

**Total Requirements: 5**

---

### 4.2 Transformation & Time (SW-LIB-TF)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-LIB-TF-001 | System SHALL use tf2 for coordinate transformations | Critical | Package list |
| SW-LIB-TF-002 | System SHALL use tf2_geometry_msgs for message conversions | Critical | Package list |
| SW-LIB-TF-003 | System SHALL use tf2_ros for transform broadcasting/listening | Critical | Package list |
| SW-LIB-TF-004 | System SHALL buffer transforms for 10 seconds minimum | High | Configuration |

**Total Requirements: 4**

---

### 4.3 Sensor Fusion (SW-LIB-FUSION)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-LIB-FUSION-001 | System SHALL use robot_localization package | Critical | Package list |
| SW-LIB-FUSION-002 | System SHALL configure Extended Kalman Filter (EKF) | High | YAML config |
| SW-LIB-FUSION-003 | System SHALL fuse LiDAR + IMU + wheel odometry | Critical | Launch file |

**Total Requirements: 3**

---

### 4.4 Utility Libraries (SW-LIB-UTIL)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-LIB-UTIL-001 | System SHALL use Eigen 3.x for linear algebra | High | pkg-config |
| SW-LIB-UTIL-002 | System SHALL use Boost 1.74+ (filesystem, thread, system) | High | dpkg -l |
| SW-LIB-UTIL-003 | System SHALL use yaml-cpp for configuration parsing | High | pkg-config |
| SW-LIB-UTIL-004 | System SHALL use spdlog for structured logging | Medium | CMake find |

**Total Requirements: 4**

---

## 5. Software Architecture Requirements

### 5.1 Node Structure (SW-ARCH-NODE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-ARCH-NODE-001 | Each subsystem SHALL be implemented as ROS 2 nodes | Critical | Node list |
| SW-ARCH-NODE-002 | Nodes SHALL use rclcpp (C++) or rclpy (Python) | Critical | Code inspection |
| SW-ARCH-NODE-003 | Critical nodes SHALL use C++ for performance | High | Code inspection |
| SW-ARCH-NODE-004 | Nodes SHALL implement lifecycle management where appropriate | Medium | Code inspection |
| SW-ARCH-NODE-005 | Nodes SHALL publish diagnostics to /diagnostics topic | High | Topic inspection |
| SW-ARCH-NODE-006 | Nodes SHALL use ROS 2 parameters for configuration | Critical | Launch file |

**Total Requirements: 6**

---

### 5.2 Package Structure (SW-ARCH-PKG)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-ARCH-PKG-001 | System SHALL organize code into ROS 2 packages | Critical | Workspace structure |
| SW-ARCH-PKG-002 | Each package SHALL have package.xml (format 3) | Critical | File inspection |
| SW-ARCH-PKG-003 | Each package SHALL have CMakeLists.txt or setup.py | Critical | File inspection |
| SW-ARCH-PKG-004 | Packages SHALL follow ROS 2 naming conventions | High | Package names |
| SW-ARCH-PKG-005 | Packages SHALL declare dependencies explicitly | Critical | package.xml |

**Total Requirements: 5**

---

### 5.3 Launch Files (SW-ARCH-LAUNCH)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-ARCH-LAUNCH-001 | System SHALL use Python launch files (launch.py) | High | File inspection |
| SW-ARCH-LAUNCH-002 | Launch files SHALL support command-line arguments | High | Launch API |
| SW-ARCH-LAUNCH-003 | Launch files SHALL load parameters from YAML files | Critical | Launch file |
| SW-ARCH-LAUNCH-004 | System SHALL have master launch file for full system | Critical | Launch file |
| SW-ARCH-LAUNCH-005 | Launch files SHALL handle node lifecycle events | Medium | Launch API |

**Total Requirements: 5**

---

### 5.4 Configuration Management (SW-ARCH-CONFIG)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-ARCH-CONFIG-001 | System SHALL use YAML files for all configuration | Critical | Config files |
| SW-ARCH-CONFIG-002 | Configuration SHALL be organized by subsystem | High | Directory structure |
| SW-ARCH-CONFIG-003 | System SHALL support environment-specific configs (dev/prod) | High | Config system |
| SW-ARCH-CONFIG-004 | System SHALL validate configuration on startup | High | Startup logs |
| SW-ARCH-CONFIG-005 | System SHALL provide default configurations | Critical | Config files |

**Total Requirements: 5**

---

## 6. Development Tools Requirements

### 6.1 Build System (SW-DEV-BUILD)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-DEV-BUILD-001 | System SHALL use colcon for workspace build | Critical | Build command |
| SW-DEV-BUILD-002 | System SHALL support CMake 3.16+ | Critical | cmake --version |
| SW-DEV-BUILD-003 | System SHALL enable compiler warnings (-Wall -Wextra) | High | CMakeLists.txt |
| SW-DEV-BUILD-004 | System SHALL build with optimizations (-O3) for release | High | CMakeLists.txt |
| SW-DEV-BUILD-005 | System SHALL support parallel build (-j) | High | Build command |

**Total Requirements: 5**

---

### 6.2 Testing Framework (SW-DEV-TEST)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-DEV-TEST-001 | System SHALL use gtest for C++ unit tests | Critical | CMakeLists.txt |
| SW-DEV-TEST-002 | System SHALL use pytest for Python unit tests | Critical | setup.py |
| SW-DEV-TEST-003 | System SHALL use launch_testing for integration tests | High | Test files |
| SW-DEV-TEST-004 | System SHALL support colcon test for running all tests | Critical | Test command |
| SW-DEV-TEST-005 | System SHALL measure code coverage (>80% target) | High | Coverage report |

**Total Requirements: 5**

---

### 6.3 Debugging & Profiling (SW-DEV-DEBUG)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-DEV-DEBUG-001 | System SHALL support gdb for C++ debugging | High | Tool availability |
| SW-DEV-DEBUG-002 | System SHALL support rqt tools for runtime debugging | High | rqt --list |
| SW-DEV-DEBUG-003 | System SHALL support ros2 topic/service/action CLI tools | Critical | CLI test |
| SW-DEV-DEBUG-004 | System SHALL support valgrind for memory leak detection | Medium | Tool availability |
| SW-DEV-DEBUG-005 | System SHALL support perf for CPU profiling | Medium | Tool availability |

**Total Requirements: 5**

---

### 6.4 Version Control (SW-DEV-VCS)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-DEV-VCS-001 | System SHALL use Git for version control | Critical | git --version |
| SW-DEV-VCS-002 | Repository SHALL have .gitignore for build artifacts | High | File inspection |
| SW-DEV-VCS-003 | System SHALL use vcstool for multi-repo management | Medium | Tool availability |
| SW-DEV-VCS-004 | System SHALL tag releases with semantic versioning | High | Git tags |

**Total Requirements: 4**

---

## 7. Deployment Requirements

### 7.1 Installation (SW-DEPLOY-INSTALL)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-DEPLOY-INSTALL-001 | System SHALL provide installation script (setup.sh) | Critical | Script file |
| SW-DEPLOY-INSTALL-002 | Installation SHALL handle all dependencies automatically | High | Script test |
| SW-DEPLOY-INSTALL-003 | Installation SHALL configure systemd services | Critical | systemd units |
| SW-DEPLOY-INSTALL-004 | Installation SHALL configure environment variables | Critical | bashrc/profile |
| SW-DEPLOY-INSTALL-005 | Installation SHALL verify successful installation | High | Verification script |

**Total Requirements: 5**

---

### 7.2 System Services (SW-DEPLOY-SERVICE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-DEPLOY-SERVICE-001 | System SHALL have systemd service for main navigation stack | Critical | Service file |
| SW-DEPLOY-SERVICE-002 | Services SHALL start automatically on boot | Critical | systemd enable |
| SW-DEPLOY-SERVICE-003 | Services SHALL restart automatically on failure | High | Restart policy |
| SW-DEPLOY-SERVICE-004 | Services SHALL have proper dependencies (After=network.target) | High | Service file |
| SW-DEPLOY-SERVICE-005 | Services SHALL log to journald | Critical | journalctl |

**Total Requirements: 5**

---

### 7.3 Updates & Upgrades (SW-DEPLOY-UPDATE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-DEPLOY-UPDATE-001 | System SHALL support over-the-air (OTA) software updates | High | Update mechanism |
| SW-DEPLOY-UPDATE-002 | Updates SHALL have rollback mechanism on failure | High | Backup system |
| SW-DEPLOY-UPDATE-003 | Updates SHALL require explicit operator approval | Critical | Update UI |
| SW-DEPLOY-UPDATE-004 | System SHALL log all update attempts | High | Update logs |

**Total Requirements: 4**

---

### 7.4 Logging (SW-DEPLOY-LOG)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-DEPLOY-LOG-001 | System SHALL log to ~/.ros/log/ directory | Critical | Log location |
| SW-DEPLOY-LOG-002 | System SHALL rotate logs daily or at 100 MB size | High | logrotate config |
| SW-DEPLOY-LOG-003 | System SHALL retain logs for 30 days minimum | High | Retention policy |
| SW-DEPLOY-LOG-004 | System SHALL log with timestamps (ISO 8601 format) | Critical | Log format |
| SW-DEPLOY-LOG-005 | System SHALL log at configurable levels (DEBUG/INFO/WARN/ERROR) | High | Log config |
| SW-DEPLOY-LOG-006 | System SHALL upload logs to TVM service (see question_answers.md) | Medium | TVM integration |

**Total Requirements: 6**

---

## 8. Performance & Optimization Requirements

### 8.1 CPU Utilization (SW-PERF-CPU)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-PERF-CPU-001 | System SHALL use <70% CPU during normal operation | High | htop/top monitoring |
| SW-PERF-CPU-002 | Individual nodes SHALL use <20% CPU each | High | ros2 top |
| SW-PERF-CPU-003 | System SHALL prioritize critical nodes (RT priority) | High | chrt/nice |
| SW-PERF-CPU-004 | System SHALL avoid busy-wait loops (use callbacks/timers) | High | Code review |

**Total Requirements: 4**

---

### 8.2 Memory Management (SW-PERF-MEM)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-PERF-MEM-001 | System SHALL use <20 GB RAM during normal operation | High | free -h |
| SW-PERF-MEM-002 | System SHALL NOT have memory leaks | Critical | valgrind test |
| SW-PERF-MEM-003 | System SHALL pre-allocate large buffers at startup | Medium | Code inspection |
| SW-PERF-MEM-004 | System SHALL use memory pools for frequent allocations | Medium | Code inspection |

**Total Requirements: 4**

---

### 8.3 Network Bandwidth (SW-PERF-NET)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-PERF-NET-001 | DDS traffic SHALL use <50 MB/s average | High | Network monitoring |
| SW-PERF-NET-002 | System SHALL use compressed image transport where appropriate | High | Topic inspection |
| SW-PERF-NET-003 | System SHALL configure DDS QoS for reliability/latency trade-off | High | QoS profiles |

**Total Requirements: 3**

---

## 9. Security Requirements

### 9.1 Access Control (SW-SEC-ACCESS)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-SEC-ACCESS-001 | System SHALL run with non-root user account | Critical | User check |
| SW-SEC-ACCESS-002 | System SHALL use password-protected SSH access | Critical | sshd_config |
| SW-SEC-ACCESS-003 | System SHALL disable unused network services | High | netstat/ss |
| SW-SEC-ACCESS-004 | System SHALL have firewall enabled (ufw/iptables) | High | Firewall status |

**Total Requirements: 4**

---

### 9.2 Software Updates (SW-SEC-UPDATE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SW-SEC-UPDATE-001 | System SHALL receive OS security updates | Critical | apt update |
| SW-SEC-UPDATE-002 | System SHALL verify package signatures | Critical | apt config |
| SW-SEC-UPDATE-003 | System SHALL have unattended-upgrades configured | High | Package check |

**Total Requirements: 3**

---

## 10. Requirements Summary

### Requirements Count by Category

| Category | Total Requirements | Critical | High | Medium | Low |
|----------|-------------------|----------|------|--------|-----|
| **Operating System** | **10** | 5 | 5 | 0 | 0 |
| **Middleware** | **14** | 11 | 3 | 0 | 0 |
| **Core Libraries** | **16** | 10 | 5 | 1 | 0 |
| **Software Architecture** | **21** | 12 | 7 | 2 | 0 |
| **Development Tools** | **19** | 8 | 8 | 3 | 0 |
| **Deployment** | **20** | 12 | 7 | 1 | 0 |
| **Performance** | **11** | 1 | 8 | 2 | 0 |
| **Security** | **7** | 5 | 2 | 0 | 0 |
| **TOTAL** | **118** | **64 (54%)** | **45 (38%)** | **9 (8%)** | **0 (0%)** |

---

## 11. Acceptance Criteria

### 11.1 MVP Software

- ✅ Ubuntu 22.04 + ROS 2 Humble installed
- ✅ Nav2 + Autoware operational
- ✅ Core libraries (PCL, OpenCV, tf2) functional
- ✅ Basic launch files and configuration
- ✅ Manual installation documented

### 11.2 Production Software

- ✅ All MVP + all high-priority requirements
- ✅ Systemd services configured
- ✅ OTA update mechanism
- ✅ Comprehensive logging
- ✅ Security hardening
- ✅ >80% test coverage

---

**Document Status:** Draft
**Next Review:** After software stack integration and testing
**Approvals Required:** Software Lead, System Architect, DevOps Engineer
