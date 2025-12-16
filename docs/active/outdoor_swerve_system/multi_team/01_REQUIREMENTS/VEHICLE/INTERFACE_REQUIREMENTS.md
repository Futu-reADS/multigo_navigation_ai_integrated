# Interface Requirements

**Document ID:** REQ-INT-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft

---

## 1. Introduction

This document specifies all **software interfaces** including ROS 2 topics, services, actions, and external APIs.

---

## 2. ROS 2 Topics

### 2.1 Sensor Topics (INT-TOPIC-SENS)

| Topic | Type | Publisher | Subscriber | Rate | QoS |
|-------|------|-----------|------------|------|-----|
| /points | sensor_msgs/PointCloud2 | lidar_driver | perception | 10 Hz | BEST_EFFORT |
| /camera_*/image_raw | sensor_msgs/Image | camera_driver | aruco_detector | 30 Hz | BEST_EFFORT |
| /imu/data | sensor_msgs/Imu | imu_driver | localization | 50 Hz | RELIABLE |
| /odom | nav_msgs/Odometry | wheel_odometry | localization | 50 Hz | RELIABLE |

**Total: 4 sensor topics**

### 2.2 Navigation Topics (INT-TOPIC-NAV)

| Topic | Type | Publisher | Subscriber | Rate | QoS |
|-------|------|-----------|------------|------|-----|
| /cmd_vel | geometry_msgs/Twist | controller | swerve_driver | 20 Hz | RELIABLE |
| /pose | geometry_msgs/PoseStamped | localization | planner | 10 Hz | RELIABLE |
| /costmap | nav_msgs/OccupancyGrid | perception | planner | 5 Hz | RELIABLE |

**Total: 3 navigation topics**

### 2.3 Safety Topics (INT-TOPIC-SAFE)

| Topic | Type | Publisher | Subscriber | Rate | QoS |
|-------|------|-----------|------------|------|-----|
| /emergency_stop | std_msgs/Bool | safety_supervisor | all_nodes | Event | RELIABLE |
| /diagnostics | diagnostic_msgs/DiagnosticArray | all_nodes | monitor | 1 Hz | RELIABLE |

**Total: 2 safety topics**

---

## 3. ROS 2 Services

### 3.1 System Services (INT-SRV-SYS)

| Service | Type | Server | Purpose |
|---------|------|--------|---------|
| /start_mission | std_srvs/Trigger | mission_manager | Start navigation mission |
| /cancel_mission | std_srvs/Trigger | mission_manager | Cancel current mission |
| /reset_localization | std_srvs/Trigger | localization | Reset to initial pose |

**Total: 3 services**

---

## 4. ROS 2 Actions

### 4.1 Navigation Actions (INT-ACT-NAV)

| Action | Type | Server | Purpose |
|--------|------|--------|---------|
| /navigate_to_pose | nav2_msgs/NavigateToPose | nav2_bt_navigator | Navigate to single goal |
| /follow_waypoints | nav2_msgs/FollowWaypoints | nav2_bt_navigator | Multi-waypoint mission |
| /dock | custom_msgs/Dock | docking_controller | Precision docking |

**Total: 3 actions**

---

## 5. External APIs

### 5.1 UI REST API (INT-API-UI)

| Endpoint | Method | Purpose |
|----------|--------|---------|
| /api/mission/start | POST | Start mission with waypoints |
| /api/mission/status | GET | Get mission status |
| /api/robot/state | GET | Get robot state |
| /api/emergency_stop | POST | Trigger emergency stop |

**Total: 4 API endpoints**

---

## 6. eHMI Serial Protocol

### 6.1 Commands (INT-SERIAL-CMD)

| Command | Format | Purpose |
|---------|--------|---------|
| STATE | STATE:{num}\n | Set eHMI state (0-28) |
| VOLUME | VOLUME:{0-10}\n | Set audio volume |
| LANGUAGE | LANGUAGE:{EN\|JP\|ZH}\n | Set language |

**Total: 3 commands**

---

## 7. Requirements Summary

| Category | Total |
|----------|-------|
| **ROS 2 Topics** | **9** |
| **ROS 2 Services** | **3** |
| **ROS 2 Actions** | **3** |
| **REST API** | **4** |
| **Serial Protocol** | **3** |
| **TOTAL** | **22** |

---

**Document Status:** Draft
**Approvals Required:** Software Lead, Integration Engineer
