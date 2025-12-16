# Docking System Requirements

**Document ID:** REQ-DOCK-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft
**Classification:** Internal

---

## Document Control

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-15 | System Architect | Initial requirements specification |

---

## 1. Introduction

### 1.1 Purpose

This document specifies the requirements for the **Autonomous Docking Subsystem** of the outdoor-first wheelchair transport robot. The docking system enables precision approach and alignment with wheelchair docking stations using ArUco visual markers and visual servoing control.

### 1.2 Scope

This specification covers:
- **ArUco marker detection** using dual RGB cameras
- **Visual servoing control** for precision alignment
- **Two-phase docking approach** (coarse navigation + fine servoing)
- **Hardware requirements** (cameras, markers, lighting)
- **Software architecture** (ROS 2 nodes, controllers, state machines)
- **Safety and reliability** requirements
- **Environmental operating conditions**

**Out of Scope:**
- Nav2 global path planning (covered in NAVIGATION_REQUIREMENTS.md)
- Physical docking mechanism (mechanical latches, sensors)
- Wheelchair detection sensors (covered in PERCEPTION_REQUIREMENTS.md)

### 1.3 Related Documents

- **SYSTEM_REQUIREMENTS.md** - Section 1.1 (DOCK-REQ-001 to 010)
- **OVERALL_SYSTEM_ARCHITECTURE.md** - Section 3.2 (Docking Subsystem)
- **VISUAL_SERVOING_DESIGN.md** - Detailed design
- **EHMI_SYSTEM_REFERENCE.md** - States 21-28 (docking states)
- **question_answers.md** - Docking strategy clarifications

### 1.4 System Overview

The docking system implements a **two-phase approach**:

**Phase 1: Coarse Navigation (Nav2)**
```
Robot position (anywhere)
    ↓
Navigate to "Docking Approach Zone" (Nav2 waypoint)
    ↓
Stop at approach zone (±10cm accuracy)
    ↓
ArUco markers now in camera range (0.5-3m)
```

**Phase 2: Fine Visual Servoing**
```
Switch to visual servoing mode
    ↓
Detect ArUco markers (dual camera)
    ↓
Compute pose error (X, Y, Yaw)
    ↓
PID control → velocity commands (vx, vy, ω)
    ↓
Precision docking (±2-5mm outdoor, ±1mm indoor)
```

---

## 2. Functional Requirements

### 2.1 ArUco Marker Detection (DOCK-ARUCO)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-ARUCO-001 | System SHALL detect ArUco markers from DICT_4X4_50 dictionary | Critical | Unit test |
| DOCK-ARUCO-002 | System SHALL detect markers at 10 Hz minimum during docking | Critical | Performance test |
| DOCK-ARUCO-003 | System SHALL detect markers in range 0.2m to 5.0m | Critical | Field test |
| DOCK-ARUCO-004 | System SHALL detect markers at angles ±45° horizontal, ±30° vertical | High | Field test |
| DOCK-ARUCO-005 | System SHALL compute 6-DOF pose (X, Y, Z, roll, pitch, yaw) from marker | Critical | Unit test |
| DOCK-ARUCO-006 | System SHALL use dual camera fusion when both cameras detect same marker | High | Integration test |
| DOCK-ARUCO-007 | System SHALL select closest marker when multiple markers detected | High | Integration test |
| DOCK-ARUCO-008 | System SHALL reject detections with reprojection error >2 pixels | Medium | Unit test |
| DOCK-ARUCO-009 | System SHALL publish marker poses to `/aruco_detections` topic | Critical | Integration test |
| DOCK-ARUCO-010 | System SHALL publish TF2 transforms `marker_X -> camera_link` | Critical | Integration test |
| DOCK-ARUCO-011 | System SHALL handle partial marker occlusion (≥3 corners visible) | Medium | Field test |
| DOCK-ARUCO-012 | System SHALL reject markers with ID outside expected range [0-49] | Low | Unit test |

**Total Requirements: 12**

---

### 2.2 Visual Servoing Control (DOCK-VS)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-VS-001 | System SHALL compute pose error (ΔX, ΔY, Δθ) from marker pose | Critical | Unit test |
| DOCK-VS-002 | System SHALL use PID control for X, Y, Yaw axes independently | Critical | Unit test |
| DOCK-VS-003 | System SHALL limit maximum docking velocity to 0.3 m/s linear | Critical | Safety test |
| DOCK-VS-004 | System SHALL limit maximum docking angular velocity to 0.2 rad/s | Critical | Safety test |
| DOCK-VS-005 | System SHALL achieve ±2-5mm position accuracy outdoor | Critical | Field test |
| DOCK-VS-006 | System SHALL achieve ±0.5° orientation accuracy | High | Field test |
| DOCK-VS-007 | System SHALL declare "docked" when error <5mm for 2 seconds | Critical | Integration test |
| DOCK-VS-008 | System SHALL publish velocity commands to `/cmd_vel` topic | Critical | Integration test |
| DOCK-VS-009 | System SHALL implement anti-windup for PID integrators | High | Unit test |
| DOCK-VS-010 | System SHALL smooth velocity commands with 0.5s ramp-up/down | Medium | Integration test |
| DOCK-VS-011 | System SHALL handle marker loss by stopping and re-searching | High | Fault injection test |
| DOCK-VS-012 | System SHALL timeout after 120 seconds if docking not achieved | High | Integration test |
| DOCK-VS-013 | System SHALL support "approach only" mode (stop at 0.5m) | Medium | Integration test |

**Total Requirements: 13**

---

### 2.3 Docking State Machine (DOCK-SM)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-SM-001 | System SHALL implement state machine: IDLE → APPROACH → SEARCH → SERVOING → DOCKED → UNDOCKING | Critical | Integration test |
| DOCK-SM-002 | System SHALL transition IDLE → APPROACH on docking goal received | Critical | Integration test |
| DOCK-SM-003 | System SHALL use Nav2 in APPROACH state to reach docking zone | Critical | Integration test |
| DOCK-SM-004 | System SHALL transition APPROACH → SEARCH on approach zone reached | Critical | Integration test |
| DOCK-SM-005 | System SHALL rotate 360° in SEARCH state if no marker detected | High | Integration test |
| DOCK-SM-006 | System SHALL transition SEARCH → SERVOING on marker detected | Critical | Integration test |
| DOCK-SM-007 | System SHALL transition SERVOING → DOCKED on success criteria met | Critical | Integration test |
| DOCK-SM-008 | System SHALL transition to IDLE on docking failure (timeout) | High | Fault injection test |
| DOCK-SM-009 | System SHALL support UNDOCKING state for reverse motion | High | Integration test |
| DOCK-SM-010 | System SHALL publish current state to `/docking/state` topic | Medium | Integration test |
| DOCK-SM-011 | System SHALL update eHMI with states 21-28 during docking | High | System test |

**Total Requirements: 11**

---

### 2.4 Dual Camera Fusion (DOCK-CAM)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-CAM-001 | System SHALL use 2× RGB cameras (front-left, front-right) | Critical | Hardware inspection |
| DOCK-CAM-002 | System SHALL fuse detections when same marker visible in both cameras | High | Integration test |
| DOCK-CAM-003 | System SHALL use single camera when only one detects marker | High | Integration test |
| DOCK-CAM-004 | System SHALL prefer stereo fusion over monocular (higher accuracy) | Medium | Performance test |
| DOCK-CAM-005 | System SHALL compute weighted average when fusing dual detections | Medium | Unit test |
| DOCK-CAM-006 | System SHALL calibrate camera intrinsics (K matrix, distortion) | Critical | Calibration procedure |
| DOCK-CAM-007 | System SHALL calibrate camera extrinsics (camera → base_link TF) | Critical | Calibration procedure |
| DOCK-CAM-008 | System SHALL maintain camera calibration accuracy <1 pixel RMS | High | Calibration test |

**Total Requirements: 8**

---

### 2.5 Marker Management (DOCK-MARKER)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-MARKER-001 | System SHALL support 4-8 ArUco markers at docking stations | Critical | Configuration |
| DOCK-MARKER-002 | System SHALL use marker size 150mm × 150mm (0.15m) | Critical | Hardware inspection |
| DOCK-MARKER-003 | System SHALL use DICT_4X4_50 dictionary (50 unique markers) | Critical | Configuration |
| DOCK-MARKER-004 | System SHALL assign unique IDs per docking station (e.g., 0-1, 2-3, ...) | High | Configuration |
| DOCK-MARKER-005 | System SHALL use dual markers per station for centering (left + right) | High | Field test |
| DOCK-MARKER-006 | System SHALL target centerpoint between dual markers | High | Integration test |
| DOCK-MARKER-007 | System SHALL store marker poses in map frame (persistent) | Medium | Integration test |
| DOCK-MARKER-008 | System SHALL detect marker damage/degradation (low corner detection rate) | Low | Fault detection test |

**Total Requirements: 8**

---

## 3. Performance Requirements

### 3.1 Docking Precision (DOCK-PERF-PREC)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-PERF-PREC-001 | System SHALL achieve ±2mm position accuracy indoor | High | Field test (100 trials) |
| DOCK-PERF-PREC-002 | System SHALL achieve ±5mm position accuracy outdoor | Critical | Field test (100 trials) |
| DOCK-PERF-PREC-003 | System SHALL achieve ±0.5° orientation accuracy | High | Field test |
| DOCK-PERF-PREC-004 | System SHALL achieve 95th percentile accuracy ±10mm outdoor | High | Statistical analysis |
| DOCK-PERF-PREC-005 | System SHALL maintain accuracy in 10-25 km/h wind | High | Wind tunnel test |
| DOCK-PERF-PREC-006 | System SHALL maintain accuracy on ±5° slope | High | Slope test |

**Total Requirements: 6**

---

### 3.2 Docking Speed & Success Rate (DOCK-PERF-SPEED)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-PERF-SPEED-001 | System SHALL complete docking in <120 seconds from approach zone | High | Field test |
| DOCK-PERF-SPEED-002 | System SHALL achieve >95% docking success rate outdoor | Critical | Field test (1000 trials) |
| DOCK-PERF-SPEED-003 | System SHALL achieve >98% docking success rate indoor | High | Field test (500 trials) |
| DOCK-PERF-SPEED-004 | System SHALL detect marker within 5 seconds of entering range | High | Performance test |
| DOCK-PERF-SPEED-005 | System SHALL converge to ±10mm within 60 seconds | High | Performance test |

**Total Requirements: 5**

---

### 3.3 Detection Performance (DOCK-PERF-DET)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-PERF-DET-001 | System SHALL detect markers at 10 Hz minimum | Critical | Performance test |
| DOCK-PERF-DET-002 | System SHALL achieve <100ms detection latency (camera → pose) | High | Latency test |
| DOCK-PERF-DET-003 | System SHALL achieve >99% detection rate at 0.5-3m range | Critical | Field test |
| DOCK-PERF-DET-004 | System SHALL achieve >90% detection rate at 3-5m range | High | Field test |
| DOCK-PERF-DET-005 | System SHALL handle 30 FPS camera framerate minimum | Critical | Hardware spec |

**Total Requirements: 5**

---

## 4. Hardware Requirements

### 4.1 Camera Hardware (DOCK-HW-CAM)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-HW-CAM-001 | System SHALL use 2× RGB cameras (stereo or independent) | Critical | Hardware BOM |
| DOCK-HW-CAM-002 | Cameras SHALL provide 1920×1080 resolution minimum | Critical | Datasheet |
| DOCK-HW-CAM-003 | Cameras SHALL provide 30 FPS framerate minimum | Critical | Datasheet |
| DOCK-HW-CAM-004 | Cameras SHALL have 90° HFOV minimum | High | Datasheet |
| DOCK-HW-CAM-005 | Cameras SHALL support USB 3.0 or GigE interface | High | Datasheet |
| DOCK-HW-CAM-006 | Cameras SHALL have global shutter (preferred) or <10ms rolling shutter | Medium | Datasheet |
| DOCK-HW-CAM-007 | Cameras SHALL be mounted at 0.4-0.6m height from ground | High | Mechanical design |
| DOCK-HW-CAM-008 | Cameras SHALL be tilted 10-15° downward | Medium | Mechanical design |
| DOCK-HW-CAM-009 | Cameras SHALL have IP66+ weatherproofing or housing | Critical | Datasheet |

**Total Requirements: 9**

---

### 4.2 ArUco Marker Hardware (DOCK-HW-MARKER)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-HW-MARKER-001 | Markers SHALL be 150mm × 150mm (±2mm) | Critical | Manufacturing spec |
| DOCK-HW-MARKER-002 | Markers SHALL use DICT_4X4_50 dictionary | Critical | Generation spec |
| DOCK-HW-MARKER-003 | Markers SHALL be printed on rigid substrate (aluminum, acrylic) | High | Material spec |
| DOCK-HW-MARKER-004 | Markers SHALL have matte finish (non-reflective) | High | Manufacturing spec |
| DOCK-HW-MARKER-005 | Markers SHALL be UV-resistant (outdoor durability) | High | Material spec |
| DOCK-HW-MARKER-006 | Markers SHALL have IP66+ weatherproofing (laminate or coating) | Critical | Material spec |
| DOCK-HW-MARKER-007 | Markers SHALL be mounted at 0.3-0.5m height from ground | High | Installation spec |
| DOCK-HW-MARKER-008 | Markers SHALL be mounted perpendicular to approach direction (±5°) | High | Installation spec |

**Total Requirements: 8**

---

### 4.3 Lighting Hardware (DOCK-HW-LIGHT)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-HW-LIGHT-001 | System SHALL have active LED illumination for night docking | High | Hardware BOM |
| DOCK-HW-LIGHT-002 | LED lights SHALL provide 500-1000 lux at 2m distance | High | Photometer test |
| DOCK-HW-LIGHT-003 | LED lights SHALL be co-located with cameras (minimize shadows) | Medium | Mechanical design |
| DOCK-HW-LIGHT-004 | LED lights SHALL activate automatically in low light (<500 lux ambient) | Medium | Integration test |

**Total Requirements: 4**

---

## 5. Software Requirements

### 5.1 ROS 2 Nodes (DOCK-SW-NODE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-SW-NODE-001 | System SHALL implement `aruco_detector_node` (C++/Python) | Critical | Code inspection |
| DOCK-SW-NODE-002 | System SHALL implement `visual_servoing_controller_node` (C++) | Critical | Code inspection |
| DOCK-SW-NODE-003 | System SHALL implement `docking_state_machine_node` (Python) | Critical | Code inspection |
| DOCK-SW-NODE-004 | Nodes SHALL run on ROS 2 Humble | Critical | CI/CD test |
| DOCK-SW-NODE-005 | Nodes SHALL use rclcpp/rclpy lifecycle management | Medium | Code inspection |
| DOCK-SW-NODE-006 | Nodes SHALL publish diagnostics to `/diagnostics` topic | Medium | Integration test |

**Total Requirements: 6**

---

### 5.2 ROS 2 Topics & Services (DOCK-SW-COMM)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-SW-COMM-001 | System SHALL subscribe to `/camera_front_left/image_raw` | Critical | Topic inspection |
| DOCK-SW-COMM-002 | System SHALL subscribe to `/camera_front_right/image_raw` | Critical | Topic inspection |
| DOCK-SW-COMM-003 | System SHALL publish to `/aruco_detections` (geometry_msgs/PoseArray) | Critical | Topic inspection |
| DOCK-SW-COMM-004 | System SHALL publish to `/cmd_vel` during visual servoing | Critical | Topic inspection |
| DOCK-SW-COMM-005 | System SHALL publish to `/docking/state` (std_msgs/String) | Medium | Topic inspection |
| DOCK-SW-COMM-006 | System SHALL provide service `/docking/start` (std_srvs/Trigger) | High | Service inspection |
| DOCK-SW-COMM-007 | System SHALL provide service `/docking/cancel` (std_srvs/Trigger) | High | Service inspection |
| DOCK-SW-COMM-008 | System SHALL provide action `/docking/dock` (custom DockAction) | High | Action inspection |

**Total Requirements: 8**

---

### 5.3 Dependencies & Libraries (DOCK-SW-LIB)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-SW-LIB-001 | System SHALL use OpenCV 4.x for ArUco detection | Critical | Build system |
| DOCK-SW-LIB-002 | System SHALL use cv_bridge for ROS 2 ↔ OpenCV conversion | Critical | Build system |
| DOCK-SW-LIB-003 | System SHALL use image_transport for camera image topics | High | Build system |
| DOCK-SW-LIB-004 | System SHALL use tf2_ros for coordinate transformations | Critical | Build system |
| DOCK-SW-LIB-005 | System SHALL use camera_info_manager for calibration | High | Build system |

**Total Requirements: 5**

---

### 5.4 Configuration Management (DOCK-SW-CFG)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-SW-CFG-001 | System SHALL load parameters from YAML config files | Critical | Integration test |
| DOCK-SW-CFG-002 | System SHALL support dynamic reconfiguration of PID gains | High | Runtime test |
| DOCK-SW-CFG-003 | System SHALL validate marker dictionary on startup | High | Unit test |
| DOCK-SW-CFG-004 | System SHALL validate camera calibration files on startup | Critical | Unit test |
| DOCK-SW-CFG-005 | System SHALL log configuration parameters at startup | Medium | Log inspection |

**Total Requirements: 5**

---

## 6. Safety Requirements

### 6.1 Collision Avoidance (DOCK-SAFE-COLL)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-SAFE-COLL-001 | System SHALL monitor LiDAR during docking for obstacles | Critical | Safety test |
| DOCK-SAFE-COLL-002 | System SHALL stop if obstacle detected <0.5m during docking | Critical | Fault injection test |
| DOCK-SAFE-COLL-003 | System SHALL limit docking velocity to 0.1 m/s when <1m from target | Critical | Safety test |
| DOCK-SAFE-COLL-004 | System SHALL abort docking if unexpected obstacle in path | High | Fault injection test |

**Total Requirements: 4**

---

### 6.2 Emergency Stop (DOCK-SAFE-ESTOP)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-SAFE-ESTOP-001 | System SHALL stop immediately on emergency stop signal | Critical | Safety test |
| DOCK-SAFE-ESTOP-002 | System SHALL disable visual servoing on emergency stop | Critical | Integration test |
| DOCK-SAFE-ESTOP-003 | System SHALL require manual reset after emergency stop | High | Integration test |

**Total Requirements: 3**

---

### 6.3 Fault Handling (DOCK-SAFE-FAULT)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-SAFE-FAULT-001 | System SHALL abort docking if marker lost for >5 seconds | High | Fault injection test |
| DOCK-SAFE-FAULT-002 | System SHALL abort docking if camera failure detected | Critical | Fault injection test |
| DOCK-SAFE-FAULT-003 | System SHALL log all docking failures to persistent storage | High | Log inspection |
| DOCK-SAFE-FAULT-004 | System SHALL publish failure reason on `/docking/status` topic | Medium | Integration test |
| DOCK-SAFE-FAULT-005 | System SHALL attempt retry (max 3 attempts) before declaring failure | Medium | Integration test |

**Total Requirements: 5**

---

### 6.4 Wheelchair Safety (DOCK-SAFE-WHEELCHAIR)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-SAFE-WHEELCHAIR-001 | System SHALL verify wheelchair presence before docking | Critical | Sensor integration test |
| DOCK-SAFE-WHEELCHAIR-002 | System SHALL detect wheelchair misalignment (>10cm offset) | High | Fault detection test |
| DOCK-SAFE-WHEELCHAIR-003 | System SHALL abort if wheelchair moves during docking | High | Fault injection test |

**Total Requirements: 3**

---

## 7. Environmental Requirements

### 7.1 Lighting Conditions (DOCK-ENV-LIGHT)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-ENV-LIGHT-001 | System SHALL operate in 100-100,000 lux (overcast to direct sun) | Critical | Field test |
| DOCK-ENV-LIGHT-002 | System SHALL operate at night (<10 lux) with active LED lighting | High | Night test |
| DOCK-ENV-LIGHT-003 | System SHALL handle direct sunlight glare (auto-exposure) | High | Field test |
| DOCK-ENV-LIGHT-004 | System SHALL handle backlit markers (marker darker than background) | Medium | Field test |

**Total Requirements: 4**

---

### 7.2 Weather Conditions (DOCK-ENV-WEATHER)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-ENV-WEATHER-001 | System SHALL operate in light rain (<2.5mm/hr) | High | Rain chamber test |
| DOCK-ENV-WEATHER-002 | System SHALL detect marker with water droplets on surface | High | Rain simulation |
| DOCK-ENV-WEATHER-003 | System SHALL handle condensation on camera lenses (heating/defogging) | Medium | Climate chamber |
| DOCK-ENV-WEATHER-004 | System SHALL reject operation in heavy rain (>7.5mm/hr) | Medium | Rain sensor |

**Total Requirements: 4**

---

### 7.3 Temperature & Durability (DOCK-ENV-TEMP)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-ENV-TEMP-001 | System SHALL operate in -10°C to +45°C ambient temperature | High | Climate chamber |
| DOCK-ENV-TEMP-002 | Cameras SHALL maintain calibration accuracy over temperature range | High | Thermal calibration test |
| DOCK-ENV-TEMP-003 | Markers SHALL maintain reflectivity after 1000 hours UV exposure | Medium | UV aging test |
| DOCK-ENV-TEMP-004 | Markers SHALL maintain adhesion after 100 freeze-thaw cycles | Medium | Thermal cycling test |

**Total Requirements: 4**

---

## 8. Integration Requirements

### 8.1 Nav2 Integration (DOCK-INT-NAV)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-INT-NAV-001 | System SHALL receive approach zone waypoint from Nav2 | Critical | Integration test |
| DOCK-INT-NAV-002 | System SHALL hand off control from Nav2 to visual servoing | Critical | Integration test |
| DOCK-INT-NAV-003 | System SHALL restore Nav2 control after docking complete | High | Integration test |
| DOCK-INT-NAV-004 | System SHALL define "docking approach zones" in map | High | Map configuration |

**Total Requirements: 4**

---

### 8.2 eHMI Integration (DOCK-INT-EHMI)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-INT-EHMI-001 | System SHALL update eHMI state to 21 (approaching wheelchair) | High | Integration test |
| DOCK-INT-EHMI-002 | System SHALL update eHMI state to 22 (docking in progress) | High | Integration test |
| DOCK-INT-EHMI-003 | System SHALL update eHMI state to 23 (docking complete) | High | Integration test |
| DOCK-INT-EHMI-004 | System SHALL update eHMI state to 28 (docking failed) | Medium | Integration test |

**Total Requirements: 4**

---

### 8.3 Safety System Integration (DOCK-INT-SAFE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-INT-SAFE-001 | System SHALL respect safety system velocity limits during docking | Critical | Safety test |
| DOCK-INT-SAFE-002 | System SHALL report docking state to safety monitor | High | Integration test |
| DOCK-INT-SAFE-003 | System SHALL abort docking on safety system request | Critical | Fault injection test |

**Total Requirements: 3**

---

## 9. Testing & Validation Requirements

### 9.1 Unit Testing (DOCK-TEST-UNIT)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-TEST-UNIT-001 | ArUco detection SHALL have >90% unit test coverage | High | Code coverage |
| DOCK-TEST-UNIT-002 | Visual servoing controller SHALL have >85% unit test coverage | High | Code coverage |
| DOCK-TEST-UNIT-003 | PID controller SHALL have dedicated test for edge cases | High | Test plan |

**Total Requirements: 3**

---

### 9.2 Field Testing (DOCK-TEST-FIELD)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| DOCK-TEST-FIELD-001 | System SHALL complete 100 successful outdoor docking tests | Critical | Test report |
| DOCK-TEST-FIELD-002 | System SHALL complete 50 successful night docking tests | High | Test report |
| DOCK-TEST-FIELD-003 | System SHALL complete 50 successful rain docking tests | High | Test report |
| DOCK-TEST-FIELD-004 | System SHALL log accuracy statistics (mean, std dev, 95th percentile) | High | Test report |

**Total Requirements: 4**

---

### 9.3 Acceptance Criteria (DOCK-TEST-ACCEPT)

| Criterion | Target | MVP Target | Validation |
|-----------|--------|------------|------------|
| Docking Success Rate (Outdoor) | >95% | >90% | 100 trials |
| Docking Success Rate (Indoor) | >98% | >95% | 50 trials |
| Position Accuracy (Outdoor) | ±5mm | ±10mm | Statistical analysis |
| Position Accuracy (Indoor) | ±2mm | ±5mm | Statistical analysis |
| Orientation Accuracy | ±0.5° | ±1° | Statistical analysis |
| Time to Dock | <120s | <180s | Mean of 50 trials |
| Detection Range | 0.2-5m | 0.5-3m | Field test |
| Night Operation Success | >90% | >80% | 50 night trials |

---

## 10. Requirements Summary

### Requirements Count by Category

| Category | Total Requirements | Critical | High | Medium | Low |
|----------|-------------------|----------|------|--------|-----|
| **Functional** | **52** | 32 | 17 | 3 | 0 |
| - ArUco Detection | 12 | 7 | 3 | 2 | 1 |
| - Visual Servoing | 13 | 8 | 4 | 1 | 0 |
| - State Machine | 11 | 7 | 4 | 0 | 0 |
| - Dual Camera | 8 | 5 | 3 | 0 | 0 |
| - Marker Management | 8 | 3 | 4 | 1 | 0 |
| **Performance** | **16** | 8 | 8 | 0 | 0 |
| - Precision | 6 | 3 | 3 | 0 | 0 |
| - Speed & Success | 5 | 3 | 2 | 0 | 0 |
| - Detection | 5 | 2 | 3 | 0 | 0 |
| **Hardware** | **21** | 10 | 9 | 2 | 0 |
| - Cameras | 9 | 5 | 3 | 1 | 0 |
| - Markers | 8 | 4 | 4 | 0 | 0 |
| - Lighting | 4 | 1 | 2 | 1 | 0 |
| **Software** | **24** | 13 | 7 | 4 | 0 |
| - ROS 2 Nodes | 6 | 4 | 0 | 2 | 0 |
| - Communication | 8 | 4 | 3 | 1 | 0 |
| - Libraries | 5 | 3 | 2 | 0 | 0 |
| - Configuration | 5 | 2 | 2 | 1 | 0 |
| **Safety** | **15** | 10 | 5 | 0 | 0 |
| - Collision Avoidance | 4 | 3 | 1 | 0 | 0 |
| - Emergency Stop | 3 | 3 | 0 | 0 | 0 |
| - Fault Handling | 5 | 2 | 2 | 1 | 0 |
| - Wheelchair Safety | 3 | 2 | 1 | 0 | 0 |
| **Environmental** | **12** | 2 | 7 | 3 | 0 |
| - Lighting | 4 | 1 | 2 | 1 | 0 |
| - Weather | 4 | 0 | 2 | 2 | 0 |
| - Temperature | 4 | 1 | 2 | 1 | 0 |
| **Integration** | **11** | 6 | 5 | 0 | 0 |
| - Nav2 | 4 | 3 | 1 | 0 | 0 |
| - eHMI | 4 | 0 | 3 | 1 | 0 |
| - Safety System | 3 | 3 | 0 | 0 | 0 |
| **Testing** | **10** | 1 | 7 | 0 | 0 |
| - Unit Testing | 3 | 0 | 3 | 0 | 0 |
| - Field Testing | 4 | 1 | 3 | 0 | 0 |
| - Acceptance | 3 | 0 | 1 | 0 | 0 |
| **TOTAL** | **161** | **82 (51%)** | **65 (40%)** | **12 (7%)** | **0 (0%)** |

**Priority Distribution:**
- **Critical (82 requirements, 51%)**: Safety-critical, system cannot function without these
- **High (65 requirements, 40%)**: Important for reliability and user experience
- **Medium (12 requirements, 7%)**: Nice-to-have features, quality improvements
- **Low (0 requirements, 0%)**: Future enhancements

---

## 11. Acceptance Criteria

### 11.1 Minimum Viable Product (MVP)

**MVP Definition:** Functional docking system with 90% success rate outdoor, ±10mm precision.

**MVP Requirements:**
- ✅ All Critical requirements (82 total) must be satisfied
- ✅ ArUco detection working at 10 Hz, range 0.5-3m
- ✅ Visual servoing achieving ±10mm accuracy
- ✅ State machine with APPROACH → SEARCH → SERVOING → DOCKED
- ✅ Dual camera fusion functional
- ✅ Safety stops on obstacle detection
- ✅ 90% success rate in 100 outdoor trials

**MVP Exclusions:**
- Night operation (can require daylight)
- Rain operation (can require dry conditions)
- Advanced PID tuning (basic tuning acceptable)

---

### 11.2 Production Ready

**Production Definition:** Robust, weather-resistant system meeting all critical + high requirements.

**Production Requirements:**
- ✅ All Critical (82) + All High (65) requirements satisfied = 147 total
- ✅ 95% success rate outdoor in 1000 trials
- ✅ ±5mm position accuracy outdoor, ±2mm indoor
- ✅ Night operation with active LED lighting
- ✅ Light rain operation (<2.5mm/hr)
- ✅ 100 successful night tests
- ✅ 50 successful rain tests
- ✅ IP66+ weatherproofing on all hardware
- ✅ 1000 hours UV durability testing on markers

---

## 12. Traceability

### 12.1 Parent Requirements Traceability

This document derives requirements from:

**SYSTEM_REQUIREMENTS.md (DOCK-REQ):**
| System Requirement | Derived Docking Requirements | Status |
|--------------------|------------------------------|--------|
| DOCK-REQ-001 (ArUco detection 10 Hz) | DOCK-ARUCO-002, DOCK-PERF-DET-001 | Traced |
| DOCK-REQ-002 (Outdoor navigation) | DOCK-INT-NAV-001 to 004 | Traced |
| DOCK-REQ-003 (±2-5mm outdoor precision) | DOCK-PERF-PREC-002, DOCK-VS-005 | Traced |
| DOCK-REQ-004 (±1mm indoor target) | DOCK-PERF-PREC-001 | Traced |
| DOCK-REQ-005 (4-8 ArUco markers) | DOCK-MARKER-001 | Traced |
| DOCK-REQ-006 (OpenCV ArUco library) | DOCK-SW-LIB-001 | Traced |
| DOCK-REQ-007 (Dual RGB cameras) | DOCK-CAM-001, DOCK-HW-CAM-001 | Traced |
| DOCK-REQ-008 (Docking in light rain) | DOCK-ENV-WEATHER-001 to 002 | Traced |
| DOCK-REQ-009 (>95% success rate) | DOCK-PERF-SPEED-002 | Traced |
| DOCK-REQ-010 (2-phase docking) | DOCK-SM-001 to 003 | Traced |

**All system-level docking requirements traced to subsystem requirements.** ✅

---

## 13. Change History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-15 | System Architect | Initial requirements specification based on SYSTEM_REQUIREMENTS.md, question_answers.md, and EHMI_SYSTEM_REFERENCE.md |

---

## Appendix A: Reference Architecture

### Docking System Block Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                     Docking Subsystem                           │
│                                                                 │
│  ┌──────────────┐          ┌──────────────────┐               │
│  │  Camera L    │──────────│  ArUco Detector  │               │
│  │  (1920×1080) │          │  Node (OpenCV)   │               │
│  └──────────────┘          │                  │               │
│                             │  - Detection 10Hz│               │
│  ┌──────────────┐          │  - Pose 6-DOF    │               │
│  │  Camera R    │──────────│  - TF2 publish   │               │
│  │  (1920×1080) │          └────────┬─────────┘               │
│  └──────────────┘                   │                          │
│                                      │ /aruco_detections        │
│                                      ↓                          │
│  ┌──────────────┐          ┌──────────────────┐               │
│  │  Nav2        │────────→ │  Docking State   │               │
│  │  (Approach)  │ waypoint │  Machine         │               │
│  └──────────────┘          │                  │               │
│                             │  IDLE            │               │
│                             │  APPROACH        │               │
│                             │  SEARCH          │               │
│                             │  SERVOING ◄──────┼───┐          │
│                             │  DOCKED          │   │          │
│                             │  UNDOCKING       │   │          │
│                             └────────┬─────────┘   │          │
│                                      │             │          │
│                                      │ state       │          │
│                                      ↓             │          │
│  ┌──────────────┐          ┌──────────────────┐   │          │
│  │  /cmd_vel    │◄─────────│  Visual Servoing │◄──┘          │
│  │  (Swerve)    │          │  Controller      │   /aruco_detections
│  └──────────────┘          │                  │               │
│                             │  - PID (X,Y,θ)   │               │
│                             │  - Anti-windup   │               │
│                             │  - Velocity limit│               │
│                             └──────────────────┘               │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

**Document Status:** Draft
**Next Review:** After design phase completion
**Approvals Required:** System Architect, Safety Engineer, Navigation Lead
