# Swerve Drive System Requirements

**Project:** Outdoor-First Wheelchair Transport Robot with Swerve Drive
**Document Type:** Subsystem Requirements Specification
**Status:** Draft v1.0
**Date:** December 15, 2025
**Parent Document:** SYSTEM_REQUIREMENTS.md

---

## Document Purpose

This document specifies the complete requirements for the swerve drive subsystem, which provides omnidirectional mobility for the wheelchair transport robot. The swerve drive enables zero-radius turning, sideways motion, and diagonal movement - critical capabilities for precision wheelchair docking in constrained environments.

---

## 1. Overview

### 1.1 Subsystem Description

The swerve drive system consists of four independent wheel modules, each capable of steering and driving independently. This configuration provides:
- Omnidirectional motion (forward, backward, sideways, diagonal)
- Zero-radius turning (rotate in place)
- Superior outdoor performance vs. mecanum wheels
- Precise position control for docking operations

### 1.2 Key Capabilities

| Capability | Specification | Importance |
|------------|--------------|------------|
| Max Linear Speed | 1.5 m/s (no passenger), 1.0 m/s (with passenger) | Critical |
| Max Angular Speed | 1.0 rad/s (57°/s) | High |
| Max Slope | 10° (17.6% grade) with 100kg load | Critical |
| Position Accuracy | ±10mm at low speeds (<0.2 m/s) | High |
| Zero-radius Turn | Rotate in place (0m turning radius) | Critical |

---

## 2. Functional Requirements

### 2.1 Motion Control (SWERVE-MC)

| ID | Requirement | Priority | Validation |
|----|-------------|----------|------------|
| SWERVE-MC-001 | System SHALL support omnidirectional motion (vx, vy, ω) | Critical | Integration test |
| SWERVE-MC-002 | System SHALL execute zero-radius turning (rotate in place) | Critical | Unit test |
| SWERVE-MC-003 | System SHALL support forward/backward motion | Critical | Unit test |
| SWERVE-MC-004 | System SHALL support sideways motion (left/right) | Critical | Docking test |
| SWERVE-MC-005 | System SHALL support diagonal motion | High | Integration test |
| SWERVE-MC-006 | System SHALL calculate inverse kinematics for all 4 modules | Critical | Unit test |
| SWERVE-MC-007 | System SHALL coordinate wheel steering and drive motors | Critical | Integration test |
| SWERVE-MC-008 | System SHALL handle velocity command updates at 20Hz minimum | Critical | Performance test |
| SWERVE-MC-009 | System SHALL smoothly blend motion transitions | High | Comfort test |
| SWERVE-MC-010 | System SHALL limit acceleration to configured max (default 0.5 m/s²) | Critical | Safety test |

**Success Criteria:** Smooth omnidirectional motion in all directions with <5% velocity tracking error

---

### 2.2 Inverse Kinematics (SWERVE-IK)

| ID | Requirement | Priority | Validation |
|----|-------------|----------|------------|
| SWERVE-IK-001 | System SHALL compute wheel speeds from body velocities (vx, vy, ω) | Critical | Unit test |
| SWERVE-IK-002 | System SHALL compute wheel angles from body velocities | Critical | Unit test |
| SWERVE-IK-003 | System SHALL handle singularities (zero velocity commands) | High | Edge case test |
| SWERVE-IK-004 | System SHALL normalize wheel speeds to respect motor limits | Critical | Safety test |
| SWERVE-IK-005 | System SHALL optimize wheel orientations to minimize rotation | High | Performance test |
| SWERVE-IK-006 | System SHALL update IK calculations at 50Hz minimum | Critical | Performance test |
| SWERVE-IK-007 | System SHALL provide accurate odometry from wheel encoders | High | Integration test |

**Success Criteria:** IK calculations complete within 1ms, accuracy ±1% vs. theoretical values

---

### 2.3 Wheel Module Control (SWERVE-WM)

| ID | Requirement | Priority | Validation |
|----|-------------|----------|------------|
| SWERVE-WM-001 | Each module SHALL independently control steering angle | Critical | Unit test |
| SWERVE-WM-002 | Each module SHALL independently control drive speed | Critical | Unit test |
| SWERVE-WM-003 | Steering angle range SHALL be ±180° (continuous rotation) | High | Hardware test |
| SWERVE-WM-004 | Steering angle accuracy SHALL be ±2° | High | Calibration test |
| SWERVE-WM-005 | Drive speed accuracy SHALL be ±5% of commanded speed | High | Performance test |
| SWERVE-WM-006 | Module SHALL report encoder feedback at 100Hz minimum | Critical | Performance test |
| SWERVE-WM-007 | Module SHALL handle reverse direction without steering 180° | High | Efficiency test |
| SWERVE-WM-008 | Module SHALL protect against mechanical limits | Critical | Safety test |

**Success Criteria:** All 4 modules achieve commanded positions within tolerances, <50ms response time

---

### 2.4 Nav2 Integration (SWERVE-NAV)

| ID | Requirement | Priority | Validation |
|----|-------------|----------|------------|
| SWERVE-NAV-001 | System SHALL implement Nav2 controller plugin interface | Critical | Integration test |
| SWERVE-NAV-002 | System SHALL accept Twist commands from Nav2 local planner | Critical | Integration test |
| SWERVE-NAV-003 | System SHALL publish odometry to /odom topic | Critical | Integration test |
| SWERVE-NAV-004 | System SHALL publish TF transform (odom → base_link) | Critical | Integration test |
| SWERVE-NAV-005 | System SHALL support dynamic reconfiguration of parameters | High | Integration test |
| SWERVE-NAV-006 | System SHALL handle emergency stop commands (<100ms) | Critical | Safety test |
| SWERVE-NAV-007 | System SHALL provide diagnostic status via ROS 2 diagnostics | High | Integration test |

**Success Criteria:** Seamless integration with Nav2 stack, robot follows planned paths with <10cm error

---

## 3. Performance Requirements

### 3.1 Speed and Acceleration (SWERVE-PERF-SPEED)

| ID | Requirement | Priority | Validation |
|----|-------------|----------|------------|
| SWERVE-PERF-001 | Max linear velocity SHALL be 1.5 m/s (no passenger) | Critical | Field test |
| SWERVE-PERF-002 | Max linear velocity SHALL be 1.0 m/s (with passenger) | Critical | Safety test |
| SWERVE-PERF-003 | Max angular velocity SHALL be 1.0 rad/s | High | Field test |
| SWERVE-PERF-004 | Max linear acceleration SHALL be 0.5 m/s² (with passenger) | Critical | Comfort test |
| SWERVE-PERF-005 | Max linear acceleration SHALL be 1.0 m/s² (no passenger) | High | Field test |
| SWERVE-PERF-006 | Max angular acceleration SHALL be 0.5 rad/s² | High | Comfort test |
| SWERVE-PERF-007 | Zero-radius turn rate SHALL be ≥30°/s | High | Maneuverability test |

**Success Criteria:** All speed/acceleration limits enforced, smooth motion profile

---

### 3.2 Precision and Accuracy (SWERVE-PERF-ACC)

| ID | Requirement | Priority | Validation |
|----|-------------|----------|------------|
| SWERVE-PERF-008 | Position accuracy SHALL be ±10mm at speeds <0.2 m/s | High | Docking test |
| SWERVE-PERF-009 | Heading accuracy SHALL be ±2° at speeds <0.2 m/s | High | Docking test |
| SWERVE-PERF-010 | Velocity tracking error SHALL be <5% of commanded velocity | High | Performance test |
| SWERVE-PERF-011 | Odometry drift SHALL be <2% over 100m travel | High | Field test |
| SWERVE-PERF-012 | Repeatability SHALL be ±5mm for same motion command | Medium | Precision test |

**Success Criteria:** Precision sufficient for ±2-5mm docking accuracy (when combined with visual servoing)

---

### 3.3 Response Time (SWERVE-PERF-TIME)

| ID | Requirement | Priority | Validation |
|----|-------------|----------|------------|
| SWERVE-PERF-013 | Controller SHALL process cmd_vel at 20Hz minimum | Critical | Performance test |
| SWERVE-PERF-014 | IK calculations SHALL complete in <1ms | Critical | Performance test |
| SWERVE-PERF-015 | Emergency stop SHALL activate in <100ms | Critical | Safety test |
| SWERVE-PERF-016 | Wheel module response time SHALL be <50ms | High | Hardware test |
| SWERVE-PERF-017 | Odometry publishing rate SHALL be 50Hz minimum | High | Integration test |

**Success Criteria:** Real-time performance maintained under all operating conditions

---

## 4. Hardware Requirements

### 4.1 Wheel Modules (SWERVE-HW-MODULE)

| ID | Requirement | Priority | Validation |
|----|-------------|----------|------------|
| SWERVE-HW-001 | System SHALL use 4 independent swerve drive modules | Critical | Design review |
| SWERVE-HW-002 | Each module SHALL use 6.5" (φ165mm) hoverboard in-wheel motor | Critical | Hardware spec |
| SWERVE-HW-003 | Drive motor power SHALL be ≥80W per wheel | Critical | Hardware spec |
| SWERVE-HW-004 | Steering motor torque SHALL be sufficient for 100kg load | Critical | Load test |
| SWERVE-HW-005 | Wheel modules SHALL be arranged in rectangular configuration | Critical | Design review |
| SWERVE-HW-006 | Wheelbase (L) SHALL be 0.6m ±0.05m | High | Hardware spec |
| SWERVE-HW-007 | Track width (W) SHALL be 0.5m ±0.05m | High | Hardware spec |

**Success Criteria:** Hardware capable of meeting all performance requirements under rated load

---

### 4.2 Sensors (SWERVE-HW-SENSOR)

| ID | Requirement | Priority | Validation |
|----|-------------|----------|------------|
| SWERVE-HW-008 | Each drive motor SHALL have encoder (resolution ≥1000 CPR) | Critical | Hardware spec |
| SWERVE-HW-009 | Each steering motor SHALL have encoder (resolution ≥1000 CPR) | Critical | Hardware spec |
| SWERVE-HW-010 | System SHALL include IMU for orientation/tilt sensing | Critical | Hardware spec |
| SWERVE-HW-011 | Encoders SHALL provide feedback at 100Hz minimum | Critical | Performance test |
| SWERVE-HW-012 | IMU SHALL provide data at 100Hz minimum | High | Performance test |

**Success Criteria:** All sensors provide accurate, high-frequency feedback for closed-loop control

---

### 4.3 Motor Controllers (SWERVE-HW-CTRL)

| ID | Requirement | Priority | Validation |
|----|-------------|----------|------------|
| SWERVE-HW-013 | Drive motor controllers SHALL support velocity control | Critical | Hardware spec |
| SWERVE-HW-014 | Steering motor controllers SHALL support position control | Critical | Hardware spec |
| SWERVE-HW-015 | Controllers SHALL have overcurrent protection | Critical | Safety test |
| SWERVE-HW-016 | Controllers SHALL have thermal protection | Critical | Safety test |
| SWERVE-HW-017 | Controllers SHALL communicate via CAN bus or serial | High | Integration test |
| SWERVE-HW-018 | Controller update rate SHALL be 100Hz minimum | High | Performance test |

**Success Criteria:** Controllers provide reliable, real-time motor control with safety protections

---

## 5. Software Requirements

### 5.1 Controller Implementation (SWERVE-SW-CTRL)

| ID | Requirement | Priority | Validation |
|----|-------------|----------|------------|
| SWERVE-SW-001 | Controller SHALL be implemented in C++17 | Critical | Code review |
| SWERVE-SW-002 | Controller SHALL use ROS 2 Humble | Critical | Integration test |
| SWERVE-SW-003 | Controller SHALL implement nav2_core::Controller interface | Critical | Integration test |
| SWERVE-SW-004 | Controller SHALL be configurable via YAML parameters | High | Integration test |
| SWERVE-SW-005 | Controller SHALL log diagnostic information | High | Integration test |
| SWERVE-SW-006 | Controller SHALL handle node lifecycle (configure/activate/deactivate) | High | Integration test |

**Success Criteria:** Controller integrates seamlessly with Nav2, configurable via standard ROS 2 mechanisms

---

### 5.2 Configuration Parameters (SWERVE-SW-CONFIG)

| ID | Requirement | Priority | Validation |
|----|-------------|----------|------------|
| SWERVE-SW-007 | System SHALL expose wheelbase parameter | Critical | Integration test |
| SWERVE-SW-008 | System SHALL expose track width parameter | Critical | Integration test |
| SWERVE-SW-009 | System SHALL expose wheel radius parameter | Critical | Integration test |
| SWERVE-SW-010 | System SHALL expose max velocities (linear, angular) | Critical | Safety test |
| SWERVE-SW-011 | System SHALL expose max accelerations | Critical | Safety test |
| SWERVE-SW-012 | System SHALL expose PID gains for wheel control | High | Tuning test |
| SWERVE-SW-013 | System SHALL validate parameters on startup | High | Integration test |

**Success Criteria:** All critical parameters configurable, validated, and documented

---

## 6. Safety Requirements

### 6.1 Operational Safety (SWERVE-SAFE-OP)

| ID | Requirement | Priority | Validation |
|----|-------------|----------|------------|
| SWERVE-SAFE-001 | System SHALL enforce max speed limits (no override) | Critical | Safety test |
| SWERVE-SAFE-002 | System SHALL enforce max acceleration limits | Critical | Safety test |
| SWERVE-SAFE-003 | System SHALL reduce speed on slopes >5° | Critical | Field test |
| SWERVE-SAFE-004 | System SHALL stop on slopes >10° | Critical | Safety test |
| SWERVE-SAFE-005 | System SHALL detect wheel slip and reduce speed | High | Safety test |
| SWERVE-SAFE-006 | System SHALL not operate with wheel module failure | Critical | Failure test |
| SWERVE-SAFE-007 | System SHALL provide emergency stop capability | Critical | Safety test |

**Success Criteria:** Zero safety incidents in 1000 test missions

---

### 6.2 Fault Detection (SWERVE-SAFE-FAULT)

| ID | Requirement | Priority | Validation |
|----|-------------|----------|------------|
| SWERVE-SAFE-008 | System SHALL detect encoder failures | Critical | Failure test |
| SWERVE-SAFE-009 | System SHALL detect motor controller failures | Critical | Failure test |
| SWERVE-SAFE-010 | System SHALL detect communication timeouts | Critical | Failure test |
| SWERVE-SAFE-011 | System SHALL detect abnormal motor currents | High | Failure test |
| SWERVE-SAFE-012 | System SHALL report faults via ROS 2 diagnostics | High | Integration test |
| SWERVE-SAFE-013 | System SHALL enter safe state on critical fault | Critical | Failure test |

**Success Criteria:** All critical faults detected within 100ms, safe state achieved within 500ms

---

## 7. Environmental Requirements

### 7.1 Operating Conditions (SWERVE-ENV)

| ID | Requirement | Priority | Validation |
|----|-------------|----------|------------|
| SWERVE-ENV-001 | System SHALL operate on flat surfaces (0° slope) | Critical | Field test |
| SWERVE-ENV-002 | System SHALL operate on slopes up to 10° with 100kg load | Critical | Field test |
| SWERVE-ENV-003 | System SHALL operate on grass, asphalt, concrete | Critical | Field test |
| SWERVE-ENV-004 | System SHALL handle ±20mm surface irregularities | High | Field test |
| SWERVE-ENV-005 | System SHALL operate in temperature range -5°C to 40°C | High | Environmental test |
| SWERVE-ENV-006 | System SHALL operate in light rain (IP54+ motors) | Critical | Environmental test |
| SWERVE-ENV-007 | System SHALL handle dust/debris without damage | High | Environmental test |

**Success Criteria:** Reliable operation across all specified environmental conditions

---

## 8. Integration Requirements

### 8.1 ROS 2 Integration (SWERVE-INT-ROS)

| ID | Requirement | Priority | Validation |
|----|-------------|----------|------------|
| SWERVE-INT-001 | System SHALL subscribe to /cmd_vel (geometry_msgs/Twist) | Critical | Integration test |
| SWERVE-INT-002 | System SHALL publish to /odom (nav_msgs/Odometry) | Critical | Integration test |
| SWERVE-INT-003 | System SHALL publish TF transform (odom → base_link) | Critical | Integration test |
| SWERVE-INT-004 | System SHALL publish joint states (sensor_msgs/JointState) | High | Integration test |
| SWERVE-INT-005 | System SHALL provide /reset_odometry service | Medium | Integration test |
| SWERVE-INT-006 | System SHALL publish diagnostics (diagnostic_msgs/DiagnosticArray) | High | Integration test |

**Success Criteria:** Full ROS 2 integration, compatible with standard Nav2 stack

---

## 9. Testing & Validation Requirements

### 9.1 Unit Testing (SWERVE-TEST-UNIT)

| ID | Requirement | Priority | Validation |
|----|-------------|----------|------------|
| SWERVE-TEST-001 | IK calculations SHALL have unit tests (80%+ coverage) | Critical | CI/CD |
| SWERVE-TEST-002 | Odometry calculations SHALL have unit tests | Critical | CI/CD |
| SWERVE-TEST-003 | Configuration validation SHALL have unit tests | High | CI/CD |
| SWERVE-TEST-004 | Safety limits SHALL have unit tests | Critical | CI/CD |

**Success Criteria:** 80%+ code coverage, all tests pass on every commit

---

### 9.2 Integration Testing (SWERVE-TEST-INT)

| ID | Requirement | Priority | Validation |
|----|-------------|----------|------------|
| SWERVE-TEST-005 | Nav2 integration SHALL be tested end-to-end | Critical | Integration test |
| SWERVE-TEST-006 | All motion directions SHALL be tested | Critical | Integration test |
| SWERVE-TEST-007 | Odometry accuracy SHALL be tested over 100m | High | Integration test |
| SWERVE-TEST-008 | Emergency stop SHALL be tested | Critical | Safety test |

**Success Criteria:** All integration tests pass, system ready for field testing

---

### 9.3 Field Testing (SWERVE-TEST-FIELD)

| ID | Requirement | Priority | Validation |
|----|-------------|----------|------------|
| SWERVE-TEST-009 | System SHALL complete 100km endurance test | High | Field test |
| SWERVE-TEST-010 | System SHALL operate on various terrains | Critical | Field test |
| SWERVE-TEST-011 | System SHALL operate on max slope (10°) | Critical | Field test |
| SWERVE-TEST-012 | System SHALL operate with max load (100kg) | Critical | Load test |

**Success Criteria:** >98% uptime during field testing, zero safety incidents

---

## 10. Documentation Requirements

### 10.1 Technical Documentation (SWERVE-DOC)

| ID | Requirement | Priority | Validation |
|----|-------------|----------|------------|
| SWERVE-DOC-001 | IK equations SHALL be documented | Critical | Doc review |
| SWERVE-DOC-002 | Configuration parameters SHALL be documented | Critical | Doc review |
| SWERVE-DOC-003 | ROS 2 interfaces SHALL be documented | Critical | Doc review |
| SWERVE-DOC-004 | Calibration procedures SHALL be documented | High | Doc review |
| SWERVE-DOC-005 | Troubleshooting guide SHALL be provided | High | Doc review |

**Success Criteria:** Complete documentation enabling independent development/maintenance

---

## 11. Requirements Traceability

### 11.1 Parent Requirements Mapping

| This Document | Parent (SYSTEM_REQUIREMENTS.md) |
|--------------|--------------------------------|
| SWERVE-MC-* | PERF-001 (Battery life), NAV-* (Navigation) |
| SWERVE-PERF-* | PERF-001 to PERF-008 |
| SWERVE-SAFE-* | SAFE-001 to SAFE-014 |
| SWERVE-ENV-* | ENV-001 to ENV-007 |

### 11.2 Dependent Subsystems

| Subsystem | Dependency | Interface |
|-----------|-----------|-----------|
| Navigation | Requires swerve drive | /cmd_vel topic |
| Docking | Requires precision motion | /cmd_vel topic |
| Safety | Monitors drive system | Emergency stop |
| UI | Displays drive status | Diagnostics |

---

## 12. Acceptance Criteria

### 12.1 Minimum Viable Product (MVP)

- [ ] All CRITICAL requirements validated
- [ ] ≥80% HIGH requirements validated
- [ ] Nav2 integration functional
- [ ] Omnidirectional motion demonstrated
- [ ] Emergency stop functional
- [ ] Basic field testing complete (10km)

### 12.2 Production Ready

- [ ] 100% CRITICAL + HIGH requirements validated
- [ ] ≥50% MEDIUM requirements validated
- [ ] Field testing: 100km or 30 days operation
- [ ] Zero safety incidents
- [ ] Customer acceptance test passed

---

## 13. Open Items / TBD

| Item | Status | Owner | Target Date |
|------|--------|-------|-------------|
| Final wheel module hardware selection | TBD | Hardware team | Sprint 3 |
| Motor controller communication protocol | TBD | Hardware team | Sprint 3 |
| PID tuning parameters | TBD | Controls team | Sprint 4 |
| Wheelbase/track width final dimensions | TBD | Mechanical team | Sprint 3 |

---

## 14. Related Documents

**Parent Documents:**
- `SYSTEM_REQUIREMENTS.md` - Overall system requirements

**Related Subsystem Docs:**
- `NAVIGATION_REQUIREMENTS.md` - Navigation subsystem (consumer of swerve drive)
- `DOCKING_SYSTEM_REQUIREMENTS.md` - Docking subsystem (consumer of swerve drive)
- `SAFETY_REQUIREMENTS.md` - Safety subsystem (monitors swerve drive)

**Architecture & Design:**
- `SWERVE_DRIVE_ARCHITECTURE.md` - Detailed architecture
- `SWERVE_DRIVE_CONTROLLER_DESIGN.md` - Controller implementation design

---

**Document Status:** Draft v1.0
**Review Status:** Pending technical review
**Next Review:** After Sprint 3 (swerve drive hardware selection)
**Approvers:** System Architect, Controls Lead, Safety Lead
**Version History:**
- v1.0 (2025-12-15): Initial swerve drive requirements specification
