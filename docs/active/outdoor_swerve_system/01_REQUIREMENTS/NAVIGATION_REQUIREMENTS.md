# Navigation Requirements

**Document ID:** REQ-NAV-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft
**Classification:** Internal

---

## Document Control

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-15 | Navigation Lead | Initial navigation requirements based on ParcelPal proven architecture |

---

## 1. Introduction

### 1.1 Purpose

This document specifies the **Navigation Requirements** for the outdoor-first wheelchair transport robot. The navigation subsystem enables autonomous point-to-point travel using proven NDT localization and Nav2 path planning, based on the ParcelPal delivery robot architecture.

### 1.2 Scope

This specification covers:
- **Localization** - NDT scan matching on pre-built maps (NO continuous SLAM during operation)
- **Path Planning** - Global (A*/Dijkstra) and local (DWB) planning with Nav2
- **Map Management** - Offline mapping, map storage, map updates
- **Multi-Waypoint Navigation** - Sequential waypoint execution for 500m-1km missions
- **Sensor Fusion** - LiDAR + IMU + wheel odometry
- **Route Execution** - Goal queue management, re-planning, recovery behaviors
- **Performance** - ±10cm accuracy over 500m, NO GPS required

**Out of Scope:**
- Docking precision control (covered in DOCKING_SYSTEM_REQUIREMENTS.md)
- Obstacle avoidance during docking (covered in DOCKING_SYSTEM_REQUIREMENTS.md)
- Safety limits and emergency stop (covered in SAFETY_REQUIREMENTS.md)

### 1.3 Related Documents

- **SYSTEM_REQUIREMENTS.md** - Section 1.2 (NAV-REQ-001 to 012)
- **OVERALL_SYSTEM_ARCHITECTURE.md** - Section 3.1 (Navigation Subsystem)
- **PARCELPAL_EXPLORATION_SUMMARY.md** - Proven NDT localization architecture
- **question_answers.md** - GPS exclusion, 500m-1km range clarifications

### 1.4 Navigation Architecture Overview

The system implements the **ParcelPal 2-Phase Navigation Strategy**:

**Phase 1: Offline Mapping (Once, Manual Drive)**
```
Manual teleoperation drive
    ↓
Record LiDAR + odometry + IMU
    ↓
SLAM Toolbox (mapping mode) with loop closure
    ↓
Save final map
    ↓
Annotate waypoints (stops, charging, docking zones)
```

**Phase 2: Online Localization (Daily Operation)**
```
Load saved map
    ↓
NDT scan matching on saved map
    ↓
Continuous pose estimation (NO drift accumulation!)
    ↓
Nav2 path planning to waypoints
    ↓
Swerve drive controller execution
```

**Key Insight:** This approach eliminates drift accumulation by localizing on a fixed pre-built map, enabling reliable 500m-1km navigation WITHOUT GPS.

---

## 2. Localization Requirements

### 2.1 NDT Scan Matching (NAV-LOC-NDT)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-LOC-NDT-001 | System SHALL use NDT (Normal Distributions Transform) for localization | Critical | Configuration |
| NAV-LOC-NDT-002 | System SHALL localize on pre-built static map (NOT continuous SLAM) | Critical | Architecture review |
| NAV-LOC-NDT-003 | System SHALL achieve ±10cm position accuracy over 500m without GPS | Critical | Field test (100 trials) |
| NAV-LOC-NDT-004 | System SHALL achieve ±2° orientation accuracy | High | Field test |
| NAV-LOC-NDT-005 | System SHALL publish pose estimates at 10 Hz minimum | Critical | Performance test |
| NAV-LOC-NDT-006 | System SHALL compute pose covariance (uncertainty estimate) | High | Unit test |
| NAV-LOC-NDT-007 | System SHALL use voxel grid size 0.5m-1.0m (outdoor tuning) | High | Configuration |
| NAV-LOC-NDT-008 | System SHALL use Newton optimization with max 30 iterations | Medium | Configuration |

**Total Requirements: 8**

---

### 2.2 Sensor Fusion (NAV-LOC-FUSION)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-LOC-FUSION-001 | System SHALL fuse 3D LiDAR + IMU + wheel odometry | Critical | Integration test |
| NAV-LOC-FUSION-002 | System SHALL use Extended Kalman Filter (EKF) or Particle Filter | High | Architecture review |
| NAV-LOC-FUSION-003 | System SHALL weight sensors: LiDAR (70%), IMU (20%), odometry (10%) | Medium | Configuration |
| NAV-LOC-FUSION-004 | System SHALL compensate for IMU tilt in pose estimation | Critical | Integration test |
| NAV-LOC-FUSION-005 | System SHALL detect and reject outlier sensor readings | High | Fault injection test |
| NAV-LOC-FUSION-006 | System SHALL maintain localization at 10 Hz even if LiDAR drops to 5 Hz | High | Performance test |

**Total Requirements: 6**

---

### 2.3 Map Management (NAV-LOC-MAP)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-LOC-MAP-001 | System SHALL load pre-built map on startup | Critical | Integration test |
| NAV-LOC-MAP-002 | System SHALL support map files in PCD (Point Cloud Data) format | Critical | File format test |
| NAV-LOC-MAP-003 | System SHALL support map files up to 500 MB (1km outdoor map) | High | Load test |
| NAV-LOC-MAP-004 | System SHALL load map within 30 seconds on startup | High | Performance test |
| NAV-LOC-MAP-005 | System SHALL store multiple maps (e.g., site A, site B) | Medium | Configuration |
| NAV-LOC-MAP-006 | System SHALL allow map updates without re-compilation | High | Integration test |
| NAV-LOC-MAP-007 | System SHALL validate map file integrity (checksum) on load | Medium | Unit test |

**Total Requirements: 7**

---

### 2.4 Initial Pose Estimation (NAV-LOC-INIT)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-LOC-INIT-001 | System SHALL require initial pose input (x, y, θ) on startup | Critical | Integration test |
| NAV-LOC-INIT-002 | System SHALL support initial pose via UI (map click) | High | UI test |
| NAV-LOC-INIT-003 | System SHALL support initial pose via saved "home" position | High | Configuration |
| NAV-LOC-INIT-004 | System SHALL verify initial pose convergence within 10 seconds | Critical | Performance test |
| NAV-LOC-INIT-005 | System SHALL refuse navigation if localization confidence <80% | Critical | Safety test |
| NAV-LOC-INIT-006 | System SHALL publish localization confidence to /nav/confidence topic | High | Integration test |

**Total Requirements: 6**

---

### 2.5 Localization Failure Handling (NAV-LOC-FAIL)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-LOC-FAIL-001 | System SHALL detect localization failure (pose jump >1m) | Critical | Fault injection test |
| NAV-LOC-FAIL-002 | System SHALL stop immediately on localization failure | Critical | Safety test |
| NAV-LOC-FAIL-003 | System SHALL request operator intervention on localization failure | Critical | Integration test |
| NAV-LOC-FAIL-004 | System SHALL log localization failures with sensor snapshot | High | Log inspection |
| NAV-LOC-FAIL-005 | System SHALL attempt auto-recovery (re-init from last known good pose) | Medium | Integration test |

**Total Requirements: 5**

---

## 3. Path Planning Requirements

### 3.1 Global Path Planning (NAV-PLAN-GLOBAL)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-PLAN-GLOBAL-001 | System SHALL use Nav2 global planner (NavFn or Smac Planner) | Critical | Configuration |
| NAV-PLAN-GLOBAL-002 | System SHALL plan shortest safe path from A to B on static map | Critical | Integration test |
| NAV-PLAN-GLOBAL-003 | System SHALL use A* or Dijkstra algorithm | High | Configuration |
| NAV-PLAN-GLOBAL-004 | System SHALL compute global plan within 1 second for 500m path | High | Performance test |
| NAV-PLAN-GLOBAL-005 | System SHALL re-plan global path if blocked by static obstacle | High | Integration test |
| NAV-PLAN-GLOBAL-006 | System SHALL respect no-go zones defined in map | Critical | Configuration test |
| NAV-PLAN-GLOBAL-007 | System SHALL prefer paved paths over grass (cost map tuning) | Medium | Field test |

**Total Requirements: 7**

---

### 3.2 Local Path Planning (NAV-PLAN-LOCAL)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-PLAN-LOCAL-001 | System SHALL use Nav2 local planner (DWB - Dynamic Window Approach) | Critical | Configuration |
| NAV-PLAN-LOCAL-002 | System SHALL plan collision-free trajectory 3-5 seconds ahead | Critical | Integration test |
| NAV-PLAN-LOCAL-003 | System SHALL update local plan at 5 Hz minimum | Critical | Performance test |
| NAV-PLAN-LOCAL-004 | System SHALL use swerve drive kinematics constraints | Critical | Configuration |
| NAV-PLAN-LOCAL-005 | System SHALL prefer smooth paths (minimize curvature) | High | Trajectory analysis |
| NAV-PLAN-LOCAL-006 | System SHALL avoid oscillations (damping factor ≥0.8) | High | Field test |
| NAV-PLAN-LOCAL-007 | System SHALL timeout if no feasible local plan found in 5 seconds | High | Integration test |

**Total Requirements: 7**

---

### 3.3 Cost Maps (NAV-PLAN-COST)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-PLAN-COST-001 | System SHALL maintain global cost map (static obstacles from map) | Critical | Integration test |
| NAV-PLAN-COST-002 | System SHALL maintain local cost map (dynamic obstacles from LiDAR) | Critical | Integration test |
| NAV-PLAN-COST-003 | System SHALL update local cost map at 5 Hz | Critical | Performance test |
| NAV-PLAN-COST-004 | System SHALL use inflation radius 0.5m (robot + safety margin) | Critical | Configuration |
| NAV-PLAN-COST-005 | System SHALL mark obstacles as LETHAL (255 cost) | Critical | Configuration |
| NAV-PLAN-COST-006 | System SHALL mark free space as FREE (0 cost) | Critical | Configuration |
| NAV-PLAN-COST-007 | System SHALL use gradient inflation (cost decreases with distance) | High | Configuration |
| NAV-PLAN-COST-008 | System SHALL clear local cost map on obstacle removal (decay rate) | Medium | Integration test |

**Total Requirements: 8**

---

### 3.4 Dynamic Obstacle Handling (NAV-PLAN-DYN)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-PLAN-DYN-001 | System SHALL detect dynamic obstacles (people, vehicles) in LiDAR | Critical | Field test |
| NAV-PLAN-DYN-002 | System SHALL track dynamic obstacles (Kalman filter) | High | Integration test |
| NAV-PLAN-DYN-003 | System SHALL predict obstacle motion 1-3 seconds ahead | High | Prediction test |
| NAV-PLAN-DYN-004 | System SHALL avoid predicted obstacle positions | Critical | Collision test |
| NAV-PLAN-DYN-005 | System SHALL wait if path blocked by dynamic obstacle (timeout 30s) | High | Integration test |
| NAV-PLAN-DYN-006 | System SHALL re-route if dynamic obstacle blocks path >30s | High | Integration test |

**Total Requirements: 6**

---

## 4. Multi-Waypoint Navigation

### 4.1 Waypoint Management (NAV-WP-MANAGE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-WP-MANAGE-001 | System SHALL support sequential waypoint navigation (A→B→C→D) | Critical | Integration test |
| NAV-WP-MANAGE-002 | System SHALL store waypoints with (x, y, θ) in map frame | Critical | Configuration |
| NAV-WP-MANAGE-003 | System SHALL support named waypoints (e.g., "Stop 1", "Charging") | High | UI test |
| NAV-WP-MANAGE-004 | System SHALL allow waypoint insertion/deletion via UI | High | UI test |
| NAV-WP-MANAGE-005 | System SHALL save waypoint lists for repeated missions | Medium | Configuration |
| NAV-WP-MANAGE-006 | System SHALL validate waypoints are within map bounds | High | Unit test |

**Total Requirements: 6**

---

### 4.2 Waypoint Execution (NAV-WP-EXEC)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-WP-EXEC-001 | System SHALL navigate to waypoints in sequence | Critical | Integration test |
| NAV-WP-EXEC-002 | System SHALL declare waypoint reached when within 0.3m, ±10° | Critical | Integration test |
| NAV-WP-EXEC-003 | System SHALL stop at each waypoint for configurable duration (0-60s) | High | Configuration |
| NAV-WP-EXEC-004 | System SHALL proceed to next waypoint after stop duration | High | Integration test |
| NAV-WP-EXEC-005 | System SHALL skip unreachable waypoints after 3 attempts | High | Integration test |
| NAV-WP-EXEC-006 | System SHALL report waypoint progress to UI | High | Integration test |
| NAV-WP-EXEC-007 | System SHALL handle waypoint cancellation (stop immediately) | High | Integration test |

**Total Requirements: 7**

---

### 4.3 Mission Planning (NAV-WP-MISSION)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-WP-MISSION-001 | System SHALL support missions up to 1km total distance | Critical | Field test |
| NAV-WP-MISSION-002 | System SHALL support missions with up to 10 waypoints | High | Integration test |
| NAV-WP-MISSION-003 | System SHALL compute total mission distance and ETA | High | Unit test |
| NAV-WP-MISSION-004 | System SHALL validate mission feasibility (battery, map coverage) | High | Unit test |
| NAV-WP-MISSION-005 | System SHALL refuse mission if battery insufficient | Critical | Safety test |
| NAV-WP-MISSION-006 | System SHALL pause mission on operator command | High | Integration test |
| NAV-WP-MISSION-007 | System SHALL resume mission from current position | High | Integration test |

**Total Requirements: 7**

---

## 5. Route Execution & Recovery

### 5.1 Goal Handling (NAV-EXEC-GOAL)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-EXEC-GOAL-001 | System SHALL accept navigation goals via ROS 2 action server | Critical | Integration test |
| NAV-EXEC-GOAL-002 | System SHALL provide goal feedback (distance remaining, ETA) | High | Integration test |
| NAV-EXEC-GOAL-003 | System SHALL declare goal succeeded when within tolerance | Critical | Integration test |
| NAV-EXEC-GOAL-004 | System SHALL declare goal failed after max attempts (3 retries) | High | Integration test |
| NAV-EXEC-GOAL-005 | System SHALL support goal cancellation | Critical | Integration test |
| NAV-EXEC-GOAL-006 | System SHALL support goal pre-emption (new goal cancels old) | High | Integration test |

**Total Requirements: 6**

---

### 5.2 Recovery Behaviors (NAV-EXEC-RECOVERY)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-EXEC-RECOVERY-001 | System SHALL implement recovery behaviors (clear, rotate, back up) | Critical | Integration test |
| NAV-EXEC-RECOVERY-002 | System SHALL attempt recovery if stuck (no progress >10s) | Critical | Stuck test |
| NAV-EXEC-RECOVERY-003 | System SHALL rotate 360° to clear local cost map | High | Integration test |
| NAV-EXEC-RECOVERY-004 | System SHALL back up 0.5m if forward path blocked | High | Integration test |
| NAV-EXEC-RECOVERY-005 | System SHALL wait 30s for dynamic obstacle to clear | High | Integration test |
| NAV-EXEC-RECOVERY-006 | System SHALL request operator help after 3 recovery attempts | High | Integration test |
| NAV-EXEC-RECOVERY-007 | System SHALL log all recovery attempts with reason | Medium | Log inspection |

**Total Requirements: 7**

---

### 5.3 Re-Planning (NAV-EXEC-REPLAN)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-EXEC-REPLAN-001 | System SHALL re-plan if deviation from path >1m | High | Integration test |
| NAV-EXEC-REPLAN-002 | System SHALL re-plan if obstacle blocks path | Critical | Integration test |
| NAV-EXEC-REPLAN-003 | System SHALL re-plan within 1 second | High | Performance test |
| NAV-EXEC-REPLAN-004 | System SHALL limit re-planning frequency to 1 Hz (stability) | High | Integration test |
| NAV-EXEC-REPLAN-005 | System SHALL maintain previous path if re-planning fails | Critical | Fault injection test |

**Total Requirements: 5**

---

## 6. Performance Requirements

### 6.1 Navigation Accuracy (NAV-PERF-ACC)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-PERF-ACC-001 | System SHALL achieve ±10cm position accuracy over 500m | Critical | Field test (50 missions) |
| NAV-PERF-ACC-002 | System SHALL achieve ±20cm position accuracy over 1km | High | Field test (20 missions) |
| NAV-PERF-ACC-003 | System SHALL achieve ±2° orientation accuracy | High | Field test |
| NAV-PERF-ACC-004 | System SHALL achieve 95th percentile accuracy ±15cm | High | Statistical analysis |
| NAV-PERF-ACC-005 | System SHALL NOT accumulate drift (localization on fixed map) | Critical | Drift analysis test |

**Total Requirements: 5**

---

### 6.2 Navigation Speed & Efficiency (NAV-PERF-SPEED)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-PERF-SPEED-001 | System SHALL maintain average speed 0.8 m/s for 500m missions | High | Field test |
| NAV-PERF-SPEED-002 | System SHALL complete 500m mission in <12 minutes | High | Mission test |
| NAV-PERF-SPEED-003 | System SHALL complete 1km mission in <25 minutes | High | Mission test |
| NAV-PERF-SPEED-004 | System SHALL optimize path length (within 10% of shortest path) | Medium | Path analysis |
| NAV-PERF-SPEED-005 | System SHALL minimize unnecessary stops (<3 per 500m) | Medium | Field test |

**Total Requirements: 5**

---

### 6.3 Computational Performance (NAV-PERF-COMP)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-PERF-COMP-001 | Localization SHALL use <20% CPU | High | Resource monitoring |
| NAV-PERF-COMP-002 | Path planning SHALL use <15% CPU | High | Resource monitoring |
| NAV-PERF-COMP-003 | Navigation stack SHALL use <5GB RAM | High | Resource monitoring |
| NAV-PERF-COMP-004 | Cost map updates SHALL complete in <100ms | High | Latency test |
| NAV-PERF-COMP-005 | Pose estimation SHALL have <50ms latency | Critical | Latency test |

**Total Requirements: 5**

---

## 7. Integration Requirements

### 7.1 ROS 2 Nav2 Integration (NAV-INT-NAV2)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-INT-NAV2-001 | System SHALL use Nav2 Humble (ROS 2 Humble) | Critical | Version check |
| NAV-INT-NAV2-002 | System SHALL use bt_navigator for behavior tree execution | Critical | Configuration |
| NAV-INT-NAV2-003 | System SHALL use controller_server for trajectory execution | Critical | Configuration |
| NAV-INT-NAV2-004 | System SHALL use planner_server for path planning | Critical | Configuration |
| NAV-INT-NAV2-005 | System SHALL use recoveries_server for recovery behaviors | High | Configuration |
| NAV-INT-NAV2-006 | System SHALL configure Nav2 via YAML files | High | Configuration test |

**Total Requirements: 6**

---

### 7.2 Swerve Drive Integration (NAV-INT-SWERVE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-INT-SWERVE-001 | System SHALL use swerve drive controller (nav2_core::Controller interface) | Critical | Code inspection |
| NAV-INT-SWERVE-002 | System SHALL support omnidirectional motion (vx, vy, ω) | Critical | Integration test |
| NAV-INT-SWERVE-003 | System SHALL use swerve kinematics in local planner | Critical | Configuration |
| NAV-INT-SWERVE-004 | System SHALL publish cmd_vel with twist (linear.x, linear.y, angular.z) | Critical | Topic inspection |

**Total Requirements: 4**

---

### 7.3 Sensor Integration (NAV-INT-SENSOR)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-INT-SENSOR-001 | System SHALL subscribe to /scan (3D LiDAR projected to 2D) | Critical | Topic inspection |
| NAV-INT-SENSOR-002 | System SHALL subscribe to /imu/data (IMU orientation + angular velocity) | Critical | Topic inspection |
| NAV-INT-SENSOR-003 | System SHALL subscribe to /odom (wheel odometry) | Critical | Topic inspection |
| NAV-INT-SENSOR-004 | System SHALL fuse sensors using robot_localization package | High | Configuration |
| NAV-INT-SENSOR-005 | System SHALL publish fused odometry to /odometry/filtered | High | Topic inspection |

**Total Requirements: 5**

---

## 8. Testing & Validation

### 8.1 Unit Testing (NAV-TEST-UNIT)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-TEST-UNIT-001 | NDT localization SHALL have >85% unit test coverage | High | Code coverage |
| NAV-TEST-UNIT-002 | Path planning algorithms SHALL have >80% unit test coverage | High | Code coverage |
| NAV-TEST-UNIT-003 | Waypoint manager SHALL have >90% unit test coverage | High | Code coverage |

**Total Requirements: 3**

---

### 8.2 Integration Testing (NAV-TEST-INT)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| NAV-TEST-INT-001 | System SHALL complete 50 successful 500m missions | Critical | Field test |
| NAV-TEST-INT-002 | System SHALL complete 20 successful 1km missions | Critical | Field test |
| NAV-TEST-INT-003 | System SHALL complete 10 multi-waypoint missions (5+ waypoints) | High | Field test |
| NAV-TEST-INT-004 | System SHALL handle 20 dynamic obstacle scenarios | High | Field test |
| NAV-TEST-INT-005 | System SHALL handle 10 localization re-initialization scenarios | High | Field test |

**Total Requirements: 5**

---

### 8.3 Acceptance Criteria (NAV-TEST-ACCEPT)

| Criterion | Target | MVP Target | Validation |
|-----------|--------|------------|------------|
| Position Accuracy (500m) | ±10cm | ±20cm | 50 missions |
| Position Accuracy (1km) | ±20cm | ±40cm | 20 missions |
| Mission Success Rate | >95% | >90% | 100 missions |
| Average Mission Speed | 0.8 m/s | 0.6 m/s | Field test |
| Localization Confidence | >95% | >90% | Continuous monitoring |
| Recovery Success Rate | >90% | >80% | 50 stuck scenarios |
| CPU Usage | <35% (loc+plan) | <50% | Resource monitoring |

---

## 9. Requirements Summary

### Requirements Count by Category

| Category | Total Requirements | Critical | High | Medium | Low |
|----------|-------------------|----------|------|--------|-----|
| **Localization** | **32** | 18 | 11 | 3 | 0 |
| - NDT Scan Matching | 8 | 4 | 3 | 1 | 0 |
| - Sensor Fusion | 6 | 3 | 3 | 0 | 0 |
| - Map Management | 7 | 3 | 3 | 1 | 0 |
| - Initial Pose | 6 | 4 | 2 | 0 | 0 |
| - Failure Handling | 5 | 4 | 1 | 0 | 0 |
| **Path Planning** | **28** | 16 | 10 | 2 | 0 |
| - Global Planning | 7 | 4 | 2 | 1 | 0 |
| - Local Planning | 7 | 5 | 2 | 0 | 0 |
| - Cost Maps | 8 | 6 | 1 | 1 | 0 |
| - Dynamic Obstacles | 6 | 3 | 3 | 0 | 0 |
| **Multi-Waypoint** | **20** | 8 | 11 | 1 | 0 |
| - Waypoint Management | 6 | 3 | 2 | 1 | 0 |
| - Waypoint Execution | 7 | 4 | 3 | 0 | 0 |
| - Mission Planning | 7 | 3 | 4 | 0 | 0 |
| **Route Execution** | **18** | 10 | 7 | 1 | 0 |
| - Goal Handling | 6 | 4 | 2 | 0 | 0 |
| - Recovery Behaviors | 7 | 3 | 4 | 0 | 0 |
| - Re-Planning | 5 | 3 | 2 | 0 | 0 |
| **Performance** | **15** | 3 | 10 | 2 | 0 |
| - Navigation Accuracy | 5 | 2 | 3 | 0 | 0 |
| - Speed & Efficiency | 5 | 0 | 3 | 2 | 0 |
| - Computational | 5 | 1 | 4 | 0 | 0 |
| **Integration** | **15** | 11 | 4 | 0 | 0 |
| - Nav2 Integration | 6 | 5 | 1 | 0 | 0 |
| - Swerve Drive | 4 | 4 | 0 | 0 | 0 |
| - Sensor Integration | 5 | 3 | 2 | 0 | 0 |
| **Testing** | **8** | 2 | 6 | 0 | 0 |
| - Unit Testing | 3 | 0 | 3 | 0 | 0 |
| - Integration Testing | 5 | 2 | 3 | 0 | 0 |
| **TOTAL** | **136** | **68 (50%)** | **59 (43%)** | **9 (7%)** | **0 (0%)** |

**Priority Distribution:**
- **Critical (68 requirements, 50%)**: Core navigation functionality, system cannot navigate without these
- **High (59 requirements, 43%)**: Important for reliability and robustness
- **Medium (9 requirements, 7%)**: Nice-to-have optimizations
- **Low (0 requirements, 0%)**: N/A for navigation subsystem

---

## 10. Acceptance Criteria

### 10.1 Minimum Viable Product (MVP)

**MVP Definition:** Reliable point-to-point autonomous navigation on pre-mapped routes, ±20cm accuracy over 500m.

**MVP Requirements:**
- ✅ All Critical requirements (68 total) must be satisfied
- ✅ NDT localization operational on pre-built maps
- ✅ Nav2 path planning functional (global + local)
- ✅ ±20cm accuracy over 500m (90% of missions)
- ✅ >90% mission success rate (50 test missions)
- ✅ Multi-waypoint navigation (up to 5 waypoints)
- ✅ Basic recovery behaviors (clear, rotate, back up)
- ✅ <50% CPU usage (navigation stack)

**MVP Exclusions:**
- 1km missions (can limit to 500m)
- Advanced recovery behaviors
- Dynamic obstacle prediction (can use basic avoidance)

---

### 10.2 Production Ready

**Production Definition:** Robust, efficient, all-weather autonomous navigation meeting all critical + high requirements.

**Production Requirements:**
- ✅ All Critical (68) + All High (59) requirements satisfied = 127 total
- ✅ ±10cm accuracy over 500m, ±20cm over 1km
- ✅ >95% mission success rate (1000 test missions)
- ✅ 0.8 m/s average mission speed
- ✅ Multi-waypoint missions (up to 10 waypoints, 1km total)
- ✅ Dynamic obstacle prediction and avoidance
- ✅ Advanced recovery behaviors (>90% success)
- ✅ <35% CPU usage (navigation stack)
- ✅ Nighttime and light rain operation

---

## 11. Traceability

### 11.1 Parent Requirements Traceability

This document derives requirements from:

**SYSTEM_REQUIREMENTS.md (NAV-REQ):**
| System Requirement | Derived Navigation Requirements | Status |
|--------------------|-------------------------------|--------|
| NAV-REQ-002 (±10cm over 500m) | NAV-PERF-ACC-001, NAV-LOC-NDT-003 | Traced |
| NAV-REQ-003 (NDT localization) | NAV-LOC-NDT-001 to 008 | Traced |
| NAV-REQ-004 (Multi-waypoint) | NAV-WP-MANAGE-001, NAV-WP-EXEC-001 | Traced |
| NAV-REQ-005 (Nav2 integration) | NAV-INT-NAV2-001 to 006 | Traced |
| NAV-REQ-006 (500m-1km range) | NAV-WP-MISSION-001 | Traced |
| NAV-REQ-007 (A*/Dijkstra) | NAV-PLAN-GLOBAL-003 | Traced |
| NAV-REQ-008 (Collision avoidance) | NAV-PLAN-DYN-004, NAV-PLAN-COST-004 | Traced |
| NAV-REQ-009 (Recovery behaviors) | NAV-EXEC-RECOVERY-001 to 007 | Traced |

**All system-level navigation requirements traced to subsystem requirements.** ✅

---

## 12. Change History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-15 | Navigation Lead | Initial navigation requirements based on ParcelPal NDT architecture |

---

**Document Status:** Draft
**Next Review:** After Nav2 configuration and initial mapping
**Approvals Required:** Navigation Lead, System Architect, Localization Engineer
