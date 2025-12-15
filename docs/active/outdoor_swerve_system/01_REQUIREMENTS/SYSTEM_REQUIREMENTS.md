# System Requirements - Wheelchair Transport Robot

**Project:** Outdoor-First Wheelchair Transport Robot with Swerve Drive
**Document Type:** System Requirements Specification (SRS)
**Status:** Draft v1.0
**Date:** December 15, 2025
**Reference Architecture:** ParcelPal delivery robot (Autoware-based)

---

## Document Purpose

This document specifies the complete functional and non-functional requirements for the wheelchair transport robot system. Requirements are organized by subsystem following separation of concerns principles to enable parallel development.

---

## 1. Functional Requirements

### 1.1 Autonomous Docking (DOCK-REQ)

| ID | Requirement | Priority | Validation Method |
|----|-------------|----------|-------------------|
| DOCK-001 | System SHALL autonomously detect wheelchair within 10m using LiDAR | Critical | Integration test |
| DOCK-002 | System SHALL detect ArUco markers at 0.5-5m range | Critical | Unit test |
| DOCK-003 | System SHALL achieve ±2-5mm docking precision outdoors | Critical | Field test (100 trials) |
| DOCK-004 | System SHALL achieve ±1mm docking precision indoors (target) | High | Lab test |
| DOCK-005 | System SHALL execute two-phase docking (coarse nav + fine servoing) | Critical | Integration test |
| DOCK-006 | System SHALL confirm mechanical coupling before marking attached | Critical | Unit test |
| DOCK-007 | System SHALL NOT attempt docking on slopes >5° | Critical | Safety test |
| DOCK-008 | System SHALL retry docking up to 3 times on failure | Medium | Integration test |
| DOCK-009 | System SHALL abort docking if unable after 3 attempts | Critical | Failure test |
| DOCK-010 | System SHALL maintain ArUco marker tracking at 10Hz minimum | High | Performance test |

**Success Criteria:** >95% docking success rate in outdoor conditions

### 1.2 Autonomous Navigation (NAV-REQ)

| ID | Requirement | Priority | Validation Method |
|----|-------------|----------|-------------------|
| NAV-001 | System SHALL navigate autonomously using saved map + NDT localization | Critical | Integration test |
| NAV-002 | System SHALL achieve ±10cm position accuracy over 500m (no GPS) | Critical | Field test |
| NAV-003 | System SHALL support multi-waypoint missions (up to 10 stops) | High | Integration test |
| NAV-004 | System SHALL replan path within 2 seconds of obstacle detection | Critical | Performance test |
| NAV-005 | System SHALL operate on slopes up to 10° with 100kg load | Critical | Field test |
| NAV-006 | System SHALL detect and avoid static obstacles (walls, curbs) | Critical | Safety test |
| NAV-007 | System SHALL detect and avoid dynamic obstacles (people, vehicles) | Critical | Safety test |
| NAV-008 | System SHALL maintain 1.5m clearance from pedestrians | Critical | Safety test |
| NAV-009 | System SHALL reduce speed to 0.5 m/s in crowded areas | High | Field test |
| NAV-010 | System SHALL return to charging station when battery <30% | Critical | Integration test |
| NAV-011 | System SHALL handle indoor/outdoor transitions automatically | High | Field test |
| NAV-012 | System SHALL navigate 500m-1km range reliably | Critical | Endurance test |

**Success Criteria:** Zero collisions in 1000km test distance

### 1.3 Passenger Safety (SAFE-REQ)

| ID | Requirement | Priority | Validation Method |
|----|-------------|----------|-------------------|
| SAFE-001 | System SHALL detect passenger presence before transport | Critical | Unit test |
| SAFE-002 | System SHALL NOT exceed 1.0 m/s with passenger onboard | Critical | Safety test |
| SAFE-003 | System SHALL limit acceleration to 0.5 m/s² with passenger | Critical | Comfort test |
| SAFE-004 | System SHALL stop within 1.5m when emergency button pressed | Critical | Safety test |
| SAFE-005 | System SHALL respond to emergency stop within 100ms | Critical | Performance test |
| SAFE-006 | System SHALL NOT pick up wheelchair on slopes >5° | Critical | Safety test |
| SAFE-007 | System SHALL monitor tilt angle continuously (10Hz minimum) | Critical | Performance test |
| SAFE-008 | System SHALL stop if tilt exceeds 15° unexpectedly | Critical | Safety test |
| SAFE-009 | System SHALL verify wheelchair attachment before moving | Critical | Unit test |
| SAFE-010 | System SHALL monitor occupant presence during transport | Critical | Safety test |
| SAFE-011 | System SHALL provide audible warnings before movement | High | Field test |
| SAFE-012 | System SHALL enforce geofencing (keep-out zones) | High | Integration test |
| SAFE-013 | System SHALL fail safe (stop) on sensor failures | Critical | Failure test |
| SAFE-014 | System SHALL have redundant emergency stop (HW + SW) | Critical | Safety test |

**Success Criteria:** Zero safety incidents in 1000 transport missions

### 1.4 Manual Control (MANUAL-REQ)

| ID | Requirement | Priority | Validation Method |
|----|-------------|----------|-------------------|
| MANUAL-001 | System SHALL support joystick teleoperation | High | User test |
| MANUAL-002 | System SHALL allow manual override of autonomous mode | Critical | Integration test |
| MANUAL-003 | System SHALL limit manual speed to 1.0 m/s | High | Safety test |
| MANUAL-004 | System SHALL record waypoints in teaching mode | Medium | Integration test |
| MANUAL-005 | System SHALL switch to manual mode within 500ms of request | High | Performance test |

### 1.5 User Interface (UI-REQ)

| ID | Requirement | Priority | Validation Method |
|----|-------------|----------|-------------------|
| UI-001 | System SHALL provide touch screen interface for wheelchair booking | High | User test |
| UI-002 | System SHALL support English, Japanese, and Chinese languages | High | User test |
| UI-003 | System SHALL display real-time robot status and location | High | Integration test |
| UI-004 | System SHALL show ETA during transport | Medium | Integration test |
| UI-005 | System SHALL provide eHMI (LED + audio) for pedestrian awareness | Critical | Field test |
| UI-006 | System SHALL announce state changes audibly (multilingual) | High | User test |
| UI-007 | System SHALL display battery level and charging status | Medium | Unit test |

### 1.6 System Management (SYS-REQ)

| ID | Requirement | Priority | Validation Method |
|----|-------------|----------|-------------------|
| SYS-001 | System SHALL log all missions to persistent storage | High | Unit test |
| SYS-002 | System SHALL monitor sensor health continuously | Critical | Unit test |
| SYS-003 | System SHALL support remote monitoring via dashboard | Medium | Integration test |
| SYS-004 | System SHALL report diagnostics (battery, sensors, motors) | High | Unit test |
| SYS-005 | System SHALL support firmware updates (OTA) | Low | Integration test |

---

## 2. Non-Functional Requirements

### 2.1 Performance (PERF-REQ)

| ID | Requirement | Priority | Validation Method |
|----|-------------|----------|-------------------|
| PERF-001 | System SHALL achieve 4+ hours battery life with passenger transport | High | Endurance test |
| PERF-002 | System SHALL complete docking in <60 seconds (approach to attached) | High | Performance test |
| PERF-003 | System SHALL process LiDAR data at 10Hz minimum | Critical | Performance test |
| PERF-004 | System SHALL process camera frames at 15Hz minimum | High | Performance test |
| PERF-005 | System SHALL respond to obstacles within 0.5 seconds | Critical | Performance test |
| PERF-006 | System SHALL boot to operational state in <90 seconds | Medium | Unit test |
| PERF-007 | System SHALL maintain CPU usage <70% during normal operation | Medium | Performance test |
| PERF-008 | System SHALL maintain localization update rate 10Hz | Critical | Performance test |

### 2.2 Reliability (REL-REQ)

| ID | Requirement | Priority | Validation Method |
|----|-------------|----------|-------------------|
| REL-001 | System SHALL achieve >98% uptime (excluding maintenance) | High | Field test (30 days) |
| REL-002 | System SHALL recover from sensor glitches automatically | High | Failure test |
| REL-003 | System SHALL gracefully degrade on non-critical sensor failure | High | Failure test |
| REL-004 | System SHALL detect and report hardware failures | Critical | Unit test |
| REL-005 | System SHALL maintain saved map integrity (checksums) | High | Unit test |

### 2.3 Environmental (ENV-REQ)

| ID | Requirement | Priority | Validation Method |
|----|-------------|----------|-------------------|
| ENV-001 | System SHALL operate in light rain (IP54+ weatherproofing) | Critical | Environmental test |
| ENV-002 | System SHALL operate in temperature range -5°C to 40°C | High | Environmental test |
| ENV-003 | System SHALL operate in daylight and nighttime conditions | Critical | Field test |
| ENV-004 | System SHALL handle outdoor lighting variations | High | Field test |
| ENV-005 | System SHALL operate on grass, asphalt, concrete, tile | Critical | Field test |
| ENV-006 | System SHALL handle uneven terrain (±20mm irregularities) | High | Field test |
| ENV-007 | System SHALL NOT operate in heavy rain or snow | Critical | Documentation |

### 2.4 Safety Standards (STD-REQ)

| ID | Requirement | Priority | Validation Method |
|----|-------------|----------|-------------------|
| STD-001 | System SHALL comply with ISO 13482 (personal care robots) | Critical | Certification |
| STD-002 | System SHALL comply with IEC 61508 (functional safety) | Critical | Certification |
| STD-003 | System SHALL implement Safety Integrity Level (SIL) 2 | Critical | Design review |
| STD-004 | System SHALL undergo FMEA (Failure Mode Effects Analysis) | High | Design review |
| STD-005 | System SHALL have emergency stop accessible from all sides | Critical | Design review |

### 2.5 Maintainability (MAINT-REQ)

| ID | Requirement | Priority | Validation Method |
|----|-------------|----------|-------------------|
| MAINT-001 | System SHALL require <2 hours/week maintenance average | High | Field test (30 days) |
| MAINT-002 | System SHALL support modular component replacement | Medium | Design review |
| MAINT-003 | System SHALL provide diagnostic logs for troubleshooting | High | Unit test |
| MAINT-004 | System SHALL alert operators to maintenance needs | Medium | Integration test |

---

## 3. Hardware Requirements Summary

| Component | Specification | Rationale |
|-----------|--------------|-----------|
| **Drive System** | Swerve drive (4 modules, 6.5" wheels, 80W/wheel) | Best outdoor performance, omnidirectional |
| **Compute** | GMKtec Nucbox K6 (Ryzen 7 7840HS, 32GB, Radeon 780M) | Sufficient for CPU-based processing |
| **3D LiDAR** | Outdoor-grade, IP65+, 360°, 0.1-30m range (TBD) | Primary navigation sensor |
| **Cameras** | 2× RGB cameras, minimum 720p, 30fps | ArUco detection + visual awareness |
| **IMU** | 9-DOF, 100Hz update rate | Tilt compensation, orientation |
| **ArUco Markers** | 4-8 markers (150mm × 150mm) at docking stations | Precision docking |
| **Emergency Stop** | Hardware button (accessible all sides) | Critical safety |
| **Battery** | Lithium-ion, 4+ hours runtime (TBD Ah) | Operational requirement |
| **Weatherproofing** | IP54+ enclosure | Light rain operation |

**Note:** GPS explicitly excluded (cost savings, NDT localization sufficient for <1km range)

---

## 4. Software Requirements Summary

| Component | Technology | Version | Purpose |
|-----------|-----------|---------|---------|
| **OS** | Ubuntu | 22.04 LTS | ROS 2 Humble support |
| **Framework** | ROS 2 | Humble | Robot middleware |
| **Navigation** | Autoware + Nav2 | Latest stable | Proven outdoor navigation |
| **Localization** | NDT scan matcher | Via SLAM Toolbox | Drift-free localization |
| **SLAM** | SLAM Toolbox / Cartographer | Latest | Manual mapping (offline) |
| **Perception** | PCL | 1.12+ | LiDAR processing |
| **Vision** | OpenCV | 4.x | ArUco detection |
| **UI** | React + Next.js | 14.x | Touch screen interface |
| **eHMI** | Arduino (ESP32-S3) + Dezyne | Latest | LED/audio external HMI |

---

## 5. Operational Requirements

### 5.1 Deployment Environment

| Aspect | Requirement |
|--------|-------------|
| **Operational Range** | 500m-1km (battery and map size dependent) |
| **Map Size** | Up to 1km² coverage per map |
| **Multi-Map Support** | Yes (switch maps for different areas) |
| **Operational Hours** | 24/7 capable (with charging schedule) |
| **Charging Time** | <4 hours to 80%, <6 hours to 100% (TBD) |
| **Concurrent Robots** | Support 1-5 robots in same area (future) |

### 5.2 Mission Types

| Mission Type | Description | Requirements |
|--------------|-------------|--------------|
| **Single Transport** | Pickup A → Dropoff A → Return to base | DOCK + NAV + SAFE |
| **Multi-Stop Shuttle** | [Pickup A → Dropoff A] × N → Charge | NAV-003, SYS-001 |
| **Scheduled Service** | Predefined routes at set times | SYS integration |
| **On-Demand** | User-requested via UI/app | UI-001, UI-002 |
| **Teaching Mode** | Human-guided waypoint recording | MANUAL-004 |

---

## 6. Requirements Traceability

### 6.1 Subsystem Requirements Mapping

| Subsystem | Requirements | Documents |
|-----------|-------------|-----------|
| **Swerve Drive** | PERF-001, PERF-003, ENV-005, ENV-006 | SWERVE_DRIVE_REQUIREMENTS.md |
| **Docking** | DOCK-001 to DOCK-010 | DOCKING_SYSTEM_REQUIREMENTS.md |
| **Navigation** | NAV-001 to NAV-012 | NAVIGATION_REQUIREMENTS.md |
| **Perception** | NAV-006, NAV-007, DOCK-002, PERF-003, PERF-004 | PERCEPTION_REQUIREMENTS.md |
| **Safety** | SAFE-001 to SAFE-014, STD-001 to STD-005 | SAFETY_REQUIREMENTS.md |
| **UI/eHMI** | UI-001 to UI-007 | INTERFACE_REQUIREMENTS.md |

### 6.2 Priority Distribution

| Priority | Count | Percentage |
|----------|-------|------------|
| **Critical** | 47 | 51% |
| **High** | 32 | 35% |
| **Medium** | 11 | 12% |
| **Low** | 2 | 2% |
| **Total** | 92 | 100% |

---

## 7. Requirements Validation Approach

### 7.1 Validation Methods

| Method | Description | Requirements Covered |
|--------|-------------|---------------------|
| **Unit Test** | Component-level testing | 18 requirements |
| **Integration Test** | Subsystem interaction testing | 24 requirements |
| **Field Test** | Real-world outdoor/indoor testing | 22 requirements |
| **Safety Test** | Safety-critical feature validation | 16 requirements |
| **Performance Test** | Timing, throughput, resource usage | 10 requirements |
| **User Test** | User experience and usability | 5 requirements |
| **Certification** | Standards compliance | 3 requirements |

### 7.2 Acceptance Criteria

**Minimum Viable Product (MVP):**
- All CRITICAL requirements (47) must pass validation
- ≥80% HIGH requirements (26/32) must pass validation
- Zero safety incidents in testing
- >95% docking success rate (outdoor)
- >98% navigation success rate (no collisions)

**Production Ready:**
- 100% CRITICAL + HIGH requirements validated
- ≥50% MEDIUM + LOW requirements validated
- Field testing: 1000 missions or 30 days operation
- Customer acceptance testing passed

---

## 8. Requirements Changes

### 8.1 Change Management

All requirement changes must follow this process:
1. Change request submitted with rationale
2. Impact analysis (technical, schedule, cost)
3. Stakeholder review and approval
4. Document update with version increment
5. Affected subsystems notified
6. Requirements traceability matrix updated

### 8.2 Open Items / TBD

| Item | Status | Owner | Target Date |
|------|--------|-------|-------------|
| 3D LiDAR model selection | TBD | Hardware team | Sprint 2 |
| Battery capacity (Ah) and charging specs | TBD | Hardware team | Sprint 2 |
| Camera resolution and model | TBD | Perception team | Sprint 3 |
| Wheelchair booking system integration | TBD | Product team | Sprint 8 |
| Fleet management (multi-robot) | Future | System architect | Phase 2 |
| Weather sensor integration | Future | Sensor team | Phase 2 |

---

## 9. Related Documents

**Reference Materials:**
- `../reference/PARCELPAL_EXPLORATION_SUMMARY.md` - ParcelPal proven architecture
- `../reference/SWERVE_DRIVE_SYSTEM_TASK.md` - Original task specification
- `../reference/question_answers.md` - Senior review clarifications

**Subsystem Requirements:**
- `SWERVE_DRIVE_REQUIREMENTS.md` - Drive system detailed requirements
- `DOCKING_SYSTEM_REQUIREMENTS.md` - Docking precision requirements
- `NAVIGATION_REQUIREMENTS.md` - Navigation and localization requirements
- `PERCEPTION_REQUIREMENTS.md` - Sensor and vision requirements
- `SAFETY_REQUIREMENTS.md` - Safety-critical requirements
- `INTERFACE_REQUIREMENTS.md` - UI and communication requirements

**Traceability:**
- `REQUIREMENTS_TRACEABILITY_MATRIX.md` - Requirements to implementation mapping

---

## 10. Glossary

| Term | Definition |
|------|------------|
| **ArUco** | OpenCV fiducial marker system for visual positioning |
| **Autoware** | Open-source autonomous driving software (tier4_autoware) |
| **NDT** | Normal Distributions Transform (scan matching algorithm) |
| **ParcelPal** | Reference delivery robot (FutureADS, Autoware-based) |
| **SLAM** | Simultaneous Localization and Mapping |
| **Swerve Drive** | 4-wheel independent steering and drive system |
| **Visual Servoing** | Control using real-time visual feedback |

---

**Document Status:** Draft v1.0
**Review Status:** Pending stakeholder review
**Next Review:** After Phase 1 Sprint 2
**Approvers:** System Architect, Safety Lead, Product Owner
**Version History:**
- v1.0 (2025-12-15): Initial requirements specification
