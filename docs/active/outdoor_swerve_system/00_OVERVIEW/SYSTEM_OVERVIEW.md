# System Overview - Outdoor-First Wheelchair Transport Robot

**Project Name:** Wheelchair Transport Robot with Swerve Drive
**Target Environment:** Outdoor-first with indoor compatibility
**Development Approach:** Agile methodology with modular architecture
**Reference Architecture:** Based on proven ParcelPal delivery robot (Autoware framework)
**Document Date:** December 15, 2025
**Version:** 1.0

---

## Executive Summary

This document provides a comprehensive overview of an autonomous wheelchair transport robot designed for **outdoor operation** with full indoor compatibility. The system uses a **swerve drive platform** for superior outdoor performance, **precision docking** (±2-5mm) using ArUco visual markers, and proven **Autoware-based navigation** without GPS dependency.

**Key Innovation:** Outdoor-first design using manual SLAM mapping + NDT scan matcher localization (proven ParcelPal approach) enables reliable 500m-1km operation without expensive RTK GPS.

---

## 1. Project Overview

### 1.1 Objective

Develop a complete autonomous wheelchair transport robot capable of:
- ✅ **Autonomous wheelchair docking** with precision attachment
- ✅ **Safe passenger transport** from pickup to destination
- ✅ **Outdoor navigation** (slopes up to 10°, light rain, rough terrain)
- ✅ **Indoor compatibility** (smooth floors, controlled environment)
- ✅ **Multi-stop shuttle service** (500m-1km operational range)
- ✅ **Manual control fallback** (joystick teleoperation)

### 1.2 Primary Use Cases

| Use Case | Description | Priority |
|----------|-------------|----------|
| **Wheelchair Docking** | Autonomous approach and precision docking (±2-5mm) | Critical |
| **Passenger Transport** | A→B navigation with person onboard, smooth ride | Critical |
| **Multi-Stop Shuttle** | Sequential waypoint navigation, scheduled service | High |
| **Charging Station Return** | Auto-return on low battery (<30%) | High |
| **Manual Override** | Joystick teleoperation, teaching mode | Medium |
| **Emergency Return** | Safe navigation to base on sensor failure | Medium |

### 1.3 Operational Environment

**Primary:** Outdoor environments
- Campus pathways, sidewalks, parking lots
- Weather: Light rain operation (IP54+ weatherproofing)
- Terrain: Slopes up to 10° (17.6% grade), rough surfaces
- Range: 500m-1km with multi-stop routing

**Secondary:** Indoor environments
- Hospital corridors, shopping mall floors
- Smooth surfaces, controlled temperature
- Higher precision achievable (±1mm docking target)

---

## 2. System Architecture

### 2.1 High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│              Wheelchair Transport Robot System                  │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │         Wheelchair Master (Main Orchestration)           │  │
│  │         • State machine coordination                     │  │
│  │         • Mission planning                               │  │
│  │         • Safety monitoring                              │  │
│  └─────────────┬────────────────────────────────────────────┘  │
│                │                                                │
│    ┌───────────┼───────────┬──────────────┬──────────────┐    │
│    ▼           ▼           ▼              ▼              ▼     │
│ ┌──────┐  ┌────────┐  ┌─────────┐  ┌──────────┐  ┌────────┐  │
│ │  UI  │  │Autoware│  │ Swerve  │  │ Docking  │  │  eHMI  │  │
│ │(Touch│  │  Nav   │  │  Drive  │  │ Subsystem│  │(ESP32) │  │
│ │Screen│  │ Stack  │  │Controller│  │(ArUco)   │  │        │  │
│ └──────┘  └────────┘  └─────────┘  └──────────┘  └────────┘  │
│                │                                                │
│                ▼                                                │
│         ┌─────────────┐                                        │
│         │   Sensors   │                                        │
│         │ LiDAR+Cameras│                                       │
│         │  +IMU       │                                        │
│         └─────────────┘                                        │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 Core Subsystems

#### **1. Swerve Drive System** (NEW)
- 4 independent wheel modules (6.5" hoverboard in-wheel motors)
- Inverse kinematics for omnidirectional motion
- Superior outdoor performance vs. mecanum wheels
- ROS 2 controller plugins for Nav2 integration

#### **2. Wheelchair Docking Subsystem** (NEW)
- ArUco marker detection (OpenCV)
- Two-phase docking: coarse navigation + fine visual servoing
- Precision: ±2-5mm outdoor, ±1mm indoor target
- Mechanical coupling with confirmation sensors

#### **3. Autoware Navigation Stack** (REUSED - ParcelPal)
- Manual SLAM mapping once (offline with loop closure)
- NDT scan matcher localization (online, no drift)
- Nav2 path planning and obstacle avoidance
- Range: 500m-1km without GPS

#### **4. Perception System** (REUSED - ParcelPal)
- 3D LiDAR (outdoor-grade, IP65+, model TBD)
- Dual RGB cameras (ArUco detection + obstacle awareness)
- 9-DOF IMU (orientation, tilt compensation)
- Binary obstacle detection (no deep learning required)

#### **5. Safety System** (EXTENDED - ParcelPal + Wheelchair)
- Passenger safety monitoring (occupant detection)
- Emergency stop (hardware button + software)
- No pickup/dropoff on slopes >5°
- Speed limiting with passenger onboard
- Geofencing and keep-out zones

#### **6. User Interface** (ADAPTED - ParcelPal)
- Touch screen (React + Next.js, wheelchair booking)
- eHMI (ESP32-S3: LED strips + LED matrix + audio)
- Multi-language support (EN, JP, ZH)
- Operator dashboard (mission monitoring)

---

## 3. Key Technical Specifications

### 3.1 Hardware Platform

| Component | Specification | Rationale |
|-----------|--------------|-----------|
| **Drive System** | Swerve drive (4 modules) | Best outdoor performance, omnidirectional |
| **Wheels** | 6.5" φ165mm hoverboard in-wheel motors | Proven, available, 80W per wheel |
| **Compute** | GMKtec Nucbox K6 (Ryzen 7 7840HS, 32GB, Radeon 780M) | Sufficient for CPU-based processing, no deep learning GPU needed |
| **3D LiDAR** | Outdoor-grade, IP65+ (model TBD) | Primary sensor for navigation and obstacle detection |
| **Cameras** | 2× RGB cameras (resolution TBD) | ArUco marker detection, visual awareness |
| **IMU** | 9-DOF (MPU-9250 or similar) | Tilt compensation, orientation |
| **GPS** | ❌ Not included | Cost savings, NDT localization sufficient for <1km range |
| **ArUco Markers** | 4-8 markers at docking stations | Precision docking, OpenCV-based detection |
| **Weatherproofing** | IP54+ enclosure | Light rain operation |

### 3.2 Performance Targets

| Parameter | Outdoor | Indoor | Notes |
|-----------|---------|--------|-------|
| **Docking Precision** | ±2-5mm | ±1mm (target) | ArUco visual servoing |
| **Navigation Range** | 500m-1km | Unlimited | Limited by battery, not localization |
| **Max Slope** | 10° (17.6%) | 15° (capable) | Design limit with 100kg load |
| **Max Speed** | 1.0 m/s | 1.5 m/s | Passenger comfort limit |
| **Localization Accuracy** | ±10cm position | ±5cm position | NDT scan matcher |
| **Obstacle Detection Range** | 0.1-30m | 0.1-15m | LiDAR + cameras |
| **Battery Life** | 4-6 hours | 6-8 hours | Depends on terrain, load |

### 3.3 Software Stack

| Layer | Technology | Purpose |
|-------|-----------|---------|
| **Framework** | ROS 2 Humble (Ubuntu 22.04) | Robot operating system |
| **Navigation** | Autoware + Nav2 | Proven outdoor navigation stack |
| **Localization** | NDT scan matcher (SLAM Toolbox) | Drift-free localization on saved map |
| **SLAM** | Manual mapping once (offline) | Loop closure, saved map for operations |
| **Perception** | PCL (Point Cloud Library) | LiDAR processing, CPU-based |
| **Vision** | OpenCV 4.x | ArUco detection, camera processing |
| **Docking** | Custom ROS 2 node (visual servoing) | Precision docking controller |
| **Swerve Drive** | Custom ROS 2 controller plugin | Inverse kinematics, Nav2 integration |
| **UI** | React + Next.js | Touch screen interface |
| **eHMI** | Arduino (ESP32-S3) + Dezyne | LED/audio external HMI |

---

## 4. Operational Workflow

### 4.1 Wheelchair Docking Sequence

```
1. IDLE → Passenger calls via UI/app
   └─> Robot receives pickup location

2. APPROACHING_WHEELCHAIR → Navigate to approach zone
   └─> Use saved map + NDT localization

3. SEARCHING_ARUCO → Detect ArUco marker on wheelchair
   └─> Switch from navigation to visual servoing

4. DOCKING → Precision approach (±2-5mm)
   └─> Visual servoing controller + mechanical coupling

5. WHEELCHAIR_ATTACHED → Confirm attachment
   └─> Occupant detection sensor active
   └─> Audio: "Wheelchair attached successfully"
```

### 4.2 Passenger Transport Sequence

```
6. TRANSPORTING → Navigate to destination
   └─> Passenger onboard, reduced speed, comfort priority
   └─> eHMI: "Passenger onboard, keep distance"

7. ARRIVED_DESTINATION → Arrive at dropoff location
   └─> Audio: "Arrived at destination"
   └─> Wait for confirmation to undock

8. UNDOCKING → Release mechanical coupling
   └─> Reverse docking procedure

9. WHEELCHAIR_DETACHED → Confirm release
   └─> Audio: "Wheelchair detached, ready"
   └─> Return to IDLE or next mission
```

### 4.3 Multi-Stop Shuttle Service

```
Mission: [Pickup A] → [Dropoff A] → [Pickup B] → [Dropoff B] → [Charging Station]

• Sequential waypoint navigation
• Queue management (multiple passengers)
• Battery monitoring (return to charge if <30%)
• Scheduled routes (e.g., hourly shuttle)
```

---

## 5. Design Principles

### 5.1 Outdoor-First Philosophy

**All requirements prioritize outdoor operation:**
- Hardware: Weatherproof, outdoor-grade sensors
- Software: Robust to GPS loss, lighting changes, terrain variation
- Performance: Validated on slopes, rough surfaces, weather conditions
- **Indoor becomes easier:** Better localization, faster speeds, tighter tolerances

### 5.2 Separation of Concerns (Agile-Ready)

**Each subsystem is independently:**
- Documented (requirements, architecture, design)
- Developable (parallel team development)
- Testable (unit + integration + system tests)
- Integrable (clear ROS 2 interfaces)

**Subsystems:**
1. Swerve drive controller
2. Wheelchair docking subsystem
3. Autoware navigation stack
4. Perception subsystem
5. Safety subsystem
6. User interface + eHMI

### 5.3 Based on Proven Architecture (ParcelPal)

**Reuse 60-70% of ParcelPal delivery robot:**
- ✅ Autoware navigation stack (100% reuse)
- ✅ NDT localization approach (100% reuse)
- ✅ UI framework pattern (70% reuse)
- ✅ eHMI controller pattern (90% reuse)
- ✅ Main orchestration pattern (80% reuse)

**New development focus:**
- Swerve drive controller (0% - new)
- Wheelchair docking subsystem (0% - new)
- Passenger safety monitoring (0% - new)

---

## 6. Development Roadmap (High-Level)

### Phase 1: Foundation (Sprints 1-4)
- Autoware + NDT localization setup
- Manual SLAM mapping of test environment
- Basic swerve drive controller
- System integration skeleton

### Phase 2: Docking System (Sprints 5-8)
- ArUco marker detection
- Visual servoing controller
- Mechanical coupling design
- Docking precision testing

### Phase 3: Safety & Transport (Sprints 9-12)
- Passenger safety monitoring
- Enhanced safety features
- Transport mode testing
- Multi-stop routing

### Phase 4: UI/UX & Polish (Sprints 13-16)
- Touch screen UI
- eHMI implementation
- Multi-language support
- User acceptance testing

### Phase 5: Field Testing (Sprints 17-20)
- Outdoor testing (various weather, terrain)
- Indoor compatibility validation
- Performance optimization
- Production readiness

---

## 7. Key Differentiators vs. Existing Systems

| Feature | This System | Typical Indoor Wheelchair Robot | Advantage |
|---------|-------------|-------------------------------|-----------|
| **Environment** | Outdoor-first + indoor | Indoor only | Real-world deployment flexibility |
| **Localization** | Manual SLAM + NDT (no GPS) | GPS or beacon-based | Cost savings, <1km reliable |
| **Drive System** | Swerve drive | Mecanum or differential | Superior outdoor performance |
| **Docking Precision** | ±2-5mm outdoor | ±10cm typical | Reliable attachment in all conditions |
| **Reference Arch** | Proven ParcelPal base | Custom from scratch | Lower risk, faster development |
| **Compute** | GMKtec Nucbox K6 (integrated GPU) | Expensive embedded GPU | Cost-effective, sufficient performance |

---

## 8. Risk Mitigation

### 8.1 Technical Risks

| Risk | Mitigation | Status |
|------|-----------|--------|
| **Outdoor localization drift** | Manual SLAM + NDT (ParcelPal proven) | ✅ Validated |
| **Docking precision outdoors** | ArUco + visual servoing | ⚠️ Requires testing |
| **Swerve drive control complexity** | Incremental development, ROS 2 plugins | ⚠️ New development |
| **Weather impact on sensors** | IP65+ LiDAR, camera lens heating | ⚠️ Testing needed |
| **Battery life on slopes** | Conservative power budget, 4hr target | ⚠️ Validation needed |

### 8.2 Safety Risks

| Risk | Mitigation | Status |
|------|-----------|--------|
| **Passenger falls during transport** | Occupant detection, emergency stop, speed limits | 🔄 Design phase |
| **Collision with pedestrians** | LiDAR obstacle detection, eHMI warnings | ✅ ParcelPal proven |
| **Slope stability with load** | 10° design limit, tilt sensor cutoff | 🔄 Testing needed |
| **Docking on slopes** | No docking allowed >5° slope | ✅ Enforced |

---

## 9. Success Criteria

### 9.1 Technical Success

- ✅ Docking success rate >95% (outdoor conditions)
- ✅ Localization accuracy ±10cm over 500m (no GPS)
- ✅ Obstacle avoidance: zero collisions in testing
- ✅ Battery life: 4+ hours with passenger transport
- ✅ Uptime: >98% (excluding maintenance)

### 9.2 Operational Success

- ✅ Passenger satisfaction: >4.5/5 rating (comfort, reliability)
- ✅ Staff satisfaction: Operator dashboard usability >4/5
- ✅ Deployment: 90% outdoor, 10% indoor (validates outdoor-first)
- ✅ Maintenance: <2 hours/week average

---

## 10. Related Documents

**Reference Materials:**
- `reference/PARCELPAL_EXPLORATION_SUMMARY.md` - ParcelPal architecture analysis
- `reference/EHMI_SYSTEM_REFERENCE.md` - eHMI implementation details
- `reference/SWERVE_DRIVE_SYSTEM_TASK.md` - Original task specification
- `reference/question_answers.md` - Senior review Q&A

**Requirements:**
- `01_REQUIREMENTS/SYSTEM_REQUIREMENTS.md` - Complete system requirements
- `01_REQUIREMENTS/SWERVE_DRIVE_REQUIREMENTS.md` - Drive system specs
- `01_REQUIREMENTS/DOCKING_SYSTEM_REQUIREMENTS.md` - Docking precision specs

**Architecture:**
- `02_ARCHITECTURE/OVERALL_SYSTEM_ARCHITECTURE.md` - Detailed architecture
- `02_ARCHITECTURE/SWERVE_DRIVE_ARCHITECTURE.md` - Drive system design
- `02_ARCHITECTURE/DOCKING_SUBSYSTEM_ARCHITECTURE.md` - Docking design

**Development:**
- `05_DEVELOPMENT/AGILE_ROADMAP.md` - Sprint planning and timeline
- `05_DEVELOPMENT/USER_STORIES.md` - Feature descriptions from user perspective

---

## 11. Glossary

| Term | Definition |
|------|------------|
| **Autoware** | Open-source autonomous driving software (tier4_autoware) |
| **ArUco** | OpenCV-based fiducial marker system for visual positioning |
| **eHMI** | External Human-Machine Interface (LED/audio for pedestrians) |
| **GMKtec Nucbox K6** | Mini PC (Ryzen 7 7840HS, 32GB RAM, Radeon 780M GPU) |
| **NDT** | Normal Distributions Transform (scan matching algorithm) |
| **Nav2** | ROS 2 navigation framework |
| **ParcelPal** | Reference delivery robot using Autoware (FutureADS project) |
| **SLAM** | Simultaneous Localization and Mapping |
| **Swerve Drive** | 4-wheel independent steering and drive system |
| **TVM** | Total Vehicle Management (log upload service, separate from robot) |
| **Visual Servoing** | Control using real-time visual feedback |

---

**Document Status:** Complete
**Next Review:** After Phase 1 Sprint 1 completion
**Maintained By:** System Architect
**Version History:**
- v1.0 (2025-12-15): Initial system overview document
