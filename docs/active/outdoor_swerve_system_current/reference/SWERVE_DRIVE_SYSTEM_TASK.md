# Swerve Drive System Development Task

**Project:** Outdoor-First Autonomous Robot with Indoor Compatibility
**Hardware Platform:** Swerve Drive System
**Development Methodology:** Agile
**Documentation Date:** December 15, 2025
**Status:** Planning & Requirements Phase

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Hardware Specifications](#hardware-specifications)
3. [System Requirements](#system-requirements)
4. [Documentation Structure](#documentation-structure)
5. [Complete System Scope](#complete-system-scope)
6. [Technology Stack](#technology-stack)
7. [Development Principles](#development-principles)

---

## Project Overview

### Objective
Develop a **complete end-to-end autonomous robot system** with swerve drive locomotion, capable of operating primarily outdoors while maintaining indoor compatibility.

### Key Capabilities
- ✅ Autonomous wheelchair docking (precision ±2-5mm)
- ✅ Wheelchair transport (person onboard, A→B navigation)
- ✅ Manual control (joystick teleoperation)
- ✅ Autonomous navigation (charging stations, shuttle services, patrol routes)
- ✅ Indoor/outdoor operation with automatic environment adaptation

### Methodology
**Agile Development** with modular, separation-of-concerns architecture enabling parallel subsystem development and incremental integration.

---

## Hardware Specifications

### Proposed Hardware Configuration

Based on **Swerve Drive Study (2025-12-10)** and system requirements:

| Component | Specification | Source |
|-----------|--------------|--------|
| **Drive System** | Swerve drive (4 independent modules) | Study confirmed |
| **Wheels** | 6.5" (φ165mm) hoverboard in-wheel motors | Study specified |
| **Traction Motors** | 80W per wheel, cantilever design | Study calculated |
| **Steer Motors** | Digital servo (3.5Nm) or power window motor (2.9Nm) | Study options |
| **Primary Sensor** | LiDAR (360°, outdoor-grade) | System requirements |
| **Vision** | 2× RGB cameras (ArUco + object detection) | System requirements |
| **GPS** | RTK GPS (cm-accuracy) | System requirements |
| **IMU** | 9-DOF (tilt/orientation) | System requirements |
| **Visual Markers** | ArUco markers (docking precision) | System requirements |
| **Docking System** | Visual servoing + mechanical coupling | System requirements |
| **Control System** | Embedded PC + GPU | System requirements |
| **Obstacle Sensors** | Proximity sensors, cliff detection | System requirements |
| **Safety Sensor** | Occupant detection, emergency stop | System requirements |

### Design Parameters

| Parameter | Specification | Source |
|-----------|--------------|--------|
| **Robot Weight** | 60kg | Swerve Drive Study |
| **Transported Load** | 100kg (wheelchair + person) | Swerve Drive Study |
| **Maximum Slope** | 10° (17.6% grade) | Swerve Drive Study |
| **Ground Clearance** | 20mm minimum | Swerve Drive Study |
| **Design Speed** | 1.0 m/s (3.6 km/h) | Swerve Drive Study |
| **Docking Precision** | ±2-5mm (outdoor), ±1mm (indoor target) | System requirements |

---

## System Requirements

### Core Functional Requirements

1. **Autonomous Docking**
   - Precision visual servoing using ArUco markers
   - Two-phase approach (coarse navigation + fine alignment)
   - Mechanical coupling mechanism
   - Success rate >95%

2. **ROS 2 Swerve Drive Integration**
   - Custom ROS 2 nodes for swerve drive kinematics
   - Inverse kinematics for 4-wheel independent steering
   - Odometry estimation and sensor fusion
   - Nav2 integration for path planning

3. **Docking-Drive Integration**
   - Seamless transition between navigation and docking modes
   - State machine for mode switching (free/docking/attached)
   - Coordinated control between subsystems

4. **Manual Control**
   - Joystick teleoperation with safety override
   - Teaching mode (human-guided waypoint recording)
   - Emergency stop integration

5. **Autonomous Operations**
   - **Wheelchair transport:** Person onboard, A→B navigation
   - **Charging station navigation:** Auto-return on low battery
   - **Shuttle service:** Scheduled routes, multi-waypoint execution
   - **Patrol routes:** Security, monitoring tasks
   - **Delivery tasks:** Equipment/goods transport

---

## Documentation Structure

### Proposed Folder Organization

```
docs/active/outdoor_swerve_system/
├── 00_OVERVIEW/
│   ├── SYSTEM_OVERVIEW.md
│   ├── GETTING_STARTED.md
│   └── DOCUMENT_INDEX.md
│
├── 01_REQUIREMENTS/
│   ├── SYSTEM_REQUIREMENTS.md
│   ├── HARDWARE_REQUIREMENTS.md
│   ├── SOFTWARE_REQUIREMENTS.md
│   ├── SWERVE_DRIVE_REQUIREMENTS.md
│   ├── DOCKING_SYSTEM_REQUIREMENTS.md
│   ├── NAVIGATION_REQUIREMENTS.md
│   ├── PERCEPTION_REQUIREMENTS.md
│   ├── SAFETY_REQUIREMENTS.md
│   ├── PERFORMANCE_REQUIREMENTS.md
│   ├── INTERFACE_REQUIREMENTS.md
│   └── REQUIREMENTS_TRACEABILITY_MATRIX.md
│
├── 02_ARCHITECTURE/
│   ├── OVERALL_SYSTEM_ARCHITECTURE.md
│   ├── SWERVE_DRIVE_ARCHITECTURE.md
│   ├── DOCKING_SUBSYSTEM_ARCHITECTURE.md
│   ├── NAVIGATION_SUBSYSTEM_ARCHITECTURE.md
│   ├── PERCEPTION_SUBSYSTEM_ARCHITECTURE.md
│   ├── CONTROL_SUBSYSTEM_ARCHITECTURE.md
│   ├── SAFETY_SUBSYSTEM_ARCHITECTURE.md
│   └── INTEGRATION_ARCHITECTURE.md
│
├── 03_DESIGN/
│   ├── SWERVE_DRIVE_DETAILED_DESIGN.md
│   ├── DOCKING_MECHANISM_DESIGN.md
│   ├── SENSOR_FUSION_DESIGN.md
│   ├── LOCALIZATION_DESIGN.md
│   └── STATE_MACHINE_DESIGN.md
│
├── 04_INTERFACES/
│   ├── INTERFACE_CONTROL_DOCUMENT.md
│   ├── ROS_TOPICS_AND_SERVICES.md
│   ├── HARDWARE_INTERFACES.md
│   └── SUBSYSTEM_INTERFACES.md
│
├── 05_DEVELOPMENT/
│   ├── AGILE_ROADMAP.md
│   ├── SPRINT_PLANNING.md
│   ├── USER_STORIES.md
│   └── BACKLOG.md
│
├── 06_TESTING/
│   ├── TEST_STRATEGY.md
│   ├── TEST_PLAN.md
│   └── ACCEPTANCE_CRITERIA.md
│
└── 07_DEPLOYMENT/
    ├── DEPLOYMENT_GUIDE.md
    ├── CONFIGURATION_MANAGEMENT.md
    └── MAINTENANCE_GUIDE.md
```

### Documentation Priorities

**Priority 1: Core Foundation**
1. SYSTEM_OVERVIEW.md
2. SYSTEM_REQUIREMENTS.md
3. OVERALL_SYSTEM_ARCHITECTURE.md
4. AGILE_ROADMAP.md

**Priority 2: Subsystem Requirements (Separation of Concerns)**
5. SWERVE_DRIVE_REQUIREMENTS.md
6. DOCKING_SYSTEM_REQUIREMENTS.md
7. NAVIGATION_REQUIREMENTS.md
8. PERCEPTION_REQUIREMENTS.md

**Priority 3: Subsystem Architecture**
9. SWERVE_DRIVE_ARCHITECTURE.md
10. DOCKING_SUBSYSTEM_ARCHITECTURE.md
11. INTEGRATION_ARCHITECTURE.md

**Priority 4: Interfaces & Integration**
12. INTERFACE_CONTROL_DOCUMENT.md
13. ROS_TOPICS_AND_SERVICES.md

**Priority 5: Development & Testing**
14. USER_STORIES.md
15. TEST_STRATEGY.md

---

## Complete System Scope

### 1. Complete Navigation Stack
- ✅ **Docking Navigation:** Precision approach to wheelchair (visual servoing)
- ✅ **Attached Mode Navigation:** Wheelchair transport A→B with person onboard
- ✅ **Free Navigation:** Patrol, charging station, shuttle services
- ✅ **Indoor Navigation:** Smooth floors, controlled environment
- ✅ **Outdoor Navigation:** Slopes (up to 10°), rough terrain, weather adaptation

### 2. Complete Perception System
- ✅ **LiDAR Processing:** Obstacle detection, terrain mapping, 3D environment understanding
- ✅ **Camera Vision:** ArUco marker detection, object recognition, lane detection
- ✅ **Sensor Fusion:** LiDAR + Camera + GPS + IMU + Odometry integration
- ✅ **Environment Understanding:** Terrain classification, obstacle type recognition
- ✅ **Dynamic Object Tracking:** Pedestrians, vehicles, wheelchairs (trajectory prediction)

### 3. Complete Planning System
- ✅ **Global Path Planning:** A* / Dijkstra / Hybrid A* for long-range routes
- ✅ **Local Motion Planning:** DWA / TEB / MPC optimized for swerve drive kinematics
- ✅ **Trajectory Optimization:** Smooth, safe, energy-efficient paths
- ✅ **Behavioral Planning:** State machine, task sequencing, decision-making
- ✅ **Re-planning:** Real-time dynamic obstacle response

### 4. Complete Avoidance System
- ✅ **Static Obstacle Avoidance:** Walls, furniture, curbs, fixed obstacles
- ✅ **Dynamic Obstacle Avoidance:** People, vehicles, moving objects
- ✅ **Predictive Avoidance:** Pedestrian intention estimation, trajectory prediction
- ✅ **Social Navigation:** Personal space respect (1.5m bubble), polite behavior
- ✅ **Emergency Avoidance:** Collision imminent → immediate hard brake

### 5. Complete Control System
- ✅ **Swerve Drive Controller:** Inverse kinematics, 4-wheel independent coordination
- ✅ **Docking Controller:** Visual servoing, PID control, precision alignment
- ✅ **Motion Controller:** Velocity/acceleration limiting, smooth motion profiles
- ✅ **Manual Control:** Joystick teleoperation, safety override capability
- ✅ **Mode Switching:** Solo / docking / attached mode transitions

### 6. Complete Safety System
- ✅ **Emergency Stop:** Hardware e-stop button + software emergency brake
- ✅ **Collision Prevention:** Safety zones, speed limiting, proximity alerts
- ✅ **Safety Monitoring:** Sensor health checks, system diagnostics, fault detection
- ✅ **Geofencing:** Keep-out zones, operational boundaries enforcement
- ✅ **Fail-Safe Behaviors:** Sensor loss → safe stop, graceful degradation
- ✅ **Redundancy:** Critical systems (braking, sensors) have backups

### 7. Complete Localization System
- ✅ **GPS Localization:** Outdoor global positioning, RTK for cm-level accuracy
- ✅ **Visual Localization:** ArUco markers, feature-based matching
- ✅ **LiDAR Localization:** Scan matching, point cloud registration
- ✅ **Odometry:** Wheel encoders, swerve drive odometry estimation
- ✅ **Sensor Fusion:** Extended Kalman Filter (EKF) / Particle Filter
- ✅ **Indoor/Outdoor Transition:** Automatic mode switching based on GPS availability

### 8. Complete SLAM System
- ✅ **Mapping:** 2D occupancy grid + 3D point cloud generation
- ✅ **Simultaneous Localization:** Online pose estimation during mapping
- ✅ **Loop Closure:** Drift correction, pose graph optimization
- ✅ **Map Management:** Multi-map support, map updates, versioning
- ✅ **LiDAR SLAM:** SLAM Toolbox / Cartographer for outdoor environments
- ✅ **Visual SLAM:** Optional fallback for indoor GPS-denied areas

### 9. Power Management
- ✅ Battery state monitoring (voltage, current, temperature)
- ✅ Charging station autonomous navigation
- ✅ Low-battery automatic return-to-base
- ✅ Energy-efficient route planning

### 10. Communication & Monitoring
- ✅ Remote teleoperation capability
- ✅ Cloud logging & analytics (mission data, diagnostics)
- ✅ Fleet management (multi-robot coordination)
- ✅ Operator dashboard (real-time monitoring)

### 11. User Interaction
- ✅ Joystick manual control (override autonomy)
- ✅ Voice announcements (status, warnings)
- ✅ LED status indicators (operational state)
- ✅ Mobile app (operator interface, mission assignment)

### 12. Environmental Adaptation
- ✅ Weather detection (rain, wind sensors)
- ✅ Lighting adaptation (day/night camera settings)
- ✅ Terrain adaptation (smooth/rough surface detection)
- ✅ Seasonal modes (summer/winter parameters)

---

## Technology Stack

### Hardware Platform
- **Drive System:** Swerve drive (4 modules, 6.5" hoverboard in-wheel motors)
- **Sensors:** LiDAR (360°), Dual RGB cameras, GPS (RTK), IMU (9-DOF), Wheel encoders
- **Safety:** Cliff detection sensors, Proximity sensors, Emergency stop button
- **Compute:** Embedded PC (x86_64), GPU for vision processing

### Software Stack
| Component | Technology | Purpose |
|-----------|-----------|---------|
| **Framework** | ROS 2 Humble | Robot operating system |
| **Navigation** | Nav2 | Path planning, obstacle avoidance |
| **SLAM** | SLAM Toolbox / Cartographer | Mapping and localization |
| **Sensor Fusion** | robot_localization | Multi-sensor integration (EKF) |
| **Vision** | OpenCV 4.x | Camera processing, ArUco detection |
| **Point Cloud** | PCL (Point Cloud Library) | LiDAR data processing |
| **Motion Planning** | MoveIt2 (optional) | Advanced trajectory planning |
| **Build System** | colcon | ROS 2 package build tool |
| **Custom Nodes** | C++17 / Python 3.10 | Swerve drive, docking, control |

---

## Development Principles

### 1. Separation of Concerns
- ✅ Each subsystem (swerve drive, docking, navigation, perception, safety) is **independently documented**
- ✅ Clear interfaces defined between subsystems
- ✅ Subsystems can be developed, tested, and validated **in parallel**
- ✅ Modular architecture enables **incremental integration**

### 2. Outdoor-First Design
- ✅ All requirements **prioritize outdoor operation** (slopes, weather, rough terrain)
- ✅ Indoor compatibility achieved as **subset of outdoor capabilities**
- ✅ Sensors selected for **outdoor robustness** (weatherproof, wide temperature range)
- ✅ Performance targets based on **outdoor worst-case scenarios**

### 3. Indoor Compatibility
- ✅ System must operate **seamlessly indoors** (smooth floors, controlled environment)
- ✅ Automatic **environment detection** (GPS availability, lighting, terrain type)
- ✅ **Precision mode** for indoor operations (tighter tolerances, faster speeds)
- ✅ **Fallback strategies** (GPS loss → visual SLAM, marker-based localization)

### 4. Agile Methodology
- ✅ **Incremental development** in sprints (2-week cycles)
- ✅ **User stories** define features from end-user perspective
- ✅ **Continuous integration** and testing
- ✅ **Iterative refinement** based on testing feedback

### 5. Integration-Ready Architecture
- ✅ **Well-defined interfaces** (ROS topics, services, actions documented)
- ✅ **Modular design** enables parallel team development
- ✅ **Integration testing** at each sprint boundary
- ✅ **System-level validation** after subsystem integration

### 6. Safety-First Approach
- ✅ **Safety requirements** defined for every subsystem
- ✅ **Multiple safety layers** (emergency stop, collision avoidance, geofencing)
- ✅ **Fail-safe defaults** (unknown state → safe stop)
- ✅ **Redundancy** in critical systems (sensors, braking)

### 7. Based on Proven Hardware
- ✅ Specifications derived from **Swerve Drive Study (2025-12-10)**
- ✅ Hardware selections validated through **analysis and simulation**
- ✅ Design parameters based on **measured requirements** (60kg robot, 100kg load, 10° slope)

---

## Use Cases

### Primary Use Cases
1. ✅ **Autonomous Wheelchair Docking** (precision ±2-5mm outdoor, ±1mm indoor target)
2. ✅ **Wheelchair Transport** (A→B navigation with person onboard, smooth ride)
3. ✅ **Charging Station Navigation** (automatic return when battery low)
4. ✅ **Shuttle Service** (scheduled routes, multi-waypoint execution)
5. ✅ **Manual Operation** (joystick override, teaching mode for waypoint recording)

### Secondary Use Cases
6. ✅ **Patrol Routes** (security monitoring, scheduled inspection)
7. ✅ **Delivery Tasks** (transport equipment, goods between locations)
8. ✅ **Emergency Return** (sensor failure → safe navigation to base)
9. ✅ **Teaching Mode** (human guides robot, system records waypoints)
10. ✅ **Fleet Coordination** (multiple robots, collision avoidance, task allocation)

---

## Next Steps

### Immediate Actions
1. ✅ Create complete documentation structure (folders and template files)
2. ✅ Develop **SYSTEM_OVERVIEW.md** (executive summary)
3. ✅ Develop **SYSTEM_REQUIREMENTS.md** (functional and non-functional requirements)
4. ✅ Develop **OVERALL_SYSTEM_ARCHITECTURE.md** (high-level design)

### Short-Term Goals
5. ✅ Define subsystem requirements (swerve drive, docking, navigation, perception, safety)
6. ✅ Design subsystem architectures (ROS node structure, interfaces)
7. ✅ Create **AGILE_ROADMAP.md** (sprint planning, timeline)
8. ✅ Define **USER_STORIES.md** (feature descriptions from user perspective)

### Medium-Term Goals
9. ✅ Develop detailed design documents (kinematics, control algorithms, state machines)
10. ✅ Define interface control documents (ROS topics, services, message definitions)
11. ✅ Create test strategy and acceptance criteria
12. ✅ Begin sprint-based development

---

**Document Version:** 1.0
**Last Updated:** December 15, 2025
**Status:** Planning Phase - Ready for Documentation Development
**Next Milestone:** Complete documentation structure creation
