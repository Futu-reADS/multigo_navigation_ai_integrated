# Team Responsibilities - Multi-Team Development

**Document ID:** DOC-TEAM-001
**Version:** 1.0
**Date:** 2025-12-16
**Status:** Active

---

## Executive Summary

This document defines **clear scope separation** across three development teams for the wheelchair transport robot project:

1. **Team 1:** Pankaj (Vehicle Software) - Autonomous navigation, docking, local UI, TVM client
2. **Team 2:** Unno (Fleet Management) - TVM server, dashboard, reservation system, user management
3. **Team 3:** Tsuchiya + Kiril (Hardware) - Mechanical, electrical, exterior hardware assembly

**Critical Interfaces (Defined in Week 1):**
- Vehicle ↔ TVM Server: `TVM_API_SPECIFICATION.md` ⚠️ **CRITICAL**
- Software ↔ Hardware: `HARDWARE_SOFTWARE_INTERFACES.md` ⚠️ **CRITICAL**

---

## Team 1: Pankaj (Vehicle Software)

### Primary Responsibility
**Vehicle-side autonomous software and TVM client interface**

### Detailed Scope

#### **1.1 Autonomous Navigation (Week 8-12)**
**Responsibility:** 100% Pankaj
- NDT scan matcher localization (±10cm accuracy)
- Nav2 integration (path planning, trajectory following)
- Multi-waypoint navigation
- Route execution and recovery
- Operational hours management

**Deliverables:**
- ROS 2 packages: `ndt_localization`, `nav_controller`, `waypoint_manager`
- Launch files for navigation stack
- Configuration files (costmap, planner parameters)
- Unit + integration tests

**Dependencies:**
- Hardware: LiDAR, IMU, wheel encoders (Tsuchiya)
- Maps: Pre-built SLAM maps (manual mapping phase)

---

#### **1.2 Wheelchair Docking (Week 13-16)**
**Responsibility:** 100% Pankaj
- ArUco marker detection (OpenCV)
- Dual-camera fusion
- Visual servoing control (PID)
- Docking state machine
- Precision attachment (±2-5mm)

**Deliverables:**
- ROS 2 packages: `aruco_detector`, `docking_controller`, `visual_servoing`
- ArUco marker specifications (size, IDs, placement)
- Calibration procedures
- Docking success metrics (>90% success rate)

**Dependencies:**
- Hardware: 2× RGB cameras, wheelchair docking mechanism (Tsuchiya)
- Markers: ArUco markers printed and mounted (Hardware team)

---

#### **1.3 Swerve Drive Control (Week 4-8)**
**Responsibility:** 100% Pankaj
- Inverse kinematics (Twist → wheel commands)
- Forward kinematics (encoders → odometry)
- Nav2 controller plugin
- Motor command interface (CAN bus)

**Deliverables:**
- ROS 2 package: `swerve_drive_controller`
- Kinematic model implementation
- Controller parameters (PID gains, velocity limits)
- Odometry publisher

**Dependencies:**
- Hardware: 4× swerve modules, encoders, motor controllers (Tsuchiya)
- CAN bus interface configured

---

#### **1.4 Perception (Week 9-12)**
**Responsibility:** 100% Pankaj
- LiDAR point cloud processing
- Ground removal (RANSAC)
- Obstacle detection (binary costmap)
- Sensor fusion (LiDAR + IMU)

**Deliverables:**
- ROS 2 packages: `pcl_processor`, `obstacle_detector`, `ground_removal`
- Costmap integration (local + global)
- Performance optimization (CPU-only, no GPU)

**Dependencies:**
- Hardware: 3D LiDAR, IMU (Tsuchiya)

---

#### **1.5 Safety Systems (Week 15-18)**
**Responsibility:** 100% Pankaj
- Emergency stop handling
- Collision detection (bumper sensors)
- Watchdog monitoring
- Health checks (sensor failures)
- Tilt detection (IMU)

**Deliverables:**
- ROS 2 package: `safety_monitor`
- Safety state machine
- Diagnostic aggregator
- Fail-safe behaviors

**Dependencies:**
- Hardware: E-stop button, bumper sensors, IMU (Tsuchiya)

---

#### **1.6 Local UI (Week 17-20)**
**Responsibility:** 100% Pankaj
- Touch screen interface (React + Next.js)
- rosbridge WebSocket connection
- Mission control (select destination, start/cancel)
- Status dashboard (battery, location, state)
- Multi-language support (EN/JA/ZH)

**Deliverables:**
- Next.js web application
- Kiosk mode deployment script
- UI/UX design (responsive, accessible)
- Chromium kiosk configuration

**Dependencies:**
- Hardware: Touch screen, mounting (Tsuchiya)

---

#### **1.7 eHMI Integration (Week 19-21)**
**Responsibility:** 80% Pankaj (ROS 2 integration), 20% Kiril (firmware)
- ROS 2 → ESP32 serial bridge
- State management (vehicle state → eHMI state)
- Audio command interface
- Language/volume configuration

**Deliverables:**
- ROS 2 package: `ehmi_interface`
- Serial protocol implementation
- State mapping (vehicle states 0-50)

**Dependencies:**
- Hardware: ESP32-S3, LEDs, speakers (Kiril)
- Firmware: ESP32 firmware (Kiril - see Section 3.3)

---

#### **1.8 TVM Client Interface (Week 20-22) ⚠️ NEW**
**Responsibility:** 100% Pankaj
- REST API client (location, status, battery, error reports)
- WebSocket client (command reception: dispatch, cancel, e-stop)
- JWT authentication
- Offline queue management (up to 1000 messages)
- Bulk upload on reconnection

**Deliverables:**
- ROS 2 package: `tvm_client`
- TVM client library (C++ or Python)
- Configuration (TVM server URL, credentials)
- Integration tests with mock TVM server

**Dependencies:**
- **Interface Spec:** `TVM_API_SPECIFICATION.md` ⚠️ **CRITICAL**
- **Data Models:** `TVM_DATA_MODELS.md`
- Fleet Management Team: TVM server API (Unno)

---

### Development Timeline (Pankaj)
**Total:** 32 weeks (8 months, solo developer + AI assistance)

| Sprint | Weeks | Subsystem | Status |
|--------|-------|-----------|--------|
| 1-2 | 1-4 | Swerve Drive | Hardware mods parallel |
| 3-4 | 5-8 | Navigation | - |
| 5-6 | 9-12 | Perception | - |
| 7-8 | 13-16 | Docking | - |
| 9-10 | 15-18 | Safety | Overlap with Docking |
| 11-12 | 17-20 | UI | Overlap with Safety |
| 13 | 19-21 | eHMI | Overlap with UI |
| 14-15 | 20-22 | TVM Client | Integration |
| 16-17 | 21-24 | System Integration | All teams |

---

### Tools & Technologies (Pankaj)
- **OS:** Ubuntu 22.04 LTS
- **Middleware:** ROS 2 Humble
- **Languages:** C++ (control, perception), Python (scripting, tools)
- **Navigation:** Nav2 + NDT scan matcher
- **Vision:** OpenCV, ArUco
- **UI:** React 18, Next.js 14, TypeScript
- **TVM Client:** Python (requests, websockets) or C++ (libcurl, Boost.Beast)
- **Testing:** pytest, gtest, integration tests
- **AI Tools:** Claude Code, GitHub Copilot (2-3x productivity boost)

---

## Team 2: Unno (Fleet Management / TVM Server)

### Primary Responsibility
**TVM server backend, fleet management dashboard, reservation system**

### Detailed Scope

#### **2.1 TVM Server Backend (Week 2-4)**
**Responsibility:** 100% Unno
- REST API server (7 endpoints - see TVM_API_SPECIFICATION.md)
- WebSocket server (real-time commands)
- JWT authentication
- Rate limiting
- Database integration
- Logging and metrics

**Deliverables:**
- Backend server (Node.js/Python/Java - TBD by Unno)
- API implementation matching `TVM_API_SPECIFICATION.md`
- Database schema (PostgreSQL/MySQL/MongoDB - TBD)
- Deployment scripts (Docker, Kubernetes, or VM)

**Dependencies:**
- **Interface Spec:** `TVM_API_SPECIFICATION.md` ⚠️ **CRITICAL**
- **Data Models:** `TVM_DATA_MODELS.md`
- Vehicle Team: TVM client implementation (Pankaj)

---

#### **2.2 Fleet Management Dashboard (Week 5-8)**
**Responsibility:** 100% Unno
- Real-time vehicle tracking (map view)
- Battery monitoring across fleet
- Route visualization
- Alert management
- Emergency controls (remote e-stop)

**Deliverables:**
- Web application (React/Vue/Angular - TBD by Unno)
- Map integration (Google Maps, Mapbox, OpenStreetMap)
- Real-time WebSocket updates (1 Hz vehicle locations)
- Dashboard UI/UX design

**Dependencies:**
- TVM server backend (Section 2.1)
- Vehicle telemetry data (Pankaj's TVM client)

---

#### **2.3 Reservation System (Week 6-10)**
**Responsibility:** 100% Unno
- Walking assistance booking (Caregiver role)
- Medicine delivery scheduling (Nurse role - Ver2)
- Robot dispatch algorithm
- Queue management
- Conflict resolution

**Deliverables:**
- Reservation service (backend)
- Booking interface (web UI)
- Dispatch algorithm (nearest available robot)
- Schedule management

**Dependencies:**
- TVM server backend (Section 2.1)

---

#### **2.4 User Role Management (Week 8-10)**
**Responsibility:** 100% Unno
- Authentication (username/password)
- Authorization (RBAC: Admin, Caregiver, Nurse)
- User CRUD operations
- Permission management

**Deliverables:**
- Authentication service
- Role-based access control (RBAC)
- Admin user management interface

**Dependencies:**
- TVM server backend (Section 2.1)

---

#### **2.5 Voice Communication System (Week 11-14)**
**Responsibility:** 100% Unno
- VoIP integration (Twilio, WebRTC, or custom)
- Staff ↔ Robot voice calls
- Call initiation from dashboard
- Audio streaming

**Deliverables:**
- VoIP service integration
- Audio streaming infrastructure
- Call UI in dashboard

**Dependencies:**
- TVM server backend (Section 2.1)
- Hardware: Microphone + speaker on vehicle (Kiril)

---

#### **2.6 Log Management (Week 12-14)**
**Responsibility:** 100% Unno
- Vehicle log aggregation
- S3 integration (display uploaded logs)
- Log viewer in dashboard

**Deliverables:**
- Log viewing interface
- Integration with existing `tvm_upload` daemon (separate system)

**Dependencies:**
- Existing `tvm_upload` daemon (already implemented, separate)
- S3 bucket access (AWS China)

---

### Development Timeline (Unno)
**Total:** 12-16 weeks (parallel with Pankaj's vehicle development)

| Week | Subsystem | Deliverable |
|------|-----------|-------------|
| 1-2 | Interface Definition | Sign-off on TVM_API_SPECIFICATION.md |
| 2-4 | TVM Server Backend | REST + WebSocket API |
| 5-8 | Fleet Dashboard | Real-time monitoring |
| 6-10 | Reservation System | Booking + dispatch |
| 8-10 | User Management | Auth + RBAC |
| 11-14 | Voice Communication | VoIP integration |
| 12-14 | Log Management | Log viewer |
| 15-16 | Integration Testing | End-to-end with vehicle |

---

### Tools & Technologies (Unno - TBD)
**To be decided by Unno's team:**
- **Backend:** Node.js (Express/NestJS), Python (FastAPI/Django), Java (Spring Boot)
- **Database:** PostgreSQL, MySQL, MongoDB
- **Frontend:** React, Vue, Angular
- **Real-Time:** WebSocket, Socket.io, Server-Sent Events
- **Deployment:** Docker, Kubernetes, AWS/Azure/GCP
- **VoIP:** Twilio, WebRTC, Asterisk

---

## Team 3: Tsuchiya + Kiril (Hardware)

### Primary Responsibility
**Physical robot assembly, electrical integration, exterior components**

### 3.1 Tsuchiya (Mechanical & Electrical)

#### **3.1.1 Mechanical Structure (Week 2-6)**
**Responsibility:** 100% Tsuchiya
- Chassis design and fabrication
- Swerve drive module assembly (4× modules)
- Wheelchair docking mechanism (physical attachment, locking)
- Sensor mounting (LiDAR, cameras, IMU)
- Weatherproofing (IP54+ sealing)

**Deliverables:**
- Assembled robot platform
- Swerve drive modules installed and calibrated
- Docking mechanism functional
- BOM (Bill of Materials)
- Assembly instructions
- CAD files (optional, for reference)

**Dependencies:**
- Software: ROS 2 interface specs (Pankaj - see HARDWARE_SOFTWARE_INTERFACES.md)

---

#### **3.1.2 Electrical System (Week 4-8)**
**Responsibility:** 100% Tsuchiya
- Power distribution (48V battery, BMS, fuses)
- Motor controllers (4× drive + 4× steer, CAN bus)
- Wiring harness (sensors, motors, power)
- Cable management and strain relief
- Safety circuits (E-stop hardwired to motors)

**Deliverables:**
- Complete electrical system
- Wiring diagrams
- Power distribution schematic
- BMS configured (CAN bus interface)
- All cables labeled

**Dependencies:**
- Software: CAN bus protocol (Pankaj - see HARDWARE_SOFTWARE_INTERFACES.md)

---

#### **3.1.3 Sensor Integration (Week 6-10)**
**Responsibility:** 100% Tsuchiya
- LiDAR installation (IP65+ outdoor LiDAR)
- Camera installation (2× RGB, weatherproof housings)
- IMU installation (center of mass, axes aligned)
- Wheel encoders (4× drive + 4× steer)
- Safety sensors (bumpers, passenger weight, e-stop)
- Battery monitoring (BMS with CAN interface)

**Deliverables:**
- All sensors installed and functional
- ROS 2 drivers tested (publishing data)
- Sensor calibration (cameras: checkerboard, IMU: magnetometer)
- Static TF transforms documented (sensor positions)

**Dependencies:**
- Software: ROS 2 sensor drivers (Pankaj)
- Calibration: Camera calibration procedure (Pankaj + Tsuchiya joint)

---

### 3.2 Kiril (Exterior Components)

#### **3.2.1 eHMI System (Week 8-12)**
**Responsibility:** 100% Kiril (hardware + firmware)
- ESP32-S3 microcontroller board
- WS2812B LED strip (2-3 meters, ≥60 LEDs/meter)
- HUB75 LED matrix (optional, 64×32 or 128×64)
- I2S audio amplifier (MAX98357A or similar)
- Speakers (≥5W, weatherproof)
- SD card (≥2GB, for audio files)
- UART interface to main computer

**Deliverables:**
- eHMI hardware assembled
- ESP32 firmware (state machine, LED patterns, audio playback)
- Audio files (EN/JA/ZH announcements)
- Serial protocol implementation (see HARDWARE_SOFTWARE_INTERFACES.md)

**Dependencies:**
- Software: ROS 2 eHMI interface (Pankaj - see Section 1.7)
- Audio files: Pre-recorded audio (Kiril to source or Pankaj to provide)

**Firmware Responsibility:** 100% Kiril
- Implement serial protocol (UART, 115200 baud)
- State machine (receive state 0-50, update LEDs + audio)
- LED patterns (WS2812B control, FastLED library)
- Audio playback (I2S, play MP3/WAV from SD card)

**Reference:** See `EHMI_SYSTEM_REFERENCE.md` (existing ParcelPal eHMI documentation)

---

#### **3.2.2 Exterior Panels & Aesthetics (Week 10-14)**
**Responsibility:** 100% Kiril
- Body panels (weatherproof, IP54+)
- Paint/finish
- Branding (logos, labels)
- Lighting (headlights, taillights, indicators)

**Deliverables:**
- Completed exterior
- Weatherproofing validated (rain test)
- Professional appearance

---

#### **3.2.3 Passenger Safety Hardware (Week 12-14)**
**Responsibility:** 100% Kiril
- Seatbelt system (if applicable for wheelchair attachment)
- Handrails
- Footrests
- Safety labels and instructions

**Deliverables:**
- Safety features installed
- Tested for passenger safety

---

### Development Timeline (Tsuchiya + Kiril)
**Total:** Parallel with software (hardware ready as needed per sprint)

| Week | Tsuchiya (Mech/Elec) | Kiril (Exterior) |
|------|----------------------|------------------|
| 1-2 | Interface sign-off | Interface sign-off |
| 2-6 | Mechanical structure, swerve assembly | - |
| 4-8 | Electrical system, power distribution | - |
| 6-10 | Sensor integration, calibration | - |
| 8-12 | - | eHMI hardware + firmware |
| 10-14 | Final assembly, testing | Exterior panels, lighting |
| 12-14 | - | Passenger safety hardware |
| 15-16 | System integration (all teams) | System integration |

---

### Tools & Technologies (Hardware)
**Tsuchiya:**
- Mechanical: CAD software (SolidWorks, Fusion 360), machining tools
- Electrical: Wiring, CAN bus tools, multimeter, oscilloscope
- Software: ROS 2 driver configuration, CAN bus testing

**Kiril:**
- Embedded: Arduino IDE (ESP32), FastLED library, ESP32-AudioI2S
- Hardware: Soldering, LED installation, audio amplifier setup
- Testing: Serial monitor, audio playback testing

---

## Interface Ownership Matrix

| Interface | Owner 1 | Owner 2 | Document | Status |
|-----------|---------|---------|----------|--------|
| **Vehicle ↔ TVM Server** | Pankaj (client) | Unno (server) | TVM_API_SPECIFICATION.md | ✅ Week 1 |
| **Software ↔ Hardware** | Pankaj (ROS 2) | Tsuchiya (sensors/motors) | HARDWARE_SOFTWARE_INTERFACES.md | ✅ Week 1 |
| **Software ↔ eHMI** | Pankaj (ROS 2 bridge) | Kiril (ESP32 firmware) | HARDWARE_SOFTWARE_INTERFACES.md (Section 4.1) | ✅ Week 1 |
| **TVM ↔ Database** | Unno | Unno (internal) | DATABASE_SCHEMA.md | Week 2 |

---

## Decision-Making Authority

| Decision Type | Owner | Consultation Required |
|---------------|-------|----------------------|
| Vehicle software architecture | Pankaj | None (solo decision) |
| TVM server technology stack | Unno | None (Unno decides) |
| Hardware component selection | Tsuchiya + Kiril | Pankaj (compatibility check) |
| API interface changes | Pankaj + Unno | **Both must agree** ⚠️ |
| ROS 2 topic changes | Pankaj + Tsuchiya | **Both must agree** ⚠️ |
| Budget allocation | Project Manager | All teams (input) |

---

## Communication Protocol

### Weekly Sync Meeting
**Attendees:** Pankaj + Unno + Tsuchiya + Kiril
**Duration:** 30 minutes
**Agenda:**
1. Progress updates (5 min each team)
2. Blockers and dependencies (10 min)
3. Next week priorities (5 min)

### Interface Reviews
**Week 1:** Interface specification sign-off (TVM API + Hardware interfaces)
**Week 8:** Mid-project integration check
**Week 16:** Final integration testing

### Issue Escalation
**Level 1:** Direct communication between teams (email, chat)
**Level 2:** Weekly sync meeting
**Level 3:** Project Manager (if blocking progress)

---

## Conflict Resolution

**If teams disagree on interface design:**
1. Both teams document their proposal (pros/cons)
2. Review in weekly sync meeting
3. Technical merit decides (performance, maintainability, safety)
4. Project Manager final decision if no consensus

**Example:** If Pankaj wants to change TVM API frequency from 1 Hz to 2 Hz:
- Pankaj documents reason (need for higher update rate)
- Unno evaluates server load impact
- Both agree on compromise or one side makes technical case
- Decision documented in interface spec (version update)

---

## Success Criteria

**Team 1 (Pankaj) Success:**
- ✅ Vehicle navigates autonomously (500m-1km range)
- ✅ Docking precision (±5mm, >90% success rate)
- ✅ TVM client operational (1 Hz telemetry, command reception)
- ✅ All ROS 2 nodes functional
- ✅ Local UI working (touch screen)

**Team 2 (Unno) Success:**
- ✅ TVM server operational (handles 10 vehicles @ 1 Hz)
- ✅ Fleet dashboard functional (real-time monitoring)
- ✅ Reservation system working (booking + dispatch)
- ✅ User management working (Auth + RBAC)

**Team 3 (Tsuchiya + Kiril) Success:**
- ✅ Robot assembled (swerve drive functional)
- ✅ All sensors operational (LiDAR, cameras, IMU, encoders)
- ✅ Power system functional (battery, BMS, charging)
- ✅ eHMI operational (LED + audio)
- ✅ Weatherproofing validated (IP54+)

**System Integration Success (All Teams):**
- ✅ End-to-end mission complete (TVM dispatch → navigation → docking → return)
- ✅ >90% mission success rate
- ✅ All interfaces functional
- ✅ Safety systems validated

---

## Document Status

**Status:** Active
**Maintained By:** Project Manager
**Review Frequency:** Monthly (or when team structure changes)
**Last Updated:** 2025-12-16

---

**END OF DOCUMENT**
