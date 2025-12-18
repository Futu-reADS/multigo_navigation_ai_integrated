# Project Comprehensive Summary
## Outdoor Wheelchair Transport Robot Fleet System

**Date:** December 17, 2025
**Version:** 1.0
**Purpose:** Complete explanation of what we're building and how we got here

---

## Table of Contents

1. [What Are We Building? (Simple Explanation)](#what-are-we-building-simple-explanation)
2. [Why Are We Building This?](#why-are-we-building-this)
3. [Complete Documentation Overview](#complete-documentation-overview)
4. [Inputs and References Used](#inputs-and-references-used)
5. [Key Decisions and Design Philosophy](#key-decisions-and-design-philosophy)
6. [Team Structure and Responsibilities](#team-structure-and-responsibilities)
7. [Implementation Strategy](#implementation-strategy)
8. [Timeline and Milestones](#timeline-and-milestones)
9. [Technology Stack Explained](#technology-stack-explained)
10. [Success Criteria](#success-criteria)

---

## What Are We Building? (Simple Explanation)

### The Big Picture

We're building **a fleet of smart robots that can transport people in wheelchairs** around outdoor environments like hospitals, elderly care facilities, or university campuses.

Think of it as **an autonomous taxi service, but specifically designed to pick up someone in a wheelchair, safely transport them to their destination, and then go help the next person.**

### What Makes It Special?

1. **Works Outdoors First** 🌤️
   - Can handle rain, slopes, rough surfaces
   - Indoor capability is a bonus, not the main focus

2. **Precision Docking** 🎯
   - Robot can attach to wheelchair with ±2-5mm accuracy
   - Like a very precise self-parking car

3. **No GPS Needed** 🗺️
   - Uses laser sensors to navigate (like a self-driving car's LiDAR)
   - Can work indoors or in GPS-denied areas

4. **Fleet Management** 📱
   - Multiple robots managed from one dashboard
   - Caregivers can request pickups via computer/tablet
   - Real-time tracking of all robots

5. **Smart Movement** 🤖
   - "Swerve drive" - can move in any direction without turning
   - Safer and more maneuverable than traditional wheels

---

## Why Are We Building This?

### Problem We're Solving

**Healthcare facilities need to transport wheelchair-bound patients/residents:**
- Staff members spend hours pushing wheelchairs
- Patients want independence but can't navigate alone
- Outdoor paths are too challenging for traditional wheelchair robots

### Our Solution

An autonomous robot that:
- Picks up wheelchairs automatically
- Transports them safely
- Works in challenging outdoor environments
- Can be scheduled in advance or called on-demand
- Manages a fleet of multiple robots efficiently

### Real-World Use Cases

1. **Hospital Shuttle** 🏥
   - Transport patients between buildings
   - Schedule: "Pick up patient from Room 301, take to Radiology at 2 PM"

2. **Elderly Care Facility** 👴👵
   - Daily walking assistance
   - "Take Mr. Tanaka for his morning walk at 9 AM"

3. **Outdoor Campus** 🏫
   - Navigate between buildings on campus
   - Handle slopes, weather, rough pathways

---

## Complete Documentation Overview

### Documentation Structure (118 Documents Total)

```
outdoor_swerve_system/
│
├── README.md                          ← Start here
│
├── multi_team/                        ← Main documentation (NEW)
│   │
│   ├── 00_OVERVIEW/                   (6 docs - What & Why)
│   │   ├── PROJECT_STATUS.md          ← Current status
│   │   ├── VEHICLE_SYSTEM_OVERVIEW.md ← Technical overview
│   │   ├── TEAM_RESPONSIBILITIES.md   ← Who does what
│   │   ├── IMPLEMENTATION_TIMELINE_2026.md     ← Original timeline
│   │   ├── RECOMMENDED_TIMELINE_2026.md        ← Optimized timeline
│   │   └── DETAILED_IMPLEMENTATION_PLAN_2026.md ← Week-by-week plan
│   │   └── [+ Japanese versions _JA.md]
│   │
│   ├── 01_REQUIREMENTS/               (26 docs - What it must do)
│   │   ├── VEHICLE/                   (9 docs - Robot software)
│   │   │   ├── NAVIGATION_REQUIREMENTS.md
│   │   │   ├── DOCKING_SYSTEM_REQUIREMENTS.md
│   │   │   ├── SWERVE_DRIVE_REQUIREMENTS.md
│   │   │   ├── PERCEPTION_REQUIREMENTS.md
│   │   │   ├── SAFETY_REQUIREMENTS.md
│   │   │   └── ... (+ more)
│   │   │
│   │   ├── FLEET_MANAGEMENT/          (6 docs - Dashboard & Server)
│   │   │   ├── FLEET_MANAGEMENT_REQUIREMENTS.md
│   │   │   ├── TVM_SERVER_REQUIREMENTS.md
│   │   │   ├── RESERVATION_SYSTEM_REQUIREMENTS.md
│   │   │   ├── USER_ROLE_MANAGEMENT_REQUIREMENTS.md
│   │   │   └── ... (+ more)
│   │   │
│   │   ├── HARDWARE/                  (5 docs - Physical robot)
│   │   │   ├── MECHANICAL_REQUIREMENTS.md
│   │   │   ├── ELECTRICAL_REQUIREMENTS.md
│   │   │   ├── COMMUNICATION_SYSTEM_REQUIREMENTS.md
│   │   │   └── ... (+ more)
│   │   │
│   │   └── INTEGRATION/               (System-wide requirements)
│   │
│   ├── 02_ARCHITECTURE/               (15 docs - How it's organized)
│   │   ├── VEHICLE/                   (System architecture)
│   │   ├── FLEET_MANAGEMENT/          (Server architecture)
│   │   ├── HARDWARE/                  (Hardware layout)
│   │   └── INTEGRATION/               (How everything connects)
│   │
│   ├── 03_DESIGN/                     (18 docs - Detailed designs)
│   │   ├── VEHICLE/                   (Software design)
│   │   ├── FLEET_MANAGEMENT/          (Server design)
│   │   └── HARDWARE/                  (Hardware design)
│   │
│   ├── 04_INTERFACES/                 (5 docs - How teams communicate)
│   │   ├── TVM_API_SPECIFICATION.md   ← Vehicle ↔ Server communication
│   │   ├── TVM_DATA_MODELS.md         ← Data formats
│   │   ├── HARDWARE_SOFTWARE_INTERFACES.md ← Hardware ↔ Software
│   │   └── ROS2_TOPICS.md             ← Internal robot communication
│   │
│   ├── 05_DEVELOPMENT/                (9 docs - How to build it)
│   │   ├── COMPLETE_DEVELOPMENT_GUIDE.md
│   │   ├── VEHICLE_SOFTWARE_DEVELOPMENT_GUIDE.md
│   │   ├── FLEET_SOFTWARE_DEVELOPMENT_GUIDE.md
│   │   ├── HARDWARE_DEVELOPMENT_GUIDE.md
│   │   ├── CODE_STANDARDS_AND_PRACTICES.md
│   │   └── AGILE_ROADMAP.md
│   │
│   ├── 06_TESTING/                    (5 docs - How to test it)
│   │   ├── COMPLETE_TESTING_GUIDE.md
│   │   ├── TESTING_REQUIREMENTS.md
│   │   └── TEST_EXECUTION_GUIDE.md
│   │
│   └── 07_DEPLOYMENT/                 (4 docs - How to deploy it)
│       ├── DEPLOYMENT_GUIDE.md
│       └── MAINTENANCE_GUIDE.md
│
└── outdoor_swerve_system_current/     ← Reference documentation
    ├── reference/                      ← Original inputs
    │   ├── SWERVE_DRIVE_SYSTEM_TASK.md      (Original task)
    │   ├── PARCELPAL_EXPLORATION_SUMMARY.md (Proven architecture)
    │   ├── EHMI_SYSTEM_REFERENCE.md         (Display/lights/sounds)
    │   └── question_answers.md              (Q&A with senior)
    │
    └── [Previous vehicle-only documentation - 87 files]
```

### Documentation Purpose

| Folder | Purpose | Who Reads It |
|--------|---------|--------------|
| **00_OVERVIEW** | High-level explanation, what & why | Everyone, managers |
| **01_REQUIREMENTS** | What the system must do | All developers |
| **02_ARCHITECTURE** | How it's organized | Architects, leads |
| **03_DESIGN** | Detailed technical design | Developers |
| **04_INTERFACES** | How teams work together | All teams |
| **05_DEVELOPMENT** | How to build it | Developers |
| **06_TESTING** | How to test it | QA, developers |
| **07_DEPLOYMENT** | How to deploy it | Operations, devops |

---

## Inputs and References Used

### 1. Original Task Specification

**Document:** `SWERVE_DRIVE_SYSTEM_TASK.md`

**What It Provided:**
- Original project requirements: "Build an outdoor-first robot"
- Hardware specifications: Swerve drive, 6.5" wheels, 80W motors
- Key capabilities: Wheelchair docking, autonomous navigation, manual control
- Design parameters: 60kg robot, 100kg load, 10° slopes, 1.0 m/s speed

**Key Quote:**
> "Develop a complete end-to-end autonomous robot system with swerve drive locomotion, capable of operating primarily outdoors while maintaining indoor compatibility."

---

### 2. ParcelPal Proven Architecture

**Document:** `PARCELPAL_EXPLORATION_SUMMARY.md`

**What It Provided:**
- **Navigation Strategy:** Manual SLAM mapping once → NDT localization during operation
- **Why This Matters:** No GPS needed, no drift accumulation, 500m-1km range
- **Technology Stack:** ROS 2 Humble, Autoware framework, Ubuntu 22.04
- **Architecture Patterns:** 60-70% reusable patterns (NOT code, just architectural approaches)

**Key Insight:**
> "ParcelPal does NOT use continuous online SLAM during operation. Maps are created offline with loop closure, then the robot localizes on the saved map using NDT scan matching."

**What We Reused:**
- ✅ NDT localization approach (100%)
- ✅ Manual mapping strategy (100%)
- ✅ UI framework patterns (70%)
- ✅ eHMI controller patterns (90%)
- ✅ Main orchestration patterns (80%)

**What We Built New:**
- ❌ Swerve drive controller (0% from ParcelPal - completely new)
- ❌ Wheelchair docking system (0% - completely new)
- ❌ Fleet management TVM system (0% - completely new)

---

### 3. eHMI System Reference

**Document:** `EHMI_SYSTEM_REFERENCE.md`

**What It Provided:**
- External Human-Machine Interface design (lights, sounds, displays)
- ESP32-S3 microcontroller setup
- LED patterns for different robot states
- Audio announcement system
- Serial communication protocol between robot and eHMI

**Purpose:**
Pedestrians need to know what the robot is doing:
- "Moving forward" → Front white lights
- "Emergency stop" → Flashing red lights + alarm sound
- "Docking mode" → Blue lights + "Approaching wheelchair" announcement

---

### 4. Senior Review Q&A

**Document:** `question_answers.md` (60KB file!)

**What It Provided:**
- Answers to 50+ technical questions from senior engineer
- Clarifications on design decisions
- Validation of architectural choices
- Budget constraints and practical considerations

**Key Topics:**
- Why no GPS? (Cost, reliability, indoor operation)
- Why swerve drive instead of mecanum wheels? (Better outdoor performance)
- Why no deep learning? (CPU-only requirement, simpler, more reliable)
- Why manual SLAM mapping? (Proven by ParcelPal, no drift issues)

---

### 5. User Requirements (Implicit from Use Cases)

**Source:** Project discussions and use case descriptions

**Use Cases Identified:**
1. **Walking Assistance** (Priority: Critical)
   - "Mr. Tanaka needs his morning walk at 9 AM"
   - Caregiver schedules via dashboard
   - Robot picks up, follows route, returns

2. **Medicine Delivery** (Priority: High, Version 2)
   - Nurse schedules delivery to specific room
   - Robot navigates to pharmacy, then to patient room

3. **Multi-Stop Shuttle** (Priority: High)
   - Pre-defined routes with multiple stops
   - "Building A → Building B → Building C → Charging Station"

4. **Emergency Transport** (Priority: Critical)
   - Immediate dispatch, bypasses queue
   - Faster speed limits allowed

5. **Teleoperation** (Priority: Medium)
   - Manual control when autonomous fails
   - Teaching mode for new routes

---

### 6. Hardware Study Results

**Source:** Swerve Drive Study (December 10, 2025)

**What It Provided:**
- Motor calculations: 80W per wheel sufficient for 10° slopes
- Ground clearance: 20mm minimum needed
- Suspension: Required for outdoor rough terrain
- Steering motors: Digital servo 3.5Nm or power window motor 2.9Nm

---

### 7. Technology Constraints

**Sources:** Team capabilities, budget, timeline

**Constraints Identified:**
1. **No Expensive GPU:** Must run on integrated GPU (Radeon 780M)
2. **No Deep Learning:** CPU-only obstacle detection
3. **No RTK GPS:** Too expensive, not needed for <1km range
4. **Standard Sensors:** LiDAR + cameras + IMU (proven, available)
5. **ROS 2 Humble:** LTS version, stable, well-documented

---

### 8. Team Structure Input

**Source:** Project organization decisions

**Three Independent Teams:**
1. **Pankaj** → Vehicle software (solo developer + AI tools)
2. **Unno** → Fleet management server (TBD team size)
3. **Tsuchiya + Kirill** → Hardware assembly

**Key Decision:**
Independent teams with clear interfaces → Parallel development possible

---

### 9. Timeline Input

**Source:** Senior's proposed timeline

**Original Plan:**
- Hardware complete: March 20 (Week 10)
- TVM MVP: June 4 (Week 20)
- Vehicle-TVM integration: 0 weeks dedicated
- Buffer: 0 weeks
- **Success probability:** 50-60%

**Our Analysis:**
- Hardware prototype by Feb 14 (Week 5) enables earlier software development
- TVM MVP by Apr 30 (Week 15) enables proper integration
- 3 weeks vehicle-TVM integration (May 11-31)
- 4 weeks contingency buffer
- **Success probability:** 85-90%

---

### 10. Black Sesame Technology Analysis

**Source:** Company research for potential partnership (January 5, 2026 meeting)

**What It Provided:**
- Hardware alternative analysis: Black Sesame A1000 Pro chip
- 160 TOPS computing power, 6.0 TOPS/W efficiency
- 2.2× more energy efficient than NVIDIA Orin
- 40-60% lower cost than NVIDIA Orin

**Decision Impact:**
Not immediately relevant to current project (using AMD Ryzen 7 7840HS), but good reference for future cost reduction.

---

### 11. Japanese Translation Requirement

**Source:** Stakeholder communication needs

**What It Required:**
- All overview documents in Japanese
- Technical accuracy maintained
- Business-appropriate language
- Complete bilingual documentation set

**Result:**
5 key overview documents now have Japanese versions (_JA.md suffix)

---

## Key Decisions and Design Philosophy

### 1. Outdoor-First Philosophy

**Decision:** Design for outdoor, indoor becomes easier

**Rationale:**
- Outdoor is harder (weather, slopes, rough terrain)
- If it works outdoors, indoor operation is guaranteed
- Sensors must be IP65+ weatherproof
- Software must handle lighting changes, GPS loss, terrain variation

**Impact:**
- Higher sensor costs (outdoor-grade LiDAR)
- More robust control algorithms
- But: Broader deployment possibilities

---

### 2. No GPS Approach

**Decision:** Use LiDAR-based localization instead of GPS

**Rationale:**
- RTK GPS costs $2000-5000 per robot
- Doesn't work indoors
- Not needed for <1km operational range
- ParcelPal proved this works in production

**Technology:**
- Manual SLAM mapping once (offline)
- NDT scan matcher localization (online)
- ±10cm accuracy over 500m
- Zero drift accumulation

**Savings:** ~$3000 per robot

---

### 3. CPU-Only Processing (No Deep Learning)

**Decision:** No GPU-accelerated deep learning

**Rationale:**
- Deep learning requires expensive GPUs
- Binary obstacle detection (occupied/free) is sufficient
- Simpler algorithms are more reliable
- Faster development and testing

**Technology:**
- Point Cloud Library (PCL) for LiDAR processing
- RANSAC ground removal
- Classical computer vision (OpenCV, ArUco)

**Trade-off:**
- Less sophisticated perception
- But: Reliable, deterministic, debuggable

---

### 4. Swerve Drive vs. Mecanum Wheels

**Decision:** Use swerve drive (4 independent steering modules)

**Rationale:**
- Better outdoor performance on rough terrain
- More efficient (each wheel optimally oriented)
- Superior climbing ability
- Better traction control

**Complexity Trade-off:**
- More complex kinematics
- More complex mechanical design
- But: Worth it for outdoor capability

**ParcelPal Reference:**
ParcelPal uses differential drive (simpler), but it operates on smooth outdoor paths. Our robot needs to handle rough terrain and slopes.

---

### 5. Fleet Management System (TVM)

**Decision:** Build comprehensive fleet management from day one

**Rationale:**
- Single robot is a toy, fleet is a product
- Reservation system enables scheduling
- Real-time monitoring enables operations
- User roles enable hospital/facility integration

**Scope:**
- TVM Server (backend)
- Fleet Dashboard (web UI)
- Reservation System
- User Management (Admin, Operator, Nurse, Caregiver)
- Real-time Telemetry (1 Hz location updates)
- Voice Communication (VoIP)

**New Development:**
0% from existing systems, completely new by Unno's team

---

### 6. Agile Modular Architecture

**Decision:** Separate subsystems with clear interfaces

**Rationale:**
- Enables parallel development by independent teams
- Each subsystem can be tested independently
- Reduces integration risks
- Facilitates incremental delivery

**Subsystems:**
1. Swerve Drive Controller (Pankaj)
2. Navigation Stack (Pankaj)
3. Perception System (Pankaj)
4. Docking Subsystem (Pankaj)
5. Safety System (Pankaj)
6. UI + eHMI (Pankaj + Kirill)
7. TVM Client (Pankaj)
8. TVM Server (Unno)
9. Hardware Platform (Tsuchiya + Kirill)

**Interface Contracts:**
- Vehicle ↔ TVM Server: REST API + WebSocket
- Software ↔ Hardware: ROS 2 topics + CAN bus
- Internal: ROS 2 topics and services

---

### 7. Two-Phase Docking Approach

**Decision:** Coarse navigation + Fine visual servoing

**Rationale:**
- GPS/NDT localization: ±10cm accuracy (not enough for docking)
- ArUco markers: ±2-5mm accuracy (perfect for docking)
- Combine both for reliable outdoor docking

**Process:**
1. **Phase 1 - Coarse (Navigation):**
   - Robot navigates to "approach zone" using NDT localization
   - Gets within 5 meters of wheelchair

2. **Phase 2 - Fine (Visual Servoing):**
   - Detects ArUco marker on wheelchair
   - Uses visual servoing (PID control based on camera)
   - Achieves ±2-5mm precision attachment

**Success Criteria:** >90% docking success rate

---

### 8. Safety-First Design

**Decision:** Multiple layers of safety

**Rationale:**
- Transporting people, not just parcels
- Regulatory requirements (ISO 13849, SIL 2)
- Reputation risk

**Safety Layers:**
1. Hardware E-stop (hardwired to motors)
2. Software safety monitor (watchdog)
3. Sensor health checks (<1s detection)
4. Geofencing (no-go zones)
5. Slope detection (no docking on >5° slopes)
6. Occupant detection (passenger onboard?)
7. Speed limits (1.0 m/s with passenger)

**Response Time:** E-stop must activate <100ms

---

## Team Structure and Responsibilities

### Team 1: Pankaj (Vehicle Software)

**Role:** Solo developer + AI assistance (Claude Code, GitHub Copilot)

**Scope:**
- Autonomous navigation (ROS 2 + Nav2)
- Wheelchair docking (ArUco + visual servoing)
- Swerve drive control (inverse kinematics)
- Perception (LiDAR processing, obstacle detection)
- Safety systems (emergency stop, health monitoring)
- Local UI (React touch screen interface)
- TVM client (REST + WebSocket communication with server)
- eHMI integration (serial communication with ESP32)

**Timeline:** 18 weeks development + testing

**Tools:**
- ROS 2 Humble (C++ and Python)
- OpenCV (ArUco detection)
- PCL (Point Cloud Library)
- React + Next.js (UI)
- Nav2 (navigation framework)

---

### Team 2: Unno (Fleet Management)

**Role:** Fleet management system development (team size TBD)

**Scope:**
- TVM Server backend (REST API + WebSocket)
- Fleet Dashboard (web application)
- Reservation System (booking + scheduling)
- User Management (authentication, RBAC)
- Real-time Monitoring (vehicle tracking)
- Voice Communication (VoIP integration)
- Log Management (display vehicle logs)

**Timeline:** 20 weeks (15 weeks MVP + 5 weeks advanced features)

**Technology Stack (TBD by Unno):**
- Backend: Node.js / Python FastAPI / Java Spring Boot
- Database: PostgreSQL / MySQL / MongoDB
- Frontend: React / Vue / Angular
- Real-time: WebSocket / Socket.io

---

### Team 3: Tsuchiya + Kirill (Hardware)

**Tsuchiya - Mechanical & Electrical**

**Scope:**
- Chassis design and fabrication
- Swerve drive module assembly (4× modules)
- Power system (48V battery, BMS)
- Sensor integration (LiDAR, cameras, IMU)
- Motor controllers (CAN bus)
- Weatherproofing (IP54+)

**Kirill - Exterior & eHMI**

**Scope:**
- eHMI hardware (ESP32-S3, LEDs, speakers)
- eHMI firmware (state machine, LED patterns)
- Exterior panels and aesthetics
- Body panels (weatherproof)
- Lighting (headlights, taillights)

**Timeline:** Parallel with software (hardware ready as needed)

---

## Implementation Strategy

### Phase 0: Hardware - Fast Prototype (5 weeks)

**Goal:** Basic functioning platform by Feb 14, 2026

**Approach:**
- Week 1-2: Select sensors, define architecture
- Week 3-4: Assemble basic chassis, install drive motors
- Week 5: Add LiDAR + cameras + CAN communication

**Key Milestone:** "Walking Skeleton"
- Platform can drive (forward/backward)
- Sensors publishing data
- Pankaj can begin software development on real hardware

**Why Fast Prototype?**
- De-risks hardware dependencies early
- Enables real-hardware testing throughout software development
- 5 weeks faster than traditional approach

---

### Phase 0B: Hardware - Refinement (11 weeks)

**Goal:** Complete all hardware features

**Activities:**
- Add steering motors (complete swerve drive)
- Install suspension system
- Add remaining sensors (bumpers, IMU, etc.)
- Implement docking mechanism
- Complete control system integration
- Final weatherproofing

**Parallel:** Exterior CAD design (Kirill)

---

### Phase 1: Vehicle Software Development (18 weeks)

**Strategy:** Incremental development + testing

**Development Sequence:**
1. Swerve Drive (10 days)
2. Navigation (11 days)
3. Perception (8 days)
4. Docking (8 days)
5. Safety (6 days)
6. UI (4 days)
7. eHMI (3 days)

**Testing Sequence:**
- Unit Testing (17 days) - Module by module
- Integration Testing (14 days) - Subsystem integration

**Key Milestone:** Vehicle software complete by May 10, 2026

---

### Phase 2: Fleet Management Development (20 weeks)

**Strategy:** Critical MVP first, then secondary features

**Critical Path MVP (15 weeks):**
- Login + Admin (2 weeks)
- Route + Map (3 weeks)
- Reservation System (3 weeks)
- Floor Management (3 weeks)
- Walking Reservation (3 weeks)
- Status Dashboard (1 week)

**Secondary Features (5 weeks):**
- Resident Info (2 weeks)
- Advanced Features (2 weeks)
- Notifications + Voice (1 week)

**Key Milestone:** TVM MVP by Apr 30, 2026 (ready for integration)

---

### Phase 3: System Integration (3 weeks)

**Goal:** Integrate vehicle + hardware + TVM

**Week 1:** Vehicle SW + Hardware
- Verify all subsystems on final hardware
- Tune control parameters

**Week 2:** Vehicle + TVM (Telemetry)
- Vehicle → TVM data streaming
- Real-time location updates
- Battery status reporting

**Week 3:** Vehicle + TVM (Mission Control)
- TVM → Vehicle commands
- Mission dispatch functional
- End-to-end mission testing

**Key Milestone:** System integration by May 31, 2026

---

### Phase 4: Field Testing (3 weeks)

**Goal:** Validate in real-world conditions

**Week 1:** Component Testing
- Swerve drive outdoor performance
- Navigation long-distance (500m, 1km)

**Week 2:** Subsystem Testing
- Perception in various conditions
- Docking outdoor precision
- UI/eHMI usability

**Week 3:** System Integration
- Complete system testing
- Real-world deployment simulation
- User acceptance testing

**Success Criteria:**
- Mission success rate >90%
- Docking success rate >90%
- No critical safety issues

**Key Milestone:** Field validation by Jun 21, 2026

---

### Phase 5: Contingency & Polish (4 weeks)

**Goal:** Fix critical bugs and prepare for deployment

**Week 1-2:** Critical bug fixes (P0/P1)
**Week 3:** Verification and regression testing
**Week 4:** Final polish and documentation

**Key Milestone:** Final delivery Jul 19, 2026

---

## Timeline and Milestones

### Overall Timeline: 26 Weeks (Jan 15 - Jul 19, 2026)

```
Week 1-5:   Hardware Fast Prototype (Feb 14: Walking Skeleton ✅)
Week 6-16:  Hardware Refinement (May 1: Hardware Complete ✅)
Week 5-18:  Vehicle Software Development (May 10: Vehicle SW Complete ✅)
Week 2-21:  Fleet Management Development (Jun 4: TVM Complete ✅)
Week 19-21: System Integration (May 31: Integration Complete ✅)
Week 22-24: Field Testing (Jun 21: Field Validation Complete ✅)
Week 25-28: Contingency & Polish (Jul 19: Final Delivery ✅)
```

### Critical Path

```
Hardware Prototype (5 wks) → Vehicle SW (18 wks) → Integration (3 wks) → Testing (3 wks)
                                    ↓
                            TVM MVP (15 wks) ────────────────────────┘
```

### Key Dates

| Date | Milestone | Owner | Status |
|------|-----------|-------|--------|
| **Feb 14, 2026** | Walking Skeleton (Hardware) | Okinawa | ⏳ Target |
| **Mar 20, 2026** | Hardware Platform Complete | Okinawa | ⏳ Target |
| **Apr 30, 2026** | TVM MVP Complete | Unno | ⏳ Target |
| **May 1, 2026** | Control System Complete | Okinawa | ⏳ Target |
| **May 10, 2026** | Vehicle Software Complete | Pankaj | ⏳ Target |
| **May 31, 2026** | System Integration Complete | All Teams | ⏳ Target |
| **Jun 21, 2026** | Field Validation Complete | Pankaj + Kirill | ⏳ Target |
| **Jul 19, 2026** | Final Delivery | All Teams | ⏳ Target |

---

## Technology Stack Explained

### Vehicle Software (Pankaj)

**Operating System:**
- Ubuntu 22.04 LTS (Long Term Support)
- Why: Stable, well-supported, ROS 2 Humble native support

**Middleware:**
- ROS 2 Humble
- Why: Robot Operating System, industry standard, modular architecture

**Navigation:**
- Nav2 (ROS 2 Navigation Framework)
- NDT Scan Matcher (localization)
- Why: Proven, ParcelPal-validated approach

**Computer Vision:**
- OpenCV 4.x (camera processing)
- ArUco markers (docking precision)
- Why: Open-source, well-documented, CPU-efficient

**Point Cloud Processing:**
- PCL (Point Cloud Library)
- RANSAC (ground removal)
- Why: CPU-only, no GPU needed

**User Interface:**
- React 18 (frontend framework)
- Next.js 14 (full-stack framework)
- TypeScript (type safety)
- Why: Modern, responsive, maintainable

**Communication:**
- REST API (HTTP requests)
- WebSocket (real-time commands)
- JWT (authentication)
- Why: Standard protocols, language-agnostic

---

### Fleet Management (Unno - TBD)

**Backend Options:**
- Node.js + Express/NestJS
- Python + FastAPI/Django
- Java + Spring Boot
- Decision: By Unno based on team expertise

**Database Options:**
- PostgreSQL (relational, proven)
- MySQL (relational, popular)
- MongoDB (document, flexible)
- Decision: By Unno based on requirements

**Frontend Options:**
- React (most popular)
- Vue (lightweight)
- Angular (enterprise)
- Decision: By Unno based on team

**Real-Time:**
- WebSocket / Socket.io
- Server-Sent Events (SSE)
- Decision: WebSocket for bi-directional

**VoIP:**
- Twilio (commercial service)
- WebRTC (open standard)
- Asterisk (open-source PBX)
- Decision: By Unno based on budget

---

### Hardware Platform

**Compute:**
- GMKtec Nucbox K6
- AMD Ryzen 7 7840HS (8 cores, 16 threads)
- 32GB RAM
- Radeon 780M integrated GPU
- Why: Sufficient for CPU-based processing, cost-effective

**Sensors:**
- 3D LiDAR (outdoor-grade, IP65+)
- 2× RGB cameras (ArUco detection, obstacle awareness)
- 9-DOF IMU (MPU-9250 or similar)
- 4× drive encoders + 4× steer encoders
- Battery: Voltage, current, temperature (via BMS)

**Actuators:**
- 4× 6.5" hoverboard in-wheel motors (80W drive)
- 4× digital servo or power window motors (steering)
- Docking mechanism actuator

**Power:**
- 48V LiFePO4 battery (20Ah, 960Wh)
- BMS (Battery Management System) with CAN interface
- 48V → 12V DC-DC converter (sensors, PC)
- 48V → 5V DC-DC converter (ESP32, minor electronics)

**Communication:**
- CAN bus 500 kbit/s (motors)
- CAN bus 250 kbit/s (sensors)
- UART/USB (compute ↔ eHMI ESP32)
- Ethernet (compute ↔ LiDAR)
- USB (compute ↔ cameras)

**eHMI (External HMI):**
- ESP32-S3 microcontroller
- WS2812B LED strip (2-3 meters, 60+ LEDs/meter)
- I2S audio amplifier (MAX98357A)
- Speakers (5W+, weatherproof)
- SD card (audio files)

---

## Success Criteria

### Technical Success Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| **Navigation Accuracy** | ±10cm over 100m | GPS ground truth |
| **Docking Precision** | ±5mm | Measurement jig |
| **Docking Success Rate** | ≥90% | 50 attempts |
| **Mission Success Rate** | ≥90% | 100 missions |
| **Obstacle Detection** | >95% precision, >90% recall | Annotated dataset |
| **Battery Life** | ≥4 hours with passenger | Field test |
| **E-Stop Response** | <100ms | Oscilloscope |
| **Localization Drift** | <10cm over 500m | Long-distance test |

---

### Operational Success Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| **Passenger Satisfaction** | >4.5/5 | Survey |
| **Staff Satisfaction** | >4/5 | Operator feedback |
| **Uptime** | >98% | Operational logs |
| **Maintenance Time** | <2 hours/week | Maintenance records |
| **Mean Time Between Failures** | >40 hours | Failure logs |

---

### Project Success Metrics

| Metric | Target | Status |
|--------|--------|--------|
| **Schedule Adherence** | ±1 week of key milestones | ⏳ In progress |
| **Budget Adherence** | Within approved budget | ⏳ Pending |
| **Requirements Coverage** | 100% critical requirements | ⏳ In progress |
| **Code Coverage** | ≥80% | ⏳ Target |
| **Documentation Completeness** | 100% required docs | ✅ Complete |

---

## Final Summary

### What We Built (Documentation-wise)

- **118 markdown documents** (English + Japanese)
- **1,900+ requirements** across all subsystems
- **Complete system specification** from overview to deployment
- **Multi-team coordination** with clear interfaces
- **26-week implementation plan** with week-by-week details

### How We Got Here

1. Started with original task: "Build outdoor wheelchair transport robot"
2. Analyzed ParcelPal proven architecture (60-70% patterns reusable)
3. Added fleet management system (0% reusable - completely new)
4. Separated into 3 independent teams with clear interfaces
5. Created comprehensive documentation (6 weeks of documentation effort)
6. Optimized timeline based on risk analysis (85-90% success probability)

### Why It Matters

This isn't just a robot project. It's a **complete system** that enables:
- **Healthcare facilities** to transport wheelchair patients efficiently
- **Elderly care** to provide walking assistance automatically
- **Campus operations** to enable accessibility services

With **proven technology** (ParcelPal validation), **clear architecture** (modular subsystems), and **comprehensive planning** (26-week detailed roadmap).

---

**Document Version:** 1.0
**Last Updated:** December 17, 2025
**Maintained By:** Project Documentation Team
**Next Review:** After Phase 1 completion (May 2026)

---

**END OF COMPREHENSIVE SUMMARY**
