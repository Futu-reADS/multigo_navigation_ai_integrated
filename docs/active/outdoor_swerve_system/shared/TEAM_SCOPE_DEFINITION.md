# Team Scope Definition - Three-Team Separation

**Document Type:** Scope Definition & Boundaries
**Status:** Active
**Version:** 1.0
**Date:** December 19, 2025
**Purpose:** Prevent scope confusion and overlap between teams

---

## Table of Contents

1. [Overview](#1-overview)
2. [Team Responsibilities](#2-team-responsibilities)
3. [Scope Boundaries](#3-scope-boundaries)
4. [Interface Contracts](#4-interface-contracts)
5. [Decision Tree](#5-decision-tree)
6. [Common Questions](#6-common-questions)
7. [Escalation Process](#7-escalation-process)

---

## 1. Overview

### 1.1 Why This Document Exists

**Problem:**
- Original documentation mixed hardware, software, and server responsibilities
- Unclear who builds what
- Risk of duplicate work or gaps

**Solution:**
- Three independent teams with clear boundaries
- Interface contracts define communication
- This document resolves ambiguity

### 1.2 Three-Team Structure

```
┌─────────────────────────────────────────────────────────────┐
│                        Project Scope                         │
│                                                              │
│  ┌──────────────────┐  ┌──────────────────┐  ┌────────────┐│
│  │  Team 1: Pankaj  │  │  Team 2: Unno    │  │ Team 3:    ││
│  │  Vehicle Software│  │  TVM Server      │  │ Tsuchiya+  ││
│  │                  │  │                  │  │ Kiril      ││
│  │  ROS 2 nodes     │  │  Backend + DB    │  │ Hardware   ││
│  │  Navigation      │  │  Fleet dashboard │  │ Sensors    ││
│  │  Control         │  │  User mgmt       │  │ Motors     ││
│  │  TVM client      │  │  Mission dispatch│  │ Power      ││
│  │  Local UI        │  │  WebSocket       │  │ Exterior   ││
│  └────────┬─────────┘  └────────┬─────────┘  └──────┬─────┘│
│           │                     │                    │      │
│           └──────────┬──────────┴───────┬────────────┘      │
│                      │                  │                   │
│               ┌──────▼──────┐    ┌──────▼──────┐           │
│               │ TVM API     │    │ Hardware-   │           │
│               │ Interface   │    │ Software    │           │
│               │ Contract    │    │ Interface   │           │
│               └─────────────┘    └─────────────┘           │
└─────────────────────────────────────────────────────────────┘
```

### 1.3 Golden Rules

**Rule 1: Stay in Your Lane**
- Only work on tasks in your scope
- Don't write code for other teams
- Don't make decisions for other teams

**Rule 2: Interface Contracts Are Law**
- Cannot change interfaces without coordination
- Both sides must agree to changes
- Document all interface changes

**Rule 3: When in Doubt, Ask**
- Use this document's decision tree
- Escalate to senior if still unclear
- Don't assume - confirm scope

---

## 2. Team Responsibilities

### 2.1 Team 1: Pankaj (Vehicle Software)

**Primary Responsibility:** Software running ON the vehicle

**What You Build:**
- ✅ ROS 2 nodes (C++ and Python)
- ✅ Navigation system (Nav2, NDT localization, path planning)
- ✅ Docking controller (ArUco visual servoing)
- ✅ Swerve drive controller (inverse kinematics, motor commands)
- ✅ Perception pipeline (LiDAR processing, obstacle detection)
- ✅ Safety monitor (software E-stop, collision avoidance)
- ✅ TVM client (REST API calls, WebSocket connection to Unno's server)
- ✅ Local UI (React app on vehicle touchscreen)
- ✅ eHMI controller (serial commands to Kiril's ESP32)

**What You DON'T Build:**
- ❌ Hardware platform (Tsuchiya builds this)
- ❌ Sensor drivers (Tsuchiya provides, you configure)
- ❌ Motor controllers (Tsuchiya wires, you send commands)
- ❌ TVM server (Unno builds this)
- ❌ Database (Unno builds this)
- ❌ Fleet dashboard (Unno builds this)

**Your Folder:** `pankaj_vehicle_software/`
**Your Timeline:** 18 weeks
**Your Deliverables:** Working ROS 2 software that controls vehicle

---

### 2.2 Team 2: Unno (TVM Server)

**Primary Responsibility:** Backend server and fleet management

**What You Build:**
- ✅ TVM server backend (REST API + WebSocket)
- ✅ Database (PostgreSQL/MySQL/MongoDB - your choice)
- ✅ User authentication (JWT for vehicles and users)
- ✅ Fleet dashboard (React/Vue/Angular - your choice)
- ✅ Mission dispatch system (create missions, send to vehicles)
- ✅ Telemetry ingestion (receive data from vehicles)
- ✅ Reservation system (schedule transport)
- ✅ User management (admin, operator, caregiver roles)

**What You DON'T Build:**
- ❌ Vehicle software (Pankaj builds this)
- ❌ ROS 2 nodes (Pankaj builds this)
- ❌ Hardware platform (Tsuchiya builds this)
- ❌ Vehicle-side TVM client (Pankaj builds this)

**Your Folder:** `unno_tvm_server/`
**Your Timeline:** 20 weeks (15 weeks MVP + 5 weeks advanced)
**Your Deliverables:** Working server that manages fleet, accessible from network

---

### 2.3 Team 3: Tsuchiya + Kiril (Hardware)

**Primary Responsibility:** Physical platform

**Tsuchiya (Mechanical + Electrical + Sensors):**
- ✅ Chassis design and fabrication
- ✅ Swerve drive modules (4x steering + 4x drive motors)
- ✅ Power system (battery, BMS, DC-DC converters)
- ✅ Sensor installation (LiDAR, cameras, IMU, encoders, bumpers)
- ✅ Compute platform installation (GMKtec Nucbox K6)
- ✅ Cable management and weatherproofing
- ✅ ROS 2 sensor drivers (LiDAR, cameras, IMU - you install, Pankaj configures)
- ✅ CAN bus wiring and motor controller setup

**Kiril (Exterior + eHMI):**
- ✅ Body panels and exterior design
- ✅ eHMI hardware (ESP32, WS2812B LEDs, speakers)
- ✅ ESP32 firmware (receives serial commands from Pankaj)
- ✅ Lighting (headlights, taillights, turn signals)
- ✅ Protective covers (LiDAR, cameras)

**What You DON'T Build:**
- ❌ Vehicle control software (Pankaj builds this)
- ❌ Navigation algorithms (Pankaj builds this)
- ❌ TVM server (Unno builds this)
- ❌ Fleet dashboard (Unno builds this)

**Your Folder:** `tsuchiya_kiril_hardware/`
**Your Timeline:** 16 weeks (5 weeks prototype + 11 weeks complete)
**Your Deliverables:** Physical platform that Pankaj's software controls

---

## 3. Scope Boundaries

### 3.1 Vehicle Software vs Hardware

**Boundary Question:** "Who installs ROS 2 drivers for sensors?"

**Answer:**
- **Tsuchiya:** Installs standard ROS 2 packages (`apt install ros-humble-velodyne-driver`)
- **Tsuchiya:** Provides sensor specifications (model, connection type, I2C address)
- **Pankaj:** Configures driver parameters (topic names, frame IDs, calibration)
- **Pankaj:** Integrates driver output into navigation/perception pipeline

**Example:**
```
LiDAR Driver Installation:

Tsuchiya does:
- Mount LiDAR on vehicle
- Connect power (12V) and data (Ethernet)
- Install driver: sudo apt install ros-humble-ouster-ros
- Provide spec sheet to Pankaj (model OS1-64, IP 192.168.1.10)

Pankaj does:
- Configure driver launch file (IP address, topic name)
- Set coordinate frame (sensor_frame -> base_link)
- Integrate point cloud into perception pipeline
- Test and verify data quality
```

---

**Boundary Question:** "Who writes motor control code?"

**Answer:**
- **Tsuchiya:** Wires CAN bus, configures motor controllers (CAN IDs, baudrate)
- **Tsuchiya:** Provides CAN protocol specification (message format, units)
- **Pankaj:** Writes ROS node that sends CAN messages (velocity commands)
- **Pankaj:** Implements swerve drive inverse kinematics
- **Pankaj:** Implements safety limits (max velocity, acceleration)

**Example:**
```
Motor Control:

Tsuchiya does:
- Wire CAN bus (twisted pair, 120Ω termination)
- Configure motor controller 1 with CAN ID 0x101
- Document: "Send 2 bytes, byte 0 = velocity (-127 to +127), byte 1 = checksum"

Pankaj does:
- Write ROS node subscribing to /control/swerve/drive_velocities
- Convert m/s to motor controller units (-127 to +127)
- Send CAN message via python-can library
- Monitor encoder feedback for closed-loop control
```

---

**Boundary Question:** "Who handles E-stop?"

**Answer:**
- **Tsuchiya:** Wires hardware E-stop circuit (dual-channel, cuts motor power)
- **Tsuchiya:** Connects E-stop status to GPIO pin
- **Pankaj:** Reads GPIO pin, publishes /safety/estop topic
- **Pankaj:** Implements software response (stop all commands, cancel mission)
- **Both:** Test together (press E-stop → motors stop within 100ms)

---

### 3.2 Vehicle Software vs TVM Server

**Boundary Question:** "Who implements user authentication?"

**Answer:**
- **Unno:** Implements user authentication (username/password login for dashboard)
- **Unno:** Generates JWT tokens for both vehicles and users
- **Pankaj:** Implements vehicle authentication (vehicle_id + api_key → JWT token)
- **Pankaj:** Stores JWT token in vehicle, sends in API requests

**Example:**
```
Vehicle Authentication:

Unno does:
- Create POST /api/v1/auth/login endpoint
- Validate vehicle_id and api_key against database
- Generate JWT token (exp: 1 hour)
- Return token to vehicle

Pankaj does:
- Send login request when vehicle starts
- Store token in memory
- Include token in all API requests (Authorization: Bearer <token>)
- Refresh token before expiration
```

---

**Boundary Question:** "Who decides mission waypoints?"

**Answer:**
- **Unno:** Provides UI for operator to select pickup/dropoff locations
- **Unno:** Stores mission in database
- **Unno:** Sends mission waypoints to vehicle via WebSocket
- **Pankaj:** Receives mission, validates waypoints (within map bounds)
- **Pankaj:** Plans path between waypoints (using Nav2)
- **Pankaj:** Executes navigation, sends status updates back to Unno

**Example:**
```
Mission Dispatch:

Unno does:
- Operator clicks "Pickup: Building A, Dropoff: Building B" on dashboard
- Convert locations to waypoints: [{x: 10, y: 5}, {x: 15, y: 8}]
- Send via WebSocket: {"type": "mission.dispatch", "waypoints": [...]}

Pankaj does:
- Receive WebSocket message
- Validate waypoints (check if within map, reachable)
- Plan path using Nav2 (handles obstacles, smoothing)
- Execute navigation (send velocity commands to motors)
- Send status updates: "navigating", "arrived", "completed"
```

---

### 3.3 Hardware vs TVM Server

**Boundary Question:** "Do hardware and server teams interact?"

**Answer:** NO - they communicate ONLY through Pankaj's vehicle software

**Example:**
```
Sensor Data Flow:

Tsuchiya → Pankaj → Unno

Tsuchiya:
- LiDAR publishes /sensors/lidar/points (ROS topic)

Pankaj:
- Reads LiDAR data for navigation (local processing)
- Extracts vehicle location from localization
- Sends location to TVM via REST API

Unno:
- Receives telemetry via POST /api/v1/telemetry
- Stores in database
- Displays on dashboard

Tsuchiya and Unno NEVER communicate directly.
```

---

## 4. Interface Contracts

### 4.1 Critical Interface Documents

**These documents define team boundaries:**

| Interface | Teams | Document | Status |
|-----------|-------|----------|--------|
| **TVM API** | Pankaj ↔ Unno | `shared/INTERFACES/TVM_API_SPECIFICATION.md` | ✅ Final |
| **Hardware-Software** | Pankaj ↔ Tsuchiya+Kiril | `shared/INTERFACES/HARDWARE_SOFTWARE_INTERFACE.md` | ✅ Final |
| **Data Models** | Pankaj ↔ Unno | `shared/INTERFACES/TVM_DATA_MODELS.md` | ✅ Final |

**Golden Rule:** CANNOT change interfaces without coordination!

### 4.2 Interface Change Process

**If you need to change an interface:**

1. **Identify affected teams**
   - TVM API change: Pankaj + Unno
   - Hardware interface change: Pankaj + Tsuchiya

2. **Propose change**
   - Document: old spec → new spec
   - Justification: why needed?
   - Impact: what breaks? Migration path?

3. **Get approval**
   - Both teams must agree
   - Senior approval for major changes
   - Update interface document (version bump if needed)

4. **Implement simultaneously**
   - Both teams update code at same time
   - Test integration after changes
   - Document migration if backward compatibility needed

**Example:**
```
Proposed Change: Add "tire_pressure" field to telemetry

Step 1: Pankaj proposes
- Old: telemetry JSON has battery, location, status
- New: Add "tire_pressure": [45.2, 44.8, 45.0, 45.5] (4 values, psi)
- Why: Needed for predictive maintenance
- Impact: Unno must update database schema, Pankaj must read sensors

Step 2: Coordinate
- Pankaj emails Unno with proposal
- Unno reviews, asks clarification on units (psi vs bar)
- Both agree on psi

Step 3: Update interface doc
- Update TVM_DATA_MODELS.md (add tire_pressure field)
- Version bump: v1.0 → v1.1

Step 4: Implement
- Unno adds tire_pressure column to vehicle_telemetry table
- Pankaj adds tire pressure sensors to telemetry JSON
- Test: verify data appears in Unno's database
```

---

## 5. Decision Tree

### 5.1 "Who Does This Task?" Decision Tree

```
START: I have a task. Who does it?

├─ Does it run ON the vehicle?
│  ├─ YES → Is it hardware or software?
│  │  ├─ Hardware (physical) → Tsuchiya or Kiril
│  │  │  ├─ Mechanical/Electrical/Sensors → Tsuchiya
│  │  │  └─ Exterior/eHMI → Kiril
│  │  └─ Software (code) → Pankaj
│  │     ├─ ROS 2 nodes → Pankaj
│  │     ├─ Control algorithms → Pankaj
│  │     ├─ Local UI → Pankaj
│  │     └─ TVM client → Pankaj
│  │
│  └─ NO → Is it server-side or management?
│     ├─ Server backend → Unno
│     ├─ Database → Unno
│     ├─ Fleet dashboard → Unno
│     └─ User management → Unno
│
└─ Is it an interface between teams?
   ├─ YES → Both teams coordinate
   │  ├─ Update interface document first
   │  └─ Both teams implement simultaneously
   │
   └─ Still unclear? → Escalate to senior
```

### 5.2 Example Scenarios

**Scenario 1: "Implement obstacle avoidance"**
```
Question: Does it run ON the vehicle?
Answer: YES (runs in ROS 2 navigation stack)

Question: Hardware or software?
Answer: Software (algorithm)

Result: Pankaj's responsibility
- Pankaj implements dynamic obstacle avoidance in Nav2
- Uses LiDAR data (provided by Tsuchiya's sensors)
- Outputs velocity commands (consumed by Tsuchiya's motors)
```

**Scenario 2: "Add battery monitoring to dashboard"**
```
Question: Does it run ON the vehicle?
Answer: Partially (battery data collected on vehicle, displayed on server)

Split responsibility:
- Tsuchiya: Install battery BMS, publish /sensors/battery/status topic
- Pankaj: Read battery topic, send to TVM via telemetry API
- Unno: Receive telemetry, store in database, display on dashboard

Interface: TVM_API_SPECIFICATION.md (telemetry endpoint)
```

**Scenario 3: "Make LEDs blink when docking"**
```
Question: Does it run ON the vehicle?
Answer: YES (LED control hardware on vehicle)

Question: Hardware or software?
Answer: Both!

Split responsibility:
- Kiril: Wires WS2812B LEDs, writes ESP32 firmware (receives serial commands)
- Pankaj: Detects docking state, sends serial command to ESP32

Interface: HARDWARE_SOFTWARE_INTERFACE.md (eHMI serial protocol)
```

**Scenario 4: "Create user registration page"**
```
Question: Does it run ON the vehicle?
Answer: NO (web dashboard)

Question: Server-side or management?
Answer: Server-side (dashboard UI)

Result: Unno's responsibility
- Unno creates user registration page in fleet dashboard
- Unno implements backend user creation API
- Unno stores user credentials in database
```

---

## 6. Common Questions

### Q1: "Can I use libraries/tools chosen by another team?"

**Answer:** Yes, but coordinate

**Examples:**
- Pankaj uses ROS 2 → Tsuchiya must install ROS 2 on compute platform
- Unno chooses PostgreSQL → Pankaj doesn't care (server internal)
- Tsuchiya chooses specific LiDAR model → Pankaj must use compatible driver

**Rule:** Technology choices that cross team boundaries require coordination

---

### Q2: "What if I find a bug in another team's scope?"

**Answer:** Report it, don't fix it yourself

**Process:**
1. Document the bug (what, where, how to reproduce)
2. Notify the responsible team
3. If urgent, escalate to senior
4. The responsible team fixes it

**Example:**
```
Pankaj finds bug in TVM API (returns 500 error on valid request)

Correct:
- Pankaj reports to Unno: "POST /api/v1/telemetry returns 500 when battery_percent > 100"
- Unno investigates and fixes validation logic
- Unno deploys fix, notifies Pankaj

Incorrect:
- Pankaj modifies Unno's server code (out of scope!)
```

---

### Q3: "Can I suggest improvements to another team's work?"

**Answer:** Yes! Suggestions welcome, but implementation is their choice

**Example:**
```
Pankaj suggests to Unno: "Dashboard could show battery history graph"

Unno can:
- Accept and implement
- Accept but defer to post-pilot
- Decline (out of scope or low priority)

Pankaj should NOT:
- Implement dashboard feature themselves
- Insist on specific implementation
```

---

### Q4: "What if our timelines don't align?"

**Answer:** Use mocks and stubs for independent development

**Examples:**
- **Pankaj before Unno's server is ready:** Use mock TVM server (provided in implementation guide)
- **Pankaj before Tsuchiya's hardware is ready:** Use ROS 2 simulation (Gazebo)
- **Unno before Pankaj's vehicle is ready:** Use mock vehicle client (provided in implementation guide)

**Rule:** Don't block on other teams - develop independently, integrate later

---

### Q5: "Who deploys the complete system?"

**Answer:** Each team deploys their own components

**Deployment:**
- **Pankaj:** Deploys ROS 2 software to vehicle compute (Nucbox K6)
- **Unno:** Deploys TVM server to cloud/on-premise server
- **Tsuchiya+Kiril:** Assembles physical platform

**Integration testing:** Week 12-15 (all teams together)

---

## 7. Escalation Process

### 7.1 When to Escalate

**Escalate to senior when:**
- Scope boundary is unclear after checking this document
- Interface change requires major redesign
- Teams disagree on who owns a task
- Timeline conflict (blocking dependency)
- Resource conflict (budget, equipment)

### 7.2 Escalation Template

```
To: Senior
Subject: Scope Clarification Needed - [Brief Description]

Issue: [What is unclear?]

Teams Involved: [Pankaj / Unno / Tsuchiya+Kiril]

Current Understanding:
- Team A believes: [...]
- Team B believes: [...]

Impact if Not Resolved:
- Timeline: [delay in weeks]
- Scope: [what work is blocked]

Proposed Solutions:
1. Option A: [...]
2. Option B: [...]

Recommendation: [Your preferred option]

Requested Decision By: [Date]
```

---

## 8. Summary

### 8.1 Key Principles

**1. Clear Boundaries**
- Each team has exclusive responsibility for their scope
- No overlap, no gaps

**2. Interface Contracts**
- Teams communicate via defined interfaces
- Interfaces cannot change without coordination

**3. Independent Development**
- Teams work in parallel using mocks
- Integration happens at defined milestones

**4. Escalation When Needed**
- Don't assume - ask and confirm
- Senior resolves disputes

### 8.2 Quick Reference Table

| Question | Team |
|----------|------|
| Navigation algorithm | Pankaj |
| Motor wiring | Tsuchiya |
| User login (dashboard) | Unno |
| Docking controller | Pankaj |
| Battery installation | Tsuchiya |
| Database schema | Unno |
| ROS 2 nodes | Pankaj |
| LED wiring | Kiril |
| ESP32 firmware | Kiril |
| TVM client (vehicle-side) | Pankaj |
| TVM server (backend) | Unno |
| Fleet dashboard | Unno |
| Chassis fabrication | Tsuchiya |
| Sensor calibration (software) | Pankaj |
| Sensor installation (hardware) | Tsuchiya |

### 8.3 Success Criteria

**You're following scope correctly if:**
- ✅ You only work on tasks in your folder
- ✅ You coordinate interface changes with other teams
- ✅ You use mocks when waiting for other teams
- ✅ You escalate unclear scope questions
- ✅ You deliver what's in your README file

**Red flags (scope violation):**
- ❌ Writing code in another team's folder
- ❌ Changing interfaces without coordination
- ❌ Blocked waiting for other teams (should use mocks)
- ❌ Implementing features in another team's scope
- ❌ Making decisions for other teams

---

## 9. Document History

**Version 1.0 (December 19, 2025):**
- Initial scope definition
- Three-team separation (Pankaj, Unno, Tsuchiya+Kiril)
- Interface contracts defined
- Decision tree created
- Common questions answered

**Approval:**
- Senior: [Pending]
- Pankaj: Acknowledged
- Unno: [Pending]
- Tsuchiya: [Pending]
- Kiril: [Pending]

---

**Document Status:** ✅ Ready for Team Review
**Location:** `shared/TEAM_SCOPE_DEFINITION.md`
**Applicable To:** All three teams

**Next Steps:**
1. All teams review this document
2. All teams acknowledge scope understanding
3. Senior approves scope boundaries
4. Begin independent development

---

**END OF SCOPE DEFINITION**
