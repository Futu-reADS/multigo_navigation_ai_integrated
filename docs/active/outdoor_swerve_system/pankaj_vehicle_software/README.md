# Vehicle Software (ROS 2) - Pankaj's Scope
# 車両ソフトウェア（ROS 2） - Pankajの範囲

**Team Lead:** Pankaj
**Scope:** ROS 2 Vehicle Software ONLY
**Timeline:** 18 weeks development + testing
**Status:** Ready for implementation

---

## 🎯 What You Build

**ROS 2 Software Stack for Autonomous Navigation:**
- ✅ **Navigation System** - Nav2 + NDT localization (no GPS)
- ✅ **Docking Controller** - ArUco visual servoing (±2-5mm precision)
- ✅ **Swerve Drive Controller** - Inverse kinematics, omnidirectional motion
- ✅ **Perception Pipeline** - LiDAR processing, obstacle detection (CPU-only)
- ✅ **Safety Monitor** - Software watchdog, emergency stop logic
- ✅ **Local UI** - React touch screen interface
- ✅ **eHMI Control** - ESP32 communication (serial)
- ✅ **TVM Client** - REST + WebSocket API calls to server

---

## ❌ What You DON'T Build

- ❌ **Hardware Platform** (Tsuchiya + Kiril's responsibility)
  - Not mechanical design, electrical wiring, sensor mounting
  - Not motor controllers, battery management, physical systems

- ❌ **TVM Server** (Unno's responsibility)
  - Not backend server, database, fleet dashboard
  - Not user management, reservation system

**Your Focus:** Software that runs ON the vehicle (ROS 2 nodes, controllers, algorithms)

---

## 🔗 Your Interfaces

### Interface to Hardware (Tsuchiya + Kiril)

**Document:** `04_INTERFACES/HARDWARE_SOFTWARE_INTERFACE.md`

**What Hardware Provides to You:**
- ROS 2 topics from sensors (LiDAR, cameras, IMU, encoders, battery)
- CAN bus responses from motor controllers
- Serial messages from eHMI (ESP32)

**What You Provide to Hardware:**
- ROS 2 command topics (velocity commands, steering angles)
- CAN bus motor control commands
- Serial commands to eHMI (LED patterns, sounds, status)

### Interface to TVM Server (Unno)

**Documents:**
- `04_INTERFACES/TVM_API_SPECIFICATION.md`
- `04_INTERFACES/TVM_DATA_MODELS.md`

**What You Send to Server:**
- Vehicle telemetry (location, status, battery, errors) via REST API
- Heartbeat every 1 second

**What Server Sends to You:**
- Mission commands (dispatch, cancel) via WebSocket
- Emergency stop commands
- Configuration updates

---

## 🛠️ Technology Stack

**Operating System:**
- Ubuntu 22.04 LTS

**Middleware:**
- ROS 2 Humble (C++ and Python)

**Key Libraries:**
- **Navigation:** Nav2, NDT scan matcher
- **Computer Vision:** OpenCV 4.x (ArUco detection)
- **Point Cloud:** PCL (LiDAR processing)
- **UI:** React 18 + Next.js 14 + TypeScript
- **TVM Client:** Python requests + websockets

**No GPU Required:** All processing runs on CPU (AMD Ryzen 7 7840HS integrated graphics)

---

## 📁 Folder Structure

```
pankaj_vehicle_software/
├── 01_REQUIREMENTS/          ← What the software must do
│   ├── NAVIGATION_REQUIREMENTS.md
│   ├── DOCKING_SYSTEM_REQUIREMENTS.md
│   ├── SWERVE_DRIVE_REQUIREMENTS.md (software control only)
│   ├── PERCEPTION_REQUIREMENTS.md
│   ├── SAFETY_REQUIREMENTS.md
│   ├── PERFORMANCE_REQUIREMENTS.md
│   ├── UI_EHMI_REQUIREMENTS.md
│   ├── SOFTWARE_REQUIREMENTS.md
│   ├── VEHICLE_TVM_CLIENT_REQUIREMENTS.md
│   ├── ERROR_HANDLING_REQUIREMENTS.md
│   ├── PRIVACY_REQUIREMENTS.md
│   └── SECURITY_REQUIREMENTS.md
│
├── 02_ARCHITECTURE/          ← How it's organized
│   ├── NAVIGATION_SUBSYSTEM_ARCHITECTURE.md
│   ├── DOCKING_SUBSYSTEM_ARCHITECTURE.md
│   ├── SWERVE_DRIVE_ARCHITECTURE.md
│   ├── PERCEPTION_SUBSYSTEM_ARCHITECTURE.md
│   ├── SAFETY_SUBSYSTEM_ARCHITECTURE.md
│   ├── UI_ARCHITECTURE.md
│   ├── EHMI_ARCHITECTURE.md
│   ├── VEHICLE_OVERALL_ARCHITECTURE.md
│   ├── DEPLOYMENT_ARCHITECTURE.md (software deployment)
│   └── SYSTEM_INTEGRATION_ARCHITECTURE.md
│
├── 03_DESIGN/                ← Detailed designs
│   ├── NAVIGATION_CONTROLLER_DESIGN.md
│   ├── DOCKING_CONTROLLER_DESIGN.md
│   ├── SWERVE_DRIVE_CONTROLLER_DESIGN.md
│   ├── PERCEPTION_PIPELINE_DESIGN.md
│   ├── SAFETY_MONITOR_DESIGN.md
│   ├── UI_COMPONENTS_DESIGN.md
│   └── EHMI_FIRMWARE_DESIGN.md
│
├── 04_INTERFACES/            ← Interfaces to other teams
│   ├── TVM_API_SPECIFICATION.md ← To Unno (critical!)
│   ├── TVM_DATA_MODELS.md ← To Unno
│   ├── HARDWARE_SOFTWARE_INTERFACE.md ← To Tsuchiya (critical!)
│   └── ROS2_TOPICS.md ← Internal
│
├── 05_DEVELOPMENT/           ← How to develop
│   ├── VEHICLE_SOFTWARE_DEVELOPMENT_GUIDE.md
│   ├── CODE_STANDARDS_AND_PRACTICES.md
│   └── DEVELOPMENT_SETUP_GUIDE.md
│
├── 06_TESTING/               ← How to test
│   └── TESTING_REQUIREMENTS.md
│
└── README.md                 ← This file
```

---

## 🚀 Quick Start

### 1. Read Interface Documents First

**Critical - Read these before coding:**
1. `04_INTERFACES/TVM_API_SPECIFICATION.md` - How to communicate with Unno's server
2. `04_INTERFACES/HARDWARE_SOFTWARE_INTERFACE.md` - How to communicate with hardware
3. `04_INTERFACES/ROS2_TOPICS.md` - Internal ROS topic structure

### 2. Review Requirements

Start with these to understand what to build:
1. `01_REQUIREMENTS/NAVIGATION_REQUIREMENTS.md`
2. `01_REQUIREMENTS/DOCKING_SYSTEM_REQUIREMENTS.md`
3. `01_REQUIREMENTS/SWERVE_DRIVE_REQUIREMENTS.md`

### 3. Set Up Development Environment

Follow: `05_DEVELOPMENT/DEVELOPMENT_SETUP_GUIDE.md`

### 4. Begin Implementation

Suggested order:
1. **Week 1-2:** Swerve drive controller (basic motion)
2. **Week 3-4:** Navigation stack (Nav2 + NDT)
3. **Week 5-6:** Perception pipeline (LiDAR processing)
4. **Week 7-8:** Docking controller (ArUco detection)
5. **Week 9-10:** Safety monitor
6. **Week 11-12:** Local UI
7. **Week 13-14:** eHMI integration
8. **Week 15-16:** TVM client
9. **Week 17-18:** Integration testing

---

## 🔑 Key Design Principles

### 1. Software ONLY
- Don't worry about hardware specs (motors, batteries, sensors)
- Tsuchiya provides hardware according to interface spec
- You just consume ROS topics and send commands

### 2. TVM Client, NOT Server
- You call APIs, you don't implement them
- Unno builds the server
- Your job: Send telemetry, receive commands

### 3. Pilot-Level Simplicity
- Basic authentication (JWT)
- Basic security (HTTPS, API keys)
- No enterprise features (disaster recovery, advanced monitoring)
- Focus on functionality

### 4. CPU-Only Processing
- No GPU required
- No deep learning
- Classical algorithms (RANSAC, OpenCV, PCL)
- Runs on integrated graphics

---

## 📋 Dependencies

### From Tsuchiya + Kiril (Hardware Team)

**What you need from them:**
- [ ] Hardware platform assembled and powered on
- [ ] ROS 2 drivers publishing sensor topics
- [ ] Motor controllers responding to CAN commands
- [ ] eHMI ESP32 responding to serial commands

**Coordination:**
- Review `shared/INTERFACES/HARDWARE_SOFTWARE_INTERFACE.md` together
- Test each interface incrementally
- Mock hardware for early software testing

### From Unno (TVM Server Team)

**What you need from them:**
- [ ] TVM server running and accessible via network
- [ ] REST API endpoints implemented
- [ ] WebSocket server accepting connections
- [ ] Authentication (JWT) working

**Coordination:**
- Review `shared/INTERFACES/TVM_API_SPECIFICATION.md` together
- Use mock server for early client testing
- Integration testing at Week 15

---

## ⚠️ Important Notes

### Scope Boundaries

**If you're doing these, you're OUTSIDE your scope:**
- ❌ Designing mechanical parts (CAD models)
- ❌ Writing electrical schematics
- ❌ Selecting motors or batteries
- ❌ Building database schemas for TVM
- ❌ Creating fleet management dashboard

**Your scope is ONLY:**
- ✅ Writing ROS 2 nodes (C++ and Python)
- ✅ Implementing control algorithms
- ✅ Creating local vehicle UI (React)
- ✅ Writing TVM API client code

### Privacy & Security

**Legal Review Required:**
- `01_REQUIREMENTS/PRIVACY_REQUIREMENTS.md` must be reviewed by Li San
- Don't implement privacy features until Li San approves
- Coordinate with senior before finalizing security requirements

### Testing Philosophy

- Unit test each ROS node independently
- Integration test subsystems (navigation, docking, etc.)
- Use mock hardware when hardware not ready
- Use mock TVM server when server not ready
- Full system testing only after all teams integrate

---

## 📞 Contacts

**Your Team:** Pankaj (Solo + AI assistance)

**Hardware Team:** Tsuchiya + Kiril
- **Interface:** HARDWARE_SOFTWARE_INTERFACE.md
- **Coordination:** Weekly integration meetings

**Server Team:** Unno
- **Interface:** TVM_API_SPECIFICATION.md
- **Coordination:** API review meetings

**Legal:** Li San
- **Review:** Privacy requirements
- **Status:** Pending review

**Senior:** [Name]
- **Approval:** Final architecture and scope

---

## ✅ Success Criteria

### Technical Metrics

- [ ] Navigation accuracy: ±10cm over 100m
- [ ] Docking precision: ±5mm
- [ ] Docking success rate: ≥90%
- [ ] Mission success rate: ≥90%
- [ ] Obstacle detection: >95% precision, >90% recall
- [ ] Battery life: ≥4 hours with passenger
- [ ] E-stop response: <100ms

### Code Quality

- [ ] All ROS nodes pass unit tests
- [ ] Code coverage ≥80%
- [ ] Code follows standards in CODE_STANDARDS_AND_PRACTICES.md
- [ ] All interfaces implemented correctly
- [ ] Documentation complete

### Integration

- [ ] Hardware-software interface working
- [ ] TVM API client working
- [ ] All subsystems integrated
- [ ] Safety systems validated

---

**Document Version:** 1.0
**Last Updated:** December 19, 2025
**Status:** Ready for implementation
**Next Action:** Begin Phase 3 (move documents to this folder)

---

**Happy Coding! 🚀**
