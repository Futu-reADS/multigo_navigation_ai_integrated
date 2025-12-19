# Multi-Team Wheelchair Transport Robot Documentation

**Project:** Outdoor-First Wheelchair Transport Robot with Fleet Management
**Version:** 1.0
**Date:** December 20, 2025
**Status:** Documentation complete, awaiting senior approval

---

## 🚀 Quick Start

### For Project Stakeholders
- **Senior Management:** Read `FINAL_SUMMARY_FOR_SENIOR_APPROVAL.md`
- **Legal Team (Li San):** Read `PRIVACY_LEGAL_REVIEW_PACKAGE_FOR_LI_SAN.md`

### For Development Teams
- **Pankaj Team (Vehicle Software):** Start with `pankaj_vehicle_software/README.md`
- **Unno Team (TVM Server):** Start with `unno_tvm_server/README.md`
- **Tsuchiya/Kiril Team (Hardware):** Start with `tsuchiya_kiril_hardware/README.md`
- **All Teams:** Read `shared/TEAM_SCOPE_DEFINITION.md` for team boundaries

### Critical Interface Contracts (FROZEN for Pilot)
1. `shared/INTERFACES/TVM_API_SPECIFICATION.md` - REST + WebSocket API (Pankaj ↔ Unno)
2. `shared/INTERFACES/HARDWARE_SOFTWARE_INTERFACES.md` - ROS topics + CAN + Serial (Pankaj ↔ Tsuchiya/Kiril)
3. `shared/INTERFACES/TVM_DATA_MODELS.md` - Database schema + data structures (Pankaj ↔ Unno)

---

## 📁 Folder Structure

```
multigo/
│
├── README.md                               ← YOU ARE HERE
├── FINAL_SUMMARY_FOR_SENIOR_APPROVAL.md    ← Complete project summary
├── PRIVACY_LEGAL_REVIEW_PACKAGE_FOR_LI_SAN.md
├── DOCS_FOLDER_ANALYSIS_AND_CLEANUP_RECOMMENDATIONS.md
│
├── pankaj_vehicle_software/                (69 files - Vehicle software)
│   ├── README.md                           ← Complete guide (507 lines)
│   ├── 01_REQUIREMENTS/
│   ├── 02_ARCHITECTURE/
│   ├── 03_DESIGN/
│   ├── 04_INTERFACES/
│   ├── 05_DEVELOPMENT/
│   └── 06_TESTING/
│
├── unno_tvm_server/                        (19 files - TVM Server)
│   ├── README.md                           ← Complete guide (410 lines)
│   ├── TVM_API_IMPLEMENTATION_GUIDE_FOR_UNNO.md  ← Step-by-step + code (742 lines)
│   ├── 01_REQUIREMENTS/
│   ├── 02_ARCHITECTURE/
│   ├── 03_DESIGN/
│   ├── 04_INTERFACES/
│   └── 05_DEVELOPMENT/
│
├── tsuchiya_kiril_hardware/                (15 files - Hardware)
│   ├── README.md                           ← Complete guide (507 lines)
│   ├── HARDWARE_REQUIREMENTS_FOR_TSUCHIYA.md  ← Specs + ROS examples (853 lines)
│   ├── 01_REQUIREMENTS/
│   ├── 02_ARCHITECTURE/
│   ├── 03_DESIGN/
│   ├── 04_INTERFACES/
│   └── 05_DEVELOPMENT/
│
└── shared/                                 (7 files - Cross-team resources)
    ├── README.md                           ← How to use shared docs (380 lines)
    ├── TEAM_SCOPE_DEFINITION.md            ← Who does what (470+ lines) **READ THIS**
    └── INTERFACES/                         ← **FROZEN CONTRACTS**
        ├── TVM_API_SPECIFICATION.md        (1,425 lines)
        ├── TVM_DATA_MODELS.md              (1,361 lines)
        ├── HARDWARE_SOFTWARE_INTERFACES.md (1,010 lines)
        ├── ROS2_TOPICS.md
        └── INTERFACE_SPECIFICATIONS_COMPLETE.md
```

**Total:** 113 markdown files (clean, organized, no duplicates)

---

## 👥 Three-Team Structure

### Team 1: Pankaj (Vehicle Software)
- **Technology:** ROS 2 Humble, Ubuntu 22.04, C++/Python
- **Responsibility:** Everything running ON the vehicle
  - Autonomous navigation (NDT localization, Nav2)
  - Wheelchair docking (ArUco detection, visual servoing)
  - Swerve drive control
  - Perception (LiDAR processing)
  - Safety systems (E-stop, bumpers, watchdog)
  - Local UI (touch screen)
  - TVM client interface
- **Timeline:** 18 weeks
- **Folder:** `pankaj_vehicle_software/`

### Team 2: Unno (TVM Server)
- **Technology:** Backend (Node.js/Python/Java), PostgreSQL, React
- **Responsibility:** Fleet management server & dashboard
  - TVM server backend (REST API + WebSocket)
  - Database (PostgreSQL)
  - Fleet dashboard (web UI)
  - User authentication (JWT)
  - Mission dispatch, telemetry ingestion
- **Timeline:** 20 weeks (15W MVP + 5W advanced)
- **Folder:** `unno_tvm_server/`

### Team 3: Tsuchiya + Kiril (Hardware)
- **Tsuchiya:** Mechanical, electrical, sensors, compute
- **Kiril:** Exterior design, eHMI (ESP32 firmware)
- **Responsibility:** Physical robot platform
  - Chassis, motors, power, sensors
  - Compute installation (GMKtec Nucbox K6)
  - ROS 2 sensor drivers (LiDAR, IMU, cameras, bumpers)
  - CAN bus wiring
  - ESP32-S3 firmware (LED/audio control)
- **Timeline:** 16 weeks (Tsuchiya), 10 weeks (Kiril)
- **Folder:** `tsuchiya_kiril_hardware/`

---

## 🎯 Key Technical Decisions

- **No GPS:** Indoor/outdoor facility - using NDT localization instead
- **CPU-only:** GMKtec Nucbox K6 (no discrete GPU) - optimized perception
- **Swerve Drive:** Omnidirectional motion (4 independent modules)
- **ArUco Docking:** Dual-camera visual servoing (±2-5mm precision)
- **Simplified Security:** 30 requirements (pilot-appropriate, not enterprise)
- **Manual Deployment:** Ubuntu 22.04 + systemd (no Kubernetes/Docker complexity)
- **Light Rain Operation:** IP54 weatherproofing, operates in light rain (<2.5mm/hr)

---

## ⚙️ Technology Stack

**Vehicle Software (Pankaj):**
- ROS 2 Humble on Ubuntu 22.04
- C++17 & Python 3.10
- Nav2, NDT localization, OpenCV
- GMKtec Nucbox K6 (AMD Ryzen 7 7840HS, 32GB RAM)

**Fleet Management (Unno):**
- Backend: Node.js (Express) OR Python (FastAPI) OR Java (Spring Boot)
- Database: PostgreSQL 15
- Frontend: React 18 + TypeScript
- Auth: JWT (1 hour expiry)
- Real-time: WebSocket (Socket.io or equivalent)

**Hardware (Tsuchiya + Kiril):**
- 4× Swerve drive modules (Vex Robotics)
- 2× 2D LiDAR (front/rear, 270°)
- 3× Bumper switches (front/left/right)
- 2× Cameras (stereo for docking)
- 1× IMU (9-DOF)
- ESP32-S3 (eHMI firmware)
- BMS (48V battery management)

---

## 📅 Development Timeline

- **Week 1:** Notify teams, send legal package to Li San
- **Week 2-3:** Development environment setup
- **Week 4+:** Parallel independent development
- **Week 6:** Integration checkpoint (Pankaj + Unno - TVM API)
- **Week 10:** Integration checkpoint (Pankaj + Tsuchiya - Hardware handoff)
- **Week 12:** Integration checkpoint (Pankaj + Kiril - eHMI)
- **Week 15:** Full system integration
- **Week 18:** Pilot deployment

---

## 🚨 Important Notes

### Interface Change Protocol
- All interface contracts in `shared/INTERFACES/` are **FROZEN for the pilot project**
- Changes require approval from both affected teams + senior management
- See `shared/TEAM_SCOPE_DEFINITION.md` Section 3 for details

### Simplified Scope for Pilot
- **Security:** 30 requirements (not enterprise-level 152)
- **Privacy:** Pending legal review by Li San (2-3 weeks)
- **Deployment:** Manual systemd (not Kubernetes/AWS)
- **Focus:** Proof of concept, not production-ready

### Independent Development
- Teams develop independently with mock clients
- Interface contracts enable parallel work
- Integration happens at defined checkpoints

---

## 📖 Navigation Guide

### I'm a Software Developer (Pankaj's Team)
1. Read: `pankaj_vehicle_software/README.md` (complete guide)
2. Read: `shared/INTERFACES/TVM_API_SPECIFICATION.md` (API you'll call)
3. Read: `shared/INTERFACES/HARDWARE_SOFTWARE_INTERFACES.md` (ROS topics you'll use)
4. Check: `pankaj_vehicle_software/01_REQUIREMENTS/` for detailed requirements

### I'm a Fleet Management Developer (Unno's Team)
1. Read: `unno_tvm_server/README.md` (complete guide)
2. Read: `unno_tvm_server/TVM_API_IMPLEMENTATION_GUIDE_FOR_UNNO.md` (step-by-step + code)
3. Read: `shared/INTERFACES/TVM_API_SPECIFICATION.md` (API spec to implement)
4. Read: `shared/INTERFACES/TVM_DATA_MODELS.md` (database schema)

### I'm a Hardware Engineer (Tsuchiya/Kiril's Team)
1. Read: `tsuchiya_kiril_hardware/README.md` (complete guide)
2. Read: `tsuchiya_kiril_hardware/HARDWARE_REQUIREMENTS_FOR_TSUCHIYA.md` (specs + ROS examples)
3. Read: `shared/INTERFACES/HARDWARE_SOFTWARE_INTERFACES.md` (ROS topics to publish)
4. Check: `tsuchiya_kiril_hardware/01_REQUIREMENTS/` for detailed requirements

### I'm a Project Manager
1. Read: `FINAL_SUMMARY_FOR_SENIOR_APPROVAL.md` (complete project summary)
2. Read: `shared/TEAM_SCOPE_DEFINITION.md` (team boundaries and decision tree)
3. Check team READMEs for timelines and deliverables

---

## 🔗 Related Documentation

- **AI Assistant Context:** `../CLAUDE.md` (comprehensive project context)
- **Main README:** `../README.md` (bilingual documentation structure)
- **Japanese Translations:** `../multigo-ja/` (100% translated - 87/87 files)

---

**Last Updated:** December 20, 2025
**Status:** Documentation complete, awaiting senior approval for implementation phase
**Maintained By:** Project team leads

---

**For questions or clarifications, refer to team-specific README files or contact team leads directly.**
