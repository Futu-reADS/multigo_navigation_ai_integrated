# CLAUDE.md - AI Assistant Context Guide

**Purpose:** This file helps Claude (AI assistant) quickly understand the project context in new sessions without re-reading all files.

**Last Updated:** December 19, 2025
**Project Status:** Documentation reorganization complete, awaiting senior approval

---

## 🎯 Project Overview

**Project Name:** Outdoor Wheelchair Transport Robot
**Type:** Autonomous elderly transportation for care facilities
**Scope:** Pilot project (1-2 vehicles, single facility)
**Timeline:** 18-20 weeks development
**Key Technology:** ROS 2 Humble, swerve drive, NDT localization, ArUco docking

**What This Robot Does:**
- Autonomously transports elderly residents in wheelchairs
- Outdoor operation on facility grounds
- Precision docking using ArUco visual markers (±2-5mm)
- Fleet management via TVM (Total Vehicle Management) server
- Safe operation with E-stop, bumpers, obstacle avoidance

---

## 📊 Current Status (December 19, 2025)

**Phase:** Documentation reorganization COMPLETE ✅
**Next:** Awaiting senior approval to start implementation

**What Was Accomplished (8 Phases):**
1. ✅ Assessment - Analyzed 127 documents, categorized by team
2. ✅ Folder Structure - Created 3-team structure + shared folder
3. ✅ Document Reorganization - Moved 113 files, deleted 41 unnecessary
4. ✅ Simplification - Reduced security (152→30 reqs), deployment (1,576→639 lines)
5. ✅ Implementation Guides - Created step-by-step guides for Unno & Tsuchiya
6. ✅ Legal Coordination - Prepared privacy review package for Li San
7. ✅ Team Scope Definition - Clear boundaries and decision tree
8. ✅ Final Summary - Comprehensive approval package for senior

**Deleted:**
- `multi_team/` folder (old structure, 21 files)
- `shared/00_OVERVIEW/` folder (redundant, 11 files)
- Root work artifacts (planning docs, 9 files)

---

## 🌐 Bilingual Documentation (CRITICAL)

**Structure:**
```
docs/
├── CLAUDE.md                    ← This file (at docs root)
├── multigo/                     ← English (source of truth)
│   ├── pankaj_vehicle_software/
│   ├── unno_tvm_server/
│   ├── tsuchiya_kiril_hardware/
│   └── shared/
└── multigo-ja/                  ← Japanese (translations only)
    ├── pankaj_vehicle_software/
    ├── unno_tvm_server/
    ├── tsuchiya_kiril_hardware/
    └── shared/
```

### **CRITICAL RULES - You MUST Follow:**

1. **ALWAYS use English docs for code work:**
   - ✅ Read requirements from: `docs/multigo/`
   - ✅ Read architecture from: `docs/multigo/`
   - ✅ Read interfaces from: `docs/multigo/`
   - ✅ Reference ANY file from: `docs/multigo/`
   - ❌ **NEVER use `docs/multigo-ja/` for code work or requirements**

2. **Auto-sync translations (MANDATORY):**
   - When you **create** a file in `multigo/`, also create it in `multigo-ja/` (Japanese translation)
   - When you **update** a file in `multigo/`, also update it in `multigo-ja/` (translate changes)
   - When you **delete** a file in `multigo/`, also delete it in `multigo-ja/`
   - Keep same file path: `multigo/path/file.md` → `multigo-ja/path/file.md`

3. **Translation approach:**
   - English is ALWAYS the source of truth
   - Japanese is for team reference only (Unno, Tsuchiya, Kiril)
   - If you can't translate immediately, add placeholder: `[要翻訳 - Translation needed]`
   - Translate technical terms consistently (see Translation Glossary below)

4. **What to translate:**
   - ✅ Requirements, architecture, design docs
   - ✅ READMEs, guides, summaries
   - ✅ Safety, testing, deployment docs
   - ✅ Comments and explanations
   - ❌ Code examples, JSON/YAML configs, API endpoints
   - ❌ ROS topic names, file paths, command-line examples

### **Translation Glossary (Keep Consistent):**

| English | Japanese | Notes |
|---------|----------|-------|
| wheelchair | 車椅子 | |
| docking | ドッキング | |
| swerve drive | スワーブドライブ | Keep technical |
| ArUco marker | ArUcoマーカー | Brand name |
| E-stop | 緊急停止 | |
| obstacle avoidance | 障害物回避 | |
| NDT localization | NDT自己位置推定 | |
| TVM server | TVMサーバー | Acronym kept |
| pilot project | パイロットプロジェクト | |
| requirements | 要求仕様 | |
| architecture | アーキテクチャ | |

### **Example Workflow:**

**Scenario:** Update safety requirements

```bash
# 1. User asks: "Add collision detection requirement"

# 2. You update English (source of truth)
Edit: docs/multigo/pankaj_vehicle_software/01_REQUIREMENTS/SAFETY_REQUIREMENTS.md
Add: "SAFE-COL-001: System SHALL detect collisions within 100ms"

# 3. You also update Japanese (auto-sync)
Edit: docs/multigo-ja/pankaj_vehicle_software/01_REQUIREMENTS/SAFETY_REQUIREMENTS.md
Add: "SAFE-COL-001: システムは100ms以内に衝突を検出すること"

# 4. Both files stay in sync ✅
```

### **Translation Status:**

- **Current:** 87/87 files translated (100%) ✅
- **Status:** All documentation fully translated to Japanese
- **Last updated:** December 20, 2025

---

## 🏗️ Documentation Structure

```
multigo/
│
├── 📄 FINAL_SUMMARY_FOR_SENIOR_APPROVAL.md    [READ FIRST - Complete summary]
├── 📄 PRIVACY_LEGAL_REVIEW_PACKAGE_FOR_LI_SAN.md [For legal review]
├── 📄 CLAUDE.md                                [This file]
├── 📄 README.md                                [Project README]
│
├── 📁 pankaj_vehicle_software/ (69 files)      [Team 1: Vehicle Software]
│   ├── README.md                               [Complete guide - 507 lines]
│   ├── 01_REQUIREMENTS/                        [Functional, security, etc.]
│   ├── 02_ARCHITECTURE/                        [ROS 2 architecture]
│   ├── 03_DESIGN/                              [Navigation, docking, control]
│   ├── 04_INTERFACES/                          [TVM client, eHMI]
│   ├── 05_DEVELOPMENT/                         [Implementation details]
│   └── 06_TESTING/                             [Test strategy]
│
├── 📁 unno_tvm_server/ (19 files)              [Team 2: TVM Server]
│   ├── README.md                               [Complete guide - 410 lines]
│   ├── TVM_API_IMPLEMENTATION_GUIDE_FOR_UNNO.md [Step-by-step + code - 742 lines]
│   ├── 01_REQUIREMENTS/                        [Server requirements]
│   ├── 02_ARCHITECTURE/                        [Backend architecture]
│   ├── 03_DESIGN/                              [Database, API design]
│   ├── 04_INTERFACES/                          [API specs]
│   └── 05_DEVELOPMENT/                         [Implementation details]
│
├── 📁 tsuchiya_kiril_hardware/ (15 files)      [Team 3: Hardware]
│   ├── README.md                               [Complete guide - 507 lines]
│   ├── HARDWARE_REQUIREMENTS_FOR_TSUCHIYA.md   [Complete specs + ROS - 853 lines]
│   ├── 01_REQUIREMENTS/                        [Hardware requirements]
│   ├── 02_ARCHITECTURE/                        [Platform architecture]
│   ├── 03_DESIGN/                              [Mechanical, electrical]
│   ├── 04_INTERFACES/                          [ROS topics, CAN bus]
│   └── 05_DEVELOPMENT/                         [Assembly, testing]
│
└── 📁 shared/ (7 files)                        [Cross-team resources]
    ├── README.md                               [How to use shared docs - 380 lines]
    ├── TEAM_SCOPE_DEFINITION.md                [Who does what - 470+ lines]
    └── INTERFACES/                             [5 FROZEN contracts - production ready]
        ├── TVM_API_SPECIFICATION.md            [19 REST + 4 WebSocket commands]
        ├── TVM_DATA_MODELS.md                  [Database schema, data structures]
        ├── HARDWARE_SOFTWARE_INTERFACES.md     [ROS topics, CAN bus, serial]
        ├── ROS2_TOPICS.md                      [Complete topic list]
        └── INTERFACE_SPECIFICATIONS_COMPLETE.md [Summary of all interfaces]
```

**Total:** 113 markdown files (clean, organized, no duplicates)

---

## 👥 Three-Team Structure

### Team 1: Pankaj (Vehicle Software)
**Technology:** ROS 2 Humble, Ubuntu 22.04, C++/Python
**Responsibility:** Everything that runs ON the vehicle
- ✅ ROS 2 nodes (navigation, docking, control, perception, safety)
- ✅ TVM client (calls Unno's APIs)
- ✅ Local UI (React on vehicle)
- ✅ eHMI controller (serial commands to Kiril's ESP32)
- ❌ Does NOT: Hardware platform, TVM server, database
**Timeline:** 18 weeks
**Key Documents:**
- `pankaj_vehicle_software/README.md`
- `shared/INTERFACES/` (contracts to follow)

### Team 2: Unno (TVM Server)
**Technology:** Backend (Node.js/Python/Java), PostgreSQL, React dashboard
**Responsibility:** Fleet management server
- ✅ Backend (REST API + WebSocket)
- ✅ Database (PostgreSQL)
- ✅ Fleet dashboard (web UI)
- ✅ User authentication (JWT)
- ✅ Mission dispatch, telemetry ingestion
- ❌ Does NOT: Vehicle software, ROS 2, hardware
**Timeline:** 20 weeks (15W MVP + 5W advanced)
**Key Documents:**
- `unno_tvm_server/README.md`
- `unno_tvm_server/TVM_API_IMPLEMENTATION_GUIDE_FOR_UNNO.md` (code examples!)
- `shared/INTERFACES/TVM_API_SPECIFICATION.md` (must implement)

### Team 3: Tsuchiya + Kiril (Hardware)
**Tsuchiya:** Mechanical, electrical, sensors, compute
**Kiril:** Exterior design, eHMI (ESP32 firmware)
**Responsibility:** Physical robot platform
- ✅ Chassis, motors, power, sensors
- ✅ Compute installation (GMKtec Nucbox K6)
- ✅ ROS 2 sensor drivers (LiDAR, IMU, cameras, bumpers)
- ✅ CAN bus wiring
- ✅ ESP32-S3 firmware (LED/audio control)
- ❌ Does NOT: Vehicle control software, navigation, TVM server
**Timeline:** 16 weeks Tsuchiya, 10 weeks Kiril
**Key Documents:**
- `tsuchiya_kiril_hardware/README.md`
- `tsuchiya_kiril_hardware/HARDWARE_REQUIREMENTS_FOR_TSUCHIYA.md` (ROS examples!)
- `shared/INTERFACES/HARDWARE_SOFTWARE_INTERFACES.md` (must follow)

---

## 🔒 Critical Interface Contracts (FROZEN)

**These 3 documents are production-ready and FROZEN for pilot:**

### 1. TVM_API_SPECIFICATION.md (1,425 lines)
- 19 REST endpoints (login, telemetry, missions, commands)
- 4 WebSocket commands (DISPATCH_MISSION, PAUSE, RESUME, CANCEL)
- JWT authentication (1 hour expiry)
- Contract between: Pankaj ↔ Unno

### 2. HARDWARE_SOFTWARE_INTERFACES.md (1,010 lines)
- 20+ ROS 2 topics (LiDAR, IMU, cameras, motors, safety)
- CAN bus protocol (500 kbps, 8 motors + BMS)
- Serial protocol (ESP32 eHMI: LED patterns, audio)
- Contract between: Pankaj ↔ Tsuchiya+Kiril

### 3. TVM_DATA_MODELS.md (1,361 lines)
- PostgreSQL schema (8 tables: vehicles, missions, telemetry, etc.)
- WebSocket message formats
- Data validation rules
- Contract between: Pankaj ↔ Unno (shared data understanding)

**Interface Change Protocol:**
- Both teams must approve changes
- Senior approval required for major changes
- Document in `shared/TEAM_SCOPE_DEFINITION.md` Section 3

---

## 🎯 Key Decisions & Context

### Why Three Teams? (Not Two)
**Senior's feedback:** "Separate TVM server to Unno's team"
- Originally: 2 teams (Pankaj+Hardware vs Unno)
- Changed to: 3 teams (Pankaj, Unno, Tsuchiya+Kiril)
- Reason: Clear separation of concerns, parallel development

### Why Simplification? (Enterprise → Pilot)
**Senior's feedback:** "Requirements are too high for pilot project"
**What was simplified:**
- Security: 152 requirements → 30 requirements (80% reduction)
  - Removed: MFA, SSO, penetration testing, disaster recovery
  - Kept: JWT, HTTPS, E-stop, basic security
- Deployment: 1,576 lines → 639 lines (60% reduction)
  - Removed: Multi-region AWS, Kubernetes, auto-scaling, DR
  - Kept: Ubuntu 22.04, systemd, manual SSH deployment
- Privacy: 1,124 lines → ~300 lines (pending Li San's legal review)
  - Question: Does HIPAA apply? (transports residents, doesn't access EHR)
  - Question: Consent mechanism? (facility vs individual)

**Development time saved:** 10-15 weeks
**Cost saved:** $20,000-45,000 (enterprise infrastructure avoided)

### Why Delete multi_team/?
- All files reorganized into 3-team structure
- Keeping both would cause confusion
- Old timelines conflicted with new simplified timelines

### Why Delete shared/00_OVERVIEW/?
- Redundant with team READMEs
- Conflicting timelines (24 weeks vs 18/20/16 weeks)
- Outdated status information

---

## 📖 What to Read First (New Session Checklist)

**If starting a new session, read in this order:**

1. **CLAUDE.md** (this file) - Quick context ✅
2. **FINAL_SUMMARY_FOR_SENIOR_APPROVAL.md** - Complete summary of all work
3. **shared/TEAM_SCOPE_DEFINITION.md** - Who does what, decision tree
4. **Team-specific README** - Depending on the question:
   - `pankaj_vehicle_software/README.md` - For vehicle software questions
   - `unno_tvm_server/README.md` - For TVM server questions
   - `tsuchiya_kiril_hardware/README.md` - For hardware questions

**Don't read everything!** Use these guides to understand context quickly.

---

## 🔍 Common Questions & Answers

### Q: "What is this project?"
**A:** Autonomous outdoor wheelchair transport robot for elderly care facilities (pilot: 1-2 vehicles, single facility)

### Q: "What's the current status?"
**A:** Documentation reorganization complete (8 phases), awaiting senior approval for implementation to begin

### Q: "Who does navigation?"
**A:** Pankaj (vehicle software team)

### Q: "Who does the TVM server?"
**A:** Unno (TVM server team)

### Q: "Who assembles the robot?"
**A:** Tsuchiya (hardware team) + Kiril (exterior/eHMI)

### Q: "Can I change the TVM API?"
**A:** NO - It's frozen. Must get approval from both Pankaj & Unno + senior for major changes. See `shared/TEAM_SCOPE_DEFINITION.md` Section 3

### Q: "Where are the requirements?"
**A:** Each team has their own `01_REQUIREMENTS/` folder. Security simplified to 30 requirements (pilot-level)

### Q: "What about privacy/legal compliance?"
**A:** Pending legal review by Li San. See `PRIVACY_LEGAL_REVIEW_PACKAGE_FOR_LI_SAN.md` (2-3 week timeline)

### Q: "Why is HIPAA mentioned?"
**A:** It might not apply! That's what Li San needs to confirm. Robot transports residents but doesn't access electronic health records.

### Q: "What happened to the multi_team folder?"
**A:** Deleted (reorganized into 3-team structure). Don't look for it.

### Q: "Why are there Japanese files?"
**A:** Some old documents had JA versions. Most were deleted during cleanup.

---

## 🚀 Next Steps After Senior Approval

### Immediate (Week 1):
1. **Notify teams:**
   - Unno: Review `unno_tvm_server/` folder + implementation guide
   - Tsuchiya: Review `tsuchiya_kiril_hardware/` folder + requirements
   - Kiril: Review eHMI sections
2. **Send to Li San:** Privacy review package (2-3 week timeline)
3. **Team acknowledgments:** Each team confirms understanding and timeline

### Week 2-3:
1. **Development environment setup** per team READMEs
2. **Independent development begins:**
   - Pankaj: ROS 2 simulation, mock TVM client
   - Unno: Backend setup, mock vehicle client (provided in guide)
   - Tsuchiya: Component procurement, CAD design

### Week 4+:
1. **Parallel development** with weekly checkpoints
2. **Integration tests:**
   - Week 6: Pankaj + Unno (TVM API)
   - Week 10: Pankaj + Tsuchiya (hardware handoff)
   - Week 12: Pankaj + Kiril (eHMI)
3. **Full system integration:** Week 15
4. **Pilot deployment:** Week 18

---

## 💡 Design Principles & Constraints

### Technical Constraints:
- **No GPS** - Indoor/outdoor facility (GPS unreliable), use NDT localization
- **CPU-only** - GMKtec Nucbox K6 (no discrete GPU), optimize perception
- **Light rain OK** - IP54 weatherproofing, operates in light rain (<2.5mm/hr), rejects heavy rain (>7.5mm/hr)
- **Outdoor** - Weather-resistant, uneven terrain handling
- **Safety-critical** - E-stop <100ms (hardware) + <500ms (software)
- **Pilot budget** - No enterprise infrastructure (AWS, Kubernetes, etc.)

### Key Technical Choices:
- **ROS 2 Humble** (Ubuntu 22.04 LTS) - Industry standard robotics
- **Swerve Drive** - Omnidirectional motion (4 independent modules)
- **NDT Localization** - LiDAR scan matching (±10cm accuracy)
- **ArUco Docking** - Dual-camera visual servoing (±2-5mm precision)
- **PostgreSQL** - TVM database (reliable, open-source)
- **JWT Authentication** - Vehicle-TVM auth (1 hour token expiry)

### Development Approach:
- **Independent development** - Teams work in parallel with mock clients
- **Interface-driven** - Frozen contracts enable parallel work
- **Pilot-appropriate** - Simplified from enterprise to pilot scope
- **Incremental integration** - Individual components → integration → system

---

## 📝 Important File Paths (Quick Reference)

**For Senior:**
- `./FINAL_SUMMARY_FOR_SENIOR_APPROVAL.md` - Approval package

**For Li San (Legal):**
- `./PRIVACY_LEGAL_REVIEW_PACKAGE_FOR_LI_SAN.md` - Legal review request
- `pankaj_vehicle_software/01_REQUIREMENTS/PRIVACY_REQUIREMENTS.md` - Full requirements (1,124 lines)

**For Pankaj:**
- `pankaj_vehicle_software/README.md` - Complete guide
- `pankaj_vehicle_software/01_REQUIREMENTS/SECURITY_REQUIREMENTS.md` - 30 requirements
- `pankaj_vehicle_software/02_ARCHITECTURE/DEPLOYMENT_ARCHITECTURE.md` - Vehicle deployment
- `shared/INTERFACES/TVM_API_SPECIFICATION.md` - API to implement
- `shared/INTERFACES/HARDWARE_SOFTWARE_INTERFACES.md` - ROS topics to use

**For Unno:**
- `unno_tvm_server/README.md` - Complete guide
- `unno_tvm_server/TVM_API_IMPLEMENTATION_GUIDE_FOR_UNNO.md` - Step-by-step + code
- `shared/INTERFACES/TVM_API_SPECIFICATION.md` - API spec to implement
- `shared/INTERFACES/TVM_DATA_MODELS.md` - Database schema

**For Tsuchiya:**
- `tsuchiya_kiril_hardware/README.md` - Complete guide
- `tsuchiya_kiril_hardware/HARDWARE_REQUIREMENTS_FOR_TSUCHIYA.md` - Specs + ROS examples
- `shared/INTERFACES/HARDWARE_SOFTWARE_INTERFACES.md` - ROS topics to publish
- `shared/INTERFACES/ROS2_TOPICS.md` - Complete topic list

**For Kiril:**
- `tsuchiya_kiril_hardware/README.md` - Section on eHMI
- `tsuchiya_kiril_hardware/HARDWARE_REQUIREMENTS_FOR_TSUCHIYA.md` - ESP32 protocol (Section 5)
- `shared/INTERFACES/HARDWARE_SOFTWARE_INTERFACES.md` - Serial protocol spec

**For Everyone:**
- `shared/TEAM_SCOPE_DEFINITION.md` - Who does what (decision tree)
- `shared/README.md` - How to use shared folder

---

## ⚠️ Things NOT to Do

1. **Don't modify frozen interface contracts** without approval from both affected teams + senior
2. **Don't add enterprise features** (MFA, SSO, Kubernetes, DR) - This is a PILOT project
3. **Don't re-organize documentation** - Structure is final and approved
4. **Don't look for multi_team/ folder** - It was deleted
5. **Don't look for shared/00_OVERVIEW/** - It was deleted
6. **Don't assume HIPAA compliance needed** - Waiting for Li San's legal review
7. **Don't create new teams** - Three teams is final (Pankaj, Unno, Tsuchiya+Kiril)

---

## 🔄 If User Asks to "Review Previous Work"

**What was done (summary):**
- 8-phase documentation reorganization (Dec 16-19, 2025)
- 127 files → 113 files (41 deleted as unnecessary/redundant)
- Simplified from enterprise to pilot scope
- Created 3-team structure with clear boundaries
- Created implementation guides with code examples
- Coordinated legal review for privacy requirements
- Created final approval package for senior

**Key artifacts:**
- `FINAL_SUMMARY_FOR_SENIOR_APPROVAL.md` - Complete summary (1,089 lines)
- `PRIVACY_LEGAL_REVIEW_PACKAGE_FOR_LI_SAN.md` - Legal review (560 lines)
- Team READMEs (507, 410, 507 lines)
- Implementation guides (742, 853 lines)
- Team scope definition (470+ lines)

**Status:** Complete and awaiting senior approval

---

## 🎓 Technical Glossary

- **TVM** - Total Vehicle Management (fleet management server)
- **NDT** - Normal Distributions Transform (LiDAR localization algorithm)
- **ArUco** - Fiducial marker system for visual servoing (docking)
- **Swerve Drive** - Omnidirectional drive with 4 independent steering modules
- **eHMI** - External Human-Machine Interface (LED/audio for communication)
- **CAN Bus** - Controller Area Network (motor control communication)
- **JWT** - JSON Web Token (authentication mechanism)
- **E-stop** - Emergency stop (hardware + software safety mechanism)
- **GMKtec Nucbox K6** - Compute platform (AMD Ryzen 7 7840HS, 32GB RAM)
- **Pilot Project** - Small-scale test (1-2 vehicles) before production

---

## 📞 Team Contacts & Roles

**Pankaj** - Vehicle Software Team Lead
- Responsibility: ROS 2 software, navigation, docking, TVM client
- Timeline: 18 weeks

**Unno** - TVM Server Team Lead
- Responsibility: Backend, database, fleet dashboard
- Timeline: 20 weeks

**Tsuchiya** - Hardware Team Lead
- Responsibility: Mechanical, electrical, sensors, integration
- Timeline: 16 weeks

**Kiril** - Exterior/eHMI Developer
- Responsibility: Exterior design, ESP32 firmware
- Timeline: 10 weeks

**Li San** - Legal Counsel
- Responsibility: Privacy requirements review
- Timeline: 2-3 weeks (pending)

**Senior** - Project Authority
- Responsibility: Final approvals, scope decisions
- Status: Awaiting approval of reorganization

---

## 🏁 Success Criteria (Pilot Completion)

**Vehicle (Pankaj):**
- ✅ Autonomous navigation between waypoints
- ✅ Docking accuracy ±5mm (ArUco markers)
- ✅ TVM integration (telemetry, missions, commands)
- ✅ All safety features (E-stop, bumpers, watchdog)
- ✅ Local UI functional

**TVM Server (Unno):**
- ✅ All 19 REST endpoints implemented
- ✅ 4 WebSocket commands working
- ✅ Fleet dashboard with real-time monitoring
- ✅ User authentication/authorization
- ✅ Database handles 10+ vehicles

**Hardware (Tsuchiya+Kiril):**
- ✅ Vehicle moves in all directions (swerve working)
- ✅ All sensors publishing to ROS topics
- ✅ E-stop response <100ms (hardware)
- ✅ Battery 4+ hours operation
- ✅ eHMI displays correct patterns

**Legal (Li San):**
- ✅ Privacy requirements approved for pilot
- ✅ Consent mechanism defined
- ✅ Data retention policy approved

---

## 📚 Version History

**v1.0 (Dec 19, 2025)** - Initial creation after 8-phase reorganization complete
- Documentation structure finalized
- Three-team separation complete
- Enterprise features removed
- Implementation guides created
- Legal review coordinated
- Final summary prepared

---

## 💬 How to Use This File

**For Claude (AI Assistant):**
1. Read this file at the start of EVERY new session
2. Use as quick reference - don't re-read all 113 files
3. Check "Common Questions & Answers" section first
4. Refer to team READMEs for detailed questions
5. Check FINAL_SUMMARY for complete context

**For Humans:**
1. This is Claude's context file - not user documentation
2. For project overview, see `README.md`
3. For team guides, see team-specific `README.md` files
4. For approval package, see `FINAL_SUMMARY_FOR_SENIOR_APPROVAL.md`

---

**Last Updated:** December 19, 2025
**Next Update:** After senior approval and implementation phase begins
**Maintained By:** Pankaj + Claude (AI assistant)

---

**NOTE:** This file should be updated whenever major project changes occur (scope changes, team changes, architectural decisions, etc.)
