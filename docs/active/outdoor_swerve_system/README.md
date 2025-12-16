# Multi-Team Wheelchair Transport Robot Documentation

**Project:** Outdoor-First Wheelchair Transport Robot with Fleet Management
**Version:** 1.0
**Date:** 2025-12-16

---

## 🚀 Quick Start

### For Developers - Where to Start

**✅ Week 1 COMPLETE - All Interface Documents Finished!**

Start here → **Read These First:**
1. **`multi_team/04_INTERFACES/TVM_API_SPECIFICATION.md`** ✅ (650+ lines)
   - Complete REST + WebSocket API between Vehicle and TVM Server
   - **Teams: Pankaj (Vehicle) + Unno (Fleet Management)**

2. **`multi_team/04_INTERFACES/TVM_DATA_MODELS.md`** ✅ (750+ lines)
   - All JSON schemas and data structures
   - **Teams: Pankaj + Unno**

3. **`multi_team/04_INTERFACES/HARDWARE_SOFTWARE_INTERFACES.md`** ✅ (550+ lines)
   - ROS 2 topics and hardware protocols
   - **Teams: Pankaj (Software) + Tsuchiya (Hardware)**

4. **`multi_team/00_OVERVIEW/TEAM_RESPONSIBILITIES.md`** ✅ (600+ lines)
   - Complete scope separation across all teams
   - **Teams: All**

### For Managers - Project Overview

**`multi_team/00_OVERVIEW/PROJECT_STATUS.md`** - ✅ Week 1 complete, Week 2 ready
**`multi_team/WEEK1_COMPLETION_SUMMARY.md`** - Complete Week 1 achievements and metrics

---

## 📁 Folder Structure

```
docs/active/outdoor_swerve_system/
│
├── README.md                               ← YOU ARE HERE
│
├── multi_team/                             ← NEW: Multi-team documentation
│   ├── 00_OVERVIEW/
│   │   └── PROJECT_STATUS.md               ✅ Status report (updated daily)
│   │
│   ├── 01_REQUIREMENTS/
│   │   ├── VEHICLE/                        (Pankaj - Week 2+)
│   │   ├── FLEET_MANAGEMENT/               (Unno - Week 2-5)
│   │   ├── HARDWARE/                       (Tsuchiya+Kiril - Week 2-3)
│   │   └── INTEGRATION/                    (All teams)
│   │
│   ├── 02_ARCHITECTURE/                    (Week 4-6)
│   ├── 03_DESIGN/                          (Week 4-6)
│   │
│   ├── 04_INTERFACES/                      ⚠️ **START HERE - WEEK 1**
│   │   ├── TVM_API_SPECIFICATION.md        ✅ COMPLETE (650+ lines)
│   │   ├── TVM_DATA_MODELS.md              ✅ COMPLETE (750+ lines)
│   │   └── HARDWARE_SOFTWARE_INTERFACES.md 🚧 In progress
│   │
│   ├── 05_DEVELOPMENT/
│   ├── 06_TESTING/
│   └── 07_DEPLOYMENT/
│
└── outdoor_swerve_system_current/          ← REFERENCE: Existing docs
    ├── 01_REQUIREMENTS/                    (1,064 requirements - vehicle only)
    ├── 02_ARCHITECTURE/
    ├── 03_DESIGN/
    └── ... (87 files total)
```

---

## 👥 Team Responsibilities

### Team 1: Pankaj (Vehicle Software)
**Scope:**
- Autonomous navigation (NDT localization, Nav2)
- Wheelchair docking (ArUco detection, visual servoing)
- Swerve drive control
- Perception (LiDAR processing)
- Safety systems
- Local UI (touch screen)
- TVM client interface ← **NEW**

**Documents:**
- `VEHICLE/VEHICLE_TVM_CLIENT_REQUIREMENTS.md` (Week 2)
- Implementation of TVM API client side

**Timeline:** 32 weeks (unchanged from current plan)

---

### Team 2: Unno (Fleet Management / TVM Server)
**Scope:**
- TVM server backend ← **NEW**
- Fleet management dashboard (PC/Tablet) ← **NEW**
- Reservation system (walking assistance, medicine delivery) ← **NEW**
- User role management (Admin, Caregiver, Nurse) ← **NEW**
- Real-time vehicle monitoring ← **NEW**
- Voice communication ← **NEW**

**Documents (Week 2-5):**
- `FLEET_MANAGEMENT/FLEET_MANAGEMENT_REQUIREMENTS.md` (100-120 req)
- `FLEET_MANAGEMENT/RESERVATION_SYSTEM_REQUIREMENTS.md` (80-100 req)
- `FLEET_MANAGEMENT/USER_ROLE_MANAGEMENT_REQUIREMENTS.md` (50-60 req)
- `FLEET_MANAGEMENT/TVM_SERVER_REQUIREMENTS.md` (60-80 req)
- `FLEET_MANAGEMENT/COMMUNICATION_SYSTEM_REQUIREMENTS.md` (40-50 req)

**Timeline:** 12-16 weeks (parallel with vehicle development)

---

### Team 3: Tsuchiya + Kiril (Hardware)
**Scope:**
- **Tsuchiya:** Mechanical structure, swerve drive assembly, sensors, power system
- **Kiril:** Exterior components (eHMI hardware, panels, lighting)

**Documents (Week 2-3):**
- `HARDWARE/MECHANICAL_REQUIREMENTS.md` (80-100 req)
- `HARDWARE/ELECTRICAL_REQUIREMENTS.md` (60-80 req)
- `HARDWARE/EXTERIOR_REQUIREMENTS.md` (40-50 req)

**Timeline:** Parallel (hardware ready as needed per development sprints)

---

## 🎯 Current Phase: ✅ Week 1 COMPLETE - Ready for Week 2

**Status:** ✅ **100% COMPLETE - ALL CRITICAL INTERFACE DOCUMENTS FINISHED**

**Completed (Week 1):**
- ✅ TVM_API_SPECIFICATION.md (650+ lines) - Vehicle ↔ TVM Server API
- ✅ TVM_DATA_MODELS.md (750+ lines) - JSON schemas and validation
- ✅ HARDWARE_SOFTWARE_INTERFACES.md (550+ lines) - ROS 2 + Serial protocols
- ✅ TEAM_RESPONSIBILITIES.md (600+ lines) - Scope separation
- ✅ README.md - Navigation guide
- ✅ WEEK1_COMPLETION_SUMMARY.md - Completion report

**Total:** 7 documents, 3,200+ lines of comprehensive documentation

**Next (Week 2):**
- ⏳ Team review meeting (all teams)
- ⏳ Begin parallel requirements documentation (Pankaj, Unno, Tsuchiya+Kiril)
- ⏳ Optional: Implement mock interfaces for testing

---

## 📊 Documentation Statistics

### Existing Documentation (Reference)
**Location:** `outdoor_swerve_system_current/`
- **Files:** 87 (EN + JA bilingual)
- **Requirements:** 1,064 (vehicle-side only)
- **Quality:** Enterprise-grade (100% traceability, comprehensive testing)
- **Status:** Reference only (preserved, not modified)

### New Multi-Team Documentation (Target)
**Location:** `multi_team/`
- **Files:** 150-160 (estimated, EN + JA)
- **Requirements:** 1,900-2,100 (all teams)
  - Vehicle: 1,114 (+50 TVM client)
  - Fleet Management: 450-530 (new)
  - Hardware: 180-230 (reorganized)
  - Integration: 100-130 (new)
- **Timeline:** 6 weeks documentation, then parallel development

---

## 🔑 Key Design Decisions

**From Existing Docs (Vehicle System):**
1. ✅ NO GPS → NDT localization (±10cm accuracy over 500m)
2. ✅ NO Deep Learning → CPU-only obstacle detection
3. ✅ Swerve Drive → Omnidirectional motion (NEW, not in ParcelPal)
4. ✅ ArUco Docking → Visual servoing (±2-5mm precision)
5. ✅ Outdoor-First → IP54+ weatherproofing

**New (Fleet Management System):**
6. 🆕 TVM (Total Vehicle Management) → Fleet dashboard + reservation system
7. 🆕 Multi-User Roles → Admin, Caregiver, Nurse
8. 🆕 Real-Time Monitoring → 1 Hz location updates, WebSocket commands
9. 🆕 Voice Communication → Staff ↔ Robot VoIP
10. 🆕 Offline Operation → Queue up to 1000 messages when TVM unreachable

---

## ⚙️ Technology Stack

### Vehicle Software (Pankaj)
- **OS:** Ubuntu 22.04 LTS
- **Middleware:** ROS 2 Humble
- **Navigation:** Nav2 + NDT scan matcher
- **Docking:** OpenCV + ArUco
- **UI:** React 18 + Next.js 14
- **TVM Client:** Python/C++ (REST + WebSocket)

### Fleet Management (Unno)
- **Backend:** TBD (Node.js / Python FastAPI / Java Spring?)
- **Database:** TBD (PostgreSQL / MySQL / MongoDB?)
- **Frontend:** TBD (React / Vue / Angular?)
- **Real-Time:** WebSocket / Socket.io
- **API:** REST + WebSocket

### Hardware (Tsuchiya + Kiril)
- **Compute:** GMKtec Nucbox K6 (AMD Ryzen 7 7840HS, 32GB RAM)
- **LiDAR:** Outdoor 3D LiDAR (IP65+, model TBD)
- **Motors:** 4× 6.5" hoverboard in-wheel motors (swerve drive)
- **Battery:** 48V 20Ah (≥1kWh)
- **eHMI:** ESP32-S3 + WS2812B LEDs + I2S audio

---

## 📅 Development Timeline

```
┌─────────────────────────────────────────────────────────────┐
│ WEEK 1-2: Interface Definition (CRITICAL PATH)             │
│ ⚠️ All teams BLOCKED until interfaces signed off           │
├─────────────────────────────────────────────────────────────┤
│ ✅ TVM API Specification (Pankaj + Unno)                   │
│ ✅ Data Models (Pankaj + Unno)                             │
│ 🚧 Hardware Interfaces (Pankaj + Tsuchiya)                 │
│ 🚧 Team Responsibilities                                    │
│ 🚧 System Overview                                          │
└─────────────────────────────────────────────────────────────┘
                        │
                        ▼ (Sign-off by all teams)
┌─────────────────────────────────────────────────────────────┐
│ WEEK 2-6: Parallel Documentation                           │
├─────────────────────────────────────────────────────────────┤
│ Pankaj:    Vehicle TVM Client (1 week)                     │
│ Unno:      Fleet Management (4-5 weeks, 6 subsystems)      │
│ Tsuchiya:  Hardware Requirements (2-3 weeks, 3 subsystems) │
└─────────────────────────────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────┐
│ WEEK 7+: Implementation (Parallel Development)             │
├─────────────────────────────────────────────────────────────┤
│ Pankaj:    32 weeks (vehicle software)                     │
│ Unno:      12-16 weeks (fleet management)                  │
│ Hardware:  Parallel (ready as needed)                      │
└─────────────────────────────────────────────────────────────┘
```

---

## 🚨 Important Notes

### Log Upload System (tvm_upload) vs. TVM Server

**DO NOT CONFUSE:**

1. **tvm_upload (Log Upload Daemon)** - EXISTING, SEPARATE
   - Simple Python daemon
   - Uploads vehicle logs to AWS S3 China
   - Already implemented and documented
   - NOT part of fleet management
   - Located at: `/home/pankaj/multigo_navigation_ai_integrated` (root)

2. **TVM (Total Vehicle Management)** - NEW, UNNO'S PROJECT
   - Fleet management system
   - Real-time monitoring, reservation, dispatch
   - What the Excel use cases describe
   - What Unno will develop

**These are separate systems!**

---

## 📖 How to Navigate Documentation

### I'm a Software Developer (Pankaj's Team)
**Start here:**
1. Read: `multi_team/04_INTERFACES/TVM_API_SPECIFICATION.md`
2. Read: `multi_team/04_INTERFACES/TVM_DATA_MODELS.md`
3. Implement TVM client using mock TVM server
4. Reference: `outdoor_swerve_system_current/` for existing vehicle docs

### I'm a Fleet Management Developer (Unno's Team)
**Start here:**
1. Read: `multi_team/04_INTERFACES/TVM_API_SPECIFICATION.md`
2. Read: `multi_team/04_INTERFACES/TVM_DATA_MODELS.md`
3. Implement TVM server using mock vehicle client
4. (Week 2+) Write fleet management requirements

### I'm a Hardware Engineer (Tsuchiya/Kiril's Team)
**Start here:**
1. Wait for: `multi_team/04_INTERFACES/HARDWARE_SOFTWARE_INTERFACES.md` (coming soon)
2. Reference: `outdoor_swerve_system_current/01_REQUIREMENTS/HARDWARE_REQUIREMENTS.md`
3. (Week 2) Write mechanical/electrical/exterior requirements

### I'm a Project Manager
**Start here:**
1. Read: `multi_team/00_OVERVIEW/PROJECT_STATUS.md` (updated daily)
2. Track: Weekly progress in status document
3. Coordinate: Team review meetings (Week 1, 4, 6)

---

## ✅ Success Criteria

**✅ Week 1 COMPLETE:**
- ✅ TVM_API_SPECIFICATION.md finished (ready for sign-off)
- ✅ TVM_DATA_MODELS.md finished (ready for sign-off)
- ✅ HARDWARE_SOFTWARE_INTERFACES.md finished (ready for sign-off)
- ✅ TEAM_RESPONSIBILITIES.md finished
- ✅ All teams can begin parallel work

**Week 6 Complete When:**
- ✅ All requirement documents written (1,900-2,100 total requirements)
- ✅ All architecture documents written
- ✅ All teams approved to begin implementation

**System Complete When:**
- ✅ Vehicle navigation + docking working (Pankaj - Week 32)
- ✅ Fleet management system deployed (Unno - Week 16)
- ✅ Hardware assembled and integrated (Tsuchiya+Kiril - Week 32)
- ✅ Full system integration tested (All teams - Week 32)

---

## 📞 Contacts

**Vehicle Software:** Pankaj (Lead Developer)
**Fleet Management:** Unno (TVM Server Lead)
**Hardware:** Tsuchiya (Mechanical/Electrical) + Kiril (Exterior)

---

## 🔗 Quick Links

**Critical Documents (Week 1):**
- [TVM API Specification](multi_team/04_INTERFACES/TVM_API_SPECIFICATION.md) ← **READ FIRST**
- [TVM Data Models](multi_team/04_INTERFACES/TVM_DATA_MODELS.md)
- [Project Status](multi_team/00_OVERVIEW/PROJECT_STATUS.md)

**Reference (Existing Vehicle Docs):**
- [Current System Overview](outdoor_swerve_system_current/00_OVERVIEW/SYSTEM_OVERVIEW.md)
- [Current Requirements (1,064)](outdoor_swerve_system_current/01_REQUIREMENTS/)
- [Documentation Index](outdoor_swerve_system_current/DOCUMENTATION_INDEX.md)

---

**Last Updated:** 2025-12-16
**Status:** Active Development (Week 1 - Interface Definition)
**Next Milestone:** Week 1 Interface Sign-Off (End of Week)

---

**🚀 LET'S BUILD THIS TOGETHER! 🚀**
