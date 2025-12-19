# Shared Documentation - Outdoor Swerve System
# 共有ドキュメント - 屋外スワーブシステム

**Purpose:** Common reference documents for all three teams
**使用目的:** 3チーム共通の参照ドキュメント

---

## 📁 Folder Contents

```
shared/
├── 00_OVERVIEW/          ← Project overview and status
│   ├── PROJECT_STATUS.md
│   ├── VEHICLE_SYSTEM_OVERVIEW.md
│   ├── TEAM_RESPONSIBILITIES.md
│   └── DETAILED_IMPLEMENTATION_PLAN_2026.md
│
└── INTERFACES/           ← Master interface specifications (contracts)
    ├── TVM_API_SPECIFICATION.md ✅ Critical
    ├── TVM_DATA_MODELS.md ✅ Critical
    ├── HARDWARE_SOFTWARE_INTERFACE.md ✅ Critical
    ├── ROS2_TOPICS.md
    └── INTERFACE_SPECIFICATIONS_COMPLETE.md
```

---

## 🎯 What This Folder Contains

### 1. Overview Documents (`00_OVERVIEW/`)

**Purpose:** High-level project information for all teams

- **PROJECT_STATUS.md** - Current project status, milestones, progress tracking
- **VEHICLE_SYSTEM_OVERVIEW.md** - System architecture overview, component interaction
- **TEAM_RESPONSIBILITIES.md** - Clear definition of each team's scope
- **DETAILED_IMPLEMENTATION_PLAN_2026.md** - Complete implementation timeline and phases

**Who uses this:**
- All three teams for project context
- Senior for status reviews
- New team members for onboarding

---

### 2. Interface Documents (`INTERFACES/`)

**Purpose:** Master copies of interface contracts between teams

**⚠️ CRITICAL - These are CONTRACTS:**
- DO NOT change without coordination between teams
- Changes require approval from all affected teams
- Version control is mandatory

#### Interface: Pankaj ↔ Unno (TVM API)

**Documents:**
- `TVM_API_SPECIFICATION.md` (1,425 lines) - Complete REST + WebSocket API
- `TVM_DATA_MODELS.md` (1,361 lines) - JSON schemas and validation rules

**What it defines:**
- REST endpoints for telemetry upload (vehicle → server)
- WebSocket commands for mission dispatch (server → vehicle)
- Authentication protocol (JWT)
- Error handling and retry logic

**Teams affected:**
- **Pankaj:** Implements TVM client (calls these APIs)
- **Unno:** Implements TVM server (provides these APIs)

**How to use:**
1. Pankaj reads to understand what to call
2. Unno reads to understand what to implement
3. Both test against these specifications
4. Integration testing verifies compliance

---

#### Interface: Pankaj ↔ Tsuchiya+Kiril (Hardware-Software)

**Documents:**
- `HARDWARE_SOFTWARE_INTERFACE.md` (1,010 lines) - Complete ROS 2 + Serial + CAN specification

**What it defines:**
- ROS 2 topics hardware publishes (sensors: LiDAR, cameras, IMU, battery, bumpers)
- ROS 2 topics software publishes (commands: velocities, steering angles)
- CAN bus protocol for motor controllers
- Serial protocol for eHMI (ESP32 communication)

**Teams affected:**
- **Pankaj:** Subscribes to sensor topics, publishes command topics
- **Tsuchiya+Kiril:** Publishes sensor topics, subscribes to command topics

**How to use:**
1. Tsuchiya sets up hardware to publish correct topics
2. Pankaj writes software to consume/produce topics
3. Both verify message formats match specification
4. Integration testing verifies data flow

---

#### Interface: Internal ROS 2 (Pankaj only)

**Documents:**
- `ROS2_TOPICS.md` - Internal ROS 2 node communication within vehicle software

**What it defines:**
- Topics between navigation, docking, perception nodes
- Message types and frequencies
- Naming conventions

**Teams affected:**
- **Pankaj only** (internal to vehicle software)

**How to use:**
- Reference when designing ROS 2 nodes
- Ensure consistent topic naming
- Verify message types match

---

## 🔒 Interface Change Protocol

**If you need to change an interface:**

1. **Identify affected teams:**
   - TVM API changes: Pankaj + Unno
   - Hardware-Software changes: Pankaj + Tsuchiya+Kiril

2. **Propose change:**
   - Document the change (old → new)
   - Explain why it's needed
   - Consider backward compatibility

3. **Coordinate approval:**
   - Both teams must agree
   - Senior approval for major changes

4. **Update master document:**
   - Update in `shared/INTERFACES/` (master copy)
   - Use semantic versioning if needed (v1 → v2)
   - Document migration path

5. **Notify all teams:**
   - Post change summary
   - Update team-specific copies
   - Schedule integration testing

**Example:**
```
Change: Add new field to telemetry JSON
Affected: TVM_DATA_MODELS.md
Teams: Pankaj (vehicle sends) + Unno (server receives)
Process:
1. Pankaj proposes: Add "tire_pressure" field to telemetry
2. Unno reviews and approves
3. Update shared/INTERFACES/TVM_DATA_MODELS.md
4. Pankaj updates vehicle client code
5. Unno updates server database schema
6. Integration test with new field
```

---

## 📋 How Teams Use This Folder

### Pankaj (Vehicle Software)

**Read these first:**
1. `INTERFACES/TVM_API_SPECIFICATION.md` - How to call Unno's server
2. `INTERFACES/HARDWARE_SOFTWARE_INTERFACE.md` - How to communicate with hardware
3. `00_OVERVIEW/TEAM_RESPONSIBILITIES.md` - Confirm your scope

**Reference during development:**
- Check interface specs when implementing API calls
- Verify topic names match specification
- Check project status for timeline

**Copy to your folder:**
- All interface documents are referenced in `pankaj_vehicle_software/04_INTERFACES/`

---

### Unno (TVM Server)

**Read these first:**
1. `INTERFACES/TVM_API_SPECIFICATION.md` - What APIs to implement
2. `INTERFACES/TVM_DATA_MODELS.md` - What JSON schemas to validate
3. `00_OVERVIEW/TEAM_RESPONSIBILITIES.md` - Confirm your scope

**Reference during development:**
- Implement REST endpoints exactly as specified
- Implement WebSocket commands exactly as specified
- Use JSON schemas for validation
- Check project status for integration testing timeline

**Copy to your folder:**
- TVM API and Data Models are referenced in `unno_tvm_server/04_INTERFACES/`

---

### Tsuchiya + Kiril (Hardware)

**Read these first:**
1. `INTERFACES/HARDWARE_SOFTWARE_INTERFACE.md` - What ROS topics to publish/subscribe
2. `00_OVERVIEW/TEAM_RESPONSIBILITIES.md` - Confirm your scope

**Reference during development:**
- Verify sensor topics match specification
- Check message frequencies (LiDAR 10Hz, IMU 50Hz, etc.)
- Verify serial protocol for eHMI
- Check project status for integration testing timeline

**Copy to your folder:**
- Hardware-Software interface is referenced in `tsuchiya_kiril_hardware/04_INTERFACES/`

---

## ⚠️ Important Rules

### 1. Interface Documents are FROZEN

**DO NOT change these without coordination:**
- ROS topic names
- Message types
- API endpoints
- JSON schemas
- Serial protocols

**Why:** Changes break integration between teams

---

### 2. Master Copies are in `shared/INTERFACES/`

**Team folders have REFERENCES, not copies:**
- `pankaj_vehicle_software/04_INTERFACES/` → references shared docs
- `unno_tvm_server/04_INTERFACES/` → references shared docs
- `tsuchiya_kiril_hardware/04_INTERFACES/` → references shared docs

**If you find a discrepancy:**
- The `shared/INTERFACES/` version is the source of truth
- Report discrepancy immediately
- Update team folder to match

---

### 3. Overview Documents for Context Only

**Files in `00_OVERVIEW/` are informational:**
- Project status updates
- Timeline references
- Team responsibility definitions

**Not binding specifications:**
- Use interface documents for technical contracts
- Use overview documents for project context

---

## 📞 Contacts

**Project Lead:** Pankaj (Vehicle Software)
**TVM Server Lead:** Unno
**Hardware Leads:** Tsuchiya (Mechanical/Electrical), Kiril (Exterior/eHMI)
**Legal Review:** Li San (Privacy requirements)
**Senior:** [Name] (Final approvals)

---

## 🔄 Document Update Process

### Updating Overview Documents

**Who can update:**
- Pankaj (project status, timeline updates)
- Any team lead (with notification)

**Process:**
1. Make changes to `shared/00_OVERVIEW/`
2. Notify other teams
3. No approval needed (informational only)

**Example:**
- Update PROJECT_STATUS.md weekly with progress

---

### Updating Interface Documents

**Who can update:**
- Only after coordination between affected teams

**Process:**
1. Propose change (with justification)
2. Get approval from all affected teams
3. Get senior approval if major change
4. Update `shared/INTERFACES/` (master)
5. Notify all teams
6. Update team-specific references
7. Schedule integration testing

**Example:**
- Adding new ROS topic requires Pankaj + Tsuchiya approval

---

## ✅ Quick Reference

### Which Interface Do I Need?

**If you're Pankaj:**
- Need to call Unno's server? → `TVM_API_SPECIFICATION.md`
- Need to read sensors? → `HARDWARE_SOFTWARE_INTERFACE.md`
- Need internal ROS topics? → `ROS2_TOPICS.md`

**If you're Unno:**
- Need to implement server APIs? → `TVM_API_SPECIFICATION.md`
- Need JSON schemas? → `TVM_DATA_MODELS.md`

**If you're Tsuchiya or Kiril:**
- Need to publish sensor data? → `HARDWARE_SOFTWARE_INTERFACE.md`
- Need to receive motor commands? → `HARDWARE_SOFTWARE_INTERFACE.md`

---

## 📊 Interface Status

| Interface Document | Status | Lines | Teams | Last Updated |
|-------------------|--------|-------|-------|--------------|
| TVM_API_SPECIFICATION.md | ✅ Production-ready | 1,425 | Pankaj ↔ Unno | Dec 19, 2025 |
| TVM_DATA_MODELS.md | ✅ Production-ready | 1,361 | Pankaj ↔ Unno | Dec 19, 2025 |
| HARDWARE_SOFTWARE_INTERFACE.md | ✅ Production-ready | 1,010 | Pankaj ↔ Tsuchiya+Kiril | Dec 19, 2025 |
| ROS2_TOPICS.md | ✅ Complete | ~500 | Pankaj (internal) | Dec 19, 2025 |

**All interfaces are frozen and ready for implementation.**

---

## 🚀 Getting Started

### For New Team Members:

1. **Read project overview:**
   - `00_OVERVIEW/VEHICLE_SYSTEM_OVERVIEW.md` - Understand the system
   - `00_OVERVIEW/TEAM_RESPONSIBILITIES.md` - Understand your team's scope

2. **Read your team's interface:**
   - Pankaj: Read both TVM API and Hardware-Software interfaces
   - Unno: Read TVM API interface
   - Tsuchiya+Kiril: Read Hardware-Software interface

3. **Go to your team folder:**
   - `pankaj_vehicle_software/README.md` - Start here if you're on software team
   - `unno_tvm_server/README.md` - Start here if you're on server team
   - `tsuchiya_kiril_hardware/README.md` - Start here if you're on hardware team

---

## 📝 Version History

**Version 1.0** (December 19, 2025)
- Initial shared folder structure
- Three interface documents finalized
- Overview documents consolidated

---

**Document Status:** ✅ Complete
**Maintained By:** Pankaj (Project Lead)
**Last Updated:** December 19, 2025

---

**Questions?**
- Interface changes: Coordinate with affected teams
- Project status: Check `00_OVERVIEW/PROJECT_STATUS.md`
- Scope questions: Check `00_OVERVIEW/TEAM_RESPONSIBILITIES.md`

---

**END OF SHARED DOCUMENTATION README**
