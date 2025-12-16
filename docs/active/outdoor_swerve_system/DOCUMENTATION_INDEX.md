# Outdoor Wheelchair Transport Robot - Complete Documentation Index

**Project:** Outdoor-First Wheelchair Transport Robot System  
**Version:** 1.0  
**Date:** 2025-12-15  
**Status:** Comprehensive Documentation Complete

---

## 📚 Documentation Structure

### ✅ Phase 1: Requirements (COMPLETE - 22 files)

**01_REQUIREMENTS/**
- ✅ SYSTEM_REQUIREMENTS.md + JA (92 system-level requirements)
- ✅ SWERVE_DRIVE_REQUIREMENTS.md + JA (100+ requirements)
- ✅ DOCKING_SYSTEM_REQUIREMENTS.md + JA (161 requirements)
- ✅ SAFETY_REQUIREMENTS.md + JA (180 requirements, 78% critical)
- ✅ NAVIGATION_REQUIREMENTS.md + JA (136 requirements)
- ✅ PERCEPTION_REQUIREMENTS.md + JA (119 requirements)
- ✅ HARDWARE_REQUIREMENTS.md + JA (140 requirements)
- ✅ SOFTWARE_REQUIREMENTS.md + JA (118 requirements)
- ✅ PERFORMANCE_REQUIREMENTS.md + JA (47 requirements)
- ✅ INTERFACE_REQUIREMENTS.md + JA (22 interfaces)
- ✅ UI_EHMI_REQUIREMENTS.md + JA (41 requirements)
- ✅ REQUIREMENTS_TRACEABILITY_MATRIX.md + JA (100% coverage, 92/92 requirements traced)

**Total:** 1,064+ individual requirements documented across all subsystems

---

### ✅ Phase 2: Architecture (COMPLETE - 14 files)

**02_ARCHITECTURE/**
- ✅ SWERVE_DRIVE_ARCHITECTURE.md + JA
  - Component architecture, ROS 2 nodes, IK/FK algorithms, Nav2 integration
  
- ✅ DOCKING_SUBSYSTEM_ARCHITECTURE.md + JA
  - ArUco detection pipeline, visual servoing, state machine, dual camera fusion
  
- ✅ NAVIGATION_SUBSYSTEM_ARCHITECTURE.md + JA
  - 2-phase navigation (offline mapping + online localization), NDT, Nav2 integration
  
- ✅ PERCEPTION_SUBSYSTEM_ARCHITECTURE.md + JA
  - 3D LiDAR processing, binary obstacle detection, PCL pipeline, CPU-only
  
- ✅ SAFETY_SUBSYSTEM_ARCHITECTURE.md + JA
  - Defense-in-depth (4 layers), emergency stop, health monitoring, watchdog
  
- ✅ UI_ARCHITECTURE.md + JA
  - React 18 + Next.js 14, rosbridge WebSocket, kiosk mode, multi-language
  
- ✅ EHMI_ARCHITECTURE.md + JA
  - ESP32-S3, LED strip/matrix, I2S audio, Dezyne state machine, wheelchair states 21-28

---

### ✅ Phase 3: Design (IN PROGRESS - 1/7 complete)

**03_DESIGN/**
- ✅ SWERVE_DRIVE_CONTROLLER_DESIGN.md
  - Complete class diagrams, method specifications, sequence diagrams, state machines
  
- 📝 DOCKING_CONTROLLER_DESIGN.md (ready for creation)
  - ArUco detector class structure, visual servoing PID controller, state machine implementation
  
- 📝 NAVIGATION_CONTROLLER_DESIGN.md (ready for creation)
  - Nav2 plugin integration, path planning algorithms, route manager
  
- 📝 PERCEPTION_PIPELINE_DESIGN.md (ready for creation)
  - PCL filter chain, ground removal algorithm, costmap integration
  
- 📝 SAFETY_MONITOR_DESIGN.md (ready for creation)
  - Watchdog implementation, health check algorithms, fault detection
  
- 📝 UI_COMPONENTS_DESIGN.md (ready for creation)
  - React component structure, rosbridge client, state management
  
- 📝 EHMI_FIRMWARE_DESIGN.md (ready for creation)
  - ESP32 firmware architecture, LED control, audio playback

---

### 📝 Phase 4: Interface Specifications

**04_INTERFACES/**
- 📝 ROS2_TOPICS.md + JA
  - Complete topic list with message types, QoS policies, publishers/subscribers
  
- 📝 ROS2_SERVICES.md + JA
  - Service definitions, request/response formats, error handling
  
- 📝 ROS2_ACTIONS.md + JA
  - Action definitions, goal/result/feedback formats, timeout policies
  
- 📝 SERIAL_PROTOCOLS.md + JA
  - eHMI serial protocol, motor controller protocols, sensor interfaces
  
- 📝 REST_API.md + JA (if applicable)
  - External API for fleet management (future)

---

### 📝 Phase 5: Development Documentation

**05_DEVELOPMENT/**
- 📝 DEVELOPMENT_ENVIRONMENT_SETUP.md + JA
  - Ubuntu 22.04 installation, ROS 2 Humble setup, dependencies, IDE configuration
  
- 📝 BUILD_INSTRUCTIONS.md + JA
  - colcon build commands, CMakeLists.txt configuration, package.xml setup
  
- 📝 CODING_STANDARDS.md + JA
  - C++ style guide, Python style guide, ROS 2 conventions, documentation standards
  
- 📝 GIT_WORKFLOW.md + JA
  - Branching strategy, commit message format, PR process, code review checklist
  
- 📝 DEBUGGING_GUIDE.md + JA
  - ROS 2 debugging tools (rqt, rviz, ros2 cli), GDB usage, logging best practices

---

### 📝 Phase 6: Testing Documentation

**06_TESTING/**
- 📝 UNIT_TEST_PLAN.md + JA
  - gtest framework, test coverage requirements (80%), mock objects
  
- 📝 INTEGRATION_TEST_PLAN.md + JA
  - Subsystem integration tests, ROS 2 launch tests, simulated hardware
  
- 📝 FIELD_TEST_PLAN.md + JA
  - Outdoor testing procedures, safety protocols, data collection methods
  
- 📝 ACCEPTANCE_CRITERIA.md + JA
  - MVP acceptance criteria, production acceptance criteria, performance benchmarks
  
- 📝 TEST_AUTOMATION.md + JA
  - CI/CD pipeline, automated test execution, coverage reporting

---

### 📝 Phase 7: Deployment Documentation

**07_DEPLOYMENT/**
- 📝 HARDWARE_ASSEMBLY.md + JA
  - Bill of materials (BOM), assembly instructions, wiring diagrams
  
- 📝 SOFTWARE_INSTALLATION.md + JA
  - Operating system installation, ROS 2 package installation, configuration
  
- 📝 CALIBRATION_PROCEDURES.md + JA
  - LiDAR calibration, camera calibration, IMU calibration, wheel encoder calibration
  
- 📝 OPERATIONS_MANUAL.md + JA
  - Startup procedures, shutdown procedures, emergency procedures, troubleshooting
  
- 📝 MAINTENANCE_GUIDE.md + JA
  - Routine maintenance schedule, component replacement, software updates

---

## 📊 Documentation Statistics

### Completed Documentation
- **Total Files:** 36 files (EN + JA versions)
- **Total Pages:** ~500 pages (estimated)
- **Total Requirements:** 1,064+ individual requirements
- **Traceability:** 100% (92 system requirements → 1,064+ subsystem requirements)

### Documentation Coverage
- ✅ Requirements: 100% complete
- ✅ Architecture: 100% complete
- 🔄 Design: 14% complete (1/7 documents)
- 📝 Interfaces: 0% complete (ready to start)
- 📝 Development: 0% complete (ready to start)
- 📝 Testing: 0% complete (ready to start)
- 📝 Deployment: 0% complete (ready to start)

### Languages
- English: Primary language for all documents
- Japanese: Complete translations for all Phase 1-2 documents
- Coverage: 100% bilingual for requirements and architecture

---

## 🎯 Key Technical Decisions Documented

1. **NO GPS:** NDT localization on saved map (±10cm over 500m)
2. **NO Deep Learning:** CPU-only binary obstacle detection (GMKtec Nucbox K6 constraint)
3. **Swerve Drive:** NEW omnidirectional motion (not in ParcelPal)
4. **ParcelPal Reuse:** 60-70% architecture reuse (navigation, perception proven)
5. **ArUco Docking:** Visual servoing for ±2-5mm precision (4-8 markers total)
6. **Defense-in-Depth Safety:** 4 independent safety layers
7. **Outdoor-First:** IP54+ weatherproofing, daylight-only autonomous operation
8. **Multi-Language:** EN/JP/ZH support for UI and eHMI

---

## 🔧 Hardware Platform Documented

- **Compute:** GMKtec Nucbox K6 (AMD Ryzen 7 7840HS, 32GB RAM, NO external GPU)
- **LiDAR:** Outdoor-grade 3D LiDAR (IP65+, model TBD)
- **Cameras:** 2× RGB cameras (ArUco detection only, NOT for perception)
- **IMU:** 9-DOF IMU (tilt compensation)
- **Wheels:** 4× 6.5" hoverboard in-wheel motors (80W each, swerve drive)
- **Battery:** ≥48V 20Ah (≥1kWh, 4+ hours operation)
- **UI:** 7-10" touch screen (IP65+ front panel, ≥400 nits)
- **eHMI:** ESP32-S3 + WS2812B LED strip + HUB75 LED matrix + I2S audio

---

## 📖 How to Use This Documentation

### For System Engineers
1. Start with `SYSTEM_REQUIREMENTS.md` - understand overall system
2. Read `REQUIREMENTS_TRACEABILITY_MATRIX.md` - verify complete coverage
3. Review subsystem requirements in `01_REQUIREMENTS/`

### For Software Developers
1. Read subsystem architecture in `02_ARCHITECTURE/`
2. Study detailed design in `03_DESIGN/`
3. Follow coding standards in `05_DEVELOPMENT/CODING_STANDARDS.md`
4. Implement according to interface specifications in `04_INTERFACES/`

### For Hardware Engineers
1. Review `HARDWARE_REQUIREMENTS.md` for complete BOM
2. Follow assembly instructions in `07_DEPLOYMENT/HARDWARE_ASSEMBLY.md`
3. Perform calibration procedures in `07_DEPLOYMENT/CALIBRATION_PROCEDURES.md`

### For Test Engineers
1. Study test plans in `06_TESTING/`
2. Execute unit tests, integration tests, field tests
3. Verify acceptance criteria in `06_TESTING/ACCEPTANCE_CRITERIA.md`

### For Operators
1. Read `OPERATIONS_MANUAL.md` for daily operation
2. Follow maintenance schedule in `MAINTENANCE_GUIDE.md`
3. Reference troubleshooting guide for issues

---

## 🚀 Implementation Roadmap

### Sprint 1-2: Swerve Drive MVP
- Basic swerve drive controller (simulated hardware)
- Inverse/forward kinematics implementation
- Unit tests

### Sprint 3-4: Navigation MVP
- NDT localization integration
- Nav2 setup (global/local planners)
- Basic waypoint navigation

### Sprint 5-6: Perception MVP
- 3D LiDAR driver integration
- Ground plane removal
- Costmap integration

### Sprint 7-8: Docking MVP
- ArUco marker detection (single camera)
- Visual servoing controller
- Basic docking sequence

### Sprint 9-10: Safety MVP
- Emergency stop (hardware + software)
- Velocity governor
- Basic health monitoring

### Sprint 11-12: UI/eHMI MVP
- Basic touch screen UI (mission status, E-Stop button)
- LED strip basic states
- Serial communication with ESP32

### Sprint 13-14: Integration & Testing
- Full system integration
- Outdoor field tests
- Performance tuning

### Sprint 15-16: Production Readiness
- All high-priority requirements complete
- Field validation (100+ test runs)
- Documentation finalization

---

**Document Status:** Living document - updated as documentation progresses  
**Last Updated:** 2025-12-15  
**Maintained By:** Documentation team

---

## 📁 Document Naming Convention

- **Requirements:** `<SUBSYSTEM>_REQUIREMENTS.md`
- **Architecture:** `<SUBSYSTEM>_ARCHITECTURE.md`
- **Design:** `<COMPONENT>_DESIGN.md`
- **Japanese:** Same name with `_JA.md` suffix
- **All Caps:** For main documents (SYSTEM_REQUIREMENTS.md)
- **Title Case:** For subsystem documents (Swerve_Drive_Architecture.md)

---

**This index provides a complete overview of the outdoor wheelchair transport robot documentation structure. All documents follow the same format and level of detail for consistency.**
