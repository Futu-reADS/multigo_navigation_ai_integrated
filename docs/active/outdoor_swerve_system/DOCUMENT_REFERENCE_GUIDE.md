# Document Reference Guide for Stakeholders

**Document ID:** DOC-REF-GUIDE-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Comprehensive Reference

---

## Purpose

This guide helps different stakeholders navigate the complete documentation set for the outdoor wheelchair transport robot system. It maps each role to the specific documents they need for their responsibilities.

---

## Quick Role-to-Document Mapping

| Role | Primary Documents | Supporting Documents | Language |
|------|------------------|---------------------|----------|
| **Project Manager** | System Requirements, Traceability Matrix | All Phase 1, Architecture Overview | EN/JA |
| **System Architect** | All Phase 2 Architecture | Requirements, Interface Specs | EN/JA |
| **Software Developer (C++)** | Phase 3 Design, Development Guide | Architecture, Interface Specs | EN/JA |
| **Software Developer (Python)** | Development Guide, Testing Guide | Design, Interface Specs | EN/JA |
| **ROS Integration Engineer** | Interface Specs, Development Guide | Architecture, Design | EN/JA |
| **Embedded Engineer (eHMI)** | eHMI Design + Architecture | Interface Specs (Serial) | EN/JA |
| **Hardware Engineer** | Hardware Requirements, Deployment | System Requirements | EN/JA |
| **Test Engineer** | Testing Guide | Requirements, Acceptance Criteria | EN/JA |
| **QA Lead** | Testing Guide, Requirements | Traceability Matrix | EN/JA |
| **Field Operator** | Operations Manual, Deployment | Troubleshooting Guide | EN/JA |
| **Maintenance Technician** | Maintenance Guide, Deployment | Hardware Requirements | EN/JA |
| **Safety Officer** | Safety Requirements, Testing | Safety Architecture | EN/JA |
| **Technical Writer** | All Documentation | Documentation Index | EN/JA |

---

## Detailed Role-Based Documentation Maps

### 1. Project Manager

**Primary Responsibilities:**
- Overall project oversight
- Requirements tracking
- Budget and timeline management
- Stakeholder communication

**Essential Documents:**

📄 **Phase 1: Requirements**
- `SYSTEM_REQUIREMENTS.md` - High-level system overview (92 requirements)
- `REQUIREMENTS_TRACEABILITY_MATRIX.md` - Verify 100% coverage
- `PERFORMANCE_REQUIREMENTS.md` - Budget and performance targets

**When to Use:**
- Project kickoff: Read System Requirements
- Weekly status: Check Traceability Matrix
- Budget review: Reference Hardware Requirements
- Stakeholder updates: Use Performance Requirements metrics

**Action Items:**
- [ ] Verify all 92 system requirements have subsystem breakdown
- [ ] Track implementation progress via traceability matrix
- [ ] Monitor cost against Hardware Requirements BOM
- [ ] Report performance against acceptance criteria

---

### 2. System Architect

**Primary Responsibilities:**
- System-level design decisions
- Subsystem integration
- Interface definitions
- Technical risk management

**Essential Documents:**

📄 **Phase 1: Requirements (All)**
- Complete understanding of system needs

📄 **Phase 2: Architecture (All 7 subsystems)**
- `SWERVE_DRIVE_ARCHITECTURE.md` - Motion control architecture
- `DOCKING_SUBSYSTEM_ARCHITECTURE.md` - Precision docking
- `NAVIGATION_SUBSYSTEM_ARCHITECTURE.md` - Localization & path planning
- `PERCEPTION_SUBSYSTEM_ARCHITECTURE.md` - Sensor processing
- `SAFETY_SUBSYSTEM_ARCHITECTURE.md` - Defense-in-depth safety
- `UI_ARCHITECTURE.md` - User interface
- `EHMI_ARCHITECTURE.md` - External communication

📄 **Phase 4: Interface Specifications**
- `INTERFACE_SPECIFICATIONS_COMPLETE.md` - All ROS 2 interfaces

**When to Use:**
- Design phase: Review all architecture documents
- Integration planning: Study interface specifications
- Technical reviews: Reference architecture decisions
- Risk assessment: Analyze safety architecture

**Action Items:**
- [ ] Ensure architectural consistency across subsystems
- [ ] Validate interface compatibility (QoS, message types)
- [ ] Review safety architecture for completeness
- [ ] Approve major design changes

---

### 3. Software Developer (ROS 2 / C++)

**Primary Responsibilities:**
- Implement ROS 2 nodes
- Write C++ control algorithms
- Unit testing
- Code reviews

**Essential Documents:**

📄 **Phase 2: Architecture (Relevant Subsystem)**
- Example: Swerve Drive developer reads `SWERVE_DRIVE_ARCHITECTURE.md`

📄 **Phase 3: Design (Relevant Subsystem)**
- Example: `SWERVE_DRIVE_CONTROLLER_DESIGN.md`
  - Contains class diagrams, method specifications, algorithms

📄 **Phase 4: Interface Specifications**
- `ROS2_TOPICS.md` - Topic definitions
- `INTERFACE_SPECIFICATIONS_COMPLETE.md` - Services, actions, custom messages

📄 **Phase 5: Development Guide**
- `COMPLETE_DEVELOPMENT_GUIDE.md`
  - CMakeLists.txt templates
  - C++ coding standards
  - ROS 2 conventions
  - Debugging tools (GDB, RViz)

📄 **Phase 6: Testing Guide**
- `COMPLETE_TESTING_GUIDE.md`
  - Unit test examples (gtest/gmock)
  - Coverage targets (80%+)

**Typical Workflow:**
1. **Understand requirements:** Read relevant section in Phase 1
2. **Study architecture:** Read Phase 2 for subsystem overview
3. **Implement design:** Follow Phase 3 class diagrams and algorithms
4. **Define interfaces:** Use Phase 4 for message/service definitions
5. **Write code:** Follow Phase 5 coding standards
6. **Test code:** Use Phase 6 unit test patterns
7. **Debug:** Reference Phase 5 debugging guide

**Code Reference Locations:**
- Swerve drive inverse kinematics: `SWERVE_DRIVE_CONTROLLER_DESIGN.md:156-180`
- ArUco outdoor optimization: `DOCKING_CONTROLLER_DESIGN.md:210-230`
- Ground removal RANSAC: `PERCEPTION_PIPELINE_DESIGN.md:180-210`

---

### 4. Software Developer (Python / UI)

**Primary Responsibilities:**
- Develop touch screen UI (React/Next.js)
- ROS 2 Python nodes
- Python testing

**Essential Documents:**

📄 **Phase 2 & 3: UI Architecture and Design**
- `UI_ARCHITECTURE.md`
- `UI_COMPONENTS_DESIGN.md`

📄 **Phase 4: Interface Specifications**
- `ROS2_TOPICS.md` - Topics to subscribe/publish
- `INTERFACE_SPECIFICATIONS_COMPLETE.md` - Custom message definitions

📄 **Phase 5: Development Guide**
- `COMPLETE_DEVELOPMENT_GUIDE.md`
  - Python coding standards (PEP 8)
  - ROS 2 Python patterns
  - package.xml for Python packages

📄 **Phase 6: Testing Guide**
- `COMPLETE_TESTING_GUIDE.md`
  - Python unit test examples (pytest)
  - Integration testing

**UI-Specific Sections:**
- rosbridge WebSocket setup: `UI_COMPONENTS_DESIGN.md:250-280`
- React component structure: `UI_COMPONENTS_DESIGN.md:150-200`
- State management (Zustand): `UI_COMPONENTS_DESIGN.md:220-240`

---

### 5. ROS Integration Engineer

**Primary Responsibilities:**
- Configure Nav2 stack
- Setup TF tree
- Launch file creation
- System integration

**Essential Documents:**

📄 **Phase 2: All Architecture Documents**
- Understand subsystem interactions

📄 **Phase 3: Navigation and Perception Design**
- `NAVIGATION_CONTROLLER_DESIGN.md` - Nav2 configuration
- `PERCEPTION_PIPELINE_DESIGN.md` - Sensor data flow

📄 **Phase 4: Interface Specifications (CRITICAL)**
- `INTERFACE_SPECIFICATIONS_COMPLETE.md`
  - Complete ROS 2 topic/service/action reference
  - QoS profile definitions
  - Communication matrix

📄 **Phase 5: Development Guide**
- `COMPLETE_DEVELOPMENT_GUIDE.md`
  - Launch file patterns
  - TF configuration
  - ROS 2 CLI tools

📄 **Phase 7: Deployment**
- `COMPLETE_DEPLOYMENT_GUIDE.md` - System services setup

**Key Reference Points:**
- TF tree structure: `NAVIGATION_CONTROLLER_DESIGN.md:80-100`
- Nav2 parameter files: `NAVIGATION_CONTROLLER_DESIGN.md:300-350`
- QoS profiles: `INTERFACE_SPECIFICATIONS_COMPLETE.md:300-330`
- Communication matrix: `INTERFACE_SPECIFICATIONS_COMPLETE.md:284-296`

---

### 6. Embedded Engineer (eHMI Firmware)

**Primary Responsibilities:**
- ESP32-S3 firmware development
- LED control (WS2812B, HUB75)
- I2S audio playback
- Serial communication

**Essential Documents:**

📄 **Phase 1: Requirements**
- `UI_EHMI_REQUIREMENTS.md` - eHMI functional requirements

📄 **Phase 2 & 3: eHMI Architecture and Design**
- `EHMI_ARCHITECTURE.md`
- `EHMI_FIRMWARE_DESIGN.md`
  - Complete firmware architecture
  - State machine (Dezyne formal methods)
  - LED patterns and audio files

📄 **Phase 4: Interface Specifications**
- `INTERFACE_SPECIFICATIONS_COMPLETE.md`
  - Serial protocol (UART 115200 baud)
  - Command/response format

📄 **Phase 5: Development Guide**
- `COMPLETE_DEVELOPMENT_GUIDE.md` - General coding standards

**Firmware-Specific Sections:**
- Serial protocol: `INTERFACE_SPECIFICATIONS_COMPLETE.md:172-202`
- State machine implementation: `EHMI_FIRMWARE_DESIGN.md:180-220`
- LED strip control: `EHMI_FIRMWARE_DESIGN.md:250-280`
- Wheelchair states (21-28): `EHMI_FIRMWARE_DESIGN.md:120-150`

---

### 7. Hardware Engineer

**Primary Responsibilities:**
- Component selection
- BOM management
- Electrical design
- Mechanical assembly

**Essential Documents:**

📄 **Phase 1: Requirements**
- `HARDWARE_REQUIREMENTS.md` - Complete hardware specifications
- `SYSTEM_REQUIREMENTS.md` - Environmental constraints (IP54, outdoor)

📄 **Phase 7: Deployment (CRITICAL)**
- `COMPLETE_DEPLOYMENT_GUIDE.md`
  - Complete BOM with 30+ components
  - Assembly instructions
  - Electrical wiring diagrams (power, CAN bus, sensors)
  - Mechanical assembly steps

**Hardware-Specific Sections:**
- BOM: `COMPLETE_DEPLOYMENT_GUIDE.md:10-100`
- Swerve module assembly: `COMPLETE_DEPLOYMENT_GUIDE.md:120-160`
- Power distribution: `COMPLETE_DEPLOYMENT_GUIDE.md:162-190`
- Wiring diagrams: `COMPLETE_DEPLOYMENT_GUIDE.md:200-250`

**Action Items:**
- [ ] Verify BOM component availability
- [ ] Review electrical wiring for safety compliance
- [ ] Validate mechanical dimensions against requirements
- [ ] Coordinate with suppliers for long-lead items (LiDAR)

---

### 8. Test Engineer / QA Lead

**Primary Responsibilities:**
- Test plan execution
- Acceptance testing
- Quality metrics
- Bug tracking

**Essential Documents:**

📄 **Phase 1: Requirements (All)**
- Baseline for all test cases

📄 **Phase 6: Testing Guide (CRITICAL)**
- `COMPLETE_TESTING_GUIDE.md`
  - Unit test plan (gtest, pytest)
  - Integration test plan (ROS 2 launch tests)
  - Field test scenarios (5 outdoor scenarios)
  - Acceptance criteria (MVP and production)
  - Test automation (CI/CD)

📄 **Phase 1: Traceability Matrix**
- `REQUIREMENTS_TRACEABILITY_MATRIX.md`
  - Verify test coverage for all requirements

**Testing Workflow:**
1. **Requirements coverage:** Map each requirement to test cases
2. **Unit testing:** Follow examples in Testing Guide
3. **Integration testing:** Use Gazebo simulation scenarios
4. **Field testing:** Execute 5 outdoor scenarios (500m nav, docking, etc.)
5. **Acceptance:** Validate against MVP/production criteria
6. **Automation:** Setup CI/CD pipeline (GitHub Actions)

**Key Metrics:**
- Code coverage target: 80%+ (line), 70%+ (branch)
- MVP success criteria: >90% mission success rate
- Production success criteria: >95% mission success rate
- Docking precision: ±5mm (outdoor), ±2mm (indoor goal)

**Reference Sections:**
- Unit test examples: `COMPLETE_TESTING_GUIDE.md:30-150`
- Field test scenarios: `COMPLETE_TESTING_GUIDE.md:220-350`
- Acceptance criteria: `COMPLETE_TESTING_GUIDE.md:400-450`

---

### 9. Field Operator

**Primary Responsibilities:**
- Daily robot operation
- Mission execution
- Basic troubleshooting
- Routine checks

**Essential Documents:**

📄 **Phase 7: Deployment (Operations Section CRITICAL)**
- `COMPLETE_DEPLOYMENT_GUIDE.md`
  - Daily startup checklist
  - Mapping procedure
  - Mission execution
  - Emergency procedures
  - Shutdown procedure

**Operator Workflow:**
1. **Daily startup:** Follow checklist (battery, sensors, E-Stop test)
2. **System check:** Verify all nodes running, status green
3. **Mission selection:** Load route from UI or config file
4. **Monitoring:** Watch robot status, safety status, docking status
5. **Emergency response:** E-Stop procedures, recovery steps
6. **Shutdown:** Proper shutdown sequence

**Critical Sections:**
- Daily startup: `COMPLETE_DEPLOYMENT_GUIDE.md:320-360`
- Mission execution: `COMPLETE_DEPLOYMENT_GUIDE.md:420-480`
- Emergency procedures: `COMPLETE_DEPLOYMENT_GUIDE.md:520-600`

**Quick Reference Cards:**
- E-Stop procedure: Press → Confirm stop → Assess → Reset → Resume
- Lost localization: Stop → Manual reposition → Set pose in RViz → Resume
- Battery low (<20%): Abort mission → Navigate to charger → Charge to 80%+

---

### 10. Maintenance Technician

**Primary Responsibilities:**
- Preventive maintenance
- Component replacement
- Calibration
- Troubleshooting

**Essential Documents:**

📄 **Phase 7: Deployment (Maintenance Section CRITICAL)**
- `COMPLETE_DEPLOYMENT_GUIDE.md`
  - Maintenance schedule (daily/weekly/monthly/quarterly/annual)
  - Component replacement procedures
  - Troubleshooting guide
  - Spare parts list

📄 **Phase 7: Calibration Procedures**
- `COMPLETE_DEPLOYMENT_GUIDE.md`
  - LiDAR calibration
  - Camera calibration (intrinsic/extrinsic)
  - IMU calibration
  - Wheel encoder calibration

**Maintenance Schedule:**
- **Daily:** Visual inspection, E-Stop test, sensor cleaning
- **Weekly:** Tire pressure, bolt torque, battery terminals
- **Monthly:** Motor bearings, CAN bus test, camera recalibration
- **Quarterly:** Tire replacement, battery capacity test
- **Annual:** Battery replacement (if capacity <80%), full system audit

**Reference Sections:**
- Maintenance schedule: `COMPLETE_DEPLOYMENT_GUIDE.md:620-680`
- Component replacement: `COMPLETE_DEPLOYMENT_GUIDE.md:690-750`
- Troubleshooting: `COMPLETE_DEPLOYMENT_GUIDE.md:760-840`
- Calibration procedures: `COMPLETE_DEPLOYMENT_GUIDE.md:260-318`

---

### 11. Safety Officer

**Primary Responsibilities:**
- Safety compliance
- Risk assessment
- Incident investigation
- Safety certification

**Essential Documents:**

📄 **Phase 1: Requirements**
- `SAFETY_REQUIREMENTS.md` (180 requirements, 78% critical)
  - Complete safety specification
  - Emergency stop requirements
  - Fail-safe behaviors

📄 **Phase 2 & 3: Safety Architecture and Design**
- `SAFETY_SUBSYSTEM_ARCHITECTURE.md`
  - Defense-in-depth (4 layers)
  - Safety state machine
- `SAFETY_MONITOR_DESIGN.md`
  - Watchdog implementation
  - Fault detection algorithms

📄 **Phase 6: Testing**
- `COMPLETE_TESTING_GUIDE.md`
  - Safety testing scenarios (Emergency Stop <100ms)
  - Field test safety protocols

**Safety Layers:**
1. **Hardware E-Stop:** <100ms physical cutoff
2. **Software E-Stop:** Distributed topic monitoring
3. **Watchdog:** Health monitoring all nodes
4. **Collision Avoidance:** Predictive obstacle detection

**Compliance Checklist:**
- [ ] ISO 13482 (Personal Care Robots) compliance
- [ ] Risk assessment (FMEA completed)
- [ ] Emergency stop response <100ms verified
- [ ] Field test safety protocols enforced
- [ ] Incident logging system operational

**Reference Sections:**
- Emergency stop design: `SAFETY_MONITOR_DESIGN.md:80-120`
- Field safety protocols: `COMPLETE_TESTING_GUIDE.md:280-320`
- Safety acceptance criteria: `COMPLETE_TESTING_GUIDE.md:420-450`

---

### 12. Technical Writer / Documentation Lead

**Primary Responsibilities:**
- Documentation maintenance
- Version control
- Consistency review
- Translation coordination

**Essential Documents:**

📄 **All Documentation**
- Complete access to all 59 files

📄 **Special Focus:**
- `DOCUMENTATION_INDEX.md` - Master index of all documents
- `REQUIREMENTS_TRACEABILITY_MATRIX.md` - Ensure traceability maintained
- This guide: `DOCUMENT_REFERENCE_GUIDE.md`

**Documentation Standards:**
- **Format:** Markdown (.md)
- **Naming:** `SUBSYSTEM_TYPE.md` (English), `SUBSYSTEM_TYPE_JA.md` (Japanese)
- **Versioning:** Document ID + version in header
- **Status:** Draft → Review → Approved → Complete
- **Bilingual:** Critical documents have EN + JA versions

**Quality Checks:**
- [ ] All cross-references valid (file:line_number format)
- [ ] Consistent terminology across documents
- [ ] All diagrams have alt-text descriptions
- [ ] Code examples properly formatted with syntax highlighting
- [ ] Japanese translations accurate and complete
- [ ] Version numbers synchronized

**Translation Coverage:**
- Phases 1-3: 100% bilingual (EN + JA)
- Phases 4-7: 100% bilingual (EN + JA)
- **Total:** 59 files (30 EN-only in Phase 1-3, 29 bilingual pairs in Phase 4-7)

---

## Documentation by Development Phase

### Phase: Requirements & Specification

**Who:** Project Manager, System Architect, All Teams (kickoff)

**Documents:**
- Phase 1: All Requirements (11 EN + 11 JA = 22 files)
- `DOCUMENTATION_INDEX.md`
- `REQUIREMENTS_TRACEABILITY_MATRIX.md`

**Purpose:** Establish baseline, align stakeholders

---

### Phase: Architecture & Design

**Who:** System Architect, Lead Developers

**Documents:**
- Phase 2: Architecture (7 EN + 7 JA = 14 files)
- Phase 3: Design (7 EN + 7 JA = 14 files)

**Purpose:** High-level system structure → Detailed implementation design

---

### Phase: Implementation

**Who:** Software Developers, Embedded Engineers, ROS Integration

**Documents:**
- Phase 4: Interface Specifications (2 EN + 2 JA = 4 files)
- Phase 5: Development Guide (1 EN + 1 JA = 2 files)
- Phase 3: Design (implementation reference)

**Purpose:** Write code following standards and interfaces

---

### Phase: Testing & Validation

**Who:** Test Engineers, QA Lead, Developers

**Documents:**
- Phase 6: Testing Guide (1 EN + 1 JA = 2 files)
- Phase 1: Requirements (test baseline)

**Purpose:** Verify implementation meets requirements

---

### Phase: Deployment & Operations

**Who:** Hardware Engineers, Field Operators, Maintenance

**Documents:**
- Phase 7: Deployment Guide (1 EN + 1 JA = 2 files)

**Purpose:** Build, deploy, operate, maintain

---

## Document Language Selection

### English (EN) Documents
**Best for:**
- International teams
- Technical specifications (ROS 2, C++ APIs)
- Code comments and documentation
- GitHub issues and pull requests

### Japanese (JA) Documents
**Best for:**
- Japanese-speaking team members
- Local operations in Japan
- Regulatory compliance (Japanese market)
- Customer-facing documentation

### Availability:
- **All phases (1-7):** Complete bilingual coverage (EN + JA)
- **Total:** 59 files

---

## Documentation Update Workflow

### When to Update Documentation

**Requirements Change:**
1. Update affected requirement document
2. Update traceability matrix
3. Propagate to architecture/design if needed

**Architecture Change:**
1. Update architecture document
2. Update affected design documents
3. Update interface specifications
4. Notify all developers

**Interface Change (CRITICAL):**
1. Update `INTERFACE_SPECIFICATIONS_COMPLETE.md`
2. Update `ROS2_TOPICS.md` if topic changed
3. Notify all teams (breaking change!)
4. Update design documents using the interface

**Implementation Discovery:**
1. Update design document with actual implementation
2. Add notes in development guide if general pattern

---

## Quick Reference: Where to Find Specific Information

| Information Needed | Document | Section |
|-------------------|----------|---------|
| **System Overview** | SYSTEM_REQUIREMENTS.md | Section 1 |
| **Hardware BOM** | COMPLETE_DEPLOYMENT_GUIDE.md | Section 1.1 |
| **ROS 2 Topic List** | ROS2_TOPICS.md | Section 10 (table) |
| **Swerve Kinematics** | SWERVE_DRIVE_CONTROLLER_DESIGN.md | Section 2.3 |
| **ArUco Detection** | DOCKING_CONTROLLER_DESIGN.md | Section 2.1 |
| **Nav2 Configuration** | NAVIGATION_CONTROLLER_DESIGN.md | Section 3.2 |
| **Safety Layers** | SAFETY_SUBSYSTEM_ARCHITECTURE.md | Section 2 |
| **UI Components** | UI_COMPONENTS_DESIGN.md | Section 2 |
| **eHMI States** | EHMI_FIRMWARE_DESIGN.md | Section 2.1 |
| **Serial Protocol** | INTERFACE_SPECIFICATIONS_COMPLETE.md | Section 3 |
| **CMakeLists Template** | COMPLETE_DEVELOPMENT_GUIDE.md | Section 2.2 |
| **Unit Test Example** | COMPLETE_TESTING_GUIDE.md | Section 1.1 |
| **Field Test Scenarios** | COMPLETE_TESTING_GUIDE.md | Section 3.2 |
| **Daily Startup** | COMPLETE_DEPLOYMENT_GUIDE.md | Section 4.1 |
| **Calibration** | COMPLETE_DEPLOYMENT_GUIDE.md | Section 3 |
| **Troubleshooting** | COMPLETE_DEPLOYMENT_GUIDE.md | Section 5.3 |

---

## Document Naming Convention Summary

**Format:** `SUBSYSTEM_TYPE.md` (English) or `SUBSYSTEM_TYPE_JA.md` (Japanese)

**Examples:**
- `SWERVE_DRIVE_ARCHITECTURE.md` (English architecture)
- `SWERVE_DRIVE_ARCHITECTURE_JA.md` (Japanese architecture)
- `COMPLETE_DEVELOPMENT_GUIDE.md` (English consolidated guide)
- `COMPLETE_DEVELOPMENT_GUIDE_JA.md` (Japanese consolidated guide)

---

## Contact & Support

**Documentation Issues:**
- Report missing/incorrect information via GitHub issues
- Tag with `documentation` label
- Assign to Technical Writer

**Translation Requests:**
- Request additional language support via project manager
- Current supported languages: English (EN), Japanese (JA)

**Document Access:**
- All documents: `docs/active/outdoor_swerve_system/`
- Index: `DOCUMENTATION_INDEX.md`
- This guide: `DOCUMENT_REFERENCE_GUIDE.md`

---

**Document Status:** Complete
**Last Updated:** 2025-12-15
**Maintained By:** Technical Writing Team
**Approvals Required:** Project Manager, System Architect, Documentation Lead
