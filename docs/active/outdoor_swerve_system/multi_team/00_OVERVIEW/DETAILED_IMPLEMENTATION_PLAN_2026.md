# Detailed Implementation Plan 2026
**Outdoor Wheelchair Transport Robot - Multi-Team System**

**Duration:** 26 weeks (January 15 - July 19, 2026)
**Strategy:** Fast prototype → Parallel development → Early integration → Focused testing
**Success Probability:** 85-90%

---

## Executive Summary

This detailed implementation plan provides week-by-week execution strategy for the outdoor wheelchair transport robot fleet system. The plan is optimized based on analysis of project risks and incorporates 4 weeks of contingency buffer to ensure successful delivery.

**Key Optimizations vs. Traditional Approach:**
- Hardware "Walking Skeleton" ready 5 weeks earlier (Feb 14 vs Mar 20)
- TVM MVP ready 5 weeks earlier (Apr 30 vs Jun 4)
- 3 weeks dedicated vehicle-TVM integration (vs 0 weeks)
- 4 weeks contingency buffer for unexpected issues

---

## Table of Contents

1. [Phase 0: Hardware - Fast Prototype](#phase-0-hardware---fast-prototype)
2. [Phase 0B: Hardware - Refinement](#phase-0b-hardware---refinement)
3. [Phase 1: Vehicle Software Development](#phase-1-vehicle-software-development)
4. [Phase 2: Fleet Management (TVM) Development](#phase-2-fleet-management-tvm-development)
5. [Phase 3: System Integration](#phase-3-system-integration)
6. [Phase 4: Field Testing](#phase-4-field-testing)
7. [Phase 5: Contingency & Polish](#phase-5-contingency--polish)
8. [Weekly Milestone Tracking](#weekly-milestone-tracking)
9. [Risk Management](#risk-management)
10. [Resource Allocation](#resource-allocation)
11. [Quality Gates](#quality-gates)

---

## Phase 0: Hardware - Fast Prototype

**Duration:** 5 weeks (Jan 15 - Feb 14, 2026)
**Owner:** Okinawa (Tsuchiya)
**Goal:** Basic functioning platform for early software development

### Week 1: Sensor Selection & Layout (Jan 15-21)

**Objectives:**
- Select outdoor-grade sensors (LiDAR, cameras, IMU)
- Define mounting positions and brackets
- Order long-lead-time components

**Deliverables:**
- [ ] Sensor list with specifications
- [ ] Mounting layout CAD (preliminary)
- [ ] Purchase orders submitted

**Success Criteria:**
- All sensors support outdoor operation (IP65+)
- Mounting positions don't interfere with swerve drive
- Lead time <4 weeks for all components

---

### Week 2: Architecture & PC Selection (Jan 22-28)

**Objectives:**
- Define electrical architecture (power distribution, CAN topology)
- Select compute platform (Jetson Orin Nano or equivalent)
- Define mechanical structure approach

**Deliverables:**
- [ ] Hardware architecture diagram
- [ ] PC selected and ordered
- [ ] Power budget calculation

**Success Criteria:**
- Architecture supports all subsystems
- PC has sufficient CPU/GPU for ROS 2 workload
- Total power <500W @ 48V

---

### Week 3-4: Basic Platform Build (Jan 29 - Feb 11)

**Objectives:**
- Assemble basic chassis (no suspension initially)
- Install 4× swerve drive motors (drive only, no steering motors yet)
- Wire power distribution (48V battery, BMS, fuses)
- Basic CAN bus setup

**Deliverables:**
- [ ] Chassis assembled
- [ ] 4× drive motors installed and wired
- [ ] Power system functional
- [ ] CAN bus tested with motor controllers

**Success Criteria:**
- Robot can roll forward/backward (no steering)
- Power system stable under load
- CAN communication verified with oscilloscope

---

### Week 5: LiDAR + Camera + CAN (Feb 12-14)

**Objectives:**
- Mount LiDAR and cameras
- Connect sensors to compute platform
- Verify CAN bus communication with motors
- Test basic ROS 2 sensor drivers

**Deliverables:**
- [ ] LiDAR publishing point clouds
- [ ] Cameras publishing images
- [ ] Motor controllers responding to CAN commands
- [ ] **Walking Skeleton Complete** ✅

**Success Criteria:**
- All sensors publishing data in ROS 2
- Motor controllers can be commanded via CAN
- Platform can drive in straight line (manual control)

**🎯 Milestone: Walking Skeleton (Feb 14, 2026)**
- Basic platform functional
- Pankaj can begin software development on real hardware
- 5 weeks ahead of traditional schedule

---

## Phase 0B: Hardware - Refinement

**Duration:** 11 weeks (Feb 15 - May 1, 2026)
**Owner:** Okinawa (Tsuchiya) + Kirill (CAD)
**Goal:** Complete hardware with all features

### Week 6-8: Swerve Drive Mechanism (Feb 15 - Mar 6)

**Objectives:**
- Add steering motors to all 4 wheels
- Implement suspension system
- Calibrate swerve module angles
- Test omnidirectional motion

**Deliverables:**
- [ ] 4× complete swerve modules (drive + steer)
- [ ] Suspension functional
- [ ] Kinematic calibration complete

**Success Criteria:**
- Robot can move omnidirectionally (forward, sideways, rotate)
- Suspension reduces vibration on rough terrain
- Steering angle accuracy ±2°

---

### Week 9-10: Final Hardware Integration (Mar 7-20)

**Objectives:**
- Install remaining sensors (bumpers, IMU, etc.)
- Complete wiring harness
- Weatherproofing (IP54+ sealing)
- Cable management

**Deliverables:**
- [ ] All sensors installed
- [ ] Complete wiring harness with strain relief
- [ ] Weatherproofing tested (water spray test)
- [ ] **Final Hardware Platform** ✅

**Success Criteria:**
- All sensors operational
- Weatherproofing validated (IP54+)
- No loose cables

**🎯 Milestone: Final Hardware Complete (Mar 20, 2026)**

---

### Week 11-12: Docking Mechanism (Mar 21 - Apr 3)

**Objectives:**
- Design docking hardware (latch, sensors)
- Fabricate docking mechanism
- Install and test mechanical coupling
- ArUco marker placement

**Deliverables:**
- [ ] Docking mechanism functional
- [ ] Attachment sensors working
- [ ] ArUco markers installed on test wheelchair

**Success Criteria:**
- Mechanical coupling can attach/detach
- Attachment confirmation sensor reliable
- ArUco markers visible from 2-5m distance

---

### Week 13-16: Control System Integration (Apr 4 - May 1)

**Objectives:**
- Integrate all motor controllers
- Emergency stop circuit
- Safety interlocks
- Final system testing

**Deliverables:**
- [ ] All motors controlled via ROS 2
- [ ] E-stop circuit tested
- [ ] Safety systems functional
- [ ] **Hardware Phase Complete** ✅

**Success Criteria:**
- Emergency stop works (<100ms response)
- All safety interlocks functional
- System ready for intensive software testing

**🎯 Milestone: Control System Complete (May 1, 2026)**

---

### Parallel: Exterior CAD (Kirill, Mar 2-27)

**Objectives:**
- Design exterior panels
- Prepare CAD for outsourcing
- Coordinate with fabrication vendor

**Deliverables:**
- [ ] CAD files complete (Week 7)
- [ ] Vendor selected (Week 8)
- [ ] Fabrication order placed (Week 9)
- [ ] Parts received (Week 11-12)

---

### Parallel: Hardware Support/Maintenance (May 1 - Jul 19)

**Objectives:**
- On-call hardware support during software testing
- Repair broken components
- Hardware modifications as needed

**Resource:** Kirill (on-site support)

---

## Phase 1: Vehicle Software Development

**Duration:** 18 weeks (Feb 15 - May 10, 2026)
**Owner:** Pankaj
**Goal:** Complete autonomous vehicle software

### Development Environment Setup (Feb 15-18)

**Objectives:**
- Set up ROS 2 Humble on real hardware
- Configure CAN bus drivers
- Test basic sensor integration
- Establish development workflow

**Deliverables:**
- [ ] ROS 2 workspace configured
- [ ] All sensors publishing in ROS 2
- [ ] Development tools installed

**Success Criteria:**
- Can SSH to vehicle computer
- All sensors accessible via ROS 2 topics
- Git repository set up

---

### Swerve Drive Development (Feb 19 - Feb 28, 10 days)

**Objectives:**
- Implement inverse kinematics
- Develop ROS 2 controller node
- Test basic motion commands
- Calibrate wheel parameters

**Deliverables:**
- [ ] `swerve_drive_controller` package
- [ ] Twist → CAN command translation
- [ ] Odometry publisher
- [ ] Nav2 controller plugin

**Testing:**
- Unit tests for kinematics
- Integration test with real hardware
- Motion profiling (acceleration, max speed)

**Success Criteria:**
- Robot responds to Twist commands
- Odometry accuracy ±5cm over 10m
- Max speed 1.5 m/s achieved

---

### Navigation Development (Mar 1 - Mar 11, 11 days)

**Objectives:**
- Integrate Nav2 stack
- Implement NDT localization
- Configure costmaps
- Waypoint navigation

**Deliverables:**
- [ ] Nav2 configured
- [ ] NDT scan matcher working
- [ ] Local + global costmaps
- [ ] Waypoint manager node

**Testing:**
- Localization drift test (100m, 500m)
- Path following accuracy
- Obstacle avoidance scenarios

**Success Criteria:**
- Localization drift <10cm over 100m
- Path following error <20cm
- Avoids static obstacles reliably

---

### Perception Development (Mar 12 - Mar 19, 8 days)

**Objectives:**
- LiDAR point cloud processing
- Ground removal (RANSAC)
- Obstacle detection
- Sensor fusion

**Deliverables:**
- [ ] `pcl_processor` package
- [ ] Ground removal node
- [ ] Obstacle detector node
- [ ] Costmap integration

**Testing:**
- Ground removal accuracy (various terrains)
- Obstacle detection precision/recall
- Computational performance (CPU usage)

**Success Criteria:**
- Ground removal >95% accuracy
- Obstacle detection range 0.1-30m
- Processing time <50ms per scan

---

### Docking Development (Mar 20 - Mar 27, 8 days)

**Objectives:**
- ArUco marker detection
- Visual servoing controller
- Docking state machine
- Precision approach

**Deliverables:**
- [ ] `aruco_detector` package
- [ ] `docking_controller` package
- [ ] Visual servoing PID tuned
- [ ] Docking state machine

**Testing:**
- ArUco detection range and robustness
- Docking precision measurement
- Success rate over 50 attempts

**Success Criteria:**
- Detection range 2-5m
- Docking precision ±5mm
- Success rate >90%

---

### Safety Development (Mar 28 - Apr 2, 6 days)

**Objectives:**
- Emergency stop handling
- Safety monitors
- Health checks
- Fail-safe behaviors

**Deliverables:**
- [ ] `safety_monitor` package
- [ ] E-stop handling
- [ ] Sensor health checks
- [ ] Watchdog timer

**Testing:**
- E-stop response time
- Sensor failure scenarios
- Recovery behaviors

**Success Criteria:**
- E-stop response <100ms
- Sensor failures detected <1s
- Safe recovery from all failure modes

---

### UI Development (Apr 3 - Apr 6, 4 days)

**Objectives:**
- React web application
- rosbridge WebSocket
- Basic mission control UI

**Deliverables:**
- [ ] Touch screen UI
- [ ] Mission selection interface
- [ ] Status dashboard

**Success Criteria:**
- UI responsive on touch screen
- Can start/stop missions
- Status updates in real-time

---

### eHMI Development (Apr 7 - Apr 9, 3 days)

**Objectives:**
- ROS 2 → ESP32 serial bridge
- State management
- Integration test

**Deliverables:**
- [ ] `ehmi_interface` package
- [ ] Serial communication working
- [ ] State mapping complete

**Success Criteria:**
- LED states update correctly
- Audio plays appropriate messages
- Latency <500ms

---

### Unit Testing Phase (Apr 10 - Apr 26, 17 days)

**Module-by-module unit testing:**

- **Swerve Drive** (Apr 10-12, 3 days)
  - Kinematics unit tests
  - Controller edge cases
  - CAN communication reliability

- **Navigation** (Apr 13-15, 3 days)
  - Localization accuracy
  - Path planning correctness
  - Recovery behaviors

- **Perception** (Apr 16-18, 3 days)
  - Point cloud processing
  - Obstacle detection accuracy
  - Performance benchmarks

- **Docking** (Apr 19-21, 3 days)
  - ArUco detection robustness
  - Visual servoing stability
  - Precision measurement

- **Safety** (Apr 22-23, 2 days)
  - E-stop timing
  - Health check coverage
  - Fail-safe verification

- **UI/eHMI** (Apr 24-26, 3 days)
  - UI responsiveness
  - State synchronization
  - Integration testing

**Success Criteria for Unit Testing:**
- ≥80% code coverage
- All critical paths tested
- No known high-severity bugs

---

### Integration Testing Phase (Apr 27 - May 10, 14 days)

**Subsystem integration:**

- **Swerve Drive Integration** (Apr 27-29, 3 days)
  - Nav2 controller plugin
  - Odometry accuracy
  - Motion profiling

- **Navigation Integration** (Apr 30 - May 2, 3 days)
  - Full navigation stack
  - Multi-waypoint missions
  - Recovery scenarios

- **Perception Integration** (May 3-5, 3 days)
  - Sensor fusion
  - Costmap integration
  - Dynamic obstacles

- **Docking Integration** (May 6-7, 2 days)
  - End-to-end docking sequence
  - State machine transitions
  - Error handling

- **Safety Integration** (May 8-9, 2 days)
  - System-wide safety checks
  - Emergency behaviors
  - Watchdog integration

- **UI/eHMI Integration** (May 10, 1 day)
  - Complete UI workflow
  - Status updates
  - User commands

**Success Criteria for Integration Testing:**
- All subsystems communicate correctly
- State machines handle all transitions
- No integration bugs

**🎯 Milestone: Vehicle Software Complete (May 10, 2026)**

---

## Phase 2: Fleet Management (TVM) Development

**Duration:** 20 weeks (Jan 16 - Jun 4, 2026)
**Owner:** Unno
**Strategy:** Critical MVP first, then secondary features

### Critical Path MVP (Jan 16 - Apr 30, 15 weeks)

**Goal:** Minimum viable TVM system for vehicle-fleet integration

#### Week 1-2: Login & Admin (Jan 16 - Feb 4)

**Login Function (Priority):**
- JWT authentication
- Session management
- Password hashing (bcrypt)

**Admin Registration + Permissions:**
- User CRUD operations
- Role-based access control (Admin, Operator, Nurse, Caregiver)
- Permission management

**Deliverables:**
- [ ] Authentication service
- [ ] Admin user management
- [ ] RBAC implemented

---

#### Week 3-4: Route & Map (Feb 5-25)

**Route Setting:**
- Waypoint management
- Path visualization
- Route templates

**Map Confirmation (Basic):**
- OSM or Google Maps integration
- Vehicle location display
- Geofence boundaries

**Battery Display:**
- Real-time battery monitoring
- Low battery alerts
- Charging status

**Deliverables:**
- [ ] Route management interface
- [ ] Map view with vehicle tracking
- [ ] Battery dashboard

---

#### Week 5-7: Reservation System (Feb 26 - Mar 18)

**Reservation Function (Basic):**
- Booking interface
- Time slot management
- Confirmation system

**Reception + Cancellation:**
- Booking acceptance workflow
- Cancellation handling
- Status updates

**Emergency Setting:**
- Emergency mode toggle
- Priority dispatch
- Alert notifications

**Deliverables:**
- [ ] Reservation system functional
- [ ] Reception workflow complete
- [ ] Emergency controls working

---

#### Week 8-10: Floor Management (Mar 19 - Apr 6)

**Floor Map Registration:**
- Upload floor plan images
- Coordinate calibration
- Multi-floor support

**Room Number Setting:**
- Room annotation
- Destination management
- Quick select interface

**Deliverables:**
- [ ] Floor map management
- [ ] Room number database
- [ ] Destination quick-select

---

#### Week 11-13: Walking Reservation (Apr 7-24)

**Walking Reservation (Basic Function):**
- Schedule walking assistance
- Vehicle assignment algorithm
- Queue management

**Call Function:**
- Staff → vehicle communication
- Call initiation interface
- Status tracking

**Deliverables:**
- [ ] Walking reservation system
- [ ] Call function operational
- [ ] Queue management working

---

#### Week 14-15: Status & Polish (Apr 25 - Apr 30)

**Status Confirmation:**
- Real-time vehicle status
- Mission progress tracking
- System health dashboard

**MVP Integration Testing:**
- End-to-end workflows
- Load testing
- Bug fixes

**Deliverables:**
- [ ] Status dashboard complete
- [ ] MVP feature-complete
- [ ] Integration tests passing

**🎯 Milestone: TVM MVP Complete (Apr 30, 2026)**

---

### Secondary Features (May 1 - Jun 4, 5 weeks)

**Goal:** Enhanced features for production deployment

#### Week 16-17: Resident Info (May 1-8)

**Resident Info Setting:**
- Resident database
- Medical information
- Preferences management

**Deliverables:**
- [ ] Resident management interface
- [ ] Data privacy compliant

---

#### Week 18-19: Advanced Features (May 9-22)

**Return Home Mode:**
- Auto-return on low battery
- End-of-shift return
- Idle management

**Walking Reservation (Advanced):**
- Recurring schedules
- Priority handling
- Multi-vehicle coordination

**Deliverables:**
- [ ] Return home logic
- [ ] Advanced scheduling

---

#### Week 20-21: Notifications & Voice (May 23 - Jun 4)

**Advance Notification:**
- Arrival notifications
- Delay alerts
- Status updates

**Voice Call Function:**
- VoIP integration
- Audio streaming
- Call history

**Deliverables:**
- [ ] Notification system
- [ ] Voice call working

**🎯 Milestone: TVM Feature Complete (Jun 4, 2026)**

---

## Phase 3: System Integration

**Duration:** 3 weeks (May 11 - May 31, 2026)
**Owners:** All Teams
**Goal:** Integrate vehicle software + hardware + TVM

### Week 1: Vehicle SW + Hardware (May 11-17)

**Objectives:**
- Verify all ROS 2 nodes work on final hardware
- Tune control parameters
- Performance optimization

**Activities:**
- Motion testing on complete platform
- Sensor calibration verification
- Performance profiling

**Deliverables:**
- [ ] All subsystems working on final hardware
- [ ] Control parameters tuned
- [ ] No hardware-software integration issues

**Success Criteria:**
- Vehicle can navigate autonomously
- Docking works on complete platform
- All sensors providing clean data

---

### Week 2: Vehicle + TVM Integration (Telemetry) (May 18-24)

**Objectives:**
- Vehicle → TVM telemetry working
- Real-time location updates
- Battery status reporting

**Activities:**
- Test TVM client on vehicle
- Verify WebSocket connection
- Load testing (telemetry at 1 Hz)

**Deliverables:**
- [ ] Telemetry streaming to TVM
- [ ] Dashboard displays vehicle status
- [ ] Network reliability tested

**Success Criteria:**
- Telemetry latency <5s
- No data loss over 1 hour test
- Dashboard updates correctly

---

### Week 3: Vehicle + TVM Integration (Mission Control) (May 25-31)

**Objectives:**
- TVM → Vehicle command working
- Mission dispatch functional
- End-to-end mission testing

**Activities:**
- Test mission commands (dispatch, cancel, e-stop)
- Verify state synchronization
- Error handling scenarios

**Deliverables:**
- [ ] Mission commands working
- [ ] Vehicle executes TVM missions
- [ ] Error recovery tested

**Success Criteria:**
- Commands execute within 1s
- State stays synchronized
- Graceful error handling

**🎯 Milestone: System Integration Complete (May 31, 2026)**

---

## Phase 4: Field Testing

**Duration:** 3 weeks (Jun 1 - Jun 21, 2026)
**Owners:** Pankaj + Kirill (On-site)
**Goal:** Validate in real-world conditions

### Week 1: Component Testing (Jun 1-8)

**Swerve Drive Field Test (Jun 1-4):**
- Outdoor terrain testing
- Slope testing (up to 10°)
- Weather testing (light rain)
- Performance measurement

**Navigation Field Test (Jun 5-8):**
- Long-distance navigation (500m, 1km)
- Multi-waypoint missions
- Localization drift measurement
- Recovery scenarios

**Deliverables:**
- [ ] Swerve drive validated outdoors
- [ ] Navigation accuracy verified
- [ ] Performance data collected

---

### Week 2: Subsystem Testing (Jun 9-15)

**Perception + Safety (Jun 9-12):**
- Obstacle detection in various conditions
- Safety system validation
- Emergency stop testing
- Tilt detection accuracy

**Docking Field Test (Jun 9-12):**
- Outdoor docking precision
- Weather impact on ArUco detection
- Success rate measurement
- Robustness testing

**UI/eHMI Field Test (Jun 13-15):**
- User interaction testing
- Touch screen usability outdoors
- eHMI visibility/audibility
- Multi-language verification

**Deliverables:**
- [ ] All subsystems validated
- [ ] Known issues documented
- [ ] Performance meets requirements

---

### Week 3: System Integration (Jun 16-21)

**Complete System Testing (Jun 16-18):**
- End-to-end missions with TVM
- Multi-mission sequences
- Error recovery
- System reliability

**Outdoor Field Validation (Final) (Jun 19-21):**
- Real-world deployment simulation
- User acceptance testing
- Performance validation
- Production readiness check

**Deliverables:**
- [ ] System validated in real conditions
- [ ] User feedback collected
- [ ] Final bugs fixed
- [ ] Production configuration complete

**Success Criteria:**
- Mission success rate >90%
- Docking success rate >90%
- No critical safety issues
- User satisfaction >4/5

**🎯 Milestone: Field Validation Complete (Jun 21, 2026)**

---

## Phase 5: Contingency & Polish

**Duration:** 4 weeks (Jun 22 - Jul 19, 2026)
**Owners:** All Teams
**Goal:** Fix critical bugs and prepare for deployment

### Week 1-2: Critical Bug Fixes (Jun 22 - Jul 5)

**Priorities:**
1. Safety-critical issues (P0)
2. Mission-blocking issues (P1)
3. Performance issues (P2)
4. UX issues (P3)

**Activities:**
- Bug triage
- Root cause analysis
- Fix implementation
- Regression testing

**Deliverables:**
- [ ] All P0/P1 bugs fixed
- [ ] P2 bugs addressed
- [ ] Known issues documented

---

### Week 3: Verification (Jul 6-12)

**Objectives:**
- Re-test all fixes
- Verify no regressions
- Performance validation

**Activities:**
- Full system regression test
- Performance benchmarks
- User acceptance test

**Deliverables:**
- [ ] All fixes verified
- [ ] Performance meets targets
- [ ] No new critical issues

---

### Week 4: Final Polish & Documentation (Jul 13-19)

**Objectives:**
- Final system polishing
- Documentation updates
- Deployment preparation

**Activities:**
- Code cleanup
- Documentation review
- Deployment checklist
- Handover preparation

**Deliverables:**
- [ ] System production-ready
- [ ] Documentation complete
- [ ] Deployment package ready
- [ ] Training materials prepared

**🎯 Final Delivery (Jul 19, 2026)**

---

## Weekly Milestone Tracking

| Week | Date | Hardware | Vehicle SW | Fleet Mgmt | Status |
|------|------|----------|------------|------------|--------|
| 1 | Jan 15-21 | Sensor selection | - | Login setup | ⬜ Pending |
| 2 | Jan 22-28 | Architecture | - | Admin + RBAC | ⬜ Pending |
| 3 | Jan 29 - Feb 4 | Basic platform | - | Route setting | ⬜ Pending |
| 4 | Feb 5-11 | Platform build | - | Map + Battery | ⬜ Pending |
| 5 | Feb 12-14 | **Walking Skeleton** ✅ | Dev setup | Reservation | ⬜ Pending |
| 6 | Feb 15-21 | Swerve mech | Swerve SW | Reception | ⬜ Pending |
| 7 | Feb 22-28 | Swerve mech | Navigation | Emergency | ⬜ Pending |
| 8 | Mar 1-6 | Swerve done | Navigation | Floor map | ⬜ Pending |
| 9 | Mar 7-13 | Final HW | Perception | Room numbers | ⬜ Pending |
| 10 | Mar 14-20 | **HW Platform** ✅ | Perception | Resident info | ⬜ Pending |
| 11 | Mar 21-27 | Docking mech | Docking SW | Walking reservation | ⬜ Pending |
| 12 | Mar 28 - Apr 3 | Docking mech | Safety | Walking reservation | ⬜ Pending |
| 13 | Apr 4-9 | Control system | UI + eHMI | Call function | ⬜ Pending |
| 14 | Apr 10-16 | Control system | Unit testing | Status | ⬜ Pending |
| 15 | Apr 17-23 | Control system | Unit testing | **TVM MVP Done** ⬜ | ⬜ Pending |
| 16 | Apr 24-30 | Control done | Unit testing | Secondary features | ⬜ Pending |
| 17 | May 1-7 | **HW Complete** ✅ | Integration | Return home | ⬜ Pending |
| 18 | May 8-10 | Support | **Vehicle SW Done** ✅ | Advanced | ⬜ Pending |
| 19 | May 11-17 | Support | Integration | Notifications | ⬜ Pending |
| 20 | May 18-24 | Support | Integration | Voice call | ⬜ Pending |
| 21 | May 25-31 | Support | **Integration Done** ✅ | Voice call | ⬜ Pending |
| 22 | Jun 1-7 | Support | Field test | **TVM Done** ✅ | ⬜ Pending |
| 23 | Jun 8-14 | Support | Field test | Testing | ⬜ Pending |
| 24 | Jun 15-21 | Support | **Field Done** ✅ | Testing | ⬜ Pending |
| 25 | Jun 22-28 | Support | Bug fixes | Bug fixes | ⬜ Pending |
| 26 | Jun 29 - Jul 5 | Support | Bug fixes | Bug fixes | ⬜ Pending |
| Buffer | Jul 6-12 | Support | Verification | Verification | ⬜ Pending |
| Buffer | Jul 13-19 | Support | **DELIVERY** ✅ | **DELIVERY** ✅ | ⬜ Pending |

---

## Risk Management

### Critical Risks

| Risk ID | Risk Description | Probability | Impact | Mitigation Strategy | Owner |
|---------|------------------|-------------|--------|---------------------|-------|
| R1 | Hardware delays beyond Feb 14 | Medium | High | Walking skeleton by Feb 14, Pankaj can work on simulation if needed | Okinawa |
| R2 | Swerve drive control more complex than expected | Medium | High | Start early (Feb 19), allocate 10 days, seek external help if needed | Pankaj |
| R3 | Outdoor docking precision fails | Low | High | ArUco proven technology, extensive testing in Week 11-12 | Pankaj |
| R4 | TVM-Vehicle integration issues | Medium | Medium | 3 weeks dedicated integration (May 11-31), clear interface specs | Both |
| R5 | Weather delays field testing | High | Low | Indoor backup facility, flexible testing schedule | All |
| R6 | Unexpected critical bug in Week 22+ | Medium | Medium | 4-week contingency buffer (Jun 22 - Jul 19) | All |

### Risk Response Actions

**If Walking Skeleton delayed beyond Feb 21 (+1 week):**
- Action: Pankaj continues on simulation
- Impact: Reduces real hardware testing time by 1 week
- Recovery: Use contingency buffer

**If TVM MVP not ready by Apr 30:**
- Action: Delay integration to Week 21-23
- Impact: Reduces field testing time by 1 week
- Recovery: Use contingency buffer

**If field testing reveals critical issues:**
- Action: Use contingency buffer (Jun 22 - Jul 19)
- Impact: May reduce polish time
- Recovery: Prioritize P0/P1 bugs only

---

## Resource Allocation

### Team Allocation by Phase

| Phase | Okinawa | Kirill | Pankaj | Unno | Notes |
|-------|---------|--------|--------|------|-------|
| **Week 1-5 (Fast Prototype)** | 100% | 0% | 0% | 10% | Okinawa full-time, Unno planning |
| **Week 6-10 (Parallel Dev)** | 100% | 20% | 100% | 100% | All teams active |
| **Week 11-16 (Parallel Dev)** | 100% | 50% | 100% | 100% | Kirill increases (eHMI) |
| **Week 17-21 (Integration)** | 50% | 100% | 100% | 100% | Kirill on-site support |
| **Week 22-24 (Field Test)** | 20% | 100% | 100% | 50% | Kirill + Pankaj on-site |
| **Week 25-28 (Buffer)** | 20% | 50% | 100% | 50% | Bug fixes |

### Total Person-Weeks

| Team Member | Weeks | Percentage | Equivalent Full-Time |
|-------------|-------|------------|----------------------|
| Okinawa | 28 | 70% | 19.6 weeks |
| Kirill | 28 | 50% | 14 weeks |
| Pankaj | 28 | 95% | 26.6 weeks |
| Unno | 28 | 80% | 22.4 weeks |
| **Total** | - | - | **82.6 person-weeks** |

---

## Quality Gates

### Gate 1: Walking Skeleton (Feb 14, 2026)

**Criteria:**
- [ ] Basic platform can drive (forward/backward)
- [ ] LiDAR publishes point clouds
- [ ] Cameras publish images
- [ ] CAN bus communication verified

**Decision:** Go/No-Go for vehicle software development

---

### Gate 2: Hardware Platform Complete (Mar 20, 2026)

**Criteria:**
- [ ] Omnidirectional motion working
- [ ] All sensors operational
- [ ] Weatherproofing validated
- [ ] No hardware blockers

**Decision:** Go/No-Go for integration phase

---

### Gate 3: TVM MVP + Vehicle SW Complete (Apr 30 & May 10, 2026)

**Criteria:**
- [ ] TVM MVP feature-complete
- [ ] Vehicle software unit tests passing
- [ ] No critical bugs
- [ ] Ready for integration

**Decision:** Go/No-Go for system integration

---

### Gate 4: System Integration Complete (May 31, 2026)

**Criteria:**
- [ ] Vehicle-TVM communication working
- [ ] End-to-end missions successful
- [ ] Integration tests passing
- [ ] Ready for field testing

**Decision:** Go/No-Go for field deployment

---

### Gate 5: Field Validation Complete (Jun 21, 2026)

**Criteria:**
- [ ] Mission success rate >90%
- [ ] No critical safety issues
- [ ] User acceptance achieved
- [ ] Performance validated

**Decision:** Go/No-Go for production deployment

---

### Final Gate: Production Ready (Jul 19, 2026)

**Criteria:**
- [ ] All P0/P1 bugs fixed
- [ ] Performance meets targets
- [ ] Documentation complete
- [ ] Deployment package ready

**Decision:** Production deployment approved

---

## Success Metrics

### Technical Metrics

| Metric | Target | Measurement Method |
|--------|--------|-------------------|
| Mission success rate | ≥90% | Track over 100 missions |
| Docking success rate | ≥90% | Track over 50 docking attempts |
| Localization drift | <10cm over 100m | GPS ground truth comparison |
| Obstacle detection | >95% precision, >90% recall | Annotated test dataset |
| Battery life | ≥4 hours | Field test with passenger |
| E-stop response | <100ms | Oscilloscope measurement |

### Schedule Metrics

| Milestone | Target Date | Buffer | Status |
|-----------|-------------|--------|--------|
| Walking Skeleton | Feb 14 | +1 week | ⬜ Pending |
| Hardware Complete | May 1 | +2 weeks | ⬜ Pending |
| Vehicle SW Complete | May 10 | +1 week | ⬜ Pending |
| TVM MVP Complete | Apr 30 | +2 weeks | ⬜ Pending |
| Integration Complete | May 31 | +1 week | ⬜ Pending |
| Field Validation | Jun 21 | +4 weeks | ⬜ Pending |
| Final Delivery | Jul 19 | - | ⬜ Pending |

---

## Communication Plan

### Daily Standups (Async)

**Participants:** All team members
**Format:** Slack/Email
**Content:**
- What I did yesterday
- What I'm doing today
- Any blockers

---

### Weekly Sync Meeting

**Participants:** Okinawa, Kirill, Pankaj, Unno
**Duration:** 60 minutes
**Agenda:**
1. Progress review (each team, 10 min)
2. Milestone tracking (5 min)
3. Blocker resolution (20 min)
4. Next week planning (15 min)
5. Risk review (10 min)

---

### Milestone Reviews

**Schedule:** Every 4 weeks
**Participants:** All teams + stakeholders
**Duration:** 90 minutes
**Content:**
- Demo of completed work
- Milestone achievement review
- Risk assessment update
- Next phase planning

---

## Appendix A: Comparison with Original Plan

| Aspect | Original Plan | Recommended Plan | Improvement |
|--------|--------------|------------------|-------------|
| Hardware prototype | Mar 20 (Week 10) | Feb 14 (Week 5) | **5 weeks earlier** |
| Real HW testing time | 8 weeks | 12 weeks | **+4 weeks** |
| TVM MVP ready | Jun 4 (Week 20) | Apr 30 (Week 15) | **5 weeks earlier** |
| Vehicle-TVM integration | 0 weeks | 3 weeks (May 11-31) | **+3 weeks** |
| Field testing | 7 weeks (fragmented) | 3 weeks (focused) | More efficient |
| Contingency buffer | 0 weeks | 4 weeks | **Risk mitigation** |
| Success probability | 50-60% | 85-90% | **+30% higher** |

---

## Appendix B: Contingency Planning

### If Walking Skeleton Delayed (>Feb 21)

**Plan B:**
- Pankaj continues on simulation
- Hardware team adds weekend work
- Reduce refinement phase by 1 week

**Impact:** -1 week real hardware testing time
**Recovery:** Use buffer week 25

---

### If Integration Issues (Week 18-20)

**Plan B:**
- Extend integration by 1 week
- Reduce field testing to 2 weeks
- Focus on critical scenarios only

**Impact:** Less field testing coverage
**Recovery:** Extensive testing in buffer period

---

### If Critical Bug Found in Week 22-24

**Plan B:**
- Use entire 4-week buffer for fixes
- Delay delivery to Jul 19
- Prioritize P0/P1 bugs only

**Impact:** Reduced polish, some P2 bugs deferred
**Recovery:** Post-deployment patch

---

**Document End**
