# Agile Development Roadmap - Wheelchair Transport Robot

**Project:** Outdoor-First Wheelchair Transport Robot with Swerve Drive
**Document Type:** Agile Development Roadmap
**Status:** ⚠️ OUTDATED - See PROJECT_TIMELINE_WITH_AI.md for current 32-week solo developer timeline
**Date:** December 15, 2025
**Development Methodology:** Solo developer with AI assistance (NOT multi-team Scrum)
**Total Duration:** 32 weeks (solo developer, AI-accelerated)
**Reference Architecture:** ParcelPal-inspired patterns (0% code reuse, 60% architectural patterns)

---

## ⚠️ IMPORTANT NOTE

**This document describes an outdated multi-team approach.**

**For the CURRENT timeline, see:**
- **PROJECT_TIMELINE_WITH_AI.md** - 32-week solo developer timeline with AI assistance
- **TEAM_STRUCTURE.md** - Solo developer structure (dev + reviewer + tester = you)

**Key Differences:**
- **Team:** 1 solo developer (NOT 12 engineers)
- **Duration:** 32 weeks (NOT 40 weeks)
- **Workflow:** Build → Test → Fix → Deploy (NOT Scrum with sprint planning/reviews)
- **Approvals:** None (self-review only)
- **Code Reuse:** 0% from ParcelPal (NOT 60-70%)
- **Stack:** Standard ROS 2 + Nav2 + PCL (NOT Autoware)

---

## Document Purpose

This document defines the complete agile development roadmap for the wheelchair transport robot system, organized into 5 phases and 20 two-week sprints. It provides sprint-level task breakdown, dependencies, resource allocation, testing strategy, and risk management approach to enable parallel development across subsystems.

---

## 1. Development Methodology

### 1.1 Agile Framework

**Sprint Structure:**
- **Duration:** 2 weeks (10 working days)
- **Sprint Planning:** Day 1 (4 hours)
- **Daily Standup:** 15 minutes daily
- **Sprint Review:** Last day (2 hours)
- **Sprint Retrospective:** Last day (1 hour)

**Team Structure:**
- **System Architect:** 1 person (full-time)
- **Navigation Team:** 2 engineers (Autoware + Nav2)
- **Swerve Drive Team:** 2 engineers (controller + hardware)
- **Docking Team:** 2 engineers (vision + control)
- **UI/eHMI Team:** 2 engineers (React + ESP32)
- **Safety Team:** 1 engineer (cross-functional)
- **QA/Testing:** 2 engineers (integration + field testing)
- **Total:** 12 engineers

**Definition of Done:**
- [ ] Code reviewed (minimum 2 reviewers)
- [ ] Unit tests pass (80%+ coverage)
- [ ] Integration tests pass
- [ ] Documentation updated
- [ ] Demo-able to stakeholders
- [ ] No critical bugs

---

## 2. Phase Overview (5 Phases, 20 Sprints)

### Phase 1: Foundation (Sprints 1-4, Weeks 1-8)
**Goal:** Establish core infrastructure and basic navigation
**Key Deliverable:** Robot navigates autonomously on saved map using NDT localization

### Phase 2: Docking System (Sprints 5-8, Weeks 9-16)
**Goal:** Implement precision wheelchair docking with ArUco markers
**Key Deliverable:** >90% docking success rate in controlled outdoor environment

### Phase 3: Safety & Transport (Sprints 9-12, Weeks 17-24)
**Goal:** Passenger safety monitoring and transport mode
**Key Deliverable:** Safe transport with passenger onboard, emergency stop validated

### Phase 4: UI/UX & Polish (Sprints 13-16, Weeks 25-32)
**Goal:** User interface, eHMI, multi-language support
**Key Deliverable:** Complete user experience for wheelchair booking and transport

### Phase 5: Field Testing (Sprints 17-20, Weeks 33-40)
**Goal:** Outdoor/indoor validation, performance optimization
**Key Deliverable:** Production-ready system meeting all critical requirements

---

## 3. Detailed Sprint Breakdown

### **PHASE 1: FOUNDATION (Sprints 1-4)**

#### **Sprint 1: Development Environment Setup**
**Weeks 1-2 | Priority: Critical**

**Goals:**
- Set up development infrastructure
- Install ROS 2 Humble + Autoware
- Configure hardware platform (GMKtec Nucbox K6)

**Tasks:**
| Task | Owner | Story Points | Dependencies |
|------|-------|--------------|--------------|
| Install Ubuntu 22.04 on GMKtec Nucbox K6 | System Arch | 2 | Hardware procurement |
| Install ROS 2 Humble + dependencies | Navigation | 3 | OS installation |
| Install Autoware (tier4_autoware) | Navigation | 5 | ROS 2 installation |
| Set up version control (Git) and CI/CD | System Arch | 3 | - |
| Install development tools (VS Code, CLion) | All teams | 1 | - |
| Configure network (ROS_DOMAIN_ID) | System Arch | 2 | - |
| Set up documentation repository | System Arch | 1 | - |
| Create ROS 2 workspace structure | All teams | 2 | ROS 2 installation |

**Deliverables:**
- ✅ Functional ROS 2 Humble installation
- ✅ Autoware running (basic launch files)
- ✅ Git repository with CI/CD pipeline
- ✅ Development environment guide

**Acceptance Criteria:**
- All team members can build Autoware from source
- Basic Autoware launch file runs without errors
- CI/CD pipeline runs on every commit

**Risks:**
- ⚠️ Autoware dependencies may conflict → Mitigation: Use Docker container
- ⚠️ Hardware procurement delays → Mitigation: Start with simulation

---

#### **Sprint 2: Manual SLAM Mapping**
**Weeks 3-4 | Priority: Critical**

**Goals:**
- Create saved map of test environment
- Configure NDT scan matcher localization
- Validate map quality

**Tasks:**
| Task | Owner | Story Points | Dependencies |
|------|-------|--------------|--------------|
| Integrate 3D LiDAR driver (model TBD) | Navigation | 5 | LiDAR procurement |
| Configure SLAM Toolbox for manual mapping | Navigation | 5 | LiDAR integration |
| Perform manual mapping of test environment | Navigation | 8 | SLAM configuration |
| Verify loop closure on saved map | Navigation | 3 | Mapping complete |
| Configure NDT scan matcher parameters | Navigation | 5 | Saved map |
| Test localization accuracy (±10cm target) | QA | 5 | NDT configuration |
| Create mapping procedure document | Navigation | 2 | Mapping complete |

**Deliverables:**
- ✅ Saved map of test environment (PCD + YAML)
- ✅ NDT localization achieving ±10cm accuracy
- ✅ Mapping procedure documentation

**Acceptance Criteria:**
- Localization accuracy ±10cm over 100m travel
- No localization drift during 500m navigation
- Map covers entire test environment (500m range)

**Risks:**
- ⚠️ LiDAR model selection delays → Mitigation: Use placeholder sensor in simulation
- ⚠️ Poor loop closure quality → Mitigation: Re-map with better sensor coverage

---

#### **Sprint 3: Basic Swerve Drive Controller (Part 1)**
**Weeks 5-6 | Priority: Critical**

**Goals:**
- Implement inverse kinematics for swerve drive
- Create ROS 2 controller plugin skeleton
- Test with simulated hardware

**Tasks:**
| Task | Owner | Story Points | Dependencies |
|------|-------|--------------|--------------|
| Design swerve drive inverse kinematics | Swerve Drive | 5 | - |
| Implement IK equations in C++17 | Swerve Drive | 8 | Design complete |
| Create ROS 2 controller plugin (Nav2) | Swerve Drive | 8 | IK implementation |
| Integrate wheel encoder feedback | Swerve Drive | 5 | Motor drivers |
| Create simulation model (Gazebo/Isaac Sim) | Swerve Drive | 5 | - |
| Unit test IK calculations | Swerve Drive | 3 | IK implementation |
| Test controller with cmd_vel inputs | QA | 3 | Controller plugin |

**Deliverables:**
- ✅ Swerve drive inverse kinematics library
- ✅ ROS 2 controller plugin (basic functionality)
- ✅ Simulation model for testing

**Acceptance Criteria:**
- IK calculations match expected wheel speeds/angles (±1%)
- Controller responds to cmd_vel inputs in simulation
- Unit tests achieve 80%+ coverage

**Risks:**
- ⚠️ Controller complexity underestimated → Mitigation: Simplify to holonomic drive first
- ⚠️ Motor driver integration delays → Mitigation: Test with simulated motors

---

#### **Sprint 4: System Integration Skeleton**
**Weeks 7-8 | Priority: High**

**Goals:**
- Create Wheelchair Master state machine skeleton
- Integrate Autoware + Swerve Drive
- Basic navigation (without docking)

**Tasks:**
| Task | Owner | Story Points | Dependencies |
|------|-------|--------------|--------------|
| Design Wheelchair Master state machine | System Arch | 5 | - |
| Implement state machine in Python 3.10 | System Arch | 8 | Design complete |
| Integrate Nav2 with swerve drive controller | Navigation + Swerve | 8 | Controller ready |
| Create basic launch files (bringup) | System Arch | 3 | - |
| Test waypoint navigation (A→B) | QA | 5 | Integration complete |
| Set up logging and diagnostics | System Arch | 3 | - |
| Create system startup procedure | System Arch | 2 | Launch files ready |

**Deliverables:**
- ✅ Wheelchair Master state machine (IDLE, APPROACHING, TRANSPORTING states)
- ✅ Robot navigates autonomously A→B using saved map
- ✅ System bringup launch file

**Acceptance Criteria:**
- Robot navigates 100m without intervention
- State machine transitions correctly (IDLE → APPROACHING → TRANSPORTING → IDLE)
- Localization accuracy maintained during navigation

**Risks:**
- ⚠️ Nav2 + swerve drive integration issues → Mitigation: Use differential drive temporarily

**Phase 1 Milestone:**
- ✅ Robot navigates autonomously on saved map
- ✅ NDT localization working (±10cm accuracy)
- ✅ Basic swerve drive control functional
- ✅ Development infrastructure complete

---

### **PHASE 2: DOCKING SYSTEM (Sprints 5-8)**

#### **Sprint 5: ArUco Marker Detection**
**Weeks 9-10 | Priority: Critical**

**Goals:**
- Integrate 2× RGB cameras
- Implement ArUco marker detection
- Calibrate cameras for outdoor use

**Tasks:**
| Task | Owner | Story Points | Dependencies |
|------|-------|--------------|--------------|
| Integrate 2× RGB cameras (model TBD) | Docking | 5 | Camera procurement |
| Calibrate cameras (intrinsics + extrinsics) | Docking | 5 | Camera integration |
| Implement ArUco detection (OpenCV) | Docking | 8 | Camera calibration |
| Test detection range (0.5-5m target) | QA | 3 | Detection implemented |
| Handle outdoor lighting variations | Docking | 5 | Detection working |
| Publish detected marker poses to TF2 | Docking | 3 | Detection working |
| Create camera launch files | Docking | 2 | - |

**Deliverables:**
- ✅ ArUco marker detection at 0.5-5m range
- ✅ Marker pose published to /aruco_poses topic
- ✅ Camera calibration files

**Acceptance Criteria:**
- Detection rate >10 Hz at 0.5-5m range
- Pose estimation error <5mm at 1m distance
- Detection works in daylight and nighttime conditions

**Risks:**
- ⚠️ Outdoor lighting affects detection → Mitigation: Use adaptive thresholding
- ⚠️ Camera resolution insufficient → Mitigation: Upgrade to higher resolution

---

#### **Sprint 6: Visual Servoing Controller**
**Weeks 11-12 | Priority: Critical**

**Goals:**
- Implement visual servoing for precision docking
- Integrate with swerve drive controller
- Test docking precision in lab

**Tasks:**
| Task | Owner | Story Points | Dependencies |
|------|-------|--------------|--------------|
| Design visual servoing controller (IBVS) | Docking | 5 | - |
| Implement IBVS in C++17 | Docking | 8 | Design complete |
| Integrate with swerve drive controller | Docking + Swerve | 8 | IBVS + controller ready |
| Create docking approach sequence | Docking | 5 | IBVS working |
| Test lab docking (indoor, flat surface) | QA | 5 | Docking sequence ready |
| Measure docking precision (±1mm target) | QA | 3 | Lab testing complete |
| Tune controller parameters (Kp, Kd) | Docking | 3 | Lab testing complete |

**Deliverables:**
- ✅ Visual servoing controller (IBVS)
- ✅ Lab docking precision ±1mm (indoor)
- ✅ Docking controller launch file

**Acceptance Criteria:**
- Docking precision ±1mm on flat surface (indoor)
- Docking success rate >95% in lab (100 trials)
- Docking time <60 seconds (approach to attached)

**Risks:**
- ⚠️ IBVS convergence issues → Mitigation: Add pose-based visual servoing (PBVS) fallback
- ⚠️ Marker occlusion during approach → Mitigation: Use multiple markers

---

#### **Sprint 7: Outdoor Docking Validation**
**Weeks 13-14 | Priority: Critical**

**Goals:**
- Test docking outdoors (rough terrain, lighting)
- Achieve ±2-5mm precision target
- Handle outdoor edge cases

**Tasks:**
| Task | Owner | Story Points | Dependencies |
|------|-------|--------------|--------------|
| Test outdoor docking on grass | QA | 5 | Lab docking working |
| Test outdoor docking on asphalt | QA | 5 | Lab docking working |
| Test docking with slope (0-5°) | QA | 5 | Outdoor testing started |
| Measure outdoor precision (±2-5mm target) | QA | 3 | Outdoor testing |
| Handle marker detection failures | Docking | 5 | Outdoor testing |
| Implement retry logic (3 attempts) | Docking | 3 | Failure handling |
| Test docking in various weather (dry, light rain) | QA | 5 | Outdoor testing |

**Deliverables:**
- ✅ Outdoor docking precision ±2-5mm
- ✅ Docking success rate >90% (outdoor)
- ✅ Retry logic (3 attempts)

**Acceptance Criteria:**
- Docking precision ±2-5mm on rough terrain
- Success rate >90% in outdoor conditions (100 trials)
- Handles slopes 0-5° without failure

**Risks:**
- ⚠️ Precision degrades outdoors → Mitigation: Use larger markers or stereo cameras
- ⚠️ Weather affects marker detection → Mitigation: Add waterproof marker coating

---

#### **Sprint 8: Mechanical Coupling & Confirmation**
**Weeks 15-16 | Priority: Critical**

**Goals:**
- Design mechanical coupling mechanism
- Implement attachment confirmation sensors
- Integrate coupling with state machine

**Tasks:**
| Task | Owner | Story Points | Dependencies |
|------|-------|--------------|--------------|
| Design mechanical coupling (TBD mechanism) | Hardware | 8 | - |
| Fabricate coupling prototype | Hardware | 8 | Design complete |
| Integrate coupling sensors (contact, force) | Docking | 5 | Coupling ready |
| Implement attachment confirmation logic | Docking | 3 | Sensors integrated |
| Test coupling reliability (100 cycles) | QA | 5 | Coupling integrated |
| Integrate coupling with state machine | System Arch | 3 | Coupling working |
| Create undocking sequence | Docking | 3 | Coupling working |

**Deliverables:**
- ✅ Mechanical coupling mechanism
- ✅ Attachment confirmation sensors
- ✅ Docking + undocking sequences

**Acceptance Criteria:**
- Coupling reliability >99% (100 cycles)
- Attachment confirmed before state transition
- Undocking success rate >99%

**Risks:**
- ⚠️ Coupling mechanism design delays → Mitigation: Use simple magnetic coupling prototype
- ⚠️ Sensor false positives → Mitigation: Use redundant sensors

**Phase 2 Milestone:**
- ✅ Docking success rate >90% (outdoor)
- ✅ Docking precision ±2-5mm (outdoor)
- ✅ Mechanical coupling functional
- ✅ Complete docking/undocking sequence

---

### **PHASE 3: SAFETY & TRANSPORT (Sprints 9-12)**

#### **Sprint 9: Passenger Safety Monitoring**
**Weeks 17-18 | Priority: Critical**

**Goals:**
- Implement occupant detection sensors
- Monitor passenger presence during transport
- Integrate with safety system

**Tasks:**
| Task | Owner | Story Points | Dependencies |
|------|-------|--------------|--------------|
| Integrate occupant detection sensor (TBD) | Safety | 5 | Sensor procurement |
| Implement passenger presence monitoring | Safety | 5 | Sensor integrated |
| Create safety monitoring node (ROS 2) | Safety | 5 | - |
| Test occupant detection accuracy | QA | 3 | Monitoring implemented |
| Integrate with state machine (WHEELCHAIR_ATTACHED state) | System Arch | 3 | Monitoring working |
| Add audible warnings (wheelchair attached) | Safety | 2 | - |
| Test false positive/negative rates | QA | 3 | Integration complete |

**Deliverables:**
- ✅ Occupant detection system
- ✅ Safety monitoring node
- ✅ Audible warnings

**Acceptance Criteria:**
- Occupant detection accuracy >99%
- False positive rate <1%
- Warnings audible at 5m distance

**Risks:**
- ⚠️ Sensor unreliable outdoors → Mitigation: Use redundant sensors (weight + vision)

---

#### **Sprint 10: Speed Limiting & Comfort**
**Weeks 19-20 | Priority: Critical**

**Goals:**
- Implement speed limiting with passenger
- Limit acceleration for comfort
- Test ride quality

**Tasks:**
| Task | Owner | Story Points | Dependencies |
|------|-------|--------------|--------------|
| Implement speed limiter (1.0 m/s max with passenger) | Navigation | 3 | - |
| Implement acceleration limiter (0.5 m/s² max) | Navigation | 3 | - |
| Integrate limiters with state machine | System Arch | 3 | Limiters ready |
| Test ride comfort (subjective evaluation) | QA | 5 | Limiters integrated |
| Tune velocity profile (jerk minimization) | Navigation | 5 | Comfort testing |
| Test on various terrains (grass, asphalt, concrete) | QA | 5 | Tuning complete |
| Measure passenger comfort metrics | QA | 3 | Terrain testing |

**Deliverables:**
- ✅ Speed limiter (1.0 m/s with passenger)
- ✅ Acceleration limiter (0.5 m/s²)
- ✅ Comfort-optimized velocity profile

**Acceptance Criteria:**
- Max speed 1.0 m/s with passenger (enforced)
- Max acceleration 0.5 m/s² (enforced)
- Passenger comfort rating >4/5

**Risks:**
- ⚠️ Comfort subjective and hard to measure → Mitigation: Use accelerometer data

---

#### **Sprint 11: Emergency Stop System**
**Weeks 21-22 | Priority: Critical**

**Goals:**
- Implement hardware emergency stop button
- Software emergency stop (ROS 2 topic)
- Test emergency stop response time

**Tasks:**
| Task | Owner | Story Points | Dependencies |
|------|-------|--------------|--------------|
| Integrate hardware emergency stop button | Safety | 5 | Hardware procurement |
| Implement emergency stop logic (SIL 2) | Safety | 8 | HW button integrated |
| Create /emergency_stop topic subscriber | Safety | 3 | - |
| Test emergency stop response time (<100ms) | QA | 5 | E-stop implemented |
| Implement graceful stop (brake within 1.5m) | Safety | 5 | E-stop working |
| Test e-stop on slopes (up to 10°) | QA | 5 | Graceful stop ready |
| Add e-stop status to UI/eHMI | UI | 3 | E-stop working |

**Deliverables:**
- ✅ Hardware + software emergency stop
- ✅ Emergency stop response <100ms
- ✅ Graceful stop within 1.5m

**Acceptance Criteria:**
- E-stop response time <100ms (100% of trials)
- Braking distance <1.5m at 1.0 m/s (100% of trials)
- E-stop functional on 10° slopes

**Risks:**
- ⚠️ Hardware button reliability → Mitigation: Use industrial-grade e-stop button

---

#### **Sprint 12: Safety System Integration**
**Weeks 23-24 | Priority: Critical**

**Goals:**
- Integrate all safety layers (4 layers)
- Implement tilt monitoring and cutoff
- Test safety system end-to-end

**Tasks:**
| Task | Owner | Story Points | Dependencies |
|------|-------|--------------|--------------|
| Integrate IMU for tilt monitoring | Safety | 5 | IMU procurement |
| Implement tilt cutoff (15° unexpected tilt) | Safety | 5 | IMU integrated |
| Implement slope detection (no pickup >5°) | Safety | 3 | IMU working |
| Create safety system architecture diagram | Safety | 2 | All layers ready |
| Integrate geofencing (keep-out zones) | Safety | 5 | - |
| Test safety system failure modes (FMEA) | QA | 8 | Integration complete |
| Validate ISO 13482 compliance (initial) | Safety | 5 | Testing complete |

**Deliverables:**
- ✅ 4-layer safety system integrated
- ✅ Tilt monitoring and cutoff
- ✅ Safety system validation report

**Acceptance Criteria:**
- No pickup/dropoff on slopes >5° (enforced)
- Tilt cutoff triggers at 15° (100% of trials)
- Zero safety incidents in 100 test missions

**Risks:**
- ⚠️ ISO 13482 compliance gaps → Mitigation: Engage safety certification consultant

**Phase 3 Milestone:**
- ✅ Passenger safety monitoring functional
- ✅ Emergency stop validated (<100ms response)
- ✅ Safety system integrated (4 layers)
- ✅ Zero safety incidents in testing

---

### **PHASE 4: UI/UX & POLISH (Sprints 13-16)**

#### **Sprint 13: Touch Screen User Interface**
**Weeks 25-26 | Priority: High**

**Goals:**
- Create wheelchair booking UI (React + Next.js)
- Multi-language support (EN, JP, ZH)
- Real-time robot status display

**Tasks:**
| Task | Owner | Story Points | Dependencies |
|------|-------|--------------|--------------|
| Design UI mockups (Figma) | UI | 5 | - |
| Set up React 18 + Next.js 14 project | UI | 3 | - |
| Implement booking flow (pickup/dropoff selection) | UI | 8 | Mockups ready |
| Integrate multi-language support (i18n) | UI | 5 | Booking flow ready |
| Display real-time robot status (state, location) | UI | 5 | ROS 2 bridge |
| Create WebSocket bridge (ROS 2 ↔ React) | UI | 5 | - |
| Test UI on touch screen hardware | QA | 3 | UI complete |

**Deliverables:**
- ✅ Wheelchair booking UI (React + Next.js)
- ✅ Multi-language support (EN, JP, ZH)
- ✅ Real-time status display

**Acceptance Criteria:**
- Booking flow completes in <30 seconds
- UI responsive on touch screen (<100ms touch response)
- Language switching works correctly (100% translations)

**Risks:**
- ⚠️ WebSocket latency → Mitigation: Use ROS 2 web bridge (rosbridge)

---

#### **Sprint 14: eHMI Implementation**
**Weeks 27-28 | Priority: High**

**Goals:**
- Implement ESP32-S3 eHMI controller
- Add wheelchair-specific states (21-28)
- Test LED + audio output

**Tasks:**
| Task | Owner | Story Points | Dependencies |
|------|-------|--------------|--------------|
| Adapt eHMI firmware (from ParcelPal) | UI | 5 | ESP32-S3 hardware |
| Add wheelchair states (21-28) | UI | 3 | Firmware adapted |
| Test LED strips (FastLED) | UI | 2 | Firmware ready |
| Test LED matrix (HUB75) | UI | 2 | Firmware ready |
| Test audio output (I2S + SD card) | UI | 3 | Firmware ready |
| Create audio files (EN, JP, ZH) | UI | 5 | - |
| Integrate eHMI with ROS 2 (serial bridge) | UI | 5 | Firmware ready |

**Deliverables:**
- ✅ ESP32-S3 eHMI controller
- ✅ Wheelchair states (21-28) implemented
- ✅ Audio files (3 languages)

**Acceptance Criteria:**
- eHMI responds to state changes <500ms
- LED patterns visible at 10m distance (daylight)
- Audio audible at 5m distance

**Risks:**
- ⚠️ Audio file quality → Mitigation: Professional voice recording

---

#### **Sprint 15: Multi-Stop Routing**
**Weeks 29-30 | Priority: Medium**

**Goals:**
- Implement multi-waypoint navigation (up to 10 stops)
- Queue management for multiple passengers
- Battery monitoring and auto-return

**Tasks:**
| Task | Owner | Story Points | Dependencies |
|------|-------|--------------|--------------|
| Implement waypoint queue manager | Navigation | 5 | - |
| Integrate Nav2 waypoint follower | Navigation | 5 | Queue manager ready |
| Test 10-waypoint navigation | QA | 5 | Integration complete |
| Implement battery monitoring | System Arch | 3 | - |
| Auto-return to charging station (<30%) | System Arch | 5 | Battery monitoring |
| Create mission scheduling logic | System Arch | 5 | Queue manager ready |
| Test multi-stop shuttle service | QA | 5 | All features ready |

**Deliverables:**
- ✅ Multi-waypoint navigation (up to 10 stops)
- ✅ Battery monitoring and auto-return
- ✅ Mission scheduling system

**Acceptance Criteria:**
- Completes 10-waypoint mission without intervention
- Auto-return triggers at 30% battery
- Queue management handles concurrent bookings

**Risks:**
- ⚠️ Nav2 waypoint follower issues → Mitigation: Implement custom waypoint manager

---

#### **Sprint 16: System Polish & UX**
**Weeks 31-32 | Priority: Medium**

**Goals:**
- Polish user experience (UI + eHMI)
- Add operator dashboard
- Performance optimization

**Tasks:**
| Task | Owner | Story Points | Dependencies |
|------|-------|--------------|--------------|
| Polish UI animations and transitions | UI | 5 | UI complete |
| Add ETA display during transport | UI | 3 | Navigation ready |
| Create operator dashboard (mission monitoring) | UI | 8 | - |
| Optimize CPU usage (<70%) | System Arch | 5 | - |
| Optimize memory usage (<20GB) | System Arch | 3 | - |
| Add diagnostic logs and metrics | System Arch | 3 | - |
| User acceptance testing (10 participants) | QA | 8 | All features ready |

**Deliverables:**
- ✅ Polished UI/UX
- ✅ Operator dashboard
- ✅ Performance optimizations

**Acceptance Criteria:**
- User satisfaction >4.5/5 (10 participants)
- CPU usage <70% during normal operation
- Memory usage <20GB

**Risks:**
- ⚠️ Performance bottlenecks → Mitigation: Profile and optimize critical paths

**Phase 4 Milestone:**
- ✅ Touch screen UI complete (multi-language)
- ✅ eHMI functional (LED + audio)
- ✅ Multi-stop routing working
- ✅ User satisfaction >4.5/5

---

### **PHASE 5: FIELD TESTING (Sprints 17-20)**

#### **Sprint 17: Outdoor Field Testing**
**Weeks 33-34 | Priority: Critical**

**Goals:**
- Test on campus pathways (outdoor)
- Validate 500m-1km range
- Test on slopes up to 10°

**Tasks:**
| Task | Owner | Story Points | Dependencies |
|------|-------|--------------|--------------|
| Map campus pathways (manual SLAM) | Navigation | 5 | Campus access |
| Test 500m navigation | QA | 5 | Map ready |
| Test 1km navigation | QA | 5 | 500m successful |
| Test on 5° slopes (with passenger) | QA | 5 | Navigation working |
| Test on 10° slopes (no passenger) | QA | 5 | 5° successful |
| Measure battery life (4+ hour target) | QA | 5 | All testing |
| Log and analyze failures | QA | 3 | Testing complete |

**Deliverables:**
- ✅ Campus pathway map
- ✅ 500m-1km navigation validated
- ✅ Slope performance validated

**Acceptance Criteria:**
- 500m navigation success rate >98%
- 1km navigation success rate >95%
- Battery life >4 hours with passenger

**Risks:**
- ⚠️ Weather conditions → Mitigation: Test in various weather (dry, light rain)

---

#### **Sprint 18: Indoor Compatibility Testing**
**Weeks 35-36 | Priority: High**

**Goals:**
- Test indoor environments (corridors, smooth floors)
- Validate ±1mm docking target
- Test indoor/outdoor transitions

**Tasks:**
| Task | Owner | Story Points | Dependencies |
|------|-------|--------------|--------------|
| Map indoor environment (hospital/mall) | Navigation | 5 | Indoor access |
| Test indoor navigation | QA | 5 | Map ready |
| Test indoor docking (±1mm target) | QA | 5 | Navigation working |
| Test indoor/outdoor transitions | QA | 5 | Both maps ready |
| Compare indoor vs outdoor performance | QA | 3 | All testing |
| Optimize for indoor (speed, precision) | Navigation | 5 | Testing complete |
| Create deployment guide (indoor vs outdoor) | System Arch | 3 | Testing complete |

**Deliverables:**
- ✅ Indoor environment map
- ✅ Indoor docking ±1mm validated
- ✅ Indoor/outdoor transition validated

**Acceptance Criteria:**
- Indoor docking precision ±1mm
- Smooth indoor/outdoor transitions (no intervention)
- Indoor speed increased to 1.5 m/s (safe)

**Risks:**
- ⚠️ Indoor localization drift → Mitigation: Use higher NDT resolution

---

#### **Sprint 19: Environmental Testing**
**Weeks 37-38 | Priority: High**

**Goals:**
- Test in light rain (IP54+ validation)
- Test in various lighting (day/night)
- Test on various terrains (grass, asphalt, concrete)

**Tasks:**
| Task | Owner | Story Points | Dependencies |
|------|-------|--------------|--------------|
| Test in light rain | QA | 5 | Weatherproofing ready |
| Test at night (low light) | QA | 5 | - |
| Test on grass | QA | 3 | - |
| Test on asphalt | QA | 3 | - |
| Test on concrete | QA | 3 | - |
| Test on tile (indoor) | QA | 3 | - |
| Log environmental failures | QA | 3 | All testing |
| Create operating condition limits document | System Arch | 2 | Testing complete |

**Deliverables:**
- ✅ IP54+ weatherproofing validated
- ✅ Day/night operation validated
- ✅ Multi-terrain performance validated

**Acceptance Criteria:**
- No failures in light rain (IP54+ validated)
- Night operation success rate >95%
- Multi-terrain success rate >90%

**Risks:**
- ⚠️ Heavy rain damage → Mitigation: Enforce operational limits (light rain only)

---

#### **Sprint 20: Production Readiness**
**Weeks 39-40 | Priority: Critical**

**Goals:**
- Complete 1000 mission endurance test
- Final safety certification
- Production deployment preparation

**Tasks:**
| Task | Owner | Story Points | Dependencies |
|------|-------|--------------|--------------|
| Execute 1000 mission endurance test | QA | 13 | All features ready |
| Analyze failure modes (FMEA) | Safety | 5 | Endurance test complete |
| Final ISO 13482 compliance review | Safety | 5 | FMEA complete |
| Create maintenance procedures | System Arch | 3 | Testing complete |
| Create operator training materials | System Arch | 5 | - |
| Package deployment artifacts | System Arch | 3 | - |
| Final stakeholder demo and signoff | All | 2 | All complete |

**Deliverables:**
- ✅ 1000 mission endurance test report
- ✅ ISO 13482 compliance certification
- ✅ Production deployment package

**Acceptance Criteria:**
- Zero safety incidents in 1000 missions
- >95% docking success rate (outdoor)
- >98% uptime (excluding maintenance)
- Stakeholder signoff obtained

**Risks:**
- ⚠️ Certification delays → Mitigation: Engage certifier early (Sprint 12)

**Phase 5 Milestone:**
- ✅ 1000 missions completed (zero safety incidents)
- ✅ Outdoor + indoor validation complete
- ✅ ISO 13482 certified
- ✅ Production ready

---

## 4. Resource Allocation

### 4.1 Team Allocation by Phase

| Phase | Navigation | Swerve | Docking | UI/eHMI | Safety | QA | Total |
|-------|-----------|--------|---------|---------|--------|----|----|
| **Phase 1** | 2 | 2 | 0 | 0 | 0 | 2 | 6 |
| **Phase 2** | 1 | 1 | 2 | 0 | 0 | 2 | 6 |
| **Phase 3** | 1 | 0 | 0 | 0 | 1 | 2 | 4 |
| **Phase 4** | 1 | 0 | 0 | 2 | 0 | 2 | 5 |
| **Phase 5** | 1 | 1 | 1 | 1 | 1 | 2 | 7 |

**Note:** System Architect allocated full-time across all phases (not shown in table).

### 4.2 Hardware Procurement Timeline

| Item | Sprint | Lead Time | Priority |
|------|--------|-----------|----------|
| GMKtec Nucbox K6 | Sprint 1 | 2 weeks | Critical |
| 3D LiDAR (outdoor-grade) | Sprint 2 | 4 weeks | Critical |
| 2× RGB cameras | Sprint 5 | 2 weeks | Critical |
| 9-DOF IMU | Sprint 4 | 1 week | High |
| ESP32-S3 + eHMI hardware | Sprint 14 | 2 weeks | High |
| Emergency stop button | Sprint 11 | 1 week | Critical |
| Occupant detection sensor | Sprint 9 | 3 weeks | Critical |
| Swerve drive modules (4×) | Sprint 3 | 6 weeks | Critical |
| Battery pack | Sprint 1 | 4 weeks | Critical |
| Touch screen display | Sprint 13 | 2 weeks | High |

---

## 5. Dependencies & Critical Path

### 5.1 Inter-Sprint Dependencies

```
Sprint 1 (Dev Setup) → Sprint 2 (Mapping)
                    ↓
Sprint 2 (Mapping) → Sprint 4 (Integration)
                    ↓
Sprint 3 (Swerve) → Sprint 4 (Integration) → Sprint 5-8 (Docking)
                                            ↓
                                    Sprint 9-12 (Safety)
                                            ↓
                                    Sprint 13-16 (UI/UX)
                                            ↓
                                    Sprint 17-20 (Field Testing)
```

**Critical Path:**
1. Dev Setup (Sprint 1)
2. Mapping (Sprint 2)
3. Swerve Controller (Sprint 3)
4. Integration (Sprint 4)
5. ArUco Detection (Sprint 5)
6. Visual Servoing (Sprint 6)
7. Outdoor Docking (Sprint 7)
8. Mechanical Coupling (Sprint 8)
9. Safety System (Sprints 9-12)
10. Field Testing (Sprints 17-20)

**Critical Path Duration:** 40 weeks (all sprints on critical path)

### 5.2 Parallel Development Opportunities

**Sprints 5-8 (Phase 2):**
- Docking team: ArUco + visual servoing
- Navigation team: Path planning optimization (parallel work)

**Sprints 9-12 (Phase 3):**
- Safety team: Safety system
- UI team: Start UI design (Sprint 13 prep)

**Sprints 13-16 (Phase 4):**
- UI team: Touch screen UI
- eHMI team: ESP32-S3 firmware (parallel)

---

## 6. Risk Management

### 6.1 Technical Risks

| Risk | Probability | Impact | Mitigation | Owner |
|------|------------|--------|------------|-------|
| **Outdoor docking precision <2-5mm** | Medium | High | Use stereo cameras or larger markers | Docking |
| **NDT localization drift >10cm** | Low | High | Re-tune NDT parameters, consider multi-sensor fusion | Navigation |
| **Swerve drive control instability** | Medium | High | Start with holonomic drive, add swerve incrementally | Swerve |
| **Battery life <4 hours** | Medium | Medium | Optimize power consumption, increase battery capacity | System Arch |
| **3D LiDAR procurement delays** | High | High | Start with 2D LiDAR in simulation | Navigation |
| **ISO 13482 certification delays** | Medium | High | Engage certifier early (Sprint 12) | Safety |
| **Weather affects sensors** | Low | Medium | Enforce operational limits (light rain only) | Safety |

### 6.2 Schedule Risks

| Risk | Probability | Impact | Mitigation | Owner |
|------|------------|--------|------------|-------|
| **Hardware procurement delays** | High | High | Order long-lead items early, use simulation | System Arch |
| **Integration issues between subsystems** | Medium | Medium | Weekly integration meetings, clear ROS 2 interfaces | System Arch |
| **Sprint scope creep** | Medium | Medium | Strict definition of done, prioritize backlog | Product Owner |
| **Team member unavailability** | Low | Medium | Cross-train team members, document thoroughly | System Arch |

### 6.3 Safety Risks

| Risk | Probability | Impact | Mitigation | Owner |
|------|------------|--------|------------|-------|
| **Passenger injury during transport** | Low | Critical | 4-layer safety system, emergency stop, speed limits | Safety |
| **Collision with pedestrians** | Medium | High | LiDAR obstacle detection, eHMI warnings | Safety |
| **Slope instability with load** | Medium | High | 10° design limit, tilt sensor cutoff, no pickup >5° | Safety |
| **Docking failure causes wheelchair tip** | Low | High | Verify attachment before movement, retry logic | Docking |

---

## 7. Testing Strategy

### 7.1 Testing Approach by Phase

**Phase 1 (Foundation):**
- Unit tests: IK calculations, state machine transitions
- Integration tests: Nav2 + swerve controller
- Field tests: 100m navigation, localization accuracy

**Phase 2 (Docking):**
- Unit tests: ArUco detection, IBVS controller
- Integration tests: Docking sequence end-to-end
- Field tests: 100 docking trials (outdoor), precision measurement

**Phase 3 (Safety):**
- Unit tests: Emergency stop response time, tilt monitoring
- Integration tests: 4-layer safety system
- Safety tests: FMEA, failure mode testing

**Phase 4 (UI/UX):**
- Unit tests: UI components, WebSocket bridge
- Integration tests: End-to-end booking flow
- User tests: 10 participants, satisfaction survey

**Phase 5 (Field Testing):**
- Endurance tests: 1000 missions
- Environmental tests: Light rain, day/night, multi-terrain
- Certification tests: ISO 13482 compliance

### 7.2 Test Coverage Targets

| Test Type | Target Coverage | Enforcement |
|-----------|----------------|-------------|
| Unit tests | 80%+ | CI/CD pipeline blocks merge if <80% |
| Integration tests | All subsystem interfaces | Required for sprint completion |
| Field tests | 100% critical requirements | Required for phase completion |
| Safety tests | 100% safety requirements | Required for certification |

### 7.3 Continuous Integration (CI/CD)

**CI Pipeline (runs on every commit):**
1. Build all ROS 2 packages (colcon build)
2. Run unit tests (colcon test)
3. Check code coverage (>80%)
4. Static analysis (cppcheck, pylint)
5. Documentation build (Doxygen)

**CD Pipeline (runs on sprint completion):**
1. Build deployment artifacts
2. Run integration tests
3. Deploy to staging environment
4. Run smoke tests
5. Deploy to production (manual approval)

---

## 8. Milestones & Deliverables

### 8.1 Phase Milestones

| Milestone | Sprint | Deliverable | Acceptance Criteria |
|-----------|--------|-------------|---------------------|
| **M1: Foundation Complete** | Sprint 4 | Autonomous navigation on saved map | >98% navigation success, ±10cm localization |
| **M2: Docking Complete** | Sprint 8 | Precision docking functional | >90% docking success, ±2-5mm precision |
| **M3: Safety Complete** | Sprint 12 | Safety system validated | Zero safety incidents in 100 missions |
| **M4: UI/UX Complete** | Sprint 16 | User interface deployed | >4.5/5 user satisfaction |
| **M5: Production Ready** | Sprint 20 | 1000 mission endurance test | ISO 13482 certified, >98% uptime |

### 8.2 Sprint Deliverables Summary

See individual sprint sections for detailed deliverables.

**Key Deliverables by Sprint:**
- Sprint 1: Development environment
- Sprint 2: Saved map + NDT localization
- Sprint 3: Swerve drive controller
- Sprint 4: System integration skeleton
- Sprint 5: ArUco detection
- Sprint 6: Visual servoing controller
- Sprint 7: Outdoor docking validation
- Sprint 8: Mechanical coupling
- Sprint 9: Passenger safety monitoring
- Sprint 10: Speed/acceleration limiting
- Sprint 11: Emergency stop system
- Sprint 12: Safety system integration
- Sprint 13: Touch screen UI
- Sprint 14: eHMI implementation
- Sprint 15: Multi-stop routing
- Sprint 16: System polish
- Sprint 17: Outdoor field testing
- Sprint 18: Indoor compatibility
- Sprint 19: Environmental testing
- Sprint 20: Production readiness

---

## 9. Integration Approach

### 9.1 Integration Strategy

**Bottom-Up Integration:**
1. **Layer 1:** Individual ROS 2 nodes (unit tested)
2. **Layer 2:** Subsystem integration (e.g., Docking = ArUco + Visual Servoing + Coupling)
3. **Layer 3:** Cross-subsystem integration (e.g., Wheelchair Master + Autoware Nav + Docking)
4. **Layer 4:** System-level integration (all subsystems)

**Integration Frequency:**
- Daily: Unit tests (CI pipeline)
- Weekly: Integration tests (end of sprint)
- Bi-weekly: System-level tests (sprint demo)

### 9.2 Integration Testing Schedule

| Sprint | Integration Test | Participants |
|--------|------------------|--------------|
| Sprint 4 | Nav2 + Swerve Drive | Navigation + Swerve teams |
| Sprint 6 | Docking subsystem (ArUco + IBVS) | Docking team |
| Sprint 8 | Docking + State Machine | Docking + System Arch |
| Sprint 12 | Safety + State Machine | Safety + System Arch |
| Sprint 14 | eHMI + State Machine | UI + System Arch |
| Sprint 16 | Full system integration | All teams |

---

## 10. Definition of Done (DoD)

### 10.1 Sprint-Level DoD

A sprint is considered "done" when:
- [ ] All planned tasks completed
- [ ] Code reviewed by ≥2 reviewers
- [ ] Unit tests pass (80%+ coverage)
- [ ] Integration tests pass
- [ ] Documentation updated (code comments + README)
- [ ] Sprint demo successful (stakeholder approval)
- [ ] No critical bugs (P0/P1 bugs resolved)
- [ ] Code merged to main branch

### 10.2 Phase-Level DoD

A phase is considered "done" when:
- [ ] All sprint-level DoDs met
- [ ] Phase milestone achieved (see Section 8.1)
- [ ] Field testing complete (acceptance criteria met)
- [ ] Phase retrospective conducted
- [ ] Stakeholder signoff obtained
- [ ] Documentation complete (requirements, design, test reports)

---

## 11. Communication & Reporting

### 11.1 Meetings

**Daily Standup (15 minutes):**
- What did I do yesterday?
- What will I do today?
- Any blockers?

**Sprint Planning (4 hours, Day 1):**
- Review backlog
- Estimate story points
- Commit to sprint goal

**Sprint Review (2 hours, Last Day):**
- Demo completed features
- Stakeholder feedback
- Accept/reject sprint deliverables

**Sprint Retrospective (1 hour, Last Day):**
- What went well?
- What could improve?
- Action items for next sprint

**Weekly Integration Meeting (1 hour):**
- Cross-team dependencies
- Integration issues
- ROS 2 interface changes

### 11.2 Reporting

**Sprint Burndown Chart:**
- Updated daily
- Tracks remaining story points

**Velocity Chart:**
- Tracks story points completed per sprint
- Used for future sprint planning

**Cumulative Flow Diagram:**
- Tracks tasks in Todo/In Progress/Done
- Identifies bottlenecks

**Risk Register:**
- Updated weekly
- Tracks risk probability, impact, mitigation status

---

## 12. Backlog Management

### 12.1 Product Backlog

See `USER_STORIES.md` for detailed feature descriptions from user perspective.

**Backlog Prioritization (MoSCoW):**
- **Must Have:** All critical requirements (47 requirements, 51%)
- **Should Have:** All high requirements (32 requirements, 35%)
- **Could Have:** All medium requirements (11 requirements, 12%)
- **Won't Have (this release):** Low requirements + future features (2 requirements, 2%)

### 12.2 Sprint Backlog

Sprint backlog created during sprint planning from product backlog.

**Story Points Scale (Fibonacci):**
- 1: Trivial (1-2 hours)
- 2: Easy (2-4 hours)
- 3: Medium (4-8 hours)
- 5: Complex (1-2 days)
- 8: Very complex (2-5 days)
- 13: Epic (requires breakdown)

**Velocity Target:**
- Average velocity: 40-50 story points per sprint (per team)
- Adjustment based on actual velocity after Sprint 4

---

## 13. Related Documents

**Requirements:**
- `01_REQUIREMENTS/SYSTEM_REQUIREMENTS.md` - Complete system requirements (92 requirements)
- `01_REQUIREMENTS/SWERVE_DRIVE_REQUIREMENTS.md` - Drive system detailed requirements

**Architecture:**
- `02_ARCHITECTURE/OVERALL_SYSTEM_ARCHITECTURE.md` - Detailed system architecture

**Development:**
- `05_DEVELOPMENT/USER_STORIES.md` - Feature descriptions from user perspective
- `05_DEVELOPMENT/SPRINT_PLANNING.md` - Sprint planning templates
- `05_DEVELOPMENT/BACKLOG.md` - Product backlog management

**Testing:**
- `06_TESTING/TEST_STRATEGY.md` - Testing approach and coverage targets
- `06_TESTING/FIELD_TEST_PROCEDURES.md` - Field testing procedures

**Reference:**
- `reference/PARCELPAL_EXPLORATION_SUMMARY.md` - ParcelPal architecture reuse guide
- `reference/question_answers.md` - Senior review clarifications

---

## 14. Success Metrics

### 14.1 Technical Metrics

| Metric | Target | Measurement Method |
|--------|--------|-------------------|
| Docking success rate (outdoor) | >95% | 100 trial field test |
| Localization accuracy | ±10cm over 500m | GPS ground truth comparison |
| Battery life | 4+ hours | Endurance test with passenger |
| Emergency stop response time | <100ms | High-speed camera measurement |
| CPU usage | <70% | ROS 2 resource monitoring |
| Uptime | >98% | 30-day field deployment |

### 14.2 Schedule Metrics

| Metric | Target | Measurement Method |
|--------|--------|-------------------|
| Sprint completion rate | >90% | Burndown chart |
| Velocity stability | ±10% variation | Velocity chart |
| Phase milestone on-time | 100% | Gantt chart |
| Total project duration | 40 weeks | Project schedule |

### 14.3 Quality Metrics

| Metric | Target | Measurement Method |
|--------|--------|-------------------|
| Code coverage | >80% | CI/CD pipeline |
| Critical bugs (P0/P1) | Zero at phase completion | Bug tracking system |
| User satisfaction | >4.5/5 | User acceptance testing survey |
| Safety incidents | Zero | Field testing log |

---

## 15. Open Questions & Assumptions

### 15.1 Open Questions (To Be Resolved)

| Question | Target Resolution | Owner |
|----------|------------------|-------|
| 3D LiDAR model selection (outdoor-grade) | Sprint 2 | Hardware team |
| Camera resolution (720p vs 1080p vs 4K) | Sprint 5 | Docking team |
| Mechanical coupling mechanism design | Sprint 8 | Hardware team |
| Battery capacity (Ah) and charging specs | Sprint 1 | Hardware team |
| Occupant detection sensor type | Sprint 9 | Safety team |
| Touch screen display size and model | Sprint 13 | UI team |

### 15.2 Assumptions

1. **Hardware Availability:** All critical hardware procurable within lead times
2. **Team Availability:** 12 engineers available full-time for 40 weeks
3. **Test Environment Access:** Campus pathways and indoor spaces available for testing
4. **ParcelPal Codebase Access:** Full access to ParcelPal source code for reuse
5. **Stakeholder Availability:** Stakeholders available for sprint reviews
6. **Weather Conditions:** Field testing weather conditions within operational limits (no heavy rain/snow during testing phases)
7. **Regulatory Approval:** ISO 13482 certification achievable within timeline

---

## 16. Glossary

| Term | Definition |
|------|------------|
| **Sprint** | 2-week development iteration |
| **Story Points** | Fibonacci scale (1, 2, 3, 5, 8, 13) for task estimation |
| **DoD** | Definition of Done (completion criteria) |
| **MoSCoW** | Prioritization method (Must/Should/Could/Won't Have) |
| **IBVS** | Image-Based Visual Servoing (docking controller) |
| **NDT** | Normal Distributions Transform (localization algorithm) |
| **FMEA** | Failure Mode and Effects Analysis (safety testing) |
| **SIL 2** | Safety Integrity Level 2 (IEC 61508) |
| **CI/CD** | Continuous Integration / Continuous Deployment |

---

**Document Status:** Planning v1.0
**Review Status:** Pending stakeholder approval
**Next Review:** After Phase 1 Sprint 1 completion
**Approvers:** Product Owner, System Architect, Development Leads
**Version History:**
- v1.0 (2025-12-15): Initial agile roadmap with 20-sprint plan

