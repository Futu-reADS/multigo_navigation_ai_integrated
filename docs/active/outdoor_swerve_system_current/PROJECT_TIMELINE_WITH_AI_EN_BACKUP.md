# Project Timeline - Outdoor Wheelchair Transport Robot (AI-Accelerated Solo Development)

**Document ID:** DOC-TIMELINE-001
**Version:** 2.0
**Date:** 2025-12-16
**Status:** Active Development Timeline

---

## Executive Summary

This document provides a realistic project timeline for developing the outdoor wheelchair transport robot system with **1 solo developer using heavy AI assistance** (Claude Code, GitHub Copilot, etc.).

**Key Assumptions:**
- **Team Size:** 1 solo developer (developer + reviewer + user - all roles)
- **AI Acceleration:** 2-3x productivity boost for code generation, testing, debugging, and documentation
- **ParcelPal Experience:** 0% code reuse, 60-70% architectural pattern reuse (proven navigation/perception approach, NOT Autoware code)
- **Hardware Team:** Parallel hardware work, field testing support when needed
- **Testing Approach:** Self-testing during development, hardware team assists with field validation
- **No Formal Approvals:** Build → Test → Fix → Iterate workflow (no PR reviews, no sign-offs)

**Total Project Duration:** 30-32 weeks (~8 months full-time)

---

## 🚀 32-Week Development Timeline

### **Weeks 1-2: Setup & Foundation**
**What:** Development environment, ROS 2 workspace, basic infrastructure

**Activities:**
- Install Ubuntu 22.04, ROS 2 Humble, dependencies
- Set up Git repo, workspace structure
- Create simulation environment (Gazebo optional)
- "Hello World" ROS 2 node

**AI Helps:** Configuration files, package setup scripts, boilerplate generation

**Deliverable:** Clean workspace, ready to code

---

### **Weeks 3-7: Swerve Drive System (5 weeks)**

#### **Week 3-4: Inverse/Forward Kinematics**
- Implement IK solver (Twist → 4 wheel states)
- Implement FK solver (4 wheel states → Twist)
- Unit tests for correctness
- **AI generates:** Math implementation, test cases

#### **Week 5: Module Coordination**
- Wheel angle optimization (avoid >90° rotations)
- Module alignment logic
- Velocity ramping during steering
- **AI generates:** Algorithm implementation, edge case handling

#### **Week 6-7: Nav2 Controller Plugin + Testing**
- Implement Nav2 controller interface
- Velocity governor integration
- **Hardware team:** Connect motors, test basic motion
- **Fix bugs:** Motor tuning, timing issues
- **Deliverable:** Robot moves omnidirectionally

---

### **Weeks 8-11: Navigation Stack (4 weeks)** 💡 **ParcelPal Architecture Pattern**

#### **Week 8-9: NDT Localization (Implement from Scratch)**
- Implement NDT using standard ROS 2 (pcl::NormalDistributionsTransform or ndt_omp, NOT Autoware)
- Load pre-built map, initial pose setup
- Sensor fusion (LiDAR + IMU + odometry)
- **ParcelPal provides:** Proven parameter ranges, integration patterns
- **AI helps:** Generate NDT integration code, parameter tuning, TF debugging

#### **Week 10: Nav2 Integration**
- Configure Nav2 planners (NavFn, DWB)
- Tune costmap parameters
- **Hardware team:** Create test map

#### **Week 11: Waypoint Navigation + Testing**
- Multi-waypoint route execution
- **Hardware team:** Test autonomous navigation
- **Fix bugs:** Localization drift, path planning
- **Deliverable:** Autonomous waypoint navigation

---

### **Weeks 12-15: Perception System (4 weeks)** 💡 **ParcelPal Architecture Pattern**

#### **Week 12-13: Point Cloud Processing (Implement from Scratch)**
- Implement PCL pipeline (write new code, NOT copy ParcelPal)
- Ground removal (RANSAC), voxel downsampling, height filtering
- **ParcelPal provides:** Proven binary detection approach, CPU-only architecture
- **AI helps:** Generate PCL filtering code, parameter tuning

#### **Week 14: Costmap Integration**
- Connect filtered point cloud to Nav2 costmap
- Obstacle inflation, occupancy grid
- **Hardware team:** Test obstacle detection outdoors

#### **Week 15: Testing + Bug Fixes**
- Lighting issues, false positives, tuning
- **Fix bugs:** Ground removal on slopes, dynamic obstacles
- **Deliverable:** Reliable obstacle avoidance

---

### **Weeks 16-21: Docking System (6 weeks)** 🆕 **NEW Subsystem**

#### **Week 16-17: ArUco Detection**
- Dual camera setup, calibration
- ArUco marker detection (OpenCV)
- Pose estimation (PnP)
- **AI generates:** OpenCV code, calibration tools

#### **Week 18-19: Visual Servoing Controller**
- PBVS controller (PID for x, y, θ)
- Velocity limiting during docking
- **Hardware team:** Set up test wheelchair with markers

#### **Week 20: Docking State Machine**
- State machine (IDLE → APPROACH → SEARCH → SERVOING → DOCKED)
- Failure handling, retry logic
- **AI generates:** State machine code

#### **Week 21: Docking Testing + Tuning**
- **Hardware team:** 50+ docking attempts outdoors
- Lighting variations, precision tuning
- **Fix bugs:** Marker detection failures, convergence
- **Deliverable:** >80% docking success rate (±5mm)

---

### **Weeks 22-24: Safety System (3 weeks)**

#### **Week 22: Safety Monitor + Watchdog**
- Safety supervisor node
- Watchdog for all critical subsystems
- Diagnostics aggregation
- **AI generates:** Monitoring logic, timeout handling

#### **Week 23: Emergency Stop Integration**
- Hardware E-Stop circuit integration
- Software E-Stop trigger logic
- Passenger detection sensors
- **Hardware team:** Test E-Stop response time (<100ms)

#### **Week 24: Safety Testing + Validation**
- Fault injection tests (LiDAR failure, localization loss)
- Speed limit enforcement
- **Fix bugs:** False E-Stop triggers, edge cases
- **Deliverable:** Safety system passes all critical tests

---

### **Weeks 25-28: UI & eHMI (4 weeks)**

#### **Week 25-26: Touch Screen UI**
- React/Next.js UI setup
- Rosbridge connection to ROS 2
- Mission status display, waypoint selection
- **AI generates:** React components, UI logic

#### **Week 27: eHMI Firmware (ESP32-S3)**
- LED strip control (WS2812B)
- Audio playback (I2S)
- Serial communication with main computer
- **AI generates:** ESP32 Arduino/PlatformIO code

#### **Week 28: UI/eHMI Testing + Polish**
- **Hardware team:** Test UI usability, eHMI visibility
- Wheelchair-specific states (21-28)
- **Deliverable:** Functional UI and eHMI

---

### **Weeks 29-31: System Integration & Field Testing (3 weeks)**

#### **Week 29: Full System Integration**
- All subsystems running together
- End-to-end mission testing
- **Fix bugs:** Inter-subsystem communication, timing

#### **Week 30-31: Field Testing & Bug Fixes**
- **Hardware team:** Intensive outdoor testing
- Weather, terrain, edge cases
- Performance tuning (battery life, speeds)
- **Deliverable:** Reliable full-mission execution

---

### **Week 32: Documentation & Deployment (1 week)**
- Clean up code, add comments
- Update configuration files
- Deployment procedures
- Handoff to hardware team
- **Deliverable:** Production-ready system

---

## 📊 Timeline Summary

| Phase | Weeks | Key Deliverable | ParcelPal Influence |
|-------|-------|----------------|---------------------|
| Setup | 1-2 | Dev environment ready | N/A |
| Swerve Drive | 3-7 | Omnidirectional motion | 🆕 NEW |
| Navigation | 8-11 | Autonomous waypoint navigation | 💡 70% architecture patterns |
| Perception | 12-15 | Obstacle avoidance | 💡 60% architecture patterns |
| Docking | 16-21 | Precision wheelchair docking | 🆕 NEW |
| Safety | 22-24 | Safety system validated | 💡 30% patterns |
| UI/eHMI | 25-28 | User interface & eHMI | 🆕 NEW |
| Integration | 29-31 | Full system tested | N/A |
| Deployment | 32 | Production ready | N/A |
| **TOTAL** | **32 weeks** | **Complete system** | **0% code, ~50% ideas** |

---

## Built-in Reality Buffers

- ✅ Each subsystem has 1-2 weeks for **testing + bug fixes**
- ✅ **No approval delays** - just build, test, iterate
- ✅ **Hardware team testing in parallel** - you keep coding
- ✅ **AI assistance** speeds up coding, debugging, testing
- ✅ **Buffer time** for unexpected issues

---

## Workflow: Build → Test → Fix → Deploy

**No formal approvals, no PR reviews. You are:**
- Developer (writes code)
- Reviewer (self-reviews before committing)
- Tester (validates functionality)
- User (defines requirements)

**Daily Workflow:**
```bash
# Start feature
git checkout -b feature/swerve-ik

# Code with AI help (Claude Code generates boilerplate, algorithms)
# ... coding ...

# Self-test locally
colcon build && colcon test

# Self-review (sanity check)
git diff  # Review your own changes

# Commit and merge
git commit -m "Add swerve IK solver with unit tests"
git checkout main
git merge feature/swerve-ik

# Done! No PR, no approvals.
```

---

## AI Productivity Multipliers

| Activity | Without AI | With AI | Speedup |
|----------|-----------|---------|---------|
| Code Generation | 100% | 40-50% | **2-2.5x** |
| Unit Test Writing | 100% | 30-40% | **2.5-3x** |
| Documentation | 100% | 35-45% | **2-2.5x** |
| Debugging | 100% | 50-70% | **1.5-2x** |
| **Overall** | **100%** | **45-55%** | **~2-2.5x** |

---

## Milestones & Deliverables

| Milestone | Week | Deliverable | Acceptance Criteria |
|-----------|------|-------------|---------------------|
| **M1: Swerve Drive** | 7 | Omnidirectional motion | Outdoor motion validated |
| **M2: Navigation** | 11 | Autonomous navigation | NDT ±10cm, waypoints working |
| **M3: Perception** | 15 | Obstacle avoidance | >90% detection rate |
| **M4: Docking** | 21 | Precision docking | ±5mm, >80% success |
| **M5: Safety** | 24 | Safety validated | E-Stop <100ms, watchdog OK |
| **M6: UI/eHMI** | 28 | UI & eHMI functional | Mission control working |
| **M7: Integration** | 31 | Full system tested | End-to-end missions pass |
| **M8: Production** | 32 | Production ready | Deployed, documented |

---

## Success Metrics

### Development Velocity
- **Target:** 1 major subsystem per 4-6 weeks (with AI)
- **Measurement:** Feature completion tracking

### Code Quality
- **Target:** 80%+ unit test coverage (AI-generated tests)
- **Measurement:** Code coverage reports (gcov, pytest-cov)

### Field Test Success Rate
- **Target:** >80% mission success rate at end
- **Measurement:** Mission logs, success/failure tracking

### Timeline Adherence
- **Target:** Complete within 32 weeks
- **Measurement:** Weekly milestone tracking

---

## Risk Management

| Risk | Impact | Mitigation |
|------|--------|------------|
| NDT outdoor drift | +2 weeks | Early outdoor testing, AI parameter tuning |
| ArUco outdoor detection | +2 weeks | Dual camera, AI optimization |
| Hardware delays | +2-4 weeks | Order early, backup suppliers |
| CPU performance | +1 week | Early profiling, AI optimization |

**Contingency Buffer:** Timeline already includes realistic testing/debug time

**Recommended:** Plan for 32-34 weeks with 2-week buffer

---

**Document Status:** Active - Living Document
**Last Updated:** 2025-12-16
**Maintained By:** You (Solo Developer)
**Review Frequency:** Weekly self-review, adjust as needed

---

**Ready to start Week 1?** 🚀
