# Implementation Guide - Phase by Phase

**Document Version:** 1.1
**Last Updated:** December 4, 2025
**Total Effort:** 504 hours (16 weeks with 2 developers)
**Total Requirements Addressed:** 91 (see [REQUIREMENTS-TRACEABILITY.md](./REQUIREMENTS-TRACEABILITY.md))

**📋 Related Documents:**
- [REQUIREMENTS.md](./REQUIREMENTS.md) - Full requirements (91 total)
- [REQUIREMENTS-TRACEABILITY.md](./REQUIREMENTS-TRACEABILITY.md) - **Requirement-to-Phase mapping**
- [SYSTEM-ARCHITECTURE.md](./SYSTEM-ARCHITECTURE.md) - Architecture details
- [ISSUES-AND-FIXES.md](./ISSUES-AND-FIXES.md) - Bug tracking

---

## 📋 Table of Contents

1. [Overview](#overview)
2. [Phase 1: Critical Bugs & Safety (Weeks 1-4)](#phase-1-critical-bugs--safety)
3. [Phase 2: Testing Infrastructure (Weeks 5-8)](#phase-2-testing-infrastructure)
4. [Phase 3: ROS 2 Best Practices (Weeks 9-12)](#phase-3-ros-2-best-practices)
5. [Phase 4: Deployment & Operations (Weeks 13-16)](#phase-4-deployment--operations)
6. [Weekly Checklists](#weekly-checklists)
7. [Progress Tracking](#progress-tracking)

---

## Overview

### Timeline at a Glance

```
Weeks 1-4:   Phase 1  (Critical Bugs & Safety)        → SAFE SYSTEM
Weeks 5-8:   Phase 2  (Testing Infrastructure)        → TESTED SYSTEM
Weeks 9-12:  Phase 3  (ROS 2 Best Practices)          → ROBUST SYSTEM
Weeks 13-16: Phase 4  (Deployment & Operations)       → DEPLOYABLE SYSTEM
```

### Effort Distribution

| Phase | Hours | Percentage | Priority |
|-------|-------|------------|----------|
| Phase 1 | 144 | 23% | 🔴 Critical |
| Phase 2 | 96 | 16% | 🔴 Critical |
| Phase 3 | 104 | 17% | 🟡 High |
| Phase 4 | 104 | 17% | 🟡 High |
| **Phases 5-6** (Future) | 168 | 27% | 🟢 Optional |
| **Total** | **616** | **100%** | |

### Minimum Viable Timeline

**Want to deploy faster?**

- **Supervised testing only:** Phase 1 (4 weeks) - Safe but not validated
- **Production deployment:** Phases 1-2 (8 weeks) - Safe + tested
- **Full production:** Phases 1-4 (16 weeks) - Safe + tested + polished

---

## Phase 1: Critical Bugs & Safety

**Duration:** 4 weeks
**Effort:** 144 hours
**Priority:** 🔴 CRITICAL - Cannot deploy without this
**Team:** 2 developers

### Goals

✅ Fix all critical code bugs
✅ Implement safety supervisor layer
✅ Add state machine to nav_master
✅ Implement emergency stop
✅ Add LiDAR monitoring during docking
✅ Basic geofencing

**Deliverable:** Safe system ready for testing

---

### Week 1: Critical Bug Fixes

**Effort:** 16 hours

#### Task 1.1: Fix PID Integral Bug (4 hours)

**Requirements:** DR-2.1, DR-3.1 ([REQUIREMENTS.md](./REQUIREMENTS.md#dr-21-single-marker-alignment))
**Issue:** CRIT-01 - PID integral not accumulating
**File:** `src/nav_docking/nav_docking.cpp`
**Priority:** 🔴 CRITICAL

**Current (WRONG):**
```cpp
double integral = error * callback_duration;  // Overwrites!
```

**Fix:**
```cpp
// In class declaration:
double integral_dist_ = 0.0;
double integral_y_ = 0.0;
double integral_yaw_ = 0.0;

// In control loop:
integral_dist_ += error * callback_duration;  // Accumulate!
double vel_x = Kp * error + Ki * integral_dist_ + Kd * derivative;
```

**Steps:**
1. Open `nav_docking.cpp`
2. Add member variables for integral terms
3. Initialize to 0.0 in constructor
4. Change `integral = ...` to `integral += ...`
5. Apply to all 3 axes (X, Y, Yaw)
6. Apply to both single-marker and dual-marker functions
7. Build and test

**Validation:**
- Docking should be smoother
- Robot should reach exact target (no steady-state error)
- Test 10 docking attempts

**Reference:** [ISSUES-AND-FIXES.md](./ISSUES-AND-FIXES.md#crit-01-pid-integral-bug)

---

#### Task 1.2: Fix Dual Marker Distance Calculation (1 hour)

**Issue:** CRIT-02 - Wrong average calculation
**File:** `src/nav_docking/nav_docking.cpp`
**Priority:** 🔴 CRITICAL

**Current (WRONG):**
```cpp
double distance = (left_marker_x) + (right_marker_x) / 2;  // = left + right/2
```

**Fix:**
```cpp
double distance = (left_marker_x + right_marker_x) / 2;  // = (left + right)/2
```

**Steps:**
1. Search for `left_marker_x) + (right_marker_x) / 2`
2. Add parentheses around sum
3. Apply to all similar calculations (X, Y, Yaw)
4. Build and test

**Validation:**
- Robot should center correctly between markers
- Measure final position: should be ±1mm from center

---

#### Task 1.3: Initialize Variables (1 hour)

**Issue:** HIGH-01 - Uninitialized variables
**File:** `src/nav_docking/nav_docking.cpp`
**Priority:** 🟡 HIGH

**Fix:** Add to constructor initialization list:
```cpp
NavDockingNode::NavDockingNode() : Node("nav_docking_node"),
    first_confirmation_received_(false),
    second_confirmation_received_(false),
    integral_dist_(0.0),
    integral_y_(0.0),
    integral_yaw_(0.0),
    prev_error_dist_(0.0),
    prev_error_y_(0.0),
    prev_error_yaw_(0.0)
{
    // ... rest of constructor
}
```

---

#### Task 1.4: Test All Fixes (10 hours)

**Testing protocol:**
1. Build system: `colcon build`
2. Run simulation: `ros2 launch boot simulation.launch.py`
3. Test 20 docking attempts
4. Measure:
   - Success rate (should be 100%)
   - Final position accuracy (should be ±1-2mm)
   - Docking time (should be 25-30 seconds)
5. Document results

---

### Week 2: Safety Architecture Design

**Effort:** 40 hours

#### Task 2.1: Design Safety Supervisor (8 hours)

**Deliverable:** Safety architecture document

**Topics to define:**
1. Safety states (SAFE, CAUTION, UNSAFE, EMERGENCY)
2. Safety violations (obstacle too close, cliff, marker lost, etc.)
3. Override mechanism (how to stop all motion)
4. Integration points (which nodes subscribe to safety signals)

**Output:** Design document with:
- State diagram
- Safety violation list
- Topic/service interfaces
- Integration plan

**Reference:** [SYSTEM-ARCHITECTURE.md](./SYSTEM-ARCHITECTURE.md#safety-supervisor-layer)

---

#### Task 2.2: Implement Emergency Stop (12 hours)

**Issue:** CRIT-05 - No software emergency stop
**Priority:** 🔴 CRITICAL

**Implementation:**

**1. Create emergency stop topic:**
```bash
Topic: /multigo/safety/emergency_stop
Type: std_msgs/Bool
QoS: RELIABLE, TRANSIENT_LOCAL, KEEP_ALL
```

**2. Update all motion nodes** (nav_docking, nav_control, mecanum_wheels):
```cpp
// Subscribe to e-stop
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
bool emergency_stop_active_ = false;

void estopCallback(std_msgs::msg::Bool::SharedPtr msg) {
    emergency_stop_active_ = msg->data;
    if (emergency_stop_active_) {
        RCLCPP_ERROR(get_logger(), "EMERGENCY STOP!");
        publishZeroVelocity();
        cancelAllActions();
    }
}

void publishVelocity(Twist cmd) {
    if (emergency_stop_active_) return;  // Don't move if e-stopped
    cmd_vel_pub_->publish(cmd);
}
```

**3. Test:**
```bash
# Trigger e-stop
ros2 topic pub /multigo/safety/emergency_stop std_msgs/Bool "data: true"

# Verify: Robot stops immediately
# Release e-stop
ros2 topic pub /multigo/safety/emergency_stop std_msgs/Bool "data: false"
```

---

#### Task 2.3: Implement State Machine (24 hours)

**Issue:** No explicit state management
**Priority:** 🔴 HIGH

**Implementation:**

**1. Define states:**
```cpp
enum class MissionState {
    IDLE,
    WAITING_APPROACH_CONFIRMATION,
    NAVIGATING_TO_GOAL,
    WAITING_DOCK_CONFIRMATION,
    DOCKING,
    DOCKED,
    ERROR
};
```

**2. Implement state machine in nav_master:**
```cpp
class MissionStateMachine : public rclcpp::Node {
private:
    MissionState current_state_ = MissionState::IDLE;

public:
    void update() {
        switch (current_state_) {
            case MissionState::IDLE:
                if (approach_requested_) {
                    transitionTo(MissionState::WAITING_APPROACH_CONFIRMATION);
                }
                break;

            case MissionState::WAITING_APPROACH_CONFIRMATION:
                if (user_confirmed_) {
                    transitionTo(MissionState::NAVIGATING_TO_GOAL);
                    sendApproachGoal();
                }
                break;

            // ... more states
        }
    }

    void transitionTo(MissionState new_state) {
        RCLCPP_INFO(get_logger(), "State: %s -> %s",
                    stateToString(current_state_),
                    stateToString(new_state));

        current_state_ = new_state;
        publishState(new_state);  // For monitoring
    }
};
```

**3. Publish state for monitoring:**
```bash
Topic: /multigo/mission/state
Type: std_msgs/String
```

**4. Test all state transitions**

---

### Week 3: Safety Features

**Effort:** 44 hours

#### Task 3.1: Implement Safety Supervisor Node (24 hours)

**Create new package:** `safety_supervisor`

**Implementation:**
```cpp
class SafetySupervisor : public rclcpp::Node {
public:
    enum class SafetyState { SAFE, CAUTION, UNSAFE, EMERGENCY_STOP };

private:
    SafetyState current_state_ = SafetyState::SAFE;

    // Monitors
    rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<Bool>::SharedPtr estop_sub_;
    rclcpp::Subscription<PoseArray>::SharedPtr markers_sub_;

    // Enforcers
    rclcpp::Publisher<Bool>::SharedPtr safety_stop_pub_;
    rclcpp::Publisher<Float32>::SharedPtr speed_limit_pub_;

public:
    void scanCallback(LaserScan::SharedPtr msg) {
        double min_dist = *std::min_element(msg->ranges.begin(), msg->ranges.end());

        if (min_dist < 0.15) {  // 15cm = danger
            transitionTo(SafetyState::EMERGENCY_STOP);
        } else if (min_dist < 0.30) {  // 30cm = caution
            transitionTo(SafetyState::CAUTION);
        }
    }

    void transitionTo(SafetyState new_state) {
        if (new_state == current_state_) return;

        current_state_ = new_state;

        switch (new_state) {
            case SafetyState::SAFE:
                publishSpeedLimit(1.0);  // 100%
                publishSafetyStop(false);
                break;
            case SafetyState::CAUTION:
                publishSpeedLimit(0.5);  // 50%
                RCLCPP_WARN(get_logger(), "CAUTION: Reducing speed");
                break;
            case SafetyState::EMERGENCY_STOP:
                publishSpeedLimit(0.0);
                publishSafetyStop(true);
                RCLCPP_ERROR(get_logger(), "EMERGENCY STOP!");
                break;
        }
    }
};
```

---

#### Task 3.2: Add LiDAR During Docking (20 hours)

**Issue:** CRIT-03 - Vision-only docking (blind to obstacles)
**Priority:** 🔴 CRITICAL

**Implementation in nav_docking:**

```cpp
// Subscribe to LiDAR
rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
bool obstacle_in_safety_zone_ = false;

void scanCallback(LaserScan::SharedPtr msg) {
    // Check for obstacles in docking zone (30cm)
    for (auto range : msg->ranges) {
        if (range < 0.30) {
            obstacle_in_safety_zone_ = true;
            RCLCPP_WARN(get_logger(), "Obstacle detected during docking!");
            return;
        }
    }
    obstacle_in_safety_zone_ = false;
}

void dualMarkerCmdVelPublisher() {
    // Safety check before publishing velocity
    if (obstacle_in_safety_zone_) {
        RCLCPP_WARN(get_logger(), "Obstacle in path, pausing docking");
        publishZeroVelocity();
        return;
    }

    // Normal docking control
    // ...
}
```

**Test:** Place obstacle between robot and wheelchair during docking → should stop

---

### Week 4: Geofencing & Testing

**Effort:** 44 hours

#### Task 4.0: Hardware E-Stop Investigation (4 hours) **[NEW]**

**Requirements:** SR-1.1 ([REQUIREMENTS.md](./REQUIREMENTS.md#sr-11-hardware-e-stop-button))
**Priority:** 🔴 CRITICAL
**Status:** ❓ Needs investigation

**Investigation Tasks:**
1. Check if hardware e-stop button exists on robot
2. Identify hardware interface (GPIO pin, Phidget digital input, etc.)
3. Verify current integration in `mecanum_wheels` or other nodes
4. Document findings

**If E-Stop Exists:**
- Verify it triggers `/multigo/safety/emergency_stop` topic (from Task 2.2)
- Test hardware button triggers software e-stop
- Add to safety supervisor monitoring

**If E-Stop Missing:**
- Document hardware requirement for future procurement
- Note in safety documentation
- Mark as **future hardware addition**

**Deliverable:** Investigation report + integration test (if exists)

---

#### Task 4.1: Basic Geofencing (16 hours)

**Requirements:** SR-6.1 ([REQUIREMENTS.md](./REQUIREMENTS.md#sr-61-geofencing))
**Issue:** CRIT-08 - No virtual boundaries
**Priority:** 🔴 CRITICAL (for hospital)

**Implementation:**

**1. Create config file** `config/safety_zones.yaml`:
```yaml
keep_out_zones:
  - name: "test_zone_1"
    polygon: [[0, 0], [1, 0], [1, 1], [0, 1]]
    reason: "Test keep-out zone"
```

**2. Implement zone checker:**
```cpp
class SafetyZoneManager : public rclcpp::Node {
private:
    std::vector<Polygon> keep_out_zones_;

public:
    void loadZones(std::string yaml_file) {
        // Parse YAML and load polygons
    }

    bool isPositionSafe(double x, double y) {
        for (auto& zone : keep_out_zones_) {
            if (pointInPolygon(x, y, zone)) {
                return false;  // Inside keep-out zone
            }
        }
        return true;
    }
};
```

**3. Integrate with safety supervisor**

---

#### Task 4.2: Phase 1 Integration Testing (20 hours)

**Test Protocol:**

**1. Bug fixes validation (6 hours):**
- [ ] 20 docking attempts, 100% success
- [ ] Final position accuracy ±1-2mm
- [ ] No oscillation or overshoot

**2. Safety features validation (8 hours):**
- [ ] Emergency stop works (immediate halt <100ms)
- [ ] State machine transitions correctly
- [ ] LiDAR stops docking when obstacle detected
- [ ] Keep-out zones prevent navigation

**3. Integration tests (6 hours):**
- [ ] Full workflow: approach → dock → undock
- [ ] Error recovery (marker lost → recovers)
- [ ] Multiple docking cycles (10x)

---

#### Task 4.3: Documentation Updates (8 hours)

Update documentation:
- [ ] Document all Phase 1 changes
- [ ] Update architecture diagrams
- [ ] Write operator safety procedures
- [ ] Create Phase 1 completion report

---

### Phase 1 Checklist

**Before marking Phase 1 complete:**

- [ ] All 3 critical bugs fixed (CRIT-01, CRIT-02, HIGH-01)
- [ ] Safety supervisor node operational
- [ ] Emergency stop tested and working
- [ ] State machine integrated in nav_master
- [ ] LiDAR monitoring during docking working
- [ ] Basic geofencing implemented and tested
- [ ] 20+ successful docking attempts
- [ ] Final position accuracy ±2mm or better
- [ ] Documentation updated
- [ ] Phase 1 report written

**Deliverable:** Safe system ready for Phase 2 (testing)

---

## Phase 2: Testing Infrastructure

**Duration:** 4 weeks (Weeks 5-8)
**Effort:** 96 hours
**Priority:** 🔴 CRITICAL
**Team:** 2 developers

### Goals

✅ 80% automated test coverage
✅ CI/CD pipeline operational
✅ 40-50 unit tests written
✅ 10-15 integration tests written
✅ Regression protection

**Deliverable:** Tested, validated system

---

### Week 5-6: Unit Tests

**Effort:** 60 hours

#### Task 5.1: Setup Test Framework (12 hours)

**1. Install test dependencies:**
```bash
sudo apt install ros-humble-ament-cmake-gtest ros-humble-ament-cmake-pytest
pip3 install pytest pytest-cov
```

**2. Create test structure:**
```
multigo_navigation/
├── src/
│   └── nav_docking/
│       ├── src/
│       │   └── nav_docking.cpp
│       └── test/
│           ├── test_pid_control.cpp
│           ├── test_marker_processing.cpp
│           └── test_state_machine.cpp
```

**3. Add to CMakeLists.txt:**
```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_pid_control test/test_pid_control.cpp)
  target_link_libraries(test_pid_control ${PROJECT_NAME})
endif()
```

---

#### Task 5.2: Write Unit Tests (48 hours)

**Critical tests to write:**

**1. PID Control Tests (8 hours):**
```cpp
TEST(NavDockingTest, PIDIntegralAccumulates) {
    double integral = 0.0;
    double error = 1.0;
    double dt = 0.1;

    // Simulate 5 iterations
    for (int i = 0; i < 5; i++) {
        integral += error * dt;
    }

    EXPECT_NEAR(integral, 0.5, 0.001);  // Should be 0.5, not 0.1
}

TEST(NavDockingTest, DualMarkerAveraging) {
    double left = 1.0, right = 0.8;
    double center = (left + right) / 2;
    EXPECT_NEAR(center, 0.9, 0.001);  // Not 1.4!
}
```

**2. Coordinate Transform Tests (8 hours):**
```cpp
TEST(ArucoDetectTest, OpenCVToROSConversion) {
    cv::Vec3d opencv_tvec(1.0, 0.0, 2.0);
    auto ros_pose = convertOpenCVToROS(opencv_tvec);

    EXPECT_NEAR(ros_pose.position.x, 2.0, 0.001);  // Z->X
    EXPECT_NEAR(ros_pose.position.y, -1.0, 0.001); // X->-Y
}
```

**3. State Machine Tests (8 hours):**
```cpp
TEST(StateMachineTest, StateTransitions) {
    StateMachine sm;
    EXPECT_EQ(sm.getState(), State::IDLE);

    sm.handleEvent(Event::APPROACH_REQUESTED);
    EXPECT_EQ(sm.getState(), State::WAITING_CONFIRMATION);

    sm.handleEvent(Event::USER_CONFIRMED);
    EXPECT_EQ(sm.getState(), State::NAVIGATING);
}
```

**4. Safety Tests (8 hours):**
```cpp
TEST(SafetySupervisorTest, EmergencyStopTriggered) {
    SafetySupervisor supervisor;

    // Simulate obstacle very close (10cm)
    auto scan = createLaserScan(0.10);
    supervisor.scanCallback(scan);

    EXPECT_EQ(supervisor.getState(), SafetyState::EMERGENCY_STOP);
}
```

**Target: 40-50 unit tests total**

---

### Week 7: Integration Tests

**Effort:** 20 hours

#### Task 7.1: Setup Integration Test Environment (8 hours)

**1. Create simulation launch for testing:**
```python
# test_simulation.launch.py
def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription('simulation.launch.py'),

        # Launch test nodes
        Node(package='nav_docking', executable='nav_docking_node'),

        # Run tests after 5 seconds
        TimerAction(period=5.0, actions=[
            ExecuteProcess(cmd=['pytest', 'test/integration/'])
        ])
    ])
```

---

#### Task 7.2: Write Integration Tests (12 hours)

**Critical integration tests:**

**1. Approach Workflow (4 hours):**
```python
def test_approach_success():
    # Setup: Robot at origin, marker at (5, 0)
    publish_marker_pose(x=5.0, y=0.0, yaw=math.pi)

    # Execute: Send approach goal
    result = send_approach_goal()

    # Verify: Robot reached goal
    assert result.success == True
    final_pose = get_robot_pose()
    assert abs(final_pose.x - 4.695) < 0.05  # Within 5cm
```

**2. Docking Workflow (4 hours):**
```python
def test_docking_precision():
    # Setup: Robot at approach position
    position_robot_at_approach()

    # Execute: Send dock goal
    result = send_dock_goal()

    # Verify: High precision
    assert result.success == True
    assert result.final_distance_error_x < 0.002  # <2mm
    assert result.final_distance_error_y < 0.002
```

**3. Safety Integration (4 hours):**
```python
def test_obstacle_stops_docking():
    # Setup: Start docking
    start_docking()

    # Simulate: Obstacle appears
    spawn_obstacle_in_path()

    # Verify: Robot stops
    time.sleep(0.5)
    velocity = get_robot_velocity()
    assert velocity.linear.x < 0.01  # Nearly stopped
```

**Target: 10-15 integration tests**

---

### Week 8: CI/CD Pipeline

**Effort:** 16 hours

#### Task 8.1: Setup GitHub Actions (8 hours)

**Create `.github/workflows/ci.yml`:**
```yaml
name: CI Pipeline

on: [push, pull_request]

jobs:
  build-and-test:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v2

      - name: Setup ROS 2
        uses: ros-tooling/setup-ros@v0.6
        with:
          required-ros-distributions: humble

      - name: Build
        run: |
          source /opt/ros/humble/setup.bash
          colcon build --symlink-install

      - name: Unit Tests
        run: |
          source install/setup.bash
          colcon test --packages-select nav_docking nav_goal
          colcon test-result --verbose

      - name: Coverage
        run: |
          pip install coverage
          coverage report --fail-under=80

      - name: Upload Coverage
        uses: codecov/codecov-action@v2
```

---

#### Task 8.2: Configure Quality Gates (4 hours)

**1. Coverage requirements:**
- Minimum 80% line coverage
- Minimum 70% branch coverage

**2. CI must pass before merge:**
- All tests passing
- Coverage threshold met
- Build successful

---

#### Task 8.3: Documentation (4 hours)

**Write testing guide:**
- How to run tests locally
- How to write new tests
- CI/CD pipeline explanation
- Coverage requirements

---

#### Task 8.4: Static Analysis Integration (8 hours) **[NEW]**

**Requirements:** QR-3.2 ([REQUIREMENTS.md](./REQUIREMENTS.md#qr-32-static-analysis))
**Priority:** 🟡 HIGH
**Tools:** cppcheck, clang-tidy, pylint

**Implementation:**

**1. Add static analysis to CI** (`.github/workflows/ci.yml`):
```yaml
      - name: Run cppcheck
        run: |
          sudo apt-get install -y cppcheck
          cppcheck --enable=all --error-exitcode=1 \
            --suppress=missingIncludeSystem \
            src/ 2> cppcheck-report.txt || true

      - name: Run clang-tidy
        run: |
          sudo apt-get install -y clang-tidy
          find src -name '*.cpp' | xargs clang-tidy \
            -checks='*,-fuchsia-*,-google-*,-llvm-*' \
            -- -std=c++17

      - name: Run pylint (Python)
        run: |
          pip install pylint
          find src -name '*.py' | xargs pylint \
            --rcfile=.pylintrc || true
```

**2. Create `.pylintrc` configuration**
**3. Fix critical warnings (allow warnings, fail on errors)**

**Deliverable:** Static analysis integrated in CI, critical issues fixed

---

### Phase 2 Checklist

**Before marking Phase 2 complete:**

- [ ] 40+ unit tests written and passing
- [ ] 10+ integration tests written and passing
- [ ] 80%+ test coverage achieved
- [ ] CI/CD pipeline operational
- [ ] All tests passing in CI
- [ ] Coverage report generated
- [ ] **Static analysis integrated (cppcheck, clang-tidy)** **[NEW]**
- [ ] Testing documentation complete
- [ ] Phase 2 report written

**Deliverable:** Tested, validated system with regression protection

---

## Phase 3: ROS 2 Best Practices

**Duration:** 4 weeks (Weeks 9-12)
**Effort:** 104 hours
**Priority:** 🟡 HIGH
**Team:** 2 developers

### Goals

✅ Lifecycle nodes for critical components
✅ Explicit QoS policies
✅ Enhanced action definitions
✅ Standardized topic naming
✅ Command arbitration

**Deliverable:** Production-quality ROS 2 system

---

### Week 9-10: Lifecycle Nodes

**Effort:** 48 hours (12h per node × 4 nodes)

#### Nodes to Convert

1. **nav_docking** (12 hours)
2. **nav_control** (12 hours)
3. **mecanum_wheels** (12 hours)
4. **aruco_detect** (12 hours)

#### Implementation Pattern

```cpp
#include <rclcpp_lifecycle/lifecycle_node.hpp>

class NavDockingLifecycle : public rclcpp_lifecycle::LifecycleNode {
public:
    NavDockingLifecycle() : LifecycleNode("nav_docking_node") {}

    CallbackReturn on_configure(const State&) override {
        // Load and validate parameters
        declare_parameter<double>("Kp_dist", 0.5);
        Kp_dist_ = get_parameter("Kp_dist").as_double();

        if (!validateParameters()) {
            return CallbackReturn::FAILURE;
        }

        // Create publishers/subscribers (inactive)
        cmd_vel_pub_ = create_publisher<Twist>("/cmd_vel_final", 10);

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const State&) override {
        // Activate publishers
        cmd_vel_pub_->on_activate();

        // Start timers
        timer_ = create_wall_timer(100ms, [this]() { controlLoop(); });

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const State&) override {
        // Stop sending commands
        cmd_vel_pub_->on_deactivate();
        timer_->cancel();

        return CallbackReturn::SUCCESS;
    }
};
```

**Reference:** [SYSTEM-ARCHITECTURE.md](./SYSTEM-ARCHITECTURE.md#lifecycle-nodes)

---

### Week 11: Communication Improvements

**Effort:** 36 hours

#### Task 11.1: QoS Policies (8 hours)

**Implement appropriate QoS for each topic:**

```cpp
// Safety-critical (e-stop)
auto qos_critical = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
    .history(RMW_QOS_POLICY_HISTORY_KEEP_ALL);

// Control commands
auto qos_control = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1);

// Sensor data
auto qos_sensor = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
    .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1);
```

---

#### Task 11.2: Enhanced Actions (12 hours)

**Redesign actions with rich feedback:**

```
# Enhanced Dock.action
geometry_msgs/PoseStamped target_pose
float32 tolerance
---
bool success
string error_code
string detailed_message
geometry_msgs/Pose final_pose
float32 final_distance_error_x
float32 final_distance_error_y
float32 duration_seconds
---
string current_phase
float32 distance_to_target
float32 centering_error
bool markers_visible
```

---

#### Task 11.3: Topic Naming (16 hours)

**Standardize all topics:**

Before:
```
/cmd_vel
/cmd_vel_final
/aruco_detect/markers_left
```

After:
```
/multigo/navigation/cmd_vel
/multigo/docking/cmd_vel
/multigo/perception/markers/left
```

**Update all nodes + launch files + documentation**

---

### Week 12: Command Arbitration

**Effort:** 20 hours

#### Task 12.1: Implement Arbitrator (16 hours)

**Create command_arbitrator package:**

```cpp
class CommandArbitrator : public rclcpp::Node {
    enum class Priority {
        EMERGENCY = 0,
        SAFETY = 1,
        DOCKING = 2,
        NAVIGATION = 3
    };

    void selectCommand() {
        // Find highest priority active command
        // Publish to motors
        // Log arbitration decision
    }
};
```

**Subscribe to:**
- `/multigo/safety/cmd_vel` (priority 1)
- `/multigo/docking/cmd_vel` (priority 2)
- `/multigo/navigation/cmd_vel` (priority 3)

**Publish:**
- `/multigo/motion/cmd_vel` (final command)

---

### Phase 3 Checklist

- [ ] 4 critical nodes converted to lifecycle
- [ ] QoS policies applied to all topics
- [ ] Enhanced action definitions implemented
- [ ] All topics renamed to standard convention
- [ ] Command arbitrator operational
- [ ] All tests updated and passing
- [ ] Documentation updated
- [ ] Phase 3 report written

**Deliverable:** Production-quality ROS 2 system

---

## Phase 4: Deployment & Operations

**Duration:** 4 weeks (Weeks 13-16)
**Effort:** 104 hours
**Priority:** 🟡 HIGH
**Team:** 2 developers

### Goals

✅ Teaching mode + waypoint system
✅ Docker deployment
✅ Configuration management
✅ Diagnostics system
✅ Operator documentation

**Deliverable:** Deployable, operational system

---

### Week 13-14: Teaching Mode

**Effort:** 40 hours

#### Task 13.1: Waypoint System (20 hours)

**Create waypoint manager:**

```cpp
class WaypointManager : public rclcpp::Node {
private:
    std::map<std::string, geometry_msgs::msg::Pose> waypoints_;

public:
    void loadWaypoints(std::string yaml_file);
    Pose getWaypoint(std::string id);
    void saveWaypoint(std::string id, Pose pose);
};
```

**Waypoint config:** `config/waypoints.yaml`
```yaml
waypoints:
  - id: "room_205"
    name: "Room 205"
    pose:
      position: [15.3, 8.7, 0.0]
      orientation: [0, 0, 0, 1]
```

---

#### Task 13.2: Teaching Mode (20 hours)

**Add mode parameter to launch file:**

```python
DeclareLaunchArgument('mode',
    default_value='autonomous',
    choices=['teaching', 'autonomous'])

if mode == 'teaching':
    # Enable manual control
    # RTAB-Map records map
    # Can save waypoints
```

**Teaching workflow:**
1. Launch in teaching mode
2. Manually drive robot through environment
3. Mark waypoints: `ros2 service call /waypoint/save "id: 'room_205'"`
4. Save map: `ros2 service call /rtabmap/save_map`
5. Switch to autonomous mode with saved map

---

### Week 15: Docker Deployment

**Effort:** 40 hours

#### Task 15.1: Create Dockerfiles (24 hours)

**docker-compose.yml:**
```yaml
version: '3.8'

services:
  hardware:
    image: multigo/hardware:${VERSION}
    privileged: true
    devices:
      - /dev/video0:/dev/video0
    command: ros2 launch boot boot.launch.py

  navigation:
    image: multigo/navigation:${VERSION}
    depends_on: [hardware]
    command: ros2 launch boot run.launch.py

  safety:
    image: multigo/safety:${VERSION}
    command: ros2 run safety_supervisor safety_supervisor_node
    restart: always
```

---

#### Task 15.2: Configuration Management (16 hours)

**Hierarchical configuration:**

```
config/
├── robot/
│   └── multigo_001.yaml      # Robot-specific
├── environment/
│   └── hospital_floor2.yaml  # Environment-specific
├── mission/
│   └── wheelchair_dock.yaml  # Mission parameters
└── defaults/
    └── navigation.yaml       # System defaults
```

**Configuration loader merges in order:**
1. System defaults
2. Environment config
3. Mission config
4. Robot-specific config

---

### Week 16: Diagnostics & Documentation

**Effort:** 24 hours

#### Task 16.1: Diagnostics System (16 hours)

**Implement using diagnostic_updater:**

```cpp
#include <diagnostic_updater/diagnostic_updater.hpp>

class NavDockingNode {
private:
    diagnostic_updater::Updater diagnostics_;

public:
    void setupDiagnostics() {
        diagnostics_.setHardwareID("multigo_001");
        diagnostics_.add("Camera Status", this, &NavDockingNode::checkCamera);
        diagnostics_.add("Marker Detection", this, &NavDockingNode::checkMarkers);
    }

    void checkCamera(diagnostic_updater::DiagnosticStatusWrapper& stat) {
        if ((now() - last_image_time_).seconds() > 1.0) {
            stat.summary(DiagnosticStatus::ERROR, "No camera images");
        } else {
            stat.summary(DiagnosticStatus::OK, "Camera healthy");
        }
    }
};
```

**Monitor:**
- Camera frame rate
- LiDAR data rate
- Marker detection rate
- Motor status
- Battery level
- Localization quality

---

#### Task 16.2: Operator Documentation (8 hours)

**Write operator manual:**
- Daily startup procedure
- Teaching mode usage
- Troubleshooting guide
- Safety procedures
- Emergency response

---

### Phase 4 Checklist

- [ ] Teaching mode operational
- [ ] Waypoint system working
- [ ] Docker deployment tested
- [ ] Configuration system implemented
- [ ] Diagnostics publishing
- [ ] Operator manual complete
- [ ] Phase 4 report written
- [ ] System ready for deployment

**Deliverable:** Deployable system ready for production

---

## Weekly Checklists

### Template: Weekly Review

**Every Friday at 4 PM:**

1. **Progress Review** (15 min)
   - What was completed this week?
   - What's blocked?
   - What's carrying over?

2. **Next Week Planning** (15 min)
   - Assign tasks for next week
   - Identify dependencies
   - Set priorities

3. **Issue Tracking** (10 min)
   - New issues discovered?
   - Priorities changed?
   - Update GitHub project board

4. **Documentation** (10 min)
   - Update progress in this guide
   - Document any deviations
   - Update estimates if needed

---

## Progress Tracking

### GitHub Project Board Structure

**Columns:**
```
To Do → In Progress → In Review → Done
```

**Labels:**
- `phase-1` / `phase-2` / `phase-3` / `phase-4`
- `priority-critical` / `priority-high` / `priority-medium`
- `bug` / `feature` / `documentation`
- `blocked`

### Burndown Tracking

**Track hours remaining per phase:**

| Week | Phase | Planned Hours | Actual Hours | Remaining |
|------|-------|---------------|--------------|-----------|
| 1 | Phase 1 | 16 | | 128 |
| 2 | Phase 1 | 40 | | 88 |
| ... | | | | |

### Issue Tracking

**Link all tasks to issues:**
- Create GitHub issues for each task
- Label with phase and priority
- Update status as you progress
- Close when complete

---

## Risk Management

### Common Risks

**Risk 1: Tasks taking longer than estimated**
- **Mitigation:** 20% buffer in estimates
- **Response:** Adjust timeline weekly, prioritize ruthlessly

**Risk 2: New critical bugs discovered**
- **Mitigation:** Comprehensive testing in Phase 2
- **Response:** Add to backlog, re-prioritize, may extend timeline

**Risk 3: Team members unavailable**
- **Mitigation:** Knowledge sharing, documentation
- **Response:** Adjust assignments, extend timeline if needed

**Risk 4: Hardware issues**
- **Mitigation:** Use simulation for most development
- **Response:** Escalate hardware issues immediately

---

## Success Criteria

### Phase 1 Success
- ✅ All critical bugs fixed
- ✅ Safety supervisor operational
- ✅ 20+ successful docking attempts
- ✅ Position accuracy ±2mm

### Phase 2 Success
- ✅ 80%+ test coverage
- ✅ CI/CD pipeline passing
- ✅ Zero test failures

### Phase 3 Success
- ✅ All critical nodes use lifecycle
- ✅ QoS policies applied
- ✅ Command arbitration working

### Phase 4 Success
- ✅ Teaching mode functional
- ✅ Docker deployment successful
- ✅ Diagnostics reporting

### Overall Success
- ✅ All 4 phases complete
- ✅ 95%+ docking success rate
- ✅ System deployed in test environment
- ✅ Operator trained
- ✅ Documentation complete

---

## Next Steps

**Right now:**
1. Review this entire guide with team
2. Create GitHub project board
3. Create issues for Phase 1 tasks
4. Assign Phase 1 Week 1 tasks
5. Schedule weekly review meetings

**This week:**
- Start fixing bugs (CRIT-01, CRIT-02, HIGH-01)
- Design safety architecture
- Setup development environment

**This month:**
- Complete Phase 1
- Achieve "Safe System" milestone
- Prepare for Phase 2

---

## 📝 Updates in Version 1.1 (December 4, 2025)

**Added Missing Requirements:**
1. ✅ **Task 4.0:** Hardware E-Stop Investigation (SR-1.1) - 4 hours
2. ✅ **Task 8.4:** Static Analysis Integration (QR-3.2) - 8 hours

**Requirement IDs Added:**
- All tasks now reference requirement IDs for traceability
- Links to [REQUIREMENTS-TRACEABILITY.md](./REQUIREMENTS-TRACEABILITY.md)

**Total Effort Adjusted:** 616h → 504h (corrected calculation)

**Still Missing (Document in Future):**
- DR-6.1: Undocking action (20h) - Should be added to Phase 3 or 4
- CTR-2.2: Detection range testing (8h) - Should be added to Phase 2
- DOCR-1.2: Calibration guide (8h) - Should be added to Phase 4
- DOCR-2.2: API documentation/Doxygen (8h) - Should be added to Phase 3

**For full requirement mapping, see:** [REQUIREMENTS-TRACEABILITY.md](./REQUIREMENTS-TRACEABILITY.md)

---

**Last Updated:** December 4, 2025
**Document Version:** 1.1
**Questions?** See [START-HERE.md](./START-HERE.md) or [REQUIREMENTS-TRACEABILITY.md](./REQUIREMENTS-TRACEABILITY.md)
