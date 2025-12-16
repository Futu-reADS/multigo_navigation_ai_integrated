# Complete Testing Guide

**Document ID:** TEST-COMPLETE-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Comprehensive Reference

---

## Phase 6: Testing Documentation Complete

This comprehensive guide consolidates all testing documentation including unit tests, integration tests, field tests, acceptance criteria, and test automation for the outdoor wheelchair transport robot system.

---

# Part 1: Unit Test Plan

## 1.1 Testing Framework

**C++ Testing:**
- Framework: Google Test (gtest) + Google Mock (gmock)
- ROS 2 Integration: `ament_cmake_gtest`
- Coverage Target: 80% line coverage minimum

**Python Testing:**
- Framework: pytest
- ROS 2 Integration: `launch_testing`
- Coverage Target: 80% line coverage minimum

---

## 1.2 Unit Test Structure

**Test Organization:**
```
package_name/
├── src/
│   └── swerve_module.cpp
├── include/
│   └── package_name/
│       └── swerve_module.hpp
└── test/
    ├── test_swerve_module.cpp       # Unit tests
    ├── test_inverse_kinematics.cpp
    └── mock_hardware_interface.hpp   # Mock objects
```

**Example Unit Test (C++):**
```cpp
// test/test_swerve_module.cpp
#include <gtest/gtest.h>
#include "swerve_drive_controller/swerve_module.hpp"

class SwerveModuleTest : public ::testing::Test {
protected:
    void SetUp() override {
        module_ = std::make_unique<SwerveModule>(
            ModuleID::FRONT_LEFT,
            Eigen::Vector2d(0.3, 0.25)  // Position
        );
    }

    std::unique_ptr<SwerveModule> module_;
};

TEST_F(SwerveModuleTest, InitialStateIsZero) {
    auto state = module_->getModuleState();

    EXPECT_DOUBLE_EQ(state.drive_velocity, 0.0);
    EXPECT_DOUBLE_EQ(state.steering_angle, 0.0);
}

TEST_F(SwerveModuleTest, SetTargetUpdatesState) {
    module_->setTarget(1.0, M_PI / 4);  // 1 m/s at 45°

    auto state = module_->getModuleState();

    EXPECT_DOUBLE_EQ(state.drive_velocity, 1.0);
    EXPECT_NEAR(state.steering_angle, M_PI / 4, 1e-6);
}

TEST_F(SwerveModuleTest, FlipOptimizationReducesRotation) {
    // Set current angle to 0° (forward)
    module_->updateState(0.0, 0.0);

    // Target 170° (almost backward) - should flip to -10° and reverse
    module_->setTarget(1.0, 170.0 * M_PI / 180.0);
    module_->optimizeSteeringAngle();

    auto state = module_->getModuleState();

    // Should be flipped to -10° with reversed drive
    EXPECT_NEAR(state.steering_angle, -10.0 * M_PI / 180.0, 0.01);
    EXPECT_DOUBLE_EQ(state.drive_direction, -1.0);
}

TEST_F(SwerveModuleTest, AlignmentCheckWithinTolerance) {
    module_->updateState(0.5, 0.1);     // Current
    module_->setTarget(0.5, 0.12);       // Target (0.02 rad difference)

    EXPECT_TRUE(module_->isAligned(0.05));   // Within 0.05 rad tolerance
    EXPECT_FALSE(module_->isAligned(0.01));  // Outside 0.01 rad tolerance
}
```

---

## 1.3 Mock Objects

**Mock Hardware Interface:**
```cpp
// test/mock_hardware_interface.hpp
#include <gmock/gmock.h>
#include "hardware_interface/system_interface.hpp"

class MockHardwareInterface : public hardware_interface::SystemInterface {
public:
    MOCK_METHOD(
        hardware_interface::CallbackReturn,
        on_init,
        (const hardware_interface::HardwareInfo&),
        (override)
    );

    MOCK_METHOD(
        std::vector<hardware_interface::StateInterface>,
        export_state_interfaces,
        (),
        (override)
    );

    MOCK_METHOD(
        hardware_interface::return_type,
        read,
        (const rclcpp::Time&, const rclcpp::Duration&),
        (override)
    );

    MOCK_METHOD(
        hardware_interface::return_type,
        write,
        (const rclcpp::Time&, const rclcpp::Duration&),
        (override)
    );
};

// Usage in test
TEST(SwerveControllerTest, WritesCommandsToHardware) {
    MockHardwareInterface mock_hw;

    EXPECT_CALL(mock_hw, write(::testing::_, ::testing::_))
        .Times(1)
        .WillOnce(::testing::Return(hardware_interface::return_type::OK));

    SwerveController controller(mock_hw);
    controller.update();  // Should call write()
}
```

---

## 1.4 Coverage Requirements

**Minimum Coverage: 80%**

**Run Tests with Coverage:**
```bash
# Build with coverage flags
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="--coverage"

# Run tests
colcon test

# Generate coverage report
colcon test-result --verbose
lcov --capture --directory build --output-file coverage.info
lcov --remove coverage.info '/usr/*' '*/test/*' --output-file coverage_filtered.info
genhtml coverage_filtered.info --output-directory coverage_html

# View report
firefox coverage_html/index.html
```

**Coverage Targets by Component:**
- Critical safety systems: 95%+ (emergency stop, collision avoidance)
- Core algorithms: 90%+ (kinematics, localization, path planning)
- ROS interface layers: 70%+ (publishers, subscribers, services)
- Utility functions: 80%+

---

# Part 2: Integration Test Plan

## 2.1 Integration Test Scope

**Subsystem Integration Tests:**
1. Swerve Drive + Hardware Interface
2. Navigation + Localization + Perception
3. Docking Controller + ArUco Detector + Cameras
4. Safety Monitor + All Subsystems
5. Mission Manager + Navigation + Docking

---

## 2.2 ROS 2 Launch Tests

**Example Integration Test:**
```python
# test/test_swerve_integration.py
import unittest
import launch
import launch_testing
import launch_testing.actions
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_test_description():
    """Launch nodes for integration testing."""
    swerve_controller_node = Node(
        package='swerve_drive_controller',
        executable='swerve_node',
        parameters=[{'wheel_base': 0.6, 'track_width': 0.5}]
    )

    mock_hardware_node = Node(
        package='test_support',
        executable='mock_hardware'
    )

    return (
        LaunchDescription([
            swerve_controller_node,
            mock_hardware_node,
            launch_testing.actions.ReadyToTest()
        ]),
        {'swerve_controller': swerve_controller_node}
    )


class TestSwerveIntegration(unittest.TestCase):

    def test_publishes_wheel_commands(self):
        """Test that swerve controller publishes wheel commands."""
        import rclpy
        from std_msgs.msg import Bool

        rclpy.init()
        node = rclpy.create_node('test_node')

        # Subscribe to wheel commands
        received_msgs = []
        def callback(msg):
            received_msgs.append(msg)

        subscription = node.create_subscription(
            SwerveModuleCommandArray,
            '/swerve/wheel_commands',
            callback,
            10
        )

        # Wait for message
        start_time = node.get_clock().now()
        while len(received_msgs) == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (node.get_clock().now() - start_time).nanoseconds > 5e9:  # 5 sec timeout
                break

        self.assertGreater(len(received_msgs), 0, "No wheel commands received")

        # Verify message content
        msg = received_msgs[0]
        self.assertEqual(len(msg.modules), 4, "Should have 4 module commands")

        node.destroy_node()
        rclpy.shutdown()
```

---

## 2.3 Simulation-Based Integration Tests

**Gazebo Simulation Tests:**
```bash
# Launch Gazebo with robot model
ros2 launch wheelchair_robot gazebo.launch.py

# Run integration tests
colcon test --packages-select swerve_drive_controller navigation_integration

# Verify results
colcon test-result --all --verbose
```

**Test Scenarios:**
1. **Straight Line Motion** - Verify robot moves 10m straight
2. **Rotation in Place** - 360° rotation accuracy
3. **Omnidirectional Motion** - Diagonal movement test
4. **Waypoint Navigation** - Navigate through 4 waypoints
5. **Obstacle Avoidance** - Detect and avoid obstacles
6. **Docking Precision** - Achieve ±5mm docking accuracy in simulation

---

# Part 3: Field Test Plan

## 3.1 Safety Protocols

**Pre-Test Safety Checklist:**
- [ ] Emergency stop button tested and functional
- [ ] Safety observer assigned and briefed
- [ ] Test area cleared of unauthorized personnel
- [ ] Weather conditions acceptable (no heavy rain, wind <10 m/s)
- [ ] Robot battery >50% charge
- [ ] Communication link to robot confirmed
- [ ] First aid kit available
- [ ] All sensors publishing data correctly

**Safety Boundaries:**
- Geofence: Robot must stay within designated test area
- Speed limit: 0.5 m/s for initial tests (gradually increase to 1.5 m/s)
- Observer distance: Minimum 2m from robot during autonomous operation
- Abort conditions: Loss of localization, sensor failure, unexpected behavior

---

## 3.2 Outdoor Test Procedures

### Test 1: Basic Mobility (Flat Surface)

**Objective:** Verify swerve drive omnidirectional motion

**Procedure:**
1. Place robot on flat, level surface
2. Enable autonomous mode
3. Command forward motion (1 m/s, 5m distance)
4. Command lateral motion (0.5 m/s, 3m distance)
5. Command diagonal motion (0.7 m/s, 5m diagonal)
6. Command rotation in place (0.5 rad/s, 360°)

**Success Criteria:**
- Position error <10cm
- Orientation error <5°
- No wheel slip detected
- All modules align before driving

**Data Collection:**
- Record `/odom` topic
- Record `/joint_states`
- Record `/cmd_vel` commands
- Video recording of robot motion

---

### Test 2: NDT Localization Accuracy

**Objective:** Verify GPS-free localization over 500m

**Procedure:**
1. Load pre-mapped route (500m outdoor path)
2. Set initial pose (ground truth from RTK GPS for validation only)
3. Execute autonomous navigation
4. Record localization error at 50m intervals

**Success Criteria:**
- Position error <10cm over 500m
- No localization failures
- NDT fitness score <1.0 throughout

**Data Collection:**
- Record `/current_pose` topic
- Record RTK GPS ground truth (for validation)
- Record NDT fitness scores
- Record point cloud data

---

### Test 3: Autonomous Docking

**Objective:** Achieve ±2-5mm wheelchair docking precision

**Setup:**
- Place wheelchair mock-up with 4 ArUco markers
- Initial robot distance: 2-3m from wheelchair

**Procedure:**
1. Initiate docking action
2. Observe approach phase (Nav2)
3. Observe search phase (ArUco detection)
4. Observe visual servoing phase
5. Observe fine alignment phase
6. Measure final docking error

**Success Criteria:**
- Docking success rate >90% (10 trials)
- Position error ±5mm (outdoor), ±2mm (indoor goal)
- Orientation error ±2°
- Docking time <90 seconds

**Data Collection:**
- Record `/docking/status` topic
- Record `/docking/aruco_detections`
- Record camera images
- Measure final position with calipers

---

### Test 4: Full Mission Execution

**Objective:** Complete wheelchair transport mission

**Scenario:**
- Pickup location → Waypoint 1 → Waypoint 2 → Destination
- Total distance: ~200m
- Include docking and undocking

**Procedure:**
1. Start at pickup location
2. Dock wheelchair
3. Navigate to waypoints
4. Navigate to destination
5. Undock wheelchair

**Success Criteria:**
- Mission completion rate >95% (20 trials)
- All waypoints reached within 0.5m
- Docking/undocking success
- Average mission time <10 minutes
- No safety incidents

**Data Collection:**
- Record all ROS topics (`ros2 bag record -a`)
- Video recording of entire mission
- GPS trace (for visualization)
- Battery consumption data

---

### Test 5: Weather Resilience

**Objective:** Verify light rain operation

**Conditions:**
- Light rain (precipitation <2mm/hr)
- Temperature 10-30°C
- Wind <5 m/s

**Procedure:**
1. Execute Test 2 (localization) in light rain
2. Execute Test 3 (docking) in light rain
3. Monitor for water ingress (IP54 rating)

**Success Criteria:**
- Localization accuracy maintained
- Docking success rate >80%
- No hardware failures
- All electronics remain dry

---

## 3.3 Data Collection Methods

**ROS Bag Recording:**
```bash
# Record all topics
ros2 bag record -a -o field_test_001

# Record specific topics
ros2 bag record /odom /current_pose /sensors/lidar/points \
    /docking/status /safety/status -o docking_test_001

# Play back for analysis
ros2 bag play field_test_001.db3
```

**Performance Metrics:**
- Position accuracy (RMSE)
- Localization fitness scores
- Docking precision (mean, std dev)
- Mission completion time
- Battery consumption (Wh/km)
- Sensor data quality

---

# Part 4: Acceptance Criteria

## 4.1 MVP (Minimum Viable Product) Acceptance

### 4.1.1 Functional Requirements

**Swerve Drive:**
- [ ] Omnidirectional motion in all directions
- [ ] Maximum speed: 1.5 m/s (no passenger)
- [ ] Module alignment <5° before driving
- [ ] Odometry drift <10% over 100m

**Navigation:**
- [ ] Load and use pre-mapped environments
- [ ] NDT localization accuracy ±10cm over 500m
- [ ] Navigate through 4+ waypoints autonomously
- [ ] Obstacle avoidance (stop distance >1m)

**Docking:**
- [ ] Detect 4 ArUco markers at 2m distance
- [ ] Docking precision ±10mm (MVP), ±5mm (goal)
- [ ] Docking success rate >80%
- [ ] Docking time <120 seconds

**Safety:**
- [ ] Emergency stop response time <100ms
- [ ] Velocity limiting based on passenger status
- [ ] Watchdog detects subsystem failures within 2s
- [ ] Safe stop on localization loss

**UI/eHMI:**
- [ ] Touch screen displays mission status
- [ ] Emergency stop button functional
- [ ] eHMI displays wheelchair states (21-28)
- [ ] Multi-language support (EN, JP)

---

### 4.1.2 Performance Benchmarks (MVP)

| Metric | Target | Method |
|--------|--------|--------|
| Max Speed (Empty) | 1.5 m/s | Field test |
| Max Speed (Passenger) | 1.0 m/s | Field test |
| Localization Error (500m) | <10 cm | RTK GPS validation |
| Docking Precision | ±10 mm | Caliper measurement |
| Docking Success Rate | >80% | 10 trials |
| Mission Completion Rate | >90% | 20 trials |
| Battery Range | >10 km | Field test |
| E-Stop Response Time | <100 ms | Oscilloscope |
| Obstacle Detection Range | >5 m | LiDAR test |

---

## 4.2 Production Acceptance Criteria

### 4.2.1 Enhanced Performance

| Metric | MVP | Production |
|--------|-----|------------|
| Docking Precision | ±10mm | ±5mm |
| Docking Success Rate | >80% | >95% |
| Mission Completion Rate | >90% | >98% |
| Localization Error (500m) | <10cm | <5cm |
| Battery Range | >10km | >15km |
| Weather Operation | Dry | Light rain |
| MTBF (Mean Time Between Failures) | 100 hours | 500 hours |

---

### 4.2.2 Safety Certification

**Required Certifications:**
- [ ] ISO 13482 (Robotics - Safety for personal care robots)
- [ ] IEC 61508 (Functional Safety - SIL 2)
- [ ] IP54 Ingress Protection rating
- [ ] EMC compliance (electromagnetic compatibility)
- [ ] Wheelchair restraint system certification

**Safety Testing:**
- [ ] Emergency stop function tested 1000+ cycles
- [ ] Collision avoidance tested with various obstacles
- [ ] Fail-safe behavior verified for all sensor failures
- [ ] Battery management system safety tested
- [ ] Passenger safety restraints load tested

---

## 4.3 Field Validation Criteria

**Production Readiness:**
- [ ] 100+ autonomous missions completed successfully
- [ ] 500+ km traveled autonomously without critical failures
- [ ] Tested in 3+ different outdoor environments
- [ ] Tested in various weather conditions (dry, light rain)
- [ ] Operator training program validated
- [ ] Maintenance procedures documented and tested

---

# Part 5: Test Automation

## 5.1 CI/CD Pipeline

**GitHub Actions Workflow:**
```yaml
# .github/workflows/ci.yml
name: ROS 2 CI

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main ]

jobs:
  build-and-test:
    runs-on: ubuntu-22.04
    container:
      image: ros:humble

    steps:
      - name: Checkout code
        uses: actions/checkout@v3

      - name: Install dependencies
        run: |
          apt-get update
          rosdep update
          rosdep install --from-paths src --ignore-src -r -y

      - name: Build workspace
        run: |
          . /opt/ros/humble/setup.sh
          colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

      - name: Run tests
        run: |
          . install/setup.sh
          colcon test --return-code-on-test-failure

      - name: Test results
        run: colcon test-result --verbose

      - name: Coverage report
        run: |
          colcon build --cmake-args -DCMAKE_CXX_FLAGS="--coverage"
          colcon test
          lcov --capture --directory build --output-file coverage.info
          lcov --remove coverage.info '/usr/*' '*/test/*' -o coverage_filtered.info

      - name: Upload coverage
        uses: codecov/codecov-action@v3
        with:
          files: coverage_filtered.info
```

---

## 5.2 Automated Test Execution

**Test Runner Script:**
```bash
#!/bin/bash
# scripts/run_tests.sh

set -e

echo "Building workspace..."
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="--coverage"

echo "Running unit tests..."
colcon test --packages-select-regex ".*_test"

echo "Running integration tests..."
colcon test --packages-select-regex ".*_integration"

echo "Generating coverage report..."
lcov --capture --directory build --output-file coverage.info
lcov --remove coverage.info '/usr/*' '*/test/*' --output-file coverage_filtered.info
genhtml coverage_filtered.info --output-directory coverage_html

echo "Test summary:"
colcon test-result --all --verbose

echo "Coverage summary:"
lcov --summary coverage_filtered.info

echo "Done! View coverage report at: coverage_html/index.html"
```

---

## 5.3 Continuous Monitoring

**Test Metrics Dashboard:**
- Test pass rate (target: >99%)
- Code coverage (target: >80%)
- Build time trend
- Test execution time
- Flaky test detection

**Automated Alerts:**
- Test failures on main branch
- Coverage drops >5%
- Build time increases >20%
- New compiler warnings

---

## 5.4 Regression Testing

**Automated Regression Suite:**
1. Run full test suite on every PR
2. Run extended tests nightly (includes simulation tests)
3. Run field test suite weekly (automated robot tests)

**Test Selection Strategy:**
- Fast tests (<1 min): Run on every commit
- Medium tests (1-10 min): Run on PR
- Slow tests (>10 min): Run nightly
- Hardware tests: Run weekly

---

**Document Status:** Complete
**Phase 6 Status:** 100% Complete (Consolidated)
**Approvals Required:** QA Lead, Test Engineer, Safety Engineer
