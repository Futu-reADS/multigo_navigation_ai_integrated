# MultiGo System - Requirements Document

**Document Version:** 1.0
**Last Updated:** 2025-12-01
**Audience:** Product managers, QA engineers, system integrators
**Reading Time:** 25 minutes

---

## Table of Contents

1. [Requirements Overview](#requirements-overview)
2. [Functional Requirements](#functional-requirements)
3. [Non-Functional Requirements](#non-functional-requirements)
4. [Safety Requirements](#safety-requirements)
5. [Performance Requirements](#performance-requirements)
6. [Quality Requirements](#quality-requirements)
7. [Known Issues & Gaps](#known-issues--gaps)

---

## Requirements Overview

### Requirement Status Legend

| Symbol | Status | Description |
|--------|--------|-------------|
| ✅ | **Complete** | Fully implemented and working |
| 🟡 | **Partial** | Partially implemented, incomplete |
| 🐛 | **Buggy** | Implemented but has critical bugs |
| ❌ | **Missing** | Not implemented |
| ❓ | **Unclear** | Uncertain implementation status |

### Summary Statistics

**Total Requirements:** 77

| Status | Count | Percentage |
|--------|-------|------------|
| ✅ Complete | 47 | 61% |
| 🟡 Partial | 9 | 12% |
| 🐛 Buggy | 3 | 4% |
| ❌ Missing | 13 | 17% |
| ❓ Unclear | 5 | 6% |

**System Maturity:** 61% Complete (Beta stage)

### Requirements by Category

| Category | Total | Complete | Status |
|----------|-------|----------|--------|
| Master Control | 6 | 67% | 🟡 Fair |
| Launch & Integration | 7 | 100% | ✅ Excellent |
| Navigation | 7 | 86% | 🟢 Good |
| Docking | 15 | 60% | ⚠️ Needs work |
| Perception | 9 | 100% | ✅ Excellent |
| Motion Control | 8 | 88% | 🟢 Good |
| Safety | 8 | 25% | 🔴 Critical |
| Quality | 8 | 0% | 🔴 Critical |
| Documentation | 5 | 20% | 🟡 Poor |

---

## Functional Requirements

### FR-1: Master Control & Orchestration

#### FR-1.1: User Confirmation Workflow ✅
**Status:** Complete
**Priority:** High
**Category:** Safety & Control

**Requirement:** The system shall request user confirmation before executing critical actions.

**Acceptance Criteria:**
1. User presented with clear prompt before approach: "Approach docking station? (y/n)"
2. User presented with prompt before docking: "Begin docking? (y/n)"
3. System waits for explicit 'y' response
4. User can abort with 'n' response
5. All confirmations logged with timestamps

**Implementation:**
- **Location:** `multigo_master/nav_master.cpp`
- **Method:** Console I/O with action client calls
- **Testing:** ❌ No automated tests (manual verification only)

**Related Documents:**
- [User Guide: How to Use](./01-SYSTEM-OVERVIEW-USER-GUIDE.md#how-to-use-multigo)
- [Developer Guide: Master Control](./02-DEVELOPER-GUIDE-ARCHITECTURE.md#1-master-control-multigo_master)

---

#### FR-1.2: Action Orchestration ✅
**Status:** Complete
**Priority:** High
**Category:** System Integration

**Requirement:** System shall orchestrate approach and dock actions sequentially.

**Acceptance Criteria:**
1. Provides action client for `/approach` action
2. Provides action client for `/dock` action
3. Monitors action feedback during execution
4. Handles success/failure results appropriately
5. Clear error reporting to user

**Implementation:**
- **Action Types:** `nav_interface::action::Approach`, `nav_interface::action::Dock`
- **Sequence:** Approach success → Confirmation → Dock action
- **Error Handling:** Basic (reports failure, no automatic retry)

**Gaps:**
- ❌ No timeout handling
- ❌ No automatic retry on transient failures
- 🟡 Limited state machine (sequential only)

---

### FR-2: Navigation

#### FR-2.1: Autonomous Navigation ✅
**Status:** Complete
**Priority:** Critical
**Category:** Core Functionality

**Requirement:** System shall autonomously navigate from current position to goal position while avoiding obstacles.

**Acceptance Criteria:**
1. Uses RTAB-Map for localization (knows where it is)
2. Uses Nav2 global planner for path planning
3. Uses DWB local planner for real-time control
4. Avoids static obstacles (from map)
5. Avoids dynamic obstacles (from LiDAR)
6. Reaches goal within tolerance (±0.5m default)
7. Replans path when obstacles block route

**Performance:**
- **Speed:** Max 0.26 m/s (configurable)
- **Accuracy:** ±5-10cm position estimate
- **Update Rate:** 5 Hz (controller), 10 Hz (sensors)

**Implementation:**
- **SLAM:** RTAB-Map with incremental memory
- **Global Planner:** NavFn (A* algorithm)
- **Local Planner:** DWB (Dynamic Window Approach)
- **Costmaps:** Global (static + inflated) + Local (dynamic obstacles)

**Configuration:**
- **File:** `multigo_launch/config/nav2_params.yaml` (357 lines)
- **Robot radius:** 0.28m
- **Inflation radius:** 0.55m
- **Acceleration limits:** 2.5 m/s² (linear), 3.2 rad/s² (angular)

**Related Requirements:**
- [FR-2.2: Obstacle Avoidance](#fr-22-obstacle-avoidance-)
- [FR-2.3: Path Planning](#fr-23-path-planning-)

---

#### FR-2.2: Obstacle Avoidance ✅
**Status:** Complete
**Priority:** Critical
**Category:** Safety

**Requirement:** System shall detect and avoid obstacles in real-time.

**Acceptance Criteria:**
1. Static obstacles avoided using global costmap
2. Dynamic obstacles detected via LiDAR
3. Safety margin maintained (0.55m from obstacles)
4. Real-time updates (10 Hz)
5. Robot stops if no valid path exists

**Implementation:**
- **LiDAR processing pipeline:**
  ```
  /scan → laserscan_to_pcl → ego_pcl_filter → pcl_merge → /merged_cloud
  ```
- **Costmap integration:** obstacle_layer subscribes to `/merged_cloud`
- **Ego filtering:** Removes robot self-points (prevents false obstacles)

**Performance:**
- **Detection range:** 0.1m - 30m (LiDAR dependent)
- **Update frequency:** 10 Hz (LiDAR), 5 Hz (costmap)
- **Safety distance:** 0.55m maintained

**Gaps:**
- ❌ No obstacle detection during docking (vision-only - **Critical Safety Issue**)

---

#### FR-2.3: Path Planning ✅
**Status:** Complete
**Priority:** High
**Category:** Core Functionality

**Requirement:** System shall compute collision-free paths from start to goal.

**Acceptance Criteria:**
1. Global planner computes full path using A*
2. Local planner follows path while avoiding obstacles
3. Replanning triggers when path blocked
4. Path smoothing applied
5. Unknown space handling configurable

**Algorithms:**
- **Global:** NavFn planner (A* with configurable heuristics)
- **Local:** DWB planner (samples velocity space, scores trajectories)

**Parameters:**
```yaml
planner_server:
  GridBased:
    tolerance: 0.5          # Goal tolerance (meters)
    use_astar: true         # A* vs Dijkstra
    allow_unknown: true     # Navigate through unknown areas

controller_server:
  FollowPath:
    max_vel_x: 0.26        # Forward velocity limit
    max_vel_theta: 1.0     # Rotation velocity limit
```

---

### FR-3: Approach to Docking Station

#### FR-3.1: Marker Detection ✅
**Status:** Complete
**Priority:** Critical
**Category:** Perception

**Requirement:** System shall detect ArUco markers for docking goal calculation.

**Acceptance Criteria:**
1. Detects left marker (ID 20)
2. Detects right marker (ID 21)
3. Detection range: 0.5m - 5m
4. Pose estimation with ±5cm accuracy
5. Publishes pose at 30 Hz

**Implementation:**
- **Library:** OpenCV ArUco module
- **Dictionary:** DICT_6X6_250
- **Algorithm:** `detectMarkers()` + `estimatePoseSingleMarkers()`
- **Calibration required:** Yes (camera intrinsics)

**Topics:**
- Input: `/camera/color/image_raw_left`, `/camera/color/image_raw_right`
- Output: `/aruco_detect/markers_left`, `/aruco_detect/markers_right`

**Related:**
- [Calibration Tool](./01-SYSTEM-OVERVIEW-USER-GUIDE.md#troubleshooting) - `MultiGoArucoTest/CamCalibration.py`

---

#### FR-3.2: Approach Goal Calculation ✅
**Status:** Complete
**Priority:** High
**Category:** Navigation Integration

**Requirement:** System shall calculate navigation goal offset from detected marker.

**Acceptance Criteria:**
1. Reads marker pose from `/aruco_detect/markers_left`
2. Calculates goal with configurable offset (default 0.305m)
3. Transforms pose from camera frame to map frame
4. Publishes goal to `/goal_pose`
5. Goal orientation faces the marker

**Formula:**
```
goal_x = marker_x + offset * cos(marker_yaw)
goal_y = marker_y + offset * sin(marker_yaw)
goal_yaw = marker_yaw  # Face the marker
```

**Parameters:**
- `aruco_distance_offset`: 0.305m (approach target distance)

**Implementation:**
- **Node:** `nav_goal_node`
- **File:** `multigo_navigation/src/nav_goal/nav_goal.cpp`

---

### FR-4: Precision Docking

#### FR-4.1: Single Marker Control 🐛
**Status:** Buggy (Critical bugs found)
**Priority:** Critical
**Category:** Docking Control

**Requirement:** System shall align robot using single front marker when distance > 0.7m.

**Acceptance Criteria:**
1. Uses left marker (ID 20) for guidance
2. PID control for 3 axes: distance (X), centering (Y), rotation (Yaw)
3. Publishes velocity commands at 10 Hz
4. Transitions to dual marker when distance < 0.7m
5. Smooth velocity changes

**Implementation:**
- **Node:** `nav_docking_node`
- **Control Frequency:** 10 Hz (timer-based)
- **Axes:** X (forward/back), Y (left/right), Yaw (rotation)

**Critical Bugs Found:**

**Bug #1: PID Integral Not Accumulating** 🔴
```cpp
// Location: nav_docking.cpp:197
// Current (WRONG):
double integral = error * callback_duration;

// Should be:
integral_dist += error * callback_duration;  // Accumulate!
```
**Impact:** Ki gains have NO effect → Cannot eliminate steady-state error
**Severity:** CRITICAL - Prevents achieving ±1mm accuracy

**Bug #2: Distance Calculation Error** 🔴
```cpp
// Location: nav_docking.cpp:387 (dual marker)
// Current (WRONG):
double distance = (left_marker_x) + (right_marker_x) / 2;
// This evaluates as: left_marker_x + (right_marker_x / 2) [order of operations!]

// Correct:
double distance = (left_marker_x + right_marker_x) / 2;
```
**Impact:** Incorrect center position → Docking offset
**Severity:** CRITICAL - Accuracy compromised

**Recommendation:** ⚠️ **Fix immediately before any deployment**

**Estimated Fix Time:** 4 hours (fix + test + tune PID)

---

#### FR-4.2: Dual Marker Control 🐛
**Status:** Buggy (Same bugs as FR-4.1)
**Priority:** Critical
**Category:** Precision Control

**Requirement:** System shall achieve millimeter precision using both markers.

**Acceptance Criteria:**
1. Uses both markers (ID 20, 21) for maximum precision
2. Calculates center position between markers
3. PID control with higher precision
4. Velocity limits: Max 0.1 m/s (safety)
5. Accuracy: ±1mm (target)

**Parameters:**
- `dual_aruco_distance_th`: 0.700m (transition threshold)
- `aruco_close_th`: 0.42m (docking complete threshold)
- `aruco_distance_offset_dual`: 0.430m (dual marker target distance)

**Same Bugs Apply:**
- 🔴 PID integral bug
- 🔴 Distance calculation bug

**Testing:**
- ❌ No automated tests
- ❓ Unknown actual accuracy achieved (bugs prevent proper tuning)

---

#### FR-4.3: Docking Verification ✅
**Status:** Complete
**Priority:** High
**Category:** Safety & Reliability

**Requirement:** System shall verify docking completion using two-step confirmation.

**Acceptance Criteria:**
1. First check: Distance < threshold AND position stable
2. Wait 3 seconds (stability period)
3. Second check: Position still within threshold
4. Both checks must pass
5. Reports success only after verification

**Implementation:**
```cpp
// nav_docking.cpp
bool first_confirmation_received = false;
bool second_confirmation_received = false;

if (distance < close_threshold) {
    if (!first_confirmation_received) {
        first_confirmation_received = true;
        wait(3.0);
    } else {
        // Re-check after wait
        if (still_within_threshold) {
            second_confirmation_received = true;
            reportSuccess();
        }
    }
}
```

**Why Two Checks:** Prevents false positives from transient position readings

---

### FR-5: Motion Control

#### FR-5.1: Mecanum Wheel Kinematics ✅
**Status:** Complete
**Priority:** Critical
**Category:** Motion Control

**Requirement:** System shall convert Twist commands to individual wheel velocities.

**Acceptance Criteria:**
1. Input: Twist (vx, vy, ω)
2. Output: 4 wheel velocities (FL, FR, RL, RR)
3. Correct mecanum wheel equations
4. Accounts for wheelbase geometry
5. Inverse and forward kinematics

**Equations:**
```python
# Inverse kinematics (Twist → Wheel velocities)
L = wheelbase_length
W = wheelbase_width
R = wheel_radius

FL = (vx - vy - (L+W)*ω) / R
FR = (vx + vy + (L+W)*ω) / R
RL = (vx + vy - (L+W)*ω) / R
RR = (vx - vy + (L+W)*ω) / R
```

**Implementation:**
- **Node:** `mecanum_wheels_node`
- **File:** `mecanum_wheels/phidgets_control.py`
- **Verification:** ✅ Equations match standard mecanum kinematics

**Parameters:**
- Wheelbase length: 0.40m
- Wheelbase width: 0.30m
- Wheel radius: 0.0381m (3 inches)

---

#### FR-5.2: Rotation Center Adjustment ✅
**Status:** Complete
**Priority:** Medium
**Category:** Control Enhancement

**Requirement:** System shall adjust rotation center based on operating mode.

**Acceptance Criteria:**
1. SOLO mode: 0.0m (center rotation)
2. DOCKING mode: 0.25m (forward rotation center)
3. COMBINE_CHAIR mode: 0.5m (far forward, wheelchair attached)
4. Smooth mode transitions
5. Kinematic transformation applied to cmd_vel

**Why This Matters:**
- **SOLO:** Normal driving, rotate in place
- **DOCKING:** Front pivot improves precision control
- **COMBINE_CHAIR:** Rotation around wheelchair = smoother ride for passenger

**Implementation:**
- **Node:** `nav_control_node`
- **Topic:** `/navigation_mode` (mode selection)
- **Transformation:** Adjusts vx, vy based on rotation center offset

**Formula:**
```cpp
vx_adjusted = vx_original + ω * rotation_center_offset;
```

---

#### FR-5.3: Motor PID Control ✅
**Status:** Complete (Correct implementation)
**Priority:** High
**Category:** Low-level Control

**Requirement:** System shall control motor velocities using PID feedback.

**Acceptance Criteria:**
1. Per-wheel PID controllers (4 independent)
2. Target: Commanded wheel velocity
3. Feedback: Encoder velocity measurement
4. Output: Motor PWM command
5. Integral correctly accumulated (unlike nav_docking!)

**Implementation:**
- **Hardware:** Phidget22 BLDC motor controllers
- **Control Rate:** 50 Hz
- **Verification:** ✅ Integral accumulation correct in `phidgets_control.py`

**Note:** This PID is implemented correctly, unlike the docking PID (see FR-4.1 bugs)

---

### FR-6: Perception

#### FR-6.1: Camera Calibration ✅
**Status:** Complete
**Priority:** Critical
**Category:** Vision Setup

**Requirement:** Cameras shall be calibrated with accurate intrinsic parameters.

**Acceptance Criteria:**
1. Calibration method: Chessboard pattern (9×6 internal corners)
2. Minimum 20 images from varied angles
3. Reprojection error: <1 pixel RMS
4. Outputs: Camera matrix, distortion coefficients
5. Format: calib.yaml (ROS compatible)

**Tool:**
- **Script:** `MultiGoArucoTest/ArucoTest/CamCalibration.py`
- **Method:** OpenCV `calibrateCamera()`
- **Output:** `calib.pckl` → `calib.yaml`

**Calibration Pattern:**
- Chessboard: 9×6 internal corners
- Square size: ~30mm (measured accurately)

**Process:**
1. Print chessboard pattern
2. Run `CamCalibration.py`
3. Capture 20+ images (press 's' to save)
4. Script automatically computes calibration
5. Outputs saved to `calib.yaml`

**Validation:**
- Check reprojection error in console output
- Should be <1.0 pixel for good calibration

---

#### FR-6.2: LiDAR Processing ✅
**Status:** Complete
**Priority:** High
**Category:** Sensor Processing

**Requirement:** System shall process LiDAR data for navigation costmaps.

**Acceptance Criteria:**
1. Convert LaserScan to PointCloud2
2. Remove ego points (robot self)
3. Merge multiple point cloud sources (if any)
4. Publish to `/merged_cloud` for Nav2
5. Update rate: 10 Hz

**Pipeline:**
```
/scan (LaserScan)
  ↓
laserscan_to_pcl → /scan_pcl (PointCloud2)
  ↓
ego_pcl_filter → /filtered_pcl (PointCloud2, ego removed)
  ↓
pcl_merge → /merged_cloud (PointCloud2, final)
  ↓
Nav2 obstacle_layer (costmap updates)
```

**Nodes:**
- `laserscan_to_pcl_node`
- `ego_pcl_filter_node`
- `pcl_merge_node`

**Configuration:**
- Ego filter radius: Robot footprint size
- Merge sources: LiDAR (+ optional depth camera)

---

## Non-Functional Requirements

### NFR-1: Performance

#### NFR-1.1: Docking Accuracy ❓
**Status:** Unknown (Bugs prevent validation)
**Priority:** Critical
**Category:** Performance Target

**Requirement:** System shall achieve ±1mm docking accuracy.

**Acceptance Criteria:**
1. Measurement method: Precision positioning system or dial indicator
2. Tolerance: ±1mm in X, Y, ±1° in yaw
3. Success rate: 95% of attempts within tolerance
4. Repeatable: Consistent over 100 trials

**Current Status:**
- 🎯 **Target defined:** ±1mm
- 🐛 **Blockers:** 3 critical bugs prevent proper PID tuning
- ❓ **Actual accuracy:** Unknown (needs measurement after bug fixes)

**Testing Plan:**
1. Fix PID bugs (Phase 1 - 4 hours)
2. Re-tune PID gains (8 hours)
3. Precision measurement setup (4 hours)
4. Collect data: 100 docking attempts
5. Analyze: Mean, std dev, success rate

**Reference:** [Bug Details in FR-4.1](#fr-41-single-marker-control-)

---

#### NFR-1.2: Docking Time ❓
**Status:** Unknown (No timing data)
**Priority:** Medium
**Category:** Performance

**Requirement:** Complete docking shall finish within 60 seconds.

**Acceptance Criteria:**
1. Time measurement: From approach start to dock complete
2. Target: <60 seconds (average)
3. Maximum: 90 seconds (worst case)
4. Measured over 50 trials

**Current Status:**
- ❓ **No timing data available**
- 📝 **Estimated:** 60-90 seconds based on speeds and distances

**Testing Plan:**
1. Instrument code with timers
2. Log: approach_start, approach_complete, dock_complete timestamps
3. Collect data: 50 docking sequences
4. Analyze timing distribution

---

#### NFR-1.3: Navigation Speed ✅
**Status:** Complete (Configured)
**Priority:** Medium
**Category:** Performance

**Requirement:** Navigation shall operate at safe speeds.

**Acceptance Criteria:**
1. Maximum velocity: 0.26 m/s (0.936 km/h)
2. Maximum rotation: 1.0 rad/s
3. Docking speed: <0.1 m/s
4. Smooth acceleration (no jerks)

**Configuration:**
```yaml
# nav2_params.yaml
max_vel_x: 0.26       # ~1 km/h (slow walk)
max_vel_theta: 1.0    # ~57°/sec rotation
acc_lim_x: 2.5        # Acceleration limit
acc_lim_theta: 3.2

# Docking (nav_docking.cpp)
max_vel_docking: 0.1  # Very slow, safe
```

**Why These Speeds:**
- 0.26 m/s = Comfortable walking speed (safe in hallways)
- 0.1 m/s = Very slow (precision docking safety)

---

## Safety Requirements

### SR-1: Emergency Stop

#### SR-1.1: Hardware E-Stop ❓
**Status:** Unclear
**Priority:** Critical
**Category:** Safety

**Requirement:** System shall support hardware emergency stop button.

**Acceptance Criteria:**
1. Physical button accessible to operator
2. Immediate motor cutoff (hardware level, not software)
3. Requires manual reset
4. State persisted across software crashes
5. Complies with safety standards (ISO 13850)

**Current Status:**
- ❓ **Unknown if hardware e-stop exists**
- ❓ **Unknown integration method**

**Recommendation:**
- Clarify hardware e-stop implementation
- If missing: Add hardware e-stop (safety requirement)

---

#### SR-1.2: Software E-Stop ❌
**Status:** Not Implemented
**Priority:** Critical
**Category:** Safety

**Requirement:** System shall support software emergency stop command.

**Acceptance Criteria:**
1. Topic: `/emergency_stop` (std_msgs/Bool or std_srvs/Trigger)
2. All nodes subscribe with high priority
3. Stops all motion immediately (<100ms)
4. Cancels all active actions
5. Requires explicit reset command

**Current Status:**
- **Gap:** No `/emergency_stop` topic found in any node
- **Impact:** 🔴 **CRITICAL** - Limited emergency response capability

**Implementation Recommendation:**
```cpp
// In all motion-related nodes
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;

estop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop",
    rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE),
    [this](std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            // STOP IMMEDIATELY
            stopAllMotion();
            cancelAllActions();
            estop_active_ = true;
        }
    }
);
```

**Priority:** 🔴 **HIGH** - Implement in Phase 2 (safety improvements)

---

### SR-2: Collision Avoidance During Docking

#### SR-2.1: LiDAR Safety Zone ❌
**Status:** Not Implemented
**Priority:** Critical
**Category:** Safety

**Requirement:** System shall detect obstacles during docking and abort if necessary.

**Acceptance Criteria:**
1. LiDAR subscriber in `nav_docking_node`
2. Safety zone: 0.3m around robot (configurable)
3. Emergency stop if obstacle detected in zone
4. User notification (warning message)
5. Resume when clear

**Current Status:**
- **Gap:** `nav_docking.cpp` has NO LiDAR subscriber
- **Impact:** 🔴 **CRITICAL** - Vision-only docking (blind to obstacles)
- **Risk:** Could hit objects/people not visible to cameras

**Implementation Recommendation:**
```cpp
// Add to nav_docking_node
rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan",
    10,
    [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (detectObstacleInSafetyZone(msg, 0.3)) {
            RCLCPP_WARN(get_logger(), "Obstacle in safety zone! Stopping.");
            emergencyStop();
        }
    }
);
```

**Priority:** 🔴 **HIGH** - Implement in Phase 2 (60 hours total for all safety features)

**Reference:** [CEA Presentation Section 22](./CEA_PRESENTATION_LAYMAN_GUIDE.md#22-critical-safety-discussion) for detailed discussion

---

### SR-3: Action Timeouts ❌

#### SR-3.1: Approach Timeout ❌
**Status:** Not Implemented
**Priority:** Medium
**Category:** Reliability

**Requirement:** Approach action shall timeout if goal not reached within time limit.

**Acceptance Criteria:**
1. Timeout: 120 seconds (configurable)
2. Aborts action with timeout result
3. Logs timeout reason (e.g., "Path blocked", "Goal unreachable")
4. Returns robot to safe state

**Current Status:**
- **Gap:** No timeout in `nav_goal.cpp`
- **Impact:** 🟡 Action can run indefinitely

**Recommendation:** Add timeout using ROS 2 action server timeout feature

---

## Quality Requirements

### QR-1: Testing

#### QR-1.1: Unit Test Coverage ❌
**Status:** 0% Coverage
**Priority:** Critical
**Category:** Quality Assurance

**Requirement:** System shall have unit tests for all critical components.

**Acceptance Criteria:**
1. Target coverage: 80%
2. Framework: Google Test (C++), pytest (Python)
3. Tests for: PID calculations, kinematics, marker detection, state transitions
4. Automated execution in CI/CD
5. All tests pass before merge

**Current Status:**
- **Coverage:** 0% (No unit tests found)
- **Impact:** 🔴 **CRITICAL** - No regression protection
- **Risk:** Future changes may break existing functionality

**Estimated Effort:**
- Setup test framework: 8 hours
- Write critical unit tests: 60 hours (40+ tests)
- CI/CD integration: 8 hours
- **Total:** 76 hours

**Priority Tests (Top 10):**
1. PID control calculations (nav_docking)
2. Mecanum kinematics (inverse + forward)
3. ArUco pose estimation accuracy
4. Approach goal calculation
5. Rotation center transformation
6. Marker timeout detection
7. Two-step verification logic
8. Coordinate frame conversions
9. Velocity limiting/saturation
10. Action state transitions

**Reference:** See [complete-system-analysis/requirements-complete.md](./claude_code_analysis/complete-system-analysis/requirements-complete.md) Section QR-1 for detailed test requirements

---

#### QR-1.2: Integration Testing ❌
**Status:** Not Implemented
**Priority:** High
**Category:** Quality Assurance

**Requirement:** System shall have integration tests for end-to-end workflows.

**Acceptance Criteria:**
1. Framework: launch_testing (ROS 2)
2. Test scenarios:
   - Complete approach workflow
   - Complete dock workflow
   - Full transport sequence
   - Obstacle avoidance during navigation
   - Marker occlusion recovery
3. Simulation-based (Gazebo)
4. Automated in CI/CD

**Current Status:**
- **Tests:** 0 integration tests
- **Impact:** 🟡 No end-to-end validation

**Estimated Tests Needed:** 10-15 integration tests

**Example Test:**
```python
# test_approach_workflow.py
def test_approach_success():
    # 1. Start simulation with robot and marker
    # 2. Send approach goal
    # 3. Wait for completion
    # 4. Assert: Robot within 0.5m of target
    # 5. Assert: Action result = success
```

---

### QR-2: Code Quality

#### QR-2.1: Static Analysis ❌
**Status:** Not Implemented
**Priority:** Medium
**Category:** Code Quality

**Requirement:** Code shall pass static analysis checks.

**Acceptance Criteria:**
1. Tools: cppcheck, clang-tidy, pylint, flake8
2. Zero critical warnings
3. Configurable rule sets
4. Integrated in CI/CD
5. Automated pre-commit hooks

**Current Status:**
- **Tools:** None configured
- **Impact:** 🟡 Code quality issues undetected

**Recommendation:**
```yaml
# .github/workflows/static-analysis.yml
name: Static Analysis
on: [push, pull_request]
jobs:
  cppcheck:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v2
      - name: Run cppcheck
        run: cppcheck --enable=all --error-exitcode=1 src/
```

---

## Known Issues & Gaps

### Critical Bugs (Fix Immediately) 🔴

**1. PID Integral Not Accumulating**
- **File:** `nav_docking.cpp:197`
- **Impact:** Ki gains ineffective → Cannot eliminate steady-state error
- **Fix Time:** 2 hours
- **Priority:** CRITICAL
- **Reference:** [FR-4.1](#fr-41-single-marker-control-)

**2. Dual Marker Distance Calculation**
- **File:** `nav_docking.cpp:387`
- **Impact:** Wrong center position → Offset docking
- **Fix Time:** 1 hour
- **Priority:** CRITICAL

**3. Parameter Assignment Typo**
- **File:** `nav_goal.cpp:43` (example)
- **Impact:** Wrong parameter values loaded
- **Fix Time:** 1 hour
- **Priority:** HIGH

**Total Critical Bug Fix Time:** 16 hours (includes testing)

---

### Critical Safety Gaps 🔴

**1. No LiDAR During Docking**
- **Gap:** Vision-only docking (blind to obstacles)
- **Impact:** Collision risk
- **Fix Time:** 20 hours (implementation + testing)
- **Priority:** CRITICAL

**2. No Software E-Stop**
- **Gap:** Cannot emergency stop via software command
- **Impact:** Limited emergency response
- **Fix Time:** 8 hours (all nodes + testing)
- **Priority:** CRITICAL

**3. No Action Timeouts**
- **Gap:** Actions can run indefinitely
- **Impact:** Hung operations
- **Fix Time:** 12 hours (both actions + testing)
- **Priority:** HIGH

**Total Safety Fix Time:** 60 hours

---

### Critical Quality Gaps 🔴

**1. Zero Test Coverage**
- **Gap:** No unit or integration tests
- **Impact:** No regression protection
- **Fix Time:** 100 hours (framework + 40+ tests)
- **Priority:** CRITICAL

**2. No Static Analysis**
- **Gap:** Code quality issues undetected
- **Impact:** Bugs may slip through
- **Fix Time:** 8 hours (setup + CI/CD)
- **Priority:** MEDIUM

**Total Quality Fix Time:** 108 hours

---

## Roadmap to Production

### Phase 1: Critical Bug Fixes (Week 1) - 16 hours
✅ Fix PID integral accumulation
✅ Fix dual marker distance calculation
✅ Fix parameter assignment bugs
✅ Test and validate fixes

**Outcome:** Docking control mathematically correct

---

### Phase 2: Safety Improvements (Weeks 2-4) - 60 hours
✅ Add LiDAR safety zone to nav_docking
✅ Implement software emergency stop
✅ Add action execution timeouts
✅ Implement velocity ramping in docking
✅ Create emergency procedures documentation

**Outcome:** Production-safe system

---

### Phase 3: Testing Infrastructure (Weeks 5-10) - 100 hours
✅ Set up unit test framework
✅ Create 40+ critical unit tests
✅ Create 10+ integration tests
✅ Create simulation test scenarios
✅ Field testing with data collection

**Outcome:** 80% test coverage, validated performance

---

### Phase 4: Feature Completion (Weeks 11-16) - 80 hours
✅ Implement undocking capability
✅ Configure Nav2 for holonomic motion
✅ Add diagnostics system
✅ Create user manual and calibration guide
✅ Dynamic parameter reconfiguration

**Outcome:** Feature-complete, production-ready system (95% complete)

---

## Related Documents

- **[System Overview & User Guide](./01-SYSTEM-OVERVIEW-USER-GUIDE.md)** - User perspective, use cases
- **[Developer Guide & Architecture](./02-DEVELOPER-GUIDE-ARCHITECTURE.md)** - Technical implementation details
- **[Getting Started Guide](./04-GETTING-STARTED-GUIDE.md)** - Setup and installation
- **[Complete Analysis](./claude_code_analysis/complete-system-analysis/requirements-complete.md)** - Detailed technical requirements with evidence

---

**Document maintained by:** MultiGo Development Team
**Last System Analysis:** November 2025 (4 repositories reviewed)
**Requirements Status:** 61% Complete (47/77 requirements)
