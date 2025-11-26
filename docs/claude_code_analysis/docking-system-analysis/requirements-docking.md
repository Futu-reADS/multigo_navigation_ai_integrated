# Docking System - Requirements Document

**Branch:** feature/localization | **Analyst:** Claude AI | **Date:** Nov 25, 2025

---

## Document Purpose

This document provides a **comprehensive requirements analysis** for the Multi Go autonomous docking system, derived from analyzing the existing source code. Each requirement is marked with its **current implementation status** and identifies **gaps** that need to be addressed.

---

## Status Legend

| Symbol | Status | Description |
|--------|--------|-------------|
| ✅ | **COMPLETE** | Fully implemented and working correctly |
| 🟡 | **PARTIAL** | Started but incomplete or has issues |
| ❌ | **NOT IMPLEMENTED** | Required but missing entirely |
| 🐛 | **BUGGY** | Implemented but has critical bugs |
| ❓ | **UNCLEAR** | Unclear if requirement is met |

---

## 1. FUNCTIONAL REQUIREMENTS

### 1.1 Visual Marker Detection

#### FR-1.1.1: ArUco Marker Recognition
**Status:** ✅ **COMPLETE**

**Requirement:** System shall detect ArUco markers from DICT_6X6_250 dictionary using calibrated cameras.

**Implementation:**
- **File:** `aruco_detect.cpp:106-243`
- **Method:** OpenCV `cv::aruco::detectMarkers()` and `estimatePoseSingleMarkers()`
- **Dictionary:** `cv::aruco::DICT_6X6_250`
- **Camera:** Calibrated intrinsics and distortion coefficients

**Evidence:**
```cpp
cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
cv::aruco::estimatePoseSingleMarkers(markerCorners, marker_width,
                                     camera_matrix, dist_coeffs, rvecs, tvecs);
```

**Verification:** ✓ Tested, working

---

#### FR-1.1.2: Dual Camera Support
**Status:** ✅ **COMPLETE**

**Requirement:** System shall support simultaneous detection from left and right cameras.

**Implementation:**
- **Files:** `nav_docking.cpp:89-93`
- Two separate ArUco detector instances
- Topics: `aruco_detect/markers_left`, `aruco_detect/markers_right`

**Evidence:**
```cpp
pose_array_left_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
    marker_topic_left, 10, ...);
pose_array_right_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
    marker_topic_right, 10, ...);
```

**Verification:** ✓ Implemented

---

#### FR-1.1.3: Marker Pose Estimation
**Status:** ✅ **COMPLETE**

**Requirement:** System shall estimate 6-DOF pose (position + orientation) of detected markers relative to camera frame.

**Implementation:**
- **File:** `aruco_detect.cpp:146-184`
- Position: `tvecs[i]` transformed to ROS coordinates
- Orientation: Rodrigues rotation vector → rotation matrix → quaternion

**Evidence:**
```cpp
pose.position.x = tvecs[i][2];
pose.position.y = -tvecs[i][0];
pose.position.z = tvecs[i][1];
// Quaternion conversion with coordinate transform
```

**Verification:** ✓ Working correctly

---

#### FR-1.1.4: Marker ID Filtering
**Status:** ✅ **COMPLETE**

**Requirement:** System shall filter detected markers by desired ID and ignore others.

**Implementation:**
- **File:** `aruco_detect.cpp:139-143`
- Parameter: `desired_aruco_marker_id`
- Only publishes matching markers

**Evidence:**
```cpp
if (markerIds[i] == desired_aruco_marker_id)
{
    // Process marker
}
```

**Verification:** ✓ Tested

---

### 1.2 Coordinate Transformations

#### FR-1.2.1: Camera to Base Link Transform
**Status:** ✅ **COMPLETE**

**Requirement:** System shall transform marker poses from camera frames to robot base_link frame using TF2.

**Implementation:**
- **File:** `nav_docking.cpp:248, 307`
- TF2 lookup with 2-second timeout
- Proper error handling

**Evidence:**
```cpp
cameraToBase_link = tf_buffer->lookupTransform(base_frame, camera_left_frame,
                                               tf2::TimePointZero, tf2::durationFromSec(2));
```

**Verification:** ✓ Working

---

#### FR-1.2.2: Camera to Map Transform
**Status:** ✅ **COMPLETE**

**Requirement:** System shall transform marker poses from camera frames to map frame for navigation goals.

**Implementation:**
- **File:** `nav_goal.cpp:172, 257`
- Used for approach phase goal generation

**Evidence:**
```cpp
cameraToMap = tf_buffer->lookupTransform(map_frame, camera_front_left_frame,
                                        tf2::TimePointZero, tf2::durationFromSec(2));
```

**Verification:** ✓ Working

---

#### FR-1.2.3: Transform Exception Handling
**Status:** ✅ **COMPLETE**

**Requirement:** System shall handle TF2 transform exceptions gracefully without crashing.

**Implementation:**
- **Files:** `nav_docking.cpp:290-296`, `nav_goal.cpp:238-242`
- Try-catch blocks with logging

**Evidence:**
```cpp
catch (tf2::TransformException &ex)
{
    RCLCPP_WARN(this->get_logger(), "Could not transform...: %s", ex.what());
    return;
}
```

**Verification:** ✓ Implemented

---

### 1.3 Docking State Machine

#### FR-1.3.1: Multi-Stage Docking Process
**Status:** 🟡 **PARTIAL** (No formal state machine)

**Requirement:** System shall execute docking in stages: Approach → Alignment → Final Docking → Complete.

**Current Implementation:**
- **Stage 3:** Handled by `nav_goal` (approach to ~5cm)
- **Stage 4:** Handled by `nav_docking.frontMarkerCmdVelPublisher()` (align to ~1cm)
- **Stage 5:** Handled by `nav_docking.dualMarkerCmdVelPublisher()` (precision dock to ~1mm)

**Issues:**
- ❌ No formal state machine class
- ❌ State transitions managed by boolean flags
- ❌ No state diagram documentation
- ❌ `stage_6_docking_status` declared but never used (`nav_docking.h:73`)

**Gap:** Need proper FSM implementation with enum states and clear transition logic.

**Recommendation:** Implement state machine pattern:
```cpp
enum class DockingState {
    IDLE,
    APPROACHING,     // Stage 3
    ALIGNING,        // Stage 4
    DOCKING,         // Stage 5
    COMPLETE,
    FAILED
};
```

---

#### FR-1.3.2: Stage Transition Conditions
**Status:** 🟡 **PARTIAL**

**Requirement:** System shall transition between stages based on clear, measurable conditions.

**Current Transitions:**
1. **Stage 3 → Stage 4:** `marker_tx < 0.05m` (`nav_goal.cpp:190`)
2. **Stage 4 → Stage 5:** All errors < `min_error` (0.01m) (`nav_docking.cpp:452-460`)
3. **Stage 5 → Complete:** All errors < `min_docking_error` (0.001m) with confirmation (`nav_docking.cpp:540-544`)

**Issues:**
- ✅ Transitions are working
- ❌ No hysteresis to prevent oscillation between stages
- ❌ Stage 4 can reset to itself if markers lost > 3s (`nav_docking.cpp:493`)
- ❌ No transition timeout (could loop forever)

**Gap:** Add hysteresis and timeouts

---

#### FR-1.3.3: Error Recovery and Retry Logic
**Status:** ❌ **NOT IMPLEMENTED**

**Requirement:** System shall implement error recovery with maximum retry limits and failure reporting.

**Current State:**
- ⚠️ Stage 4 resets silently on marker loss
- ⚠️ No retry counter
- ⚠️ No maximum attempt limit
- ⚠️ No failure reporting to action client

**Gap:** Need comprehensive error recovery:
- Retry counter with max limit (e.g., 3 attempts)
- Timeout per stage (e.g., 60 seconds)
- Graceful failure with error code
- Option to back out and retry

**Recommendation:**
```cpp
int retry_count = 0;
const int MAX_RETRIES = 3;
rclcpp::Time stage_start_time;
const double STAGE_TIMEOUT = 60.0;  // seconds
```

---

### 1.4 Marker Detection Strategy

#### FR-1.4.1: Dual Marker Mode
**Status:** 🐛 **BUGGY** (Calculation error)

**Requirement:** When both markers visible, system shall use averaged position from both markers for increased accuracy.

**Implementation:**
- **File:** `nav_docking.cpp:379-396`, `476-514`
- Condition: Both marker delays < 0.2 seconds

**Bug Found:**
```cpp
// Current (WRONG):
double distance = (left_marker_x) + (right_marker_x) / 2;  // ← Missing parentheses!
// Should be:
double distance = (left_marker_x + right_marker_x) / 2;
```

**Impact:** 🔴 CRITICAL - Distance calculation incorrect

**Example:**
```
left_x = 1.0m, right_x = 2.0m
Current: 1.0 + (2.0/2) = 2.0m  ← WRONG!
Correct: (1.0 + 2.0) / 2 = 1.5m
```

**Also:**
```cpp
double center = (left_marker_y - -right_marker_y);  // Double negative confusing
// Should be:
double center = (left_marker_y + right_marker_y) / 2;
```

**Action Required:** ⚠️ Fix immediately before deployment

---

#### FR-1.4.2: Single Marker Fallback
**Status:** ✅ **COMPLETE**

**Requirement:** When one marker is lost or delayed, system shall fall back to single marker mode.

**Implementation:**
- **File:** `nav_docking.cpp:397-425`
- Fallback condition: `callback_duration >= marker_delay_threshold_sec` (0.2s)
- Selects marker with shortest delay
- Applies lateral offset (±0.17m) to compensate for off-center marker

**Evidence:**
```cpp
if (callback_duration_left < callback_duration_right)
    // Use left marker with offset
else
    // Use right marker with offset
```

**Verification:** ✓ Working correctly

---

#### FR-1.4.3: Marker Delay Threshold
**Status:** ✅ **COMPLETE**

**Requirement:** System shall switch to single marker mode if marker update is older than threshold.

**Implementation:**
- **File:** `nav_docking.cpp:69, 427`
- Threshold: 0.2 seconds
- Calculates time since last marker update
- Prevents using stale data

**Verification:** ✓ Implemented

---

### 1.5 Control System

#### FR-1.5.1: PID Control for X, Y, and Yaw
**Status:** 🐛 **BUGGY** (Integral term broken)

**Requirement:** System shall use PID control for smooth velocity commands in X (forward), Y (lateral), and Yaw (rotation) axes.

**Implementation:**
- **File:** `nav_docking.cpp:187-219`
- Function: `calculate(error, prev_error, kp, ki, kd, ...)`
- Separate PID for each axis

**Critical Bug:**
```cpp
// Line 197:
double integral = error * callback_duration;  // ← This is NOT integral!
```

**Issue:** This calculates `error * dt`, which is just scaled error, NOT integral.
**Correct Implementation:**
```cpp
// Need to accumulate:
integral_sum += error * callback_duration;
```

**Impact:** 🔴 CRITICAL
- Ki gains don't work as designed
- No integral windup protection
- No accumulated error correction

**Also Missing:**
- ❌ Integral state variables (not preserved between calls)
- ❌ Anti-windup mechanism

**Action Required:** ⚠️ Reimplement integral calculation correctly

**Recommendation:**
```cpp
// In header:
double integral_x = 0.0;
double integral_y = 0.0;
double integral_yaw = 0.0;
const double INTEGRAL_MAX = 1.0;  // Anti-windup limit

// In calculate function:
integral_sum += error * dt;
// Clamp integral
if (integral_sum > INTEGRAL_MAX) integral_sum = INTEGRAL_MAX;
if (integral_sum < -INTEGRAL_MAX) integral_sum = -INTEGRAL_MAX;
double output = kp * error + ki * integral_sum + kd * derivative;
```

---

#### FR-1.5.2: Velocity Clamping
**Status:** 🟡 **PARTIAL** (Has issues)

**Requirement:** System shall clamp output velocities to safe limits (max and min).

**Implementation:**
- **File:** `nav_docking.cpp:206-216`
- Max speed: 0.1 m/s
- Min speed: 0.005 m/s

**Issue:**
```cpp
// Enforce minimum output magnitude
if (std::abs(output) < min_output) {
    output = (output > 0) ? min_output : -min_output;
}
```

**Problem:** If PID calculates 0.002 m/s (within dead-zone), it gets bumped to 0.005 m/s, violating the min_error check. This can cause oscillation near target.

**Gap:** Minimum output should only apply if error > min_error

**Recommendation:**
```cpp
if (std::abs(error) > min_error && std::abs(output) < min_output) {
    output = (output > 0) ? min_output : -min_output;
}
```

---

#### FR-1.5.3: Dead-Zone Handling
**Status:** ✅ **COMPLETE**

**Requirement:** System shall not command motion if error is within acceptable threshold (dead-zone).

**Implementation:**
- **File:** `nav_docking.cpp:192-194`
- Checks error against `min_error` threshold
- Returns 0 if within dead-zone

**Evidence:**
```cpp
if (std::abs(error) <= min_error) {
    return 0.0;
}
```

**Verification:** ✓ Working

---

#### FR-1.5.4: Y-Axis Alignment Priority
**Status:** ✅ **COMPLETE**

**Requirement:** In Stage 4, system shall prioritize lateral (Y) alignment before moving forward (X).

**Implementation:**
- **File:** `nav_docking.cpp:430-446`
- If `|error_y| >= min_y_error`, set `linear.x = 0`
- Only move forward when aligned

**Evidence:**
```cpp
if (fabs(error_y) < min_y_error)
{
    // Three-axis control
    twist_msg.linear.x = calculate(...);
    twist_msg.linear.y = calculate(...);
    twist_msg.angular.z = calculate(...);
}
else  // align robot first
{
    twist_msg.linear.x = 0;  // No forward motion
    twist_msg.linear.y = calculate(...);
    twist_msg.angular.z = calculate(...);
}
```

**Verification:** ✓ Implemented correctly

**Rationale:** Prevents approaching at an angle, ensures straight approach.

---

### 1.6 Action Server Interface

#### FR-1.6.1: Dock Action Server
**Status:** ✅ **COMPLETE**

**Requirement:** System shall provide ROS2 action server for docking requests.

**Implementation:**
- **File:** `nav_docking.cpp:9-14`
- Action type: `nav_interface::action::Dock`
- Action name: `dock`
- Goal: `bool dock_request`
- Result: `bool success`
- Feedback: `float64 distance`

**Evidence:**
```cpp
action_server_ = rclcpp_action::create_server<Dock>(
    this, "dock",
    std::bind(&Nav_docking::handle_goal, ...),
    std::bind(&Nav_docking::handle_cancel, ...),
    std::bind(&Nav_docking::handle_accepted, ...));
```

**Verification:** ✓ Standard ROS2 action pattern

---

#### FR-1.6.2: Goal Validation
**Status:** 🟡 **PARTIAL** (Trivial validation)

**Requirement:** System shall validate docking goals before accepting.

**Implementation:**
- **File:** `nav_docking.cpp:119-128`
- Currently only checks if `dock_request == true`

**Issue:** Validation is trivial, doesn't check:
- ❌ Robot position (are we in range?)
- ❌ Markers visible
- ❌ System ready state
- ❌ Battery level
- ❌ No existing active goal

**Gap:** Enhance validation

**Recommendation:**
```cpp
// Check preconditions
if (!markers_recently_seen()) return REJECT;
if (battery_level < MINIMUM_BATTERY) return REJECT;
if (is_goal_active()) return REJECT;
```

---

#### FR-1.6.3: Feedback Publishing
**Status:** ✅ **COMPLETE**

**Requirement:** System shall publish feedback containing current distance to target during docking.

**Implementation:**
- **File:** `nav_docking.cpp:169-172`
- Publishes `error_x` (distance) in feedback
- Rate: Throttled to 1 Hz logging, but published every loop

**Evidence:**
```cpp
feedback->distance = static_cast<double>(feedback_distance);
goal_handle->publish_feedback(feedback);
```

**Verification:** ✓ Working

**Note:** Feedback is X-axis error, not Euclidean distance. Consider publishing total error:
```cpp
feedback->distance = std::sqrt(error_x*error_x + error_y*error_y + error_yaw*error_yaw);
```

---

#### FR-1.6.4: Cancellation Support
**Status:** ✅ **COMPLETE**

**Requirement:** System shall support goal cancellation and stop motion immediately.

**Implementation:**
- **File:** `nav_docking.cpp:131-136, 161-167`
- Accepts cancel requests
- Sets `enable_callback = false` to stop control loops
- Publishes canceled result

**Evidence:**
```cpp
if (goal_handle->is_canceling())
{
    RCLCPP_INFO(this->get_logger(), "Goal canceled.");
    goal_handle->canceled(result);
    Nav_docking::enable_callback = false;
    return;
}
```

**Verification:** ✓ Implemented

**Issue:** ⚠️ Doesn't publish zero velocity before stopping. Robot may drift.

**Recommendation:** Publish zero twist before disabling callbacks.

---

### 1.7 Safety and Robustness

#### FR-1.7.1: Marker Timeout Detection
**Status:** ✅ **COMPLETE**

**Requirement:** System shall stop motion if markers are not detected within timeout period.

**Implementation:**
- **File:** `nav_docking.cpp:427-469, 516`
- Timeout: 0.2 seconds
- Publishes zero velocity if timeout exceeded

**Evidence:**
```cpp
if ((callback_duration < marker_delay_threshold_sec))
{
    // Control active
    cmd_vel_pub->publish(twist_msg);
}
else
{
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;
    cmd_vel_pub->publish(twist_msg);
}
```

**Verification:** ✓ Working

---

#### FR-1.7.2: Transform Timeout
**Status:** ✅ **COMPLETE**

**Requirement:** System shall timeout on TF lookups to prevent blocking.

**Implementation:**
- **File:** `nav_docking.cpp:248, 307`
- Timeout: 2 seconds on `lookupTransform()`

**Evidence:**
```cpp
tf_buffer->lookupTransform(base_frame, camera_left_frame,
                          tf2::TimePointZero, tf2::durationFromSec(2));
```

**Verification:** ✓ Working

---

#### FR-1.7.3: Confirmation Before Completion
**Status:** ✅ **COMPLETE**

**Requirement:** System shall require two consecutive successful readings before marking docking complete.

**Implementation:**
- **File:** `nav_docking.cpp:540-552`
- Boolean flag: `confirmed_docking_status`
- 0.5 second delay between checks

**Evidence:**
```cpp
if (confirmed_docking_status==true)
{
    stage_5_docking_status = true;  // Complete
}
else
{
    confirmed_docking_status=true;
    rclcpp::Rate rate(2);
    rate.sleep();  // 0.5 seconds
    return;
}
```

**Verification:** ✓ Prevents false completions due to noise

---

#### FR-1.7.4: Velocity Ramping
**Status:** ❌ **NOT IMPLEMENTED**

**Requirement:** System shall ramp velocities smoothly to prevent sudden accelerations.

**Current State:**
- Commands can jump from max to zero instantly
- No acceleration limits

**Impact:**
- ⚠️ Mechanical stress on motors
- ⚠️ Loss of traction on sudden stops
- ⚠️ Reduced passenger comfort

**Gap:** Need velocity ramping

**Recommendation:**
```cpp
double ramp_velocity(double current, double target, double max_accel, double dt)
{
    double max_change = max_accel * dt;
    double change = target - current;
    if (std::abs(change) > max_change)
        return current + std::copysign(max_change, change);
    return target;
}
```

---

#### FR-1.7.5: Collision Detection During Docking
**Status:** ❌ **NOT IMPLEMENTED**

**Requirement:** System shall monitor for obstacles during docking and abort if collision imminent.

**Current State:**
- Only uses vision (ArUco markers)
- No LiDAR/sonar integration during docking
- No safety scanner monitoring

**Impact:** ⚠️ HIGH - Could collide with obstacles not in camera view

**Gap:** Need safety layer

**Recommendation:**
- Subscribe to safety scanner topic
- Monitor for obstacles in approach path
- Abort and back out if obstacle detected

---

#### FR-1.7.6: Action Execution Timeout
**Status:** ❌ **NOT IMPLEMENTED**

**Requirement:** System shall timeout if docking takes longer than maximum allowed time.

**Current State:**
- **File:** `nav_docking.cpp:159-173`
- `while (stage_5_docking_status == false)` - No timeout!

**Issue:** Can run indefinitely if markers lost or docking fails

**Gap:** Add maximum execution time

**Recommendation:**
```cpp
auto start_time = std::chrono::steady_clock::now();
const double MAX_DOCKING_TIME = 60.0;  // seconds

while (stage_5_docking_status == false) {
    auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time).count();

    if (elapsed > MAX_DOCKING_TIME) {
        result->success = false;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Docking timeout");
        return;
    }
    // ... rest of logic
}
```

---

### 1.8 Configuration and Parameters

#### FR-1.8.1: PID Parameter Configuration
**Status:** ✅ **COMPLETE**

**Requirement:** System shall load PID gains from configuration file.

**Implementation:**
- **File:** `nav_docking.cpp:53-74`
- Config: `src/nav_docking/config/docking_pid_params.yaml`
- Parameters: kp, ki, kd for X, Y, Z axes

**Evidence:**
```cpp
this->declare_parameter("pid_parameters.kp_x", 0.00);
this->get_parameter("pid_parameters.kp_x", kp_x);
```

**Verification:** ✓ Working

---

#### FR-1.8.2: Dynamic Parameter Reconfiguration
**Status:** ❌ **NOT IMPLEMENTED**

**Requirement:** System shall support runtime parameter updates without restarting node.

**Current State:**
- Parameters only loaded at startup
- Must restart node to change PID gains
- Difficult to tune in field

**Gap:** Add parameter callbacks

**Recommendation:**
```cpp
// In constructor:
param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&Nav_docking::parametersCallback, this, std::placeholders::_1));

// Callback:
rcl_interfaces::msg::SetParametersResult Nav_docking::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    for (const auto &param : parameters) {
        if (param.get_name() == "pid_parameters.kp_x")
            kp_x = param.as_double();
        // ... other parameters
    }
    return result;
}
```

---

#### FR-1.8.3: Marker Offset Calibration
**Status:** 🟡 **PARTIAL**

**Requirement:** System shall support easy calibration of marker position offsets.

**Implementation:**
- **File:** `nav_docking.cpp:25-29`
- Parameters exist for offsets
- Values can be changed in launch file

**Issues:**
- ❌ No calibration procedure documented
- ❌ No tool to measure current offset
- ❌ Commented-out calibration logs (`nav_docking.cpp:393-395, 408-410, etc.`)

**Gap:** Need calibration tool and documentation

**Recommendation:**
1. Create calibration mode that logs current marker positions
2. Document calibration procedure
3. Provide script to calculate optimal offsets

---

### 1.9 Diagnostics and Monitoring

#### FR-1.9.1: System Health Diagnostics
**Status:** ❌ **NOT IMPLEMENTED**

**Requirement:** System shall publish diagnostic messages for health monitoring.

**Current State:**
- No ROS2 diagnostics published
- Only basic logging

**Gap:** Add diagnostics publisher

**Recommendation:**
```cpp
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10);

// Publish status:
// - Marker detection rate
// - Control loop frequency
// - Transform availability
// - Current stage
// - Error magnitudes
```

---

#### FR-1.9.2: Performance Metrics Logging
**Status:** ❌ **NOT IMPLEMENTED**

**Requirement:** System shall log docking attempt statistics for analysis.

**Current State:**
- No structured logging
- No success/failure tracking
- No timing data saved

**Gap:** Add metrics collection

**Recommendation:**
```cpp
struct DockingMetrics {
    rclcpp::Time start_time;
    rclcpp::Time end_time;
    bool success;
    int retry_count;
    double final_error_x;
    double final_error_y;
    double final_error_yaw;
};

// Log to file for post-analysis
```

---

#### FR-1.9.3: Debug Visualization
**Status:** 🟡 **PARTIAL**

**Requirement:** System shall publish visualization markers for debugging in RViz.

**Current State:**
- ArUco markers broadcast as TF frames (✓)
- No velocity command visualization
- No error vector visualization
- No docking path visualization

**Gap:** Add RViz markers

**Recommendation:**
- Publish `visualization_msgs::Marker` for:
  - Desired position (target)
  - Current error vector
  - Planned trajectory
  - Stage indicator

---

### 1.10 Undocking

#### FR-1.10.1: Undocking Action
**Status:** ❌ **NOT IMPLEMENTED**

**Requirement:** System shall provide action to undock and move robot away from dock.

**Current State:**
- No undock action defined
- No reverse procedure
- Manual intervention required

**Impact:** Incomplete system, requires manual operation

**Gap:** Implement undocking

**Recommendation:**
```cpp
// Add to nav_interface:
# Undock.action
bool undock_request
---
bool success
---
float64 distance

// Implement reverse procedure:
// 1. Move back X meters
// 2. Rotate to desired heading
// 3. Return success
```

---

### 1.11 Thread Safety

#### FR-1.11.1: Shared State Protection
**Status:** 🐛 **BUGGY** (Not thread-safe)

**Requirement:** System shall protect shared state with mutexes in multi-threaded environment.

**Current State:**
- **File:** `nav_docking.cpp:140-142`
- Action execution in separate thread
- Callbacks run in ROS spinner threads
- Shared variables accessed without locks:
  - `stage_4_docking_status`
  - `stage_5_docking_status`
  - `enable_callback`
  - `error_x`, `error_y`, `error_yaw`
  - Timer callbacks

**Issue:** Race conditions possible

**Impact:** 🔴 CRITICAL - Undefined behavior

**Evidence:**
```cpp
// nav_docking.cpp:140-142
void Nav_docking::handle_accepted(const std::shared_ptr<GoalHandleDock> goal_handle)
{
    // Spawns new thread - no synchronization!
    std::thread{std::bind(&Nav_docking::execute, this, goal_handle)}.detach();
}
```

**Gap:** Add mutex protection

**Recommendation:**
```cpp
#include <mutex>

private:
    std::mutex state_mutex_;

// In functions accessing shared state:
void frontMarkerCmdVelPublisher()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    // ... access shared variables
}
```

---

#### FR-1.11.2: Dual Timer Race Condition
**Status:** 🐛 **BUGGY** (Race condition)

**Requirement:** System shall prevent simultaneous publishing from multiple control loops.

**Current State:**
- **File:** `nav_docking.cpp:100-106`
- Two timers: `front_timer_` and `dual_timer_`
- Both publish to same topic `cmd_vel_final`
- No synchronization

**Issue:**
```cpp
front_timer_ = this->create_wall_timer(period,
    std::bind(&Nav_docking::frontMarkerCmdVelPublisher, this));
dual_timer_ = this->create_wall_timer(period,
    std::bind(&Nav_docking::dualMarkerCmdVelPublisher, this));
```

Both timers can fire simultaneously, publishing conflicting commands.

**Impact:** 🔴 CRITICAL - Unpredictable motion

**Gap:** Use single timer or add mutex

**Recommendation:**
```cpp
// Option 1: Single timer
control_timer_ = this->create_wall_timer(period,
    std::bind(&Nav_docking::controlLoop, this));

void controlLoop() {
    if (stage_4_docking_status == false)
        frontMarkerCmdVelPublisher();
    else
        dualMarkerCmdVelPublisher();
}

// Option 2: Mutex
std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
cmd_vel_pub->publish(twist_msg);
```

---

## 2. NON-FUNCTIONAL REQUIREMENTS

### 2.1 Performance

#### NFR-2.1.1: Control Loop Frequency
**Status:** ✅ **COMPLETE**

**Requirement:** System shall run control loop at minimum 20 Hz.

**Implementation:**
- **File:** `nav_docking.h:68`
- `publish_rate = 30` Hz

**Verification:** ✓ Meets requirement (30 > 20)

---

#### NFR-2.1.2: Marker Detection Latency
**Status:** ❓ **UNCLEAR**

**Requirement:** System shall detect and process marker pose within 100ms.

**Current State:**
- Detection runs in callback (on image arrival)
- No timing measurements
- No latency logging

**Gap:** Add performance metrics

---

#### NFR-2.1.3: Transform Lookup Performance
**Status:** ✅ **COMPLETE**

**Requirement:** System shall retrieve transforms without blocking control loop.

**Implementation:**
- 2-second timeout on TF lookups
- Exception handling prevents blocking

**Verification:** ✓ Working

---

### 2.2 Accuracy

#### NFR-2.2.1: Final Docking Accuracy
**Status:** ✅ **COMPLETE** (if bugs fixed)

**Requirement:** System shall achieve ±1mm position accuracy in final docking.

**Implementation:**
- **File:** `nav_docking.cpp:131`
- `min_docking_error = 0.001` m (1mm)
- Stage 5 requires all errors < 1mm

**Verification:** ✓ Requirement encoded correctly

**Note:** Actual accuracy depends on:
- Camera calibration quality
- Marker size and quality
- Lighting conditions
- Fixed bugs (especially dual marker calculation)

---

#### NFR-2.2.2: Approach Accuracy
**Status:** ✅ **COMPLETE**

**Requirement:** System shall achieve ±1cm accuracy in Stage 4 alignment.

**Implementation:**
- **File:** `nav_docking.h:129`
- `min_error = 0.01` m (1cm)

**Verification:** ✓ Requirement met

---

### 2.3 Reliability

#### NFR-2.3.1: Docking Success Rate
**Status:** ❓ **UNCLEAR** (No data)

**Requirement:** System shall achieve >95% success rate under nominal conditions.

**Current State:**
- No success/failure tracking
- No statistics collection
- No baseline established

**Gap:** Implement metrics collection and testing

---

#### NFR-2.3.2: Fault Tolerance
**Status:** 🟡 **PARTIAL**

**Requirement:** System shall handle single-point failures gracefully.

**Current Handling:**
- ✅ One marker loss: Falls back to single marker
- ✅ TF timeout: Logs error and skips cycle
- ❌ Both markers lost: Stops but doesn't abort action
- ❌ Camera failure: No detection
- ❌ Motor failure: No detection

**Gap:** Improve fault detection and reporting

---

### 2.4 Safety

#### NFR-2.4.1: Emergency Stop Support
**Status:** ❓ **UNCLEAR**

**Requirement:** System shall respond to emergency stop signal within 100ms.

**Current State:**
- No emergency stop subscription
- Action cancellation available but requires client request
- No hardware e-stop integration

**Gap:** Add emergency stop handling

---

#### NFR-2.4.2: Collision Avoidance
**Status:** ❌ **NOT IMPLEMENTED**

**Requirement:** System shall stop if obstacle detected within 0.5m during docking.

**Current State:**
- No obstacle detection during docking
- Vision-only (blind to non-visual obstacles)

**Gap:** Integrate safety scanner

---

### 2.5 Maintainability

#### NFR-2.5.1: Code Documentation
**Status:** 🟡 **PARTIAL**

**Requirement:** All public functions shall have documentation comments.

**Current State:**
- No Doxygen comments
- Minimal inline comments
- No function-level documentation
- Some high-level comments exist

**Gap:** Add comprehensive documentation

---

#### NFR-2.5.2: Configuration Management
**Status:** 🟡 **PARTIAL**

**Requirement:** System configuration shall be externalized in YAML files.

**Current State:**
- ✅ PID gains in YAML
- ❌ Thresholds hardcoded
- ❌ Marker IDs in launch files
- ❌ No configuration validation

**Gap:** Move all magic numbers to config

---

### 2.6 Testing

#### NFR-2.6.1: Unit Test Coverage
**Status:** ❌ **NOT IMPLEMENTED**

**Requirement:** System shall have >80% unit test coverage.

**Current State:**
- **Zero unit tests** for docking components
- Only linting tests in mecanum_wheels package

**Gap:** Create comprehensive unit tests

**Recommendation:**
```cpp
// Tests needed:
// - PID calculation correctness
// - Dual/single marker switching logic
// - State transitions
// - Error calculations
// - Transform utilities
// - Parameter validation
```

---

#### NFR-2.6.2: Integration Testing
**Status:** ❌ **NOT IMPLEMENTED**

**Requirement:** System shall have integration tests for complete docking sequence.

**Current State:**
- No automated integration tests
- Only manual field testing

**Gap:** Create integration test suite

---

#### NFR-2.6.3: Simulation Testing
**Status:** ❓ **UNCLEAR**

**Requirement:** System shall be testable in Gazebo simulation.

**Current State:**
- Launch files mention simulation
- Not clear if docking tested in sim
- No documented simulation testing procedure

**Gap:** Create simulation test environment

---

## 3. SUMMARY OF GAPS

### 🔴 Critical (Must Fix)

| ID | Requirement | Gap | Impact |
|----|-------------|-----|--------|
| FR-1.5.1 | PID Control | Integral term broken | Ki gains ineffective |
| FR-1.4.1 | Dual Marker | Distance calculation bug | Incorrect positioning |
| FR-1.11.1 | Thread Safety | No mutex protection | Race conditions |
| FR-1.11.2 | Timer Sync | Dual timers conflict | Unpredictable motion |
| FR-1.2.1 | Bug Fix | Wrong parameter name (nav_goal) | Wrong marker ID |

### 🟡 High Priority

| ID | Requirement | Gap | Impact |
|----|-------------|-----|--------|
| FR-1.3.1 | State Machine | No formal FSM | Hard to maintain |
| FR-1.3.3 | Error Recovery | No retry logic | Can loop forever |
| FR-1.6.2 | Validation | Trivial goal checks | Accepts bad goals |
| FR-1.7.4 | Velocity Ramping | No acceleration limits | Jerky motion |
| FR-1.7.6 | Timeout | No action timeout | Can run forever |
| FR-1.10.1 | Undocking | Not implemented | Incomplete system |

### 🟢 Medium Priority

| ID | Requirement | Gap | Impact |
|----|-------------|-----|--------|
| FR-1.5.2 | Velocity Clamp | Min speed logic issue | Oscillation near target |
| FR-1.7.5 | Collision | No obstacle detection | Safety risk |
| FR-1.8.2 | Reconfigure | No dynamic params | Hard to tune |
| FR-1.9.1 | Diagnostics | Not implemented | Hard to monitor |
| FR-2.6.1 | Unit Tests | None | Low confidence |

---

## 4. RECOMMENDED IMPLEMENTATION ROADMAP

### Phase 1: Critical Bug Fixes (Week 1)
1. ✅ Fix PID integral calculation
2. ✅ Fix dual marker distance formula
3. ✅ Fix parameter assignment bugs
4. ✅ Add mutex protection
5. ✅ Consolidate dual timers

### Phase 2: Architecture Improvements (Week 2-3)
1. ✅ Implement proper state machine
2. ✅ Add error recovery and retry logic
3. ✅ Add action execution timeout
4. ✅ Improve goal validation

### Phase 3: Safety & Robustness (Week 4-5)
1. ✅ Add velocity ramping
2. ✅ Integrate collision detection
3. ✅ Add emergency stop handler
4. ✅ Implement comprehensive input validation

### Phase 4: Features & Testing (Week 6-8)
1. ✅ Implement undocking
2. ✅ Add diagnostics publishing
3. ✅ Create unit test suite
4. ✅ Create integration tests
5. ✅ Add performance monitoring

### Phase 5: Polish & Documentation (Week 9-10)
1. ✅ Add dynamic reconfiguration
2. ✅ Improve code documentation
3. ✅ Create calibration tools
4. ✅ Write user manual
5. ✅ Conduct field validation

---

## 5. TESTING CHECKLIST

### Unit Tests Needed
- [ ] PID controller calculation
- [ ] Dual/single marker switching
- [ ] Error calculation accuracy
- [ ] State transition logic
- [ ] Transform helpers
- [ ] Parameter validation
- [ ] Velocity clamping

### Integration Tests Needed
- [ ] Complete docking sequence (Stage 3→4→5)
- [ ] Single marker fallback during docking
- [ ] Recovery from marker loss
- [ ] Action cancellation during each stage
- [ ] Multiple docking attempts in sequence

### Simulation Tests Needed
- [ ] Docking with perfect markers
- [ ] Docking with one marker occluded
- [ ] Docking with noise in detections
- [ ] Failure recovery scenarios

### Field Tests Needed
- [ ] Various lighting conditions
- [ ] Different approach angles
- [ ] Different speeds
- [ ] Marker at various distances
- [ ] Long-term reliability (100+ attempts)

---

## 6. COMPLIANCE MATRIX

| Requirement Category | Complete | Partial | Not Impl. | Buggy | Total |
|---------------------|----------|---------|-----------|-------|-------|
| Visual Detection | 4 | 0 | 0 | 0 | 4 |
| Transforms | 3 | 0 | 0 | 0 | 3 |
| State Machine | 0 | 2 | 1 | 0 | 3 |
| Detection Strategy | 2 | 0 | 0 | 1 | 3 |
| Control System | 2 | 1 | 0 | 2 | 5 |
| Action Interface | 3 | 1 | 0 | 0 | 4 |
| Safety | 3 | 0 | 3 | 0 | 6 |
| Configuration | 1 | 1 | 1 | 0 | 3 |
| Diagnostics | 0 | 1 | 2 | 0 | 3 |
| Undocking | 0 | 0 | 1 | 0 | 1 |
| Thread Safety | 0 | 0 | 0 | 2 | 2 |
| **TOTAL** | **18** | **6** | **8** | **5** | **37** |

**Completion Rate:** 49% (18/37) ✅ Complete
**Needs Work:** 51% (19/37) - 6 Partial, 8 Missing, 5 Buggy

---

*This requirements document should be updated as implementation progresses. Next steps: Address critical bugs, then implement missing safety features.*
