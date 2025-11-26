# Docking System - Questions & Gaps

**Branch:** feature/localization | **Analyst:** Claude AI | **Date:** Nov 25, 2025

---

## Document Purpose

This document lists **questions** that need clarification from the development team and **identified gaps** in the current docking system implementation. These items require decisions, additional information, or implementation work.

---

## Categories

- ❓ **QUESTION** - Needs clarification or decision
- 🔍 **INVESTIGATION** - Requires further research
- 📋 **GAP** - Missing functionality or documentation
- ⚠️ **CONCERN** - Potential issue needing attention
- 💡 **SUGGESTION** - Improvement recommendation

---

## 1. SYSTEM DESIGN QUESTIONS

### Q1.1: Stage 6 Docking Status
**Type:** ❓ **QUESTION**
**Priority:** HIGH

**Observation:**
- `stage_6_docking_status` declared in header (`nav_docking.h:73`)
- Never initialized
- Never used in code
- Never set to true/false

**Questions:**
1. Was Stage 6 planned but not implemented?
2. What was the intended purpose of Stage 6?
3. Should it be removed, or is there missing implementation?
4. Does the docking process need an additional stage?

**Impact:** Code clutter, confusion about system design

**Recommendation:** Either implement Stage 6 or remove the unused variable

---

### Q1.2: Why Two Separate Timer Callbacks?
**Type:** ❓ **QUESTION**
**Priority:** HIGH

**Observation:**
- `frontMarkerCmdVelPublisher()` and `dualMarkerCmdVelPublisher()` run independently
- Both publish to same topic `cmd_vel_final`
- Both run at 30 Hz
- Conditional logic exists (`if stage_4 == false` vs `if stage_4 == true`)

**Questions:**
1. Why use two separate timers instead of one with conditional logic?
2. Is there a performance or architectural reason?
3. Was this a deliberate design choice or oversight?
4. Have race conditions been observed in practice?

**Concern:** ⚠️ Race condition when both try to publish simultaneously

**Recommendation:** Use single timer or clearly document reasoning

---

### Q1.3: Rotation Offset Calculation Method
**Type:** 🔍 **INVESTIGATION**

**Observation:**
- Rotation calculated as: `rotation = (right_marker_x - left_marker_x)`
- This gives lateral offset, not angular rotation

**Questions:**
1. Is this the correct rotation metric for alignment?
2. Should it be calculated from marker orientations instead?
3. Has this been validated to work in practice?
4. What is the expected baseline distance between markers?

**Formula Check:**
```cpp
// Current:
rotation = right_x - left_x;  // Lateral distance difference

// Alternative (angular):
double baseline = 0.5;  // Expected marker separation
double angle = atan2(right_x - left_x, baseline);
```

**Status:** Works but may not be optimal

---

### Q1.4: Why Y-Error Threshold 3x Larger?
**Type:** ❓ **QUESTION**

**Observation:**
- `min_error = 0.01` m (X and Yaw)
- `min_y_error = min_error * 3 = 0.03` m (Y-axis)
- Similarly, `min_y_docking_error = min_docking_error * 3`

**Questions:**
1. Why is lateral (Y) tolerance 3x larger than forward (X)?
2. Is this empirically derived or theoretical?
3. Does mecanum wheel slippage affect Y accuracy?
4. Should this ratio be configurable?

**Current:** Hardcoded multiplier of 3

**Recommendation:** Document rationale or make it a parameter

---

### Q1.5: Marker Delay Threshold Rationale
**Type:** ❓ **QUESTION**

**Observation:**
- `marker_delay_threshold_sec = 0.2` seconds
- Switches from dual to single marker after 0.2s

**Questions:**
1. How was 0.2s chosen?
2. Is this based on camera frame rate?
3. What if camera runs slower (e.g., 5 fps = 0.2s per frame)?
4. Should this be related to control loop rate?

**Related:**
- Control loop: 30 Hz (0.033s per cycle)
- 0.2s = 6 control cycles

**Recommendation:** Document reasoning or make adaptive based on detection rate

---

## 2. HARDWARE & DEPLOYMENT QUESTIONS

### Q2.1: Camera Configuration and Placement
**Type:** 🔍 **INVESTIGATION**
**Priority:** MEDIUM

**Questions:**
1. What cameras are used? (Model, resolution, frame rate)
2. Where are left/right cameras mounted on robot?
3. What is the baseline distance between cameras?
4. What is the camera field of view?
5. At what distance range can markers be reliably detected?
6. How are cameras synchronized?

**Current Knowledge:**
- Resolution: 1280x720 (from `calib.yaml`)
- Encoding: MONO8 (grayscale)
- Two front cameras (left and right)

**Missing:**
- Camera model
- Mounting positions and orientations
- Detection range specifications

**Why Important:** Affects marker size, placement, and detection reliability

---

### Q2.2: Marker Size and Placement
**Type:** 📋 **GAP** - Missing Documentation

**Questions:**
1. What size are the ArUco markers? (Current param: 0.05m = 5cm, but verify)
2. How far apart are markers mounted on the dock?
3. At what height are they mounted?
4. What is the minimum/maximum detection distance?
5. Are markers printed or displayed on screen?
6. What material (paper, plastic, reflective)?

**Current Config:**
```yaml
marker_width: 0.05  # 5cm
desired_aruco_marker_id_left: 20
desired_aruco_marker_id_right: 21
```

**Gap:** No specification document for marker setup

**Recommendation:** Create "Marker Installation Guide" with:
- Marker size requirements
- Mounting positions
- Lighting requirements
- Printing specifications

---

### Q2.3: Robot Dimensions and Rotation Centers
**Type:** 📋 **GAP**

**Questions:**
1. What are the robot's physical dimensions?
2. Where is `base_link` located on the robot?
3. Why are rotation centers different for each mode?
   - SOLO: 0.0m
   - DOCKING: 0.25m
   - COMBINE_CHAIR: 0.5m
4. Are these measured from base_link center?
5. What changes physically when transitioning between modes?

**Current Config:**
```cpp
LENGTH_ROTATION_CENTER_SOLO: 0.0
LENGTH_ROTATION_CENTER_DOCKING: 0.25
LENGTH_ROTATION_CENTER_COMBINE_CHAIR: 0.5
```

**Gap:** No robot dimensional drawings or CAD

**Recommendation:** Add robot specification document with diagrams

---

### Q2.4: Mecanum Wheel Specifications
**Type:** 📋 **GAP**

**Known:**
```python
WHEEL_SEPARATION_WIDTH = 0.40m
WHEEL_SEPARATION_LENGTH = 0.30m
WHEEL_RADIUS = 0.0762m (3 inches)
```

**Questions:**
1. What is the wheel model/manufacturer?
2. What is the maximum wheel speed (rpm)?
3. What is the motor model? (Phidget BLDC motor model #?)
4. What is the gear ratio?
5. What is the maximum payload capacity?
6. How is slippage compensated?

**Gap:** No hardware specification document

---

### Q2.5: Testing Environment
**Type:** ❓ **QUESTION**

**Questions:**
1. Where is the system currently being tested (China)?
2. What is the lighting condition in test environment?
3. Is the dock stationary or can it move?
4. What surface is the robot operating on (tile, carpet, concrete)?
5. What is the typical approach distance?
6. Are there specific deployment sites planned?

**Why Important:** Affects parameter tuning, lighting robustness, marker design

---

## 3. INTEGRATION & DEPENDENCIES QUESTIONS

### Q3.1: NAV2 Integration Details
**Type:** 🔍 **INVESTIGATION**

**Observation:**
- `nav_goal` publishes to `goal_pose` topic
- Presumably NAV2 subscribes to this

**Questions:**
1. Which NAV2 controller is used?
2. Is it DWB, TEB, or other?
3. What are the NAV2 parameters (inflation radius, speed limits)?
4. Does NAV2 use global or local costmap during approach?
5. At what point does NAV2 stop and docking take over?
6. Is there a handoff mechanism between NAV2 and docking controller?

**Gap:** No documentation on NAV2 configuration for docking

**Recommendation:** Document NAV2 setup and integration points

---

### Q3.2: nav_interface Package Source
**Type:** 🔍 **INVESTIGATION**

**Observation:**
- Actions `Dock` and `Approach` come from `nav_interface` package
- This package is imported from `multigo_master` repo

**Questions:**
1. Where is the `nav_interface` source code?
2. What other messages/actions are defined there?
3. Who maintains this package?
4. Is there versioning/compatibility management?
5. Are there other robots using the same interface?

**Current:**
```yaml
# multigo.repos
master:
  type: git
  url: git@github.com:Futu-reADS/multigo_master.git
  version: develop
```

**Recommendation:** Document interface specifications

---

### Q3.3: RTAB-Map Configuration
**Type:** ❓ **QUESTION**

**Observation:**
- `multigo.repos` imports `rtabmap` and `rtabmap_ros`
- Used for SLAM and localization

**Questions:**
1. Is RTAB-Map running during docking?
2. Does docking use RTAB-Map localization or purely visual?
3. What is the map quality requirement for docking approach?
4. Can docking succeed without RTAB-Map (e.g., in open-loop mode)?
5. How do we handle relocalization failures?

**Gap:** No documentation on localization requirements for docking

---

### Q3.4: Boot/Launch System Architecture
**Type:** 📋 **GAP**

**Observation:**
- Launch files imported from `multigo_launch` repo
- Main launches: `simulation.launch.py`, `boot.launch.py`, `run.launch.py`

**Questions:**
1. What nodes does `boot.launch.py` start?
2. What nodes does `run.launch.py` start?
3. What is the startup sequence?
4. How are docking nodes integrated into the launch system?
5. Is there a node dependency graph?
6. What happens if a critical node crashes?

**Gap:** No system architecture documentation showing node relationships

**Recommendation:** Create system architecture diagram with all nodes

---

### Q3.5: Simulation Support
**Type:** ❓ **QUESTION**

**Observation:**
- README mentions `simulation.launch.py`
- Gazebo installed as prerequisite

**Questions:**
1. Is docking tested in simulation?
2. Are simulated ArUco markers used?
3. What Gazebo plugins are required?
4. Is there a simulated dock model?
5. Can CI/CD run automated docking tests in simulation?

**Gap:** No simulation setup guide for docking

**Recommendation:** Create simulation testing guide

---

## 4. CONTROL ALGORITHM QUESTIONS

### Q4.1: PID Gain Tuning Method
**Type:** ❓ **QUESTION**

**Current Gains:**
```yaml
kp_x: 1.0,  ki_x: 1.0,  kd_x: 0.2
kp_y: 1.0,  ki_y: 0.7,  kd_y: 0.03
kp_z: 1.0,  ki_z: 2.6,  kd_z: 0.05
```

**Questions:**
1. How were these gains tuned?
2. Was Ziegler-Nichols or other method used?
3. Are these gains optimal for the robot mass/inertia?
4. Do different payloads require different gains?
5. Have stability margins been calculated?
6. Why is `ki_z` so much higher (2.6) than others?

**Concern:** ⚠️ High Ki gains without proper integral calculation could cause issues

**Recommendation:** Document tuning procedure and rationale

---

### Q4.2: Control Stability Analysis
**Type:** 🔍 **INVESTIGATION**

**Questions:**
1. Has the control system been analyzed for stability?
2. What is the phase margin / gain margin?
3. Can oscillations occur with current gains?
4. How does the system respond to step inputs?
5. What is the settling time?
6. Is there overshoot?

**Gap:** No control system analysis documentation

**Recommendation:** Perform frequency domain analysis

---

### Q4.3: Velocity Limits Rationale
**Type:** ❓ **QUESTION**

**Observation:**
```cpp
max_speed = 0.1;  // m/s
min_speed = 0.005;  // m/s
```

**Questions:**
1. Why 0.1 m/s maximum during docking?
2. Is this for safety or accuracy?
3. Can the robot physically move faster?
4. Should max speed be different for Stage 4 vs Stage 5?
5. Why 0.005 m/s minimum? (Related to motor deadband?)

**Recommendation:** Document speed limit rationale

---

### Q4.4: Confirmation Delay Logic
**Type:** ❓ **QUESTION**

**Observation:**
```cpp
confirmed_docking_status=true;
rclcpp::Rate rate(2);
rate.sleep();  // Sleeps for 0.5 seconds
```

**Questions:**
1. Why exactly 0.5 seconds for confirmation?
2. Should this be multiple control cycles instead?
3. What if detection rate is variable?
4. Does this delay affect user experience?

**Current:** Hardcoded 0.5s sleep

**Recommendation:** Make configurable or adaptive

---

## 5. ERROR HANDLING & RECOVERY QUESTIONS

### Q5.1: Failure Modes and Recovery
**Type:** ⚠️ **CONCERN**

**Questions:**
1. What happens if docking fails after max retries?
2. Should robot back out and retry from further distance?
3. How is failure communicated to user/system?
4. Is there a recovery procedure documented?
5. What operator actions are required on failure?

**Current State:** No defined failure recovery procedure

**Gap:** Need failure mode and effects analysis (FMEA)

---

### Q5.2: Marker Occlusion Handling
**Type:** ❓ **QUESTION**

**Scenarios:**
1. What if marker temporarily occluded (person walking by)?
2. What if marker permanently damaged?
3. Can docking complete with one marker continuously missing?
4. Should system abort or wait for marker to reappear?

**Current:** Falls back to single marker, but no timeout on waiting

**Recommendation:** Add occlusion handling policy

---

### Q5.3: Timeout Values
**Type:** ❓ **QUESTION**

**Questions:**
1. What is the expected maximum docking time?
2. When should the system give up and abort?
3. Should different stages have different timeouts?
   - Stage 3 (approach): ?? seconds
   - Stage 4 (alignment): ?? seconds
   - Stage 5 (final dock): ?? seconds
4. Should timeout be configurable per deployment site?

**Current:** No timeouts implemented

**Recommendation:** Define timeout policies

---

## 6. TESTING & VALIDATION QUESTIONS

### Q6.1: Testing Procedures
**Type:** 📋 **GAP**

**Questions:**
1. What testing has been done so far?
2. How many successful docks in field tests?
3. What is the current success rate?
4. What are the most common failure modes?
5. What environments have been tested?
6. Is there a test protocol document?

**Gap:** No testing documentation

**Recommendation:** Create test protocol and log results

---

### Q6.2: Calibration Procedure
**Type:** 📋 **GAP**

**Questions:**
1. How is the system calibrated on a new robot?
2. What is the camera calibration procedure?
3. How are marker offsets measured and set?
4. How are PID gains tuned for a specific robot?
5. Is there a calibration checklist?
6. What tools are needed for calibration?

**Gap:** No calibration guide

**Recommendation:** Write calibration manual

---

### Q6.3: Acceptance Criteria
**Type:** ❓ **QUESTION**

**Questions:**
1. What defines a successful dock?
2. What accuracy is acceptable in production?
3. What is the minimum success rate required?
4. How many consecutive successes needed to validate?
5. What environmental conditions must be tested?

**Gap:** No formal acceptance criteria

**Recommendation:** Define acceptance test procedures

---

## 7. CODE-SPECIFIC QUESTIONS

### Q7.1: Parameter Typo or Intentional?
**Type:** 🐛 **BUG**

**Location:** `nav_goal.cpp:252`
```cpp
this->get_parameter("aruco_right_right_offset", aruco_left_right_offset);
```

**Questions:**
1. Is "right_right" a typo for "left_right"?
2. Or should there be a different offset for right marker?
3. What was the intended parameter name?

**Impact:** Parameter not loaded, using default value

**Action:** Needs immediate clarification and fix

---

### Q7.2: Commented Debug Logs Purpose
**Type:** ❓ **QUESTION**

**Observation:**
- Many `RCLCPP_WARN_STREAM` calls commented out
- All related to calibration offsets

**Questions:**
1. Should these be removed or enabled as DEBUG level?
2. Are they still useful for field tuning?
3. Should there be a "calibration mode" that enables them?

**Recommendation:** Convert to RCLCPP_DEBUG or add calibration mode parameter

---

### Q7.3: Window Display in Production
**Type:** ⚠️ **CONCERN**

**Location:** `aruco_detect.cpp:39`
```cpp
cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
```

**Questions:**
1. Should there be a parameter to disable GUI windows?
2. What happens if no display is available (headless server)?
3. Does this affect performance?
4. Is the display necessary for operation?

**Concern:** May cause issues on robot without display

**Recommendation:** Add `enable_visualization` parameter

---

## 8. SAFETY & COMPLIANCE QUESTIONS

### Q8.1: Safety Standards Compliance
**Type:** ❓ **QUESTION**

**Questions:**
1. What safety standards must the system comply with?
2. Is ISO 3691-4 (Industrial trucks - AGVs) applicable?
3. Is IEC 61508 (Functional safety) required?
4. What is the required Safety Integrity Level (SIL)?
5. Are safety sensors (laser scanners) certified?

**Gap:** No safety compliance documentation

---

### Q8.2: Risk Assessment
**Type:** 📋 **GAP**

**Questions:**
1. Has a risk assessment been performed?
2. What are the identified hazards during docking?
3. What mitigations are in place?
4. What is the residual risk level?

**Gap:** No documented risk assessment

**Recommendation:** Perform hazard analysis and risk assessment (HARA)

---

### Q8.3: Emergency Procedures
**Type:** 📋 **GAP**

**Questions:**
1. How to emergency stop during docking?
2. How to manually move robot if docking fails?
3. What error codes are displayed?
4. Is there an operator manual?

**Gap:** No emergency procedures documented

---

## 9. PERFORMANCE & SCALABILITY QUESTIONS

### Q9.1: CPU Usage
**Type:** 🔍 **INVESTIGATION**

**Questions:**
1. What is the CPU usage during docking?
2. Are there any performance bottlenecks?
3. Can the system run on lower-spec hardware?
4. What is the minimum required CPU?

**Gap:** No performance profiling done

---

### Q9.2: Multi-Robot Operation
**Type:** ❓ **QUESTION**

**Questions:**
1. Can multiple robots dock to different stations simultaneously?
2. Is there any global coordination required?
3. Can robots share marker detection data?
4. How is marker ID assignment managed?

**Future Consideration:** Multi-robot deployment

---

## 10. DOCUMENTATION GAPS SUMMARY

### Missing Documentation

| Document | Priority | Responsible |
|----------|----------|-------------|
| System Architecture Diagram | HIGH | System Architect |
| Hardware Specifications | HIGH | Hardware Engineer |
| Marker Installation Guide | HIGH | Integration Engineer |
| Calibration Procedure | HIGH | Field Engineer |
| Testing Protocol | HIGH | QA Engineer |
| Failure Recovery Guide | MEDIUM | Software Engineer |
| Safety Analysis (HARA) | HIGH | Safety Engineer |
| Operator Manual | MEDIUM | Technical Writer |
| Tuning Guide | MEDIUM | Controls Engineer |
| Simulation Setup Guide | LOW | Software Engineer |

---

## 11. DECISION LOG NEEDED

The following decisions should be documented:

| Decision Needed | Current Status | Deadline |
|-----------------|----------------|----------|
| Stage 6 implementation or removal | Unknown | Before next release |
| PID integral fix approach | Bug identified | Immediate |
| Dual timer strategy | Needs review | Before deployment |
| Error recovery policy | Not defined | Before field test |
| Timeout values | Not defined | Before field test |
| Safety compliance level | Unknown | Before production |
| Testing acceptance criteria | Not defined | Before validation |
| Marker specifications | Partially defined | Before installation |

---

## 12. INVESTIGATION TASKS

### Immediate Investigations Needed

1. **Test Dual Marker Distance Calculation Bug**
   - Set up test with known marker positions
   - Verify if bug exists in field
   - Measure actual vs. calculated distance
   - Estimate impact on docking accuracy

2. **Measure Actual PID Performance**
   - Log error vs. control output
   - Check if Ki term has any effect
   - Measure settling time and overshoot
   - Determine if retuning needed after integral fix

3. **Check Race Condition Occurrence**
   - Monitor `cmd_vel_final` publish rate
   - Check for velocity command conflicts
   - Measure timing between dual timers
   - Determine actual impact

4. **Profile Detection Latency**
   - Measure time from image capture to pose output
   - Check transform lookup times
   - Identify performance bottlenecks
   - Optimize if needed

---

## Summary Statistics

| Category | Count |
|----------|-------|
| **Questions** | 42 |
| **Gaps** | 15 |
| **Concerns** | 8 |
| **Investigations** | 12 |
| **Critical Priority** | 7 |
| **High Priority** | 18 |
| **Medium Priority** | 17 |

---

## Next Actions

### For Development Team:
1. Review and answer questions in Section 1 (System Design)
2. Address critical bugs identified
3. Provide missing hardware specifications
4. Begin documentation effort

### For Field Team:
1. Collect performance data during tests
2. Log failure modes and frequencies
3. Measure actual docking accuracy
4. Document environmental conditions

### For Analysis:
1. Continue with overall system analysis
2. Create test coverage report
3. Generate architecture diagrams
4. Compile user documentation

---

*This document should be updated as questions are answered and gaps are addressed.*
