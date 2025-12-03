# Issues and Fixes - Complete Reference

**Last Updated:** 2025-12-02
**Total Issues:** 28 (10 Critical, 8 High, 7 Medium, 3 Low)

---

## Quick Reference

### By Phase

| Phase | Issues | Effort | Status |
|-------|--------|--------|--------|
| **Phase 1** | CRIT-01, CRIT-02, CRIT-03, CRIT-05, HIGH-01 | 53h | 🔴 Must fix |
| **Phase 2** | CRIT-04 (Testing) | 100h | 🔴 Must have |
| **Phase 3** | CRIT-09, CRIT-10, HIGH-02 to HIGH-08 | 104h | 🟡 Should have |
| **Phase 4** | CRIT-06, CRIT-07, CRIT-08 | 86h | 🟡 Should have |
| **Future** | MED-01 to LOW-03 | 180h | 🟢 Nice to have |

---

## Phase 1: Critical Bugs (Week 1)

### CRIT-01: PID Integral Not Accumulating ⏱️ 4 hours

**File:** `src/nav_docking/nav_docking.cpp:197`

**Problem:** Integral term overwrites instead of accumulates
```cpp
// WRONG:
double integral = error * callback_duration;
```

**Fix:**
```cpp
// Class member:
double integral_dist_ = 0.0;

// In control loop:
integral_dist_ += error * callback_duration;  // Accumulate!
```

**Test:** 10 docking attempts, verify smooth approach + no steady-state error

---

### CRIT-02: Dual Marker Distance Calculation ⏱️ 1 hour

**File:** `src/nav_docking/nav_docking.cpp:387, 503`

**Problem:** Missing parentheses in average
```cpp
// WRONG:
double distance = (left_x) + (right_x) / 2;  // = left + right/2
```

**Fix:**
```cpp
double distance = (left_x + right_x) / 2;  // = (left + right)/2
```

---

### HIGH-01: Uninitialized Variables ⏱️ 1 hour

**File:** `src/nav_docking/nav_docking.cpp` constructor

**Fix:** Initialize all member variables:
```cpp
NavDockingNode::NavDockingNode() : Node("nav_docking_node"),
    first_confirmation_received_(false),
    second_confirmation_received_(false),
    integral_dist_(0.0),
    integral_y_(0.0),
    integral_yaw_(0.0)
{ /* ... */ }
```

---

## Phase 1: Safety Features (Weeks 2-4)

### CRIT-03: No LiDAR During Docking ⏱️ 20 hours

**Problem:** Vision-only = blind to obstacles

**Fix:** Add LiDAR subscription in nav_docking:
```cpp
void scanCallback(LaserScan::SharedPtr msg) {
    double min_dist = *std::min_element(msg->ranges.begin(), msg->ranges.end());
    if (min_dist < 0.30) {  // 30cm safety zone
        obstacle_detected_ = true;
        pauseDocking();
    }
}
```

**Reference:** [IMPLEMENTATION-GUIDE.md - Task 3.2](./IMPLEMENTATION-GUIDE.md#task-32-add-lidar-during-docking-20-hours)

---

### CRIT-05: No Emergency Stop ⏱️ 12 hours

**Fix:** Implement `/emergency_stop` topic, all motion nodes subscribe

**Reference:** [IMPLEMENTATION-GUIDE.md - Task 2.2](./IMPLEMENTATION-GUIDE.md#task-22-implement-emergency-stop-12-hours)

---

### CRIT-06: No Teaching Mode ⏱️ 40 hours

**Phase 4 task**

**What:** Human-guided map building + waypoint saving

**Reference:** [IMPLEMENTATION-GUIDE.md - Week 13-14](./IMPLEMENTATION-GUIDE.md#week-13-14-teaching-mode)

---

### CRIT-07: No Cliff Detection ⏱️ 30 hours

**Phase 4 task** (hardware decision needed)

**What:** IR sensors or depth camera to detect stairs/drops

---

### CRIT-08: No Geofencing ⏱️ 16 hours

**Phase 1 task (basic), Phase 4 (full)**

**What:** Keep-out zones + allowed boundaries

**Reference:** [IMPLEMENTATION-GUIDE.md - Task 4.1](./IMPLEMENTATION-GUIDE.md#task-41-basic-geofencing-16-hours)

---

## Phase 2: Testing

### CRIT-04: Zero Test Coverage ⏱️ 100 hours

**Goal:** 80% automated coverage

**Breakdown:**
- Unit tests: 60 hours (40-50 tests)
- Integration tests: 30 hours (10-15 tests)
- CI/CD: 10 hours

**Reference:** [IMPLEMENTATION-GUIDE.md - Phase 2](./IMPLEMENTATION-GUIDE.md#phase-2-testing-infrastructure)

---

## Phase 3: ROS 2 Improvements

### CRIT-09: No Holonomic Motion Config ⏱️ 8 hours

**Problem:** Mecanum wheels not used (max_vel_y = 0)

**Fix:** Update `nav2_params.yaml`:
```yaml
max_vel_y: 0.15  # Enable sideways!
acc_lim_y: 2.0
```

---

### CRIT-10: No Action Timeouts ⏱️ 12 hours

**Problem:** Actions can hang indefinitely

**Fix:** Add timeout checks in action servers

---

### HIGH-02 to HIGH-08

See detailed implementations in [IMPLEMENTATION-GUIDE.md - Phase 3](./IMPLEMENTATION-GUIDE.md#phase-3-ros-2-best-practices)

---

## Complete Issue List

For full details on all 28 issues, see archived document:
`../archive/IDENTIFIED-ISSUES-AND-GAPS.md`

**Summary by priority:**
- 🔴 Critical (10): Phases 1-2
- 🟡 High (8): Phase 3
- 🟢 Medium (7): Phase 4-5
- 🔵 Low (3): Future

---

**See Also:**
- [IMPLEMENTATION-GUIDE.md](./IMPLEMENTATION-GUIDE.md) - How to fix each issue
- [SYSTEM-ARCHITECTURE.md](./SYSTEM-ARCHITECTURE.md) - Architecture context
- [START-HERE.md](./START-HERE.md) - Overview

**Last Updated:** 2025-12-02
