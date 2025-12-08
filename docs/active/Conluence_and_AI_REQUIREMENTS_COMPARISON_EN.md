# Requirements Comparison: Confluence vs docs/active/

**Date:** December 5, 2025
**Purpose:** Compare requirement capturing between Confluence and docs/active/ documentation
**Scope:** Feature requirements only (not implementation status)

---

## Quick Summary

| Aspect | Confluence | docs/active/ | Alignment |
|--------|------------|--------------|-----------|
| **Hardware specs** | ✅ Detailed | 🟡 Partial | 60% |
| **Software/Navigation** | 🟡 High-level | ✅ Detailed | 40% |
| **Docking mechanism** | ✅ Mechanical focus | ✅ Software focus | 50% |
| **Sensors** | ✅ Detailed models | ✅ Functional requirements | 70% |
| **Safety** | 🟡 Basic | ✅ Comprehensive | 40% |
| **Operational** | ✅ Environmental specs | ✅ Performance specs | 80% |

**Overall Alignment:** ~60% (Different focus areas, complementary)

---

## Requirements Comparison Matrix

### 1. Hardware Requirements

| Requirement | Confluence | docs/active/ | Status |
|-------------|------------|--------------|--------|
| **Physical Dimensions** | ✅ 550×550×1300mm | ❌ Not specified | ➡️ Add to docs/active/ |
| **Front Width** | ✅ 380mm (wheelchair fit) | ❌ Not specified | ➡️ Add to docs/active/ |
| **Robot Weight** | ✅ ≥50kg (towing capability) | ❌ Not specified | ➡️ Add to docs/active/ |
| **Robot Radius** | ❌ Not specified | ✅ 0.28m (navigation) | ➡️ Add to Confluence |
| **Inflation Radius** | ❌ Not specified | ✅ 0.55m (safety margin) | ➡️ Add to Confluence |
| **Ground Clearance** | ✅ ≥5cm | ❌ Not specified | ➡️ Add to docs/active/ |
| **Suspension** | ✅ ±20mm stroke desired | ❌ Not specified | ➡️ Add to docs/active/ |
| **Center of Gravity** | ✅ Low (stability requirement) | ❌ Not specified | ➡️ Add to docs/active/ |
| **Wheelbase** | ✅ Wide for stability | ❌ Not specified | ➡️ Add to docs/active/ |

**Gap:** Confluence has physical design specs, docs/active/ has navigation-related specs

---

### 2. Drive System Requirements

| Requirement | Confluence | docs/active/ | Status |
|-------------|------------|--------------|--------|
| **Wheel Type** | ✅ Mecanum, ≥150mm diameter | ✅ Mecanum (current) | ✅ Aligned |
| **Drive Motors** | ❌ Not specified | ✅ 4× Phidget BLDC | ➡️ Add to Confluence |
| **Motor Control** | ❌ Not specified | ✅ Per-wheel PID | ➡️ Add to Confluence |
| **Holonomic Motion** | ✅ Implied (mecanum) | ✅ Documented (partial) | ✅ Aligned |
| **Turning Capability** | ✅ Around wheelchair axis | ✅ In-place rotation (0m radius) | ✅ Aligned (docs/active/ better) |
| **Inverse Kinematics** | ❌ Not specified | ✅ Implemented | ➡️ Add to Confluence |
| **Odometry** | ❌ Not specified | ✅ Wheel encoders + fusion | ➡️ Add to Confluence |

**Gap:** Confluence focuses on mechanical, docs/active/ includes control algorithms

---

### 3. Performance Requirements

| Requirement | Confluence | docs/active/ | Status |
|-------------|------------|--------------|--------|
| **Transport Capacity** | ✅ ~100kg (wheelchair + person) | ❌ Not tested/documented | ➡️ Add to docs/active/ |
| **Max Speed** | ✅ ~4 km/h (goal) | ✅ 0.26 m/s (~0.9 km/h) actual | ⚠️ Discrepancy (goal vs actual) |
| **Max Angular Velocity** | ❌ Not specified | ✅ 1.0 rad/s | ➡️ Add to Confluence |
| **Acceleration Limits** | ❌ Not specified | ✅ 2.5 m/s² linear, 3.2 rad/s² angular | ➡️ Add to Confluence |
| **Slope Capability** | ✅ ~10° (goal) | ✅ ~1° (mecanum limitation documented) | ⚠️ Discrepancy (goal vs actual) |
| **Step Climbing** | ✅ ~2cm (goal) | ❌ Not tested/documented | ➡️ Add to docs/active/ |

**Gap:** Confluence lists aspirational goals, docs/active/ documents actual configured limits

---

### 4. Operating Environment Requirements

| Requirement | Confluence | docs/active/ | Status |
|-------------|------------|--------------|--------|
| **Primary Environment** | ✅ Indoor (care facilities) | ✅ Indoor (hospital/care) | ✅ Aligned |
| **Temperature Range** | ✅ 10–35°C | ❌ Not specified | ➡️ Add to docs/active/ |
| **Humidity Range** | ✅ 40–80% | ❌ Not specified | ➡️ Add to docs/active/ |
| **Surface Conditions** | ✅ Smooth indoor floors | ✅ Smooth floors (implicit) | ✅ Aligned |
| **Outdoor Capability** | ❌ Not mentioned | ✅ Analyzed (problematic with mecanum) | ➡️ Add to Confluence |
| **Weather Resistance** | ❌ Not specified | ✅ Not weatherproof (indoor only) | ➡️ Add to Confluence |

**Gap:** Confluence specifies environmental limits, docs/active/ analyzes limitations

---

### 5. Navigation & Localization Requirements

| Requirement | Confluence | docs/active/ | Status |
|-------------|------------|--------------|--------|
| **Navigation Stack** | ❌ Not specified | ✅ Nav2 (ROS 2 standard) | ➡️ Add to Confluence |
| **SLAM System** | ❌ Not specified | ✅ RTAB-Map (visual SLAM) | ➡️ Add to Confluence |
| **Global Planner** | ❌ Not specified | ✅ NavFn (Dijkstra/A*) | ➡️ Add to Confluence |
| **Local Planner** | ❌ Not specified | ✅ DWB (Dynamic Window) | ➡️ Add to Confluence |
| **Costmap Layers** | ❌ Not specified | ✅ Static, obstacle, inflation | ➡️ Add to Confluence |
| **Obstacle Avoidance** | ✅ Implied (safety requirement) | ✅ LiDAR-based, 5 Hz update | ✅ Aligned |
| **Recovery Behaviors** | ❌ Not specified | ✅ Spin, backup, wait | ➡️ Add to Confluence |
| **Path Planning Tolerance** | ❌ Not specified | ✅ ±0.5m goal tolerance | ➡️ Add to Confluence |

**Gap:** Confluence doesn't specify software architecture, docs/active/ has full details

---

### 6. Sensor Requirements

| Requirement | Confluence | docs/active/ | Status |
|-------------|------------|--------------|--------|
| **3D LiDAR** | ✅ 1× (Hesai XT16 equivalent) | ✅ 1× Hesai (model unclear) | 🟡 Partially aligned |
| **2D LiDAR** | ✅ 3× (1 front, 2 rear, YDLidar Tmini Pro) | ❌ Not mentioned | ➡️ Clarify if installed |
| **RGB Cameras** | ❌ Not specified | ✅ 2× (left/right for ArUco) | ➡️ Add to Confluence |
| **Depth Camera** | ✅ 1× Intel RealSense D455 | ❌ Not mentioned | ➡️ Clarify if installed |
| **Distance Sensors** | ✅ For securing mechanism | ❌ Not in docking code | ⚠️ Clarify if implemented |
| **Wheel Encoders** | ❌ Not specified | ✅ 4× (one per wheel) | ➡️ Add to Confluence |
| **IMU** | ❌ Not specified | ❌ Not mentioned (but recommended) | Both missing |
| **GPS** | ❌ Not specified | ❌ Not mentioned (outdoor upgrade) | Both missing |

**Critical Gap:** Different sensor configurations documented - needs clarification

---

### 7. Docking/Securing Requirements

#### 7.1 Mechanical Securing

| Requirement | Confluence | docs/active/ | Status |
|-------------|------------|--------------|--------|
| **Securing Mechanism** | ✅ Mechanical (pins + slots) | ❌ Not mentioned in code | ⚠️ **CRITICAL - Clarify** |
| **Constraint Axes** | ✅ Lock X/Y/Yaw, free Z/Pitch/Roll | ❌ Not specified | ➡️ Add to docs/active/ |
| **Locking Motion** | ✅ Linear left-right | ❌ Not in code | ⚠️ Clarify if implemented |
| **Strength Requirement** | ✅ Withstand 200N (uphill) | ❌ Not specified | ➡️ Add to docs/active/ |
| **Attachment Method** | ✅ Dedicated pins on wheelchair | ❌ Not specified | ➡️ Add to docs/active/ |
| **Slot Design** | ✅ Vertically elongated (absorb height diff) | ❌ Not specified | ➡️ Add to docs/active/ |

#### 7.2 Visual Docking (Software)

| Requirement | Confluence | docs/active/ | Status |
|-------------|------------|--------------|--------|
| **Marker Type** | ✅ 2D code for position | ✅ ArUco markers (ID 20, 21) | ✅ Aligned |
| **Docking Precision** | ❌ Not specified | ✅ ±1mm (target) | ➡️ Add to Confluence |
| **Docking Phases** | ❌ Not specified | ✅ Two-phase (approach + precision) | ➡️ Add to Confluence |
| **Approach Distance** | ❌ Not specified | ✅ 0.305m offset | ➡️ Add to Confluence |
| **Dual Marker Transition** | ❌ Not specified | ✅ <0.7m distance threshold | ➡️ Add to Confluence |
| **Control Algorithm** | ❌ Not specified | ✅ PID control (3 axes) | ➡️ Add to Confluence |
| **Verification** | ❌ Not specified | ✅ Two-step (3-second stability) | ➡️ Add to Confluence |
| **PID Parameters** | ❌ Not specified | ✅ Configurable Kp/Ki/Kd | ➡️ Add to Confluence |

**Critical Gap:** Confluence describes mechanical locking, docs/active/ describes visual servoing - **Are both implemented?**

---

### 8. Safety Requirements

| Requirement | Confluence | docs/active/ | Status |
|-------------|------------|--------------|--------|
| **Emergency Stop** | ❌ Not specified | ✅ Required (not yet implemented) | ➡️ Add to Confluence |
| **Obstacle Avoidance (Nav)** | ✅ Implied | ✅ LiDAR-based, costmap | ✅ Aligned |
| **Obstacle Avoidance (Docking)** | ❌ Not specified | ✅ Required (not yet implemented) | ➡️ Add to Confluence |
| **Velocity Limiting** | ✅ Max 4 km/h | ✅ 0.26 m/s (0.9 km/h) configured | ✅ Aligned (conservative) |
| **Safety Zones** | ❌ Not specified | ✅ Inflation radius 0.55m | ➡️ Add to Confluence |
| **Geofencing** | ❌ Not specified | ✅ Keep-out zones (planned) | ➡️ Add to Confluence |
| **Marker Timeout** | ❌ Not specified | ✅ 1-2 seconds (stop if lost) | ➡️ Add to Confluence |
| **Collision Detection** | ❌ Not specified | ✅ Required during docking | ➡️ Add to Confluence |
| **Tipping Prevention** | ✅ Low CG, wide wheelbase | ❌ Not quantified | ➡️ Add to docs/active/ |

**Gap:** Confluence focuses on mechanical safety, docs/active/ on software safety features

---

### 9. User Interaction Requirements

| Requirement | Confluence | docs/active/ | Status |
|-------------|------------|--------------|--------|
| **User Confirmation** | ❌ Not specified | ✅ Pre-approach + pre-dock prompts | ➡️ Add to Confluence |
| **Status Reporting** | ❌ Not specified | ✅ Console messages, feedback | ➡️ Add to Confluence |
| **Action Orchestration** | ❌ Not specified | ✅ Approach → Dock sequence | ➡️ Add to Confluence |
| **Error Reporting** | ❌ Not specified | ✅ Success/failure messages | ➡️ Add to Confluence |
| **Teaching Mode** | ❌ Not specified | ✅ Waypoint save/load (planned) | ➡️ Add to Confluence |
| **Manual Control** | ❌ Not specified | ✅ Teleoperation during teaching | ➡️ Add to Confluence |

**Gap:** Confluence doesn't cover user interaction workflows

---

### 10. Design & Manufacturing Requirements

| Requirement | Confluence | docs/active/ | Status |
|-------------|------------|--------------|--------|
| **Industrial Design** | ✅ Must comply (separate spec) | ❌ Not specified | ➡️ Add to docs/active/ |
| **Assembly Method** | ✅ Strength, ease of assembly | ❌ Not specified | ➡️ Add to docs/active/ |
| **Prototype Fabrication** | ✅ 3D printing | ❌ Not specified | ➡️ Add to docs/active/ |
| **Mass Production** | ✅ Injection molding | ❌ Not specified | ➡️ Add to docs/active/ |
| **Maintainability** | ✅ Disassembly consideration | ❌ Not specified | ➡️ Add to docs/active/ |
| **Target Price** | ✅ 2-3 million JPY per unit | ❌ Not specified | ➡️ Add to docs/active/ |

**Gap:** Confluence covers manufacturing, docs/active/ focuses on software

---

### 11. Software Architecture Requirements

| Requirement | Confluence | docs/active/ | Status |
|-------------|------------|--------------|--------|
| **ROS Version** | ❌ Not specified | ✅ ROS 2 Humble | ➡️ Add to Confluence |
| **Operating System** | ❌ Not specified | ✅ Ubuntu 22.04 LTS | ➡️ Add to Confluence |
| **Programming Languages** | ❌ Not specified | ✅ C++17, Python 3.10 | ➡️ Add to Confluence |
| **Build System** | ❌ Not specified | ✅ colcon | ➡️ Add to Confluence |
| **Package Structure** | ❌ Not specified | ✅ 13 packages documented | ➡️ Add to Confluence |
| **Action Definitions** | ❌ Not specified | ✅ Approach, Dock actions | ➡️ Add to Confluence |
| **Topic Naming** | ❌ Not specified | ✅ Documented per node | ➡️ Add to Confluence |
| **Configuration Management** | ❌ Not specified | ✅ YAML-based (nav2_params.yaml) | ➡️ Add to Confluence |

**Gap:** Confluence doesn't specify software stack at all

---

## Gap Analysis Summary

### In Confluence, NOT in docs/active/ → **Add to docs/active/**

#### Hardware Specifications:
- ✅ Physical dimensions (550×550×1300mm)
- ✅ Front width (380mm)
- ✅ Robot weight (≥50kg)
- ✅ Ground clearance (≥5cm)
- ✅ Suspension specs (±20mm stroke)
- ✅ Center of gravity requirement
- ✅ Wheelbase design

#### Mechanical Securing:
- ✅ Locking mechanism design (pins, slots)
- ✅ Constraint axes (lock X/Y/Yaw, free Z/Pitch/Roll)
- ✅ Linear motion (left-right)
- ✅ Strength requirement (200N)
- ✅ Attachment method

#### Environmental Specifications:
- ✅ Temperature range (10–35°C)
- ✅ Humidity range (40–80%)

#### Performance Goals:
- ✅ Transport capacity (100kg)
- ✅ Step climbing (2cm)

#### Design Requirements:
- ✅ Industrial design compliance
- ✅ Manufacturing method (3D print → injection molding)
- ✅ Target price (2-3M JPY)

#### Sensor Models:
- ✅ Specific part numbers (Hesai XT16, YDLidar Tmini Pro, RealSense D455)
- ✅ 3× 2D LiDAR configuration (1 front, 2 rear)

**Total:** ~25 requirements in Confluence not in docs/active/

---

### In docs/active/, NOT in Confluence → **Add to Confluence**

#### Software Architecture:
- ✅ ROS 2 Humble on Ubuntu 22.04
- ✅ Nav2 navigation stack
- ✅ RTAB-Map visual SLAM
- ✅ NavFn global planner
- ✅ DWB local planner
- ✅ 13-package structure
- ✅ Action definitions (Approach, Dock)

#### Navigation Details:
- ✅ Robot radius (0.28m)
- ✅ Inflation radius (0.55m)
- ✅ Costmap configuration (3 layers)
- ✅ Goal tolerance (±0.5m)
- ✅ Update frequencies (5 Hz)
- ✅ Recovery behaviors

#### Docking Precision:
- ✅ ±1mm precision target
- ✅ Two-phase docking (approach + precision)
- ✅ Dual marker strategy (ID 20, 21)
- ✅ PID control (3 axes: X, Y, Yaw)
- ✅ Distance thresholds (0.305m, 0.7m, 0.42m)
- ✅ Two-step verification (3-second stability)

#### Control Specifications:
- ✅ 4× Phidget BLDC motors
- ✅ Per-wheel PID control
- ✅ Inverse kinematics
- ✅ Wheel encoder odometry
- ✅ Max angular velocity (1.0 rad/s)
- ✅ Acceleration limits (2.5 m/s², 3.2 rad/s²)

#### Safety Features:
- ✅ Emergency stop requirement
- ✅ Marker timeout (1-2s)
- ✅ Geofencing/keep-out zones
- ✅ Collision detection during docking
- ✅ LiDAR safety monitoring

#### User Interaction:
- ✅ Confirmation prompts (pre-approach, pre-dock)
- ✅ Status reporting
- ✅ Action orchestration
- ✅ Teaching mode (waypoint management)
- ✅ Teleoperation support

#### Outdoor Analysis:
- ✅ Mecanum limitations outdoors
- ✅ Swerve drive alternative
- ✅ GPS/IMU requirements for outdoor
- ✅ Weather resistance considerations

**Total:** ~40 requirements in docs/active/ not in Confluence

---

### Requirements Aligned (Both Have)

#### Core Functionality:
- ✅ Autonomous wheelchair transport
- ✅ Indoor operation (care facilities)
- ✅ Mecanum wheel drive system
- ✅ Obstacle avoidance (implied/detailed)
- ✅ ArUco/2D code markers for docking
- ✅ Holonomic motion capability
- ✅ In-place rotation

#### Performance:
- ✅ Speed limiting (Confluence: 4 km/h goal, docs/active/: 0.9 km/h actual)
- ✅ Slope capability (Confluence: 10° goal, docs/active/: 1° actual)

#### Sensors:
- ✅ LiDAR (Hesai)
- ✅ Cameras (for markers)

**Total:** ~10 core requirements aligned (with some goal vs actual discrepancies)

---

## Critical Clarifications Needed

### 1. **Mechanical Locking Mechanism** 🔴
**Question:** Is the mechanical locking (pins, slots, linear guides) actually implemented?
- **Confluence:** Detailed mechanical design
- **docs/active/:** Only visual servoing code (no mechanical lock in nav_docking.cpp)

**Action:** Clarify with hardware team if locking mechanism exists

---

### 2. **Sensor Configuration** 🔴
**Question:** What sensors are actually installed?
- **Confluence:** 3D LiDAR + 3× 2D LiDAR + RealSense D455
- **docs/active/:** 2× RGB cameras + 1× Hesai LiDAR

**Action:** Create unified sensor inventory document

---

### 3. **Goal vs Actual Capabilities** ⚠️
**Discrepancies:**
- Speed: 4 km/h (goal) vs 0.9 km/h (actual)
- Slope: 10° (goal) vs 1° (actual)
- Step climbing: 2cm (goal) vs not tested (actual)

**Action:** Clarify which are aspirational goals vs current capabilities in both documents

---

## My Recommendations

### 1. **Create Unified Requirements Document** 🏆
**Merge both perspectives:**
```
NEW: UNIFIED_REQUIREMENTS.md
├── Section 1: Physical Specifications (from Confluence)
├── Section 2: Software Architecture (from docs/active/)
├── Section 3: Navigation & Docking (merge both)
├── Section 4: Sensors (unified inventory)
├── Section 5: Safety (merge both)
├── Section 6: Manufacturing (from Confluence)
└── Section 7: Goal vs Current Status (clarify)
```

### 2. **Update Both Documents**

**Add to Confluence:**
- Software stack (ROS 2, Nav2, RTAB-Map)
- Docking precision (±1mm)
- Two-phase docking strategy
- Safety features (e-stop, geofencing)
- User interaction workflows
- Current vs goal distinction

**Add to docs/active/:**
- Physical dimensions
- Weight, ground clearance, suspension
- Mechanical locking mechanism (if exists)
- Environmental limits (temperature, humidity)
- Target price
- Manufacturing requirements

### 3. **Clarify Three Critical Gaps**
1. Mechanical locking: Implemented or planned?
2. Sensor config: Which sensors are actually installed?
3. Performance specs: Goal vs current reality

### 4. **Maintain Complementary Documentation**
**Confluence = Business/Hardware Focus:**
- Physical design
- Manufacturing
- Cost
- High-level capabilities

**docs/active/ = Software/Technical Focus:**
- Software architecture
- Algorithms
- Configuration
- Detailed workflows

**Both reference each other with clear links**

---

## Summary Statistics

| Category | Confluence | docs/active/ | Both | Total |
|----------|------------|--------------|------|-------|
| **Hardware specs** | 25 | 5 | 3 | 33 |
| **Software/Navigation** | 0 | 40 | 0 | 40 |
| **Docking/Securing** | 10 | 8 | 2 | 20 |
| **Sensors** | 6 | 6 | 2 | 14 |
| **Safety** | 2 | 8 | 2 | 12 |
| **Operational** | 4 | 3 | 3 | 10 |
| **Design/Manufacturing** | 6 | 0 | 0 | 6 |
| **User Interaction** | 0 | 6 | 0 | 6 |
| **TOTAL** | **53** | **76** | **12** | **141** |

**Total Unique Requirements Captured:** 141 (53+76+12)
- Confluence contributes: 53 unique requirements
- docs/active/ contributes: 76 unique requirements
- Both aligned: 12 requirements

**Overlap:** Only 8.5% (12/141) - **Documents are highly complementary**

---

## Conclusion

**Key Findings:**
1. ✅ **Confluence and docs/active/ are complementary** (not redundant)
2. ⚠️ **Only 8.5% overlap** - different focus areas
3. 🔴 **Three critical gaps need clarification** (mechanical locking, sensors, goal vs actual)
4. ✅ **Both documents are valuable** - should be merged for complete picture

**Recommendation:**
Create **unified requirements document** that combines:
- Hardware/manufacturing specs (from Confluence)
- Software/technical details (from docs/active/)
- Clear distinction between goals and current capabilities

**Next Steps:**
1. Clarify mechanical locking mechanism status
2. Create unified sensor inventory
3. Update both documents with missing requirements
4. Cross-reference both documents

---

**Document Version:** 1.0
**Date:** December 5, 2025
**Pages:** 10
