# MultiGo System - Overview & User Guide

**Document Version:** 1.0
**Last Updated:** 2025-12-01
**Audience:** New users, operators, project managers
**Reading Time:** 20 minutes

---

## Table of Contents

1. [What is MultiGo?](#what-is-multigo)
2. [System Capabilities](#system-capabilities)
3. [How to Use MultiGo](#how-to-use-multigo)
4. [Understanding the Journey](#understanding-the-journey)
5. [Safety Features](#safety-features)
6. [Common Use Cases](#common-use-cases)
7. [Troubleshooting](#troubleshooting)
8. [FAQs](#faqs)

---

## What is MultiGo?

### Simple Explanation

**MultiGo** is an **autonomous wheelchair transport robot** that can:
- Find wheelchairs automatically
- Navigate safely through buildings
- Dock with millimeter precision
- Transport people autonomously

**Think of it as:** A self-driving car + automatic parallel parking + surgical robot precision, all designed specifically for wheelchair transport.

### Key Features

| Feature | Description | Benefit |
|---------|-------------|---------|
| **Autonomous Navigation** | Finds its way through buildings like GPS | No human driver needed |
| **Millimeter Precision** | Docks within ±1mm accuracy | Safe, reliable connection |
| **Obstacle Avoidance** | Sees and avoids obstacles in real-time | Safe operation in busy environments |
| **User Confirmation** | Asks permission before major actions | Safety and control |
| **Omni-directional Movement** | Can move in any direction | Tight space navigation |

### Real-World Analogy

**MultiGo is like an Uber driver that:**
- Never gets tired or makes mistakes
- Can parallel park perfectly every single time
- Sees in 360 degrees
- Drives extra carefully when carrying passengers
- Always asks permission before doing anything risky

---

## System Capabilities

### 1. Navigation Capabilities

**What it can do:**
- Navigate through complex building layouts
- Remember floor plans automatically (no pre-mapping needed!)
- Plan optimal routes around obstacles
- Adapt to changes in the environment

**Range:** Can navigate 100+ meters through multiple rooms and hallways

**Accuracy:** Knows its position within ±5-10cm

**Speed:** Travels at 0.26 m/s (about 1 km/h - slower than walking, very safe)

### 2. Vision & Detection

**Cameras:**
- Two RGB cameras (left and right)
- 1280×720 resolution, 30 fps
- Detects special markers (ArUco codes) up to 5 meters away

**What it sees:**
- Docking station markers (like runway lights for landing)
- Position and orientation relative to wheelchair
- Exact distance with millimeter accuracy

### 3. Obstacle Detection

**LiDAR Sensor:**
- 360-degree scanning laser
- Detects obstacles up to 30 meters away
- Updates 10 times per second

**What it prevents:**
- Hitting walls
- Bumping into people
- Colliding with furniture
- Running over objects on the floor

### 4. Precision Docking

**Accuracy Levels:**
- **Approach phase:** ±5cm (like parking a car)
- **Alignment phase:** ±1cm (like parallel parking)
- **Final docking:** ±1mm (like plugging in a USB cable)

**Success Rate Target:** 95% successful dockings

**Time:** Complete docking in 60-90 seconds

---

## How to Use MultiGo

### Step-by-Step: Transport a Person

#### Phase 1: Setup (One-time)
1. **Place ArUco markers** on the wheelchair/docking station
   - Left marker: ID 20
   - Right marker: ID 21
   - Distance between markers: ~40cm
   - Height: Eye level for robot cameras

2. **Power on the robot**
   - Wait for system initialization (~30 seconds)
   - Green light indicates ready

#### Phase 2: Request Transport

**Option A: Tell the robot the location**
```
User: "Go to wheelchair in Room 101"
Robot: Navigates to Room 101 area, looks for markers
```

**Option B: Robot scans for markers**
```
User: "Find nearest wheelchair"
Robot: Rotates 360°, scans for ArUco markers
Robot: "Found wheelchair 15 meters away"
```

#### Phase 3: Approach

**What you see:**
```
[Robot displays] "Approach wheelchair? (y/n)"
You: Press 'y'
Robot: Starts moving toward wheelchair
Robot: Navigates around obstacles automatically
Robot: Arrives ~30cm in front of wheelchair
[Robot displays] "Approach complete!"
```

**Timeline:** 30-60 seconds depending on distance

**What's happening behind the scenes:**
- Robot plans safe route through building
- Avoids obstacles in real-time
- Uses LiDAR + maps to navigate
- Stops at safe distance from wheelchair

#### Phase 4: Docking

**What you see:**
```
[Robot displays] "Begin precision docking? (y/n)"
You: Press 'y'
Robot: Starts very slow, careful movements
Robot: Adjusts position (forward, sideways, rotating)
Robot: Makes final approach (inch by inch)
[Robot displays] "Docking complete! Position verified."
```

**Timeline:** 25-30 seconds

**What's happening:**
- Robot sees markers with cameras
- Calculates exact position needed
- Makes tiny adjustments 10 times per second
- Achieves millimeter precision
- Double-checks position before confirming

#### Phase 5: Loading Passenger

**Manual steps:**
1. Help person into wheelchair (if needed)
2. Secure wheelchair to robot platform
3. Verify everything is stable
4. Check that person is comfortable

**On control panel:**
```
[Robot displays] "Passenger loaded? Ready to transport? (y/n)"
You: Press 'y'
```

#### Phase 6: Transport

**What you experience:**
```
[Robot displays] "Destination: Room 205. Confirm? (y/n)"
You: Press 'y'
Robot: Starts moving smoothly
Robot: Takes gentle turns
Robot: Slows down for people crossing
Robot: Arrives at destination
[Robot displays] "Arrived at Room 205"
```

**Speed:** Maximum 0.26 m/s (very comfortable ride)

**Safety:** Constantly monitors surroundings, stops if anything unexpected

#### Phase 7: Unloading

**Manual steps:**
1. Help person exit wheelchair
2. Unsecure wheelchair from platform

**On control panel:**
```
[Robot displays] "Undock? (y/n)"
You: Press 'y'
Robot: Reverses slowly, maintaining alignment
Robot: Backs away to safe distance
[Robot displays] "Undocking complete. Ready for next task."
```

---

## Understanding the Journey

### Complete Timeline Example

**Scenario:** Transport person from Room 101 to Room 205 (100 meters apart)

```
00:00 - User: "Go to wheelchair in Room 101"
00:05 - Robot: Starts navigation
01:00 - Robot: Arrives at Room 101, stops 30cm away
01:05 - System: "Begin docking? (y/n)" → User: "y"
01:30 - System: "Docking complete!"
01:35 - Staff: Helps person into wheelchair, secures it
01:45 - System: "Ready to transport? (y/n)" → User: "y"
01:50 - Robot: Starts transport journey
04:20 - Robot: Arrives at Room 205 (2.5 minute journey)
04:25 - Staff: Helps person exit, unsecures wheelchair
04:30 - System: "Undock? (y/n)" → User: "y"
04:45 - System: "Ready for next task"

Total time: ~4 minutes 45 seconds
```

### What Makes Each Phase Special

#### Navigation Phase (Long Distance)
- **Uses:** RTAB-Map (learns environment) + Nav2 (plans routes)
- **Like:** Google Maps + self-driving car
- **Accuracy:** Gets within 5-10cm of target

#### Approach Phase (Close Range)
- **Uses:** Camera sees one marker
- **Like:** Pulling into parking space
- **Accuracy:** Gets within 5cm of docking position

#### Alignment Phase (Fine Tuning)
- **Uses:** Camera guides adjustments
- **Like:** Fine-tuning car position in tight spot
- **Accuracy:** Gets within 1cm

#### Precision Docking (Final Touch)
- **Uses:** Cameras see BOTH markers for maximum precision
- **Like:** Threading a needle
- **Accuracy:** Gets within 1mm

---

## Safety Features

### Layer 1: User Confirmation
✅ Human approval required for:
- Starting approach
- Beginning docking
- Starting transport
- Any major action

**Why:** You're always in control

### Layer 2: Obstacle Avoidance
✅ LiDAR constantly scanning:
- 360-degree awareness
- 30-meter detection range
- Maintains 0.55m safety distance from obstacles

**Why:** Won't hit anything even if you don't see it

### Layer 3: Speed Limiting
✅ Maximum speeds:
- Navigation: 0.26 m/s (slow walk)
- Docking: 0.1 m/s (very slow)
- With passenger: Even slower, smoother turns

**Why:** Safe, comfortable ride

### Layer 4: Marker Timeout
✅ Vision monitoring:
- If markers not seen for 2 seconds → STOP
- Prevents blind operation

**Why:** Only operates when it can "see" clearly

### Layer 5: Emergency Stop
✅ Can cancel anytime:
- Press cancel button
- Robot stops immediately
- Safe shutdown

**Why:** Ultimate safety control

### Safety Analogy

**MultiGo safety is like driving with:**
- Co-pilot who confirms every turn (User Confirmation)
- 360-degree cameras seeing everything (LiDAR)
- Speed governor preventing speeding (Velocity Limits)
- Automatic braking when obstacle detected (Marker Timeout)
- Emergency brake pedal (E-Stop)

---

## Common Use Cases

### Use Case 1: Elderly Care Facility

**Scenario:** Moving residents between rooms, therapy, dining hall

**Benefits:**
- Reduces staff workload (staff can focus on patient care)
- Consistent, safe transport every time
- No staff fatigue or distraction
- Gentle, comfortable rides for residents

**Workflow:**
1. Resident needs transport from Room 110 to Physical Therapy
2. Staff member initiates MultiGo
3. Robot navigates to Room 110
4. Staff helps resident into wheelchair, confirms transport
5. Robot transports to therapy room
6. Staff helps resident out

**Time saved:** ~5 minutes per transport × 20 transports/day = 100 minutes/day

### Use Case 2: Hospital Patient Transport

**Scenario:** Moving patients between departments

**Benefits:**
- Reduces porter workload
- Predictable timing (helps scheduling)
- Safe transport (multiple safety layers)
- Can integrate with hospital management system

**Workflow:**
1. Nurse requests transport via app: "Patient in Room 305 → X-Ray"
2. Robot navigates to Room 305
3. Nurse confirms patient is ready
4. Robot transports to X-Ray waiting area
5. X-Ray staff receives patient

### Use Case 3: Airport Passenger Assistance

**Scenario:** Helping mobility-impaired passengers get to gates

**Benefits:**
- Faster assistance (no waiting for staff)
- Covers long distances efficiently
- Professional, consistent service
- Handles peak times better

---

## Troubleshooting

### Problem: Robot won't start approach

**Possible causes:**
1. ❌ Markers not visible
   - **Solution:** Ensure markers are in camera view, check lighting
2. ❌ Distance too far (>5m)
   - **Solution:** Move robot closer OR tell robot approximate location
3. ❌ Wrong marker IDs
   - **Solution:** Verify markers are ID 20 (left) and ID 21 (right)

### Problem: Docking fails / Robot stops during docking

**Possible causes:**
1. ❌ One marker became occluded
   - **Solution:** Ensure nothing blocks camera view of markers
2. ❌ Wheelchair moved during docking
   - **Solution:** Ensure wheelchair is stable (brakes locked)
3. ❌ Lighting changed dramatically
   - **Solution:** Maintain consistent lighting

### Problem: Robot stops during navigation

**Possible causes:**
1. ❌ Obstacle in path
   - **Solution:** Wait for obstacle to clear or manually guide robot around
2. ❌ Lost localization (doesn't know where it is)
   - **Solution:** Restart navigation from known position
3. ❌ No valid path to goal
   - **Solution:** Check that goal is reachable, clear obstacles

### Problem: Inaccurate docking

**Possible causes:**
1. ❌ Cameras not calibrated
   - **Solution:** Run camera calibration procedure (see Calibration Guide)
2. ❌ Markers damaged or poorly printed
   - **Solution:** Reprint markers with high-quality printer
3. ❌ Markers not parallel/level
   - **Solution:** Ensure markers are mounted flat and level

---

## FAQs

### Q: How does it know where it is?
**A:** Two ways:
1. **Long distance:** Builds and remembers a map as it explores (RTAB-Map SLAM)
2. **Short distance:** Sees ArUco markers like GPS satellites

### Q: Does it need a pre-made map?
**A:** No! It learns the environment on the fly. First time takes a bit longer as it explores, then it remembers and gets faster.

### Q: What if someone walks in front during docking?
**A:**
- **During navigation:** LiDAR sees them, robot stops or goes around
- **During final docking:** If they block the markers, robot stops (safety feature)

### Q: How accurate is it really?
**A:**
- **Navigation:** ±5-10cm (like parking a car)
- **Final docking:** ±1mm (100x more precise than humans!)

### Q: Can it work in the dark?
**A:**
- **Navigation:** Yes (LiDAR doesn't need light)
- **Docking:** Needs lighting for cameras to see markers

### Q: What happens if battery dies?
**A:**
- Low battery warning with plenty of time to return to base
- If dies during operation: Brakes engage automatically (fail-safe)

### Q: How long does setup take?
**A:**
- **One-time camera calibration:** ~2 hours
- **One-time marker placement:** ~15 minutes per location
- **Daily startup:** ~5 minutes
- **Per-use:** Just press start!

### Q: Can it learn new environments?
**A:** Yes! RTAB-Map automatically builds and remembers new areas. First visit takes longer (exploring), subsequent visits are fast (already knows the way).

### Q: What if the wheelchair is different?
**A:** As long as it has the ArUco markers in the right positions, MultiGo can dock with any wheelchair configuration.

### Q: Is it safe for patients/residents?
**A:** Yes, multiple safety layers:
- User confirmation required
- Obstacle detection and avoidance
- Very slow speeds (comfortable ride)
- Emergency stop available
- Constant monitoring

---

## Next Steps

**For Users/Operators:**
→ Read: [Getting Started Guide](./04-GETTING-STARTED-GUIDE.md)
→ Learn: How to calibrate cameras
→ Practice: Run simulation first before real operation

**For Technical Staff:**
→ Read: [Developer Guide & Architecture](./02-DEVELOPER-GUIDE-ARCHITECTURE.md)
→ Understand: How the system works internally
→ Review: [Requirements Document](./03-REQUIREMENTS-DOCUMENT.md)

**For Project Managers:**
→ Review: System capabilities and limitations
→ Plan: Deployment timeline and training
→ Assess: Use cases for your facility

---

## Related Documents

- **[Developer Guide & Architecture](./02-DEVELOPER-GUIDE-ARCHITECTURE.md)** - Technical deep dive
- **[Requirements Document](./03-REQUIREMENTS-DOCUMENT.md)** - Detailed requirements
- **[Getting Started Guide](./04-GETTING-STARTED-GUIDE.md)** - Quick start instructions
- **[CEA Presentation (Layman)](./CEA_PRESENTATION_LAYMAN_GUIDE.md)** - Detailed component explanations

---

**Document maintained by:** MultiGo Development Team
**For support:** See project README or contact system administrator
**Version history:** See git log for this document
