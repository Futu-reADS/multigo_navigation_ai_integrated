# Outdoor Physical Environment Questionnaire
## Critical Questions for Hardware & Software Design

**Date:** December 8, 2025
**Purpose:** Design decisions for outdoor campus navigation
**Context:** Transitioning from indoor (hospital floors) to outdoor (campus paths/roads)
**Future:** May evolve to support both indoor and outdoor operation

---

## Executive Summary

This questionnaire identifies **critical physical environment factors** that will determine:
1. **Hardware choices** (sensors, motors, wheels, weatherproofing)
2. **Software design** (algorithms, parameters, safety systems)
3. **System architecture** (fallback modes, redundancy, adaptive behavior)
4. **Cost & timeline** (procurement, development, testing)

**Already asked questions** (from your team):
- ✅ Maximum slope degree
- ✅ Ground level clearance for swerve drive
- ✅ Campus park as destination
- ✅ Operational hours (evening/dark time)

**Additional critical questions** (48 more identified below)

---

## Table of Contents

1. [Terrain & Ground Surface](#1-terrain--ground-surface) - 12 questions
2. [Slopes & Grades](#2-slopes--grades) - 6 questions
3. [Obstacles & Clearance](#3-obstacles--clearance) - 8 questions
4. [Weather Conditions](#4-weather-conditions) - 10 questions
5. [Lighting & Visibility](#5-lighting--visibility) - 6 questions
6. [Traffic & Dynamic Environment](#6-traffic--dynamic-environment) - 8 questions
7. [Route Infrastructure](#7-route-infrastructure) - 6 questions
8. [Safety Zones & Boundaries](#8-safety-zones--boundaries) - 6 questions
9. [Operational Scenarios](#9-operational-scenarios) - 8 questions
10. [Summary: Impact Matrix](#summary-impact-matrix)

---

## 1. Terrain & Ground Surface

### Q1.1: Surface Types Distribution
**Question:** What percentage of routes will be on each surface type?

**Options to measure:**
- [ ] Smooth asphalt/concrete: ____%
- [ ] Rough asphalt (cracked): ____%
- [ ] Pavement tiles/bricks: ____%
- [ ] Gravel paths: ____%
- [ ] Grass (manicured): ____%
- [ ] Dirt paths (packed): ____%
- [ ] Mixed (transitions): ____%

**Impact on:**
- **Hardware:**
  - Smooth (>80%) → Mecanum might work (with upgrades)
  - Rough/gravel (>20%) → **Must use swerve drive** or differential
  - Mixed → Need **adaptive suspension** or larger wheel diameter
- **Software:**
  - Smooth → Standard Nav2 costmap (5cm resolution OK)
  - Rough → **Increase costmap resolution to 2cm** (detect potholes)
  - Gravel → Need **point cloud filtering** (ignore ground clutter)
- **Cost Impact:**
  - Smooth: +$0 (current wheels OK with upgrades)
  - Rough: +$1500 (outdoor-grade mecanum)
  - Mixed: +$5000-7000 (swerve drive mandatory)

**Priority:** 🔴 **CRITICAL** - Determines entire wheel system choice

---

### Q1.2: Surface Transitions
**Question:** How frequent are abrupt surface transitions (e.g., asphalt → gravel)?

**Options:**
- [ ] Rare (<5 per route)
- [ ] Occasional (5-15 per route)
- [ ] Frequent (>15 per route)

**Impact on:**
- **Hardware:**
  - Frequent → Need **larger wheels** (>20cm diameter) to handle bumps
  - Frequent → **Suspension required** (spring-loaded wheels)
- **Software:**
  - Frequent → Need **transition detection** (accelerometer/IMU)
  - Must **slow down** before transitions (adaptive speed controller)
- **Cost Impact:**
  - Suspension system: +$2000-3000
  - Larger wheels: +$500-1000

**Priority:** 🟡 **HIGH** - Affects ride comfort and sensor stability

---

### Q1.3: Puddles & Water Accumulation
**Question:** Do paths have standing water after rain? How deep?

**Options:**
- [ ] No water (good drainage)
- [ ] Shallow puddles (<2cm depth): Frequency _____
- [ ] Deep puddles (2-5cm): Frequency _____
- [ ] Flooding (>5cm): Frequency _____

**Impact on:**
- **Hardware:**
  - >2cm deep → **IP67 motor enclosures required** (+$1500)
  - >2cm deep → **Waterproof wheel encoders** (+$800)
  - >5cm deep → **Cannot operate** (need drainage plan)
- **Software:**
  - Need **puddle detection** (camera + LiDAR fusion)
  - **Route planning** to avoid flooded areas
  - Fallback: **Stop and request human assistance**
- **Safety:**
  - Water → **Wheel slip risk** (reduce speed in wet conditions)
  - Water + electricity → **Short circuit risk** (critical)

**Priority:** 🔴 **CRITICAL** - Safety and hardware survival

---

### Q1.4: Curbs & Elevation Changes
**Question:** What is the maximum curb height the robot might encounter?

**Options:**
- [ ] No curbs (smooth paths)
- [ ] Low curbs (2-5cm): Can drive over?
- [ ] Standard curbs (10-15cm): Must avoid
- [ ] Mixed (some low, some high)

**Impact on:**
- **Hardware:**
  - Can drive over 2-5cm → Need **15cm+ ground clearance**
  - Must avoid curbs → **Curb detection sensor** (ultrasonic or 3D camera)
- **Software:**
  - Low curbs → **3D costmap** (detect height changes)
  - High curbs → **Geo-fence** around curb areas
  - Nav2: **Add curb layer** to costmap
- **Cost Impact:**
  - 3D camera (RealSense D435): +$200
  - Ground clearance increase: Chassis redesign +$1000

**Priority:** 🟡 **HIGH** - Prevents robot from getting stuck

---

### Q1.5: Debris & Obstacles on Ground
**Question:** What types of debris are common on paths?

**Check all that apply:**
- [ ] Leaves (autumn)
- [ ] Small branches/sticks
- [ ] Litter (bottles, papers)
- [ ] Gravel/loose stones
- [ ] Sand/dirt
- [ ] Snow/ice (seasonal)
- [ ] Other: ___________

**Impact on:**
- **Hardware:**
  - Leaves/branches → **Debris shields** for wheels (+$200)
  - Loose stones → **Avoid mecanum wheels** (rollers jam)
- **Software:**
  - Need **ground plane segmentation** (LiDAR)
  - Filter out **small obstacles** (<5cm height)
  - Adaptive navigation: **Slow through debris areas**
- **Maintenance:**
  - Mecanum: Clean after **every run** (5-10 min/day)
  - Swerve: Clean **weekly** (30 min/week)

**Priority:** 🟡 **HIGH** - Affects uptime and maintenance cost

---

### Q1.6: Surface Friction Variation
**Question:** Are there areas with low friction (polished stone, wet leaves, ice)?

**Options:**
- [ ] No low-friction areas
- [ ] Seasonal (winter ice): ___months/year
- [ ] Occasional (wet tiles after rain)
- [ ] Frequent (shaded wet areas)

**Impact on:**
- **Hardware:**
  - Low friction → **Need higher traction wheels** (rubber compound)
  - Ice → **Cannot operate safely** (skip winter deployment)
- **Software:**
  - **Wheel slip detection** (compare odometry vs visual/GPS)
  - **Traction control** (reduce acceleration limits)
  - Safety: **Stop if excessive slip detected**
- **Operational:**
  - Winter shutdown? Or invest in **heated wheel systems** (+$3000)?

**Priority:** 🔴 **CRITICAL** - Safety (slip → crash)

---

### Q1.7: Ground Level Roughness (Vibration)
**Question:** How rough/bumpy are the paths? (affects sensor data quality)

**Measurement:** Drive a car at 5 km/h on the path. Is it:
- [ ] Smooth (car barely vibrates)
- [ ] Moderate (noticeable vibration)
- [ ] Rough (strong vibration, coffee would spill)

**Impact on:**
- **Hardware:**
  - Moderate/rough → **Vibration dampening** for sensors (+$500)
  - Rough → **Gimbal for cameras** (stabilize image) (+$1500)
- **Software:**
  - Moderate → **IMU filtering** (remove vibration noise)
  - Rough → **SLAM might fail** (camera blur, LiDAR jitter)
  - Need **sensor fusion** (combine multiple sources)
- **Docking Precision:**
  - Rough → **Cannot achieve ±1mm** (more like ±5-10mm)
  - May need **multi-stage docking** (rough approach → fine positioning)

**Priority:** 🔴 **CRITICAL** - Affects core sensors (RTAB-Map, ArUco)

---

### Q1.8: Compacted Soil vs Loose Soil
**Question:** On dirt paths, is the soil compacted (hard) or loose (sinks under weight)?

**Test:** Step on the path. Does your foot:
- [ ] Stay on surface (compacted)
- [ ] Sink slightly (<1cm)
- [ ] Sink significantly (>2cm)

**Impact on:**
- **Hardware:**
  - Loose soil → **Wider wheels** (distribute weight, prevent sinking)
  - Loose soil → **Cannot use mecanum** (rollers sink and slip)
  - Loose soil → **Swerve with large pneumatic tires** required
- **Software:**
  - Loose soil → **Odometry unreliable** (wheel slip)
  - Must rely on **GPS + visual odometry**
- **Operational:**
  - Loose soil → **Battery drain increases** (motors work harder)
  - Estimate **30-50% shorter battery life**

**Priority:** 🔴 **CRITICAL** - Determines wheel choice

---

### Q1.9: Manhole Covers / Grates
**Question:** Are there metal grates, manhole covers, or drain covers on paths?

**Options:**
- [ ] None
- [ ] Rare (<5 per route)
- [ ] Common (>10 per route)
- [ ] Size: Width ____ cm, Depth ____ cm

**Impact on:**
- **Hardware:**
  - Grates → **Wheel diameter must be > grate gap** (prevent wheel dropping in)
  - Recommendation: **>25cm diameter wheels** if grates common
- **Software:**
  - Metal detection: **LiDAR reflectivity analysis**
  - **Slow down over metal** (slippery when wet)
- **Safety:**
  - Small wheels + grates = **Robot gets stuck** (wheel caught in gap)

**Priority:** 🟡 **MEDIUM** - Preventable with wheel sizing

---

### Q1.10: Underground Utilities / Soft Spots
**Question:** Are there areas where ground might be soft (buried pipes, recent digging)?

**Options:**
- [ ] No concerns
- [ ] Construction zones (temporary): Frequency _____
- [ ] Known soft spots: Locations _____

**Impact on:**
- **Operational:**
  - Soft spots → **Update map regularly** (weekly)
  - Add **keep-out zones** dynamically
- **Software:**
  - **Ground detection** (visual change in terrain texture)
  - **Geofencing** around construction areas
- **Safety:**
  - Soft ground → **Risk of sinking** (robot gets stuck)
  - Need **stuck detection** (motor current spike, no movement)

**Priority:** 🟢 **MEDIUM** - Operational planning issue

---

### Q1.11: Painted Lines / Reflective Markings
**Question:** Are there painted lines, crosswalks, or reflective markers on paths?

**Options:**
- [ ] No markings
- [ ] Lane markings (like roads)
- [ ] Crosswalks
- [ ] Reflective markers

**Impact on:**
- **Software Opportunity:**
  - Can use **line detection** for lane keeping (like autonomous cars)
  - Crosswalks → **Slow down** (pedestrian priority)
- **LiDAR:**
  - Reflective paint → **False obstacles** (high reflectivity)
  - Need **intensity filtering** in costmap

**Priority:** 🟢 **LOW** - Bonus feature, not critical

---

### Q1.12: Expansion Joints / Cracks
**Question:** Are there large cracks or expansion joints (>2cm wide)?

**Options:**
- [ ] None (smooth surfaces)
- [ ] Small cracks (<1cm): OK to ignore
- [ ] Large gaps (2-5cm): Count per route _____
- [ ] Very large (>5cm): Locations _____

**Impact on:**
- **Hardware:**
  - >2cm gaps → **Wheels must be >20cm diameter** (avoid catching)
  - Mecanum rollers → **Will catch in gaps** (avoid mecanum)
- **Software:**
  - **Crack detection** (visual or LiDAR)
  - **Route planning** to avoid large cracks
- **Safety:**
  - Wheel caught in gap → **Robot stuck or damaged**

**Priority:** 🟡 **HIGH** - Determines wheel diameter

---

## 2. Slopes & Grades

### Q2.1: Maximum Slope on Regular Routes (Already Asked)
**Question:** What's the maximum slope degree on regular routes?

**Options:**
- [ ] 0-2° (essentially flat)
- [ ] 2-5° (gentle slope, common in parking lots)
- [ ] 5-10° (noticeable hill)
- [ ] 10-15° (steep, might need help)
- [ ] >15° (very steep, wheelchair users struggle)

**Impact on:**
- **Hardware:**
  - 0-2° → Mecanum OK (with tuning)
  - 2-5° → **Mecanum struggles** (lateral drift)
  - >5° → **Swerve drive required** (better traction)
  - >10° → **Motor power upgrade** needed (+20-30% torque)
- **Software:**
  - >2° → **Tilt compensation** in docking (IMU-based)
  - >5° → **Slope-aware path planning** (avoid uphill when battery low)
  - **Gravity compensation** in PID control
- **Battery:**
  - Every +5° → **~20% more battery consumption**
  - >10° → May need **larger battery** (+$500, +2kg weight)
- **Docking:**
  - 0-1° → ±1mm achievable
  - 2-5° → ±5-10mm realistic
  - >5° → **Docking might fail** (slide during approach)

**Priority:** 🔴 **CRITICAL** - Top impact on system design

---

### Q2.2: Slope Direction Relative to Travel
**Question:** Are slopes typically in direction of travel or across the path?

**Options:**
- [ ] Uphill/downhill (along path)
- [ ] Cross-slope (sideways tilt)
- [ ] Mixed

**Impact on:**
- **Hardware:**
  - Cross-slope → **Worse for mecanum** (slides sideways)
  - Cross-slope + >3° → **Must use swerve drive**
- **Software:**
  - Cross-slope → **Lateral drift compensation** (IMU + wheel odometry)
  - **Increase PID Y-axis gains** (fight sideways slide)
- **Docking:**
  - Cross-slope → **Extremely difficult** with mecanum
  - May need **leveling pads** on docking station

**Priority:** 🔴 **CRITICAL** - Affects mecanum vs swerve decision

---

### Q2.3: Slope Transitions (Crest/Valley)
**Question:** Are there sudden slope changes (hilltops, valley bottoms)?

**Example:** Path goes uphill, then **suddenly levels** (crest)

**Options:**
- [ ] Smooth transitions (gradual)
- [ ] Sharp transitions: Count per route _____

**Impact on:**
- **Hardware:**
  - Sharp transitions → **Suspension required** (prevent wheel lift-off)
  - **Longer wheelbase** helps (4-wheel contact maintained)
- **Software:**
  - **Pitch change detection** (IMU accelerometer)
  - **Slow down before crest** (prevent wheel lift = odometry error)
- **Safety:**
  - Sharp crest → **Wheels can lose contact** → loss of control

**Priority:** 🟡 **HIGH** - Safety (control loss risk)

---

### Q2.4: Downhill Braking
**Question:** On downhills, will robot need to brake to control speed?

**Options:**
- [ ] No steep downhills (no braking needed)
- [ ] Yes, gentle braking on 2-5° slopes
- [ ] Yes, strong braking on >5° slopes

**Impact on:**
- **Hardware:**
  - Braking needed → **Motor controllers must support regenerative braking**
  - Check: Current Phidget BLDC controllers - **Verify braking capability**
  - Might need **mechanical brakes** (rare, +$1000)
- **Software:**
  - **Downhill detection** (IMU pitch)
  - **Speed limiting** (max 0.1 m/s on downhill)
  - **Braking algorithm** (gradual deceleration)
- **Safety:**
  - Runaway on downhill → **Collision risk** (critical)
  - **Emergency stop must work on slopes**

**Priority:** 🔴 **CRITICAL** - Safety (runaway prevention)

---

### Q2.5: Slope Stability for Docking
**Question:** Will docking stations be on slopes? If yes, max slope?

**Options:**
- [ ] Always on flat ground (ideal)
- [ ] Sometimes on slopes: Max ____ degrees
- [ ] Often on slopes: Max ____ degrees

**Impact on:**
- **Docking Design:**
  - On slope → **Docking station needs leveling mechanism** (+$500-1000)
  - Or: **Accept lower precision** (±10mm instead of ±1mm)
- **Software:**
  - **Slope compensation** in docking controller
  - **Adaptive PID gains** (different for uphill vs downhill approach)
- **Success Rate:**
  - Flat: 95%+ success
  - 2° slope: 80% success (estimated)
  - >5° slope: <50% success (mecanum) | 80% (swerve)

**Priority:** 🔴 **CRITICAL** - Affects docking requirement (±1mm)

---

### Q2.6: Emergency Slope Parking
**Question:** If robot must stop mid-route (emergency), are slopes a concern?

**Example:** Battery dies on 5° slope → robot rolls downhill

**Options:**
- [ ] Routes are flat enough (no roll risk)
- [ ] Some slopes → need parking brake
- [ ] Steep slopes → critical concern

**Impact on:**
- **Hardware:**
  - Slopes present → **Parking brake required** (+$500-1000)
  - Types:
    - Electromagnetic brake (locks motors)
    - Mechanical brake (friction pads on wheels)
- **Software:**
  - **Auto-brake on power loss** (failsafe)
  - **Slope detection** → engage brake if stopped on >3° slope
- **Safety:**
  - No brake + slope → **Robot rolls into obstacle/person** (critical)

**Priority:** 🔴 **CRITICAL** - Safety (runaway prevention)

---

## 3. Obstacles & Clearance

### Q3.1: Ground Clearance Requirement (Already Asked)
**Question:** What's the minimum ground clearance needed for swerve drive?

**Recommendation based on terrain:**
- Smooth asphalt: 5-8cm OK
- Rough terrain: 10-15cm recommended
- Mixed with curbs: 15-20cm required

**Current System:**
- Mecanum: ~5cm ground clearance
- Swerve (typical): 10-15cm

**Impact on:**
- **Chassis Design:**
  - Higher clearance → **Taller robot** → higher center of gravity → less stable
  - Trade-off: Clearance vs stability
- **Obstacle Crossing:**
  - 15cm clearance → Can cross 5cm obstacles
- **Cost:**
  - Redesign chassis for higher clearance: +$1000-2000

**Priority:** 🟡 **HIGH** - Determined by Q1.4 (curbs)

---

### Q3.2: Overhead Clearance
**Question:** Are there low-hanging obstacles (tree branches, signs, canopies)?

**Measurement:** Robot height + sensor mast = ____ cm

**Obstacles:**
- [ ] No overhead obstacles
- [ ] Tree branches: Min height ____ cm
- [ ] Signs/canopies: Min height ____ cm
- [ ] Parking barriers: Height ____ cm

**Impact on:**
- **Hardware:**
  - Branches <150cm → **Need overhead clearance sensor** (ultrasonic, +$50)
  - Limit total height to **<120cm** (typical)
- **Software:**
  - **3D costmap** with overhead layer
  - Route planning: **Avoid low-clearance zones**
- **Safety:**
  - Branch hits camera → **Sensor damage** → navigation fail

**Priority:** 🟡 **MEDIUM** - Prevents sensor damage

---

### Q3.3: Width Constraints (Narrow Passages)
**Question:** What's the narrowest passage the robot must navigate?

**Measurement:** Narrowest point = ____ cm wide

**Robot width:** ~0.6m (60cm) currently

**Margin needed:** +20cm (10cm each side) = 80cm minimum

**Impact on:**
- **Robot Design:**
  - If passages <80cm → **Robot must be narrower** (redesign)
  - Current: 60cm wide → needs 80cm passage
- **Software:**
  - **Narrow passage detection** (costmap analysis)
  - **Slow down in narrow areas** (safety)
- **Operational:**
  - Cannot fit → **Route must avoid narrow areas** (geofencing)

**Priority:** 🟡 **MEDIUM** - Determines robot max width

---

### Q3.4: Pedestrian Density
**Question:** How many pedestrians typically on the path?

**Measurement:**
- Peak hours: ____ pedestrians/minute passing
- Off-peak: ____ pedestrians/minute

**Options:**
- [ ] Low (<5/min): Robot rarely encounters people
- [ ] Medium (5-20/min): Frequent encounters
- [ ] High (>20/min): Crowded, constant avoidance

**Impact on:**
- **Hardware:**
  - High density → **Better obstacle detection** (3D camera, +$200)
  - Need **wider LiDAR field of view** (270° → 360°, +$300)
- **Software:**
  - **People tracking** (predict pedestrian motion)
  - **Social navigation** (maintain personal space, 1.5m bubble)
  - **Adaptive speed** (slow in crowds, fast when clear)
- **Success Rate:**
  - High density → **More frequent stops** (wait for clear path)
  - May need **human-robot interaction** (light/sound signals)

**Priority:** 🟡 **HIGH** - User experience and safety

---

### Q3.5: Cyclists & Scooters
**Question:** Are there cyclists or e-scooter users sharing the path?

**Options:**
- [ ] None (pedestrian-only)
- [ ] Rare (<5/hour)
- [ ] Common (>10/hour)
- [ ] Dedicated cycle lane: Separate or shared?

**Impact on:**
- **Safety:**
  - Fast-moving (15-25 km/h) → **Hard to avoid** with slow robot
  - **Collision risk** if robot sudden stops
- **Software:**
  - **Fast object detection** (cyclist approaching from behind)
  - **Predictive avoidance** (clear path before cyclist arrives)
  - **Audio warning** (beep before moving)
- **Operational:**
  - Shared lane → **Robot must yield** (slow, move to side)
  - May need **time-based routing** (avoid peak cyclist hours)

**Priority:** 🔴 **CRITICAL** - Safety (collision with fast objects)

---

### Q3.6: Animals (Dogs, Squirrels, Birds)
**Question:** Are there animals that might interact with the robot?

**Common scenarios:**
- [ ] Dogs (leashed): Owner controls
- [ ] Dogs (unleashed): May approach robot
- [ ] Small animals (squirrels, cats): Darting across path
- [ ] Birds (pigeons): On ground, fly away when approached

**Impact on:**
- **Software:**
  - **Animal detection** (ML model or motion-based)
  - **Unpredictable motion** → conservative obstacle avoidance
  - **Stop if animal nearby** (don't chase/scare)
- **Operational:**
  - Unleashed dogs → **Risk of robot damage** (dog bites sensors)
  - May need **deterrent** (ultrasonic sound, +$100)

**Priority:** 🟢 **LOW** - Rare but consider edge cases

---

### Q3.7: Benches, Trash Cans, Poles
**Question:** Are there fixed obstacles (benches, posts) on or near paths?

**Mapping:**
- [ ] Pre-mapped (static obstacles known)
- [ ] Not mapped (robot discovers during navigation)
- [ ] Movable (trash cans relocated)

**Impact on:**
- **Mapping:**
  - Fixed → **Pre-map environment** (SLAM, one-time)
  - Movable → **Dynamic obstacle detection** (LiDAR, real-time)
- **Software:**
  - **Static layer** in costmap (fixed obstacles)
  - **Dynamic layer** (moving/relocated objects)
- **Operational:**
  - Movable obstacles → **Map must be updated** (weekly/monthly)

**Priority:** 🟡 **MEDIUM** - Affects mapping strategy

---

### Q3.8: Seasonal Obstacles (Snow Piles, Leaf Piles)
**Question:** Do seasonal obstacles appear (snow banks in winter, leaf piles in autumn)?

**Seasons:**
- [ ] Winter: Snow piles, ice patches
- [ ] Autumn: Leaf piles, wet leaves
- [ ] Spring: Mud, puddles
- [ ] Summer: No major obstacles

**Impact on:**
- **Operational:**
  - Seasonal → **Cannot operate year-round** without adaptation
  - May need **seasonal mode** (different parameters per season)
- **Software:**
  - **Obstacle classification** (snow pile = temporary, don't remap)
  - **Seasonal costmap** (expect obstacles in certain areas)
- **Hardware:**
  - Winter → **Heated camera lenses** (+$500)
  - Snow → **Cannot operate** (or invest $5k+ in snow capability)

**Priority:** 🟡 **HIGH** - Determines year-round viability

---

## 4. Weather Conditions

### Q4.1: Operating in Rain
**Question:** Must robot operate in rain? Light or heavy rain?

**Options:**
- [ ] No rain operation (stop if rains)
- [ ] Light rain (<5mm/hr): Must operate
- [ ] Moderate rain (5-20mm/hr): Nice to have
- [ ] Heavy rain (>20mm/hr): Not required

**Impact on:**
- **Hardware:**
  - Light rain → **IP65 enclosures** (motors, electronics) (+$1500)
  - Moderate rain → **IP67 enclosures** (+$2500)
  - Heavy rain → **Not feasible** (sensors fail)
  - **Waterproof cameras** or **rain shields** (+$500)
- **Software:**
  - Rain → **Camera performance degrades** (water drops on lens)
  - **Fallback to LiDAR-only navigation** (RTAB-Map disabled)
  - **Slip detection** (wet = low friction)
- **Operational:**
  - Rain → **Return to shelter mode** (auto-navigate to covered area)

**Priority:** 🔴 **CRITICAL** - Major hardware cost impact

---

### Q4.2: Operating in Wind
**Question:** What wind speeds are typical? Max gusts?

**Measurement:**
- Typical: ____ km/h
- Max gusts: ____ km/h

**Concern threshold:**
- >30 km/h → Light robot may be pushed off course
- >50 km/h → Stability risk (robot might tip)

**Impact on:**
- **Hardware:**
  - High wind → **Lower center of gravity** (heavier base, +1-2kg)
  - **Wind-resistant design** (aerodynamic, lower profile)
- **Software:**
  - **Wind detection** (IMU accelerometer shows lateral force)
  - **Compensate in navigation** (lean into wind, like human walking)
- **Safety:**
  - >40 km/h → **Stop operation** (too dangerous)
  - **Wind speed monitoring** (API or onboard sensor, +$200)

**Priority:** 🟡 **MEDIUM** - Safety (tipping risk)

---

### Q4.3: Temperature Range
**Question:** What are the min/max temperatures?

**Measurement:**
- Summer max: ____ °C
- Winter min: ____ °C

**Concern thresholds:**
- <0°C → Ice, battery performance
- >35°C → Electronics overheating

**Impact on:**
- **Hardware:**
  - <0°C → **Heated battery enclosure** (+$500) or **Li-ion batteries fail**
  - >35°C → **Cooling fans** for electronics (+$200)
  - **Operating range:** -10°C to +45°C (industrial-grade components)
- **Software:**
  - **Temperature monitoring** (shutdown if out of range)
  - Cold → **Battery capacity reduced** (shorter missions)
- **Operational:**
  - Extreme temps → **Limited operation hours** (midday only)

**Priority:** 🟡 **HIGH** - Equipment survival

---

### Q4.4: Humidity & Fog
**Question:** Are there high humidity or fog conditions?

**Options:**
- [ ] Low humidity (<60%)
- [ ] High humidity (>80%): Frequency _____
- [ ] Fog (visibility <50m): Frequency _____

**Impact on:**
- **Hardware:**
  - High humidity → **Conformal coating** on electronics (+$300)
  - Fog → **Camera useless** (cannot see ArUco markers)
- **Software:**
  - Fog → **Switch to LiDAR-only SLAM** (SLAM Toolbox)
  - **GPS still works** (use for localization)
- **Operational:**
  - Dense fog → **Stop operation** (too dangerous, no visibility)

**Priority:** 🟡 **MEDIUM** - Affects sensor choice

---

### Q4.5: Direct Sunlight & Glare
**Question:** Will robot face direct sunlight? Morning/afternoon sun in eyes?

**Scenarios:**
- [ ] Shaded paths (trees, buildings)
- [ ] Full sun exposure: Duration ____ hours/day
- [ ] Sun directly ahead: Times of day _____

**Impact on:**
- **Hardware:**
  - **Camera lens hoods** (+$100) to reduce glare
  - **HDR cameras** (+$400) for bright/dark contrast
- **Software:**
  - **Sun glare → ArUco detection fails** (markers washed out)
  - **Auto-exposure adjustment** (OpenCV camera settings)
  - **Fallback to LiDAR** when camera saturated
- **Operational:**
  - **Route planning** to avoid sun-facing directions (if possible)

**Priority:** 🟡 **MEDIUM** - Vision sensor reliability

---

### Q4.6: Snow & Ice (Already Mentioned)
**Question:** Does it snow? If yes, how much accumulation?

**Options:**
- [ ] No snow (warm climate)
- [ ] Light snow (<5cm): Frequency _____
- [ ] Moderate snow (5-15cm): Frequency _____
- [ ] Heavy snow (>15cm): Frequency _____

**Impact on:**
- **Operational:**
  - Any snow → **Cannot operate** (wheels slip, sensors obscured)
  - Alternative: **Seasonal shutdown** (winter = no robot)
  - Or: **Massive investment** ($10k+) for snow capability (not recommended)
- **Software:**
  - Snow → **Ground detection fails** (white = featureless)
  - **GPS unreliable** (snow blocks signal)

**Priority:** 🔴 **CRITICAL** - Determines year-round viability

---

### Q4.7: Dust & Pollen
**Question:** Are there dusty conditions or high pollen (spring)?

**Options:**
- [ ] No dust/pollen concerns
- [ ] Seasonal pollen: Months _____
- [ ] Dust from construction: Frequency _____

**Impact on:**
- **Hardware:**
  - **Air filters** for cooling fans (+$100)
  - **Sealed enclosures** (prevent dust ingress)
- **Maintenance:**
  - Dust → **Clean sensors weekly** (cameras, LiDAR)
  - Pollen → **May block camera lens** (daily cleaning)

**Priority:** 🟢 **LOW** - Maintenance issue, not critical

---

### Q4.8: Lightning & Thunderstorms
**Question:** Are thunderstorms common? Lightning risk?

**Options:**
- [ ] Rare (<5/year)
- [ ] Seasonal (summer storms): Frequency _____
- [ ] Common (>20/year)

**Impact on:**
- **Safety:**
  - Lightning → **Shelter immediately** (critical)
  - **Weather API integration** (get storm warnings)
- **Operational:**
  - Storm approaching → **Auto-navigate to shelter** (covered area)
  - **Do not operate during storm** (safety policy)

**Priority:** 🟡 **MEDIUM** - Safety policy needed

---

### Q4.9: Saltwater/Coastal Environment
**Question:** Is this a coastal area with salt air?

**Options:**
- [ ] No (inland)
- [ ] Yes (near ocean): Distance ____ km from coast

**Impact on:**
- **Hardware:**
  - Salt air → **Corrosion risk** (electronics, motors)
  - **Stainless steel fasteners** (+$200)
  - **Anti-corrosion coatings** (+$500)
- **Maintenance:**
  - **More frequent inspections** (monthly vs quarterly)

**Priority:** 🟢 **LOW** - Unless coastal

---

### Q4.10: Seasonal Weather Variability
**Question:** How much do conditions vary season-to-season?

**Example:** Summer = dry, smooth paths | Winter = wet, icy, rough

**Options:**
- [ ] Stable year-round
- [ ] Moderate variation (2 seasons)
- [ ] High variation (4 distinct seasons)

**Impact on:**
- **Software:**
  - High variation → **Seasonal configuration profiles**
  - **Auto-detect season** (temperature, precipitation, daylight hours)
  - Load appropriate costmap, speed limits, sensor priorities
- **Testing:**
  - High variation → **Test in all seasons** (longer validation)

**Priority:** 🟡 **MEDIUM** - Software complexity

---

## 5. Lighting & Visibility

### Q5.1: Operational Hours - Darkness (Already Asked)
**Question:** Must robot operate in evening/night (dark)?

**Options:**
- [ ] Daylight only (6am-6pm)
- [ ] Extended hours (6am-9pm): Twilight OK
- [ ] Night operation (6pm-midnight): Full darkness
- [ ] 24/7 operation: All hours

**Impact on:**
- **Hardware:**
  - Twilight → **Brighter LED lights** (+$200)
  - Full darkness → **Headlights + taillights** (+$500)
  - Night → **Thermal camera** (optional, +$1500) for person detection
- **Software:**
  - Dark → **ArUco markers need illumination** (IR LEDs, +$300)
  - **Camera auto-exposure** (wide dynamic range)
  - Dark → **LiDAR becomes primary sensor** (SLAM Toolbox)
- **Safety:**
  - Night → **Must be visible to others** (reflective strips, lights)

**Priority:** 🔴 **CRITICAL** - Major sensor choice impact

---

### Q5.2: Street Lighting Quality
**Question:** Are paths well-lit at night, or pitch black?

**Measurement:** At night, can you read a book outdoors?
- [ ] Well-lit (yes, like daytime)
- [ ] Moderate (dim, can see path but not details)
- [ ] Dark (no, cannot see clearly)
- [ ] Pitch black (no lights at all)

**Impact on:**
- **Hardware:**
  - Well-lit → **Standard cameras OK** (ArUco works)
  - Moderate → **Low-light cameras** (+$400/camera)
  - Dark/pitch black → **Thermal or IR camera** (+$1500)
- **Software:**
  - Dark → **Visual SLAM fails** (RTAB-Map unusable)
  - **Switch to LiDAR SLAM** + GPS
- **Cost Impact:**
  - Well-lit: +$0
  - Moderate: +$800
  - Dark: +$3000 (thermal + IR illumination)

**Priority:** 🔴 **CRITICAL** - Determines camera choice

---

### Q5.3: Shadows & Contrast
**Question:** Are there areas with strong light/shadow contrast (trees, buildings)?

**Example:** Path alternates between bright sunlight and deep shade

**Options:**
- [ ] Even lighting (cloudy, or uniformly lit)
- [ ] Some contrast (occasional shadows)
- [ ] High contrast (frequent sun/shade transitions)

**Impact on:**
- **Hardware:**
  - High contrast → **HDR cameras** (+$400/camera)
  - Or: **Auto-exposure** (software, may lag)
- **Software:**
  - **Adaptive exposure** (adjust camera settings on-the-fly)
  - **Multi-exposure fusion** (combine bright/dark images)
- **Vision Performance:**
  - High contrast → **ArUco detection harder** (markers in shadow)
  - May need **marker illumination** (IR LEDs)

**Priority:** 🟡 **HIGH** - Affects vision reliability

---

### Q5.4: Reflective Surfaces
**Question:** Are there reflective surfaces (glass windows, metal signs, wet pavement)?

**Scenarios:**
- [ ] No reflective surfaces
- [ ] Building windows: Can confuse sensors?
- [ ] Metal poles/signs: LiDAR reflections
- [ ] Wet pavement: Mirror effect

**Impact on:**
- **LiDAR:**
  - Reflective → **False readings** (LiDAR sees reflection, not real object)
  - Need **multi-echo LiDAR** (Hesai might have this, verify)
- **Camera:**
  - Glass → **Glare** (sun reflecting off windows)
  - Wet pavement → **Confuses ground detection** (looks like water)
- **Software:**
  - **Reflectivity filtering** (LiDAR intensity-based)
  - **Ignore highly reflective points** (outlier removal)

**Priority:** 🟡 **MEDIUM** - Sensor noise management

---

### Q5.5: Headlights from Vehicles
**Question:** Will robot cross roads or be near vehicle traffic? (headlights in camera)

**Options:**
- [ ] Pedestrian paths only (no vehicles)
- [ ] Cross roads occasionally: Frequency _____
- [ ] Operate near roads: Headlights visible often

**Impact on:**
- **Camera:**
  - Headlights → **Camera saturation** (ArUco detection fails)
  - Need **HDR or ND filter** (+$200)
- **Safety:**
  - Near roads → **Robot must be highly visible** (reflective, lights)
  - **Traffic signal integration?** (stop at red lights, +$2000 for signal detection)

**Priority:** 🟡 **MEDIUM** - Vision + safety

---

### Q5.6: Signage & Landmarks
**Question:** Are there consistent visual landmarks (buildings, signs) for navigation?

**Options:**
- [ ] Rich landmarks (urban, many buildings)
- [ ] Some landmarks (suburban)
- [ ] Featureless (parks, fields)

**Impact on:**
- **SLAM:**
  - Rich landmarks → **Visual SLAM works well** (RTAB-Map)
  - Featureless → **Visual SLAM fails** (no features to track)
  - Featureless → **Must use LiDAR SLAM** + GPS
- **Software:**
  - **Landmark database** (pre-mapped features for localization)

**Priority:** 🟡 **HIGH** - Affects SLAM strategy

---

## 6. Traffic & Dynamic Environment

### Q6.1: Vehicle Traffic Presence
**Question:** Are there vehicles (cars, trucks, carts) sharing the space?

**Options:**
- [ ] No vehicles (pedestrian-only)
- [ ] Service vehicles (slow, <10 km/h): Frequency _____
- [ ] Regular traffic (cars, 30-50 km/h): Frequency _____

**Impact on:**
- **Safety:**
  - Fast vehicles → **Robot must yield** (stop, move to side)
  - **Detection range must be long** (20m+) to react in time
- **Hardware:**
  - **360° LiDAR** (+$300 vs 270°)
  - **Rear-facing sensors** (detect vehicles approaching from behind)
- **Software:**
  - **Vehicle detection** (LiDAR + camera fusion)
  - **Speed estimation** (predict vehicle path)
  - **Emergency stop** if vehicle too close

**Priority:** 🔴 **CRITICAL** - Safety (collision with vehicles)

---

### Q6.2: Crosswalks & Intersections
**Question:** Will robot cross roads at intersections? Traffic lights?

**Options:**
- [ ] No road crossings
- [ ] Cross at marked crosswalks: Count _____
- [ ] Traffic lights present: Must obey?

**Impact on:**
- **Hardware:**
  - Traffic lights → **Camera for signal detection** (+$500)
  - Or: **V2I communication** (infrastructure sends signal, +$2000)
- **Software:**
  - **Red light detection** (ML model or color-based)
  - **Wait for green** (or pedestrian signal)
  - **Crosswalk detection** (painted lines)
- **Safety:**
  - Must obey traffic laws → **Complex behavior planning**
  - May need **legal approval** for autonomous road crossing

**Priority:** 🔴 **CRITICAL** - Legal and safety

---

### Q6.3: Loading Zones / Temporary Blockages
**Question:** Are there areas that are sometimes blocked (delivery trucks, construction)?

**Options:**
- [ ] Always clear
- [ ] Occasional blockages: Frequency _____
- [ ] Predictable (delivery times known)

**Impact on:**
- **Software:**
  - **Dynamic re-routing** (detect blocked path, find alternate)
  - **Temporary geofencing** (mark blocked area on map)
- **Operational:**
  - **Human oversight** (remote operator updates map)
  - **Communication** (robot requests help if stuck)

**Priority:** 🟡 **MEDIUM** - Operational robustness

---

### Q6.4: Crowds & Events
**Question:** Are there occasional events (gatherings, markets) that change environment?

**Examples:** Weekly farmers market, graduation ceremony

**Options:**
- [ ] No events
- [ ] Rare events (<5/year)
- [ ] Regular events (weekly/monthly): Schedule _____

**Impact on:**
- **Operational:**
  - Events → **Robot cannot operate** (too crowded, paths blocked)
  - **Event calendar integration** (robot auto-pauses during events)
- **Software:**
  - **Crowd density estimation** (LiDAR point count)
  - **Auto-stop** if density > threshold

**Priority:** 🟢 **MEDIUM** - Operational planning

---

### Q6.5: Emergency Vehicles
**Question:** Are there emergency vehicles (ambulances, fire trucks) that might use paths?

**Options:**
- [ ] No emergency vehicle access
- [ ] Rare (emergency driveways nearby)
- [ ] Common (hospital campus, frequent ambulances)

**Impact on:**
- **Safety:**
  - Emergency vehicle → **Robot must clear path immediately**
  - **Siren detection** (audio sensor, +$200)
- **Software:**
  - **Audio processing** (detect siren frequency)
  - **Emergency yield mode** (move to side, stop)
- **Legal:**
  - May be **required by law** (emergency vehicle priority)

**Priority:** 🔴 **CRITICAL** (if applicable) - Legal + safety

---

### Q6.6: Children & Unpredictable Behavior
**Question:** Are there children who might run toward or around the robot?

**Settings:** Schools, playgrounds, family areas

**Options:**
- [ ] Adults only (university, office)
- [ ] Some children (residential areas)
- [ ] Many children (schools, parks)

**Impact on:**
- **Safety:**
  - Children → **Unpredictable motion** (sudden running)
  - **Extra safety margin** (stop distance +50%)
  - **Lower speed** in child-heavy areas (0.1 m/s max)
- **Hardware:**
  - **360° obstacle detection** (child approaching from any direction)
- **Software:**
  - **Human height classification** (child vs adult)
  - **Approach detection** (if human approaching, stop)

**Priority:** 🔴 **CRITICAL** - Safety (vulnerable population)

---

### Q6.7: Queue/Waiting Behavior
**Question:** If path blocked, can robot wait, or must it find alternate route immediately?

**Scenarios:**
- Person stops in path for 30 seconds
- Delivery truck blocks path for 5 minutes

**Options:**
- [ ] Wait up to ____ seconds, then re-route
- [ ] Always re-route immediately (no waiting)

**Impact on:**
- **Software:**
  - **Wait timer** (configurable)
  - **Patience algorithm** (wait vs reroute decision)
- **User Experience:**
  - Too impatient → **Frequent reroutes** (inefficient)
  - Too patient → **Long delays** (user frustration)

**Priority:** 🟢 **LOW** - User experience tuning

---

### Q6.8: Delivery Robots / Other Robots
**Question:** Are there other autonomous robots operating in the same area?

**Examples:** Food delivery robots, security patrol robots

**Options:**
- [ ] No other robots
- [ ] Few other robots: Types _____
- [ ] Many robots (shared space)

**Impact on:**
- **Software:**
  - **Robot-robot communication** (avoid collisions, coordinate)
  - **Multi-agent navigation** (cooperative path planning)
  - Standard: **ROS 2 multi-robot** (namespace-based)
- **Operational:**
  - Shared space → **Traffic management** (like airplanes, assigned routes)

**Priority:** 🟢 **LOW** (unless many robots) - Future consideration

---

## 7. Route Infrastructure

### Q7.1: Path Markings & Signage
**Question:** Are paths marked with signs, arrows, or painted lines?

**Examples:**
- Directional arrows
- "Bike lane" / "Pedestrian only" signs
- Painted centerlines

**Options:**
- [ ] No markings
- [ ] Some signs: Types _____
- [ ] Comprehensive (like roads)

**Impact on:**
- **Software Opportunity:**
  - **Sign recognition** (ML model, +20 hours development)
  - **Lane following** (use painted lines, like Autoware)
- **Navigation:**
  - Can **simplify navigation** (follow marked path)

**Priority:** 🟢 **LOW** - Bonus feature

---

### Q7.2: GPS Signal Quality
**Question:** Are there GPS dead zones (under buildings, tunnels, dense trees)?

**Test:** Walk path with GPS app. Does signal drop?

**Options:**
- [ ] Strong signal everywhere
- [ ] Weak in some areas: Locations _____
- [ ] No signal in tunnels/covered areas: Locations _____

**Impact on:**
- **Hardware:**
  - Weak signal → **RTK GPS** (cm-accuracy, +$500)
  - Dead zones → **GPS cannot be primary** (use SLAM)
- **Software:**
  - **GPS + SLAM fusion** (robot_localization)
  - **GPS dropout handling** (switch to SLAM-only)
- **Operational:**
  - Tunnels → **Pre-map with SLAM** (visual landmarks)

**Priority:** 🔴 **CRITICAL** - Affects localization strategy

---

### Q7.3: Covered Areas (Shade, Tunnels)
**Question:** Are there covered areas (tunnels, building overhangs)?

**Measurement:**
- Covered distance: ____ meters
- Lighting in covered areas: Bright / Dim / Dark

**Options:**
- [ ] All outdoor (no cover)
- [ ] Some shade (trees, overhangs)
- [ ] Tunnels/underpasses: Count _____, Length _____

**Impact on:**
- **GPS:**
  - Covered → **GPS signal lost** (must use SLAM)
- **Lighting:**
  - Dark tunnels → **Headlights required**
- **Software:**
  - **Localization switch** (GPS → SLAM in tunnel)

**Priority:** 🟡 **HIGH** - Affects sensor fusion

---

### Q7.4: Wi-Fi / Cellular Coverage
**Question:** Is there Wi-Fi or cellular coverage along routes?

**Use case:** Remote monitoring, cloud logging, emergency communication

**Options:**
- [ ] Full coverage (Wi-Fi/LTE)
- [ ] Partial coverage: Dead zones _____
- [ ] No coverage (offline operation)

**Impact on:**
- **Operational:**
  - Full coverage → **Cloud logging** (diagnostics, telemetry)
  - No coverage → **Onboard storage** (download later)
- **Hardware:**
  - **LTE modem** (+$200) if cellular preferred
- **Software:**
  - **Offline mode** (store data, sync when connected)
  - **Remote emergency stop** (if coverage reliable)

**Priority:** 🟡 **MEDIUM** - Operational monitoring

---

### Q7.5: Power Outlets / Charging Stations
**Question:** Are there outdoor power outlets for emergency charging?

**Scenario:** Robot battery low, cannot return to base

**Options:**
- [ ] No outdoor power
- [ ] Some outlets: Locations _____
- [ ] Dedicated charging stations: Locations _____

**Impact on:**
- **Operational:**
  - Outlets available → **Emergency charge mode** (navigate to outlet)
  - No outlets → **Must return to base** (plan conservatively)
- **Software:**
  - **Battery-aware routing** (ensure enough charge to return)
  - **Waypoint: nearest charging station**

**Priority:** 🟢 **LOW** - Nice to have

---

### Q7.6: Docking Station Locations
**Question:** Where will docking stations (for wheelchair coupling) be located?

**Options:**
- [ ] Indoors only (covered, protected)
- [ ] Outdoors under shelter (canopy)
- [ ] Outdoors exposed (rain, sun)

**Impact on:**
- **Docking Station Hardware:**
  - Outdoors exposed → **Weatherproof docking station** (+$2000)
  - **Heating** (prevent ice) / **Cooling** (prevent overheat)
- **Software:**
  - **Approach path planning** (avoid puddles near station)
- **Maintenance:**
  - Outdoor → **More frequent inspection** (weekly vs monthly)

**Priority:** 🔴 **CRITICAL** - Docking station design

---

## 8. Safety Zones & Boundaries

### Q8.1: Geofencing / Keep-Out Zones (Mentioned in Docs)
**Question:** Are there areas robot must NEVER enter?

**Examples:**
- Construction zones
- Private property
- Hazardous areas (stairs, water features)

**Options:**
- [ ] No restrictions (entire campus allowed)
- [ ] Some keep-out zones: Count _____, Locations _____
- [ ] Dynamic zones (temporary barriers)

**Impact on:**
- **Software:**
  - **Geofencing** (polygon-based keep-out zones)
  - **Route planning** respects boundaries
  - **Alert if approaching** (slow down, stop)
- **Operational:**
  - **Map keep-out zones** (admin interface)
  - **Update map** when zones change

**Priority:** 🔴 **CRITICAL** - Safety + legal compliance

---

### Q8.2: Water Hazards (Ponds, Fountains)
**Question:** Are there water features near paths (risk of falling in)?

**Examples:**
- Ponds, lakes
- Fountains, pools
- Drainage ditches

**Options:**
- [ ] No water hazards
- [ ] Water present but fenced/protected
- [ ] Open water hazards: Locations _____

**Impact on:**
- **Safety:**
  - Open water → **Robot could drive into water** (critical!)
  - **Cliff detection** (ultrasonic downward-facing, +$100)
- **Software:**
  - **Water detection** (camera or LiDAR reflectivity)
  - **Geofence around water** (keep-out zone)
- **Hardware:**
  - **Cliff sensors** (4× ultrasonics at corners, +$400)

**Priority:** 🔴 **CRITICAL** - Catastrophic failure risk

---

### Q8.3: Stairs & Elevated Platforms
**Question:** Are there stairs or drop-offs near paths?

**Measurement:**
- Stairs present: Yes / No
- Unprotected edges (>10cm drop): Locations _____

**Options:**
- [ ] No stairs/drop-offs
- [ ] Stairs protected by barriers/fences
- [ ] Unprotected drop-offs: Count _____

**Impact on:**
- **Safety:**
  - Drop-offs → **Robot could fall** (critical!)
  - **Cliff detection** (same as water hazards)
- **Software:**
  - **Edge detection** (3D camera or downward LiDAR)
  - **Geofence** around stairs

**Priority:** 🔴 **CRITICAL** - Catastrophic failure risk

---

### Q8.4: Campus Perimeter / Exit Prevention
**Question:** Must robot stay within campus? Risk of exiting to public roads?

**Scenario:** Robot follows path, ends up on public road by mistake

**Options:**
- [ ] Paths are fully enclosed (cannot exit)
- [ ] Open campus, but clear boundaries
- [ ] Risk of exiting to roads: Locations _____

**Impact on:**
- **Software:**
  - **Campus boundary geofence** (do not cross)
  - **Boundary proximity alert** (warn if approaching edge)
- **Safety:**
  - Exit to road → **Traffic danger** (legal liability)
- **Operational:**
  - **Perimeter mapping** (define campus boundary polygon)

**Priority:** 🔴 **CRITICAL** - Legal liability

---

### Q8.5: Accessibility Ramps & Slopes
**Question:** Are accessibility ramps available for wheelchair users?

**Context:** Robot with wheelchair must use same routes as wheelchair users

**Options:**
- [ ] All routes have ramps (ADA compliant)
- [ ] Some routes require stairs (robot cannot use)
- [ ] Robot has different routes than wheelchairs

**Impact on:**
- **Route Planning:**
  - Must use **ADA-compliant routes** (no stairs)
  - **Pre-map accessible routes** only
- **Software:**
  - **Route attributes** (accessibility flag)
  - **Wheelchair-friendly navigation** (avoid steps)

**Priority:** 🟡 **HIGH** - Operational requirement

---

### Q8.6: Secure Areas / Access Control
**Question:** Are there areas requiring access badges, permissions?

**Examples:**
- Staff-only zones
- Building entries with badge readers

**Options:**
- [ ] No access control (all areas public)
- [ ] Some restricted areas: Types _____
- [ ] Access control integration required

**Impact on:**
- **Hardware:**
  - **RFID reader** for badge access (+$200)
  - Or: **Door integration** (robot signals, door opens)
- **Software:**
  - **Access permission database** (robot knows allowed areas)
  - **Request door open** (API call to building system)
- **Operational:**
  - **Robot credentials** (grant access to certain zones)

**Priority:** 🟡 **MEDIUM** - Depends on campus security

---

## 9. Operational Scenarios

### Q9.1: Campus Park as Destination (Already Asked)
**Question:** Can robot navigate through campus park? What are park characteristics?

**Park features:**
- [ ] Paved paths (like roads)
- [ ] Gravel paths
- [ ] Grass paths (mowed)
- [ ] No formal paths (cross grass field)
- [ ] Obstacles: Benches, trees, trash cans

**Impact on:**
- **Hardware:**
  - Grass paths → **Swerve drive required**
  - No formal paths → **Must create path** (GPS waypoints)
- **Software:**
  - Grass → **Different costmap** (no static map, dynamic obstacles)
  - **Pure GPS navigation** (no visual landmarks)
- **Docking:**
  - Park docking → **Uneven ground** (±1mm impossible, more like ±10mm)

**Priority:** 🔴 **CRITICAL** - Determines if park is viable

---

### Q9.2: Building-to-Building Distance
**Question:** What's the longest distance between buildings?

**Measurement:** Max distance = ____ meters

**Battery implications:**
- <100m → Battery consumption negligible
- 100-500m → Moderate battery use
- >500m → Significant (may need larger battery)

**Impact on:**
- **Battery:**
  - >500m → **Larger battery** (+2kg weight, +$500-1000)
  - Or: **Charging station halfway** (infrastructure)
- **Localization:**
  - >100m → **GPS more important** (SLAM drift accumulates)
- **Operational:**
  - >500m → **Mission time increases** (20-30 minutes round trip)

**Priority:** 🟡 **HIGH** - Battery sizing

---

### Q9.3: Loading/Unloading Zones
**Question:** Where will robot pick up and drop off wheelchairs?

**Locations:**
- [ ] Building entrances (covered)
- [ ] Parking lots (exposed)
- [ ] Designated zones (purpose-built)

**Impact on:**
- **Docking:**
  - Outdoor exposed → **Docking in rain/sun** (challenging)
  - Covered → **Easier docking** (stable conditions)
- **Infrastructure:**
  - Designated zones → **Install docking stations** (+$2000 each)
  - Multi-location → **Multiple stations needed**

**Priority:** 🟡 **HIGH** - Docking location design

---

### Q9.4: Mission Duration
**Question:** How long is a typical mission (one-way or round-trip)?

**Scenarios:**
- Building A → Building B: Distance ____ m, Time ____ min
- Return to base: Time ____ min

**Total mission:** ____ minutes

**Impact on:**
- **Battery:**
  - <30 min → **Current battery OK** (estimate 2-4 hours runtime)
  - >60 min → **Larger battery** or **mid-mission charging**
- **Operational:**
  - Long missions → **More wear** (daily maintenance)

**Priority:** 🟡 **MEDIUM** - Battery sizing

---

### Q9.5: Multi-Destination Missions
**Question:** Will robot serve multiple destinations in one trip?

**Example:** Building A → B → C → return to base

**Options:**
- [ ] Single destination (A → B → A)
- [ ] Multiple waypoints (A → B → C → A)
- [ ] On-demand routing (user requests next destination)

**Impact on:**
- **Software:**
  - Multi-waypoint → **Waypoint queue** (MCR-5.1 in requirements)
  - **Route optimization** (shortest path for all stops)
- **Battery:**
  - More stops → **Longer mission** (bigger battery)

**Priority:** 🟡 **MEDIUM** - Feature planning

---

### Q9.6: Emergency Return to Base
**Question:** If robot encounters problem, can it return to base autonomously?

**Scenarios:**
- Low battery
- Sensor failure
- Obstacle blocking path

**Options:**
- [ ] Must return autonomously (no human rescue)
- [ ] Human can retrieve (acceptable to stop in place)

**Impact on:**
- **Software:**
  - Autonomous return → **Robust failsafe navigation** (GPS-only mode)
  - **Emergency route** (pre-mapped, known safe)
- **Safety:**
  - Cannot return → **Robot stuck outdoors** (overnight exposure)
  - **Weatherproofing critical** if stranded

**Priority:** 🔴 **CRITICAL** - Operational robustness

---

### Q9.7: Seasonal Route Changes
**Question:** Do routes change seasonally (e.g., winter path vs summer path)?

**Examples:**
- Summer: Direct path across grass
- Winter: Detour to plowed sidewalk

**Options:**
- [ ] Same routes year-round
- [ ] Seasonal route variants: Count _____

**Impact on:**
- **Software:**
  - **Seasonal maps** (switch map based on season)
  - **Date-based auto-selection**
- **Operational:**
  - **Remap for each season** (4× mapping effort)

**Priority:** 🟢 **LOW** - Unless significant changes

---

### Q9.8: Human Supervision Level
**Question:** Will robot operate fully autonomous, or with human monitoring?

**Options:**
- [ ] Fully autonomous (no human oversight)
- [ ] Remote monitoring (human watches, can intervene)
- [ ] Teleoperation available (human takes control if needed)
- [ ] Always supervised (human nearby)

**Impact on:**
- **Hardware:**
  - Remote monitoring → **LTE modem** (+$200), **cameras for remote view** (+$400)
  - Teleoperation → **Gamepad/joystick interface** (+50 hours software)
- **Software:**
  - **Remote control system** (ROS 2 web interface)
  - **Emergency remote stop** (kill switch)
- **Safety:**
  - Fully autonomous → **Higher safety requirements** (redundancy)
  - Supervised → **Lower risk** (human can intervene)

**Priority:** 🔴 **CRITICAL** - Safety certification level

---

## Summary: Impact Matrix

### Hardware Impact Summary

| Question Category | Hardware Additions | Estimated Cost | Priority |
|-------------------|-------------------|----------------|----------|
| **Terrain (smooth)** | Outdoor mecanum wheels | +$1500 | 🔴 CRITICAL |
| **Terrain (rough)** | Swerve drive system | +$5000-7000 | 🔴 CRITICAL |
| **Slopes (>5°)** | Higher torque motors | +$1000 | 🔴 CRITICAL |
| **Weather (rain)** | IP67 enclosures | +$2500 | 🔴 CRITICAL |
| **Darkness** | Headlights + IR LEDs | +$500 | 🔴 CRITICAL |
| **GPS needed** | RTK GPS + base station | +$800 | 🔴 CRITICAL |
| **IMU** | 9-DOF IMU sensor | +$50-1000 | 🔴 CRITICAL |
| **Cliff detection** | 4× ultrasonic sensors | +$400 | 🔴 CRITICAL |
| **LiDAR upgrade** | 360° LiDAR (if needed) | +$300 | 🟡 HIGH |
| **Weatherproofing** | Heated lenses, seals | +$1500 | 🟡 HIGH |
| **Battery (long range)** | Larger battery pack | +$500-1000 | 🟡 HIGH |
| **Suspension** | Spring-loaded wheels | +$2000 | 🟡 MEDIUM |
| **Remote monitoring** | LTE + cameras | +$600 | 🟡 MEDIUM |
| **TOTAL (Minimum)** | Swerve + sensors + weather | **~$12,000** | - |
| **TOTAL (Conservative)** | All high-priority items | **~$18,000** | - |

---

### Software Impact Summary

| Question Category | Software Additions | Estimated Effort | Priority |
|-------------------|-------------------|-----------------|----------|
| **Swerve drive** | Kinematics, odometry | 80 hours | 🔴 CRITICAL |
| **Sensor fusion** | robot_localization (GPS+IMU+odom) | 30 hours | 🔴 CRITICAL |
| **Costmap tuning** | Multi-layer, outdoor obstacles | 60 hours | 🔴 CRITICAL |
| **SLAM switch** | SLAM Toolbox (LiDAR SLAM) | 30 hours | 🔴 CRITICAL |
| **Slope compensation** | IMU-based PID adjustment | 20 hours | 🔴 CRITICAL |
| **Cliff detection** | Downward sensor integration | 12 hours | 🔴 CRITICAL |
| **Geofencing** | Keep-out zones (basic) | 16 hours | 🔴 CRITICAL |
| **Adaptive speed** | Environment-based speed limits | 12 hours | 🟡 HIGH |
| **Weather fallback** | Camera fail → LiDAR-only mode | 20 hours | 🟡 HIGH |
| **Remote monitoring** | Web interface + telemetry | 40 hours | 🟡 MEDIUM |
| **People tracking** | Pedestrian prediction | 30 hours | 🟡 MEDIUM |
| **Seasonal configs** | Multi-profile management | 16 hours | 🟢 LOW |
| **TOTAL (Minimum)** | Critical items | **~250 hours** | - |
| **TOTAL (Full)** | All items | **~366 hours** | - |

---

### Testing Impact Summary

| Question Category | Testing Additions | Estimated Effort | Priority |
|-------------------|------------------|-----------------|----------|
| **Outdoor surfaces** | Gravel, grass, pavement tests | 40 hours | 🔴 CRITICAL |
| **Slopes** | 2°, 5°, 10° slope tests | 30 hours | 🔴 CRITICAL |
| **Weather** | Rain, wind, temperature tests | 40 hours | 🔴 CRITICAL |
| **Lighting** | Day, twilight, night tests | 30 hours | 🔴 CRITICAL |
| **Docking outdoor** | 100+ attempts, various conditions | 50 hours | 🔴 CRITICAL |
| **Long-range nav** | >100m routes, GPS validation | 30 hours | 🟡 HIGH |
| **Battery life** | Multi-hour endurance tests | 20 hours | 🟡 HIGH |
| **Safety scenarios** | Emergency stop, cliff, obstacles | 40 hours | 🟡 HIGH |
| **Seasonal testing** | If applicable (4 seasons) | 80 hours | 🟢 MEDIUM |
| **TOTAL (Minimum)** | Critical tests | **~190 hours** | - |
| **TOTAL (Comprehensive)** | All test scenarios | **~360 hours** | - |

---

## Prioritized Question List for Immediate Action

### 🔴 ANSWER THESE FIRST (Critical - Determine System Viability)

**Terrain & Motion:**
1. Q1.1: Surface type distribution (mecanum vs swerve decision)
2. Q2.1: Maximum slope on routes (already asked - **GET ANSWER**)
3. Q2.2: Slope direction (cross-slope = critical for mecanum)
4. Q1.7: Ground roughness (affects sensors)
5. Q1.8: Compacted vs loose soil (wheel choice)

**Weather:**
6. Q4.1: Must operate in rain? (IP67 requirement = +$2500)
7. Q4.3: Temperature range (battery heating/cooling)
8. Q4.6: Snow/ice presence (seasonal shutdown?)

**Lighting:**
9. Q5.1: Operational hours - darkness (already asked - **GET ANSWER**)
10. Q5.2: Street lighting quality (camera type choice)

**Localization:**
11. Q7.2: GPS signal quality (GPS viability)
12. Q7.3: Covered areas/tunnels (GPS dropouts)

**Safety:**
13. Q8.2: Water hazards (cliff sensors required)
14. Q8.3: Stairs/drop-offs (cliff sensors required)
15. Q8.4: Campus perimeter (boundary geofencing)
16. Q6.1: Vehicle traffic (safety critical)

**Docking:**
17. Q2.5: Docking on slopes? (precision impact)
18. Q7.6: Docking station locations (indoor vs outdoor)
19. Q9.1: Campus park as destination (already asked - **GET ANSWER**)

**Operational:**
20. Q9.8: Human supervision level (autonomous vs remote)

---

### 🟡 ANSWER THESE NEXT (High - Affects Design Choices)

**Terrain:**
21. Q1.2: Surface transitions (suspension needed?)
22. Q1.3: Puddles/water (waterproofing level)
23. Q1.4: Curbs (ground clearance - already asked - **GET ANSWER**)
24. Q1.5: Debris types (maintenance plan)
25. Q1.12: Large cracks (wheel diameter)

**Slopes:**
26. Q2.3: Slope transitions (suspension)
27. Q2.4: Downhill braking (motor capability)
28. Q2.6: Emergency parking on slope (parking brake)

**Obstacles:**
29. Q3.1: Ground clearance for swerve (already asked - **GET ANSWER**)
30. Q3.2: Overhead clearance (height limit)
31. Q3.4: Pedestrian density (social navigation)
32. Q3.5: Cyclists/scooters (fast object avoidance)
33. Q3.8: Seasonal obstacles (year-round viability)

**Weather:**
34. Q4.2: Wind speeds (stability)
35. Q4.4: Humidity/fog (sensor redundancy)
36. Q4.5: Direct sunlight (camera glare)

**Lighting:**
37. Q5.3: Shadows/contrast (HDR cameras?)
38. Q5.4: Reflective surfaces (LiDAR filtering)

**Traffic:**
39. Q6.2: Crosswalks/traffic lights (legal)
40. Q6.6: Children present (safety margin)

**Route:**
41. Q7.4: Wi-Fi/cellular coverage (remote monitoring)
42. Q9.2: Building-to-building distance (battery sizing)

---

### 🟢 ANSWER THESE LAST (Medium/Low - Nice to Have)

43. Q1.6: Surface friction variation
44. Q1.9: Manhole covers/grates
45. Q1.10: Soft spots
46. Q1.11: Painted lines
47. Q3.3: Width constraints
48. Q3.6: Animals
49. Q3.7: Fixed obstacles (benches)
50. Q4.7: Dust/pollen
51. Q4.8: Thunderstorms
52. Q4.9: Coastal/saltwater
53. Q4.10: Seasonal variability
54. Q5.4: Reflective surfaces
55. Q5.5: Vehicle headlights
56. Q5.6: Landmarks for SLAM
57. Q6.3: Loading zones/blockages
58. Q6.4: Crowds/events
59. Q6.5: Emergency vehicles
60. Q6.7: Queue behavior
61. Q6.8: Other robots
62. Q7.1: Path markings
63. Q7.5: Charging stations
64. Q8.1: Geofencing (already in docs)
65. Q8.5: Accessibility ramps
66. Q8.6: Access control
67. Q9.3: Loading zones
68. Q9.4: Mission duration
69. Q9.5: Multi-destination
70. Q9.6: Emergency return
71. Q9.7: Seasonal routes

---

## Recommended Action Plan

### Step 1: Initial Survey (This Week)
**Goal:** Answer top 20 critical questions

**Method:**
1. **Physical site visit** (2 hours)
   - Walk actual routes with measuring tools
   - Take photos/videos
   - GPS track recording
   - Slope measurements (smartphone inclinometer app)

2. **Campus facilities contact** (1 hour)
   - Ask about weather patterns
   - Construction schedules
   - Access control
   - Safety policies

3. **Team discussion** (1 hour)
   - Review answers
   - Identify showstoppers
   - Preliminary hardware decisions

---

### Step 2: Prototype Testing (Next Week)
**Goal:** Validate critical assumptions

**Method:**
1. **Wheel test** (4 hours)
   - Take current mecanum robot to campus
   - Try different surface types
   - Measure slip, traction, debris accumulation
   - **Decision point:** Mecanum OK or must switch to swerve?

2. **Sensor test** (4 hours)
   - Test camera ArUco detection in various lighting
   - Test LiDAR on outdoor surfaces
   - Measure GPS accuracy (compare to known location)
   - **Decision point:** Which sensors need upgrade?

3. **Environment mapping** (4 hours)
   - Create rough map with SLAM Toolbox (LiDAR)
   - Identify problem areas (blind spots, GPS dropouts)
   - **Decision point:** Is environment SLAM-friendly?

---

### Step 3: Design Document (Week After)
**Goal:** Finalize outdoor system design

**Deliverable:** "Outdoor System Design Specification"

**Contents:**
1. Hardware bill of materials (BOM)
   - Wheels: Mecanum upgrade or swerve ($X)
   - Sensors: GPS, IMU, cameras ($X)
   - Weatherproofing: Enclosures, heating ($X)
   - **Total cost estimate**

2. Software development plan
   - Sensor fusion (robot_localization)
   - Costmap tuning
   - Slope compensation
   - **Total hours estimate**

3. Testing plan
   - Outdoor test scenarios
   - Acceptance criteria
   - Timeline (weeks)

4. Risk assessment
   - Showstoppers identified
   - Mitigation strategies
   - **Go/no-go recommendation**

---

### Step 4: Stakeholder Decision (Final Meeting)
**Goal:** Approve/reject outdoor project

**Decision points:**
1. **Budget:** Can we afford hardware? (~$12-18k)
2. **Timeline:** Can we wait for development? (~6-8 months)
3. **Risk:** Are showstoppers acceptable?
4. **ROI:** Is outdoor capability worth investment?

**Outcomes:**
- ✅ **GO:** Proceed with outdoor development
- 🟡 **GO (LIMITED):** Proceed with smooth-surface-only outdoor (cheaper)
- ❌ **NO-GO:** Stick to indoor operation

---

## Next Steps

**Immediate (Today):**
1. Print this questionnaire
2. Assign team member to conduct site survey
3. Schedule campus walkthrough (2 hours)

**This Week:**
1. Answer top 20 critical questions
2. Take current robot to campus for wheel test
3. Measure slopes, surfaces, lighting

**Next Week:**
1. Compile answers into design document
2. Calculate total cost (hardware + software + testing)
3. Prepare decision presentation for stakeholders

**Within 2 Weeks:**
1. Stakeholder meeting with GO/NO-GO decision
2. If GO: Order long-lead hardware (swerve modules, GPS)
3. If NO-GO: Document why, archive for future

---

**Document End**

**Created:** December 8, 2025
**Author:** Claude AI (Sonnet 4.5)
**Purpose:** Comprehensive outdoor environment analysis for hardware/software design
**Total Questions:** 71 (20 critical, 22 high, 29 medium/low)
**Estimated Impact:** $12-18k hardware, 250-366 hours software, 190-360 hours testing
