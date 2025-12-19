# Mechanical System Requirements

**Document Control:**
- **Document ID:** MECH-REQ-001
- **Version:** 1.0
- **Date:** 2025-12-16
- **Status:** Draft
- **Owner:** Tsuchiya (Hardware Team - Mechanical/Electrical Lead)
- **Reviewers:** Pankaj (Software Integration), Kiril (Exterior/eHMI)

---

## 1. Introduction

### 1.1 Purpose

This document specifies the mechanical requirements for the outdoor wheelchair transport robot, including chassis design, swerve drive assembly, wheelchair docking mechanism, sensor mounting, power system integration, and weatherproofing. The mechanical system must support autonomous outdoor navigation, precision docking, and safe passenger transport.

### 1.2 Scope

**In Scope:**
- Chassis and frame structural design
- Swerve drive mechanical assembly (4× modules)
- Wheelchair docking mechanism
- Sensor mounting systems (LiDAR, cameras, IMU)
- Power system mounting (battery, BMS)
- Weatherproofing and environmental protection (IP54+)
- Safety hardware (bumpers, emergency stops)
- Cable routing and wire management
- Assembly procedures and manufacturing specs

**Out of Scope:**
- Electrical system design (covered in ELECTRICAL_REQUIREMENTS.md)
- Exterior components and eHMI hardware (covered in EXTERIOR_REQUIREMENTS.md)
- Software and ROS 2 integration (covered in VEHICLE requirements)

### 1.3 Definitions and Acronyms

| Term | Definition |
|------|------------|
| Swerve Drive | Four independently steered and driven wheels |
| Docking Mechanism | Mechanical system to secure wheelchair |
| IP Rating | Ingress Protection (dust/water resistance) |
| Payload | Maximum wheelchair + passenger weight |
| CoG | Center of Gravity |
| FEA | Finite Element Analysis |
| CAD | Computer-Aided Design |

### 1.4 References

- HARDWARE_SOFTWARE_INTERFACES.md - ROS 2 topics and sensor interfaces
- ELECTRICAL_REQUIREMENTS.md - Power system and wiring
- EXTERIOR_REQUIREMENTS.md - eHMI and safety hardware
- Existing outdoor robot specs (ParcelPal reference, if applicable)

---

## 2. System Overview

### 2.1 Vehicle Configuration

- **Platform Type:** Four-wheel swerve drive (omnidirectional)
- **Dimensions:** Approximately 1200mm (L) × 800mm (W) × 1400mm (H) including docked wheelchair
- **Wheelbase:** 600-700mm (to be finalized)
- **Track Width:** 500-600mm (to be finalized)
- **Ground Clearance:** ≥80mm (outdoor terrain)
- **Payload Capacity:** 150kg (wheelchair 30kg + passenger 120kg)
- **Robot Weight (empty):** Target ≤80kg

### 2.2 Operating Environment

- **Outdoor Usage:** Primary design for outdoor facility pathways
- **Weather Resistance:** IP54 (rain, dust, wind)
- **Temperature Range:** -10°C to +40°C
- **Terrain:** Paved pathways, gentle slopes (≤5°), doorway thresholds
- **Obstacles:** Curbs (≤50mm with ramps), wet surfaces, gravel (limited)

---

## 3. Functional Requirements

### 3.1 Chassis and Frame Structure

#### 3.1.1 Structural Design

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-FRAME-001 | CRITICAL | Chassis SHALL support total payload of 150kg (wheelchair + passenger) | • Static load test: 150kg × 1.5 safety factor = 225kg<br>• No permanent deformation under load<br>• FEA analysis confirms stress <50% yield strength |
| MECH-FRAME-002 | CRITICAL | Chassis SHALL support dynamic loads during motion | • Acceleration: ±0.5 m/s²<br>• Vibration: 10 Hz, 2g peak<br>• Impact: 50mm drop test with payload |
| MECH-FRAME-003 | HIGH | Chassis material SHALL be aluminum alloy 6061-T6 or equivalent | • Lightweight (density ~2.7 g/cm³)<br>• Good strength-to-weight ratio<br>• Corrosion resistant (outdoor use) |
| MECH-FRAME-004 | HIGH | Chassis SHALL provide mounting points for all major components | • Mounting for: swerve modules (4×), battery, compute unit, sensors, docking mechanism<br>• Threaded inserts (M6/M8) or tapped holes<br>• CAD model includes mounting hole pattern |
| MECH-FRAME-005 | MEDIUM | Chassis design SHALL minimize center of gravity height | • CoG height ≤400mm from ground when loaded<br>• Improves stability on slopes<br>• Battery placement low in chassis |
| MECH-FRAME-006 | MEDIUM | Chassis SHALL use welded or bolted construction | • Welded aluminum for primary structure (TIG welding)<br>• Bolted joints (M6/M8 with Loctite) for serviceable components<br>• Weld inspection per standard |
| MECH-FRAME-007 | LOW | Chassis SHALL include lifting/handling points | • 4× lifting lugs (one per corner)<br>• Rated for 100kg lifting capacity<br>• For assembly and maintenance |

#### 3.1.2 Dimensional Requirements

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-DIM-001 | HIGH | Vehicle overall length SHALL not exceed 1500mm | • Length measured with wheelchair docked<br>• Fits through standard doorways (≥750mm width)<br>• Allows for turning radius in hallways |
| MECH-DIM-002 | HIGH | Vehicle overall width SHALL not exceed 900mm | • Width allows passing through 900mm doorways with clearance<br>• Stable on 500-600mm track width |
| MECH-DIM-003 | HIGH | Vehicle overall height SHALL not exceed 1600mm | • Height with wheelchair docked and passenger seated<br>• Clearance for ceiling height (2m standard) |
| MECH-DIM-004 | MEDIUM | Ground clearance SHALL be ≥80mm | • Measured at lowest point (excluding wheels)<br>• Allows for 50mm curbs with approach angle<br>• Avoids belly scraping on uneven ground |

---

### 3.2 Swerve Drive Mechanical Assembly

#### 3.2.1 Swerve Module Design

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-SWERVE-001 | CRITICAL | Each swerve module SHALL have independent steering and drive | • Steering motor rotates wheel assembly ±180°<br>• Drive motor propels wheel forward/backward<br>• No mechanical coupling between modules |
| MECH-SWERVE-002 | CRITICAL | Swerve module SHALL use 6.5" hoverboard in-wheel motor | • Motor spec: 350W, 36V nominal, hall sensors<br>• Pneumatic tire (6.5" diameter, inflatable for cushioning)<br>• Motor mounting via custom bracket |
| MECH-SWERVE-003 | HIGH | Steering axis SHALL use precision bearing for low friction | • Angular contact ball bearing or tapered roller bearing<br>• Preloaded to eliminate play<br>• Sealed bearing (weatherproof) |
| MECH-SWERVE-004 | HIGH | Steering mechanism SHALL use belt or gear transmission | • Option 1: Timing belt (GT2, 3:1 or 5:1 reduction)<br>• Option 2: Planetary gearbox (off-the-shelf, 1:4 ratio)<br>• Backlash <0.5° for precise steering |
| MECH-SWERVE-005 | HIGH | Swerve module SHALL include absolute encoder for steering angle | • Magnetic encoder (AS5048A or similar, 14-bit resolution = 0.022°)<br>• Mounted on steering axis<br>• Retains position when powered off |
| MECH-SWERVE-006 | MEDIUM | Swerve module design SHALL allow wheel replacement without full disassembly | • Quick-release axle or bolt pattern<br>• Max 15 minutes per wheel change<br>• Documented procedure |
| MECH-SWERVE-007 | MEDIUM | Swerve module SHALL have cable routing through hollow steering axis | • Slip ring (4-12 wires) or hollow shaft for drive motor wires<br>• Prevents cable tangling during steering<br>• Rated for continuous rotation |

#### 3.2.2 Module Mounting and Alignment

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-SWERVE-MOUNT-001 | HIGH | Swerve modules SHALL mount to chassis with adjustable alignment | • Slotted holes for ±2mm X/Y adjustment<br>• Shims for height adjustment<br>• Alignment pins for repeatability |
| MECH-SWERVE-MOUNT-002 | HIGH | Module mounting SHALL support static load of 60kg per module | • 240kg total (1.5× safety factor for 150kg payload + 80kg robot)<br>• Mounting bolts: M8 Grade 8.8 minimum<br>• Torque specification: 25 Nm |
| MECH-SWERVE-MOUNT-003 | MEDIUM | Modules SHALL be positioned at chassis corners for maximum stability | • Rectangular layout, equal spacing<br>• Wheelbase and track width optimized for maneuverability vs stability<br>• CAD model confirms CoG within support polygon |

#### 3.2.3 Wheel and Tire Specifications

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-WHEEL-001 | HIGH | Tires SHALL be pneumatic for outdoor shock absorption | • 6.5" inflatable tire (standard hoverboard tire)<br>• Pressure: 30-40 PSI<br>• Tread pattern for traction on wet surfaces |
| MECH-WHEEL-002 | MEDIUM | Spare tires and tubes SHALL be included in initial delivery | • 2× spare tires, 2× spare inner tubes<br>• Tire pump and repair kit<br>• Documented tire change procedure |
| MECH-WHEEL-003 | LOW | Wheels SHALL have colored markings for easy identification | • Front-left: red, front-right: blue, rear-left: green, rear-right: yellow<br>• Helps during assembly and troubleshooting |

---

### 3.3 Wheelchair Docking Mechanism

#### 3.3.1 Docking Interface Design

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-DOCK-INT-001 | CRITICAL | Docking mechanism SHALL securely hold standard manual wheelchair | • Compatible with wheelchair widths 600-700mm<br>• Clamping force sufficient to prevent movement during transit<br>• No damage to wheelchair frame |
| MECH-DOCK-INT-002 | CRITICAL | Docking mechanism SHALL use visual markers for alignment | • 2× ArUco markers (front and side) on docking interface<br>• Marker size: 200mm × 200mm<br>• High-contrast (black/white) for camera detection |
| MECH-DOCK-INT-003 | HIGH | Docking SHALL engage/disengage without manual intervention | • Electric actuator (linear actuator or servo motor)<br>• Docking time: <10 seconds<br>• Undocking time: <5 seconds |
| MECH-DOCK-INT-004 | HIGH | Docking mechanism SHALL include physical stops for wheelchair positioning | • Hard stops prevent wheelchair from rolling backward during docking<br>• Padded contact surfaces (rubber or foam)<br>• Height adjustable (±30mm) for different wheelchair models |

#### 3.3.2 Docking Actuator and Locking

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-DOCK-ACT-001 | HIGH | Docking actuator SHALL provide ≥200N clamping force | • Sufficient to prevent wheelchair movement during 0.5 m/s² acceleration<br>• Linear actuator (12V/24V, 200mm stroke) recommended<br>• Feedback: limit switches at engaged/disengaged positions |
| MECH-DOCK-ACT-002 | HIGH | Docking mechanism SHALL include safety interlock | • Cannot undock while vehicle is in motion (speed >0.1 m/s)<br>• Software interlock via ROS 2 topic (/safety/docking_lock)<br>• Manual override button (for emergency) |
| MECH-DOCK-ACT-003 | MEDIUM | Docking mechanism SHALL have manual release mechanism | • Mechanical lever accessible from outside robot<br>• Releases clamp in case of power failure<br>• Clearly labeled "EMERGENCY RELEASE" |

#### 3.3.3 Docking Sensor Integration

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-DOCK-SENS-001 | HIGH | Docking mechanism SHALL include contact sensors for wheelchair detection | • Proximity sensors (capacitive or IR) detect wheelchair presence<br>• Publishes to ROS 2 topic: /docking/wheelchair_detected (bool)<br>• Mounted on docking interface |
| MECH-DOCK-SENS-002 | MEDIUM | Docking mechanism SHALL include ArUco marker mounting plates | • Rigid mounting for 2× ArUco markers<br>• Markers perpendicular to camera view<br>• Protected from weather (clear acrylic cover) |

---

### 3.4 Sensor Mounting Systems

#### 3.4.1 LiDAR Mounting

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-LIDAR-001 | CRITICAL | LiDAR SHALL mount at height 600-800mm from ground | • Field of view unobstructed by vehicle body<br>• Detects obstacles and terrain ahead<br>• Horizontal 360° scan or multi-plane 3D LiDAR |
| MECH-LIDAR-002 | HIGH | LiDAR mounting SHALL be vibration-isolated | • Rubber dampers or spring-loaded mount<br>• Reduces vibration from driving on rough terrain<br>• Mounting rigidity: natural frequency >50 Hz |
| MECH-LIDAR-003 | HIGH | LiDAR SHALL have weatherproof enclosure | • IP65 rated enclosure<br>• Clear window (polycarbonate or glass) for laser transmission<br>• Heating element (optional, for defogging) |
| MECH-LIDAR-004 | MEDIUM | LiDAR mounting SHALL allow ±10° tilt adjustment | • Adjustable bracket for pitch/roll alignment<br>• Alignment to be perpendicular to ground plane<br>• Locking mechanism after adjustment |

#### 3.4.2 Camera Mounting

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-CAM-001 | CRITICAL | Two cameras SHALL mount on front of vehicle for stereo vision | • Camera separation: 150-250mm (baseline)<br>• Height: 800-1000mm from ground<br>• Forward-facing (0° ± 5° downward tilt) |
| MECH-CAM-002 | HIGH | Camera mounts SHALL be rigid (no flex) | • Aluminum or steel bracket<br>• Rigidity test: <0.5mm deflection under 5N load<br>• Prevents stereo calibration drift |
| MECH-CAM-003 | HIGH | Cameras SHALL have weather-resistant housings | • IP54 rated housing<br>• Lens protection (UV filter or glass window)<br>• Heating element or anti-fog coating |
| MECH-CAM-004 | MEDIUM | Camera mounting SHALL allow lens adjustment | • Pan/tilt adjustment (±5° each axis)<br>• Focus lock (for fixed-focus lenses)<br>• Cable strain relief |

#### 3.4.3 IMU Mounting

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-IMU-001 | HIGH | IMU SHALL mount rigidly to chassis center | • Near center of gravity (CoG)<br>• Aluminum mounting plate (no flex)<br>• Axes aligned with vehicle frame (X: forward, Y: left, Z: up) |
| MECH-IMU-002 | MEDIUM | IMU mounting SHALL be vibration-isolated (optional) | • Low-pass vibration damping if needed<br>• Affects high-frequency measurements<br>• May not be needed with rigid mount |

---

### 3.5 Power System Mounting

#### 3.5.1 Battery Mounting

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-BAT-MOUNT-001 | CRITICAL | Battery SHALL mount low in chassis for low CoG | • Placement in bottom 1/3 of vehicle height<br>• Central location (between swerve modules)<br>• Secure mounting to prevent movement |
| MECH-BAT-MOUNT-002 | CRITICAL | Battery mounting SHALL support battery weight (≥15kg for 48V 20Ah) | • Mounting tray or bracket (steel or aluminum)<br>• Bolted or riveted to chassis<br>• Vibration-resistant (rubber padding) |
| MECH-BAT-MOUNT-003 | HIGH | Battery SHALL be easily removable for charging | • Slide-in tray with locking mechanism<br>• OR hinged access panel with quick-release connectors<br>• Max 5 minutes to remove/reinstall |
| MECH-BAT-MOUNT-004 | HIGH | Battery compartment SHALL be weatherproof | • IP54 rated enclosure<br>• Sealed lid with gasket<br>• Drain holes for condensation |
| MECH-BAT-MOUNT-005 | MEDIUM | Battery mounting SHALL include thermal management | • Ventilation slots or fan for cooling<br>• Temperature sensor (BMS integrated)<br>• Prevents overheating during fast charging |

#### 3.5.2 BMS and Power Distribution Mounting

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-BMS-001 | HIGH | BMS SHALL mount in same compartment as battery | • Direct connection to battery pack<br>• Secure mounting (vibration-resistant)<br>• Accessible for diagnostics |
| MECH-BMS-002 | MEDIUM | Power distribution panel SHALL be centrally located | • Fuses, contactors, DC-DC converters<br>• Accessible for maintenance<br>• Labeled wiring |

---

### 3.6 Weatherproofing and Environmental Protection

#### 3.6.1 Ingress Protection (IP Rating)

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-WEATHER-001 | CRITICAL | Vehicle SHALL achieve IP54 rating overall | • Dust protected (5): Dust ingress limited, no harmful deposit<br>• Splash water protected (4): Water splashing from any direction<br>• Tested per IEC 60529 standard |
| MECH-WEATHER-002 | HIGH | All electrical enclosures SHALL have IP65 rating minimum | • Compute box, BMS enclosure, motor controllers<br>• Dust-tight (6), water jet protected (5)<br>• Gaskets on all access panels |
| MECH-WEATHER-003 | HIGH | Cable entry points SHALL be sealed | • Waterproof cable glands (PG9, PG13 sizes)<br>• Silicone sealant for permanent cables<br>• No exposed wire entry points |

#### 3.6.2 Drainage and Ventilation

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-DRAIN-001 | MEDIUM | Chassis SHALL include drainage holes for water egress | • Bottom of enclosures has 3-5mm drain holes<br>• Prevents water pooling<br>• Covered with mesh to prevent debris entry |
| MECH-DRAIN-002 | MEDIUM | Ventilation SHALL be provided for heat dissipation | • Passive vents (louvered) or active fans<br>• Prevents overheating of compute unit and motor controllers<br>• Filtered to prevent dust ingress |

#### 3.6.3 Material Corrosion Resistance

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-CORR-001 | HIGH | All metal parts SHALL be corrosion-resistant | • Aluminum (anodized or powder-coated)<br>• Stainless steel (304/316) for fasteners<br>• Zinc-plated or galvanized steel where needed |
| MECH-CORR-002 | MEDIUM | Fasteners SHALL be stainless steel or coated | • Stainless steel A2/A4 grade preferred<br>• OR zinc-plated Grade 8.8 with anti-seize compound<br>• Prevents rust and seizing |

---

### 3.7 Safety Hardware

#### 3.7.1 Bumper System

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-BUMP-001 | CRITICAL | Vehicle SHALL have front and rear bumpers with contact sensors | • Bumpers around entire perimeter (front, rear, sides)<br>• Pressure-sensitive or limit switch based<br>• Publishes to ROS 2 topic: /safety/bumper (bool array) |
| MECH-BUMP-002 | HIGH | Bumpers SHALL absorb low-speed impacts (≤0.5 m/s) | • Foam padding (EVA or polyurethane)<br>• Compressible 10-20mm<br>• Protects vehicle and obstacles |
| MECH-BUMP-003 | MEDIUM | Bumpers SHALL be replaceable modular design | • Bolt-on bumper sections<br>• Spare bumper pads included<br>• Easy replacement (30 minutes max) |

#### 3.7.2 Emergency Stop Hardware

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-ESTOP-001 | CRITICAL | Vehicle SHALL have hardwired emergency stop button | • Red mushroom-head button, twist-to-release<br>• Accessible from outside vehicle<br>• Hardwired to motor power relays (immediate cutoff) |
| MECH-ESTOP-002 | HIGH | E-stop SHALL be mounted at waist height (1000-1200mm) | • Easy to reach by standing person<br>• Protected from accidental activation (recessed or guarded)<br>• Labeled "EMERGENCY STOP" in EN and JA |

#### 3.7.3 Passenger Detection

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-PASS-001 | HIGH | Docking mechanism SHALL include weight sensor for passenger detection | • Load cell or pressure sensor (0-150kg range)<br>• Detects passenger seated in wheelchair<br>• Publishes to ROS 2 topic: /safety/passenger_detected (bool) |
| MECH-PASS-002 | MEDIUM | Passenger detection SHALL prevent undocking when passenger present | • Software interlock (ROS 2)<br>• Warning beep if undocking attempted with passenger<br>• Manual override (admin only) |

---

### 3.8 Cable Routing and Wire Management

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-CABLE-001 | HIGH | All cables SHALL be routed through cable trays or conduits | • Prevents cable damage from moving parts<br>• Organized by type (power, signal, data)<br>• Secured with cable ties every 150mm |
| MECH-CABLE-002 | HIGH | Moving joints (swerve steering) SHALL use flexible cables | • Highly flexible cable (rated for 1M+ bend cycles)<br>• Cable carrier or cable chain for continuous rotation<br>• Strain relief at both ends |
| MECH-CABLE-003 | MEDIUM | Cable routing SHALL avoid pinch points and sharp edges | • Cable trays with rounded edges<br>• Grommets at chassis openings<br>• No cables under tension |
| MECH-CABLE-004 | MEDIUM | All cables SHALL be labeled at both ends | • Label tape or heat shrink labels<br>• Format: "MOTOR_FL_POWER" (clear description)<br>• Color coding: red (power), black (ground), others per function |

---

## 4. Non-Functional Requirements

### 4.1 Reliability and Durability

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-REL-001 | HIGH | Mechanical system SHALL operate for 5000 hours without major failure | • Major failure = component replacement required<br>• MTBF (Mean Time Between Failures) ≥ 5000 hours<br>• Tracked during field testing |
| MECH-REL-002 | HIGH | Swerve modules SHALL withstand 100,000 steering cycles | • Lifecycle test: 100k cycles at ±180° rotation<br>• No excessive wear or backlash increase<br>• Bearing and gear inspection after test |
| MECH-REL-003 | MEDIUM | Fasteners SHALL not loosen under vibration | • Vibration test per ISO 16750-3<br>• Use of threadlocker (Loctite) required<br>• Periodic inspection schedule (every 100 hours) |

### 4.2 Maintainability

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-MAINT-001 | HIGH | Common maintenance tasks SHALL be tool-less or require common tools only | • Battery removal: no tools (latch/lock)<br>• Wheel change: single wrench (M8 or M10)<br>• Panel access: captive screws or quarter-turn fasteners |
| MECH-MAINT-002 | MEDIUM | Maintenance documentation SHALL be provided | • Assembly manual with exploded diagrams<br>• Torque specifications for all fasteners<br>• Preventive maintenance schedule |
| MECH-MAINT-003 | MEDIUM | Spare parts list SHALL be provided with vehicle | • Commonly replaced parts: tires, tubes, bearings, belts<br>• Part numbers and suppliers<br>• Recommended stock quantities |

### 4.3 Safety

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-SAFE-001 | CRITICAL | Vehicle SHALL have no exposed sharp edges or pinch points | • All edges rounded (radius ≥3mm) or covered<br>• Pinch point guards on moving parts<br>• Inspection per safety checklist |
| MECH-SAFE-002 | HIGH | Vehicle SHALL be stable on 5° slopes when stationary | • Tilt table test: no tipping at 5° (front, rear, side)<br>• With maximum payload (150kg) at highest CoG<br>• Safety factor: stable up to 10° |
| MECH-SAFE-003 | HIGH | Docking mechanism SHALL not pinch wheelchair frame | • Rubber or foam padding on clamp surfaces<br>• Maximum clamping force limited by actuator selection<br>• No damage to wheelchair paint or frame |

---

## 5. Interface Requirements

### 5.1 Software Interface

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-INT-SW-001 | HIGH | Mechanical sensors SHALL publish to ROS 2 topics per HARDWARE_SOFTWARE_INTERFACES.md | • Bumper sensors: /safety/bumper<br>• E-stop: /safety/estop<br>• Passenger detection: /safety/passenger_detected<br>• Docking sensors: /docking/wheelchair_detected |
| MECH-INT-SW-002 | HIGH | Actuators SHALL subscribe to ROS 2 commands | • Docking actuator: /docking/command<br>• Response time: <500ms<br>• Status feedback published |

### 5.2 Electrical Interface

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-INT-ELEC-001 | HIGH | Swerve modules SHALL use standardized connectors | • Drive motor: XT60 or Anderson Powerpole<br>• Steering motor: JST-PH or similar<br>• Encoder: JST-XH or similar<br>• Documented pinout |
| MECH-INT-ELEC-002 | MEDIUM | Connectors SHALL be keyed to prevent mis-mating | • Different connector types for power vs signal<br>• Polarized connectors (cannot reverse)<br>• Color-coded (red=power, black=ground, etc.) |

---

## 6. Manufacturing and Assembly

### 6.1 Manufacturing Specifications

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-MFG-001 | HIGH | CAD models SHALL be provided in STEP format for manufacturing | • Complete assembly model<br>• Individual part files<br>• Bill of Materials (BOM) generated |
| MECH-MFG-002 | HIGH | Manufacturing drawings SHALL include tolerances | • General tolerance: ±0.5mm<br>• Critical fits: ±0.1mm (bearing bores, shaft diameters)<br>• Surface finish: Ra 3.2 µm (general), Ra 1.6 µm (bearing surfaces) |
| MECH-MFG-003 | MEDIUM | Parts SHALL be manufacturable with standard machine shop equipment | • CNC milling, lathe, drilling, tapping<br>• No exotic processes (EDM, 5-axis milling) unless justified<br>• Welding: TIG or MIG (standard) |

### 6.2 Assembly Procedures

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| MECH-ASSY-001 | HIGH | Assembly instructions SHALL be provided step-by-step | • Illustrated assembly manual<br>• Tools required for each step<br>• Estimated time per step |
| MECH-ASSY-002 | MEDIUM | Sub-assemblies SHALL be tested before final integration | • Swerve module function test (steering, drive)<br>• Docking mechanism test (engage/disengage)<br>• Sensor mounting verification |
| MECH-ASSY-003 | MEDIUM | Final vehicle assembly SHALL include alignment checks | • Wheel alignment (toe, camber)<br>• Sensor alignment (LiDAR level, camera stereo baseline)<br>• CoG measurement |

---

## 7. Testing and Validation

### 7.1 Structural Testing

| Test Type | Acceptance Criteria |
|-----------|---------------------|
| Static Load Test | 225kg (1.5× safety factor) for 1 hour, no deformation |
| Dynamic Load Test | Vibration (10 Hz, 2g) for 1 hour, no loosening |
| Impact Test | 50mm drop with 150kg payload, no damage |

### 7.2 Functional Testing

| Test Type | Acceptance Criteria |
|-----------|---------------------|
| Swerve Module Test | 360° steering range, ±0.5° accuracy, <2s for 180° |
| Docking Test | Engage/disengage 100 times, success rate >95% |
| Weatherproofing Test | IP54 spray test (1 hour), no water ingress |

### 7.3 Safety Testing

| Test Type | Acceptance Criteria |
|-----------|---------------------|
| Stability Test | 5° slope, no tipping (static) |
| Bumper Test | Detect 5N contact force, response time <100ms |
| E-stop Test | Motor cutoff within 50ms of button press |

---

## 8. Traceability Matrix

| Requirement Category | Requirement Count | Priority Distribution |
|---------------------|-------------------|----------------------|
| Chassis and Frame | 10 | CRITICAL: 3, HIGH: 4, MEDIUM: 2, LOW: 1 |
| Dimensional Requirements | 4 | HIGH: 4 |
| Swerve Drive Assembly | 17 | CRITICAL: 2, HIGH: 7, MEDIUM: 6, LOW: 2 |
| Wheelchair Docking | 11 | CRITICAL: 3, HIGH: 6, MEDIUM: 2 |
| Sensor Mounting | 11 | CRITICAL: 2, HIGH: 6, MEDIUM: 3 |
| Power System Mounting | 7 | CRITICAL: 2, HIGH: 3, MEDIUM: 2 |
| Weatherproofing | 9 | CRITICAL: 1, HIGH: 5, MEDIUM: 3 |
| Safety Hardware | 7 | CRITICAL: 3, HIGH: 3, MEDIUM: 1 |
| Cable Management | 4 | HIGH: 2, MEDIUM: 2 |
| Non-Functional (Reliability) | 3 | HIGH: 2, MEDIUM: 1 |
| Non-Functional (Maintainability) | 3 | HIGH: 1, MEDIUM: 2 |
| Non-Functional (Safety) | 3 | CRITICAL: 1, HIGH: 2 |
| Interface Requirements | 4 | HIGH: 3, MEDIUM: 1 |
| Manufacturing | 3 | HIGH: 2, MEDIUM: 1 |
| Assembly | 3 | HIGH: 1, MEDIUM: 2 |
| **TOTAL** | **99** | **CRITICAL: 17, HIGH: 49, MEDIUM: 30, LOW: 3** |

---

## 9. Bill of Materials (Preliminary)

| Category | Component | Quantity | Specification | Notes |
|----------|-----------|----------|---------------|-------|
| Chassis | Aluminum frame | 1 | 6061-T6, welded/bolted | Custom fabrication |
| Swerve Drive | Hoverboard motor | 4 | 6.5", 350W, 36V | Off-the-shelf |
| Swerve Drive | Steering motor | 4 | NEMA 23 stepper or brushless | TBD |
| Swerve Drive | Absolute encoder | 4 | AS5048A (14-bit magnetic) | Off-the-shelf |
| Swerve Drive | Timing belt/gearbox | 4 | GT2 belt or planetary gearbox | TBD |
| Sensors | 3D LiDAR | 1 | IP65, outdoor-rated | Model TBD |
| Sensors | Cameras | 2 | USB/CSI, global shutter preferred | Model TBD |
| Sensors | IMU | 1 | 6-DOF or 9-DOF | Model TBD |
| Power | Battery pack | 1 | 48V 20Ah LiFePO4 | Custom or off-the-shelf |
| Power | BMS | 1 | 48V, 40A, CAN interface | Off-the-shelf |
| Safety | E-stop button | 1 | Red mushroom, twist-release | Off-the-shelf |
| Safety | Bumper sensors | 8 | Pressure switch or limit switch | Off-the-shelf |
| Docking | Linear actuator | 1 | 12V/24V, 200mm stroke, 200N | Off-the-shelf |
| Misc | Fasteners | Various | Stainless steel A2/A4 | Standard sizes |
| Misc | Cable glands | ~20 | PG9, PG13 | Off-the-shelf |

---

## 10. Assumptions and Dependencies

### 10.1 Assumptions

1. Standard manual wheelchairs (600-700mm width) are used
2. Facility pathways are paved (asphalt, concrete)
3. Maximum slope encountered is 5°
4. Ambient temperature -10°C to +40°C

### 10.2 Dependencies

1. ELECTRICAL_REQUIREMENTS.md (motor controllers, wiring)
2. EXTERIOR_REQUIREMENTS.md (eHMI hardware, panels)
3. HARDWARE_SOFTWARE_INTERFACES.md (ROS 2 topics for sensors)
4. Supplier availability for hoverboard motors and components

---

## 11. Open Issues and Risks

### 11.1 Open Issues

| ID | Issue | Owner | Target Resolution |
|----|-------|-------|-------------------|
| MECH-ISSUE-001 | Final wheelbase/track width optimization | Tsuchiya | Week 4 (CAD iteration) |
| MECH-ISSUE-002 | Steering motor selection (stepper vs brushless) | Tsuchiya | Week 3 |
| MECH-ISSUE-003 | LiDAR model selection and procurement | Tsuchiya | Week 3 |

### 11.2 Risks

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| Hoverboard motor unavailability | LOW | HIGH | Identify 2-3 alternative suppliers, consider custom hub motor |
| Swerve module complexity delays assembly | MEDIUM | MEDIUM | Build and test one prototype module early (Week 5) |
| Weatherproofing IP54 difficult to achieve | MEDIUM | MEDIUM | Design review with experienced engineer, use proven gasket designs |
| CoG too high, stability issues | LOW | HIGH | FEA and CAD analysis before fabrication, battery placement critical |

---

## 12. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-16 | Tsuchiya | Initial draft - Week 2 documentation |

---

**END OF DOCUMENT**

**Total Requirements:** 99
**Document Status:** Draft for Review
**Next Review:** Week 3 (with Kiril for integration with exterior components)
