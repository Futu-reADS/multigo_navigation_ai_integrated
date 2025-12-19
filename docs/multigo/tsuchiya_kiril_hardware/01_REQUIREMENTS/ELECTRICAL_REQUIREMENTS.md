# Electrical System Requirements

**Document Control:**
- **Document ID:** ELEC-REQ-001
- **Version:** 1.0
- **Date:** 2025-12-16
- **Status:** Draft
- **Owner:** Tsuchiya (Hardware Team - Electrical Lead)
- **Reviewers:** Pankaj (Software Integration), Safety Engineer

---

## 1. Introduction

### 1.1 Purpose

This document specifies the electrical system requirements for the outdoor wheelchair transport robot, including power distribution, motor control, battery management, sensor power, communication buses, safety circuits, and electromagnetic compatibility (EMC). The electrical system must safely power all robot subsystems and enable reliable operation in outdoor environments.

### 1.2 Scope

**In Scope:**
- Power distribution architecture (48V main bus, 24V/12V/5V regulators)
- Battery system and Battery Management System (BMS)
- Motor controllers (swerve drive and steering motors)
- Sensor power and signal conditioning
- Communication buses (CAN bus for motors/BMS, UART for eHMI)
- Safety circuits (E-stop, bumper sensors, current limiting)
- Wiring harness design and specifications
- Electromagnetic compatibility (EMC) and noise reduction
- Grounding and shielding

**Out of Scope:**
- Mechanical mounting (covered in MECHANICAL_REQUIREMENTS.md)
- Software/ROS 2 integration (covered in HARDWARE_SOFTWARE_INTERFACES.md)
- eHMI circuit design (covered in EXTERIOR_REQUIREMENTS.md - Kiril's scope)

### 1.3 Definitions and Acronyms

| Term | Definition |
|------|------------|
| BMS | Battery Management System |
| CAN | Controller Area Network (bus protocol) |
| UART | Universal Asynchronous Receiver-Transmitter |
| DC-DC Converter | Voltage step-down/step-up converter |
| E-stop | Emergency Stop (safety circuit) |
| PWM | Pulse Width Modulation |
| EMC | Electromagnetic Compatibility |
| LiPo / LiFePO4 | Lithium Polymer / Lithium Iron Phosphate battery |

### 1.4 References

- MECHANICAL_REQUIREMENTS.md - Mechanical mounting and assembly
- HARDWARE_SOFTWARE_INTERFACES.md - ROS 2 topics and sensor interfaces
- TVM_API_SPECIFICATION.md - Battery status reporting
- EXTERIOR_REQUIREMENTS.md - eHMI power requirements

---

## 2. System Overview

### 2.1 Electrical Architecture

```
┌──────────────────────────────────────────────────┐
│            48V Battery Pack (20Ah)                │
│              with BMS (CAN bus)                   │
└───────┬──────────────────────────────────────────┘
        │
        ├─→ E-STOP Circuit (Hardwired)
        │
        ├─→ 48V Bus ─→ Swerve Drive Motors (4×)
        │           └→ Motor Controllers (4×, CAN bus)
        │
        ├─→ DC-DC 48V→24V (10A) ─→ Steering Motors (4×)
        │                        └→ Motor Controllers (4×, CAN bus)
        │
        ├─→ DC-DC 48V→12V (15A) ─→ Compute Unit (AMD Ryzen)
        │                        └→ LiDAR, Cameras
        │
        ├─→ DC-DC 48V→5V (10A)  ─→ IMU, Encoders, Sensors
        │                        └→ eHMI (ESP32, LEDs, audio)
        │
        └─→ BMS CAN Bus ─────────→ Compute Unit (battery monitoring)
```

### 2.2 Power Budget

| Subsystem | Voltage | Current (Typical) | Current (Peak) | Power (W) |
|-----------|---------|-------------------|----------------|-----------|
| Swerve Drive Motors (4×) | 48V | 4A (1A each) | 40A (10A each) | 192W / 1920W peak |
| Steering Motors (4×) | 24V | 2A (0.5A each) | 8A (2A each) | 48W / 192W peak |
| Compute Unit | 12V | 8A | 12A | 96W / 144W |
| LiDAR | 12V | 2A | 2.5A | 24W / 30W |
| Cameras (2×) | 12V | 1A (0.5A each) | 1.5A | 12W / 18W |
| IMU + Sensors | 5V | 0.5A | 1A | 2.5W / 5W |
| eHMI (ESP32, LEDs, Audio) | 5V / 12V | 2A | 4A | 10W / 48W |
| BMS + Misc | 5V | 0.5A | 1A | 2.5W / 5W |
| **TOTAL** | - | **20A @ 48V** | **69A @ 48V** | **387W / 2,362W peak** |

**Battery Capacity:** 48V × 20Ah = 960Wh
**Runtime (Typical):** 960Wh / 387W = **2.5 hours**
**Runtime (Peak continuous):** 960Wh / 2,362W = **24 minutes** (unlikely scenario)

---

## 3. Functional Requirements

### 3.1 Power Distribution

#### 3.1.1 Main Power Bus (48V)

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| ELEC-PWR-MAIN-001 | CRITICAL | System SHALL use 48V nominal as main power bus | • Voltage range: 42V (empty) to 54.6V (full charge)<br>• All high-power loads (motors) connected directly<br>• Bus voltage monitored by BMS |
| ELEC-PWR-MAIN-002 | CRITICAL | Main power bus SHALL include inline fuse for overcurrent protection | • Fuse rating: 60A (1.5× peak current)<br>• Fast-blow automotive fuse<br>• Accessible for replacement |
| ELEC-PWR-MAIN-003 | HIGH | Main power bus SHALL use appropriate wire gauge | • Wire gauge: 10 AWG (5.26 mm²) minimum for 48V bus<br>• Rated for 60A continuous<br>• Flexible silicone wire (high strand count) |

#### 3.1.2 DC-DC Converters

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| ELEC-PWR-DCDC-001 | CRITICAL | System SHALL include 48V→24V DC-DC converter for steering motors | • Output: 24V ± 5%, 10A continuous, 15A peak<br>• Efficiency ≥85%<br>• Overload protection |
| ELEC-PWR-DCDC-002 | CRITICAL | System SHALL include 48V→12V DC-DC converter for compute and sensors | • Output: 12V ± 5%, 15A continuous, 20A peak<br>• Efficiency ≥85%<br>• Ripple <100mV peak-to-peak |
| ELEC-PWR-DCDC-003 | HIGH | System SHALL include 48V→5V DC-DC converter for low-power electronics | • Output: 5V ± 5%, 10A continuous, 12A peak<br>• Efficiency ≥80%<br>• USB-compatible output |
| ELEC-PWR-DCDC-004 | HIGH | DC-DC converters SHALL have overvoltage and undervoltage protection | • Overvoltage: shutdown at >60V input<br>• Undervoltage: shutdown at <36V input (battery critical low)<br>• Auto-restart when voltage normal |

---

### 3.2 Battery System and BMS

#### 3.2.1 Battery Pack Specifications

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| ELEC-BAT-PACK-001 | CRITICAL | Battery pack SHALL be 48V nominal, 20Ah capacity minimum | • Chemistry: LiFePO4 (safer, longer life) or LiPo<br>• Configuration: 13S (13 cells in series for 48V nominal)<br>• Energy: ≥960Wh |
| ELEC-BAT-PACK-002 | HIGH | Battery pack SHALL support 60A continuous discharge | • C-rate: 3C for 20Ah battery<br>• Burst: 100A for 10 seconds (motor acceleration)<br>• Cell balancing required |
| ELEC-BAT-PACK-003 | HIGH | Battery pack SHALL include built-in BMS | • BMS monitors: voltage, current, temperature per cell<br>• Balancing current ≥200mA per cell<br>• CAN bus interface for monitoring |
| ELEC-BAT-PACK-004 | MEDIUM | Battery pack SHALL be removable for charging | • XT90 or Anderson Powerpole connector<br>• Keyed connector (prevents reverse polarity)<br>• Max 30kg weight for manual handling |

#### 3.2.2 Battery Management System (BMS)

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| ELEC-BMS-001 | CRITICAL | BMS SHALL monitor per-cell voltage and temperature | • Voltage accuracy: ±10mV<br>• Temperature accuracy: ±2°C<br>• Sampling rate: ≥1 Hz |
| ELEC-BMS-002 | CRITICAL | BMS SHALL disconnect battery on overcurrent or overvoltage | • Overcurrent threshold: 100A (settable)<br>• Overvoltage: 4.2V per cell (54.6V pack)<br>• Undervoltage: 2.5V per cell (32.5V pack) |
| ELEC-BMS-003 | HIGH | BMS SHALL provide CAN bus interface for monitoring | • CAN bus: 250 kbit/s<br>• Messages: SoC, voltage, current, temperature, error codes<br>• Update rate: 1 Hz |
| ELEC-BMS-004 | HIGH | BMS SHALL balance cells during charging | • Active or passive balancing<br>• Target: <50mV cell voltage difference<br>• Balancing current ≥200mA |
| ELEC-BMS-005 | MEDIUM | BMS SHALL estimate State of Charge (SoC) accurately | • SoC accuracy: ±5%<br>• Coulomb counting + voltage-based estimation<br>• Published to ROS 2 via CAN→compute |

---

### 3.3 Motor Controllers

#### 3.3.1 Swerve Drive Motor Controllers (4×)

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| ELEC-MOTOR-DRIVE-001 | CRITICAL | Each swerve drive motor SHALL have dedicated brushless motor controller | • Input: 48V nominal (36-54V range)<br>• Output: 10A continuous, 20A peak<br>• Supports FOC (Field-Oriented Control) or sensorless control |
| ELEC-MOTOR-DRIVE-002 | HIGH | Drive motor controllers SHALL support hall sensor feedback | • 3-phase brushless motor with hall sensors<br>• Speed range: 0-1000 RPM<br>• Direction reversible |
| ELEC-MOTOR-DRIVE-003 | HIGH | Drive motor controllers SHALL communicate via CAN bus | • CAN bus: 500 kbit/s (motor network)<br>• CANopen CiA 402 profile (drive control)<br>• Commands: velocity, current limit, enable/disable |
| ELEC-MOTOR-DRIVE-004 | HIGH | Drive motor controllers SHALL include overcurrent protection | • Current limit: 20A (configurable)<br>• Thermal protection (shutdown at >80°C)<br>• Auto-restart when cool |

#### 3.3.2 Steering Motor Controllers (4×)

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| ELEC-MOTOR-STEER-001 | CRITICAL | Each steering motor SHALL have dedicated motor controller | • Input: 24V (from DC-DC converter)<br>• Output: 2A continuous, 5A peak<br>• Supports stepper motor or brushless servo |
| ELEC-MOTOR-STEER-002 | HIGH | Steering motor controllers SHALL provide position control | • Position accuracy: ±0.5°<br>• Response time: <200ms for 90° rotation<br>• Encoder feedback (absolute encoder on steering axis) |
| ELEC-MOTOR-STEER-003 | HIGH | Steering motor controllers SHALL communicate via CAN bus | • Same CAN bus as drive motors (500 kbit/s)<br>• CANopen CiA 402 profile (position control)<br>• Commands: target angle, speed, torque limit |

---

### 3.4 Sensor Power and Signal Conditioning

#### 3.4.1 LiDAR Power

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| ELEC-SENSOR-LIDAR-001 | HIGH | LiDAR SHALL be powered from 12V bus | • Power: 12V ± 5%, 2.5A max<br>• Dedicated power line (not shared with noisy loads)<br>• Inline fuse: 3A |
| ELEC-SENSOR-LIDAR-002 | MEDIUM | LiDAR power SHALL include soft-start circuit | • Inrush current limited to <5A<br>• Prevents voltage sag on power-on<br>• Capacitor bank for ripple filtering |

#### 3.4.2 Camera Power

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| ELEC-SENSOR-CAM-001 | HIGH | Cameras SHALL be powered from 12V bus or 5V (USB) | • Option 1: 12V barrel jack (dedicated cameras)<br>• Option 2: 5V USB (USB cameras)<br>• Power budget: 1.5A total for both cameras |
| ELEC-SENSOR-CAM-002 | MEDIUM | Camera power SHALL be filtered to reduce noise | • LC filter on power line<br>• Ferrite beads on USB data lines<br>• Prevents EMI from affecting image quality |

#### 3.4.3 IMU and Encoder Power

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| ELEC-SENSOR-IMU-001 | HIGH | IMU SHALL be powered from 5V bus | • Power: 5V ± 5%, <500mA<br>• Clean power (low noise for accurate measurements)<br>• Dedicated regulator if needed |
| ELEC-SENSOR-ENC-001 | HIGH | Absolute encoders (4×) SHALL be powered from 5V bus | • Power: 5V, 100mA per encoder<br>• Total: 400mA for 4 encoders<br>• Shared 5V bus acceptable |

---

### 3.5 Communication Buses

#### 3.5.1 CAN Bus for Motors and BMS

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| ELEC-CAN-MOTOR-001 | CRITICAL | Motor CAN bus SHALL operate at 500 kbit/s | • Standard CAN 2.0B protocol<br>• Twisted pair cable (CAN_H, CAN_L)<br>• 120Ω termination resistors at both ends |
| ELEC-CAN-MOTOR-002 | HIGH | Motor CAN bus SHALL connect all motor controllers and compute unit | • Nodes: 4× drive controllers, 4× steering controllers, compute unit<br>• Total: 9 nodes<br>• Daisy-chain topology |
| ELEC-CAN-MOTOR-003 | HIGH | Motor CAN bus SHALL use CANopen protocol | • CANopen CiA 402 device profile (drive control)<br>• Node IDs: Drive FL=1, FR=2, RL=3, RR=4; Steer FL=5, FR=6, RL=7, RR=8<br>• Heartbeat messages every 100ms |
| ELEC-CAN-BMS-001 | HIGH | BMS CAN bus SHALL operate at 250 kbit/s (separate bus) | • Dedicated CAN bus for BMS<br>• Nodes: BMS, compute unit<br>• 120Ω termination |

#### 3.5.2 UART for eHMI

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| ELEC-UART-EHMI-001 | HIGH | eHMI UART SHALL operate at 115200 baud | • 8 data bits, no parity, 1 stop bit (8N1)<br>• Connects compute unit UART to ESP32 (eHMI controller)<br>• 3.3V logic level (level shifter if compute is 5V) |
| ELEC-UART-EHMI-002 | MEDIUM | eHMI UART SHALL use shielded cable | • Length <3 meters<br>• Shielded twisted pair<br>• Reduces EMI pickup |

---

### 3.6 Safety Circuits

#### 3.6.1 Emergency Stop (E-stop)

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| ELEC-SAFETY-ESTOP-001 | CRITICAL | E-stop circuit SHALL be hardwired (not software-controlled) | • E-stop button → contactor/relay → motor power<br>• Cuts power to all motor controllers immediately<br>• Latching circuit (requires manual reset) |
| ELEC-SAFETY-ESTOP-002 | CRITICAL | E-stop SHALL cut power to drive motors within 50ms | • Relay response time <20ms<br>• Motor coasting time <30ms<br>• Total stop initiation <50ms |
| ELEC-SAFETY-ESTOP-003 | HIGH | E-stop circuit SHALL be fail-safe (normally open) | • Button release = E-stop activated<br>• Button pressed = normal operation<br>• Prevents accidental restart |
| ELEC-SAFETY-ESTOP-004 | HIGH | E-stop state SHALL be published to ROS 2 | • GPIO input to compute unit<br>• Topic: /safety/estop (std_msgs/Bool)<br>• Debounced (10ms) |

#### 3.6.2 Bumper Sensors

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| ELEC-SAFETY-BUMP-001 | HIGH | Bumper sensors SHALL use normally closed (NC) switches | • Multiple bumper switches in series<br>• Any bumper contact opens circuit<br>• Fail-safe: wire break = bumper triggered |
| ELEC-SAFETY-BUMP-002 | HIGH | Bumper circuit SHALL connect to compute GPIO | • GPIO input (pulled up internally)<br>• Ground on contact<br>• Debounced in software (20ms) |
| ELEC-SAFETY-BUMP-003 | MEDIUM | Bumper sensors SHALL trigger software E-stop | • ROS 2 node monitors /safety/bumper<br>• Sends zero velocity command to motors<br>• Not hardwired (allows resumption) |

---

### 3.7 Wiring Harness Design

#### 3.7.1 Wire Specifications

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| ELEC-WIRE-SPEC-001 | HIGH | Power wires SHALL use appropriate gauge for current | • 48V bus: 10 AWG (60A)<br>• 24V: 14 AWG (15A)<br>• 12V: 16 AWG (10A)<br>• 5V: 18 AWG (5A)<br>• Signal: 22-24 AWG |
| ELEC-WIRE-SPEC-002 | HIGH | All power wires SHALL be stranded (flexible) | • High strand count (≥100 strands for 10 AWG)<br>• Silicone insulation (flexible, temperature resistant)<br>• Color-coded: Red (positive), Black (ground), Yellow/White (signal) |
| ELEC-WIRE-SPEC-003 | MEDIUM | Wire insulation SHALL be rated for temperature and abrasion | • Temperature rating: -20°C to +80°C<br>• Abrasion resistant (outdoor use)<br>• UV resistant (outdoor exposure) |

#### 3.7.2 Connectors

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| ELEC-CONN-001 | HIGH | High-current connectors SHALL use XT60, XT90, or Anderson Powerpole | • Battery: XT90 (90A rating)<br>• Motors: XT60 (60A rating)<br>• Keyed to prevent reverse polarity |
| ELEC-CONN-002 | HIGH | Signal connectors SHALL use JST, Molex, or similar | • CAN bus: 4-pin Molex (CAN_H, CAN_L, GND, optional +V)<br>• UART: 4-pin JST (TX, RX, GND, +3.3V/5V)<br>• Sensors: JST-XH or JST-PH |
| ELEC-CONN-003 | MEDIUM | All connectors SHALL be rated for ≥100 mating cycles | • Durable connectors (gold-plated contacts)<br>• Strain relief on cables<br>• IP54+ rated for outdoor use (where applicable) |

---

### 3.8 Electromagnetic Compatibility (EMC)

#### 3.8.1 Noise Reduction

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| ELEC-EMC-NOISE-001 | HIGH | Motor controller outputs SHALL use shielded cables | • 3-phase motor wires: shielded<br>• Shield grounded at one end (motor controller)<br>• Reduces radiated EMI |
| ELEC-EMC-NOISE-002 | MEDIUM | DC-DC converters SHALL include input/output filtering | • Input: Ceramic capacitors (100nF) + electrolytic (100µF)<br>• Output: LC filter (ferrite bead + capacitor)<br>• Reduces conducted EMI |
| ELEC-EMC-NOISE-003 | MEDIUM | CAN bus and UART SHALL use twisted pair cables | • Twists: ≥20 twists per meter<br>• Reduces differential-mode noise<br>• Shielded twisted pair preferred |

#### 3.8.2 Grounding and Shielding

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| ELEC-GND-001 | HIGH | System SHALL have single-point ground (star ground) | • All subsystems ground to central ground point<br>• Chassis ground connected to battery negative<br>• Prevents ground loops |
| ELEC-GND-002 | MEDIUM | Cable shields SHALL be grounded at one end only | • Motor shields: grounded at controller<br>• Sensor shields: grounded at compute unit<br>• Prevents circulating currents in shield |

---

## 4. Non-Functional Requirements

### 4.1 Reliability and Fault Tolerance

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| ELEC-REL-001 | HIGH | Electrical system SHALL operate for 5000 hours MTBF | • Mean Time Between Failures ≥5000 hours<br>• Tested with accelerated life testing<br>• Component derating (operate at <80% max ratings) |
| ELEC-REL-002 | HIGH | Critical components SHALL have overcurrent/overvoltage protection | • Fuses on all power buses<br>• TVS diodes on signal lines<br>• Automatic shutdown and recovery |

### 4.2 Environmental

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| ELEC-ENV-001 | HIGH | Electrical components SHALL operate in -10°C to +40°C ambient | • Derating for temperature extremes<br>• Thermal management (heatsinks, fans if needed)<br>• Cold-start testing at -10°C |
| ELEC-ENV-002 | MEDIUM | Electrical enclosures SHALL achieve IP54 rating minimum | • Dust protection (limited ingress)<br>• Splash water protection<br>• Gaskets on enclosure lids |

### 4.3 Maintainability

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| ELEC-MAINT-001 | MEDIUM | Fuses and relays SHALL be accessible without disassembly | • Labeled fuse panel<br>• Tool-free access (flip-lid enclosure)<br>• Spare fuses included |
| ELEC-MAINT-002 | MEDIUM | Wiring SHALL be labeled at both ends | • Heat-shrink labels or cable tags<br>• Format: "MOTOR_FL_POWER" (clear description)<br>• Wiring diagram provided |

---

## 5. Testing Requirements

| Test Type | Acceptance Criteria |
|-----------|---------------------|
| Power-On Test | All voltage rails within spec (48V, 24V, 12V, 5V ±5%) |
| Load Test | System operates at full load (20A @ 48V) for 1 hour, no overheating |
| Motor Test | All 8 motors (4 drive, 4 steer) respond to commands, no excessive current |
| CAN Bus Test | All CAN nodes communicate, no bus errors for 24 hours |
| E-stop Test | E-stop cuts motor power within 50ms, repeatable 100× |
| EMC Test | Radiated emissions <EN 55011 Class B limits |
| Battery Test | Charge/discharge 100 cycles, capacity degradation <10% |

---

## 6. Traceability Matrix

| Requirement Category | Requirement Count | Priority Distribution |
|---------------------|-------------------|----------------------|
| Power Distribution | 7 | CRITICAL: 5, HIGH: 2 |
| Battery and BMS | 9 | CRITICAL: 5, HIGH: 3, MEDIUM: 1 |
| Motor Controllers | 8 | CRITICAL: 3, HIGH: 5 |
| Sensor Power | 6 | HIGH: 4, MEDIUM: 2 |
| Communication Buses | 7 | CRITICAL: 1, HIGH: 5, MEDIUM: 1 |
| Safety Circuits | 7 | CRITICAL: 3, HIGH: 4 |
| Wiring Harness | 6 | HIGH: 3, MEDIUM: 3 |
| EMC and Grounding | 5 | HIGH: 2, MEDIUM: 3 |
| Non-Functional | 6 | HIGH: 4, MEDIUM: 2 |
| **TOTAL** | **61** | **CRITICAL: 17, HIGH: 32, MEDIUM: 12** |

---

## 7. Bill of Materials (Key Electrical Components)

| Component | Quantity | Specification | Estimated Cost (USD) |
|-----------|----------|---------------|---------------------|
| Battery Pack | 1 | 48V 20Ah LiFePO4 with BMS | $400-600 |
| DC-DC 48V→24V | 1 | 10A, 85% efficiency | $40-60 |
| DC-DC 48V→12V | 1 | 15A, 85% efficiency | $50-70 |
| DC-DC 48V→5V | 1 | 10A, 80% efficiency | $30-50 |
| Brushless Motor Controller | 4 | 48V, 20A, CAN bus | $60-80 each |
| Stepper/Servo Controller | 4 | 24V, 5A, CAN bus | $40-60 each |
| CAN Transceiver | 2 | SN65HVD230 or similar | $2-5 each |
| E-stop Button | 1 | Red mushroom, twist-release | $10-20 |
| Contactors/Relays | 2 | 60A, 48V coil | $15-30 each |
| Fuses | 10+ | Various ratings (3A-60A) | $1-5 each |
| Connectors (XT60, XT90, JST, Molex) | Various | - | $50-100 total |
| Wire/Cable | 50m+ | 10-24 AWG, various colors | $100-200 |

**Total Estimated Cost:** $1,200 - $1,800 (electrical components only)

---

## 8. Assumptions and Dependencies

### 8.1 Assumptions

1. AC mains power available for battery charging (48V charger not part of vehicle)
2. Compute unit (AMD Ryzen) operates on 12V input (DC power supply)
3. LiDAR and cameras support 12V or 5V power input
4. Facility has designated charging area with ventilation

### 8.2 Dependencies

1. MECHANICAL_REQUIREMENTS.md (component mounting)
2. HARDWARE_SOFTWARE_INTERFACES.md (ROS 2 topics for sensors/motors)
3. EXTERIOR_REQUIREMENTS.md (eHMI power requirements)
4. Supplier availability for battery packs and motor controllers

---

## 9. Open Issues and Risks

### 9.1 Open Issues

| ID | Issue | Owner | Target Resolution |
|----|-------|-------|-------------------|
| ELEC-ISSUE-001 | Battery pack supplier selection (LiFePO4 vs LiPo) | Tsuchiya | Week 4 |
| ELEC-ISSUE-002 | Motor controller CAN bus compatibility verification | Tsuchiya | Week 4 |
| ELEC-ISSUE-003 | Charger specification (on-board vs off-board) | Tsuchiya | Week 5 |

### 9.2 Risks

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| Battery thermal runaway (fire risk) | LOW | CRITICAL | Use LiFePO4 (safer chemistry), thermal monitoring, proper ventilation |
| CAN bus errors due to EMI | MEDIUM | MEDIUM | Shielded cables, twisted pair, proper termination, testing |
| DC-DC converter failure | LOW | HIGH | Use quality components (Mean Well, TDK-Lambda), overcurrent protection |
| E-stop relay welding (stuck closed) | LOW | CRITICAL | Use automotive-grade relay, redundant safety circuit, periodic testing |

---

## 10. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-16 | Tsuchiya | Initial draft - Week 3 documentation |

---

**END OF DOCUMENT**

**Total Requirements:** 61
**Document Status:** Draft for Review
**Next Review:** Week 4 (with Kiril for eHMI power integration, with Pankaj for ROS 2 sensor interfaces)
