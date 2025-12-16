# Hardware Requirements

**Document ID:** REQ-HW-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft
**Classification:** Internal

---

## Document Control

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-15 | Hardware Lead | Initial hardware requirements consolidating all subsystem hardware specs |

---

## 1. Introduction

### 1.1 Purpose

This document consolidates **all hardware requirements** for the outdoor-first wheelchair transport robot, including compute platform, sensors, actuators, power system, mechanical structure, and environmental protection.

### 1.2 Scope

This specification covers:
- **Compute Platform** - GMKtec Nucbox K6 specifications
- **Sensors** - 3D LiDAR, cameras, IMU, encoders, safety sensors
- **Actuators** - Swerve drive motors, brakes, docking mechanism
- **Power System** - Battery, BMS, distribution, charging
- **Mechanical Structure** - Chassis, wheelchair attachment, weatherproofing
- **Environmental Protection** - IP ratings, temperature, weatherproofing

### 1.3 Related Documents

- **SYSTEM_REQUIREMENTS.md** - Section 1.5 (HW-REQ-001 to 010)
- **OVERALL_SYSTEM_ARCHITECTURE.md** - Section 5 (Hardware Architecture)
- **question_answers.md** - GMKtec Nucbox K6 specifications, no GPS

---

## 2. Compute Platform Requirements

### 2.1 Main Computer (HW-COMP-MAIN)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-COMP-MAIN-001 | System SHALL use GMKtec Nucbox K6 as main compute platform | Critical | Hardware BOM |
| HW-COMP-MAIN-002 | Main computer SHALL have AMD Ryzen 7 7840HS CPU (8 cores, 16 threads, 3.8-5.1 GHz) | Critical | Datasheet |
| HW-COMP-MAIN-003 | Main computer SHALL have 32GB DDR5 5600MHz RAM | Critical | Datasheet |
| HW-COMP-MAIN-004 | Main computer SHALL have AMD Radeon 780M integrated GPU | Critical | Datasheet |
| HW-COMP-MAIN-005 | Main computer SHALL have 1TB PCIe 4.0 SSD storage | Critical | Datasheet |
| HW-COMP-MAIN-006 | Main computer SHALL have ≥4× USB 3.0 ports | High | Datasheet |
| HW-COMP-MAIN-007 | Main computer SHALL have ≥1× Gigabit Ethernet port | Critical | Datasheet |
| HW-COMP-MAIN-008 | Main computer SHALL support Ubuntu 22.04 LTS | Critical | Compatibility test |
| HW-COMP-MAIN-009 | Main computer SHALL operate in -10°C to +45°C (with cooling) | High | Thermal test |
| HW-COMP-MAIN-010 | Main computer SHALL be housed in IP54+ enclosure | Critical | Environmental test |

**Total Requirements: 10**

---

### 2.2 Power & Thermal Management (HW-COMP-POWER)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-COMP-POWER-001 | Main computer SHALL consume <65W peak power | High | Power measurement |
| HW-COMP-POWER-002 | Main computer SHALL have active cooling (fan) | Critical | Hardware inspection |
| HW-COMP-POWER-003 | System SHALL monitor CPU temperature and throttle if >85°C | Critical | Thermal test |
| HW-COMP-POWER-004 | System SHALL have UPS backup for graceful shutdown (≥60s) | High | Power loss test |

**Total Requirements: 4**

---

## 3. Sensor Requirements

### 3.1 3D LiDAR (HW-SENS-LIDAR)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-SENS-LIDAR-001 | System SHALL have outdoor-grade 3D LiDAR (IP65+ rating) | Critical | Datasheet |
| HW-SENS-LIDAR-002 | LiDAR SHALL provide 360° horizontal × ≥30° vertical FOV | Critical | Datasheet |
| HW-SENS-LIDAR-003 | LiDAR SHALL have 50-100m range outdoor | Critical | Field test |
| HW-SENS-LIDAR-004 | LiDAR SHALL operate at 10 Hz minimum | Critical | Datasheet |
| HW-SENS-LIDAR-005 | LiDAR SHALL provide >100,000 points/second | High | Datasheet |
| HW-SENS-LIDAR-006 | LiDAR SHALL have <±3cm range accuracy | High | Calibration test |
| HW-SENS-LIDAR-007 | LiDAR SHALL have ROS 2 Humble driver available | Critical | Driver test |
| HW-SENS-LIDAR-008 | LiDAR SHALL be mounted at 0.6-0.8m height | High | Mechanical design |
| HW-SENS-LIDAR-009 | LiDAR SHALL consume <20W power | Medium | Power measurement |

**Total Requirements: 9**

---

### 3.2 RGB Cameras (HW-SENS-CAM)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-SENS-CAM-001 | System SHALL have 2× RGB cameras (front-left, front-right) | Critical | Hardware BOM |
| HW-SENS-CAM-002 | Cameras SHALL provide 1920×1080 resolution minimum | Critical | Datasheet |
| HW-SENS-CAM-003 | Cameras SHALL operate at 30 FPS minimum | Critical | Datasheet |
| HW-SENS-CAM-004 | Cameras SHALL have ≥90° horizontal FOV | High | Datasheet |
| HW-SENS-CAM-005 | Cameras SHALL support USB 3.0 or GigE interface | High | Datasheet |
| HW-SENS-CAM-006 | Cameras SHALL have global shutter or <10ms rolling shutter | Medium | Datasheet |
| HW-SENS-CAM-007 | Cameras SHALL be mounted at 0.4-0.6m height | High | Mechanical design |
| HW-SENS-CAM-008 | Cameras SHALL have IP66+ weatherproofing or housing | Critical | Datasheet |
| HW-SENS-CAM-009 | Cameras SHALL consume <5W each | Medium | Power measurement |

**Total Requirements: 9**

---

### 3.3 IMU (HW-SENS-IMU)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-SENS-IMU-001 | System SHALL have 9-DOF IMU (accel + gyro + mag) | Critical | Hardware BOM |
| HW-SENS-IMU-002 | IMU SHALL operate at 50 Hz minimum | Critical | Datasheet |
| HW-SENS-IMU-003 | IMU SHALL have <0.5° orientation accuracy | High | Calibration test |
| HW-SENS-IMU-004 | IMU SHALL have ROS 2 driver available | Critical | Driver test |
| HW-SENS-IMU-005 | System SHALL have redundant IMU for safety | High | Hardware BOM |
| HW-SENS-IMU-006 | IMU SHALL consume <1W power | Low | Power measurement |

**Total Requirements: 6**

---

### 3.4 Wheel Encoders (HW-SENS-ENC)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-SENS-ENC-001 | Each swerve module SHALL have wheel speed encoder | Critical | Hardware BOM |
| HW-SENS-ENC-002 | Each swerve module SHALL have steering angle encoder | Critical | Hardware BOM |
| HW-SENS-ENC-003 | Encoders SHALL provide ≥1000 CPR resolution | High | Datasheet |
| HW-SENS-ENC-004 | Encoders SHALL update at 100 Hz minimum | High | Performance test |
| HW-SENS-ENC-005 | Encoders SHALL have <0.1° angle accuracy | High | Calibration test |

**Total Requirements: 5**

---

### 3.5 Safety Sensors (HW-SENS-SAFE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-SENS-SAFE-001 | System SHALL have passenger weight sensors (load cells, >20kg threshold) | Critical | Hardware BOM |
| HW-SENS-SAFE-002 | System SHALL have seatbelt engagement sensor (magnetic) | Critical | Hardware BOM |
| HW-SENS-SAFE-003 | System SHALL have rain sensor for weather detection | High | Hardware BOM |
| HW-SENS-SAFE-004 | System SHALL have bumper contact sensors (optional, safety redundancy) | Medium | Hardware BOM |
| HW-SENS-SAFE-005 | All safety sensors SHALL publish data at ≥10 Hz | Critical | Performance test |

**Total Requirements: 5**

---

## 4. Actuator Requirements

### 4.1 Swerve Drive Motors (HW-ACT-SWERVE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-ACT-SWERVE-001 | System SHALL have 4× swerve drive modules | Critical | Hardware BOM |
| HW-ACT-SWERVE-002 | Each module SHALL use 6.5" (φ165mm) hoverboard in-wheel motor | Critical | Datasheet |
| HW-ACT-SWERVE-003 | Each wheel motor SHALL provide 80W nominal power | Critical | Datasheet |
| HW-ACT-SWERVE-004 | Each steering motor SHALL provide ≥5 Nm torque | High | Datasheet |
| HW-ACT-SWERVE-005 | Motors SHALL have integrated motor controllers | High | Datasheet |
| HW-ACT-SWERVE-006 | Motor controllers SHALL support CAN bus or serial interface | Critical | Datasheet |
| HW-ACT-SWERVE-007 | Motors SHALL have thermal protection (shutdown >90°C) | Critical | Thermal test |
| HW-ACT-SWERVE-008 | Motors SHALL have hardware current limiting | Critical | Hardware inspection |

**Total Requirements: 8**

---

### 4.2 Braking System (HW-ACT-BRAKE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-ACT-BRAKE-001 | System SHALL have electromechanical brakes (fail-safe engaged) | Critical | Hardware BOM |
| HW-ACT-BRAKE-002 | Brakes SHALL engage automatically on power loss | Critical | Power loss test |
| HW-ACT-BRAKE-003 | Brakes SHALL engage on emergency stop | Critical | Safety test |
| HW-ACT-BRAKE-004 | Brakes SHALL hold robot on 10° slope with 100kg load | Critical | Brake test |
| HW-ACT-BRAKE-005 | Brake engagement time SHALL be <200ms | Critical | Latency test |

**Total Requirements: 5**

---

### 4.3 LED Lighting (HW-ACT-LIGHT)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-ACT-LIGHT-001 | System SHALL have LED headlights for night operation | High | Hardware BOM |
| HW-ACT-LIGHT-002 | Headlights SHALL provide 500-1000 lux at 2m distance | High | Photometer test |
| HW-ACT-LIGHT-003 | System SHALL have safety beacon (red flashing LED) | Critical | Hardware BOM |
| HW-ACT-LIGHT-004 | Headlights SHALL activate automatically when ambient <500 lux | High | Integration test |

**Total Requirements: 4**

---

## 5. Power System Requirements

### 5.1 Battery (HW-POWER-BATT)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-POWER-BATT-001 | System SHALL have Li-ion or LiFePO4 battery pack | Critical | Hardware BOM |
| HW-POWER-BATT-002 | Battery SHALL provide ≥48V nominal voltage | Critical | Datasheet |
| HW-POWER-BATT-003 | Battery SHALL provide ≥20Ah capacity (≥1kWh energy) | Critical | Datasheet |
| HW-POWER-BATT-004 | Battery SHALL support 4+ hours operation with passenger transport | Critical | Endurance test |
| HW-POWER-BATT-005 | Battery SHALL support ≥500 charge cycles (80% capacity retention) | High | Datasheet |
| HW-POWER-BATT-006 | Battery SHALL have IP54+ weatherproof enclosure | Critical | Environmental test |
| HW-POWER-BATT-007 | Battery SHALL be hot-swappable (optional, preferred) | Low | Mechanical design |

**Total Requirements: 7**

---

### 5.2 Battery Management System (HW-POWER-BMS)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-POWER-BMS-001 | System SHALL have BMS monitoring voltage, current, temperature | Critical | Hardware inspection |
| HW-POWER-BMS-002 | BMS SHALL prevent over-discharge (<20% SOC cutoff) | Critical | Battery test |
| HW-POWER-BMS-003 | BMS SHALL prevent over-current (>50A continuous) | Critical | Current test |
| HW-POWER-BMS-004 | BMS SHALL cut power if temperature >60°C | Critical | Thermal test |
| HW-POWER-BMS-005 | BMS SHALL publish battery status to ROS 2 at 1 Hz | High | Integration test |
| HW-POWER-BMS-006 | BMS SHALL support CAN bus or serial interface | High | Datasheet |

**Total Requirements: 6**

---

### 5.3 Power Distribution (HW-POWER-DIST)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-POWER-DIST-001 | System SHALL have 48V → 12V DC-DC converter (≥10A) | Critical | Hardware BOM |
| HW-POWER-DIST-002 | System SHALL have 12V → 5V DC-DC converter (≥5A) | Critical | Hardware BOM |
| HW-POWER-DIST-003 | System SHALL have fuses/circuit breakers on all power rails | Critical | Hardware inspection |
| HW-POWER-DIST-004 | System SHALL have power distribution board with monitoring | High | Hardware BOM |
| HW-POWER-DIST-005 | System SHALL isolate compute power from motor power | High | Electrical design |

**Total Requirements: 5**

---

### 5.4 Charging System (HW-POWER-CHARGE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-POWER-CHARGE-001 | System SHALL have onboard charger or external charging port | Critical | Hardware BOM |
| HW-POWER-CHARGE-002 | Charger SHALL support 220V AC input | Critical | Datasheet |
| HW-POWER-CHARGE-003 | Charger SHALL fully charge battery in <6 hours | High | Charging test |
| HW-POWER-CHARGE-004 | System SHALL support autonomous docking for charging (optional) | Low | Integration test |

**Total Requirements: 4**

---

## 6. Mechanical Structure Requirements

### 6.1 Chassis (HW-MECH-CHASSIS)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-MECH-CHASSIS-001 | Chassis SHALL support 100kg payload (wheelchair + passenger) | Critical | Load test |
| HW-MECH-CHASSIS-002 | Chassis SHALL have wheelbase 0.6m, track width 0.5m | Critical | Mechanical drawing |
| HW-MECH-CHASSIS-003 | Chassis SHALL have ground clearance ≥0.1m | High | Mechanical design |
| HW-MECH-CHASSIS-004 | Chassis SHALL be constructed from aluminum or steel | High | Material spec |
| HW-MECH-CHASSIS-005 | Chassis SHALL have mounting points for all sensors and components | Critical | Mechanical design |

**Total Requirements: 5**

---

### 6.2 Wheelchair Attachment (HW-MECH-DOCK)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-MECH-DOCK-001 | System SHALL have mechanical docking interface for wheelchair | Critical | Mechanical design |
| HW-MECH-DOCK-002 | Docking SHALL support standard wheelchair dimensions (600-700mm width) | Critical | Compatibility test |
| HW-MECH-DOCK-003 | Docking SHALL have locking mechanism (manual or automatic) | Critical | Mechanical test |
| HW-MECH-DOCK-004 | Docking SHALL withstand 10g shock load | High | Shock test |
| HW-MECH-DOCK-005 | Docking SHALL align wheelchair within ±5mm (precision requirement) | Critical | Alignment test |

**Total Requirements: 5**

---

### 6.3 Enclosures & Weatherproofing (HW-MECH-ENCL)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-MECH-ENCL-001 | Compute enclosure SHALL have IP54+ rating (dust + water splash) | Critical | IP test |
| HW-MECH-ENCL-002 | Battery enclosure SHALL have IP54+ rating | Critical | IP test |
| HW-MECH-ENCL-003 | Motor controllers SHALL have IP65+ rating | Critical | IP test |
| HW-MECH-ENCL-004 | All connectors SHALL be IP67+ rated (submersible) | High | IP test |
| HW-MECH-ENCL-005 | System SHALL have cable glands for all external cables | High | Hardware inspection |
| HW-MECH-ENCL-006 | System SHALL have ventilation/cooling for enclosed components | High | Thermal design |

**Total Requirements: 6**

---

## 7. User Interface Hardware Requirements

### 7.1 Touch Screen (HW-UI-SCREEN)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-UI-SCREEN-001 | System SHALL have touch screen display (7-10 inches diagonal) | Critical | Hardware BOM |
| HW-UI-SCREEN-002 | Touch screen SHALL have 1280×800 resolution minimum | High | Datasheet |
| HW-UI-SCREEN-003 | Touch screen SHALL support capacitive touch (multi-touch) | High | Datasheet |
| HW-UI-SCREEN-004 | Touch screen SHALL be viewable in daylight (≥400 nits brightness) | High | Field test |
| HW-UI-SCREEN-005 | Touch screen SHALL have IP65+ front panel (waterproof) | Critical | IP test |
| HW-UI-SCREEN-006 | Touch screen SHALL be mounted at 1.0-1.2m height (wheelchair accessible) | High | Ergonomic test |

**Total Requirements: 6**

---

### 7.2 eHMI Hardware (HW-UI-EHMI)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-UI-EHMI-001 | System SHALL have ESP32-S3 microcontroller for eHMI | Critical | Hardware BOM |
| HW-UI-EHMI-002 | System SHALL have LED strip (WS2812B, ≥60 LEDs) | Critical | Hardware BOM |
| HW-UI-EHMI-003 | System SHALL have LED matrix (HUB75, 64×32 pixels) | High | Hardware BOM |
| HW-UI-EHMI-004 | System SHALL have I2S audio amplifier + speaker | High | Hardware BOM |
| HW-UI-EHMI-005 | eHMI SHALL communicate with main computer via serial (115200 baud) | Critical | Interface test |
| HW-UI-EHMI-006 | eHMI hardware SHALL have IP54+ rating | High | IP test |

**Total Requirements: 6**

---

### 7.3 Emergency Stop Button (HW-UI-ESTOP)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-UI-ESTOP-001 | System SHALL have physical emergency stop button (red mushroom-head) | Critical | Hardware BOM |
| HW-UI-ESTOP-002 | Emergency stop SHALL be accessible to passenger | Critical | Ergonomic test |
| HW-UI-ESTOP-003 | Emergency stop SHALL have dual-channel redundant circuit | Critical | Electrical design |
| HW-UI-ESTOP-004 | Emergency stop SHALL comply with ISO 13850 | Critical | Certification |
| HW-UI-ESTOP-005 | Emergency stop SHALL be twist-to-reset type | Critical | Hardware inspection |

**Total Requirements: 5**

---

## 8. Communication Hardware Requirements

### 8.1 Wireless Communication (HW-COMM-WIRELESS)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-COMM-WIRELESS-001 | System SHALL have WiFi 6 (802.11ax) or better | Critical | Datasheet |
| HW-COMM-WIRELESS-002 | System SHALL have 4G/5G LTE modem (optional, for remote areas) | Medium | Hardware BOM |
| HW-COMM-WIRELESS-003 | WiFi SHALL support 5 GHz band (less interference) | High | Datasheet |
| HW-COMM-WIRELESS-004 | WiFi SHALL have external antenna (better range) | High | Hardware BOM |

**Total Requirements: 4**

---

### 8.2 Wired Communication (HW-COMM-WIRED)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-COMM-WIRED-001 | System SHALL have CAN bus for motor controllers | Critical | Electrical design |
| HW-COMM-WIRED-002 | System SHALL have USB 3.0 hubs for sensor connections | Critical | Hardware BOM |
| HW-COMM-WIRED-003 | System SHALL have Ethernet switch for GigE cameras (if used) | Medium | Hardware BOM |
| HW-COMM-WIRED-004 | System SHALL use shielded cables for all signal connections | High | Cable spec |

**Total Requirements: 4**

---

## 9. Environmental & Durability Requirements

### 9.1 Temperature (HW-ENV-TEMP)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-ENV-TEMP-001 | System SHALL operate in -10°C to +45°C ambient temperature | High | Climate chamber |
| HW-ENV-TEMP-002 | Battery SHALL operate in 0°C to +40°C (charging: 10-35°C) | Critical | Battery test |
| HW-ENV-TEMP-003 | Motors SHALL operate in -20°C to +60°C | High | Motor test |
| HW-ENV-TEMP-004 | System SHALL have active cooling for compute and power electronics | Critical | Thermal design |

**Total Requirements: 4**

---

### 9.2 Weather Protection (HW-ENV-WEATHER)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-ENV-WEATHER-001 | System SHALL operate in light rain (<2.5mm/hr) | High | Rain chamber |
| HW-ENV-WEATHER-002 | All outdoor-facing sensors SHALL have IP65+ rating | Critical | IP test |
| HW-ENV-WEATHER-003 | All connectors SHALL be IP67+ rated | High | IP test |
| HW-ENV-WEATHER-004 | System SHALL have drainage paths for water ingress | High | Design review |

**Total Requirements: 4**

---

### 9.3 Vibration & Shock (HW-ENV-VIB)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| HW-ENV-VIB-001 | System SHALL withstand 2g continuous vibration | High | Vibration test |
| HW-ENV-VIB-002 | System SHALL withstand 10g shock load | High | Shock test |
| HW-ENV-VIB-003 | All mounting hardware SHALL have thread locking | High | Hardware inspection |
| HW-ENV-VIB-004 | Compute and sensitive electronics SHALL have vibration damping | Medium | Mechanical design |

**Total Requirements: 4**

---

## 10. Requirements Summary

### Requirements Count by Category

| Category | Total Requirements | Critical | High | Medium | Low |
|----------|-------------------|----------|------|--------|-----|
| **Compute Platform** | **14** | 11 | 3 | 0 | 0 |
| **Sensors** | **34** | 19 | 13 | 2 | 1 |
| - 3D LiDAR | 9 | 7 | 1 | 1 | 0 |
| - RGB Cameras | 9 | 6 | 2 | 1 | 0 |
| - IMU | 6 | 4 | 1 | 0 | 1 |
| - Encoders | 5 | 2 | 3 | 0 | 0 |
| - Safety Sensors | 5 | 4 | 1 | 0 | 0 |
| **Actuators** | **17** | 12 | 4 | 1 | 0 |
| **Power System** | **22** | 17 | 4 | 0 | 1 |
| **Mechanical** | **16** | 10 | 6 | 0 | 0 |
| **User Interface HW** | **17** | 12 | 5 | 0 | 0 |
| **Communication** | **8** | 3 | 4 | 1 | 0 |
| **Environmental** | **12** | 5 | 6 | 1 | 0 |
| **TOTAL** | **140** | **89 (64%)** | **45 (32%)** | **5 (4%)** | **1 (1%)** |

---

## 11. Acceptance Criteria

### 11.1 MVP Hardware

- ✅ GMKtec Nucbox K6 operational
- ✅ 3D LiDAR + 2× cameras + IMU functional
- ✅ Swerve drive motors + encoders operational
- ✅ Battery providing 4+ hours runtime
- ✅ Touch screen UI + basic eHMI functional
- ✅ Emergency stop circuit tested

### 11.2 Production Hardware

- ✅ All MVP + all high-priority requirements
- ✅ IP54+ weatherproofing tested (rain chamber)
- ✅ -10°C to +45°C operation validated
- ✅ 500 charge cycles tested
- ✅ Full eHMI (LED strip + matrix + audio)
- ✅ 1000 hours durability testing

---

**Document Status:** Draft
**Next Review:** After hardware procurement and initial integration
**Approvals Required:** Hardware Lead, System Architect, Safety Engineer
