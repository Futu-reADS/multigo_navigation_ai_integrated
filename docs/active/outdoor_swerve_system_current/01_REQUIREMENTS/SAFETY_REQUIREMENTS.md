# Safety Requirements

**Document ID:** REQ-SAFE-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft
**Classification:** Internal

---

## Document Control

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-15 | Safety Engineer | Initial safety requirements specification |

---

## 1. Introduction

### 1.1 Purpose

This document specifies the **Safety Requirements** for the outdoor-first wheelchair transport robot. Safety is the highest priority subsystem, ensuring passenger safety, operational safety, and environmental safety during all operational modes.

### 1.2 Scope

This specification covers:
- **Passenger Safety** - Protection of wheelchair-bound passenger during transport
- **Operational Safety** - Safe autonomous operation (collision avoidance, emergency stop)
- **Environmental Safety** - Safe operation in various weather, slopes, lighting conditions
- **System Safety** - Redundancy, fault detection, fail-safe modes
- **Software Safety** - Safety supervisor, velocity governor, collision predictor
- **Hardware Safety** - Emergency stop circuit, sensors, brakes

**Out of Scope:**
- Mechanical design of wheelchair attachment (covered in mechanical specs)
- Physical docking mechanism safety (covered in DOCKING_SYSTEM_REQUIREMENTS.md)
- Battery chemistry and electrical safety (covered in HARDWARE_REQUIREMENTS.md)

### 1.3 Related Documents

- **SYSTEM_REQUIREMENTS.md** - Section 1.3 (SAFE-REQ-001 to 014)
- **OVERALL_SYSTEM_ARCHITECTURE.md** - Section 4 (Safety Architecture)
- **question_answers.md** - Safety clarifications
- **EHMI_SYSTEM_REFERENCE.md** - Safety-related eHMI states

### 1.4 Safety Philosophy

The system follows a **defense-in-depth** safety strategy with 4 layers:

**Layer 1: Emergency Stop (Hardware)**
- Physical emergency stop button (passenger + remote operator)
- Hardware-based motor cutoff (<100ms response)
- Independent of software control

**Layer 2: System Monitoring (Software Watchdog)**
- Real-time monitoring of all critical subsystems
- Automatic degraded mode on sensor failure
- Safe state on communication timeout

**Layer 3: Collision Avoidance (Perception + Planning)**
- 3D LiDAR obstacle detection
- Dynamic obstacle tracking
- Predictive collision avoidance
- Safety zones (stop <0.5m, slow <1.5m, caution <3m)

**Layer 4: Operational Safety (Constraints)**
- Speed limits (1.0 m/s with passenger, 1.5 m/s without)
- Acceleration limits (0.5 m/s²)
- Slope limits (10° max)
- Weather limits (light rain only)

---

## 2. Passenger Safety Requirements

### 2.1 Speed Limits (SAFE-PASS-SPEED)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-PASS-SPEED-001 | System SHALL NOT exceed 1.0 m/s with passenger onboard | Critical | Safety test |
| SAFE-PASS-SPEED-002 | System SHALL limit speed to 0.5 m/s during docking operations | Critical | Safety test |
| SAFE-PASS-SPEED-003 | System SHALL limit speed to 0.3 m/s on slopes >5° | Critical | Slope test |
| SAFE-PASS-SPEED-004 | System SHALL reduce speed to 0.7 m/s in low visibility (<10m) | High | Field test |
| SAFE-PASS-SPEED-005 | System SHALL enforce speed limits via hardware velocity governor | Critical | Hardware inspection |

**Total Requirements: 5**

---

### 2.2 Acceleration Limits (SAFE-PASS-ACCEL)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-PASS-ACCEL-001 | System SHALL limit linear acceleration to 0.5 m/s² with passenger | Critical | Safety test |
| SAFE-PASS-ACCEL-002 | System SHALL limit angular acceleration to 0.3 rad/s² with passenger | Critical | Safety test |
| SAFE-PASS-ACCEL-003 | System SHALL limit jerk to 2.0 m/s³ with passenger | High | Safety test |
| SAFE-PASS-ACCEL-004 | System SHALL ramp velocity changes over ≥1 second | High | Safety test |
| SAFE-PASS-ACCEL-005 | System SHALL use S-curve acceleration profiles (smooth start/stop) | Medium | Integration test |

**Total Requirements: 5**

---

### 2.3 Passenger Detection (SAFE-PASS-DETECT)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-PASS-DETECT-001 | System SHALL detect passenger presence via weight sensors (>20kg) | Critical | Hardware test |
| SAFE-PASS-DETECT-002 | System SHALL verify passenger detection before starting transport | Critical | Safety test |
| SAFE-PASS-DETECT-003 | System SHALL stop if passenger weight drops below threshold during transport | Critical | Fault injection test |
| SAFE-PASS-DETECT-004 | System SHALL require operator confirmation after passenger detection | High | Integration test |
| SAFE-PASS-DETECT-005 | System SHALL log passenger detection events with timestamp | Medium | Log inspection |

**Total Requirements: 5**

---

### 2.4 Tilt Detection & Stability (SAFE-PASS-TILT)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-PASS-TILT-001 | System SHALL monitor tilt angle (roll, pitch) at 50 Hz minimum | Critical | Performance test |
| SAFE-PASS-TILT-002 | System SHALL issue warning if tilt exceeds ±15° (audio + eHMI) | Critical | Tilt test |
| SAFE-PASS-TILT-003 | System SHALL stop immediately if tilt exceeds ±20° | Critical | Safety test |
| SAFE-PASS-TILT-004 | System SHALL prevent operation on slopes >10° | Critical | Slope test |
| SAFE-PASS-TILT-005 | System SHALL compute dynamic stability margin (real-time) | High | Unit test |
| SAFE-PASS-TILT-006 | System SHALL refuse mission if initial tilt >5° | High | Integration test |

**Total Requirements: 6**

---

### 2.5 Emergency Stop (Passenger-Initiated) (SAFE-PASS-ESTOP)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-PASS-ESTOP-001 | System SHALL have physical emergency stop button accessible to passenger | Critical | Hardware inspection |
| SAFE-PASS-ESTOP-002 | System SHALL stop within 0.5m at 1.0 m/s on emergency stop activation | Critical | Braking test |
| SAFE-PASS-ESTOP-003 | System SHALL cut motor power within 100ms of emergency stop | Critical | Hardware test |
| SAFE-PASS-ESTOP-004 | System SHALL activate eHMI emergency state (red flashing, siren) | Critical | Integration test |
| SAFE-PASS-ESTOP-005 | System SHALL require manual reset after emergency stop | Critical | Safety test |
| SAFE-PASS-ESTOP-006 | System SHALL log emergency stop events with location, timestamp, reason | High | Log inspection |

**Total Requirements: 6**

---

### 2.6 Seatbelt & Restraints (SAFE-PASS-RESTRAINT)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-PASS-RESTRAINT-001 | System SHALL detect seatbelt engagement via magnetic sensor | Critical | Hardware test |
| SAFE-PASS-RESTRAINT-002 | System SHALL refuse to start transport if seatbelt not engaged | Critical | Safety test |
| SAFE-PASS-RESTRAINT-003 | System SHALL alert operator if seatbelt disengages during transport | Critical | Fault injection test |
| SAFE-PASS-RESTRAINT-004 | System SHALL stop immediately if seatbelt disengages during transport | Critical | Safety test |

**Total Requirements: 4**

---

## 3. Operational Safety Requirements

### 3.1 Obstacle Detection (SAFE-OPS-OBS)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-OPS-OBS-001 | System SHALL detect obstacles in 360° field of view | Critical | Field test |
| SAFE-OPS-OBS-002 | System SHALL detect obstacles at 50 Hz minimum (3D LiDAR) | Critical | Performance test |
| SAFE-OPS-OBS-003 | System SHALL detect obstacles ≥0.1m height, ≥0.05m diameter | Critical | Obstacle test |
| SAFE-OPS-OBS-004 | System SHALL detect obstacles up to 15m range | High | Field test |
| SAFE-OPS-OBS-005 | System SHALL classify obstacles by distance (stop, slow, caution zones) | Critical | Integration test |
| SAFE-OPS-OBS-006 | System SHALL track dynamic obstacles (pedestrians, vehicles) | High | Tracking test |
| SAFE-OPS-OBS-007 | System SHALL predict obstacle trajectories (1-3 seconds) | High | Integration test |

**Total Requirements: 7**

---

### 3.2 Safety Zones (SAFE-OPS-ZONE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-OPS-ZONE-001 | System SHALL define stop zone (<0.5m from obstacle) | Critical | Integration test |
| SAFE-OPS-ZONE-002 | System SHALL stop immediately if obstacle enters stop zone | Critical | Safety test |
| SAFE-OPS-ZONE-003 | System SHALL define slow zone (0.5-1.5m from obstacle) | Critical | Integration test |
| SAFE-OPS-ZONE-004 | System SHALL reduce speed to 0.3 m/s in slow zone | Critical | Safety test |
| SAFE-OPS-ZONE-005 | System SHALL define caution zone (1.5-3m from obstacle) | High | Integration test |
| SAFE-OPS-ZONE-006 | System SHALL reduce speed to 0.7 m/s in caution zone | High | Safety test |
| SAFE-OPS-ZONE-007 | System SHALL adapt zone sizes based on robot speed | High | Integration test |

**Total Requirements: 7**

---

### 3.3 Collision Avoidance (SAFE-OPS-COLL)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-OPS-COLL-001 | System SHALL compute time-to-collision (TTC) for all obstacles | Critical | Unit test |
| SAFE-OPS-COLL-002 | System SHALL initiate braking if TTC <2 seconds | Critical | Safety test |
| SAFE-OPS-COLL-003 | System SHALL re-plan path if collision predicted within 5 seconds | High | Integration test |
| SAFE-OPS-COLL-004 | System SHALL prefer slowing over swerving (passenger comfort) | High | Safety test |
| SAFE-OPS-COLL-005 | System SHALL maintain safe distance ≥1.0m from obstacles | Critical | Field test |
| SAFE-OPS-COLL-006 | System SHALL use conservative bounding box (robot + 0.2m margin) | Critical | Configuration |

**Total Requirements: 6**

---

### 3.4 Emergency Stop (Automatic) (SAFE-OPS-ESTOP)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-OPS-ESTOP-001 | System SHALL initiate emergency stop if obstacle <0.3m | Critical | Safety test |
| SAFE-OPS-ESTOP-002 | System SHALL initiate emergency stop if tilt >20° | Critical | Tilt test |
| SAFE-OPS-ESTOP-003 | System SHALL initiate emergency stop if sensor failure detected | Critical | Fault injection test |
| SAFE-OPS-ESTOP-004 | System SHALL initiate emergency stop if communication timeout (>1s) | Critical | Timeout test |
| SAFE-OPS-ESTOP-005 | System SHALL initiate emergency stop if software crash detected | Critical | Fault injection test |
| SAFE-OPS-ESTOP-006 | System SHALL log emergency stop reason with full sensor snapshot | High | Log inspection |

**Total Requirements: 6**

---

### 3.5 Remote Emergency Stop (SAFE-OPS-REMOTE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-OPS-REMOTE-001 | System SHALL support remote emergency stop via UI | Critical | Integration test |
| SAFE-OPS-REMOTE-002 | System SHALL support remote emergency stop via dedicated wireless button | High | Hardware test |
| SAFE-OPS-REMOTE-003 | System SHALL execute remote emergency stop within 500ms | Critical | Latency test |
| SAFE-OPS-REMOTE-004 | System SHALL maintain remote emergency stop connection at 10 Hz heartbeat | Critical | Performance test |
| SAFE-OPS-REMOTE-005 | System SHALL initiate automatic emergency stop if heartbeat lost | Critical | Fault injection test |

**Total Requirements: 5**

---

### 3.6 Degraded Mode Operation (SAFE-OPS-DEGRADE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-OPS-DEGRADE-001 | System SHALL enter degraded mode if LiDAR fails (camera-only) | High | Fault injection test |
| SAFE-OPS-DEGRADE-002 | System SHALL reduce max speed to 0.5 m/s in degraded mode | Critical | Safety test |
| SAFE-OPS-DEGRADE-003 | System SHALL notify operator of degraded mode (eHMI + UI) | Critical | Integration test |
| SAFE-OPS-DEGRADE-004 | System SHALL refuse passenger transport in degraded mode | Critical | Safety test |
| SAFE-OPS-DEGRADE-005 | System SHALL enter limp-home mode if navigation fails | High | Integration test |
| SAFE-OPS-DEGRADE-006 | System SHALL stop if multiple critical sensors fail | Critical | Fault injection test |

**Total Requirements: 6**

---

## 4. Environmental Safety Requirements

### 4.1 Slope Limits (SAFE-ENV-SLOPE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-ENV-SLOPE-001 | System SHALL operate on slopes up to 10° (17.6% grade) with 100kg load | Critical | Slope test |
| SAFE-ENV-SLOPE-002 | System SHALL refuse operation on slopes >10° | Critical | Slope test |
| SAFE-ENV-SLOPE-003 | System SHALL reduce speed to 0.3 m/s on slopes >5° | Critical | Safety test |
| SAFE-ENV-SLOPE-004 | System SHALL compute real-time slope angle (roll + pitch) | Critical | Unit test |
| SAFE-ENV-SLOPE-005 | System SHALL warn operator if approaching slope >7° | High | Integration test |

**Total Requirements: 5**

---

### 4.2 Weather Limits (SAFE-ENV-WEATHER)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-ENV-WEATHER-001 | System SHALL operate in light rain (<2.5mm/hr) | High | Rain chamber test |
| SAFE-ENV-WEATHER-002 | System SHALL refuse operation in heavy rain (>7.5mm/hr) | Critical | Rain sensor test |
| SAFE-ENV-WEATHER-003 | System SHALL refuse operation if rain sensor fails | High | Fault injection test |
| SAFE-ENV-WEATHER-004 | System SHALL monitor wind speed, refuse operation if >40 km/h | High | Wind test |
| SAFE-ENV-WEATHER-005 | System SHALL reduce speed to 0.7 m/s in rain | High | Safety test |

**Total Requirements: 5**

---

### 4.3 Temperature Limits (SAFE-ENV-TEMP)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-ENV-TEMP-001 | System SHALL operate in -10°C to +45°C ambient temperature | High | Climate chamber |
| SAFE-ENV-TEMP-002 | System SHALL warn operator if temperature <-5°C or >40°C | Medium | Integration test |
| SAFE-ENV-TEMP-003 | System SHALL monitor battery temperature, refuse operation if >60°C | Critical | Temperature sensor test |
| SAFE-ENV-TEMP-004 | System SHALL monitor motor temperature, reduce power if >80°C | High | Thermal test |

**Total Requirements: 4**

---

### 4.4 Lighting Requirements (SAFE-ENV-LIGHT)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-ENV-LIGHT-001 | System SHALL operate in daylight (1,000-100,000 lux) | Critical | Field test |
| SAFE-ENV-LIGHT-002 | System SHALL operate at night (<10 lux) with active lighting | High | Night test |
| SAFE-ENV-LIGHT-003 | System SHALL activate headlights automatically when ambient <500 lux | High | Integration test |
| SAFE-ENV-LIGHT-004 | System SHALL activate safety beacon (red flashing) during operation | Critical | Hardware inspection |
| SAFE-ENV-LIGHT-005 | System SHALL reduce speed to 0.7 m/s in low visibility (<10m) | High | Safety test |

**Total Requirements: 5**

---

## 5. System Safety Requirements

### 5.1 Redundancy (SAFE-SYS-REDUND)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-SYS-REDUND-001 | System SHALL have redundant emergency stop circuits (2× independent) | Critical | Hardware inspection |
| SAFE-SYS-REDUND-002 | System SHALL have redundant communication links (WiFi + LTE) | High | Integration test |
| SAFE-SYS-REDUND-003 | System SHALL have dual IMUs for tilt detection | High | Hardware inspection |
| SAFE-SYS-REDUND-004 | System SHALL fuse data from multiple sensors (LiDAR + cameras) | Critical | Integration test |
| SAFE-SYS-REDUND-005 | System SHALL use dual power supplies (main + backup) | Medium | Hardware inspection |

**Total Requirements: 5**

---

### 5.2 Fault Detection (SAFE-SYS-FAULT)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-SYS-FAULT-001 | System SHALL detect LiDAR failure (no data for >1 second) | Critical | Fault injection test |
| SAFE-SYS-FAULT-002 | System SHALL detect camera failure (no frames for >1 second) | Critical | Fault injection test |
| SAFE-SYS-FAULT-003 | System SHALL detect IMU failure (no data or frozen values) | Critical | Fault injection test |
| SAFE-SYS-FAULT-004 | System SHALL detect motor controller failure (no feedback) | Critical | Fault injection test |
| SAFE-SYS-FAULT-005 | System SHALL detect battery failure (voltage drop, over-current) | Critical | Battery fault test |
| SAFE-SYS-FAULT-006 | System SHALL publish diagnostics at 1 Hz minimum | High | Integration test |

**Total Requirements: 6**

---

### 5.3 Fail-Safe Modes (SAFE-SYS-FAILSAFE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-SYS-FAILSAFE-001 | System SHALL enter safe state on critical sensor failure (stop, hold) | Critical | Fault injection test |
| SAFE-SYS-FAILSAFE-002 | System SHALL enter limp-home mode on non-critical sensor failure | High | Integration test |
| SAFE-SYS-FAILSAFE-003 | System SHALL disable passenger transport on safety-critical failure | Critical | Safety test |
| SAFE-SYS-FAILSAFE-004 | System SHALL maintain emergency stop capability in all failure modes | Critical | Safety test |
| SAFE-SYS-FAILSAFE-005 | System SHALL log all transitions to fail-safe modes | High | Log inspection |

**Total Requirements: 5**

---

### 5.4 Watchdog Timers (SAFE-SYS-WATCHDOG)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-SYS-WATCHDOG-001 | System SHALL implement hardware watchdog timer (1 second timeout) | Critical | Hardware inspection |
| SAFE-SYS-WATCHDOG-002 | System SHALL reset on watchdog timeout | Critical | Fault injection test |
| SAFE-SYS-WATCHDOG-003 | System SHALL implement software watchdog for critical nodes | Critical | Integration test |
| SAFE-SYS-WATCHDOG-004 | System SHALL enter safe state if software watchdog expires | Critical | Fault injection test |
| SAFE-SYS-WATCHDOG-005 | System SHALL require explicit heartbeat from safety supervisor at 10 Hz | Critical | Performance test |

**Total Requirements: 5**

---

### 5.5 Communication Timeouts (SAFE-SYS-TIMEOUT)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-SYS-TIMEOUT-001 | System SHALL timeout if no cmd_vel received for >1 second | Critical | Timeout test |
| SAFE-SYS-TIMEOUT-002 | System SHALL timeout if no sensor data for >1 second | Critical | Timeout test |
| SAFE-SYS-TIMEOUT-003 | System SHALL timeout if no UI heartbeat for >5 seconds | High | Timeout test |
| SAFE-SYS-TIMEOUT-004 | System SHALL initiate safe stop on any critical timeout | Critical | Safety test |

**Total Requirements: 4**

---

## 6. Software Safety Requirements

### 6.1 Safety Supervisor Node (SAFE-SW-SUPER)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-SW-SUPER-001 | System SHALL implement dedicated safety_supervisor_node (C++) | Critical | Code inspection |
| SAFE-SW-SUPER-002 | Safety supervisor SHALL run on dedicated CPU core (isolation) | High | System configuration |
| SAFE-SW-SUPER-003 | Safety supervisor SHALL monitor all critical subsystems at 10 Hz | Critical | Performance test |
| SAFE-SW-SUPER-004 | Safety supervisor SHALL have authority to override velocity commands | Critical | Integration test |
| SAFE-SW-SUPER-005 | Safety supervisor SHALL publish diagnostics to /safety/status topic | Critical | Integration test |
| SAFE-SW-SUPER-006 | Safety supervisor SHALL log all safety violations | High | Log inspection |

**Total Requirements: 6**

---

### 6.2 Velocity Governor (SAFE-SW-VGOV)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-SW-VGOV-001 | System SHALL implement velocity governor filtering all cmd_vel commands | Critical | Integration test |
| SAFE-SW-VGOV-002 | Velocity governor SHALL enforce passenger speed limit (1.0 m/s) | Critical | Safety test |
| SAFE-SW-VGOV-003 | Velocity governor SHALL enforce acceleration limits (0.5 m/s²) | Critical | Safety test |
| SAFE-SW-VGOV-004 | Velocity governor SHALL reject commands exceeding safety limits | Critical | Unit test |
| SAFE-SW-VGOV-005 | Velocity governor SHALL publish filtered commands to /cmd_vel_safe topic | Critical | Integration test |

**Total Requirements: 5**

---

### 6.3 Collision Predictor (SAFE-SW-PRED)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-SW-PRED-001 | System SHALL implement collision predictor using LiDAR + velocity | Critical | Unit test |
| SAFE-SW-PRED-002 | Collision predictor SHALL compute time-to-collision (TTC) for all obstacles | Critical | Integration test |
| SAFE-SW-PRED-003 | Collision predictor SHALL predict collisions 1-3 seconds ahead | Critical | Safety test |
| SAFE-SW-PRED-004 | Collision predictor SHALL trigger emergency stop if TTC <1 second | Critical | Safety test |
| SAFE-SW-PRED-005 | Collision predictor SHALL publish collision warnings to /safety/collision topic | High | Integration test |

**Total Requirements: 5**

---

### 6.4 Emergency Stop Handler (SAFE-SW-ESTOP)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-SW-ESTOP-001 | System SHALL implement emergency stop handler (hardware + software) | Critical | Code inspection |
| SAFE-SW-ESTOP-002 | Emergency stop SHALL publish zero velocity command within 50ms | Critical | Latency test |
| SAFE-SW-ESTOP-003 | Emergency stop SHALL disable all motion controllers | Critical | Integration test |
| SAFE-SW-ESTOP-004 | Emergency stop SHALL activate eHMI emergency state | Critical | Integration test |
| SAFE-SW-ESTOP-005 | Emergency stop SHALL require manual reset via UI | Critical | Safety test |

**Total Requirements: 5**

---

### 6.5 Heartbeat Monitoring (SAFE-SW-HEART)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-SW-HEART-001 | All critical nodes SHALL publish heartbeat at 10 Hz minimum | Critical | Integration test |
| SAFE-SW-HEART-002 | Safety supervisor SHALL monitor heartbeats from all critical nodes | Critical | Integration test |
| SAFE-SW-HEART-003 | System SHALL initiate safe stop if any critical heartbeat lost | Critical | Fault injection test |
| SAFE-SW-HEART-004 | Heartbeat messages SHALL include node status and timestamp | Medium | Message inspection |

**Total Requirements: 4**

---

## 7. Hardware Safety Requirements

### 7.1 Emergency Stop Circuit (SAFE-HW-ESTOP)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-HW-ESTOP-001 | System SHALL have hardware emergency stop circuit (independent of software) | Critical | Hardware inspection |
| SAFE-HW-ESTOP-002 | Emergency stop circuit SHALL cut motor power within 100ms | Critical | Hardware test |
| SAFE-HW-ESTOP-003 | Emergency stop button SHALL be red, mushroom-head, twist-to-reset | Critical | Hardware inspection |
| SAFE-HW-ESTOP-004 | Emergency stop circuit SHALL be dual-channel (redundant) | Critical | Hardware inspection |
| SAFE-HW-ESTOP-005 | Emergency stop circuit SHALL comply with ISO 13850 | Critical | Certification |

**Total Requirements: 5**

---

### 7.2 Battery Management System (SAFE-HW-BMS)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-HW-BMS-001 | System SHALL have BMS monitoring voltage, current, temperature | Critical | Hardware inspection |
| SAFE-HW-BMS-002 | BMS SHALL prevent over-discharge (<20% SOC) | Critical | Battery test |
| SAFE-HW-BMS-003 | BMS SHALL prevent over-current (>50A continuous) | Critical | Current test |
| SAFE-HW-BMS-004 | BMS SHALL cut power if temperature >60°C | Critical | Thermal test |
| SAFE-HW-BMS-005 | BMS SHALL publish battery status to ROS 2 at 1 Hz | High | Integration test |
| SAFE-HW-BMS-006 | System SHALL initiate return-to-home if battery <30% SOC | High | Integration test |

**Total Requirements: 6**

---

### 7.3 Motor Controllers (SAFE-HW-MOTOR)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-HW-MOTOR-001 | Motor controllers SHALL have hardware current limiting (10A per motor) | Critical | Hardware inspection |
| SAFE-HW-MOTOR-002 | Motor controllers SHALL have thermal protection (shutdown >90°C) | Critical | Thermal test |
| SAFE-HW-MOTOR-003 | Motor controllers SHALL accept emergency stop signal (hardware input) | Critical | Hardware inspection |
| SAFE-HW-MOTOR-004 | Motor controllers SHALL report status to ROS 2 at 10 Hz | High | Integration test |
| SAFE-HW-MOTOR-005 | Motor controllers SHALL enter safe state on communication loss | Critical | Fault injection test |

**Total Requirements: 5**

---

### 7.4 Brake System (SAFE-HW-BRAKE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-HW-BRAKE-001 | System SHALL have electromechanical brakes (fail-safe engaged) | Critical | Hardware inspection |
| SAFE-HW-BRAKE-002 | Brakes SHALL engage automatically on power loss | Critical | Power loss test |
| SAFE-HW-BRAKE-003 | Brakes SHALL engage automatically on emergency stop | Critical | Safety test |
| SAFE-HW-BRAKE-004 | Brakes SHALL hold robot on 10° slope with 100kg load | Critical | Brake test |
| SAFE-HW-BRAKE-005 | Brake engagement time SHALL be <200ms | Critical | Latency test |

**Total Requirements: 5**

---

### 7.5 Safety Sensors (SAFE-HW-SENSOR)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-HW-SENSOR-001 | System SHALL have 3D LiDAR for obstacle detection | Critical | Hardware inspection |
| SAFE-HW-SENSOR-002 | System SHALL have dual IMUs (9-DOF) for tilt detection | Critical | Hardware inspection |
| SAFE-HW-SENSOR-003 | System SHALL have weight sensors for passenger detection | Critical | Hardware inspection |
| SAFE-HW-SENSOR-004 | System SHALL have seatbelt engagement sensor | Critical | Hardware inspection |
| SAFE-HW-SENSOR-005 | System SHALL have rain sensor for weather monitoring | High | Hardware inspection |
| SAFE-HW-SENSOR-006 | All safety sensors SHALL publish data at ≥10 Hz | Critical | Performance test |

**Total Requirements: 6**

---

## 8. Testing & Validation Requirements

### 8.1 Safety Testing (SAFE-TEST-SAFETY)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-TEST-SAFETY-001 | System SHALL complete 100 emergency stop tests (0% failure) | Critical | Test report |
| SAFE-TEST-SAFETY-002 | System SHALL complete 50 collision avoidance tests | Critical | Test report |
| SAFE-TEST-SAFETY-003 | System SHALL complete 50 slope stability tests (up to 10°) | Critical | Test report |
| SAFE-TEST-SAFETY-004 | System SHALL complete 20 sensor failure tests | Critical | Test report |
| SAFE-TEST-SAFETY-005 | System SHALL complete 20 passenger safety tests (full load) | Critical | Test report |

**Total Requirements: 5**

---

### 8.2 Fault Injection Testing (SAFE-TEST-FAULT)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| SAFE-TEST-FAULT-001 | System SHALL be tested with LiDAR failure injection | Critical | Fault injection test |
| SAFE-TEST-FAULT-002 | System SHALL be tested with camera failure injection | Critical | Fault injection test |
| SAFE-TEST-FAULT-003 | System SHALL be tested with IMU failure injection | Critical | Fault injection test |
| SAFE-TEST-FAULT-004 | System SHALL be tested with motor controller failure injection | Critical | Fault injection test |
| SAFE-TEST-FAULT-005 | System SHALL be tested with communication timeout injection | Critical | Fault injection test |
| SAFE-TEST-FAULT-006 | All fault injection tests SHALL verify safe state entry | Critical | Test report |

**Total Requirements: 6**

---

### 8.3 Acceptance Criteria (SAFE-TEST-ACCEPT)

| Criterion | Target | MVP Target | Validation |
|-----------|--------|------------|------------|
| Emergency Stop Success Rate | 100% | 100% | 100 tests |
| Collision Avoidance Success Rate | >99% | >95% | 500 scenarios |
| Passenger Speed Limit Compliance | 100% | 100% | Continuous monitoring |
| Tilt Detection & Stop | 100% | 100% | 50 tilt tests |
| Sensor Failure Safe State Entry | 100% | 100% | Fault injection tests |
| Brake Engagement Time | <200ms | <300ms | Latency test |
| Emergency Stop Stopping Distance | <0.5m @ 1.0 m/s | <0.7m @ 1.0 m/s | Braking test |

---

## 9. Requirements Summary

### Requirements Count by Category

| Category | Total Requirements | Critical | High | Medium | Low |
|----------|-------------------|----------|------|--------|-----|
| **Passenger Safety** | **31** | 24 | 7 | 0 | 0 |
| - Speed Limits | 5 | 5 | 0 | 0 | 0 |
| - Acceleration Limits | 5 | 3 | 2 | 0 | 0 |
| - Passenger Detection | 5 | 3 | 1 | 1 | 0 |
| - Tilt & Stability | 6 | 5 | 1 | 0 | 0 |
| - Emergency Stop | 6 | 5 | 1 | 0 | 0 |
| - Seatbelt | 4 | 3 | 1 | 0 | 0 |
| **Operational Safety** | **42** | 34 | 8 | 0 | 0 |
| - Obstacle Detection | 7 | 5 | 2 | 0 | 0 |
| - Safety Zones | 7 | 6 | 1 | 0 | 0 |
| - Collision Avoidance | 6 | 5 | 1 | 0 | 0 |
| - Auto Emergency Stop | 6 | 6 | 0 | 0 | 0 |
| - Remote Emergency Stop | 5 | 4 | 1 | 0 | 0 |
| - Degraded Mode | 6 | 4 | 2 | 0 | 0 |
| **Environmental Safety** | **19** | 6 | 11 | 2 | 0 |
| - Slope Limits | 5 | 4 | 1 | 0 | 0 |
| - Weather Limits | 5 | 1 | 4 | 0 | 0 |
| - Temperature Limits | 4 | 1 | 2 | 1 | 0 |
| - Lighting | 5 | 2 | 2 | 1 | 0 |
| **System Safety** | **25** | 20 | 5 | 0 | 0 |
| - Redundancy | 5 | 3 | 2 | 0 | 0 |
| - Fault Detection | 6 | 6 | 0 | 0 | 0 |
| - Fail-Safe Modes | 5 | 4 | 1 | 0 | 0 |
| - Watchdog Timers | 5 | 5 | 0 | 0 | 0 |
| - Communication Timeouts | 4 | 3 | 1 | 0 | 0 |
| **Software Safety** | **25** | 22 | 3 | 0 | 0 |
| - Safety Supervisor | 6 | 5 | 1 | 0 | 0 |
| - Velocity Governor | 5 | 5 | 0 | 0 | 0 |
| - Collision Predictor | 5 | 4 | 1 | 0 | 0 |
| - Emergency Stop Handler | 5 | 5 | 0 | 0 | 0 |
| - Heartbeat Monitoring | 4 | 3 | 1 | 0 | 0 |
| **Hardware Safety** | **27** | 24 | 3 | 0 | 0 |
| - Emergency Stop Circuit | 5 | 5 | 0 | 0 | 0 |
| - Battery Management | 6 | 4 | 2 | 0 | 0 |
| - Motor Controllers | 5 | 4 | 1 | 0 | 0 |
| - Brake System | 5 | 5 | 0 | 0 | 0 |
| - Safety Sensors | 6 | 6 | 0 | 0 | 0 |
| **Testing** | **11** | 11 | 0 | 0 | 0 |
| - Safety Testing | 5 | 5 | 0 | 0 | 0 |
| - Fault Injection | 6 | 6 | 0 | 0 | 0 |
| **TOTAL** | **180** | **141 (78%)** | **37 (21%)** | **2 (1%)** | **0 (0%)** |

**Priority Distribution:**
- **Critical (141 requirements, 78%)**: Safety-critical, system cannot operate safely without these
- **High (37 requirements, 21%)**: Important for reliability and robustness
- **Medium (2 requirements, 1%)**: Quality improvements
- **Low (0 requirements, 0%)**: N/A for safety subsystem

---

## 10. Acceptance Criteria

### 10.1 Minimum Viable Product (MVP)

**MVP Definition:** Safe autonomous operation with passenger onboard, passing all critical safety tests.

**MVP Requirements:**
- ✅ All Critical requirements (141 total) must be satisfied
- ✅ 100% emergency stop success rate (100 tests)
- ✅ >95% collision avoidance success (500 scenarios)
- ✅ 100% passenger speed limit compliance
- ✅ 100% tilt detection and stop (<20°)
- ✅ All sensor failure tests pass (safe state entry)
- ✅ <300ms brake engagement time
- ✅ <0.7m stopping distance @ 1.0 m/s

**MVP Exclusions:**
- Weather operation (light rain) - can require dry conditions for MVP
- Night operation - can require daylight for MVP
- Advanced degraded modes - basic safe-stop acceptable

---

### 10.2 Production Ready

**Production Definition:** Robust, all-weather, 24/7 operation meeting all critical + high requirements.

**Production Requirements:**
- ✅ All Critical (141) + All High (37) requirements satisfied = 178 total
- ✅ 100% emergency stop success rate (1000 tests)
- ✅ >99% collision avoidance success (10,000 scenarios)
- ✅ Light rain operation (<2.5mm/hr)
- ✅ Night operation with active lighting
- ✅ <200ms brake engagement time
- ✅ <0.5m stopping distance @ 1.0 m/s
- ✅ 1000 hours MTBF (Mean Time Between Failures)
- ✅ Complete fault injection test suite

---

## 11. Traceability

### 11.1 Parent Requirements Traceability

This document derives requirements from:

**SYSTEM_REQUIREMENTS.md (SAFE-REQ):**
| System Requirement | Derived Safety Requirements | Status |
|--------------------|----------------------------|--------|
| SAFE-REQ-002 (Speed limit 1.0 m/s) | SAFE-PASS-SPEED-001, SAFE-SW-VGOV-002 | Traced |
| SAFE-REQ-005 (Tilt detection) | SAFE-PASS-TILT-001 to 006 | Traced |
| SAFE-REQ-006 (Emergency stop) | SAFE-PASS-ESTOP-001 to 006, SAFE-OPS-ESTOP-001 to 006 | Traced |
| SAFE-REQ-007 (Collision avoidance) | SAFE-OPS-COLL-001 to 006 | Traced |
| SAFE-REQ-010 (Slope 10° max) | SAFE-ENV-SLOPE-001 to 005 | Traced |

**All system-level safety requirements traced to subsystem requirements.** ✅

---

## 12. Change History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-15 | Safety Engineer | Initial safety requirements based on SYSTEM_REQUIREMENTS.md and safety best practices |

---

**Document Status:** Draft
**Next Review:** After preliminary safety analysis (FMEA)
**Approvals Required:** Safety Engineer, System Architect, Navigation Lead, Hardware Lead
