# Communication System Requirements

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Document Type:** Hardware Requirements Specification
**Team Responsibility:** Tsuchiya (Hardware) + Pankaj (Vehicle Software Integration)
**Status:** Week 4 - Active Development
**Date:** December 16, 2025
**Version:** 1.0

---

## Document Control

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-16 | Multi-Team | Initial communication system requirements |

**Related Documents:**
- ELECTRICAL_REQUIREMENTS.md (power supply for comm modules)
- TVM_API_SPECIFICATION.md (fleet communication protocols)
- HARDWARE_SOFTWARE_INTERFACES.md (ROS 2 topic mappings)
- INTERFACE_REQUIREMENTS.md (vehicle-level protocols)

---

## 1. Purpose and Scope

This document specifies requirements for all communication subsystems in the outdoor wheelchair transport robot, including:
- **CAN bus** (motor controllers, BMS)
- **UART** (eHMI ESP32, sensors)
- **ROS 2 DDS** (internal software communication)
- **WiFi 802.11ac** (TVM fleet management, updates)
- **4G LTE** (remote teleoperation, fallback connectivity)

**Design Principles:**
- Multi-layer redundancy (WiFi primary, 4G backup)
- Real-time deterministic communication for safety-critical data (CAN, UART)
- Secure encrypted channels for fleet management (TLS 1.3)
- Graceful degradation on link failure

---

## 2. CAN Bus Communication Requirements

### 2.1 CAN Bus Architecture

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| COMM-CAN-001 | System SHALL implement two CAN buses: CAN0 (motors) at 500 kbit/s, CAN1 (BMS/sensors) at 250 kbit/s | CRITICAL | Bus speeds verified with oscilloscope |
| COMM-CAN-002 | CAN0 SHALL connect 8 motor controllers (4 drive + 4 steer) with 10ms cycle time | CRITICAL | All 8 controllers respond within 10ms |
| COMM-CAN-003 | CAN1 SHALL connect BMS, IMU, and auxiliary sensors with 100ms cycle time | HIGH | Sensor data received at ≥10Hz |
| COMM-CAN-004 | System SHALL use CAN-FD (Flexible Data-rate) for motor controllers | HIGH | Data payload >8 bytes supported |
| COMM-CAN-005 | System SHALL implement CAN bus termination (120Ω resistors at both ends) | CRITICAL | Bus signal quality verified |
| COMM-CAN-006 | System SHALL monitor CAN bus errors and log to diagnostics | HIGH | Error frames logged with timestamp |
| COMM-CAN-007 | System SHALL support hot-plug detection for CAN devices | MEDIUM | Device connect/disconnect detected <1s |

**Success Criteria:** Zero CAN bus errors in 100-hour endurance test

### 2.2 CAN Message Definitions

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| COMM-CAN-MSG-001 | Motor command message SHALL include: motor_id, target_velocity, target_angle, torque_limit | CRITICAL | Message format validated |
| COMM-CAN-MSG-002 | Motor status message SHALL include: motor_id, current_velocity, current_angle, current_draw, temperature, faults | CRITICAL | All fields decoded correctly |
| COMM-CAN-MSG-003 | BMS status message SHALL include: voltage, current, SOC, SOH, cell_temps, faults | CRITICAL | BMS data matches specifications |
| COMM-CAN-MSG-004 | Emergency stop message SHALL broadcast on CAN ID 0x001 (highest priority) | CRITICAL | Emergency stop propagates <10ms |
| COMM-CAN-MSG-005 | System SHALL use extended CAN IDs (29-bit) for future scalability | MEDIUM | Extended ID format supported |

**Success Criteria:** 100% message delivery rate for safety-critical messages

### 2.3 CAN Bus Error Handling

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| COMM-CAN-ERR-001 | System SHALL detect CAN bus-off condition and attempt auto-recovery | CRITICAL | Recovery attempted within 1s |
| COMM-CAN-ERR-002 | System SHALL enter safe mode on persistent CAN failure (>10 errors/sec for 5s) | CRITICAL | Safe mode engaged within 5s |
| COMM-CAN-ERR-003 | System SHALL log all CAN error frames with timestamp and error type | HIGH | Error log includes frame dumps |
| COMM-CAN-ERR-004 | System SHALL provide diagnostic LED indicators for CAN bus status (green=OK, red=error) | MEDIUM | LED status matches bus state |

---

## 3. UART Communication Requirements

### 3.1 UART Configuration

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| COMM-UART-001 | System SHALL implement UART at 115200 baud, 8N1 (8 data bits, no parity, 1 stop bit) | CRITICAL | UART configuration verified |
| COMM-UART-002 | eHMI ESP32 SHALL communicate with Jetson via UART0 at 115200 baud | CRITICAL | Bidirectional communication verified |
| COMM-UART-003 | System SHALL support additional UART ports for GPS (future), external sensors | MEDIUM | UART1/2 configurable |
| COMM-UART-004 | UART messages SHALL use checksum validation (CRC-16) | HIGH | Invalid checksums rejected |
| COMM-UART-005 | System SHALL implement flow control (RTS/CTS) for UART connections | MEDIUM | No data loss under high load |

**Success Criteria:** Zero data corruption in 1M UART message exchanges

### 3.2 UART Message Protocol (eHMI)

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| COMM-UART-EHMI-001 | eHMI command message SHALL include: command_type, led_pattern, buzzer_tone, display_text | CRITICAL | Commands executed correctly |
| COMM-UART-EHMI-002 | eHMI status message SHALL include: button_state, emergency_stop_pressed, battery_display | CRITICAL | Status updates received at 10Hz |
| COMM-UART-EHMI-003 | Emergency stop button press SHALL send immediate message (within 10ms) | CRITICAL | Emergency stop latency <10ms |
| COMM-UART-EHMI-004 | System SHALL support firmware update for ESP32 via UART bootloader | MEDIUM | Firmware update successful |

---

## 4. ROS 2 DDS Communication Requirements

### 4.1 ROS 2 Configuration

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| COMM-ROS2-001 | System SHALL use ROS 2 Humble with Cyclone DDS as default middleware | CRITICAL | ROS 2 nodes communicate reliably |
| COMM-ROS2-002 | System SHALL configure DDS QoS profiles: RELIABLE for commands, BEST_EFFORT for sensor data | CRITICAL | QoS profiles applied correctly |
| COMM-ROS2-003 | System SHALL use ROS 2 domain ID 42 to isolate from other robots | HIGH | No cross-talk with other systems |
| COMM-ROS2-004 | System SHALL limit DDS discovery traffic to local subnet (TTL=1) | MEDIUM | No discovery packets leave subnet |
| COMM-ROS2-005 | System SHALL use FastDDS for low-latency critical paths (docking, safety) | HIGH | DDS latency <5ms for critical topics |

**Success Criteria:** 99.99% message delivery for RELIABLE topics

### 4.2 ROS 2 Topic Design

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| COMM-ROS2-TOPIC-001 | Safety-critical topics SHALL use RELIABLE QoS with history depth ≥10 | CRITICAL | No message loss in failure scenarios |
| COMM-ROS2-TOPIC-002 | Sensor topics SHALL use BEST_EFFORT QoS for low latency | HIGH | Sensor latency <20ms |
| COMM-ROS2-TOPIC-003 | Command topics SHALL use transient_local durability for late-joining nodes | MEDIUM | Late joiners receive last command |
| COMM-ROS2-TOPIC-004 | System SHALL publish diagnostics to /diagnostics topic at 1Hz | HIGH | Diagnostics available system-wide |

---

## 5. WiFi Communication Requirements

### 5.1 WiFi Hardware and Configuration

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| COMM-WIFI-001 | System SHALL use WiFi 802.11ac (5GHz) dual-band adapter | CRITICAL | 5GHz connectivity verified |
| COMM-WIFI-002 | WiFi adapter SHALL support 2×2 MIMO for 866 Mbps max throughput | HIGH | Throughput ≥400 Mbps achieved |
| COMM-WIFI-003 | System SHALL connect to TVM fleet management server via WiFi (primary link) | CRITICAL | TVM connection established <10s |
| COMM-WIFI-004 | System SHALL auto-reconnect on WiFi disconnection with exponential backoff | CRITICAL | Reconnection within 30s |
| COMM-WIFI-005 | System SHALL roam between access points seamlessly (802.11r Fast Roaming) | HIGH | Roaming handoff <100ms |
| COMM-WIFI-006 | System SHALL fall back to 2.4GHz band if 5GHz unavailable | MEDIUM | Band fallback automatic |

**Success Criteria:** WiFi uptime >99% in operational areas

### 5.2 WiFi Security

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| COMM-WIFI-SEC-001 | WiFi connection SHALL use WPA3-Enterprise with 802.1X authentication | CRITICAL | WPA3 encryption verified |
| COMM-WIFI-SEC-002 | System SHALL validate TVM server certificate (TLS 1.3) before sending data | CRITICAL | Invalid certs rejected |
| COMM-WIFI-SEC-003 | System SHALL encrypt all TVM communication with AES-256 | CRITICAL | Packet sniffing shows encryption |
| COMM-WIFI-SEC-004 | System SHALL use separate WiFi credentials per vehicle (no shared passwords) | HIGH | Each vehicle has unique credentials |

### 5.3 WiFi Data Transmission

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| COMM-WIFI-DATA-001 | System SHALL send telemetry to TVM server every 5 seconds (position, battery, status) | CRITICAL | Telemetry received at 0.2Hz |
| COMM-WIFI-DATA-002 | System SHALL send logs to TVM server every 60 seconds (compressed) | MEDIUM | Logs uploaded successfully |
| COMM-WIFI-DATA-003 | System SHALL receive mission updates from TVM server (goals, reservations) | CRITICAL | Mission updates processed <1s |
| COMM-WIFI-DATA-004 | System SHALL support remote diagnostics access via WiFi (SSH over VPN) | MEDIUM | SSH connection successful |
| COMM-WIFI-DATA-005 | System SHALL limit WiFi bandwidth usage to 10 Mbps average to prevent congestion | MEDIUM | Bandwidth cap enforced |

---

## 6. 4G LTE Communication Requirements

### 6.1 LTE Hardware and Configuration

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| COMM-LTE-001 | System SHALL include 4G LTE modem (Cat 4, 150 Mbps down / 50 Mbps up) | HIGH | LTE modem detected by system |
| COMM-LTE-002 | LTE modem SHALL support multi-carrier SIM (automatic carrier selection) | HIGH | SIM connects to available networks |
| COMM-LTE-003 | System SHALL use LTE as backup link when WiFi unavailable | CRITICAL | Automatic failover to LTE <30s |
| COMM-LTE-004 | System SHALL reconnect to WiFi when available (WiFi preferred over LTE) | HIGH | Failback to WiFi within 60s |
| COMM-LTE-005 | System SHALL monitor LTE signal strength (RSSI) and switch to WiFi if LTE <-100 dBm | MEDIUM | Signal monitoring functional |

**Success Criteria:** LTE failover successful in 100% of WiFi outage scenarios

### 6.2 LTE Data Management

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| COMM-LTE-DATA-001 | System SHALL limit LTE data usage to 1 GB/month (avoid excessive charges) | HIGH | Data usage tracked and capped |
| COMM-LTE-DATA-002 | System SHALL send critical telemetry only over LTE (position, emergency alerts) | CRITICAL | Non-critical data suppressed on LTE |
| COMM-LTE-DATA-003 | System SHALL support remote emergency stop over LTE | CRITICAL | E-stop command received <2s |
| COMM-LTE-DATA-004 | System SHALL support remote teleoperation over LTE (low resolution video, 1 fps) | MEDIUM | Teleoperation functional over LTE |

### 6.3 LTE Security

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| COMM-LTE-SEC-001 | LTE connection SHALL use VPN tunnel (WireGuard) to TVM server | CRITICAL | VPN encryption verified |
| COMM-LTE-SEC-002 | System SHALL authenticate TVM server before establishing LTE VPN | CRITICAL | Invalid servers rejected |
| COMM-LTE-SEC-003 | System SHALL use SIM PIN lock to prevent unauthorized SIM usage | MEDIUM | PIN lock enabled |

---

## 7. Network Failover and Redundancy

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| COMM-FAILOVER-001 | System SHALL automatically detect WiFi link failure within 10 seconds | CRITICAL | Failure detection time <10s |
| COMM-FAILOVER-002 | System SHALL switch from WiFi to LTE within 30 seconds of WiFi failure | CRITICAL | Failover time <30s |
| COMM-FAILOVER-003 | System SHALL buffer telemetry data during network outage (up to 1 hour) | HIGH | Buffer holds 720 telemetry packets |
| COMM-FAILOVER-004 | System SHALL upload buffered data when connectivity restored | HIGH | All buffered data uploaded |
| COMM-FAILOVER-005 | System SHALL operate autonomously during network outage (use cached map, goals) | CRITICAL | Vehicle completes mission offline |
| COMM-FAILOVER-006 | System SHALL notify TVM server of connectivity issues via status flags | MEDIUM | Connectivity status reported |

**Success Criteria:** Zero mission failures due to temporary network outages

---

## 8. Inter-Vehicle Communication (V2V) - Future

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| COMM-V2V-001 | System SHALL reserve hardware interfaces for future V2V communication (802.11p or C-V2X) | LOW | Hardware support available |
| COMM-V2V-002 | System SHALL support broadcasting position/velocity to nearby vehicles (collision avoidance) | LOW | V2V messages transmitted |
| COMM-V2V-003 | V2V communication SHALL use DSRC 5.9 GHz band (dedicated short-range) | LOW | DSRC frequency configured |

---

## 9. Communication Performance Requirements

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| COMM-PERF-001 | CAN bus latency SHALL be <10ms for motor commands | CRITICAL | Oscilloscope measurement <10ms |
| COMM-PERF-002 | UART latency SHALL be <10ms for eHMI emergency stop | CRITICAL | Emergency stop latency <10ms |
| COMM-PERF-003 | ROS 2 DDS latency SHALL be <20ms for sensor data topics | HIGH | DDS latency measured <20ms |
| COMM-PERF-004 | WiFi round-trip time to TVM server SHALL be <100ms | HIGH | Ping RTT <100ms |
| COMM-PERF-005 | LTE round-trip time to TVM server SHALL be <500ms | MEDIUM | Ping RTT <500ms |
| COMM-PERF-006 | Total communication stack CPU usage SHALL be <10% on Jetson Orin Nano | MEDIUM | CPU profiling shows <10% |

---

## 10. Communication Diagnostics and Monitoring

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| COMM-DIAG-001 | System SHALL publish communication diagnostics to ROS 2 /diagnostics topic | HIGH | Diagnostics available |
| COMM-DIAG-002 | Diagnostics SHALL include: CAN error count, UART error count, WiFi signal strength, LTE signal strength, packet loss | HIGH | All metrics reported |
| COMM-DIAG-003 | System SHALL log communication errors to persistent storage | MEDIUM | Error logs saved |
| COMM-DIAG-004 | System SHALL provide LED indicators: WiFi status (blue), LTE status (green), error (red) | LOW | LEDs match comm status |

---

## 11. Communication Safety Requirements

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| COMM-SAFE-001 | System SHALL operate safely during total communication failure (WiFi + LTE down) | CRITICAL | Vehicle stops safely |
| COMM-SAFE-002 | System SHALL validate all received commands with checksum/CRC | CRITICAL | Corrupted commands rejected |
| COMM-SAFE-003 | System SHALL timeout stale commands (e.g., motor commands >100ms old) | CRITICAL | Stale commands ignored |
| COMM-SAFE-004 | System SHALL implement watchdog timer for communication stack (reset on hang) | HIGH | Watchdog triggers on hang |

---

## 12. Communication Testing Requirements

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| COMM-TEST-001 | Test CAN bus communication with all 8 motor controllers under load | CRITICAL | 100% message delivery |
| COMM-TEST-002 | Test UART communication with 1M message exchanges (no errors) | CRITICAL | Zero errors in 1M messages |
| COMM-TEST-003 | Test WiFi failover to LTE in <30s | CRITICAL | Failover time verified |
| COMM-TEST-004 | Test network outage resilience (1-hour offline operation) | HIGH | Vehicle operates offline |
| COMM-TEST-005 | Test communication stack under EMI conditions (near high-voltage lines) | MEDIUM | No errors under EMI |

---

## 13. Requirements Summary

| Category | Count | Priority Breakdown |
|----------|-------|-------------------|
| CAN Bus Communication | 12 | Critical: 8, High: 3, Medium: 1 |
| UART Communication | 9 | Critical: 6, High: 1, Medium: 2 |
| ROS 2 DDS Communication | 8 | Critical: 4, High: 3, Medium: 1 |
| WiFi Communication | 15 | Critical: 10, High: 3, Medium: 2 |
| 4G LTE Communication | 11 | Critical: 6, High: 4, Medium: 1 |
| Network Failover | 6 | Critical: 4, High: 2, Medium: 0 |
| V2V Communication (Future) | 3 | Low: 3 |
| Performance | 6 | Critical: 2, High: 3, Medium: 1 |
| Diagnostics | 4 | High: 2, Medium: 1, Low: 1 |
| Safety | 4 | Critical: 3, High: 1 |
| Testing | 5 | Critical: 3, High: 1, Medium: 1 |
| **TOTAL** | **83** | **Critical: 51, High: 23, Medium: 6, Low: 3** |

---

## 14. Traceability Matrix

| Requirement Category | Related System Components |
|---------------------|---------------------------|
| CAN Bus | Swerve Drive Controllers, BMS, Motor Controllers |
| UART | eHMI ESP32, Future GPS Module |
| ROS 2 DDS | All ROS 2 Nodes (Navigation, Perception, Docking, Safety) |
| WiFi | TVM Client, Fleet Management Server, Remote Diagnostics |
| 4G LTE | TVM Client, Remote Teleoperation, Emergency Connectivity |

---

## 15. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-16 | Multi-Team | Initial communication system requirements (83 requirements) |

---

**Document End**
