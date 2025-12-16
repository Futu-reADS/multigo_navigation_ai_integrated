# Charging Infrastructure Requirements

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Document Type:** Hardware Requirements Specification
**Team Responsibility:** Tsuchiya (Hardware) + Pankaj (Auto-charging Software)
**Status:** Week 5 - Active Development
**Date:** December 16, 2025
**Version:** 1.0

---

## Document Control

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-16 | Multi-Team | Initial charging infrastructure requirements |

**Related Documents:**
- ELECTRICAL_REQUIREMENTS.md (48V battery, BMS, power distribution)
- NAVIGATION_REQUIREMENTS.md (auto-return to charging station)
- DOCKING_SYSTEM_REQUIREMENTS.md (precision docking for charging contacts)
- FLEET_MANAGEMENT_REQUIREMENTS.md (charging schedule optimization)

---

## 1. Purpose and Scope

This document specifies requirements for the **charging infrastructure** of the wheelchair transport robot fleet, including:
- **Charging station hardware** (docking contacts, power delivery)
- **Automated charging workflow** (auto-return, docking, charging, departure)
- **Battery management during charging** (charging profiles, cell balancing)
- **Fleet charging coordination** (queue management, priority)
- **Safety systems** (overcurrent protection, thermal monitoring)

**Design Goals:**
- Fully automated charging (no manual intervention)
- Support for 2-4 vehicles per charging station
- Charge time: 2 hours (20% → 80% SOC)
- Weatherproof outdoor charging (IP54+)

---

## 2. Charging Station Hardware Requirements

### 2.1 Physical Design

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| CHARGE-HW-001 | Charging station SHALL support 2 simultaneous vehicle charging bays minimum | HIGH | 2 bays functional |
| CHARGE-HW-002 | Each charging bay SHALL have footprint 2000mm × 1500mm (vehicle + clearance) | HIGH | Dimensions verified |
| CHARGE-HW-003 | Charging station SHALL be weatherproof (IP54 rating minimum) | CRITICAL | IP54 certification obtained |
| CHARGE-HW-004 | Charging station SHALL have overhead canopy to protect vehicles during charging | MEDIUM | Canopy installed |
| CHARGE-HW-005 | Station floor SHALL be level (slope <2°) for stable vehicle docking | CRITICAL | Floor levelness measured |

### 2.2 Power Supply

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| CHARGE-PWR-001 | Charging station SHALL connect to 200V 3-phase AC power supply | CRITICAL | 3-phase connection verified |
| CHARGE-PWR-002 | Station SHALL provide 48V DC output per bay (20A maximum per vehicle) | CRITICAL | 48V @ 20A output measured |
| CHARGE-PWR-003 | Total station power capacity SHALL be ≥2kW per bay (2 bays = 4kW minimum) | CRITICAL | Power capacity verified |
| CHARGE-PWR-004 | Station SHALL include AC-DC converter (200V AC → 48V DC, 95% efficiency) | HIGH | Efficiency measured |
| CHARGE-PWR-005 | Station SHALL have power factor correction (PF >0.95) | MEDIUM | Power factor measured |
| CHARGE-PWR-006 | Station SHALL support load balancing across bays (distribute available power) | MEDIUM | Load balancing functional |

### 2.3 Charging Contacts

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| CHARGE-CONTACT-001 | Charging contacts SHALL use spring-loaded pogo pins (gold-plated, 10A rated per pin) | CRITICAL | Contacts rated correctly |
| CHARGE-CONTACT-002 | Station SHALL have 4 contact pins: +48V, GND, CAN_H, CAN_L (power + communication) | CRITICAL | 4 pins present and functional |
| CHARGE-CONTACT-003 | Contacts SHALL auto-align with vehicle contacts (±50mm tolerance in X-Y) | CRITICAL | Auto-alignment functional |
| CHARGE-CONTACT-004 | Contacts SHALL withstand 10,000 mating cycles without degradation | HIGH | Durability test passed |
| CHARGE-CONTACT-005 | Contacts SHALL be IP65 rated (dust-tight, water jet resistant) | CRITICAL | IP65 certification obtained |
| CHARGE-CONTACT-006 | Contact resistance SHALL be <10mΩ when mated | HIGH | Resistance measured |

---

## 3. Vehicle Charging Port Requirements

### 3.1 Vehicle-Side Contacts

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| CHARGE-VPORT-001 | Vehicle SHALL have rear-mounted charging port (centered, 500mm height) | CRITICAL | Port location verified |
| CHARGE-VPORT-002 | Charging port SHALL use recessed contacts (protected from rain when not charging) | CRITICAL | Contacts protected |
| CHARGE-VPORT-003 | Vehicle contacts SHALL be spring-loaded (self-cleaning, maintain pressure) | HIGH | Contacts self-cleaning |
| CHARGE-VPORT-004 | Charging port SHALL have LED indicator: off=disconnected, green=charging, blue=fully charged | HIGH | LED indicator functional |
| CHARGE-VPORT-005 | Port SHALL include ArUco marker for precision docking (100mm × 100mm, ID 50) | CRITICAL | ArUco marker detected |

### 3.2 Vehicle Charging Logic

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| CHARGE-VLOGIC-001 | Vehicle SHALL detect charging connection via CAN bus handshake with station | CRITICAL | Connection detection functional |
| CHARGE-VLOGIC-002 | Vehicle SHALL communicate battery status to station: SOC, voltage, temperature, health | CRITICAL | Battery telemetry transmitted |
| CHARGE-VLOGIC-003 | Vehicle SHALL accept charge current only after successful handshake (prevent accidental charging) | CRITICAL | Handshake required |
| CHARGE-VLOGIC-004 | Vehicle SHALL monitor charging safety: overcurrent, overvoltage, overtemperature | CRITICAL | Safety monitoring active |
| CHARGE-VLOGIC-005 | Vehicle SHALL disconnect charging on fault detection (emergency disconnect relay) | CRITICAL | Emergency disconnect functional |

---

## 4. Automated Charging Workflow Requirements

### 4.1 Auto-Return to Charging Station

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| CHARGE-AUTO-001 | Vehicle SHALL automatically navigate to charging station when battery <30% SOC | CRITICAL | Auto-return triggered at 30% |
| CHARGE-AUTO-002 | Vehicle SHALL check charging bay availability before departure (avoid occupied bays) | HIGH | Bay availability check functional |
| CHARGE-AUTO-003 | Vehicle SHALL wait in queue if all bays occupied (park nearby, monitor bay status) | HIGH | Queue management functional |
| CHARGE-AUTO-004 | Vehicle SHALL re-attempt docking if first attempt fails (up to 3 retries) | HIGH | Retry logic functional |
| CHARGE-AUTO-005 | Vehicle SHALL abort charging mission and alert operator after 3 failed docking attempts | CRITICAL | Abort and alert functional |

### 4.2 Precision Docking for Charging

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| CHARGE-DOCK-001 | Vehicle SHALL use ArUco marker on charging port for final docking alignment | CRITICAL | ArUco-based docking functional |
| CHARGE-DOCK-002 | Docking precision SHALL be ±20mm in X-Y, ±5° in yaw (contacts auto-align within ±50mm) | CRITICAL | Docking precision measured |
| CHARGE-DOCK-003 | Vehicle SHALL reverse into charging bay (rear charging port aligns with station contacts) | CRITICAL | Reverse docking functional |
| CHARGE-DOCK-004 | Vehicle SHALL stop movement and engage parking brake after docking | CRITICAL | Parking brake engaged |
| CHARGE-DOCK-005 | Vehicle SHALL verify contact mating via CAN handshake before starting charge | CRITICAL | Handshake verification functional |

### 4.3 Charging Process

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| CHARGE-PROC-001 | Charging SHALL use CC-CV profile: constant current (20A) until 54V, then constant voltage | CRITICAL | CC-CV profile implemented |
| CHARGE-PROC-002 | Charging SHALL terminate when current drops below 1A (full charge indicator) | CRITICAL | Charge termination correct |
| CHARGE-PROC-003 | BMS SHALL perform cell balancing during charging (equalize cell voltages to within 10mV) | HIGH | Cell balancing functional |
| CHARGE-PROC-004 | Charging time SHALL be ≤2 hours for 20% → 80% SOC (typical daily recharge) | HIGH | Charge time measured |
| CHARGE-PROC-005 | Vehicle SHALL remain connected until SOC ≥80% or mission priority requires early departure | HIGH | Charge completion logic correct |
| CHARGE-PROC-006 | Vehicle SHALL transmit charging progress to TVM server every 60 seconds | HIGH | Progress updates transmitted |

### 4.4 Post-Charging Departure

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| CHARGE-DEPART-001 | Vehicle SHALL automatically disconnect from charger when charging complete (release relay) | CRITICAL | Auto-disconnect functional |
| CHARGE-DEPART-002 | Vehicle SHALL navigate out of charging bay (forward motion, clear bay for next vehicle) | CRITICAL | Bay departure functional |
| CHARGE-DEPART-003 | Vehicle SHALL notify TVM server of charging completion and availability | HIGH | Completion notification sent |
| CHARGE-DEPART-004 | Vehicle SHALL accept new mission assignment immediately after departure | HIGH | Mission assignment accepted |

---

## 5. Battery Management During Charging

### 5.1 Charging Safety

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| CHARGE-SAFE-001 | BMS SHALL monitor cell voltages during charging (4.2V maximum per cell, 13S = 54.6V pack) | CRITICAL | Voltage monitoring active |
| CHARGE-SAFE-002 | BMS SHALL monitor cell temperatures during charging (0°C to 45°C allowed) | CRITICAL | Temperature monitoring active |
| CHARGE-SAFE-003 | BMS SHALL stop charging if any cell >4.25V (overvoltage protection) | CRITICAL | Overvoltage protection functional |
| CHARGE-SAFE-004 | BMS SHALL stop charging if any cell >50°C (overtemperature protection) | CRITICAL | Overtemperature protection functional |
| CHARGE-SAFE-005 | BMS SHALL stop charging if charge current >25A (overcurrent protection, 125% of rated) | CRITICAL | Overcurrent protection functional |
| CHARGE-SAFE-006 | Station SHALL have ground fault detection (trip on >30mA leakage current) | CRITICAL | GFCI functional |

### 5.2 Battery Health Monitoring

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| CHARGE-HEALTH-001 | BMS SHALL track battery cycle count (increment on each 20% → 80% charge cycle) | HIGH | Cycle count tracked |
| CHARGE-HEALTH-002 | BMS SHALL estimate battery state-of-health (SOH) based on capacity fade | MEDIUM | SOH estimation functional |
| CHARGE-HEALTH-003 | BMS SHALL log charging history: timestamp, SOC start/end, duration, energy delivered | MEDIUM | Charging history logged |
| CHARGE-HEALTH-004 | System SHALL alert operator when battery SOH <80% (replacement recommended) | MEDIUM | SOH alert functional |

---

## 6. Fleet Charging Coordination Requirements

### 6.1 Charging Queue Management

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| CHARGE-QUEUE-001 | TVM server SHALL maintain charging queue (list of vehicles awaiting charging) | HIGH | Queue management functional |
| CHARGE-QUEUE-002 | Queue SHALL prioritize vehicles by urgency: critical (<15% SOC), high (<30% SOC), normal | CRITICAL | Priority queue functional |
| CHARGE-QUEUE-003 | TVM server SHALL notify next vehicle in queue when bay becomes available | HIGH | Queue notifications sent |
| CHARGE-QUEUE-004 | System SHALL prevent charging conflicts (two vehicles assigned to same bay) | CRITICAL | Conflict prevention functional |

### 6.2 Charging Schedule Optimization

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| CHARGE-SCHED-001 | TVM server SHALL predict vehicle battery depletion based on mission schedule | MEDIUM | Depletion prediction functional |
| CHARGE-SCHED-002 | System SHALL proactively schedule charging during low-demand periods (e.g., lunch break 12-13:00) | MEDIUM | Proactive scheduling functional |
| CHARGE-SCHED-003 | System SHALL allow operator to manually send vehicle to charging (override auto schedule) | HIGH | Manual charging command functional |
| CHARGE-SCHED-004 | System SHALL prevent vehicles from charging during peak demand (if policy configured) | LOW | Peak demand avoidance functional |

---

## 7. Charging Station Monitoring and Diagnostics

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| CHARGE-MON-001 | Charging station SHALL publish status to TVM server: bay occupancy, power output, faults | HIGH | Status published to TVM |
| CHARGE-MON-002 | Station SHALL log charging events: vehicle connected, charge started, charge completed, errors | MEDIUM | Event logging functional |
| CHARGE-MON-003 | Station SHALL measure and report energy consumption per charging session (kWh) | MEDIUM | Energy metering functional |
| CHARGE-MON-004 | Station SHALL display local status on LED panel: bay 1 status, bay 2 status, power available | LOW | LED status display functional |
| CHARGE-MON-005 | Station SHALL support remote diagnostics via Ethernet connection | LOW | Remote diagnostics accessible |

---

## 8. Emergency and Fault Handling

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| CHARGE-FAULT-001 | Station SHALL disconnect power immediately on contact short circuit (within 100ms) | CRITICAL | Short circuit protection functional |
| CHARGE-FAULT-002 | Station SHALL alert operator on charging fault: overcurrent, overvoltage, timeout | CRITICAL | Fault alerts sent |
| CHARGE-FAULT-003 | Vehicle SHALL disconnect from charging on emergency stop button press | CRITICAL | E-stop disconnects charging |
| CHARGE-FAULT-004 | Station SHALL have manual emergency stop button (red mushroom-head, disconnects all power) | CRITICAL | Manual e-stop functional |
| CHARGE-FAULT-005 | System SHALL log all faults with timestamp, error code, vehicle ID, bay number | HIGH | Fault logging functional |

---

## 9. Charging Station Installation Requirements

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| CHARGE-INSTALL-001 | Station SHALL be installed on concrete pad (level, load-bearing) | CRITICAL | Concrete pad installed |
| CHARGE-INSTALL-002 | Station SHALL be anchored to pad (resist 500N horizontal force, prevent tipping) | HIGH | Anchoring tested |
| CHARGE-INSTALL-003 | Electrical installation SHALL comply with local electrical codes (e.g., NEC, IEC) | CRITICAL | Code compliance verified |
| CHARGE-INSTALL-004 | Station SHALL be installed in accessible location (vehicle approach path clear) | HIGH | Location accessibility verified |
| CHARGE-INSTALL-005 | Station SHALL have adequate drainage (prevent water pooling around contacts) | MEDIUM | Drainage functional |

---

## 10. Charging Performance Requirements

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| CHARGE-PERF-001 | Charging efficiency SHALL be ≥90% (AC input to battery) | HIGH | Efficiency measured |
| CHARGE-PERF-002 | Charging time 20% → 80% SOC SHALL be ≤2 hours (960Wh capacity, 20A @ 48V) | HIGH | Charge time verified |
| CHARGE-PERF-003 | Docking success rate SHALL be ≥95% (first attempt) | HIGH | Docking success rate measured |
| CHARGE-PERF-004 | Contact mating reliability SHALL be ≥99% (successful electrical connection) | CRITICAL | Mating reliability verified |
| CHARGE-PERF-005 | Station uptime SHALL be ≥99% (excluding scheduled maintenance) | MEDIUM | Uptime measured |

---

## 11. Future Expansion Requirements

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| CHARGE-FUTURE-001 | Station design SHALL support expansion to 4 bays (modular bay addition) | LOW | Expansion capability verified |
| CHARGE-FUTURE-002 | Station SHALL reserve space for future solar panel integration (roof-mounted) | LOW | Solar panel mounting points present |
| CHARGE-FUTURE-003 | Station SHALL support future fast-charging upgrade (20A → 40A per bay) | LOW | Power supply upgradeable |

---

## 12. Requirements Summary

| Category | Count | Priority Breakdown |
|----------|-------|-------------------|
| Charging Station Hardware | 16 | Critical: 7, High: 6, Medium: 3 |
| Vehicle Charging Port | 10 | Critical: 7, High: 3 |
| Automated Charging Workflow | 19 | Critical: 13, High: 6 |
| Battery Management During Charging | 10 | Critical: 7, High: 2, Medium: 1 |
| Fleet Charging Coordination | 7 | Critical: 2, High: 3, Medium: 2 |
| Charging Station Monitoring | 5 | High: 1, Medium: 3, Low: 1 |
| Emergency and Fault Handling | 5 | Critical: 4, High: 1 |
| Installation Requirements | 5 | Critical: 2, High: 2, Medium: 1 |
| Performance Requirements | 5 | Critical: 1, High: 3, Medium: 1 |
| Future Expansion | 3 | Low: 3 |
| **TOTAL** | **85** | **Critical: 43, High: 27, Medium: 12, Low: 3** |

---

## 13. Charging Station System Diagram

```
┌─────────────────────────────────────────────────────────┐
│            Charging Station (2 Bays)                    │
│                                                         │
│  200V 3-Phase AC Input                                  │
│         │                                               │
│         ▼                                               │
│  ┌──────────────┐                                       │
│  │  AC-DC Conv. │  (200V AC → 48V DC, 4kW total)       │
│  │  + Load Bal. │                                       │
│  └──────┬───────┘                                       │
│         │                                               │
│    ┌────┴────┐                                          │
│    │         │                                          │
│    ▼         ▼                                          │
│ ┌─────┐   ┌─────┐                                       │
│ │Bay 1│   │Bay 2│  (48V @ 20A each, 2kW per bay)       │
│ │     │   │     │                                       │
│ │ 🔌  │   │ 🔌  │  (Pogo pin contacts: +48V, GND,      │
│ │     │   │     │   CAN_H, CAN_L)                       │
│ └─────┘   └─────┘                                       │
│   │         │                                           │
│   │ ArUco   │ ArUco                                     │
│   │ Marker  │ Marker                                    │
│   │         │                                           │
└───┼─────────┼───────────────────────────────────────────┘
    │         │
    ▼         ▼
 Vehicle   Vehicle
 (Rear     (Rear
  Port)     Port)
```

---

## 14. Traceability Matrix

| Charging Requirement Category | Related System Components |
|------------------------------|---------------------------|
| Station Hardware | AC-DC converter, power distribution, contact assembly |
| Vehicle Port | BMS, charging relay, CAN bus, ArUco marker |
| Automated Workflow | Navigation controller, docking controller, fleet management server |
| Battery Management | BMS, temperature sensors, voltage sensors, current sensors |

---

## 15. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-16 | Multi-Team | Initial charging infrastructure requirements (85 requirements) |

---

**Document End**
