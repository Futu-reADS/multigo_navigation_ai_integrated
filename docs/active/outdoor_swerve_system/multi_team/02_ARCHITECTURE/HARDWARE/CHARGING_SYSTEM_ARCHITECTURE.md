# Charging System Architecture

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Document Type:** Hardware Architecture Specification
**Team:** Tsuchiya (Hardware) + Pankaj (Auto-Charging Software)
**Date:** December 16, 2025
**Version:** 1.0

---

## System Overview

```
┌─────────────────────────────────────────────────────────┐
│           Charging Station (Fixed Infrastructure)       │
│                                                         │
│  200V 3-Phase AC Input                                  │
│         │                                               │
│         ▼                                               │
│  ┌──────────────────┐                                   │
│  │  AC-DC Converter │  (2 kW per bay)                   │
│  │  200V AC → 48V DC│                                   │
│  │  (Isolated)      │                                   │
│  └────────┬─────────┘                                   │
│           │                                             │
│      ┌────┴────┐                                        │
│      │         │                                        │
│   ┌──▼───┐ ┌──▼───┐                                    │
│   │Bay 1 │ │Bay 2 │ (48V @ 20A each)                   │
│   │      │ │      │                                    │
│   │  🔌  │ │  🔌  │ Spring-loaded pogo pins            │
│   │ +48V │ │ +48V │ (4 pins: +48V, GND, CAN_H, CAN_L) │
│   │ GND  │ │ GND  │                                    │
│   │CAN_H │ │CAN_H │                                    │
│   │CAN_L │ │CAN_L │                                    │
│   └──────┘ └──────┘                                    │
│      │         │                                        │
│   [ArUco]  [ArUco] (Docking alignment markers)         │
└──────┼─────────┼───────────────────────────────────────┘
       │         │
       ▼         ▼
    Vehicle   Vehicle
    (Rear     (Rear
     Port)     Port)
```

## Vehicle Charging Port

### Physical Design

```
Rear View of Robot

     ┌────────────────────┐
     │   Robot Chassis    │
     │                    │
     │  ┌──────────────┐  │
     │  │ Charging Port│  │
     │  │  (Recessed)  │  │
     │  │              │  │
     │  │  ●  ●        │  │ ← 4 spring-loaded contacts
     │  │              │  │   (+48V, GND, CAN_H, CAN_L)
     │  │  ●  ●        │  │
     │  │              │  │
     │  │ [ArUco ID50] │  │ ← 100mm × 100mm marker
     │  └──────────────┘  │
     │        500mm height│
     └────────────────────┘
```

**Contacts:**
- Material: Gold-plated copper (corrosion-resistant)
- Type: Spring-loaded pogo pins, 10A rated per pin
- Travel: 10mm compression range
- Resistance: <10mΩ when mated
- IP rating: IP65 (sealed when not charging)

**ArUco Marker:**
- Size: 100mm × 100mm
- ID: 50 (unique to charging port)
- Material: UV-resistant vinyl
- Reflectivity: Retroreflective coating (nighttime visibility)

### Electrical Interface

**Power Pins:**
- Pin 1: +48V (charge positive)
- Pin 2: GND (charge negative)
- Current capacity: 20A continuous

**Communication Pins:**
- Pin 3: CAN_H (ISO 11898 CAN bus)
- Pin 4: CAN_L
- Baud rate: 250 kbit/s
- Purpose: Charging handshake, BMS telemetry

## Charging Station Design

### AC-DC Converter

**Specifications:**
- Input: 200V 3-phase AC, 50/60 Hz
- Output: 48V DC, 2 kW per bay (20A × 2 bays = 40A total)
- Efficiency: ≥95%
- Power Factor: >0.95 (PFC circuit)
- Protection: Overcurrent, overvoltage, short-circuit

**Safety Features:**
- Ground fault detection (<30mA trip)
- Isolation transformer (input-output isolation)
- Emergency stop button (disconnects all power)

### Pogo Pin Contacts (Station Side)

**Specifications:**
- Force: 200g per pin (maintain contact pressure)
- Plating: Gold-plated (10μm thickness)
- Durability: 10,000 mating cycles
- Alignment tolerance: ±50mm in X-Y plane
- Auto-alignment: Tapered guide funnels

### Weatherproofing

- Contacts: IP65 sealed housing
- Cables: IP67 connectors, strain relief
- Enclosure: IP54 (outdoor canopy overhead)
- Drainage: Sloped contact holder (water runoff)

## Charging Protocol

### Handshake Sequence (CAN Bus)

1. **Vehicle connects** (physical contact detected via voltage sensing)
2. **Station sends:** `CHARGING_READY` message (CAN ID 0x100)
3. **Vehicle responds:** `BMS_STATUS` (SOC, voltage, temp, health)
4. **Station validates:** Check battery status acceptable for charging
5. **Station enables:** Close charging relay, begin CC-CV charging
6. **Continuous monitoring:** Vehicle sends `BMS_STATUS` every 1s
7. **Charging complete:** Current <1A → Station sends `CHARGING_COMPLETE`
8. **Vehicle disconnects:** Open relay, navigate away from bay

### CAN Message Definitions

**CHARGING_READY (Station → Vehicle)**
```
CAN ID: 0x100
Data: [0x01, bay_id, max_current (A), reserved...]
```

**BMS_STATUS (Vehicle → Station)**
```
CAN ID: 0x101
Data: [SOC (%), voltage_high, voltage_low, temp (°C), current_high, current_low, flags]
flags: bit 0 = charging_enabled, bit 1 = temp_ok, bit 2 = voltage_ok
```

**CHARGING_COMPLETE (Station → Vehicle)**
```
CAN ID: 0x102
Data: [0x01, bay_id, energy_delivered_kWh, duration_min]
```

### CC-CV Charging Profile

**Constant Current (CC) Phase:**
- Current: 20A (0.5C for 40Ah battery)
- Voltage limit: 54.6V (4.2V per cell × 13S)
- Duration: ~1.5 hours (20% → 80% SOC)

**Constant Voltage (CV) Phase:**
- Voltage: 54.6V
- Current tapers: 20A → 1A
- Termination: Current <1A (charge complete)
- Duration: ~30 minutes

**Total Charging Time:** ~2 hours (20% → 80% SOC)

## Automated Docking

### ArUco-Based Precision Docking

1. **Approach:** Vehicle navigates to charging station (GPS/NDT localization)
2. **Coarse alignment:** Position within 2m of bay, facing backward
3. **ArUco detection:** Rear camera detects ArUco marker on bay
4. **Fine alignment:** Visual servoing to align vehicle port with station contacts
5. **Final docking:** Reverse until contacts mate (±20mm precision)
6. **Confirmation:** Voltage sensing + CAN handshake confirms connection

### Alignment Tolerance

- Required docking precision: ±20mm (X-Y plane), ±5° (yaw)
- Contact auto-alignment: ±50mm (spring-loaded contacts compensate)
- Success rate target: ≥95% first-attempt docking

## Charging Bay Layout

### Physical Dimensions

```
Top View

Bay 1              Bay 2
┌──────┐          ┌──────┐
│      │          │      │
│  🔌  │   2m     │  🔌  │
│ArUco │          │ArUco │
└──────┘          └──────┘
   ↑                 ↑
  1.5m             1.5m
   ↓                 ↓
Vehicle           Vehicle
approach          approach
zone              zone
```

- Bay width: 1.5m (vehicle width 0.9m + 0.6m clearance)
- Bay depth: 2m (vehicle length 1.2m + 0.8m clearance)
- Bay separation: 2m (prevent cross-traffic)

### Station Enclosure

- Material: Powder-coated steel frame, polycarbonate roof
- Dimensions: 5m (W) × 3m (D) × 3m (H)
- Weather protection: Overhead canopy (protects vehicles during charging)
- Lighting: LED strips (nighttime visibility)
- Mounting: Concrete pad, anchor bolts

## Power Management

### Load Balancing

- Total station capacity: 4 kW (2 bays × 2 kW)
- If both bays occupied: 2 kW per bay (standard charge rate)
- If one bay occupied: 2 kW (full rate, or 3 kW fast-charge if supported)

### Energy Metering

- Measure energy delivered per session (kWh)
- Store in station controller (log to TVM server)
- Used for maintenance planning and cost accounting

---

**Total:** 85 requirements covered in CHARGING_INFRASTRUCTURE_REQUIREMENTS.md

**Document End**
