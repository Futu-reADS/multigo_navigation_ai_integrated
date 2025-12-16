# Electrical System Architecture

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Document Type:** Hardware Architecture Specification
**Team:** Tsuchiya (Hardware)
**Date:** December 16, 2025
**Version:** 1.0

---

## Power Distribution Architecture

```
┌────────────────────────────────────────────────────────────┐
│                  48V LiFePO4 Battery                      │
│                  (13S, 20Ah, 960Wh)                       │
│                       + BMS                                │
└──────────────────┬─────────────────────────────────────────┘
                   │
        ┌──────────┴──────────────┐
        │                         │
   ┌────▼────┐              ┌────▼────┐
   │ 48V Bus │              │Emergency│
   │(Motors) │              │  Relay  │
   └────┬────┘              └────┬────┘
        │                        │
   ┌────┴───────┬───────────┬───┴───────────┐
   │            │           │                │
┌──▼──┐    ┌──▼──┐    ┌──▼──┐         ┌───▼────┐
│Motor│    │Motor│    │Motor│         │DC-DC   │
│Ctrl │    │Ctrl │    │Ctrl │   ...   │48V→24V │
│×8   │    │     │    │     │         │(10A)   │
└─────┘    └─────┘    └─────┘         └───┬────┘
                                           │
                                      ┌────▼────┐
                                      │DC-DC    │
                                      │24V→12V  │
                                      │(15A)    │
                                      └───┬─────┘
                                          │
                                     ┌────▼────┐
                                     │DC-DC    │
                                     │12V→5V   │
                                     │(10A)    │
                                     └────┬────┘
                                          │
                         ┌────────────────┼────────────┐
                         │                │            │
                    ┌────▼─────┐    ┌────▼────┐  ┌───▼────┐
                    │Jetson    │    │Sensors  │  │USB     │
                    │Orin Nano │    │(LiDAR,  │  │Periph. │
                    │(12V/5V)  │    │Cameras) │  │        │
                    └──────────┘    └─────────┘  └────────┘
```

## Voltage Rails

| Rail | Source | Current Capacity | Consumers | Purpose |
|------|--------|------------------|-----------|---------|
| 48V | Battery | 30A continuous, 60A peak | 8× Motor controllers | Swerve drive propulsion |
| 24V | 48V→24V DC-DC | 10A | eHMI ESP32, exterior lights | External systems |
| 12V | 24V→12V DC-DC | 15A | Jetson Orin Nano, LiDAR, cameras | Compute and sensors |
| 5V | 12V→5V DC-DC | 10A | USB peripherals, touchscreen | Low-power devices |

## Communication Buses

### CAN Bus Network

**CAN0 (High-speed: 500 kbit/s)**
- 8× Motor controllers (drive + steer)
- Cycle time: 10ms
- Termination: 120Ω at both ends

**CAN1 (Medium-speed: 250 kbit/s)**
- Battery Management System (BMS)
- IMU sensor
- Safety controller
- Cycle time: 100ms

### UART Connections

| UART Port | Baud Rate | Connection | Purpose |
|-----------|-----------|------------|---------|
| UART0 | 115200 | Jetson ↔ ESP32 (eHMI) | External display/audio control |
| UART1 | 115200 | Reserved (GPS future) | Future expansion |

### Ethernet

- Jetson Orin Nano ↔ Router (1 Gbps)
- Router ↔ WiFi AP
- Router ↔ LTE modem

## Safety Circuits

### Emergency Stop Chain

```
Physical E-Stop Button (NC) ──┬── UI E-Stop Command
                               │
                               ▼
                        Safety Controller
                               │
                               ▼
                        Emergency Relay
                     (Disconnects 48V to motors)
                               │
                               ▼
                        Motors Stop (<100ms)
```

### Overcurrent Protection

- Main 48V bus: 60A circuit breaker
- 24V rail: 12A fuse
- 12V rail: 18A fuse
- 5V rail: 12A fuse

### Thermal Management

- BMS temperature monitoring (0°C to 45°C operating range)
- Motor controller temperature limits (85°C max)
- Forced-air cooling for compute unit (40mm fan, 12V)

## Connectors and Wiring

### Main Power Connectors

- Battery: Anderson PowerPole 75A
- Motor controllers: XT60 connectors
- DC-DC converters: Molex Mini-Fit Jr.

### Signal Connectors

- CAN bus: JST-XH 4-pin (CAN_H, CAN_L, GND, +5V)
- UART: JST-XH 4-pin (TX, RX, GND, +3.3V)
- Emergency stop: M12 4-pin connector (IP67)

### Wiring Specifications

- 48V power: 10 AWG (5.26mm²) silicone wire, 40A rated
- 24V power: 14 AWG (2.08mm²) 
- 12V power: 16 AWG (1.31mm²)
- CAN bus: Twisted pair, shielded, 120Ω impedance
- All wiring: Flame-retardant, automotive-grade

## PCB Designs

### Safety Controller PCB

- Microcontroller: STM32F4 (automotive-grade)
- Inputs: E-stop button, software e-stop command
- Outputs: Emergency relay control, status LEDs
- Watchdog timer: 500ms timeout
- Redundant power supply (24V + 12V)

### Power Distribution PCB

- Fuse holders for all rails
- Voltage monitoring (ADC)
- Current sensing (Hall-effect sensors)
- Status LEDs for each rail

## Grounding and EMC

- Single-point grounding at battery negative terminal
- Chassis ground isolated from battery ground
- Shielded cables for all high-frequency signals (CAN, Ethernet)
- Ferrite beads on power rails (suppress EMI)
- Star grounding topology

## IP Ratings

| Component | IP Rating | Protection |
|-----------|-----------|------------|
| Battery enclosure | IP65 | Dust-tight, water jet resistant |
| Motor controllers | IP67 | Dust-tight, temporary immersion |
| Main compute enclosure | IP54 | Dust protected, splash resistant |
| Connectors (external) | IP67 | Waterproof sealed connectors |

---

**Total:** 61 requirements covered in ELECTRICAL_REQUIREMENTS.md  
**Related:** COMMUNICATION_SYSTEM_REQUIREMENTS.md (83 req)

**Document End**
