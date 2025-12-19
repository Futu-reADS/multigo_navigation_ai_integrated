# System Integration Architecture

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Document Type:** Integration Architecture Specification
**Team Responsibility:** System Architect (All Teams Coordination)
**Status:** Week 7 - Final Integration
**Date:** December 16, 2025
**Version:** 1.0

---

## Document Control

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-16 | System Architect | Complete multi-team integration architecture |

**Related Documents:**
- All Week 1-6 Requirements Documents
- VEHICLE_OVERALL_ARCHITECTURE.md (single-vehicle architecture)
- TVM_API_SPECIFICATION.md (fleet-vehicle communication)
- HARDWARE_SOFTWARE_INTERFACES.md (team boundaries)

---

## 1. System Overview - Multi-Team Integration

This document describes how **Vehicle (Pankaj), Fleet Management (Unno), and Hardware (Tsuchiya)** subsystems integrate to form a complete outdoor wheelchair transport fleet system.

### 1.1 Three-Layer Architecture

```
┌────────────────────────────────────────────────────────────┐
│         LAYER 3: Fleet Management (Unno Team)              │
│                                                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │
│  │  TVM Server  │  │   Fleet UI   │  │ Reservation  │   │
│  │  (Node.js)   │  │   (React)    │  │   System     │   │
│  └──────┬───────┘  └──────────────┘  └──────────────┘   │
│         │ REST API + WebSocket                            │
└─────────┼────────────────────────────────────────────────┘
          │
          ▼
┌─────────┼────────────────────────────────────────────────┐
│         │    LAYER 2: Vehicle Software (Pankaj Team)     │
│         │                                                 │
│    ┌────▼────┐   ┌────────────┐   ┌──────────────┐     │
│    │TVM Client│──│ Nav Stack  │──│ Docking Ctrl │      │
│    │(ROS 2)  │   │ (Nav2, NDT)│   │  (ArUco)     │     │
│    └─────────┘   └──────┬─────┘   └──────┬───────┘     │
│                         │                  │              │
└─────────────────────────┼──────────────────┼─────────────┘
                          │                  │
                          ▼                  ▼
┌──────────────────────────────────────────────────────────┐
│      LAYER 1: Hardware Platform (Tsuchiya Team)          │
│                                                          │
│  ┌───────────┐  ┌───────────┐  ┌────────────┐         │
│  │  Swerve   │  │  Sensors  │  │  Electrical │         │
│  │  Drive    │  │ (LiDAR,   │  │  (Battery,  │         │
│  │ (8 Motors)│  │  Cameras) │  │   BMS, CAN) │         │
│  └───────────┘  └───────────┘  └────────────┘         │
└──────────────────────────────────────────────────────────┘
```

---

## 2. Team Responsibilities and Integration Points

| Team | Core Responsibilities | Integration Interfaces |
|------|----------------------|------------------------|
| **Pankaj (Vehicle SW)** | Navigation, Perception, Docking, Safety, TVM Client | ↓ ROS 2 topics/CAN to Hardware<br>↑ REST/WebSocket to Fleet |
| **Unno (Fleet Mgmt)** | TVM Server, Fleet UI, Reservations, User Management | ↓ REST API to Vehicles<br>↑ Web UI to Operators |
| **Tsuchiya (Hardware)** | Mechanical, Electrical, Sensors, Motors, Charging | ↑ Sensor data/motor control to Vehicle SW |

---

## 3. Key Integration Scenarios

### 3.1 Mission Assignment Flow
```
Operator (Fleet UI) → TVM Server → Vehicle TVM Client → Navigation Controller → Swerve Drive
```

### 3.2 Autonomous Charging Flow
```
Vehicle (SOC <30%) → Nav to Charging Station → ArUco Docking → Charging Contacts → BMS Charging
```

### 3.3 Emergency Stop Flow
```
E-Stop Button → Safety Controller → All Subsystems (CAN broadcast) → Motors Stop (<100ms)
```

---

**Document End**
