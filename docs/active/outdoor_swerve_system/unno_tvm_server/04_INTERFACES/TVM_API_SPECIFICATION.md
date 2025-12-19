# TVM API Specification - Vehicle ↔ Fleet Management Server

**Document ID:** INT-TVM-API-001
**Version:** 1.0
**Date:** 2025-12-16
**Status:** Draft - CRITICAL (Week 1 Priority 1)
**Classification:** Internal

**Responsible Teams:**
- **Vehicle Client:** Pankaj (Software - Vehicle Side)
- **TVM Server:** Unno (Software - Fleet Management Side)

**Purpose:** Define exact API contract between autonomous vehicle and TVM (Total Vehicle Management) server to enable parallel development with mock interfaces.

---

## Document Control

| Version | Date | Authors | Changes |
|---------|------|---------|---------|
| 1.0 | 2025-12-16 | Pankaj + Unno | Initial API specification for multi-team development |

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [API Overview](#2-api-overview)
3. [Authentication & Authorization](#3-authentication--authorization)
4. [Vehicle → TVM Endpoints (REST)](#4-vehicle--tvm-endpoints-rest)
5. [TVM → Vehicle Commands (WebSocket)](#5-tvm--vehicle-commands-websocket)
6. [Data Models](#6-data-models)
7. [Error Handling](#7-error-handling)
8. [Offline Operation](#8-offline-operation)
9. [Mock Interfaces for Parallel Development](#9-mock-interfaces-for-parallel-development)
10. [Testing & Validation](#10-testing--validation)
11. [Deployment & Versioning](#11-deployment--versioning)

---

## 1. Introduction

### 1.1 Purpose

This document defines the **complete API contract** between:
- **Vehicle Client** (autonomous wheelchair transport robot - ROS 2 based)
- **TVM Server** (Total Vehicle Management fleet management system)

**Critical Dependencies:**
- Vehicle software CANNOT communicate with TVM server without this spec
- TVM server CANNOT receive vehicle data without this spec
- Both teams MUST implement exactly as specified

**Sign-Off Required:** Both Pankaj (Vehicle) and Unno (TVM Server) must approve this document before development begins.

---

### 1.2 Scope

**In Scope:**
- ✅ REST API for vehicle telemetry upload (location, status, battery, errors)
- ✅ WebSocket API for real-time TVM→Vehicle commands (dispatch, cancel, emergency stop)
- ✅ Authentication mechanism (JWT tokens)
- ✅ Data models (JSON schemas for all messages)
- ✅ Error codes and handling
- ✅ Offline operation behavior
- ✅ Mock interfaces for parallel development

**Out of Scope:**
- ❌ Internal TVM server implementation details
- ❌ Internal vehicle software architecture
- ❌ Database schema (TVM server internal)
- ❌ ROS 2 topics (vehicle internal)
- ❌ Log upload system (separate tvm_upload daemon)

---

### 1.3 Related Documents

- **TVM_DATA_MODELS.md** - Detailed JSON schemas for all messages
- **HARDWARE_SOFTWARE_INTERFACES.md** - ROS 2 interface specifications
- **VEHICLE_TVM_CLIENT_REQUIREMENTS.md** - Vehicle-side client requirements
- **TVM_SERVER_REQUIREMENTS.md** - Server-side requirements
- **TVM_INTEGRATION_REQUIREMENTS.md** - Integration requirements and acceptance criteria

---

## 2. API Overview

### 2.1 Technology Stack

**Communication Protocols:**
- **REST API (HTTPS):** Vehicle → TVM telemetry upload (unidirectional push)
- **WebSocket (WSS):** TVM → Vehicle commands (bidirectional, real-time)

**Data Format:**
- **JSON:** All message payloads
- **UTF-8 Encoding:** All text
- **ISO 8601 (UTC):** All timestamps

**Authentication:**
- **JWT (JSON Web Tokens):** Bearer token authentication
- **Token Expiration:** 24 hours (configurable)
- **Token Refresh:** Automatic refresh before expiration

**API Versioning:**
- **URL Versioning:** `/api/v1/...`
- **Current Version:** v1
- **Backward Compatibility:** Maintained for 6 months after version upgrade

---

### 2.2 Base URLs

**Production:**
```
REST API Base URL: https://tvm.example.com/api/v1
WebSocket URL:     wss://tvm.example.com/ws/v1
```

**Staging:**
```
REST API Base URL: https://tvm-staging.example.com/api/v1
WebSocket URL:     wss://tvm-staging.example.com/ws/v1
```

**Development (Local):**
```
REST API Base URL: http://localhost:8000/api/v1
WebSocket URL:     ws://localhost:8000/ws/v1
```

---

### 2.3 Rate Limiting

| Endpoint Category | Rate Limit | Burst | Notes |
|-------------------|------------|-------|-------|
| Location updates | 1 req/sec per vehicle | 5 | Normal telemetry |
| Status updates | 1 req/sec per vehicle | 5 | State changes |
| Battery updates | 1 req/5sec per vehicle | 2 | Lower frequency |
| Error reports | 10 req/sec per vehicle | 20 | High priority, bypass rate limit |
| Commands (WebSocket) | No limit | N/A | Real-time critical |

**Rate Limit Headers:**
```
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1702742400
```

**Rate Limit Exceeded Response:**
```json
{
  "error": "rate_limit_exceeded",
  "message": "Too many requests. Please wait 15 seconds.",
  "retry_after": 15
}
```

---

### 2.4 Communication Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                   Autonomous Vehicle                        │
│                                                             │
│  ┌────────────────────────────────────────────────────┐    │
│  │           Vehicle Software (ROS 2)                 │    │
│  │                                                    │    │
│  │  ┌──────────┐  ┌──────────┐  ┌────────────┐      │    │
│  │  │Navigation│  │ Docking  │  │   Safety   │      │    │
│  │  │ Subsystem│  │Subsystem │  │  Monitor   │      │    │
│  │  └─────┬────┘  └─────┬────┘  └──────┬─────┘      │    │
│  │        │             │              │             │    │
│  │        └─────────────┼──────────────┘             │    │
│  │                      │                            │    │
│  │              ┌───────▼────────┐                   │    │
│  │              │  TVM Client    │                   │    │
│  │              │   Library      │                   │    │
│  │              │                │                   │    │
│  │              │ ┌────────────┐ │                   │    │
│  │              │ │ REST Client│ │ ← Location,      │    │
│  │              │ └─────┬──────┘ │   Status,        │    │
│  │              │       │        │   Battery,       │    │
│  │              │       │ HTTPS  │   Errors         │    │
│  │              │ ┌─────▼──────┐ │                   │    │
│  │              │ │ WebSocket  │ │ ← Dispatch,      │    │
│  │              │ │   Client   │ │   Cancel,        │    │
│  │              │ └─────┬──────┘ │   E-Stop         │    │
│  │              └───────┼────────┘                   │    │
│  └──────────────────────┼────────────────────────────┘    │
└─────────────────────────┼──────────────────────────────────┘
                          │
                    ══════╪══════ INTERNET ══════
                          │
┌─────────────────────────┼──────────────────────────────────┐
│                         │                                  │
│              ┌──────────▼───────────┐                      │
│              │   Load Balancer      │                      │
│              │    (NGINX/HAProxy)   │                      │
│              └──────────┬───────────┘                      │
│                         │                                  │
│         ┌───────────────┴────────────────┐                │
│         │                                 │                │
│   ┌─────▼──────┐                  ┌──────▼─────┐         │
│   │ REST API   │                  │ WebSocket  │         │
│   │  Server    │                  │   Server   │         │
│   └─────┬──────┘                  └──────┬─────┘         │
│         │                                 │                │
│         └─────────────┬─────────────────┘                │
│                       │                                    │
│              ┌────────▼─────────┐                         │
│              │  TVM Server      │                         │
│              │  (Fleet Mgmt)    │                         │
│              │                  │                         │
│              │  ┌────────────┐  │                         │
│              │  │ Dashboard  │  │  ← PC/Tablet App        │
│              │  │  Service   │  │                         │
│              │  └────────────┘  │                         │
│              │  ┌────────────┐  │                         │
│              │  │Reservation │  │  ← Booking System       │
│              │  │  Service   │  │                         │
│              │  └────────────┘  │                         │
│              │  ┌────────────┐  │                         │
│              │  │  Database  │  │  ← PostgreSQL           │
│              │  │ (Postgres) │  │                         │
│              │  └────────────┘  │                         │
│              └──────────────────┘                         │
│                                                            │
│              TVM Server (Unno's Team)                     │
└────────────────────────────────────────────────────────────┘
```

---

## 3. Authentication & Authorization

### 3.1 Vehicle Registration

**Initial Setup (One-Time):**
1. Vehicle is registered in TVM server database (manual process)
2. Vehicle is assigned unique `vehicle_id` (e.g., `wheelchair_001`)
3. Vehicle is assigned API credentials (client_id + client_secret)

**Vehicle Credentials:**
```json
{
  "vehicle_id": "wheelchair_001",
  "client_id": "veh_abc123xyz456",
  "client_secret": "secret_789def012ghi"
}
```

**Storage:** Credentials stored securely on vehicle (encrypted configuration file)

---

### 3.2 Token Acquisition

**Endpoint:** `POST /api/v1/auth/token`

**Request:**
```json
{
  "client_id": "veh_abc123xyz456",
  "client_secret": "secret_789def012ghi",
  "grant_type": "client_credentials"
}
```

**Response (Success):**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 86400,
  "vehicle_id": "wheelchair_001"
}
```

**Response (Error):**
```json
{
  "error": "invalid_client",
  "error_description": "Invalid client credentials"
}
```

**Token Expiration:** 24 hours (86400 seconds)

---

### 3.3 Using Access Token

**All REST API requests MUST include:**
```
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Example:**
```bash
curl -X POST https://tvm.example.com/api/v1/vehicle/wheelchair_001/location \
  -H "Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..." \
  -H "Content-Type: application/json" \
  -d '{
    "latitude": 35.681236,
    "longitude": 139.767125,
    "timestamp": "2025-12-16T10:30:45.123Z"
  }'
```

---

### 3.4 Token Refresh

**Vehicle client SHALL:**
- Monitor token expiration (check `expires_in`)
- Refresh token 1 hour before expiration (proactive)
- Re-authenticate immediately if token expired (reactive)

**Refresh Logic:**
```python
# Pseudocode
def ensure_valid_token():
    if current_time > (token_acquired_at + expires_in - 3600):
        # Token expires in less than 1 hour, refresh proactively
        new_token = acquire_token(client_id, client_secret)
        update_token(new_token)
    return current_token
```

---

### 3.5 WebSocket Authentication

**WebSocket connection MUST include token in URL:**
```
wss://tvm.example.com/ws/v1/vehicle/wheelchair_001?token=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Connection Rejection (Invalid Token):**
```json
{
  "type": "error",
  "code": "auth_failed",
  "message": "Invalid or expired token"
}
```

**Connection close code:** 4001 (Authentication Failed)

---

## 4. Vehicle → TVM Endpoints (REST)

### 4.1 Location Update

**Endpoint:** `POST /api/v1/vehicle/{vehicle_id}/location`

**Frequency:** 1 Hz (once per second) during navigation, 0.2 Hz (once per 5 seconds) when idle

**Request Headers:**
```
Authorization: Bearer {access_token}
Content-Type: application/json
```

**Request Body:**
```json
{
  "timestamp": "2025-12-16T10:30:45.123Z",
  "location": {
    "latitude": 35.681236,
    "longitude": 139.767125,
    "altitude": 45.5,
    "heading": 90.5,
    "speed": 0.8,
    "accuracy": 0.15
  }
}
```

**Field Descriptions:**
| Field | Type | Unit | Range | Required | Description |
|-------|------|------|-------|----------|-------------|
| timestamp | string (ISO 8601 UTC) | - | - | ✅ | Time of measurement |
| latitude | float | degrees | -90 to 90 | ✅ | WGS84 latitude |
| longitude | float | degrees | -180 to 180 | ✅ | WGS84 longitude |
| altitude | float | meters | - | ⚠️ Optional | Altitude above sea level (if available) |
| heading | float | degrees | 0-360 | ✅ | True north heading (0=North, 90=East) |
| speed | float | m/s | 0-2.0 | ✅ | Current speed |
| accuracy | float | meters | 0-1.0 | ✅ | Position accuracy (±meters) |

**Response (Success):**
```json
{
  "status": "ok",
  "received_at": "2025-12-16T10:30:45.234Z"
}
```

**Response (Error):**
```json
{
  "error": "invalid_location",
  "message": "Latitude out of range",
  "field": "location.latitude",
  "received_value": 95.123
}
```

**HTTP Status Codes:**
- `200 OK` - Location updated successfully
- `400 Bad Request` - Invalid data format or out-of-range values
- `401 Unauthorized` - Invalid or expired token
- `429 Too Many Requests` - Rate limit exceeded
- `500 Internal Server Error` - Server error

---

### 4.2 Status Update

**Endpoint:** `POST /api/v1/vehicle/{vehicle_id}/status`

**Frequency:** On state change (event-driven), or every 10 seconds (heartbeat)

**Request Body:**
```json
{
  "timestamp": "2025-12-16T10:30:45.123Z",
  "state": "navigating",
  "mission_id": "mission_12345",
  "current_waypoint": "room_201",
  "passengers_onboard": true,
  "operational_hours": true,
  "details": {
    "progress_percent": 65.5,
    "eta_seconds": 180,
    "distance_remaining_meters": 145.2
  }
}
```

**Vehicle States (Enum):**
| State | Description | Transitions From | Transitions To |
|-------|-------------|------------------|----------------|
| `idle` | Vehicle stopped, no mission | charging, error | navigating, charging |
| `navigating` | Actively moving to destination | idle, docking | docking, idle, error |
| `docking` | Docking with wheelchair | navigating | idle, error |
| `charging` | Charging at station | idle | idle |
| `error` | System error, requires attention | any | idle (after recovery) |
| `emergency_stop` | Emergency stop activated | any | idle (after reset) |

**Field Descriptions:**
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| timestamp | string (ISO 8601 UTC) | ✅ | Time of status update |
| state | string (enum) | ✅ | Current vehicle state |
| mission_id | string | ⚠️ Optional | Current mission ID (if navigating) |
| current_waypoint | string | ⚠️ Optional | Current target waypoint (if navigating) |
| passengers_onboard | boolean | ✅ | True if wheelchair/passenger attached |
| operational_hours | boolean | ✅ | True if within operational hours window |
| details | object | ⚠️ Optional | State-specific details |

**Response (Success):**
```json
{
  "status": "ok",
  "received_at": "2025-12-16T10:30:45.234Z"
}
```

---

### 4.3 Battery Update

**Endpoint:** `POST /api/v1/vehicle/{vehicle_id}/battery`

**Frequency:** Every 5 seconds (lower frequency to reduce load)

**Request Body:**
```json
{
  "timestamp": "2025-12-16T10:30:45.123Z",
  "battery": {
    "soc_percent": 85.5,
    "voltage": 52.3,
    "current": -8.2,
    "temperature": 32.5,
    "charging": false,
    "estimated_runtime_minutes": 180,
    "health_percent": 95.0
  }
}
```

**Field Descriptions:**
| Field | Type | Unit | Required | Description |
|-------|------|------|----------|-------------|
| timestamp | string (ISO 8601 UTC) | - | ✅ | Time of measurement |
| soc_percent | float | % | ✅ | State of Charge (0-100%) |
| voltage | float | V | ✅ | Battery voltage |
| current | float | A | ✅ | Current (negative = discharging, positive = charging) |
| temperature | float | °C | ✅ | Battery temperature |
| charging | boolean | - | ✅ | True if actively charging |
| estimated_runtime_minutes | integer | minutes | ⚠️ Optional | Remaining runtime at current load |
| health_percent | float | % | ⚠️ Optional | Battery health (0-100%) |

**Response (Success):**
```json
{
  "status": "ok",
  "received_at": "2025-12-16T10:30:45.234Z",
  "warning": null
}
```

**Response (Low Battery Warning):**
```json
{
  "status": "ok",
  "received_at": "2025-12-16T10:30:45.234Z",
  "warning": "low_battery",
  "message": "Battery below 30%. Consider returning to charging station.",
  "recommended_action": "return_to_base"
}
```

---

### 4.4 Error Report

**Endpoint:** `POST /api/v1/vehicle/{vehicle_id}/error`

**Frequency:** Event-driven (immediate on error detection)

**Priority:** High (bypasses rate limiting)

**Request Body:**
```json
{
  "timestamp": "2025-12-16T10:35:12.456Z",
  "error_code": "NAV_OBSTACLE_BLOCKED",
  "severity": "warning",
  "subsystem": "navigation",
  "location": {
    "latitude": 35.681236,
    "longitude": 139.767125
  },
  "description": "Path blocked by obstacle, attempting reroute",
  "recovery_action": "rerouting",
  "user_action_required": false
}
```

**Severity Levels:**
| Severity | Description | Staff Alert | Vehicle Behavior |
|----------|-------------|-------------|------------------|
| `info` | Informational, no action needed | No | Continue operation |
| `warning` | Minor issue, vehicle can recover | Optional | Attempt automatic recovery |
| `error` | Significant issue, may affect mission | Yes (email/dashboard) | Stop or return to base |
| `critical` | Safety-critical, immediate attention | Yes (SMS + voice call) | Emergency stop, manual intervention required |

**Common Error Codes:**
| Code | Severity | Description |
|------|----------|-------------|
| `NAV_OBSTACLE_BLOCKED` | warning | Path blocked, rerouting |
| `NAV_LOCALIZATION_LOST` | error | Localization confidence too low |
| `DOCK_ALIGNMENT_FAILED` | warning | Docking alignment failed, retrying |
| `DOCK_MARKER_NOT_FOUND` | error | ArUco marker not detected |
| `SAFE_ESTOP_ACTIVATED` | critical | Emergency stop triggered |
| `SAFE_COLLISION_DETECTED` | critical | Collision detected by bumper |
| `POWER_BATTERY_CRITICAL` | critical | Battery below 10% |
| `SENSOR_LIDAR_FAILURE` | error | LiDAR sensor failure |

**Response (Success):**
```json
{
  "status": "acknowledged",
  "received_at": "2025-12-16T10:35:12.567Z",
  "alert_id": "alert_67890",
  "action": "alert_staff",
  "staff_notified": true
}
```

---

### 4.5 Docking Status Update

**Endpoint:** `POST /api/v1/vehicle/{vehicle_id}/docking`

**Frequency:** Event-driven during docking sequence

**Request Body:**
```json
{
  "timestamp": "2025-12-16T10:40:15.789Z",
  "phase": "visual_servoing",
  "marker_id": 42,
  "distance_to_marker_meters": 0.85,
  "alignment_error_degrees": 2.3,
  "docking_success": false,
  "attempts": 1,
  "details": {
    "marker_visible": true,
    "confidence": 0.95
  }
}
```

**Docking Phases:**
| Phase | Description | Expected Duration |
|-------|-------------|-------------------|
| `approaching` | Navigating to docking zone | 10-30 seconds |
| `marker_detection` | Searching for ArUco marker | 5-10 seconds |
| `visual_servoing` | Fine alignment using visual feedback | 15-30 seconds |
| `final_approach` | Slow final approach to dock | 10-15 seconds |
| `attachment` | Physical attachment mechanism | 5-10 seconds |
| `success` | Docking complete, wheelchair secured | - |
| `failed` | Docking failed, returning to idle | - |

**Response (Success):**
```json
{
  "status": "ok",
  "received_at": "2025-12-16T10:40:15.890Z"
}
```

---

## 5. TVM → Vehicle Commands (WebSocket)

### 5.1 WebSocket Connection

**Connection URL:**
```
wss://tvm.example.com/ws/v1/vehicle/{vehicle_id}?token={access_token}
```

**Connection Lifecycle:**
```
Vehicle                          TVM Server
  │                                   │
  ├─── WebSocket Connect ────────────>│
  │    (with JWT token in URL)        │
  │                                   │
  │<─── Connection Accepted ──────────┤
  │    (200 WebSocket Upgrade)        │
  │                                   │
  │<─── Welcome Message ──────────────┤
  │    { "type": "welcome", ... }     │
  │                                   │
  ├─── Ping ─────────────────────────>│ (every 30s)
  │<─── Pong ──────────────────────────┤
  │                                   │
  │<─── Dispatch Command ─────────────┤
  ├─── Command Acknowledgment ───────>│
  │                                   │
  │<─── Emergency Stop ───────────────┤
  ├─── Execution Confirmation ───────>│
  │                                   │
  ├─── Disconnect ───────────────────>│ (graceful)
  │    (close code 1000)              │
  │<─── Close Acknowledged ────────────┤
  │                                   │
```

---

### 5.2 Welcome Message (Server → Vehicle)

**Sent immediately after connection established**

**Message:**
```json
{
  "type": "welcome",
  "vehicle_id": "wheelchair_001",
  "server_time": "2025-12-16T10:30:00.000Z",
  "protocol_version": "1.0",
  "ping_interval_seconds": 30
}
```

---

### 5.3 Heartbeat (Ping/Pong)

**Vehicle SHALL send Ping every 30 seconds**

**Ping (Vehicle → Server):**
```json
{
  "type": "ping",
  "timestamp": "2025-12-16T10:30:30.123Z"
}
```

**Pong (Server → Vehicle):**
```json
{
  "type": "pong",
  "timestamp": "2025-12-16T10:30:30.234Z"
}
```

**Connection Timeout:**
- If no Pong received within 10 seconds → reconnect
- If no Ping received by server within 60 seconds → disconnect vehicle

---

### 5.4 Dispatch Command

**Purpose:** Send robot to specific location (with optional passenger pickup)

**Command (Server → Vehicle):**
```json
{
  "type": "dispatch",
  "command_id": "cmd_12345",
  "timestamp": "2025-12-16T10:40:00.000Z",
  "priority": "normal",
  "mission": {
    "mission_id": "mission_67890",
    "pickup": {
      "waypoint_id": "lobby_entrance",
      "latitude": 35.681100,
      "longitude": 139.767000,
      "action": "dock_wheelchair"
    },
    "destination": {
      "waypoint_id": "room_201",
      "latitude": 35.682456,
      "longitude": 139.768234,
      "action": "undock_wheelchair"
    },
    "scheduled_time": "2025-12-16T11:00:00.000Z"
  }
}
```

**Priority Levels:**
| Priority | Description | Behavior |
|----------|-------------|----------|
| `low` | Non-urgent task | Queue normally |
| `normal` | Standard task | Default priority |
| `high` | Urgent task | Queue ahead of normal |
| `urgent` | Emergency task | Interrupt current mission if safe |

**Vehicle Acknowledgment (Vehicle → Server):**
```json
{
  "type": "command_ack",
  "command_id": "cmd_12345",
  "status": "accepted",
  "estimated_arrival": "2025-12-16T10:55:00.000Z",
  "message": "Mission accepted, starting navigation"
}
```

**Rejection (Vehicle → Server):**
```json
{
  "type": "command_ack",
  "command_id": "cmd_12345",
  "status": "rejected",
  "reason": "battery_too_low",
  "message": "Battery at 15%, insufficient for mission"
}
```

**Rejection Reasons:**
| Reason | Description |
|--------|-------------|
| `battery_too_low` | Battery below 20% |
| `already_on_mission` | Vehicle already executing mission |
| `in_error_state` | Vehicle in error state |
| `outside_operational_hours` | Outside configured operational window |
| `invalid_destination` | Destination waypoint not found in map |

---

### 5.5 Cancel Mission Command

**Purpose:** Cancel current mission and return to idle

**Command (Server → Vehicle):**
```json
{
  "type": "cancel",
  "command_id": "cmd_23456",
  "timestamp": "2025-12-16T10:45:00.000Z",
  "mission_id": "mission_67890",
  "reason": "user_requested",
  "safe_stop": true
}
```

**safe_stop:**
- `true` → Stop safely at current location, undock if passenger onboard
- `false` → Stop immediately (emergency cancel)

**Vehicle Acknowledgment:**
```json
{
  "type": "command_ack",
  "command_id": "cmd_23456",
  "status": "executed",
  "stopped_at": {
    "latitude": 35.681500,
    "longitude": 139.767500
  },
  "message": "Mission canceled, returned to idle"
}
```

---

### 5.6 Emergency Stop Command

**Purpose:** Immediate stop (safety-critical)

**Priority:** CRITICAL (highest priority, immediate execution required)

**Command (Server → Vehicle):**
```json
{
  "type": "emergency_stop",
  "command_id": "cmd_ESTOP_34567",
  "timestamp": "2025-12-16T10:50:00.000Z",
  "reason": "staff_initiated",
  "message": "Emergency stop requested by operator"
}
```

**Vehicle MUST:**
1. Stop all motors within 1 second
2. Activate hazard lights/audio warning
3. Send acknowledgment immediately
4. Log emergency stop event

**Vehicle Acknowledgment:**
```json
{
  "type": "command_ack",
  "command_id": "cmd_ESTOP_34567",
  "status": "executed",
  "stopped_at": "2025-12-16T10:50:00.567Z",
  "location": {
    "latitude": 35.681600,
    "longitude": 139.767600
  }
}
```

**Resume from Emergency Stop:**
```json
{
  "type": "resume",
  "command_id": "cmd_45678",
  "timestamp": "2025-12-16T10:55:00.000Z"
}
```

---

### 5.7 Configuration Update Command

**Purpose:** Update vehicle configuration remotely

**Command (Server → Vehicle):**
```json
{
  "type": "configure",
  "command_id": "cmd_56789",
  "timestamp": "2025-12-16T11:00:00.000Z",
  "config_updates": {
    "max_speed": 0.8,
    "audio_volume": 75,
    "ehmi_language": "ja"
  }
}
```

**Configurable Parameters:**
| Parameter | Type | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| `max_speed` | float | 0.1-1.5 m/s | 1.0 | Maximum navigation speed |
| `audio_volume` | int | 0-100 | 80 | eHMI audio volume (%) |
| `ehmi_language` | string | en/ja/zh | ja | eHMI audio language |
| `telemetry_frequency` | float | 0.1-2.0 Hz | 1.0 | Location update frequency |
| `operational_hours_start` | string | HH:MM | 09:00 | Daily operation start time |
| `operational_hours_end` | string | HH:MM | 17:00 | Daily operation end time |

**Vehicle Acknowledgment:**
```json
{
  "type": "command_ack",
  "command_id": "cmd_56789",
  "status": "applied",
  "updated_config": {
    "max_speed": 0.8,
    "audio_volume": 75,
    "ehmi_language": "ja"
  }
}
```

---

## 6. Data Models

**See TVM_DATA_MODELS.md for complete JSON schemas**

### 6.1 Core Data Types

**Location:**
```typescript
interface Location {
  latitude: number;      // -90 to 90 degrees (WGS84)
  longitude: number;     // -180 to 180 degrees (WGS84)
  altitude?: number;     // meters above sea level (optional)
  heading: number;       // 0-360 degrees (true north)
  speed: number;         // m/s (0-2.0)
  accuracy: number;      // ±meters (0-1.0)
}
```

**Timestamp:**
```typescript
type Timestamp = string; // ISO 8601 UTC format: "2025-12-16T10:30:45.123Z"
```

**Vehicle State:**
```typescript
type VehicleState =
  | "idle"
  | "navigating"
  | "docking"
  | "charging"
  | "error"
  | "emergency_stop";
```

**Error Severity:**
```typescript
type ErrorSeverity = "info" | "warning" | "error" | "critical";
```

**Command Priority:**
```typescript
type CommandPriority = "low" | "normal" | "high" | "urgent";
```

---

## 7. Error Handling

### 7.1 HTTP Error Responses

**Standard Error Format:**
```json
{
  "error": "error_code",
  "message": "Human-readable error description",
  "field": "field_name",
  "received_value": "invalid_value",
  "timestamp": "2025-12-16T10:30:45.123Z"
}
```

**Common HTTP Status Codes:**
| Code | Error | Description |
|------|-------|-------------|
| 400 | Bad Request | Invalid data format or validation error |
| 401 | Unauthorized | Invalid or expired authentication token |
| 403 | Forbidden | Vehicle not authorized for this action |
| 404 | Not Found | Endpoint or resource not found |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server-side error |
| 503 | Service Unavailable | Server temporarily unavailable |

---

### 7.2 WebSocket Error Messages

**Format:**
```json
{
  "type": "error",
  "code": "error_code",
  "message": "Human-readable error description",
  "timestamp": "2025-12-16T10:30:45.123Z"
}
```

**WebSocket Close Codes:**
| Code | Reason | Description |
|------|--------|-------------|
| 1000 | Normal Closure | Graceful disconnect |
| 1001 | Going Away | Server shutting down |
| 4001 | Authentication Failed | Invalid or expired token |
| 4002 | Invalid Message | Malformed message received |
| 4003 | Rate Limit Exceeded | Too many commands |

---

### 7.3 Retry Logic

**Vehicle Client SHALL implement exponential backoff:**

```python
# Pseudocode
def retry_with_backoff(request_func, max_retries=5):
    base_delay = 1  # second
    max_delay = 60  # seconds

    for attempt in range(max_retries):
        try:
            return request_func()
        except (ConnectionError, Timeout) as e:
            if attempt == max_retries - 1:
                raise  # Give up after max retries

            delay = min(base_delay * (2 ** attempt), max_delay)
            wait_with_jitter(delay)
```

**Retry Policy:**
| Error Type | Retry | Max Retries | Backoff |
|------------|-------|-------------|---------|
| Connection timeout | Yes | 5 | Exponential (1s, 2s, 4s, 8s, 16s) |
| 500 Internal Server Error | Yes | 3 | Exponential |
| 503 Service Unavailable | Yes | 3 | Exponential |
| 429 Too Many Requests | Yes | Unlimited | Wait `retry_after` seconds |
| 400 Bad Request | No | 0 | N/A (fix data and retry manually) |
| 401 Unauthorized | Yes (re-auth) | 1 | Immediate (acquire new token) |

---

## 8. Offline Operation

### 8.1 Vehicle Behavior When TVM Server Unreachable

**Vehicle SHALL:**
1. ✅ Continue autonomous operation (use local navigation)
2. ✅ Queue telemetry messages locally (up to 1000 messages or 10 MB)
3. ✅ Attempt reconnection every 30 seconds
4. ✅ Log all offline events
5. ❌ NOT accept new missions (only complete current mission)
6. ✅ Return to base if battery <30% and cannot upload status

**Message Queueing:**
```
┌────────────────────────────────────┐
│ Vehicle (Offline Mode)             │
│                                    │
│ ┌────────────────────────────────┐ │
│ │  Local Message Queue           │ │
│ │                                │ │
│ │  1. Location (t=10:30:00)      │ │
│ │  2. Location (t=10:30:01)      │ │
│ │  3. Battery  (t=10:30:05)      │ │
│ │  4. Error    (t=10:30:10)      │ │
│ │  ... (up to 1000 messages)     │ │
│ └────────────────────────────────┘ │
│                                    │
│ Reconnection Timer: 30s            │
└────────────────────────────────────┘
         │
         │ Reconnection Successful
         ▼
┌────────────────────────────────────┐
│ TVM Server                         │
│                                    │
│ Bulk Upload Endpoint:              │
│ POST /api/v1/vehicle/{id}/bulk     │
│                                    │
│ Accepts up to 1000 messages        │
└────────────────────────────────────┘
```

---

### 8.2 Bulk Upload After Reconnection

**Endpoint:** `POST /api/v1/vehicle/{vehicle_id}/bulk`

**Request (Bulk Upload):**
```json
{
  "vehicle_id": "wheelchair_001",
  "offline_period": {
    "start": "2025-12-16T10:30:00.000Z",
    "end": "2025-12-16T10:35:00.000Z"
  },
  "messages": [
    {
      "type": "location",
      "timestamp": "2025-12-16T10:30:00.123Z",
      "data": { /* location data */ }
    },
    {
      "type": "battery",
      "timestamp": "2025-12-16T10:30:05.123Z",
      "data": { /* battery data */ }
    }
    // ... up to 1000 messages
  ]
}
```

**Response:**
```json
{
  "status": "ok",
  "messages_received": 247,
  "messages_processed": 247,
  "offline_duration_seconds": 300
}
```

---

## 9. Mock Interfaces for Parallel Development

### 9.1 Mock TVM Server (for Pankaj - Vehicle Development)

**Purpose:** Allow vehicle software development without waiting for real TVM server

**Implementation:** Python FastAPI mock server

**Features:**
- ✅ Accept all REST API endpoints
- ✅ Return success responses
- ✅ Log all received messages
- ✅ Send random test commands via WebSocket
- ✅ Simulate network delays (configurable)

**Example Mock Server (Python):**
```python
# mock_tvm_server.py
from fastapi import FastAPI, WebSocket
import uvicorn

app = FastAPI()

@app.post("/api/v1/vehicle/{vehicle_id}/location")
async def location_update(vehicle_id: str, data: dict):
    print(f"[MOCK] Received location from {vehicle_id}: {data}")
    return {"status": "ok", "received_at": "2025-12-16T10:30:45.234Z"}

@app.post("/api/v1/vehicle/{vehicle_id}/status")
async def status_update(vehicle_id: str, data: dict):
    print(f"[MOCK] Received status from {vehicle_id}: {data}")
    return {"status": "ok", "received_at": "2025-12-16T10:30:45.234Z"}

@app.websocket("/ws/v1/vehicle/{vehicle_id}")
async def websocket_endpoint(websocket: WebSocket, vehicle_id: str):
    await websocket.accept()
    await websocket.send_json({
        "type": "welcome",
        "vehicle_id": vehicle_id,
        "server_time": "2025-12-16T10:30:00.000Z"
    })

    # Send test dispatch command after 10 seconds
    await asyncio.sleep(10)
    await websocket.send_json({
        "type": "dispatch",
        "command_id": "cmd_test_001",
        "mission": {
            "destination": {
                "waypoint_id": "test_waypoint",
                "latitude": 35.681236,
                "longitude": 139.767125
            }
        }
    })

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

**Running Mock Server:**
```bash
pip install fastapi uvicorn
python mock_tvm_server.py
```

**Vehicle connects to:** `http://localhost:8000`

---

### 9.2 Mock Vehicle Client (for Unno - TVM Server Development)

**Purpose:** Allow TVM server development without waiting for real vehicle

**Implementation:** Python script simulating vehicle behavior

**Features:**
- ✅ Send location updates (1 Hz)
- ✅ Send status updates (on state change)
- ✅ Send battery updates (every 5s)
- ✅ Respond to commands via WebSocket
- ✅ Simulate realistic vehicle behavior (navigation, docking, errors)

**Example Mock Vehicle (Python):**
```python
# mock_vehicle.py
import requests
import asyncio
import websockets
import json
from datetime import datetime

VEHICLE_ID = "wheelchair_001"
TVM_BASE_URL = "https://tvm.example.com/api/v1"
TVM_WS_URL = f"wss://tvm.example.com/ws/v1/vehicle/{VEHICLE_ID}"

async def send_location_updates():
    """Send location updates every 1 second"""
    while True:
        location_data = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "location": {
                "latitude": 35.681236,
                "longitude": 139.767125,
                "heading": 90.0,
                "speed": 0.5,
                "accuracy": 0.10
            }
        }
        response = requests.post(
            f"{TVM_BASE_URL}/vehicle/{VEHICLE_ID}/location",
            json=location_data,
            headers={"Authorization": f"Bearer {TOKEN}"}
        )
        print(f"Location update: {response.status_code}")
        await asyncio.sleep(1)

async def websocket_command_listener():
    """Listen for commands from TVM server"""
    async with websockets.connect(TVM_WS_URL) as websocket:
        async for message in websocket:
            cmd = json.loads(message)
            print(f"Received command: {cmd['type']}")

            if cmd['type'] == 'dispatch':
                # Send acknowledgment
                ack = {
                    "type": "command_ack",
                    "command_id": cmd['command_id'],
                    "status": "accepted"
                }
                await websocket.send(json.dumps(ack))

# Run both tasks concurrently
asyncio.run(asyncio.gather(
    send_location_updates(),
    websocket_command_listener()
))
```

---

## 10. Testing & Validation

### 10.1 API Contract Testing

**Both teams SHALL:**
1. ✅ Use API specification as contract
2. ✅ Write integration tests against specification
3. ✅ Use mock interfaces during development
4. ✅ Perform end-to-end testing before production

**Test Categories:**
| Test Type | Responsibility | Tools |
|-----------|----------------|-------|
| Unit tests (API endpoints) | Unno (TVM Server) | pytest, FastAPI TestClient |
| Unit tests (TVM client lib) | Pankaj (Vehicle) | pytest, requests-mock |
| Integration tests | Both (joint) | Postman, pytest + real endpoints |
| Load testing | Unno (TVM Server) | Locust, Apache Bench |
| WebSocket testing | Both (joint) | ws (Node.js), websockets (Python) |

---

### 10.2 Acceptance Criteria

**API specification is ACCEPTED when:**
- ✅ All endpoints documented with examples
- ✅ All data models defined with JSON schemas
- ✅ Error handling specified
- ✅ Mock interfaces available for both teams
- ✅ Integration tests pass (>95% success rate)
- ✅ Load testing: 10 vehicles @ 1 Hz telemetry (sustained 10 minutes)
- ✅ WebSocket commands: <500ms latency (95th percentile)
- ✅ Offline operation: Queue and bulk upload tested

---

## 11. Deployment & Versioning

### 11.1 API Versioning

**URL Versioning:** `/api/v1/...` → `/api/v2/...`

**Version Support:**
- Current version (v1): Fully supported
- Previous version (v0): Supported for 6 months after v1 release
- Deprecated version: 3 months notice before removal

**Breaking Changes (Require New Version):**
- Removing endpoints
- Changing required fields
- Changing data types
- Renaming fields

**Non-Breaking Changes (Same Version):**
- Adding optional fields
- Adding new endpoints
- Deprecating fields (with warning period)

---

### 11.2 Deployment Checklist

**Before Production Deployment:**
- ✅ API specification reviewed and approved by both teams
- ✅ Integration tests passing (>95%)
- ✅ Load testing completed (10 vehicles, 1 Hz, 10 minutes)
- ✅ Security audit completed (JWT implementation, HTTPS)
- ✅ Monitoring and logging configured (CloudWatch, Prometheus)
- ✅ Error alerting configured (PagerDuty, email)
- ✅ Documentation published (API reference, developer guides)
- ✅ Rollback plan prepared

---

## Document Status

**Status:** Draft - Pending Review
**Next Review:** 2025-12-17 (Kickoff meeting - Pankaj + Unno)
**Sign-Off Required:** Both teams must approve before development begins

---

## Appendices

### Appendix A: Quick Reference - REST API Endpoints

| Method | Endpoint | Frequency | Purpose |
|--------|----------|-----------|---------|
| POST | `/api/v1/auth/token` | On startup / token expiry | Get JWT token |
| POST | `/api/v1/vehicle/{id}/location` | 1 Hz | Location telemetry |
| POST | `/api/v1/vehicle/{id}/status` | Event-driven / 10s | Status updates |
| POST | `/api/v1/vehicle/{id}/battery` | Every 5s | Battery telemetry |
| POST | `/api/v1/vehicle/{id}/error` | Event-driven | Error reports |
| POST | `/api/v1/vehicle/{id}/docking` | Event-driven | Docking status |
| POST | `/api/v1/vehicle/{id}/bulk` | After offline | Bulk message upload |

---

### Appendix B: Quick Reference - WebSocket Commands

| Command Type | Direction | Priority | Purpose |
|--------------|-----------|----------|---------|
| `welcome` | Server → Vehicle | N/A | Connection confirmation |
| `ping` / `pong` | Bidirectional | N/A | Heartbeat |
| `dispatch` | Server → Vehicle | Normal/High/Urgent | Send to destination |
| `cancel` | Server → Vehicle | High | Cancel mission |
| `emergency_stop` | Server → Vehicle | CRITICAL | Immediate stop |
| `resume` | Server → Vehicle | High | Resume from E-Stop |
| `configure` | Server → Vehicle | Normal | Update config |
| `command_ack` | Vehicle → Server | N/A | Command acknowledgment |

---

### Appendix C: Example cURL Commands

**Get Token:**
```bash
curl -X POST https://tvm.example.com/api/v1/auth/token \
  -H "Content-Type: application/json" \
  -d '{
    "client_id": "veh_abc123xyz456",
    "client_secret": "secret_789def012ghi",
    "grant_type": "client_credentials"
  }'
```

**Send Location:**
```bash
curl -X POST https://tvm.example.com/api/v1/vehicle/wheelchair_001/location \
  -H "Authorization: Bearer eyJhbG..." \
  -H "Content-Type: application/json" \
  -d '{
    "timestamp": "2025-12-16T10:30:45.123Z",
    "location": {
      "latitude": 35.681236,
      "longitude": 139.767125,
      "heading": 90.0,
      "speed": 0.5,
      "accuracy": 0.10
    }
  }'
```

---

**END OF DOCUMENT**

**Critical Next Steps:**
1. ⚠️ **Week 1 Day 1-2:** Pankaj + Unno review this specification
2. ⚠️ **Week 1 Day 3-4:** Discuss and finalize API design (meeting required)
3. ⚠️ **Week 1 Day 5:** Sign-off by both teams
4. ⚠️ **Week 1 Day 6-10:** Implement mock interfaces
5. ⚠️ **Week 2+:** Begin parallel development (Vehicle client + TVM server)

**Questions/Clarifications:** Contact Pankaj (Vehicle) or Unno (TVM Server)
