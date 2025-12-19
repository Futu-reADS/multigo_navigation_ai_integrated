# TVM Data Models - Shared Data Structures

**Document ID:** INT-TVM-DATA-001
**Version:** 1.0
**Date:** 2025-12-16
**Status:** Draft - CRITICAL (Week 1 Priority 2)
**Classification:** Internal

**Responsible Teams:**
- **Vehicle Client:** Pankaj (Software - Vehicle Side)
- **TVM Server:** Unno (Software - Fleet Management Side)

**Purpose:** Define exact data structures, JSON schemas, and validation rules for all messages exchanged between vehicle and TVM server.

---

## Document Control

| Version | Date | Authors | Changes |
|---------|------|---------|---------|
| 1.0 | 2025-12-16 | Pankaj + Unno | Initial data model specification |

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Common Data Types](#2-common-data-types)
3. [Vehicle Telemetry Models](#3-vehicle-telemetry-models)
4. [Command Models](#4-command-models)
5. [Error Models](#5-error-models)
6. [JSON Schema Definitions](#6-json-schema-definitions)
7. [Validation Rules](#7-validation-rules)
8. [Enum Definitions](#8-enum-definitions)

---

## 1. Introduction

### 1.1 Purpose

This document defines **complete data models** for all messages in the TVM API:
- JSON structure for each message type
- Field types, ranges, and constraints
- Required vs. optional fields
- Validation rules
- Enum value definitions

**All implementations MUST validate messages against these schemas.**

---

### 1.2 JSON Schema Standard

**Format:** JSON Schema Draft 07
**Validation Libraries:**
- Python: `jsonschema`, `pydantic`
- JavaScript: `ajv`, `joi`
- Go: `gojsonschema`

---

### 1.3 Data Format Conventions

**Timestamps:**
- Format: ISO 8601 UTC
- Example: `"2025-12-16T10:30:45.123Z"`
- Precision: Milliseconds (3 decimal places)
- Timezone: Always UTC (Z suffix)

**Coordinates:**
- System: WGS84
- Latitude: -90.0 to +90.0 degrees
- Longitude: -180.0 to +180.0 degrees
- Precision: 6 decimal places (~0.1 meter)

**Units:**
- Distance: meters (m)
- Speed: meters per second (m/s)
- Angle: degrees (0-360, clockwise from true north)
- Temperature: degrees Celsius (°C)
- Voltage: volts (V)
- Current: amperes (A)
- Battery: percentage (0-100%)

---

## 2. Common Data Types

### 2.1 Location

**TypeScript Interface:**
```typescript
interface Location {
  latitude: number;      // -90.0 to +90.0 degrees (WGS84)
  longitude: number;     // -180.0 to +180.0 degrees (WGS84)
  altitude?: number;     // meters above sea level (optional)
  heading: number;       // 0-360 degrees (true north = 0, clockwise)
  speed: number;         // m/s (0.0 to 2.0)
  accuracy: number;      // ±meters (position accuracy, 0.0 to 1.0)
}
```

**JSON Schema:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "latitude": {
      "type": "number",
      "minimum": -90.0,
      "maximum": 90.0,
      "description": "Latitude in WGS84 degrees"
    },
    "longitude": {
      "type": "number",
      "minimum": -180.0,
      "maximum": 180.0,
      "description": "Longitude in WGS84 degrees"
    },
    "altitude": {
      "type": "number",
      "description": "Altitude in meters above sea level (optional)"
    },
    "heading": {
      "type": "number",
      "minimum": 0.0,
      "maximum": 360.0,
      "description": "Heading in degrees (0=North, 90=East, clockwise)"
    },
    "speed": {
      "type": "number",
      "minimum": 0.0,
      "maximum": 2.0,
      "description": "Speed in m/s"
    },
    "accuracy": {
      "type": "number",
      "minimum": 0.0,
      "maximum": 1.0,
      "description": "Position accuracy in ±meters"
    }
  },
  "required": ["latitude", "longitude", "heading", "speed", "accuracy"],
  "additionalProperties": false
}
```

**Example:**
```json
{
  "latitude": 35.681236,
  "longitude": 139.767125,
  "altitude": 45.5,
  "heading": 90.5,
  "speed": 0.8,
  "accuracy": 0.15
}
```

---

### 2.2 Timestamp

**Type:** String (ISO 8601 UTC)

**Format:** `YYYY-MM-DDTHH:MM:SS.sssZ`

**JSON Schema:**
```json
{
  "type": "string",
  "format": "date-time",
  "pattern": "^\\d{4}-\\d{2}-\\d{2}T\\d{2}:\\d{2}:\\d{2}\\.\\d{3}Z$",
  "description": "ISO 8601 UTC timestamp with millisecond precision"
}
```

**Examples:**
```json
"2025-12-16T10:30:45.123Z"  ✅ Valid
"2025-12-16T10:30:45Z"      ❌ Invalid (missing milliseconds)
"2025-12-16 10:30:45"       ❌ Invalid (wrong format)
"2025-12-16T10:30:45+09:00" ❌ Invalid (not UTC, must use Z)
```

---

### 2.3 Vehicle State (Enum)

**Type:** String (enum)

**Allowed Values:**
```typescript
type VehicleState =
  | "idle"            // Stopped, no active mission
  | "navigating"      // Actively moving to destination
  | "docking"         // Docking with wheelchair
  | "charging"        // Charging at station
  | "error"           // System error
  | "emergency_stop"; // Emergency stop activated
```

**JSON Schema:**
```json
{
  "type": "string",
  "enum": ["idle", "navigating", "docking", "charging", "error", "emergency_stop"],
  "description": "Current operational state of the vehicle"
}
```

**State Transitions:**
```
idle ──────────> navigating ──────────> docking ──────────> idle
  │                   │                    │                    │
  │                   │                    │                    │
  └──────> charging <─┴────────────────────┴────────────────────┘
           │
           │
      error / emergency_stop (from any state)
           │
           └──────> idle (after recovery)
```

---

### 2.4 Error Severity (Enum)

**Type:** String (enum)

**Allowed Values:**
```typescript
type ErrorSeverity =
  | "info"     // Informational, no action needed
  | "warning"  // Minor issue, vehicle can recover
  | "error"    // Significant issue, may affect mission
  | "critical" // Safety-critical, immediate attention required
```

**JSON Schema:**
```json
{
  "type": "string",
  "enum": ["info", "warning", "error", "critical"],
  "description": "Severity level of the error"
}
```

**Severity Behavior:**
| Severity | Staff Alert | Vehicle Behavior | Examples |
|----------|-------------|------------------|----------|
| `info` | No | Continue | Waypoint reached, task completed |
| `warning` | Dashboard only | Attempt recovery | Path blocked (rerouting) |
| `error` | Email + Dashboard | Stop or return | Localization lost, sensor failure |
| `critical` | SMS + Voice call | Emergency stop | Collision, E-stop, battery critical |

---

### 2.5 Command Priority (Enum)

**Type:** String (enum)

**Allowed Values:**
```typescript
type CommandPriority =
  | "low"    // Non-urgent, queue normally
  | "normal" // Standard priority (default)
  | "high"   // Urgent, queue ahead
  | "urgent" // Emergency, interrupt if safe
```

**JSON Schema:**
```json
{
  "type": "string",
  "enum": ["low", "normal", "high", "urgent"],
  "default": "normal",
  "description": "Priority level for mission dispatch"
}
```

---

## 3. Vehicle Telemetry Models

### 3.1 Location Update

**Message Type:** `POST /api/v1/vehicle/{id}/location`

**TypeScript Interface:**
```typescript
interface LocationUpdate {
  timestamp: string;    // ISO 8601 UTC
  location: Location;   // See 2.1
}
```

**JSON Schema:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "timestamp": {
      "$ref": "#/definitions/Timestamp"
    },
    "location": {
      "$ref": "#/definitions/Location"
    }
  },
  "required": ["timestamp", "location"],
  "additionalProperties": false
}
```

**Example:**
```json
{
  "timestamp": "2025-12-16T10:30:45.123Z",
  "location": {
    "latitude": 35.681236,
    "longitude": 139.767125,
    "heading": 90.5,
    "speed": 0.8,
    "accuracy": 0.15
  }
}
```

---

### 3.2 Status Update

**Message Type:** `POST /api/v1/vehicle/{id}/status`

**TypeScript Interface:**
```typescript
interface StatusUpdate {
  timestamp: string;
  state: VehicleState;
  mission_id?: string;
  current_waypoint?: string;
  passengers_onboard: boolean;
  operational_hours: boolean;
  details?: {
    progress_percent?: number;
    eta_seconds?: number;
    distance_remaining_meters?: number;
  };
}
```

**JSON Schema:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "timestamp": {
      "$ref": "#/definitions/Timestamp"
    },
    "state": {
      "$ref": "#/definitions/VehicleState"
    },
    "mission_id": {
      "type": "string",
      "pattern": "^mission_[a-zA-Z0-9]+$",
      "description": "Current mission ID (if active)"
    },
    "current_waypoint": {
      "type": "string",
      "description": "Current target waypoint (if navigating)"
    },
    "passengers_onboard": {
      "type": "boolean",
      "description": "True if wheelchair/passenger attached"
    },
    "operational_hours": {
      "type": "boolean",
      "description": "True if within operational hours window"
    },
    "details": {
      "type": "object",
      "properties": {
        "progress_percent": {
          "type": "number",
          "minimum": 0,
          "maximum": 100
        },
        "eta_seconds": {
          "type": "integer",
          "minimum": 0
        },
        "distance_remaining_meters": {
          "type": "number",
          "minimum": 0
        }
      },
      "additionalProperties": false
    }
  },
  "required": ["timestamp", "state", "passengers_onboard", "operational_hours"],
  "additionalProperties": false
}
```

**Example:**
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

---

### 3.3 Battery Update

**Message Type:** `POST /api/v1/vehicle/{id}/battery`

**TypeScript Interface:**
```typescript
interface BatteryUpdate {
  timestamp: string;
  battery: {
    soc_percent: number;                // State of Charge (0-100%)
    voltage: number;                    // Volts
    current: number;                    // Amperes (negative = discharging)
    temperature: number;                // Celsius
    charging: boolean;                  // True if actively charging
    estimated_runtime_minutes?: number; // Remaining runtime (optional)
    health_percent?: number;            // Battery health 0-100% (optional)
  };
}
```

**JSON Schema:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "timestamp": {
      "$ref": "#/definitions/Timestamp"
    },
    "battery": {
      "type": "object",
      "properties": {
        "soc_percent": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "State of Charge percentage"
        },
        "voltage": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "Battery voltage in volts"
        },
        "current": {
          "type": "number",
          "minimum": -100,
          "maximum": 100,
          "description": "Current in amperes (negative = discharging)"
        },
        "temperature": {
          "type": "number",
          "minimum": -20,
          "maximum": 80,
          "description": "Battery temperature in Celsius"
        },
        "charging": {
          "type": "boolean",
          "description": "True if actively charging"
        },
        "estimated_runtime_minutes": {
          "type": "integer",
          "minimum": 0,
          "description": "Estimated runtime in minutes"
        },
        "health_percent": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "Battery health percentage"
        }
      },
      "required": ["soc_percent", "voltage", "current", "temperature", "charging"],
      "additionalProperties": false
    }
  },
  "required": ["timestamp", "battery"],
  "additionalProperties": false
}
```

**Example:**
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

---

### 3.4 Error Report

**Message Type:** `POST /api/v1/vehicle/{id}/error`

**TypeScript Interface:**
```typescript
interface ErrorReport {
  timestamp: string;
  error_code: string;
  severity: ErrorSeverity;
  subsystem: string;
  location?: Location;
  description: string;
  recovery_action?: string;
  user_action_required: boolean;
}
```

**JSON Schema:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "timestamp": {
      "$ref": "#/definitions/Timestamp"
    },
    "error_code": {
      "type": "string",
      "pattern": "^[A-Z_]+$",
      "description": "Error code (e.g., NAV_OBSTACLE_BLOCKED)"
    },
    "severity": {
      "$ref": "#/definitions/ErrorSeverity"
    },
    "subsystem": {
      "type": "string",
      "enum": ["navigation", "docking", "perception", "power", "safety", "hardware"],
      "description": "Subsystem that generated the error"
    },
    "location": {
      "$ref": "#/definitions/Location"
    },
    "description": {
      "type": "string",
      "maxLength": 500,
      "description": "Human-readable error description"
    },
    "recovery_action": {
      "type": "string",
      "maxLength": 200,
      "description": "Automated recovery action being attempted"
    },
    "user_action_required": {
      "type": "boolean",
      "description": "True if manual intervention needed"
    }
  },
  "required": ["timestamp", "error_code", "severity", "subsystem", "description", "user_action_required"],
  "additionalProperties": false
}
```

**Example:**
```json
{
  "timestamp": "2025-12-16T10:35:12.456Z",
  "error_code": "NAV_OBSTACLE_BLOCKED",
  "severity": "warning",
  "subsystem": "navigation",
  "location": {
    "latitude": 35.681236,
    "longitude": 139.767125,
    "heading": 90.0,
    "speed": 0.0,
    "accuracy": 0.10
  },
  "description": "Path blocked by obstacle, attempting reroute",
  "recovery_action": "rerouting",
  "user_action_required": false
}
```

---

### 3.5 Docking Status Update

**Message Type:** `POST /api/v1/vehicle/{id}/docking`

**TypeScript Interface:**
```typescript
interface DockingUpdate {
  timestamp: string;
  phase: DockingPhase;
  marker_id?: number;
  distance_to_marker_meters?: number;
  alignment_error_degrees?: number;
  docking_success: boolean;
  attempts: number;
  details?: {
    marker_visible?: boolean;
    confidence?: number;
  };
}

type DockingPhase =
  | "approaching"
  | "marker_detection"
  | "visual_servoing"
  | "final_approach"
  | "attachment"
  | "success"
  | "failed";
```

**JSON Schema:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "timestamp": {
      "$ref": "#/definitions/Timestamp"
    },
    "phase": {
      "type": "string",
      "enum": ["approaching", "marker_detection", "visual_servoing", "final_approach", "attachment", "success", "failed"]
    },
    "marker_id": {
      "type": "integer",
      "minimum": 0,
      "maximum": 999,
      "description": "ArUco marker ID"
    },
    "distance_to_marker_meters": {
      "type": "number",
      "minimum": 0,
      "maximum": 10,
      "description": "Distance to marker in meters"
    },
    "alignment_error_degrees": {
      "type": "number",
      "minimum": 0,
      "maximum": 180,
      "description": "Alignment error in degrees"
    },
    "docking_success": {
      "type": "boolean",
      "description": "True if docking succeeded"
    },
    "attempts": {
      "type": "integer",
      "minimum": 0,
      "description": "Number of docking attempts"
    },
    "details": {
      "type": "object",
      "properties": {
        "marker_visible": {
          "type": "boolean"
        },
        "confidence": {
          "type": "number",
          "minimum": 0,
          "maximum": 1
        }
      }
    }
  },
  "required": ["timestamp", "phase", "docking_success", "attempts"],
  "additionalProperties": false
}
```

**Example:**
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

---

## 4. Command Models

### 4.1 Dispatch Command

**WebSocket Message Type:** `dispatch`

**TypeScript Interface:**
```typescript
interface DispatchCommand {
  type: "dispatch";
  command_id: string;
  timestamp: string;
  priority: CommandPriority;
  mission: {
    mission_id: string;
    pickup?: {
      waypoint_id: string;
      latitude: number;
      longitude: number;
      action: "dock_wheelchair" | "wait";
    };
    destination: {
      waypoint_id: string;
      latitude: number;
      longitude: number;
      action: "undock_wheelchair" | "park";
    };
    scheduled_time?: string;
  };
}
```

**JSON Schema:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "type": {
      "const": "dispatch"
    },
    "command_id": {
      "type": "string",
      "pattern": "^cmd_[a-zA-Z0-9]+$"
    },
    "timestamp": {
      "$ref": "#/definitions/Timestamp"
    },
    "priority": {
      "$ref": "#/definitions/CommandPriority"
    },
    "mission": {
      "type": "object",
      "properties": {
        "mission_id": {
          "type": "string",
          "pattern": "^mission_[a-zA-Z0-9]+$"
        },
        "pickup": {
          "type": "object",
          "properties": {
            "waypoint_id": {
              "type": "string"
            },
            "latitude": {
              "type": "number",
              "minimum": -90,
              "maximum": 90
            },
            "longitude": {
              "type": "number",
              "minimum": -180,
              "maximum": 180
            },
            "action": {
              "type": "string",
              "enum": ["dock_wheelchair", "wait"]
            }
          },
          "required": ["waypoint_id", "latitude", "longitude", "action"]
        },
        "destination": {
          "type": "object",
          "properties": {
            "waypoint_id": {
              "type": "string"
            },
            "latitude": {
              "type": "number",
              "minimum": -90,
              "maximum": 90
            },
            "longitude": {
              "type": "number",
              "minimum": -180,
              "maximum": 180
            },
            "action": {
              "type": "string",
              "enum": ["undock_wheelchair", "park"]
            }
          },
          "required": ["waypoint_id", "latitude", "longitude", "action"]
        },
        "scheduled_time": {
          "$ref": "#/definitions/Timestamp"
        }
      },
      "required": ["mission_id", "destination"]
    }
  },
  "required": ["type", "command_id", "timestamp", "priority", "mission"],
  "additionalProperties": false
}
```

**Example:**
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

---

### 4.2 Command Acknowledgment

**WebSocket Message Type:** `command_ack`

**TypeScript Interface:**
```typescript
interface CommandAcknowledgment {
  type: "command_ack";
  command_id: string;
  status: "accepted" | "rejected" | "executed";
  reason?: string;
  message?: string;
  estimated_arrival?: string;
  stopped_at?: Location;
}
```

**JSON Schema:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "type": {
      "const": "command_ack"
    },
    "command_id": {
      "type": "string"
    },
    "status": {
      "type": "string",
      "enum": ["accepted", "rejected", "executed"]
    },
    "reason": {
      "type": "string",
      "enum": ["battery_too_low", "already_on_mission", "in_error_state", "outside_operational_hours", "invalid_destination"]
    },
    "message": {
      "type": "string",
      "maxLength": 200
    },
    "estimated_arrival": {
      "$ref": "#/definitions/Timestamp"
    },
    "stopped_at": {
      "$ref": "#/definitions/Location"
    }
  },
  "required": ["type", "command_id", "status"],
  "additionalProperties": false
}
```

**Example (Accepted):**
```json
{
  "type": "command_ack",
  "command_id": "cmd_12345",
  "status": "accepted",
  "estimated_arrival": "2025-12-16T10:55:00.000Z",
  "message": "Mission accepted, starting navigation"
}
```

**Example (Rejected):**
```json
{
  "type": "command_ack",
  "command_id": "cmd_12345",
  "status": "rejected",
  "reason": "battery_too_low",
  "message": "Battery at 15%, insufficient for mission"
}
```

---

### 4.3 Emergency Stop Command

**WebSocket Message Type:** `emergency_stop`

**TypeScript Interface:**
```typescript
interface EmergencyStopCommand {
  type: "emergency_stop";
  command_id: string;
  timestamp: string;
  reason: string;
  message?: string;
}
```

**JSON Schema:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "type": {
      "const": "emergency_stop"
    },
    "command_id": {
      "type": "string",
      "pattern": "^cmd_ESTOP_[a-zA-Z0-9]+$"
    },
    "timestamp": {
      "$ref": "#/definitions/Timestamp"
    },
    "reason": {
      "type": "string",
      "maxLength": 200
    },
    "message": {
      "type": "string",
      "maxLength": 500
    }
  },
  "required": ["type", "command_id", "timestamp", "reason"],
  "additionalProperties": false
}
```

**Example:**
```json
{
  "type": "emergency_stop",
  "command_id": "cmd_ESTOP_34567",
  "timestamp": "2025-12-16T10:50:00.000Z",
  "reason": "staff_initiated",
  "message": "Emergency stop requested by operator"
}
```

---

## 5. Error Models

### 5.1 HTTP Error Response

**TypeScript Interface:**
```typescript
interface HTTPError {
  error: string;
  message: string;
  field?: string;
  received_value?: any;
  timestamp: string;
}
```

**JSON Schema:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "error": {
      "type": "string",
      "description": "Machine-readable error code"
    },
    "message": {
      "type": "string",
      "description": "Human-readable error message"
    },
    "field": {
      "type": "string",
      "description": "Field that caused the error (if applicable)"
    },
    "received_value": {
      "description": "Invalid value received (any type)"
    },
    "timestamp": {
      "$ref": "#/definitions/Timestamp"
    }
  },
  "required": ["error", "message", "timestamp"],
  "additionalProperties": false
}
```

**Example:**
```json
{
  "error": "invalid_location",
  "message": "Latitude out of range",
  "field": "location.latitude",
  "received_value": 95.123,
  "timestamp": "2025-12-16T10:30:45.234Z"
}
```

---

## 6. JSON Schema Definitions

### 6.1 Complete Schema Bundle

**Filename:** `tvm_api_schemas.json`

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "definitions": {
    "Timestamp": {
      "type": "string",
      "format": "date-time",
      "pattern": "^\\d{4}-\\d{2}-\\d{2}T\\d{2}:\\d{2}:\\d{2}\\.\\d{3}Z$"
    },
    "Location": {
      "type": "object",
      "properties": {
        "latitude": { "type": "number", "minimum": -90, "maximum": 90 },
        "longitude": { "type": "number", "minimum": -180, "maximum": 180 },
        "altitude": { "type": "number" },
        "heading": { "type": "number", "minimum": 0, "maximum": 360 },
        "speed": { "type": "number", "minimum": 0, "maximum": 2 },
        "accuracy": { "type": "number", "minimum": 0, "maximum": 1 }
      },
      "required": ["latitude", "longitude", "heading", "speed", "accuracy"]
    },
    "VehicleState": {
      "type": "string",
      "enum": ["idle", "navigating", "docking", "charging", "error", "emergency_stop"]
    },
    "ErrorSeverity": {
      "type": "string",
      "enum": ["info", "warning", "error", "critical"]
    },
    "CommandPriority": {
      "type": "string",
      "enum": ["low", "normal", "high", "urgent"]
    }
  }
}
```

---

## 7. Validation Rules

### 7.1 Field Validation

**All implementations MUST validate:**

| Field | Validation Rule | Error if Invalid |
|-------|----------------|------------------|
| `latitude` | -90.0 ≤ value ≤ 90.0 | `invalid_location` |
| `longitude` | -180.0 ≤ value ≤ 180.0 | `invalid_location` |
| `heading` | 0.0 ≤ value ≤ 360.0 | `invalid_heading` |
| `speed` | 0.0 ≤ value ≤ 2.0 | `invalid_speed` |
| `timestamp` | Valid ISO 8601 UTC, not in future | `invalid_timestamp` |
| `soc_percent` | 0.0 ≤ value ≤ 100.0 | `invalid_battery` |
| `vehicle_id` | Matches registered vehicle | `unknown_vehicle` |
| `command_id` | Unique, not reused | `duplicate_command` |

---

### 7.2 Timestamp Validation

**Rules:**
1. MUST be valid ISO 8601 UTC format
2. MUST have millisecond precision (3 decimal places)
3. MUST have 'Z' suffix (UTC timezone)
4. MUST NOT be in the future (allow 5 second clock skew)
5. SHOULD be recent (warn if >1 hour old)

**Python Validation Example:**
```python
from datetime import datetime, timedelta

def validate_timestamp(timestamp_str: str) -> bool:
    try:
        ts = datetime.fromisoformat(timestamp_str.replace('Z', '+00:00'))
        now = datetime.utcnow()

        # Not in future (allow 5s skew)
        if ts > now + timedelta(seconds=5):
            raise ValueError("Timestamp in future")

        # Not too old (warn only)
        if ts < now - timedelta(hours=1):
            print("WARNING: Timestamp >1 hour old")

        return True
    except Exception as e:
        raise ValueError(f"Invalid timestamp: {e}")
```

---

### 7.3 Location Validation

**Rules:**
1. Latitude: -90 to +90 degrees
2. Longitude: -180 to +180 degrees
3. Accuracy: 0 to 1.0 meters (reasonable for outdoor NDT localization)
4. Speed: 0 to 2.0 m/s (max vehicle speed)

**Python Validation Example:**
```python
def validate_location(location: dict) -> bool:
    if not (-90 <= location['latitude'] <= 90):
        raise ValueError("Invalid latitude")
    if not (-180 <= location['longitude'] <= 180):
        raise ValueError("Invalid longitude")
    if not (0 <= location['speed'] <= 2.0):
        raise ValueError("Invalid speed")
    if not (0 <= location['accuracy'] <= 1.0):
        raise ValueError("Invalid accuracy")
    return True
```

---

## 8. Enum Definitions

### 8.1 Vehicle State

```typescript
enum VehicleState {
  IDLE = "idle",
  NAVIGATING = "navigating",
  DOCKING = "docking",
  CHARGING = "charging",
  ERROR = "error",
  EMERGENCY_STOP = "emergency_stop"
}
```

---

### 8.2 Error Severity

```typescript
enum ErrorSeverity {
  INFO = "info",
  WARNING = "warning",
  ERROR = "error",
  CRITICAL = "critical"
}
```

---

### 8.3 Command Priority

```typescript
enum CommandPriority {
  LOW = "low",
  NORMAL = "normal",
  HIGH = "high",
  URGENT = "urgent"
}
```

---

### 8.4 Docking Phase

```typescript
enum DockingPhase {
  APPROACHING = "approaching",
  MARKER_DETECTION = "marker_detection",
  VISUAL_SERVOING = "visual_servoing",
  FINAL_APPROACH = "final_approach",
  ATTACHMENT = "attachment",
  SUCCESS = "success",
  FAILED = "failed"
}
```

---

### 8.5 Error Codes

**Common Error Codes:**

| Code | Subsystem | Severity | Description |
|------|-----------|----------|-------------|
| `NAV_OBSTACLE_BLOCKED` | navigation | warning | Path blocked by obstacle |
| `NAV_LOCALIZATION_LOST` | navigation | error | Localization confidence too low |
| `NAV_PATH_NOT_FOUND` | navigation | error | No valid path to destination |
| `DOCK_ALIGNMENT_FAILED` | docking | warning | Docking alignment failed |
| `DOCK_MARKER_NOT_FOUND` | docking | error | ArUco marker not detected |
| `DOCK_ATTACHMENT_FAILED` | docking | error | Physical attachment failed |
| `SAFE_ESTOP_ACTIVATED` | safety | critical | Emergency stop triggered |
| `SAFE_COLLISION_DETECTED` | safety | critical | Collision detected by bumper |
| `POWER_BATTERY_CRITICAL` | power | critical | Battery below 10% |
| `SENSOR_LIDAR_FAILURE` | hardware | error | LiDAR sensor failure |
| `SENSOR_CAMERA_FAILURE` | hardware | error | Camera sensor failure |
| `SENSOR_IMU_FAILURE` | hardware | error | IMU sensor failure |

---

## Document Status

**Status:** Draft - Pending Review
**Next Review:** 2025-12-17 (Joint review with TVM_API_SPECIFICATION.md)
**Sign-Off Required:** Both Pankaj (Vehicle) and Unno (TVM Server)

---

## Appendices

### Appendix A: Python Pydantic Models

**Example Pydantic models for Python validation:**

```python
from pydantic import BaseModel, Field, validator
from datetime import datetime
from typing import Optional, Literal

class Location(BaseModel):
    latitude: float = Field(..., ge=-90, le=90)
    longitude: float = Field(..., ge=-180, le=180)
    altitude: Optional[float] = None
    heading: float = Field(..., ge=0, le=360)
    speed: float = Field(..., ge=0, le=2.0)
    accuracy: float = Field(..., ge=0, le=1.0)

class LocationUpdate(BaseModel):
    timestamp: datetime
    location: Location

    @validator('timestamp')
    def timestamp_not_future(cls, v):
        if v > datetime.utcnow():
            raise ValueError('Timestamp cannot be in future')
        return v

# Usage:
try:
    update = LocationUpdate.parse_obj({
        "timestamp": "2025-12-16T10:30:45.123Z",
        "location": {
            "latitude": 35.681236,
            "longitude": 139.767125,
            "heading": 90.0,
            "speed": 0.8,
            "accuracy": 0.15
        }
    })
    print("Valid!")
except Exception as e:
    print(f"Invalid: {e}")
```

---

### Appendix B: TypeScript Validation

**Example Zod schemas for TypeScript:**

```typescript
import { z } from 'zod';

const LocationSchema = z.object({
  latitude: z.number().min(-90).max(90),
  longitude: z.number().min(-180).max(180),
  altitude: z.number().optional(),
  heading: z.number().min(0).max(360),
  speed: z.number().min(0).max(2.0),
  accuracy: z.number().min(0).max(1.0),
});

const LocationUpdateSchema = z.object({
  timestamp: z.string().datetime(),
  location: LocationSchema,
});

// Usage:
try {
  const update = LocationUpdateSchema.parse({
    timestamp: "2025-12-16T10:30:45.123Z",
    location: {
      latitude: 35.681236,
      longitude: 139.767125,
      heading: 90.0,
      speed: 0.8,
      accuracy: 0.15,
    },
  });
  console.log("Valid!");
} catch (e) {
  console.error("Invalid:", e.errors);
}
```

---

**END OF DOCUMENT**

**Next Steps:**
1. Review alongside TVM_API_SPECIFICATION.md
2. Validate JSON schemas in both vehicle client and TVM server
3. Implement validation in code (Pydantic/Zod)
4. Test message validation with mock data
