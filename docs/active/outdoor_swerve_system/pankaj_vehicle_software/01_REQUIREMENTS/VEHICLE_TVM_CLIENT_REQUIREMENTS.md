# Vehicle TVM Client Requirements

**Document Control:**
- **Document ID:** VEH-TVM-REQ-001
- **Version:** 1.0
- **Date:** 2025-12-16
- **Status:** Draft
- **Owner:** Pankaj (Vehicle Software Team)
- **Reviewers:** Unno (TVM Server Integration), System Architect

---

## 1. Introduction

### 1.1 Purpose

This document specifies the requirements for the Vehicle TVM Client, the vehicle-side software component that communicates with the TVM (Total Vehicle Management) Server. The client is responsible for publishing vehicle telemetry (location, status, battery, errors, docking updates) to the TVM Server and receiving mission commands via REST API and WebSocket connections.

### 1.2 Scope

**In Scope:**
- REST API client for telemetry publishing
- WebSocket client for real-time command reception
- Offline operation with message queuing
- JWT authentication and token management
- Message validation and error handling
- Integration with vehicle ROS 2 system
- Retry logic and connection recovery

**Out of Scope:**
- TVM Server backend implementation (covered in TVM_SERVER_REQUIREMENTS.md)
- Fleet Management frontend (covered in FLEET_MANAGEMENT_REQUIREMENTS.md)
- Vehicle navigation, docking, and perception (covered in other VEHICLE requirements)

### 1.3 Definitions and Acronyms

| Term | Definition |
|------|------------|
| TVM Client | Vehicle-side software communicating with TVM Server |
| TVM Server | Fleet management backend server |
| Telemetry | Vehicle sensor data (location, battery, status) |
| Offline Queue | Local message buffer when TVM Server unreachable |
| JWT | JSON Web Token (authentication) |
| Heartbeat | Periodic ping to maintain WebSocket connection |

### 1.4 References

- TVM_API_SPECIFICATION.md - Complete API contract (7 REST endpoints, 5 WebSocket commands)
- TVM_DATA_MODELS.md - JSON schemas and validation rules
- TVM_SERVER_REQUIREMENTS.md - Server-side implementation
- FLEET_MANAGEMENT_REQUIREMENTS.md - Frontend application

---

## 2. System Overview

### 2.1 Architecture

The TVM Client is a ROS 2 node that:
- Subscribes to vehicle ROS 2 topics (location, battery, status, errors, docking)
- Publishes telemetry to TVM Server via REST API at specified frequencies
- Receives mission commands from TVM Server via WebSocket and publishes to ROS 2
- Queues messages locally when offline (TVM Server unreachable)
- Uploads queued messages via bulk upload when connection restored
- Manages JWT token lifecycle (authentication, refresh)

### 2.2 Technology Stack

**Implementation Language:** Python 3.10+ (recommended for ROS 2 Humble compatibility)

**Libraries:**
- ROS 2 Humble (rclpy for ROS 2 node)
- requests (REST API client)
- websockets or python-socketio (WebSocket client)
- pyjwt (JWT token handling)
- pydantic (message validation against JSON schemas)

---

## 3. Functional Requirements

### 3.1 REST API Client

#### 3.1.1 Location Update Publishing

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| VEH-TVM-LOC-001 | CRITICAL | Client SHALL publish location updates to TVM Server at 1 Hz | • Subscribe to ROS 2 topic: /tvm/location<br>• POST to /api/v1/vehicle/{vehicle_id}/location every 1 second<br>• Rate limiting: max 1 req/s |
| VEH-TVM-LOC-002 | HIGH | Client SHALL validate location data before sending | • Use LocationUpdate schema from TVM_DATA_MODELS.md<br>• Pydantic validation<br>• Skip invalid messages (log error) |
| VEH-TVM-LOC-003 | HIGH | Client SHALL include accurate timestamp in location updates | • ISO 8601 UTC format with millisecond precision<br>• Use vehicle system time (NTP synchronized)<br>• Timestamp at time of measurement, not send time |
| VEH-TVM-LOC-004 | MEDIUM | Client SHALL queue location updates when offline | • Store in local queue (max 1000 messages)<br>• FIFO eviction if queue full<br>• Upload via bulk endpoint when connection restored |

#### 3.1.2 Vehicle Status Publishing

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| VEH-TVM-STATUS-001 | CRITICAL | Client SHALL publish vehicle status updates on state changes | • Subscribe to ROS 2 topic: /tvm/status<br>• POST to /api/v1/vehicle/{vehicle_id}/status on state change (idle, navigating, docking, charging, error, emergency_stop)<br>• Event-driven (not periodic) |
| VEH-TVM-STATUS-002 | HIGH | Client SHALL include mission_id in status updates when in mission | • Link status to active mission<br>• Empty/null when idle<br>• Validated against UUID format |

#### 3.1.3 Battery Update Publishing

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| VEH-TVM-BAT-001 | HIGH | Client SHALL publish battery updates every 5 seconds | • Subscribe to ROS 2 topic: /power/battery_state<br>• POST to /api/v1/vehicle/{vehicle_id}/battery every 5s<br>• Rate limiting: max 1 req/5s |
| VEH-TVM-BAT-002 | MEDIUM | Client SHALL include all battery metrics | • SoC (0-100%), voltage, current, temperature, charging status<br>• Validated per BatteryUpdate schema<br>• Precision: SoC ±1%, voltage ±0.1V, temperature ±0.5°C |

#### 3.1.4 Error Report Publishing

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| VEH-TVM-ERR-001 | CRITICAL | Client SHALL publish error reports immediately when errors occur | • Subscribe to ROS 2 topic: /tvm/error<br>• POST to /api/v1/vehicle/{vehicle_id}/error within 1 second of error<br>• Event-driven |
| VEH-TVM-ERR-002 | HIGH | Client SHALL include error severity and subsystem | • Severity: INFO, WARNING, ERROR, CRITICAL<br>• Subsystem: navigation, perception, docking, power, communication<br>• Validated per ErrorReport schema |

#### 3.1.5 Docking Update Publishing

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| VEH-TVM-DOCK-001 | HIGH | Client SHALL publish docking updates during docking operations | • Subscribe to ROS 2 topic: /tvm/docking<br>• POST to /api/v1/vehicle/{vehicle_id}/docking on phase changes (approaching, aligning, docking, docked)<br>• Event-driven |
| VEH-TVM-DOCK-002 | MEDIUM | Client SHALL include ArUco marker detection results | • Marker ID, distance, alignment error<br>• Helps TVM Server track docking performance<br>• Optional fields (null if marker not detected) |

#### 3.1.6 Bulk Upload (Offline Recovery)

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| VEH-TVM-BULK-001 | HIGH | Client SHALL upload queued messages via bulk endpoint when connection restored | • POST to /api/v1/vehicle/{vehicle_id}/bulk<br>• Max 1000 messages per request<br>• Retry on failure (3 attempts) |
| VEH-TVM-BULK-002 | HIGH | Client SHALL clear queued messages after successful upload | • Remove messages from local queue<br>• Log success count<br>• Resume normal telemetry publishing |
| VEH-TVM-BULK-003 | MEDIUM | Client SHALL deduplicate queued messages before bulk upload | • Check for duplicate timestamps<br>• Keep most recent message if duplicates exist<br>• Reduces server load |

---

### 3.2 WebSocket Client

#### 3.2.1 Connection Management

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| VEH-TVM-WS-CONN-001 | CRITICAL | Client SHALL establish WebSocket connection to TVM Server at startup | • Connect to wss://{server}/ws/vehicle/{vehicle_id}<br>• Include JWT token in query parameter or header<br>• Retry on connection failure (exponential backoff) |
| VEH-TVM-WS-CONN-002 | HIGH | Client SHALL implement connection heartbeat | • Respond to server ping within 5 seconds<br>• Send ping every 30 seconds if no messages received<br>• Detect disconnect if no response to ping |
| VEH-TVM-WS-CONN-003 | HIGH | Client SHALL auto-reconnect on WebSocket disconnection | • Exponential backoff: 1s, 2s, 4s, 8s, ... max 60s<br>• Log disconnect reason<br>• Resume normal operation after reconnect |

#### 3.2.2 Command Reception

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| VEH-TVM-WS-CMD-001 | CRITICAL | Client SHALL receive "dispatch" commands from TVM Server | • Parse dispatch message (mission_id, pickup, destination, priority)<br>• Publish to ROS 2 topic: /navigation/mission/dispatch<br>• Acknowledge command within 2 seconds |
| VEH-TVM-WS-CMD-002 | CRITICAL | Client SHALL receive "cancel" commands from TVM Server | • Parse cancel message (mission_id)<br>• Publish to ROS 2 topic: /navigation/mission/cancel<br>• Acknowledge command within 2 seconds |
| VEH-TVM-WS-CMD-003 | CRITICAL | Client SHALL receive "emergency_stop" commands from TVM Server | • Publish to ROS 2 topic: /safety/emergency_stop (highest priority)<br>• Acknowledge command within 500ms<br>• Immediate action (no validation delay) |
| VEH-TVM-WS-CMD-004 | HIGH | Client SHALL receive "resume" commands from TVM Server | • Publish to ROS 2 topic: /safety/resume<br>• Acknowledge command<br>• Only valid after emergency_stop |
| VEH-TVM-WS-CMD-005 | MEDIUM | Client SHALL receive "configure" commands from TVM Server | • Parse configuration updates (speed_limit, operational_hours)<br>• Publish to ROS 2 topic: /config/update<br>• Acknowledge with updated config |

#### 3.2.3 Message Validation

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| VEH-TVM-WS-VAL-001 | HIGH | Client SHALL validate all WebSocket messages against JSON schemas | • Use schemas from TVM_DATA_MODELS.md<br>• Pydantic validation<br>• Reject invalid messages (send error response) |
| VEH-TVM-WS-VAL-002 | MEDIUM | Client SHALL validate command IDs are unique | • Check command_id not duplicate of recent commands (last 100)<br>• Ignore duplicate commands<br>• Log warning |

---

### 3.3 Authentication and Token Management

#### 3.3.1 JWT Token Acquisition

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| VEH-TVM-AUTH-001 | CRITICAL | Client SHALL authenticate with TVM Server at startup | • POST to /api/v1/auth/token with client_id and client_secret<br>• Receive JWT token (24h expiration) + refresh token<br>• Store tokens securely (in-memory, not written to disk) |
| VEH-TVM-AUTH-002 | HIGH | Client SHALL include JWT token in all REST API requests | • Authorization header: "Bearer {token}"<br>• Request fails with 401 if token missing or invalid<br>• Auto-refresh token if 401 received |

#### 3.3.2 Token Refresh

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| VEH-TVM-AUTH-REF-001 | HIGH | Client SHALL automatically refresh JWT token before expiration | • Refresh at 80% of token lifetime (19.2 hours for 24h token)<br>• POST to /api/v1/auth/refresh with refresh token<br>• Update stored JWT token |
| VEH-TVM-AUTH-REF-002 | HIGH | Client SHALL retry authentication if token refresh fails | • Retry with exponential backoff (1s, 2s, 4s)<br>• Max 5 attempts<br>• Log critical error if all retries fail |

---

### 3.4 Offline Operation

#### 3.4.1 Connection Detection

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| VEH-TVM-OFFLINE-001 | HIGH | Client SHALL detect when TVM Server is unreachable | • Connection timeout: 10 seconds<br>• Retry 3 times before declaring offline<br>• Log offline state |
| VEH-TVM-OFFLINE-002 | MEDIUM | Client SHALL periodically check for TVM Server availability when offline | • Retry connection every 60 seconds<br>• Resume normal operation when connection restored<br>• Log online state |

#### 3.4.2 Message Queuing

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| VEH-TVM-QUEUE-001 | HIGH | Client SHALL queue telemetry messages when TVM Server unreachable | • Max queue size: 1000 messages<br>• FIFO eviction if queue full (drop oldest messages)<br>• Queue stored in memory (ephemeral, lost on restart) |
| VEH-TVM-QUEUE-002 | MEDIUM | Client SHALL prioritize error reports in offline queue | • Error reports kept in queue even if queue full<br>• Drop location/battery updates first when queue full<br>• Ensures critical errors are not lost |

---

### 3.5 Error Handling and Retry Logic

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| VEH-TVM-ERR-HANDLE-001 | HIGH | Client SHALL implement exponential backoff for failed requests | • Backoff: 1s, 2s, 4s, 8s, 16s, ... max 60s<br>• Max 10 retry attempts<br>• After 10 failures, queue message for bulk upload |
| VEH-TVM-ERR-HANDLE-002 | HIGH | Client SHALL log all communication errors | • Log level: ERROR for connection failures, WARNING for retries<br>• Include: timestamp, endpoint, error message, retry count<br>• Structured logging (JSON format) |
| VEH-TVM-ERR-HANDLE-003 | MEDIUM | Client SHALL publish TVM connection status to ROS 2 | • Topic: /tvm/connection_status (std_msgs/Bool)<br>• True = connected, False = offline<br>• Updated on state change |

---

## 4. Non-Functional Requirements

### 4.1 Performance

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| VEH-TVM-PERF-001 | HIGH | Client SHALL publish location updates at 1 Hz consistently | • Measured latency: <100ms from ROS 2 topic receive to REST POST<br>• No dropped messages under normal conditions<br>• CPU usage <5% on target hardware |
| VEH-TVM-PERF-002 | MEDIUM | Client SHALL handle WebSocket commands with <500ms latency | • From WebSocket receive to ROS 2 publish: <500ms<br>• Command processing does not block telemetry publishing |

### 4.2 Reliability

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| VEH-TVM-REL-001 | HIGH | Client SHALL recover from network outages automatically | • Auto-reconnect after network restored<br>• Upload queued messages via bulk endpoint<br>• Resume normal operation (no manual intervention) |
| VEH-TVM-REL-002 | MEDIUM | Client SHALL run as a ROS 2 lifecycle node (optional) | • Supports configure, activate, deactivate, cleanup states<br>• Graceful shutdown on SIGTERM<br>• Flush queue before shutdown |

### 4.3 Security

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| VEH-TVM-SEC-001 | CRITICAL | Client SHALL use HTTPS and WSS (secure WebSocket) | • All REST API calls use https://<br>• WebSocket connection uses wss://<br>• Reject unencrypted connections |
| VEH-TVM-SEC-002 | HIGH | Client SHALL validate TVM Server SSL certificate | • Certificate validation enabled (not self-signed unless testing)<br>• Reject connection if certificate invalid<br>• Configurable certificate bundle path |
| VEH-TVM-SEC-003 | HIGH | Client SHALL not log sensitive data (JWT tokens, secrets) | • Tokens redacted in logs (show only first 8 chars)<br>• client_secret never logged<br>• Log sanitization |

---

## 5. Interface Requirements

### 5.1 ROS 2 Topics (Input - Vehicle → TVM Client)

| Topic | Message Type | Frequency | Description |
|-------|--------------|-----------|-------------|
| /tvm/location | custom_msgs/LocationUpdate | 1 Hz | Vehicle location, heading, speed |
| /tvm/status | custom_msgs/VehicleStatus | event | Vehicle state changes |
| /power/battery_state | sensor_msgs/BatteryState | 0.2 Hz | Battery SoC, voltage, current, temp |
| /tvm/error | custom_msgs/ErrorReport | event | Error reports |
| /tvm/docking | custom_msgs/DockingUpdate | event | Docking phase updates |

**Note:** Custom message definitions SHALL match JSON schemas in TVM_DATA_MODELS.md

### 5.2 ROS 2 Topics (Output - TVM Client → Vehicle)

| Topic | Message Type | Description |
|-------|--------------|-------------|
| /navigation/mission/dispatch | custom_msgs/MissionDispatch | New mission from TVM Server |
| /navigation/mission/cancel | std_msgs/String | Cancel mission (mission_id) |
| /safety/emergency_stop | std_msgs/Empty | Emergency stop command |
| /safety/resume | std_msgs/Empty | Resume from emergency stop |
| /config/update | custom_msgs/ConfigUpdate | Configuration updates |
| /tvm/connection_status | std_msgs/Bool | TVM Server connection status |

### 5.3 Configuration Parameters (ROS 2 Parameters)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| tvm_server_url | string | "https://tvm.example.com" | TVM Server base URL |
| vehicle_id | string | "vehicle_001" | Unique vehicle identifier |
| client_id | string | "vehicle_001" | Authentication client ID |
| client_secret | string | "" | Authentication secret (load from env var) |
| queue_max_size | int | 1000 | Max offline queue size |
| retry_max_attempts | int | 10 | Max retry attempts for failed requests |

---

## 6. Testing Requirements

| Test Type | Coverage | Acceptance |
|-----------|----------|------------|
| Unit Tests | ≥80% code coverage | All tests pass, critical paths 100% |
| Integration Tests | TVM Client ↔ Mock TVM Server | All API endpoints tested |
| Offline Tests | Network disconnect/reconnect scenarios | Queue and bulk upload work correctly |
| Load Tests | 1 Hz telemetry for 8 hours | No memory leaks, stable CPU usage |

---

## 7. Traceability Matrix

| Requirement Category | Requirement Count | Priority Distribution |
|---------------------|-------------------|----------------------|
| REST API Client | 16 | CRITICAL: 3, HIGH: 10, MEDIUM: 3 |
| WebSocket Client | 11 | CRITICAL: 4, HIGH: 5, MEDIUM: 2 |
| Authentication | 4 | CRITICAL: 1, HIGH: 3 |
| Offline Operation | 4 | HIGH: 2, MEDIUM: 2 |
| Error Handling | 3 | HIGH: 2, MEDIUM: 1 |
| Non-Functional (Performance) | 2 | HIGH: 1, MEDIUM: 1 |
| Non-Functional (Reliability) | 2 | HIGH: 1, MEDIUM: 1 |
| Non-Functional (Security) | 3 | CRITICAL: 1, HIGH: 2 |
| **TOTAL** | **45** | **CRITICAL: 9, HIGH: 26, MEDIUM: 10** |

---

## 8. Assumptions and Dependencies

### 8.1 Assumptions

1. Vehicle has reliable internet connectivity (Wi-Fi or 4G/5G)
2. Vehicle system time is NTP-synchronized
3. ROS 2 Humble installed on vehicle compute unit
4. Python 3.10+ available

### 8.2 Dependencies

1. TVM_API_SPECIFICATION.md finalized (Week 1 ✅)
2. TVM_DATA_MODELS.md finalized (Week 1 ✅)
3. TVM Server deployed and accessible (TVM_SERVER_REQUIREMENTS.md)
4. Vehicle ROS 2 system publishes required topics (VEHICLE requirements)

---

## 9. Open Issues and Risks

### 9.1 Open Issues

| ID | Issue | Owner | Target Resolution |
|----|-------|-------|-------------------|
| VEH-TVM-ISSUE-001 | Custom ROS 2 message definitions need creation | Pankaj | Week 7 (implementation) |
| VEH-TVM-ISSUE-002 | TVM Server URL configuration (dev vs production) | Pankaj | Week 7 |

### 9.2 Risks

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| Network latency >1s affects 1 Hz publishing | MEDIUM | MEDIUM | Implement asynchronous publishing (non-blocking) |
| WebSocket connection drops frequently | LOW | HIGH | Robust reconnect logic, exponential backoff |
| Queue fills up during long offline periods | LOW | MEDIUM | Increase queue size or implement persistent queue (disk storage) |

---

## 10. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-16 | Pankaj | Initial draft - Week 2 documentation |

---

**END OF DOCUMENT**

**Total Requirements:** 45
**Document Status:** Draft for Review
**Next Review:** Week 3 (with Unno for TVM Server integration testing)
