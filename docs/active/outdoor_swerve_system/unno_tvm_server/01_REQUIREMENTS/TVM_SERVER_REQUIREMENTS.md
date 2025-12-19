# TVM Server Backend Requirements

**Document Control:**
- **Document ID:** TVM-SRV-REQ-001
- **Version:** 1.0
- **Date:** 2025-12-16
- **Status:** Draft
- **Owner:** Unno (Fleet Management Team)
- **Reviewers:** Pankaj (Vehicle Integration), System Architect

---

## 1. Introduction

### 1.1 Purpose

This document specifies the requirements for the TVM (Total Vehicle Management) Server backend, which implements the server-side API contract defined in TVM_API_SPECIFICATION.md. The TVM Server handles vehicle telemetry ingestion, command dispatch, data persistence, and serves the Fleet Management frontend application.

### 1.2 Scope

**In Scope:**
- REST API endpoint implementation (7 endpoints as defined in TVM_API_SPECIFICATION.md)
- WebSocket server for real-time vehicle commands
- Database schema design and management
- JWT authentication and authorization
- Message validation against JSON schemas
- Rate limiting and offline operation support
- Error handling and logging
- Performance optimization and scalability

**Out of Scope:**
- Fleet Management frontend UI (covered in FLEET_MANAGEMENT_REQUIREMENTS.md)
- Vehicle-side TVM client (covered in VEHICLE_TVM_CLIENT_REQUIREMENTS.md)
- Hardware infrastructure (servers, load balancers)
- Database administration (backups, replication - operational concern)

### 1.3 Definitions and Acronyms

| Term | Definition |
|------|------------|
| TVM Server | Backend server implementing TVM API |
| REST API | REpresentational State Transfer API (HTTP-based) |
| WebSocket | Bidirectional communication protocol |
| JWT | JSON Web Token (authentication) |
| Rate Limiting | Throttling request frequency |
| Bulk Upload | Offline vehicle data upload |
| Telemetry | Vehicle sensor data (location, battery, etc.) |

### 1.4 References

- TVM_API_SPECIFICATION.md - Complete API contract (7 REST endpoints, 5 WebSocket commands)
- TVM_DATA_MODELS.md - JSON schemas and validation rules
- FLEET_MANAGEMENT_REQUIREMENTS.md - Frontend application requirements
- VEHICLE_TVM_CLIENT_REQUIREMENTS.md - Vehicle-side client requirements

---

## 2. System Overview

### 2.1 Architecture

The TVM Server is a backend application that:
- Receives vehicle telemetry via REST API (POST requests)
- Stores telemetry in database for historical analysis
- Broadcasts vehicle state changes to Fleet Management frontend via WebSocket
- Receives commands from frontend and dispatches to vehicles via WebSocket
- Validates all messages against JSON schemas
- Implements JWT-based authentication

### 2.2 Technology Stack (Recommendations)

**Backend Framework:**
- Option 1: Python FastAPI (async, high performance, auto-generated OpenAPI docs)
- Option 2: Node.js + Express + Socket.io (JavaScript ecosystem)
- Option 3: Java Spring Boot (enterprise-grade, mature ecosystem)

**Database:**
- PostgreSQL 14+ (ACID compliance, JSON support, mature)

**WebSocket:**
- FastAPI WebSockets / Socket.io / Spring WebSocket

**Caching (optional):**
- Redis (for rate limiting, session storage)

---

## 3. Functional Requirements

### 3.1 REST API Implementation

#### 3.1.1 Location Update Endpoint

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-API-LOC-001 | CRITICAL | Server SHALL implement POST /api/v1/vehicle/{vehicle_id}/location | • Endpoint accepts JSON payload as per TVM_DATA_MODELS.md<br>• Validates against LocationUpdate schema<br>• Returns 200 with received_at timestamp |
| TVM-API-LOC-002 | HIGH | Server SHALL enforce 1 Hz rate limit for location updates | • Max 1 request per second per vehicle<br>• Returns 429 Too Many Requests if exceeded<br>• Rate limit tracked in Redis (or in-memory) |
| TVM-API-LOC-003 | HIGH | Server SHALL store location updates in database | • Table: vehicle_locations<br>• Columns: vehicle_id, timestamp, latitude, longitude, heading, speed, accuracy<br>• Index on (vehicle_id, timestamp) for fast queries |
| TVM-API-LOC-004 | HIGH | Server SHALL broadcast location updates to connected frontend clients via WebSocket | • Message type: "location_update"<br>• Broadcast within 500ms of receipt<br>• Only to clients subscribed to that vehicle |
| TVM-API-LOC-005 | MEDIUM | Server SHALL reject location updates with invalid timestamps | • Timestamp must be ISO 8601 UTC<br>• Within ±5 minutes of server time<br>• Returns 400 Bad Request with error details |

#### 3.1.2 Vehicle Status Endpoint

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-API-STATUS-001 | CRITICAL | Server SHALL implement POST /api/v1/vehicle/{vehicle_id}/status | • Accepts VehicleStatusUpdate payload<br>• Validates state transitions (idle → navigating → docking → charging)<br>• Returns 200 OK |
| TVM-API-STATUS-002 | HIGH | Server SHALL store vehicle status in database | • Table: vehicle_status<br>• Columns: vehicle_id, timestamp, state, mission_id, error_code, error_message<br>• Current status also cached in Redis for fast access |
| TVM-API-STATUS-003 | HIGH | Server SHALL trigger alerts when vehicle enters error or emergency_stop state | • Publish alert message to message queue (Redis Pub/Sub)<br>• Fleet Management frontend subscribes to alerts<br>• Alert includes: vehicle_id, state, error_code, severity |

#### 3.1.3 Battery Update Endpoint

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-API-BAT-001 | HIGH | Server SHALL implement POST /api/v1/vehicle/{vehicle_id}/battery | • Accepts BatteryUpdate payload<br>• Validates: soc (0-100), voltage (>0), current, temperature<br>• Returns 200 OK |
| TVM-API-BAT-002 | HIGH | Server SHALL store battery updates in database | • Table: vehicle_battery<br>• Rate limit: 1 request per 5 seconds per vehicle<br>• Index on (vehicle_id, timestamp) |
| TVM-API-BAT-003 | MEDIUM | Server SHALL calculate and store battery health metrics | • Cycle count (charge/discharge cycles)<br>• Voltage degradation over time<br>• Used for maintenance alerts |

#### 3.1.4 Error Report Endpoint

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-API-ERR-001 | CRITICAL | Server SHALL implement POST /api/v1/vehicle/{vehicle_id}/error | • Accepts ErrorReport payload<br>• Validates error_code against known codes (TVM_DATA_MODELS.md)<br>• Returns 200 OK |
| TVM-API-ERR-002 | HIGH | Server SHALL store error reports in database | • Table: vehicle_errors<br>• Columns: vehicle_id, timestamp, code, severity, subsystem, location, description, resolved_at<br>• Index on (vehicle_id, timestamp, severity) |
| TVM-API-ERR-003 | CRITICAL | Server SHALL trigger immediate alerts for CRITICAL severity errors | • Alert published to message queue<br>• Desktop notification to all connected admins<br>• Email alert (optional, configurable) |

#### 3.1.5 Docking Update Endpoint

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-API-DOCK-001 | HIGH | Server SHALL implement POST /api/v1/vehicle/{vehicle_id}/docking | • Accepts DockingUpdate payload<br>• Validates phase transitions (approaching → aligning → docking → docked)<br>• Returns 200 OK |
| TVM-API-DOCK-002 | MEDIUM | Server SHALL store docking updates in database | • Table: vehicle_docking<br>• Used for docking success rate analytics<br>• Link to mission_id |

#### 3.1.6 Bulk Upload Endpoint

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-API-BULK-001 | HIGH | Server SHALL implement POST /api/v1/vehicle/{vehicle_id}/bulk | • Accepts array of mixed telemetry messages<br>• Max 1000 messages per request<br>• Returns 200 OK with processed count |
| TVM-API-BULK-002 | HIGH | Server SHALL process bulk uploads atomically | • All messages inserted in single database transaction<br>• Rollback if any message validation fails<br>• Returns 400 with first error if validation fails |
| TVM-API-BULK-003 | MEDIUM | Server SHALL deduplicate bulk uploads based on timestamp | • Check if message with same vehicle_id + timestamp already exists<br>• Skip duplicate messages<br>• Return count of inserted vs skipped |

#### 3.1.7 Authentication Endpoint

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-API-AUTH-001 | CRITICAL | Server SHALL implement POST /api/v1/auth/token | • Accepts client_id and client_secret<br>• Validates credentials against database<br>• Returns JWT token (24h expiration) + refresh token |
| TVM-API-AUTH-002 | CRITICAL | Server SHALL implement JWT token validation middleware | • Extract JWT from Authorization header (Bearer scheme)<br>• Verify signature and expiration<br>• Reject invalid/expired tokens with 401 Unauthorized |
| TVM-API-AUTH-003 | HIGH | Server SHALL implement token refresh mechanism | • POST /api/v1/auth/refresh endpoint<br>• Accepts refresh token<br>• Issues new JWT token (24h)<br>• Refresh token valid for 30 days |

---

### 3.2 WebSocket Server Implementation

#### 3.2.1 Connection Management

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-WS-CONN-001 | CRITICAL | Server SHALL implement WebSocket endpoint at /ws/vehicle/{vehicle_id} | • Accepts WebSocket upgrade requests<br>• Validates JWT token in query parameter or header<br>• Maintains connection for bidirectional messaging |
| TVM-WS-CONN-002 | HIGH | Server SHALL support multiple concurrent WebSocket connections per vehicle | • Both vehicle and frontend can connect simultaneously<br>• Vehicle connection: sends telemetry, receives commands<br>• Frontend connection: receives telemetry, sends commands |
| TVM-WS-CONN-003 | HIGH | Server SHALL implement connection heartbeat/ping-pong | • Server sends ping every 30 seconds<br>• Expects pong within 10 seconds<br>• Close connection if no pong received |
| TVM-WS-CONN-004 | MEDIUM | Server SHALL handle connection drops gracefully | • Queue messages for disconnected vehicle (max 100 messages)<br>• Deliver queued messages on reconnect<br>• Discard oldest messages if queue full |

#### 3.2.2 Command Dispatch

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-WS-CMD-001 | CRITICAL | Server SHALL implement "dispatch" command forwarding | • Frontend sends dispatch command → Server validates → Forward to vehicle WebSocket<br>• Command includes: mission_id, pickup, destination, priority<br>• Timeout 5 seconds, retry if vehicle doesn't acknowledge |
| TVM-WS-CMD-002 | CRITICAL | Server SHALL implement "cancel" command forwarding | • Same flow as dispatch<br>• Vehicle acknowledges with current mission status |
| TVM-WS-CMD-003 | CRITICAL | Server SHALL implement "emergency_stop" command forwarding | • Highest priority command<br>• No validation delay, immediate forward to vehicle<br>• Logged to audit trail |
| TVM-WS-CMD-004 | HIGH | Server SHALL implement "resume" command forwarding | • Only valid after emergency_stop<br>• Vehicle validates it's safe to resume |
| TVM-WS-CMD-005 | MEDIUM | Server SHALL implement "configure" command forwarding | • Update vehicle settings (speed_limit, operational_hours)<br>• Vehicle acknowledges with updated config |

#### 3.2.3 Message Broadcasting

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-WS-BCAST-001 | HIGH | Server SHALL broadcast vehicle telemetry to all subscribed frontend clients | • When location/status update received via REST, broadcast via WebSocket<br>• Only to clients subscribed to that vehicle or "all vehicles"<br>• Broadcast within 500ms |
| TVM-WS-BCAST-002 | MEDIUM | Server SHALL implement subscription management | • Clients send "subscribe" message with vehicle_id list<br>• Server tracks subscriptions per connection<br>• Unsubscribe on connection close |

---

### 3.3 Database Management

#### 3.3.1 Schema Design

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-DB-SCHEMA-001 | HIGH | Server SHALL implement database schema for vehicle telemetry | • Tables: vehicle_locations, vehicle_status, vehicle_battery, vehicle_errors, vehicle_docking<br>• Foreign key: vehicle_id references vehicles(id)<br>• Indexes on (vehicle_id, timestamp) for all tables |
| TVM-DB-SCHEMA-002 | HIGH | Server SHALL implement database schema for missions and reservations | • Tables: missions, reservations<br>• Link missions to vehicle_id and reservation_id<br>• Store: pickup, destination, start_time, end_time, status |
| TVM-DB-SCHEMA-003 | HIGH | Server SHALL implement database schema for users and authentication | • Tables: users (id, username, email, password_hash, role)<br>• Table: refresh_tokens (token, user_id, expires_at)<br>• Unique constraints on username and email |

#### 3.3.2 Data Access Patterns

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-DB-ACCESS-001 | HIGH | Server SHALL optimize queries for recent vehicle data | • Query latest location per vehicle (1 query using DISTINCT ON or window function)<br>• Query latest status per vehicle<br>• Query execution time <100ms |
| TVM-DB-ACCESS-002 | MEDIUM | Server SHALL implement database connection pooling | • Min 5, max 20 connections<br>• Connection timeout 30s<br>• Monitor connection usage |
| TVM-DB-ACCESS-003 | MEDIUM | Server SHALL implement database migrations | • Use migration tool (Alembic for Python, Flyway for Java, Knex for Node.js)<br>• Version-controlled migration files<br>• Rollback capability |

#### 3.3.3 Data Retention

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-DB-RET-001 | MEDIUM | Server SHALL implement data archival for old telemetry | • Archive vehicle_locations older than 30 days to separate table<br>• Archive vehicle_status older than 90 days<br>• Scheduled job runs daily at 02:00 |
| TVM-DB-RET-002 | MEDIUM | Server SHALL implement audit log retention | • Keep audit_logs for 90 days<br>• Export to cold storage (S3) before deletion<br>• Configurable retention period |

---

### 3.4 Message Validation

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-VAL-001 | CRITICAL | Server SHALL validate all incoming messages against JSON schemas | • Use schemas from TVM_DATA_MODELS.md<br>• Validation library (jsonschema for Python, ajv for Node.js)<br>• Return 400 Bad Request with detailed error if validation fails |
| TVM-VAL-002 | HIGH | Server SHALL validate vehicle_id exists before processing | • Check vehicle_id in database vehicles table<br>• Return 404 Not Found if vehicle doesn't exist<br>• Cache valid vehicle_ids in Redis for fast lookup |
| TVM-VAL-003 | HIGH | Server SHALL validate timestamp format and range | • ISO 8601 UTC format<br>• Within ±5 minutes of server time<br>• Reject future timestamps |
| TVM-VAL-004 | MEDIUM | Server SHALL validate enum values | • VehicleState, ErrorSeverity, CommandPriority, DockingPhase<br>• Return 400 with error message if invalid |

---

### 3.5 Error Handling

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-ERR-001 | HIGH | Server SHALL return standardized error responses | • Format: {"error": {"code": "INVALID_TIMESTAMP", "message": "...", "details": {...}}}<br>• HTTP status codes: 400 (validation), 401 (auth), 403 (permission), 404 (not found), 429 (rate limit), 500 (server error) |
| TVM-ERR-002 | HIGH | Server SHALL log all errors to application log | • Structured logging (JSON format)<br>• Include: timestamp, level, message, vehicle_id, request_id, stack_trace<br>• Log rotation (daily, max 10 files) |
| TVM-ERR-003 | MEDIUM | Server SHALL implement graceful degradation | • If database unavailable, return 503 Service Unavailable<br>• If Redis unavailable, disable rate limiting (allow all requests)<br>• Health check endpoint reflects component status |

---

### 3.6 Security

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-SEC-001 | CRITICAL | Server SHALL use HTTPS for all HTTP communication | • TLS 1.2+ with strong ciphers<br>• Valid SSL certificate<br>• HTTP redirects to HTTPS |
| TVM-SEC-002 | CRITICAL | Server SHALL use WSS (WebSocket Secure) for WebSocket connections | • TLS encryption for WebSocket<br>• Reject unencrypted WebSocket connections |
| TVM-SEC-003 | CRITICAL | Server SHALL implement rate limiting per vehicle | • Location: 1 req/s, Battery: 1 req/5s, Error: event-driven (no limit)<br>• Rate limiter: token bucket or sliding window<br>• Store in Redis for distributed rate limiting |
| TVM-SEC-004 | HIGH | Server SHALL sanitize all inputs to prevent SQL injection | • Use parameterized queries (prepared statements)<br>• ORM (SQLAlchemy, Sequelize, Hibernate) helps prevent SQL injection<br>• No string concatenation in SQL queries |
| TVM-SEC-005 | HIGH | Server SHALL implement CORS policy | • Allow origins: Fleet Management frontend domain<br>• Allow methods: GET, POST, OPTIONS<br>• Allow headers: Authorization, Content-Type |
| TVM-SEC-006 | MEDIUM | Server SHALL implement request ID tracing | • Generate unique request_id for each request<br>• Include in response header (X-Request-ID)<br>• Log with request_id for debugging |

---

## 4. Non-Functional Requirements

### 4.1 Performance

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-PERF-001 | HIGH | Server SHALL handle 10 vehicles sending telemetry at 1 Hz | • 10 location updates/sec sustained<br>• API response time <200ms (p95)<br>• CPU usage <70%, memory <2GB |
| TVM-PERF-002 | HIGH | Server SHALL support 50 concurrent WebSocket connections | • 10 vehicles + 40 frontend clients<br>• Message latency <500ms (p95)<br>• No connection drops |
| TVM-PERF-003 | MEDIUM | Server SHALL handle bulk uploads of 1000 messages in <5 seconds | • Database batch insert optimized<br>• Transaction commit time <5s<br>• Response time <5s |
| TVM-PERF-004 | MEDIUM | Server database queries SHALL execute in <100ms (p95) | • Indexes on commonly queried columns<br>• Query plan optimization<br>• Monitored with slow query log |

### 4.2 Scalability

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-SCALE-001 | MEDIUM | Server SHALL support horizontal scaling behind load balancer | • Stateless server design (state in database/Redis)<br>• WebSocket sticky sessions (load balancer config)<br>• Test with 2+ server instances |
| TVM-SCALE-002 | MEDIUM | Server SHALL support database read replicas (optional) | • Write to primary, read from replica for historical queries<br>• Replication lag <1 second<br>• Automatic failover |

### 4.3 Reliability

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-REL-001 | HIGH | Server SHALL implement health check endpoint | • GET /health returns 200 if healthy<br>• Checks: database connection, Redis connection (if used)<br>• Used by load balancer for liveness probe |
| TVM-REL-002 | HIGH | Server SHALL implement graceful shutdown | • On SIGTERM, stop accepting new connections<br>• Wait for active requests to complete (max 30s)<br>• Close database connections cleanly |
| TVM-REL-003 | MEDIUM | Server SHALL implement automatic retry for transient database errors | • Retry on connection timeout, deadlock<br>• Exponential backoff (1s, 2s, 4s)<br>• Max 3 retries |

### 4.4 Observability

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-OBS-001 | MEDIUM | Server SHALL expose metrics endpoint | • Prometheus-compatible metrics at /metrics<br>• Metrics: request_count, request_duration, active_connections, db_pool_usage<br>• Updated in real-time |
| TVM-OBS-002 | MEDIUM | Server SHALL implement structured logging | • JSON format logs<br>• Include: timestamp, level, message, request_id, vehicle_id, user_id<br>• Forwarded to centralized logging (ELK, Splunk) |
| TVM-OBS-003 | LOW | Server SHALL support distributed tracing (optional) | • OpenTelemetry integration<br>• Trace requests across REST → DB → WebSocket<br>• Send traces to Jaeger/Zipkin |

---

## 5. Interface Requirements

### 5.1 API Compliance

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-INT-001 | CRITICAL | Server SHALL implement API exactly as specified in TVM_API_SPECIFICATION.md | • All 7 REST endpoints<br>• All 5 WebSocket commands<br>• Same request/response formats<br>• Same error codes |
| TVM-INT-002 | HIGH | Server SHALL provide OpenAPI/Swagger documentation | • Auto-generated from code (FastAPI) or manually maintained<br>• Accessible at /docs or /api-docs<br>• Includes examples for all endpoints |

### 5.2 Database Interface

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-INT-DB-001 | HIGH | Server SHALL use database connection string from environment variable | • Format: postgresql://user:pass@host:port/dbname<br>• No hardcoded credentials<br>• Support for connection pooling parameters |

---

## 6. Operational Requirements

### 6.1 Deployment

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-OPS-001 | MEDIUM | Server SHALL provide Dockerfile for containerized deployment | • Multi-stage build for small image size<br>• Non-root user<br>• Exposes port 8000 (HTTP) |
| TVM-OPS-002 | MEDIUM | Server SHALL read configuration from environment variables | • All config (DB URL, JWT secret, rate limits) from env vars<br>• .env.example file provided<br>• Validation on startup |

### 6.2 Documentation

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| TVM-OPS-DOC-001 | HIGH | Server SHALL provide deployment guide | • Installation instructions<br>• Database setup (schema creation)<br>• Environment variable reference<br>• Troubleshooting section |
| TVM-OPS-DOC-002 | MEDIUM | Server SHALL provide developer setup guide | • Local development environment setup<br>• Running tests<br>• Code style guide |

---

## 7. Testing Requirements

| Test Type | Coverage | Acceptance |
|-----------|----------|------------|
| Unit Tests | ≥80% code coverage | All tests pass, critical paths 100% covered |
| Integration Tests | All API endpoints | Test with real database (test DB) |
| WebSocket Tests | Connection, commands, broadcasting | Automated tests with WebSocket client |
| Load Tests | 10 vehicles, 1 Hz for 1 hour | No errors, response time <200ms (p95) |
| Security Tests | OWASP API Top 10 | No critical/high vulnerabilities |

---

## 8. Traceability Matrix

| Requirement Category | Requirement Count | Priority Distribution |
|---------------------|-------------------|----------------------|
| REST API Implementation | 20 | CRITICAL: 5, HIGH: 12, MEDIUM: 3 |
| WebSocket Implementation | 11 | CRITICAL: 4, HIGH: 5, MEDIUM: 2 |
| Database Management | 11 | HIGH: 6, MEDIUM: 5 |
| Message Validation | 4 | CRITICAL: 1, HIGH: 3 |
| Error Handling | 3 | HIGH: 2, MEDIUM: 1 |
| Security | 6 | CRITICAL: 3, HIGH: 2, MEDIUM: 1 |
| Non-Functional (Performance) | 4 | HIGH: 2, MEDIUM: 2 |
| Non-Functional (Scalability) | 2 | MEDIUM: 2 |
| Non-Functional (Reliability) | 3 | HIGH: 2, MEDIUM: 1 |
| Non-Functional (Observability) | 3 | MEDIUM: 2, LOW: 1 |
| Interface Requirements | 2 | CRITICAL: 1, HIGH: 1 |
| Operational Requirements | 3 | MEDIUM: 2, HIGH: 1 |
| **TOTAL** | **72** | **CRITICAL: 14, HIGH: 35, MEDIUM: 21, LOW: 2** |

---

## 9. Assumptions and Dependencies

### 9.1 Assumptions

1. PostgreSQL 14+ database server available
2. Server deployed in same network as database (low latency)
3. SSL certificate for HTTPS (Let's Encrypt or CA-issued)
4. Load balancer handles TLS termination (optional, server can do TLS)

### 9.2 Dependencies

1. TVM_API_SPECIFICATION.md finalized (Week 1 ✅)
2. TVM_DATA_MODELS.md finalized (Week 1 ✅)
3. Vehicle TVM client implements API (VEHICLE_TVM_CLIENT_REQUIREMENTS.md)
4. Fleet Management frontend consumes API (FLEET_MANAGEMENT_REQUIREMENTS.md)

---

## 10. Open Issues and Risks

### 10.1 Open Issues

| ID | Issue | Owner | Target Resolution |
|----|-------|-------|-------------------|
| TVM-SRV-ISSUE-001 | Technology stack selection (Python/Node.js/Java) | Unno | Week 3 |
| TVM-SRV-ISSUE-002 | Message queue for alerts (Redis Pub/Sub vs RabbitMQ) | Unno | Week 3 |

### 10.2 Risks

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| WebSocket scalability under load | MEDIUM | HIGH | Load test early (Week 7), use Redis for message broadcasting if needed |
| Database performance with high telemetry volume | MEDIUM | MEDIUM | Optimize indexes, consider TimescaleDB for time-series data |
| JWT secret compromise | LOW | HIGH | Rotate JWT secret regularly, store in secure vault (HashiCorp Vault) |

---

## 11. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-16 | Unno | Initial draft - Week 2 documentation |

---

**END OF DOCUMENT**

**Total Requirements:** 72
**Document Status:** Draft for Review
**Next Review:** Week 3 (with Pankaj for vehicle integration)
