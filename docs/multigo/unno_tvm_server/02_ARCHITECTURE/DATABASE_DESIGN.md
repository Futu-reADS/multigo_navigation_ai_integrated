# Database Design

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Document Type:** Architecture Specification
**Status:** Active
**Version:** 1.0
**Last Updated:** 2025-12-17
**Owner:** Team 2 - TVM Backend

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Database Technology](#2-database-technology)
3. [Schema Overview](#3-schema-overview)
4. [Core Tables](#4-core-tables)
5. [Relationships and Constraints](#5-relationships-and-constraints)
6. [Indexing Strategy](#6-indexing-strategy)
7. [Data Types and Validation](#7-data-types-and-validation)
8. [Query Patterns](#8-query-patterns)
9. [Partitioning and Sharding](#9-partitioning-and-sharding)
10. [Data Lifecycle Management](#10-data-lifecycle-management)
11. [Performance Optimization](#11-performance-optimization)
12. [Backup and Recovery](#12-backup-and-recovery)
13. [Security and Access Control](#13-security-and-access-control)
14. [Migration Strategy](#14-migration-strategy)

---

## 1. Executive Summary

### 1.1 Purpose

This document defines the database design for the TVM (Transport Vehicle Management) system. The database stores:
- **Residents:** People who use the wheelchair transport service
- **Vehicles:** Fleet of autonomous robots
- **Missions:** Transport requests (pickup/dropoff)
- **Telemetry:** Vehicle sensor data, battery, location
- **Users:** Operators, nurses, caregivers
- **Audit Logs:** Security and compliance logs

### 1.2 Database Requirements

**Functional Requirements:**
- Store resident profiles (PII, medical info, preferences)
- Track vehicles (status, location, battery, maintenance)
- Manage missions (queue, execution, history)
- Log telemetry (real-time updates from vehicles)
- User authentication and authorization (RBAC)
- Audit trail (all data access, changes)

**Non-Functional Requirements:**
- **Availability:** 99.5% uptime (multi-AZ deployment)
- **Performance:**
  - Read latency: <50ms (95th percentile)
  - Write latency: <100ms (95th percentile)
  - Throughput: 1000 queries/second
- **Scalability:** Support 50 vehicles, 500 residents, 10,000 missions/day
- **Data Integrity:** ACID transactions, foreign key constraints
- **Security:** Encryption at rest (AES-256), encryption in transit (TLS 1.3)
- **Compliance:** GDPR, PIPEDA, HIPAA (data retention, privacy)

### 1.3 Design Principles

1. **Normalization:** 3rd Normal Form (3NF) to avoid redundancy
2. **Denormalization (Selective):** For read-heavy queries (e.g., mission list with resident name)
3. **Soft Deletes:** Use `deleted_at` timestamp instead of hard delete (audit trail)
4. **Timestamps:** Every table has `created_at`, `updated_at` columns
5. **UUIDs:** Use UUIDs for primary keys (distributed, non-sequential)
6. **Constraints:** Enforce data integrity at database level (foreign keys, check constraints)

---

## 2. Database Technology

### 2.1 RDBMS Choice: PostgreSQL

**Why PostgreSQL?**
- **ACID Compliance:** Strong consistency, transactions
- **JSON Support:** Native JSONB type (flexible schema for telemetry)
- **Advanced Features:** CTEs, window functions, full-text search
- **Geospatial:** PostGIS extension (location queries)
- **Mature:** 25+ years, battle-tested
- **Open Source:** No licensing costs

**Alternatives Considered:**
| Database | Pros | Cons | Decision |
|----------|------|------|----------|
| **MySQL** | Popular, good performance | Less advanced SQL features | ❌ PostgreSQL more feature-rich |
| **MongoDB** | Flexible schema, horizontal scaling | No ACID (single doc only), joins expensive | ❌ Need strong consistency |
| **CockroachDB** | Distributed, PostgreSQL-compatible | Complex ops, expensive | ❌ Single-region fine for MVP |

**Decision:** PostgreSQL 15+ (latest stable)

### 2.2 Deployment

**AWS RDS PostgreSQL:**
- **Instance:** db.t3.large (2 vCPU, 8 GB RAM) for MVP, scale up as needed
- **Storage:** 100 GB gp3 SSD (auto-scale up to 1 TB)
- **Multi-AZ:** Yes (high availability, automatic failover)
- **Encryption:** At rest (AES-256), in transit (TLS 1.3)
- **Backups:** Daily automated backups (7-day retention), manual snapshots

---

## 3. Schema Overview

### 3.1 Entity-Relationship Diagram

```
┌────────────┐         ┌─────────────┐
│  Residents │         │  Vehicles   │
│  (PII)     │         │  (Fleet)    │
└─────┬──────┘         └──────┬──────┘
      │                       │
      │ 1:N                   │ 1:N
      ↓                       ↓
┌──────────────────────────────────┐
│           Missions               │
│  (Transport Requests)            │
└───────────┬──────────────────────┘
            │ 1:N
            ↓
┌──────────────────────────────────┐
│        Mission Events            │
│  (State Changes)                 │
└──────────────────────────────────┘

┌────────────┐         ┌─────────────┐
│   Users    │         │   Roles     │
│  (Operators│◄────────┤  (RBAC)     │
│   Nurses)  │   N:M   │             │
└────────────┘         └─────────────┘

┌──────────────────────────────────┐
│         Telemetry                │
│  (Vehicle Sensor Data)           │
│  - Time-series data              │
└──────────────────────────────────┘

┌──────────────────────────────────┐
│         Audit Logs               │
│  (Security, Compliance)          │
└──────────────────────────────────┘
```

### 3.2 Table Categories

| Category | Tables | Purpose |
|----------|--------|---------|
| **Core Entities** | residents, vehicles, missions | Business objects |
| **Operational** | mission_events, vehicle_status | Real-time state |
| **Telemetry** | telemetry, vehicle_logs | Sensor data, diagnostics |
| **User Management** | users, roles, permissions, user_roles | Authentication, RBAC |
| **Configuration** | facilities, locations, maps | System config |
| **Audit & Compliance** | audit_logs, data_access_logs | Security, GDPR |

---

## 4. Core Tables

### 4.1 `residents` Table

**Purpose:** Store resident (passenger) profiles

```sql
CREATE TABLE residents (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    facility_id UUID NOT NULL REFERENCES facilities(id),

    -- Personal Information (PII)
    first_name VARCHAR(100) NOT NULL,
    last_name VARCHAR(100) NOT NULL,
    date_of_birth DATE NOT NULL,
    gender VARCHAR(20),  -- 'male', 'female', 'non-binary', 'prefer-not-to-say'

    -- Contact Information
    phone VARCHAR(20),
    emergency_contact_name VARCHAR(200),
    emergency_contact_phone VARCHAR(20),

    -- Medical Information (encrypted, HIPAA if US)
    medical_notes TEXT,  -- Encrypted at app layer (e.g., allergies, mobility restrictions)
    wheelchair_type VARCHAR(50),  -- 'manual', 'powered', 'transport_chair'
    requires_assistance BOOLEAN DEFAULT FALSE,

    -- Preferences
    preferred_language VARCHAR(10) DEFAULT 'en',  -- 'en', 'ja', 'fr', etc.

    -- Photo (optional, for staff identification)
    photo_url VARCHAR(500),

    -- Privacy & Consent
    gdpr_consent BOOLEAN DEFAULT FALSE,
    gdpr_consent_date TIMESTAMP WITH TIME ZONE,

    -- Metadata
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    deleted_at TIMESTAMP WITH TIME ZONE,  -- Soft delete

    -- Constraints
    CONSTRAINT check_gender CHECK (gender IN ('male', 'female', 'non-binary', 'prefer-not-to-say', NULL))
);

-- Indexes
CREATE INDEX idx_residents_facility ON residents(facility_id) WHERE deleted_at IS NULL;
CREATE INDEX idx_residents_last_name ON residents(last_name) WHERE deleted_at IS NULL;
CREATE INDEX idx_residents_deleted ON residents(deleted_at);  -- For soft delete queries

-- Trigger: Update updated_at on row change
CREATE TRIGGER update_residents_updated_at BEFORE UPDATE ON residents
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
```

**Notes:**
- `medical_notes` encrypted at application layer (not database encryption, for field-level granularity)
- `gdpr_consent` required for GDPR compliance (explicit consent for processing PII)
- Soft delete: `deleted_at IS NULL` for active residents

### 4.2 `vehicles` Table

**Purpose:** Store vehicle fleet information

```sql
CREATE TABLE vehicles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    facility_id UUID NOT NULL REFERENCES facilities(id),

    -- Vehicle Identification
    name VARCHAR(100) NOT NULL UNIQUE,  -- Human-readable (e.g., "Vehicle Alpha")
    serial_number VARCHAR(100) UNIQUE,  -- Hardware serial number

    -- Vehicle Type
    model VARCHAR(100),  -- e.g., "Swerve-Drive-V1"
    firmware_version VARCHAR(50),  -- e.g., "v1.2.3"

    -- Status
    status VARCHAR(50) NOT NULL DEFAULT 'idle',
    -- 'idle', 'navigating', 'docking', 'charging', 'maintenance', 'error'

    -- Current State
    current_mission_id UUID REFERENCES missions(id),
    current_location JSONB,  -- {"lat": 35.6, "lon": 139.7, "zone": "Building A"}
    battery_level INTEGER CHECK (battery_level >= 0 AND battery_level <= 100),  -- Percentage
    last_heartbeat TIMESTAMP WITH TIME ZONE,  -- Last communication with TVM

    -- Capabilities
    max_payload_kg INTEGER DEFAULT 150,  -- Max weight capacity
    max_speed_mps NUMERIC(3,1) DEFAULT 2.0,  -- Max speed in m/s

    -- Maintenance
    last_maintenance_date DATE,
    next_maintenance_due DATE,
    total_distance_km NUMERIC(10,2) DEFAULT 0.0,  -- Odometer
    total_missions INTEGER DEFAULT 0,

    -- Metadata
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    deleted_at TIMESTAMP WITH TIME ZONE,

    -- Constraints
    CONSTRAINT check_status CHECK (status IN ('idle', 'navigating', 'docking', 'charging', 'maintenance', 'error'))
);

-- Indexes
CREATE INDEX idx_vehicles_facility ON vehicles(facility_id) WHERE deleted_at IS NULL;
CREATE INDEX idx_vehicles_status ON vehicles(status) WHERE deleted_at IS NULL;
CREATE INDEX idx_vehicles_last_heartbeat ON vehicles(last_heartbeat);  -- For offline detection
CREATE INDEX idx_vehicles_name ON vehicles(name);

-- Trigger: Update updated_at
CREATE TRIGGER update_vehicles_updated_at BEFORE UPDATE ON vehicles
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
```

**Notes:**
- `status` enum-like (check constraint, not true enum for flexibility)
- `current_location` JSONB (flexible, can add fields without migration)
- `last_heartbeat` used to detect offline vehicles (alert if >5 minutes old)

### 4.3 `missions` Table

**Purpose:** Store transport requests (pickup/dropoff)

```sql
CREATE TABLE missions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    facility_id UUID NOT NULL REFERENCES facilities(id),

    -- Mission Participants
    resident_id UUID NOT NULL REFERENCES residents(id),
    vehicle_id UUID REFERENCES vehicles(id),  -- Assigned vehicle (NULL if unassigned)
    created_by_user_id UUID NOT NULL REFERENCES users(id),  -- Who created the mission

    -- Mission Details
    pickup_location VARCHAR(200) NOT NULL,  -- Human-readable (e.g., "Room 101")
    pickup_coordinates POINT,  -- PostGIS point (lat, lon)
    dropoff_location VARCHAR(200) NOT NULL,
    dropoff_coordinates POINT,

    -- Priority
    priority VARCHAR(20) NOT NULL DEFAULT 'normal',
    -- 'low', 'normal', 'high', 'urgent'

    -- Status
    status VARCHAR(50) NOT NULL DEFAULT 'pending',
    -- 'pending', 'assigned', 'in_progress', 'completed', 'cancelled', 'failed'

    -- Timestamps
    scheduled_at TIMESTAMP WITH TIME ZONE,  -- Requested pickup time (NULL for ASAP)
    assigned_at TIMESTAMP WITH TIME ZONE,  -- When vehicle assigned
    started_at TIMESTAMP WITH TIME ZONE,  -- When vehicle started mission
    completed_at TIMESTAMP WITH TIME ZONE,  -- When mission completed

    -- Notes
    notes TEXT,  -- Additional instructions (e.g., "Bring oxygen tank")
    failure_reason TEXT,  -- If status='failed', reason

    -- Metadata
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    deleted_at TIMESTAMP WITH TIME ZONE,

    -- Constraints
    CONSTRAINT check_priority CHECK (priority IN ('low', 'normal', 'high', 'urgent')),
    CONSTRAINT check_status CHECK (status IN ('pending', 'assigned', 'in_progress', 'completed', 'cancelled', 'failed'))
);

-- Indexes
CREATE INDEX idx_missions_facility ON missions(facility_id) WHERE deleted_at IS NULL;
CREATE INDEX idx_missions_resident ON missions(resident_id);
CREATE INDEX idx_missions_vehicle ON missions(vehicle_id);
CREATE INDEX idx_missions_status ON missions(status) WHERE status IN ('pending', 'assigned', 'in_progress');
CREATE INDEX idx_missions_created_at ON missions(created_at DESC);  -- For recent missions query
CREATE INDEX idx_missions_scheduled_at ON missions(scheduled_at) WHERE status = 'pending';

-- GiST index for geospatial queries (PostGIS)
CREATE INDEX idx_missions_pickup_location ON missions USING GIST (pickup_coordinates);
CREATE INDEX idx_missions_dropoff_location ON missions USING GIST (dropoff_coordinates);

-- Trigger: Update updated_at
CREATE TRIGGER update_missions_updated_at BEFORE UPDATE ON missions
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
```

**Notes:**
- `status` lifecycle: pending → assigned → in_progress → completed (or cancelled/failed)
- `pickup_coordinates` and `dropoff_coordinates` as PostGIS `POINT` for geospatial queries
- Index on `created_at DESC` for "recent missions" query

### 4.4 `mission_events` Table

**Purpose:** Log all mission state changes (audit trail, timeline)

```sql
CREATE TABLE mission_events (
    id BIGSERIAL PRIMARY KEY,  -- Use BIGINT for high-volume inserts
    mission_id UUID NOT NULL REFERENCES missions(id) ON DELETE CASCADE,
    vehicle_id UUID REFERENCES vehicles(id),

    -- Event Type
    event_type VARCHAR(50) NOT NULL,
    -- 'created', 'assigned', 'started', 'pickup_arrived', 'pickup_completed',
    -- 'in_transit', 'dropoff_arrived', 'dropoff_completed', 'completed', 'cancelled', 'failed'

    -- Event Details
    details JSONB,  -- Flexible field for event-specific data
    -- Example: {"reason": "resident not ready", "retry_count": 2}

    -- Metadata
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    created_by_user_id UUID REFERENCES users(id)  -- Who triggered event (if manual)
);

-- Indexes
CREATE INDEX idx_mission_events_mission ON mission_events(mission_id);
CREATE INDEX idx_mission_events_created_at ON mission_events(created_at DESC);
CREATE INDEX idx_mission_events_type ON mission_events(event_type);
```

**Notes:**
- `BIGSERIAL` for high-volume inserts (missions generate 5-10 events each)
- `details` JSONB for flexible event data
- No soft delete (immutable audit trail)

### 4.5 `telemetry` Table

**Purpose:** Store time-series vehicle sensor data

```sql
CREATE TABLE telemetry (
    id BIGSERIAL PRIMARY KEY,
    vehicle_id UUID NOT NULL REFERENCES vehicles(id),

    -- Timestamp
    timestamp TIMESTAMP WITH TIME ZONE NOT NULL,

    -- Location
    latitude NUMERIC(10,7),  -- -90 to 90
    longitude NUMERIC(10,7),  -- -180 to 180
    heading NUMERIC(5,2),  -- 0-360 degrees

    -- Motion
    speed_mps NUMERIC(4,2),  -- meters per second
    acceleration_mps2 NUMERIC(4,2),

    -- Battery
    battery_level INTEGER,  -- Percentage 0-100
    battery_voltage NUMERIC(5,2),  -- Volts
    battery_current NUMERIC(6,2),  -- Amperes

    -- Sensors
    lidar_status VARCHAR(20),  -- 'ok', 'degraded', 'error'
    camera_status VARCHAR(20),
    imu_status VARCHAR(20),

    -- System
    cpu_usage INTEGER,  -- Percentage 0-100
    memory_usage INTEGER,  -- MB
    disk_usage INTEGER,  -- Percentage 0-100

    -- Errors/Warnings
    error_codes VARCHAR(100)[],  -- Array of error codes (e.g., ['VEH-NAV-001', 'VEH-LID-001'])

    -- Metadata
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Partitioning: By month (TimescaleDB or native partitioning)
-- See Section 9.1 for partitioning strategy

-- Indexes
CREATE INDEX idx_telemetry_vehicle_timestamp ON telemetry(vehicle_id, timestamp DESC);
CREATE INDEX idx_telemetry_timestamp ON telemetry(timestamp DESC);

-- Retention policy: Delete telemetry older than 90 days (see Section 10)
```

**Notes:**
- High-volume table: 10 Hz × 10 vehicles × 8 hours/day = 2.88M rows/day
- **Partitioning required** (by month or week, see Section 9)
- Consider TimescaleDB extension for optimized time-series queries
- **Retention:** 90 days (PRIVACY_REQUIREMENTS.md REQ-PRIV-012)

### 4.6 `users` Table

**Purpose:** Store user accounts (operators, nurses, caregivers, admins)

```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    facility_id UUID NOT NULL REFERENCES facilities(id),

    -- Credentials
    email VARCHAR(255) NOT NULL UNIQUE,
    password_hash VARCHAR(255) NOT NULL,  -- bcrypt hash

    -- Personal Information
    first_name VARCHAR(100) NOT NULL,
    last_name VARCHAR(100) NOT NULL,
    phone VARCHAR(20),

    -- Account Status
    status VARCHAR(20) NOT NULL DEFAULT 'active',
    -- 'active', 'inactive', 'locked', 'pending_verification'
    email_verified BOOLEAN DEFAULT FALSE,
    email_verification_token VARCHAR(255),

    -- MFA (Multi-Factor Authentication)
    mfa_enabled BOOLEAN DEFAULT FALSE,
    mfa_secret VARCHAR(255),  -- TOTP secret (encrypted)

    -- Password Reset
    password_reset_token VARCHAR(255),
    password_reset_expires TIMESTAMP WITH TIME ZONE,

    -- Last Login
    last_login_at TIMESTAMP WITH TIME ZONE,
    last_login_ip INET,

    -- Metadata
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    deleted_at TIMESTAMP WITH TIME ZONE,

    -- Constraints
    CONSTRAINT check_status CHECK (status IN ('active', 'inactive', 'locked', 'pending_verification'))
);

-- Indexes
CREATE UNIQUE INDEX idx_users_email ON users(LOWER(email)) WHERE deleted_at IS NULL;
CREATE INDEX idx_users_facility ON users(facility_id) WHERE deleted_at IS NULL;

-- Trigger: Update updated_at
CREATE TRIGGER update_users_updated_at BEFORE UPDATE ON users
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
```

**Notes:**
- `password_hash` using bcrypt (cost factor 12, see SECURITY_REQUIREMENTS.md REQ-SEC-006)
- `email` case-insensitive unique index (LOWER(email))
- `mfa_secret` encrypted at application layer (if MFA enabled)

### 4.7 `roles` Table (RBAC)

**Purpose:** Define user roles (Admin, Operator, Nurse, Caregiver)

```sql
CREATE TABLE roles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    name VARCHAR(50) NOT NULL UNIQUE,  -- 'admin', 'operator', 'nurse', 'caregiver', 'guest'
    description TEXT,

    -- Metadata
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Seed data (insert default roles)
INSERT INTO roles (name, description) VALUES
    ('admin', 'Full system access, user management, config changes'),
    ('operator', 'Monitor fleet, dispatch missions, view telemetry'),
    ('nurse', 'Request transport, view mission status, cancel own missions'),
    ('caregiver', 'Request transport for assigned residents only'),
    ('maintenance', 'View vehicle diagnostics, initiate maintenance mode'),
    ('guest', 'View public status dashboard only');
```

### 4.8 `permissions` Table

**Purpose:** Define granular permissions (e.g., "mission:create", "vehicle:view")

```sql
CREATE TABLE permissions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    resource VARCHAR(50) NOT NULL,  -- 'mission', 'vehicle', 'resident', 'user'
    action VARCHAR(50) NOT NULL,  -- 'create', 'read', 'update', 'delete', 'list'
    description TEXT,

    -- Metadata
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),

    CONSTRAINT unique_permission UNIQUE (resource, action)
);

-- Seed data
INSERT INTO permissions (resource, action, description) VALUES
    ('mission', 'create', 'Create new missions'),
    ('mission', 'read', 'View mission details'),
    ('mission', 'update', 'Update mission (e.g., cancel)'),
    ('mission', 'delete', 'Delete mission'),
    ('mission', 'list', 'List all missions'),
    ('vehicle', 'read', 'View vehicle details'),
    ('vehicle', 'control', 'Control vehicle (e-stop, reboot)'),
    ('resident', 'create', 'Add new residents'),
    ('resident', 'read', 'View resident profiles (PII access)'),
    ('user', 'create', 'Create new users'),
    ('user', 'delete', 'Delete users');
```

### 4.9 `role_permissions` Table (Many-to-Many)

**Purpose:** Assign permissions to roles

```sql
CREATE TABLE role_permissions (
    role_id UUID NOT NULL REFERENCES roles(id) ON DELETE CASCADE,
    permission_id UUID NOT NULL REFERENCES permissions(id) ON DELETE CASCADE,

    PRIMARY KEY (role_id, permission_id)
);

-- Seed data: Example for 'operator' role
INSERT INTO role_permissions (role_id, permission_id)
SELECT
    (SELECT id FROM roles WHERE name = 'operator'),
    id
FROM permissions
WHERE (resource = 'mission' AND action IN ('create', 'read', 'list'))
   OR (resource = 'vehicle' AND action = 'read');
```

### 4.10 `user_roles` Table (Many-to-Many)

**Purpose:** Assign roles to users

```sql
CREATE TABLE user_roles (
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    role_id UUID NOT NULL REFERENCES roles(id) ON DELETE CASCADE,

    -- Metadata
    assigned_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    assigned_by_user_id UUID REFERENCES users(id),  -- Who assigned this role

    PRIMARY KEY (user_id, role_id)
);

-- Indexes
CREATE INDEX idx_user_roles_user ON user_roles(user_id);
CREATE INDEX idx_user_roles_role ON user_roles(role_id);
```

### 4.11 `audit_logs` Table

**Purpose:** Audit trail for security and compliance

```sql
CREATE TABLE audit_logs (
    id BIGSERIAL PRIMARY KEY,

    -- Who
    user_id UUID REFERENCES users(id),  -- NULL for system events
    user_email VARCHAR(255),  -- Denormalized (user might be deleted)

    -- What
    action VARCHAR(100) NOT NULL,  -- 'login', 'mission_create', 'resident_view', 'config_change'
    resource_type VARCHAR(50),  -- 'mission', 'resident', 'vehicle', 'user'
    resource_id UUID,  -- ID of affected resource

    -- Details
    details JSONB,  -- Flexible field for action-specific data
    -- Example: {"old_status": "pending", "new_status": "in_progress"}

    -- When & Where
    timestamp TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    ip_address INET,
    user_agent TEXT,

    -- Result
    success BOOLEAN NOT NULL,  -- TRUE = success, FALSE = failure (e.g., unauthorized)
    failure_reason TEXT  -- If success=FALSE, reason
);

-- Indexes
CREATE INDEX idx_audit_logs_user ON audit_logs(user_id);
CREATE INDEX idx_audit_logs_timestamp ON audit_logs(timestamp DESC);
CREATE INDEX idx_audit_logs_action ON audit_logs(action);
CREATE INDEX idx_audit_logs_resource ON audit_logs(resource_type, resource_id);

-- Retention: 7 years (PRIVACY_REQUIREMENTS.md REQ-PRIV-048)
```

**Notes:**
- `user_email` denormalized (in case user deleted, still know who performed action)
- `details` JSONB for flexible audit data
- High volume: Retention policy required (partition or archive old data)

---

## 5. Relationships and Constraints

### 5.1 Foreign Key Relationships

```sql
-- Mission → Resident (who is being transported)
missions.resident_id → residents.id

-- Mission → Vehicle (which vehicle assigned)
missions.vehicle_id → vehicles.id

-- Mission → User (who created mission)
missions.created_by_user_id → users.id

-- Mission Events → Mission
mission_events.mission_id → missions.id (CASCADE on delete)

-- Telemetry → Vehicle
telemetry.vehicle_id → vehicles.id

-- Users → Facility
users.facility_id → facilities.id

-- User Roles → Users, Roles
user_roles.user_id → users.id (CASCADE on delete)
user_roles.role_id → roles.id (CASCADE on delete)
```

### 5.2 Check Constraints

```sql
-- Battery level: 0-100%
ALTER TABLE vehicles ADD CONSTRAINT check_battery_level
    CHECK (battery_level >= 0 AND battery_level <= 100);

-- Mission priority: enum-like
ALTER TABLE missions ADD CONSTRAINT check_priority
    CHECK (priority IN ('low', 'normal', 'high', 'urgent'));

-- Vehicle status: enum-like
ALTER TABLE vehicles ADD CONSTRAINT check_status
    CHECK (status IN ('idle', 'navigating', 'docking', 'charging', 'maintenance', 'error'));
```

### 5.3 Unique Constraints

```sql
-- Vehicle name unique per facility
CREATE UNIQUE INDEX idx_vehicles_name_unique ON vehicles(facility_id, name) WHERE deleted_at IS NULL;

-- User email unique (case-insensitive)
CREATE UNIQUE INDEX idx_users_email_unique ON users(LOWER(email)) WHERE deleted_at IS NULL;
```

---

## 6. Indexing Strategy

### 6.1 Index Types

| Index Type | Use Case | Example |
|------------|----------|---------|
| **B-Tree (default)** | Equality, range queries | `CREATE INDEX idx_missions_created_at ON missions(created_at)` |
| **GiST (geospatial)** | PostGIS point/geometry queries | `CREATE INDEX idx_missions_pickup USING GIST (pickup_coordinates)` |
| **GIN (JSONB)** | JSONB key/value queries | `CREATE INDEX idx_telemetry_details ON telemetry USING GIN (details)` |
| **Partial Index** | Index subset of rows | `CREATE INDEX idx_active_missions ON missions(status) WHERE status='in_progress'` |

### 6.2 Query Performance Targets

**Goal:**
- **Point queries (by ID):** <5ms
- **List queries (paginated):** <50ms
- **Aggregation queries (dashboard):** <200ms

**Tools:**
- Use `EXPLAIN ANALYZE` to profile slow queries
- Monitor slow query log (queries >1 second)
- Grafana dashboard for query latency (p50, p95, p99)

---

## 7. Data Types and Validation

### 7.1 Data Type Choices

| Domain | PostgreSQL Type | Rationale |
|--------|----------------|-----------|
| **Primary Key** | UUID | Distributed-friendly, non-sequential |
| **Timestamps** | TIMESTAMP WITH TIME ZONE | Time zone aware, avoids DST issues |
| **Money** | NUMERIC(10,2) | Exact decimal, no floating-point errors |
| **Geolocation** | POINT (PostGIS) | Geospatial queries (distance, bounding box) |
| **Enum-like** | VARCHAR + CHECK | Flexibility (can add values without migration) |
| **Flexible Data** | JSONB | Semi-structured data, indexed |

### 7.2 Validation Rules

**Application-Layer Validation:**
- Email format: Regex `/^[^\s@]+@[^\s@]+\.[^\s@]+$/`
- Phone format: E.164 (e.g., +1-555-123-4567)
- Coordinates: Latitude -90 to 90, Longitude -180 to 180

**Database-Layer Validation:**
- Check constraints (enums, ranges)
- Foreign key constraints (referential integrity)
- NOT NULL constraints (required fields)

---

## 8. Query Patterns

### 8.1 Common Queries

#### Query 1: List Active Missions (Dashboard)

```sql
SELECT
    m.id,
    m.status,
    m.priority,
    m.pickup_location,
    m.dropoff_location,
    r.first_name || ' ' || r.last_name AS resident_name,
    v.name AS vehicle_name,
    m.created_at
FROM missions m
JOIN residents r ON m.resident_id = r.id
LEFT JOIN vehicles v ON m.vehicle_id = v.id
WHERE m.status IN ('pending', 'assigned', 'in_progress')
  AND m.deleted_at IS NULL
ORDER BY
    CASE m.priority
        WHEN 'urgent' THEN 1
        WHEN 'high' THEN 2
        WHEN 'normal' THEN 3
        WHEN 'low' THEN 4
    END,
    m.created_at ASC
LIMIT 50;
```

**Performance:** <50ms (indexed on `status`, `created_at`)

#### Query 2: Vehicle Fleet Status

```sql
SELECT
    v.id,
    v.name,
    v.status,
    v.battery_level,
    v.current_location->>'zone' AS current_zone,
    v.last_heartbeat,
    CASE
        WHEN v.last_heartbeat > NOW() - INTERVAL '5 minutes' THEN 'online'
        ELSE 'offline'
    END AS connectivity_status
FROM vehicles v
WHERE v.deleted_at IS NULL
ORDER BY v.name;
```

**Performance:** <10ms (small table, <100 vehicles)

#### Query 3: Recent Telemetry for Vehicle

```sql
SELECT
    timestamp,
    latitude,
    longitude,
    speed_mps,
    battery_level,
    error_codes
FROM telemetry
WHERE vehicle_id = $1  -- Parameter: vehicle UUID
  AND timestamp > NOW() - INTERVAL '1 hour'
ORDER BY timestamp DESC
LIMIT 1000;
```

**Performance:** <100ms (indexed on `vehicle_id, timestamp`)

#### Query 4: Mission Timeline (Audit Trail)

```sql
SELECT
    me.event_type,
    me.details,
    me.created_at,
    u.email AS triggered_by
FROM mission_events me
LEFT JOIN users u ON me.created_by_user_id = u.id
WHERE me.mission_id = $1  -- Parameter: mission UUID
ORDER BY me.created_at ASC;
```

**Performance:** <20ms (indexed on `mission_id`)

### 8.2 Optimization Techniques

**1. Denormalization (Selective):**
- Store `resident_name` in `missions` table (avoid JOIN on every query)
- Trade-off: Update complexity vs. read performance

**2. Materialized Views:**
```sql
CREATE MATERIALIZED VIEW mission_summary AS
SELECT
    m.id,
    m.status,
    m.priority,
    r.first_name || ' ' || r.last_name AS resident_name,
    v.name AS vehicle_name,
    m.created_at
FROM missions m
JOIN residents r ON m.resident_id = r.id
LEFT JOIN vehicles v ON m.vehicle_id = v.id
WHERE m.deleted_at IS NULL;

-- Refresh periodically (e.g., every 5 minutes)
REFRESH MATERIALIZED VIEW CONCURRENTLY mission_summary;
```

**3. Connection Pooling:**
- Use PgBouncer to pool connections (reduce connection overhead)
- Pool size: 50 connections (for 1000 client connections)

---

## 9. Partitioning and Sharding

### 9.1 Table Partitioning (Telemetry)

**Problem:** `telemetry` table grows to 100M+ rows (2.88M rows/day × 90 days retention = 259M rows)

**Solution:** Partition by month (native PostgreSQL partitioning)

```sql
-- Parent table (partitioned)
CREATE TABLE telemetry (
    id BIGSERIAL,
    vehicle_id UUID NOT NULL,
    timestamp TIMESTAMP WITH TIME ZONE NOT NULL,
    ...
    PRIMARY KEY (id, timestamp)
) PARTITION BY RANGE (timestamp);

-- Child partitions (one per month)
CREATE TABLE telemetry_2025_12 PARTITION OF telemetry
    FOR VALUES FROM ('2025-12-01') TO ('2026-01-01');

CREATE TABLE telemetry_2026_01 PARTITION OF telemetry
    FOR VALUES FROM ('2026-01-01') TO ('2026-02-01');

-- Automatically create partitions (using pg_partman extension)
SELECT partman.create_parent('public.telemetry', 'timestamp', 'native', 'monthly');
```

**Benefits:**
- Query performance: Only scan relevant partition (e.g., last month)
- Retention: Drop old partitions (fast, no DELETE scan)

### 9.2 Sharding (Future, if needed)

**Sharding Strategy:** By `facility_id` (multi-tenant)

**When to shard:**
- Single database cannot handle load (>10,000 writes/second)
- Data size exceeds 5 TB

**Implementation:**
- **Citus (PostgreSQL extension):** Distributed PostgreSQL
- **Application-level sharding:** Route queries based on `facility_id`

**Complexity:** High operational overhead, avoid unless necessary

---

## 10. Data Lifecycle Management

### 10.1 Retention Policies

| Table | Retention | Action |
|-------|-----------|--------|
| `missions` | 3 years | Soft delete, archive to S3 Glacier after 1 year |
| `mission_events` | 3 years | Archive with missions |
| `telemetry` | 90 days | Delete (DROP partition) |
| `vehicle_logs` | 90 days | Delete |
| `audit_logs` | 7 years | Partition by year, archive after 1 year |
| `residents` | Active + 3 years | Soft delete, anonymize after 3 years |

### 10.2 Automated Cleanup Job

```sql
-- Cron job (using pg_cron extension) or external scheduler
-- Run daily at 2 AM UTC

-- Delete telemetry older than 90 days
DELETE FROM telemetry WHERE timestamp < NOW() - INTERVAL '90 days';

-- Or drop partition (faster)
DROP TABLE telemetry_2025_09;  -- If current month is December, drop September

-- Anonymize old residents (GDPR right to erasure)
UPDATE residents
SET
    first_name = 'Deleted',
    last_name = 'User #' || id,
    email = NULL,
    phone = NULL,
    medical_notes = NULL
WHERE deleted_at < NOW() - INTERVAL '3 years';
```

---

## 11. Performance Optimization

### 11.1 Query Optimization

**Slow Query Analysis:**
```sql
-- Enable slow query log (log queries >1 second)
ALTER SYSTEM SET log_min_duration_statement = 1000;  -- milliseconds

-- View slow queries
SELECT query, calls, mean_exec_time, max_exec_time
FROM pg_stat_statements
ORDER BY mean_exec_time DESC
LIMIT 20;
```

**Common Fixes:**
1. **Missing Index:** Add index on filter/join columns
2. **Sequential Scan:** Check `EXPLAIN ANALYZE`, ensure index used
3. **Large OFFSET:** Use cursor-based pagination instead of OFFSET

### 11.2 Vacuum and Analyze

**Auto-Vacuum:** Enabled by default, but tune for high-traffic tables

```sql
-- Tune auto-vacuum for telemetry table (high insert rate)
ALTER TABLE telemetry SET (
    autovacuum_vacuum_scale_factor = 0.05,  -- Vacuum when 5% of rows are dead tuples
    autovacuum_analyze_scale_factor = 0.02  -- Analyze when 2% of rows changed
);
```

**Manual Vacuum:**
```sql
-- Full vacuum (reclaims disk space, locks table)
VACUUM FULL telemetry;  -- Run during maintenance window

-- Analyze (updates statistics for query planner)
ANALYZE missions;
```

### 11.3 Connection Pooling

**PgBouncer Configuration:**
```ini
[databases]
tvm = host=tvm-db.internal port=5432 dbname=tvm

[pgbouncer]
pool_mode = transaction  # Transaction-level pooling
max_client_conn = 1000   # Max client connections
default_pool_size = 50   # Max connections to PostgreSQL
```

**Result:** 1000 client connections → 50 PostgreSQL connections (reduces overhead)

---

## 12. Backup and Recovery

### 12.1 Backup Strategy

**AWS RDS Automated Backups:**
- **Frequency:** Daily (1 AM UTC)
- **Retention:** 7 days (RDS default)
- **Type:** Full snapshot + transaction logs (point-in-time recovery)

**Manual Snapshots:**
- **Before Major Changes:** Take manual snapshot before schema migrations
- **Retention:** Indefinite (until manually deleted)

**Offsite Backups:**
- **S3 Glacier:** Weekly full backup replicated to different region (eu-west-1)
- **Retention:** 7 years (compliance)

### 12.2 Recovery Procedures

**Point-in-Time Recovery (PITR):**
```bash
# Restore to specific time (e.g., before accidental DELETE)
aws rds restore-db-instance-to-point-in-time \
  --source-db-instance-identifier tvm-production-primary \
  --target-db-instance-identifier tvm-restored-2025-12-17-14-25 \
  --restore-time 2025-12-17T14:25:00Z
```

**Disaster Recovery:** See DISASTER_RECOVERY_PLAN.md

---

## 13. Security and Access Control

### 13.1 Database Users

| User | Privileges | Purpose |
|------|------------|---------|
| `tvm_admin` | SUPERUSER | Database administration (migrations, maintenance) |
| `tvm_app` | CONNECT, SELECT, INSERT, UPDATE, DELETE on all tables | Application user (TVM server) |
| `tvm_readonly` | CONNECT, SELECT on all tables | Read-only user (analytics, reporting) |

**Create Application User:**
```sql
CREATE USER tvm_app WITH PASSWORD 'secure-password-xyz123';
GRANT CONNECT ON DATABASE tvm TO tvm_app;
GRANT SELECT, INSERT, UPDATE, DELETE ON ALL TABLES IN SCHEMA public TO tvm_app;
GRANT USAGE, SELECT ON ALL SEQUENCES IN SCHEMA public TO tvm_app;

-- Future tables (grant automatically)
ALTER DEFAULT PRIVILEGES IN SCHEMA public GRANT SELECT, INSERT, UPDATE, DELETE ON TABLES TO tvm_app;
```

### 13.2 Row-Level Security (RLS)

**Use Case:** Multi-tenant isolation (users can only access data for their facility)

```sql
-- Enable RLS on residents table
ALTER TABLE residents ENABLE ROW LEVEL SECURITY;

-- Policy: Users can only see residents in their facility
CREATE POLICY residents_facility_isolation ON residents
    USING (facility_id = current_setting('app.current_facility_id')::UUID);

-- Application sets facility_id at session start
SET app.current_facility_id = '123e4567-e89b-12d3-a456-426614174000';
```

**Note:** Adds overhead, only use if strict isolation required (e.g., SaaS multi-tenant)

---

## 14. Migration Strategy

### 14.1 Schema Versioning

**Tool:** Flyway or Liquibase (or native ORM migrations like Alembic for Python, TypeORM for Node.js)

**Migration Files:**
```
migrations/
├── V001__create_residents_table.sql
├── V002__create_vehicles_table.sql
├── V003__create_missions_table.sql
├── V004__add_mission_priority.sql
└── V005__add_telemetry_partitions.sql
```

**Naming Convention:** `V<version>__<description>.sql`

### 14.2 Migration Best Practices

**1. Backward Compatibility:**
- Add columns with DEFAULT values (old code continues to work)
- Drop columns in separate migration (after code updated)

**2. Test in Staging:**
- Run migration in staging first
- Verify application still works

**3. Rollback Plan:**
- Document rollback procedure (e.g., `V004_rollback.sql`)
- For complex migrations, use transactions

**4. Zero-Downtime Migrations:**
- Add new column (with default)
- Deploy code that writes to both old and new columns
- Backfill new column (background job)
- Deploy code that reads from new column only
- Drop old column

---

## 15. Monitoring and Alerting

### 15.1 Database Metrics

**Key Metrics (Prometheus):**
- **Connection Count:** `pg_stat_database_numbackends`
- **Query Rate:** `pg_stat_database_xact_commit` / time
- **Slow Queries:** Custom exporter (queries >1 second)
- **Disk Usage:** `pg_database_size`
- **Replication Lag:** `pg_stat_replication_replay_lag`

**Alerts:**
- Connection count >80% of max → Scale up or investigate connection leak
- Slow query rate >10/min → Investigate query performance
- Disk usage >90% → Expand storage or cleanup
- Replication lag >5 min → Check network, investigate standby performance

### 15.2 Grafana Dashboard

**Panels:**
1. **Query Rate (QPS):** Line chart, last 24 hours
2. **Slow Queries:** Table, top 10 slow queries
3. **Connection Pool:** Gauge, active/idle/waiting connections
4. **Disk Usage:** Gauge, percentage full
5. **Replication Lag:** Line chart, secondary region lag

---

## Appendix A: Helper Functions

### A.1 `update_updated_at_column()` Function

```sql
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;
```

**Usage:** Trigger to automatically update `updated_at` column on row update

---

## Appendix B: Seed Data Script

```sql
-- Create default facility
INSERT INTO facilities (id, name, address, timezone)
VALUES ('123e4567-e89b-12d3-a456-426614174000', 'Main Campus', '123 Healthcare Ave', 'America/New_York');

-- Create default admin user
INSERT INTO users (facility_id, email, password_hash, first_name, last_name, status)
VALUES (
    '123e4567-e89b-12d3-a456-426614174000',
    'admin@example.com',
    '$2a$12$examplehash',  -- bcrypt hash of 'changeme'
    'Admin',
    'User',
    'active'
);

-- Assign admin role
INSERT INTO user_roles (user_id, role_id)
SELECT
    (SELECT id FROM users WHERE email = 'admin@example.com'),
    (SELECT id FROM roles WHERE name = 'admin');
```

---

## Approval

| Role | Name | Signature | Date |
|------|------|-----------|------|
| **Project Lead** | Maeda-san | __________ | _______ |
| **TVM Lead** | Unno-san | __________ | _______ |
| **Database Architect** | TBD | __________ | _______ |

---

**Document Metadata:**
- **Version:** 1.0
- **Created:** 2025-12-17
- **Next Review:** 2026-03-17 (quarterly review)
- **Owner:** Team 2 - TVM Backend

---

**Related Documents:**
- `DEPLOYMENT_ARCHITECTURE.md` - RDS deployment, backups
- `SECURITY_ARCHITECTURE.md` - Database security, access control
- `PRIVACY_REQUIREMENTS.md` - Data retention, GDPR compliance
- `DISASTER_RECOVERY_PLAN.md` - Backup and recovery procedures

---

*End of Document*
