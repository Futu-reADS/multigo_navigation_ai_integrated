# Database Architecture

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Document Type:** Database Architecture Specification
**Team:** Unno (Fleet Management)
**Date:** December 16, 2025
**Version:** 1.0

---

## Database Technology

**Primary Database:** PostgreSQL 15
- ACID compliance for mission-critical data
- JSON/JSONB support for flexible schemas
- Full-text search capabilities
- PostGIS extension for geospatial queries

**Cache/Session Store:** Redis 7.x
- Session management (JWT token blacklist)
- Real-time vehicle status cache
- Rate limiting counters
- Background job queue (Bull)

---

## Schema Overview

### Core Tables

**users**
```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    full_name VARCHAR(255) NOT NULL,
    role_id UUID REFERENCES roles(id),
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW(),
    last_login TIMESTAMP,
    status VARCHAR(20) DEFAULT 'active'
);
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_role ON users(role_id);
```

**roles**
```sql
CREATE TABLE roles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    name VARCHAR(50) UNIQUE NOT NULL, -- admin, operator, nurse, caregiver
    permissions JSONB NOT NULL,
    created_at TIMESTAMP DEFAULT NOW()
);
```

**vehicles**
```sql
CREATE TABLE vehicles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    vehicle_id VARCHAR(20) UNIQUE NOT NULL, -- VEH-001
    name VARCHAR(100),
    status VARCHAR(20) DEFAULT 'offline', -- idle, navigating, charging, error
    battery_percent INTEGER,
    location GEOMETRY(POINT, 4326), -- PostGIS for lat/lon
    last_seen TIMESTAMP,
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);
CREATE INDEX idx_vehicles_status ON vehicles(status);
CREATE INDEX idx_vehicles_location ON vehicles USING GIST(location);
```

**missions**
```sql
CREATE TABLE missions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    vehicle_id UUID REFERENCES vehicles(id),
    type VARCHAR(50) NOT NULL, -- walking_assistance, medicine_delivery
    priority VARCHAR(20) DEFAULT 'routine', -- routine, urgent, emergency
    status VARCHAR(50) DEFAULT 'pending',
    pickup_location GEOMETRY(POINT, 4326),
    dropoff_location GEOMETRY(POINT, 4326),
    created_by UUID REFERENCES users(id),
    created_at TIMESTAMP DEFAULT NOW(),
    started_at TIMESTAMP,
    completed_at TIMESTAMP,
    metadata JSONB -- flexible additional data
);
CREATE INDEX idx_missions_vehicle ON missions(vehicle_id);
CREATE INDEX idx_missions_status ON missions(status);
CREATE INDEX idx_missions_created_at ON missions(created_at DESC);
```

**reservations**
```sql
CREATE TABLE reservations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    patient_name VARCHAR(255) NOT NULL,
    type VARCHAR(50) NOT NULL,
    priority VARCHAR(20) DEFAULT 'routine',
    scheduled_time TIMESTAMP NOT NULL,
    pickup_location GEOMETRY(POINT, 4326),
    dropoff_location GEOMETRY(POINT, 4326),
    recurring_pattern VARCHAR(50), -- null, daily, weekly
    recurring_until DATE,
    created_by UUID REFERENCES users(id),
    created_at TIMESTAMP DEFAULT NOW(),
    status VARCHAR(20) DEFAULT 'scheduled' -- scheduled, completed, cancelled
);
CREATE INDEX idx_reservations_scheduled_time ON reservations(scheduled_time);
CREATE INDEX idx_reservations_status ON reservations(status);
```

**telemetry_logs** (Partitioned by month)
```sql
CREATE TABLE telemetry_logs (
    id BIGSERIAL,
    vehicle_id UUID REFERENCES vehicles(id),
    timestamp TIMESTAMP NOT NULL,
    location GEOMETRY(POINT, 4326),
    battery_percent INTEGER,
    speed FLOAT,
    status VARCHAR(50),
    metadata JSONB
) PARTITION BY RANGE (timestamp);

CREATE TABLE telemetry_logs_2025_12 PARTITION OF telemetry_logs
    FOR VALUES FROM ('2025-12-01') TO ('2026-01-01');
```

**audit_logs**
```sql
CREATE TABLE audit_logs (
    id BIGSERIAL PRIMARY KEY,
    user_id UUID REFERENCES users(id),
    action VARCHAR(100) NOT NULL,
    resource_type VARCHAR(50),
    resource_id UUID,
    ip_address INET,
    timestamp TIMESTAMP DEFAULT NOW(),
    details JSONB
);
CREATE INDEX idx_audit_user ON audit_logs(user_id);
CREATE INDEX idx_audit_timestamp ON audit_logs(timestamp DESC);
```

---

## Redis Schema

### Cache Keys

```
vehicle:{vehicle_id}:status → JSON (5s TTL)
vehicle:{vehicle_id}:location → JSON (5s TTL)
user:session:{token} → user_id (24h TTL)
rate_limit:{ip}:{endpoint} → count (1min TTL)
mission:{mission_id}:progress → JSON (10s TTL)
```

### Queue (Bull)

```
queue:mission_scheduler → Mission scheduling jobs
queue:analytics → Analytics computation jobs
queue:notifications → Email/SMS notification jobs
```

---

## Backup and Recovery

- **Daily backups:** Full PostgreSQL backup at 02:00 UTC
- **Point-in-time recovery:** WAL archiving enabled
- **Retention:** 30 days backup retention
- **Recovery Time Objective (RTO):** <1 hour
- **Recovery Point Objective (RPO):** <15 minutes

---

## Performance Optimization

- **Connection Pooling:** pgBouncer (100 connections)
- **Query Optimization:** EXPLAIN ANALYZE for slow queries
- **Indexing Strategy:** Composite indexes on frequently queried columns
- **Partitioning:** telemetry_logs partitioned by month
- **Vacuum:** Auto-vacuum enabled

---

## Security

- **Encryption at Rest:** AWS RDS encryption
- **Encryption in Transit:** SSL/TLS connections
- **Row-Level Security:** Enabled for multi-tenancy (future)
- **Access Control:** IAM-based database access
- **Audit Logging:** All DDL/DML operations logged

---

**Document End**
