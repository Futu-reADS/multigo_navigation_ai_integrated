# Database Schema Design

**Team:** Unno (Fleet Management)
**Date:** 2025-12-16
**Version:** 1.0

## ER Diagram

```
┌─────────┐       ┌──────────┐       ┌──────────────┐
│  users  │───────│  roles   │       │ reservations │
└────┬────┘   1:N └──────────┘       └──────┬───────┘
     │                                       │
     │ created_by                            │ created_by
     │                                       │
     └────────────┬──────────────────────────┘
                  │
                  │ 1:N
              ┌───▼────────┐
              │  missions  │
              └───┬────────┘
                  │ N:1
                  │
              ┌───▼──────┐
              │ vehicles │
              └──────────┘
```

## Index Strategy

### High-Traffic Queries

```sql
-- Query: Get active missions for vehicle
SELECT * FROM missions 
WHERE vehicle_id = ? AND status IN ('pending', 'in_progress');

-- Optimal index:
CREATE INDEX idx_missions_vehicle_status ON missions(vehicle_id, status);

-- Query: Recent reservations for user
SELECT * FROM reservations 
WHERE created_by = ? 
ORDER BY scheduled_time DESC 
LIMIT 20;

-- Optimal index:
CREATE INDEX idx_reservations_user_time ON reservations(created_by, scheduled_time DESC);
```

### Partitioning Strategy

```sql
-- telemetry_logs: Monthly partitions
-- Keep 3 months of detailed data, aggregate older data

CREATE TABLE telemetry_logs_2025_12 PARTITION OF telemetry_logs
    FOR VALUES FROM ('2025-12-01') TO ('2026-01-01');

-- Auto-partition creation (cron job)
-- Archive strategy: After 3 months, aggregate to daily summaries
```

---
**References:** DATABASE_ARCHITECTURE.md
