# Observability Architecture

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Document Type:** Architecture Specification
**Status:** Active
**Version:** 1.0
**Last Updated:** 2025-12-17
**Owner:** Integration Team (Team 2 - TVM Operations)

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Observability Principles](#2-observability-principles)
3. [Three Pillars of Observability](#3-three-pillars-of-observability)
4. [Monitoring Architecture](#4-monitoring-architecture)
5. [Metrics Collection](#5-metrics-collection)
6. [Logging Architecture](#6-logging-architecture)
7. [Distributed Tracing](#7-distributed-tracing)
8. [Alerting and Notifications](#8-alerting-and-notifications)
9. [Dashboards](#9-dashboards)
10. [SLIs, SLOs, and SLAs](#10-slis-slos-and-slas)
11. [Incident Detection and Response](#11-incident-detection-and-response)
12. [Performance Monitoring](#12-performance-monitoring)
13. [Cost and Resource Monitoring](#13-cost-and-resource-monitoring)
14. [Observability for Compliance](#14-observability-for-compliance)

---

## 1. Executive Summary

### 1.1 Purpose

This document defines the observability architecture for the outdoor wheelchair transport robot fleet system. Observability enables us to:
- **Understand System State:** What is the system doing right now?
- **Debug Issues:** Why did the system fail?
- **Optimize Performance:** Where are the bottlenecks?
- **Ensure Reliability:** Are we meeting SLAs?
- **Detect Security Incidents:** Is someone attacking us?

### 1.2 Observability Goals

1. **Proactive Problem Detection:** Detect issues before users report them
2. **Rapid Troubleshooting:** Mean Time To Detect (MTTD) <1 minute, Mean Time To Resolve (MTTR) <1 hour
3. **Data-Driven Decisions:** Use metrics to optimize system performance and cost
4. **Compliance:** Audit logs for GDPR, HIPAA, ISO 27001
5. **User Experience:** Track mission success rate, response times

### 1.3 Technology Stack

| Component | Technology | Purpose |
|-----------|------------|---------|
| **Metrics** | Prometheus + Grafana | Time-series metrics, visualization |
| **Logging** | ELK Stack (Elasticsearch, Logstash, Kibana) | Centralized logging, search |
| **Tracing** | Jaeger (optional, for complex debugging) | Distributed tracing |
| **Alerting** | Prometheus Alertmanager + PagerDuty | Alert routing, on-call |
| **Uptime Monitoring** | UptimeRobot or Pingdom | External health checks |
| **APM** | DataDog or New Relic (optional, commercial) | Application performance monitoring |

---

## 2. Observability Principles

### 2.1 Design for Observability

**Principle 1: Instrumentation First**
- All code should emit metrics, logs, and traces
- Instrumentation is not an afterthought (add during development)

**Principle 2: High-Cardinality Data**
- Support high-cardinality dimensions (e.g., user_id, vehicle_id)
- Enable debugging specific entities (e.g., "Why did Vehicle #5 fail?")

**Principle 3: Actionable Alerts**
- Alerts must be actionable (not noise)
- Every alert should have a runbook (what to do)

**Principle 4: Contextual Logging**
- Logs include context: request_id, user_id, vehicle_id, timestamp
- Enables correlation across services

**Principle 5: Open Standards**
- Use open standards (Prometheus, OpenTelemetry)
- Avoid vendor lock-in

---

## 3. Three Pillars of Observability

### 3.1 Metrics

**What:** Numerical measurements over time (CPU usage, request rate, error count)
**Purpose:** Track trends, detect anomalies, trigger alerts
**Tool:** Prometheus + Grafana

**Examples:**
- `http_requests_total{endpoint="/api/missions", status="200"}` → 15,342
- `vehicle_battery_level{vehicle_id="V123"}` → 67%
- `mission_success_rate` → 98.5%

### 3.2 Logs

**What:** Textual records of events (errors, warnings, info messages)
**Purpose:** Debugging, audit trail, compliance
**Tool:** ELK Stack (Elasticsearch, Logstash, Kibana)

**Examples:**
```
2025-12-17T10:30:00Z [ERROR] Vehicle V123 navigation failed: Path planning timeout
2025-12-17T10:30:05Z [INFO] Mission M456 assigned to Vehicle V5
2025-12-17T10:30:10Z [WARN] Database query slow: SELECT * FROM missions took 2.3s
```

### 3.3 Traces

**What:** Request flow across services (e.g., API → TVM Server → Database → Vehicle)
**Purpose:** Understand latency, identify bottlenecks
**Tool:** Jaeger (optional, use if microservices architecture)

**Example:**
```
Trace ID: abc123
Span 1: API Gateway (50ms)
  ├─ Span 2: TVM Server /api/missions (200ms)
  │   ├─ Span 3: Database query (150ms)
  │   └─ Span 4: Vehicle command send (30ms)
```

---

## 4. Monitoring Architecture

### 4.1 High-Level Architecture

```
┌────────────────────────────────────────────────────────────────┐
│                     Data Sources                               │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐      │
│  │ TVM      │  │ Vehicles │  │ Database │  │ Infra    │      │
│  │ Server   │  │ (ROS 2)  │  │ (RDS)    │  │ (AWS)    │      │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘      │
│       │             │              │             │             │
│       │ /metrics    │ /metrics     │ Exporter    │ CloudWatch  │
└───────┼─────────────┼──────────────┼─────────────┼─────────────┘
        │             │              │             │
        └─────────────┴──────────────┴─────────────┘
                             ↓
        ┌─────────────────────────────────────────────────┐
        │          Prometheus (Metrics Storage)           │
        │  - Scrapes /metrics endpoints every 15 seconds  │
        │  - Stores time-series data (15-day retention)   │
        └─────────────────┬───────────────────────────────┘
                          │
                          ↓
        ┌─────────────────────────────────────────────────┐
        │              Grafana (Visualization)            │
        │  - Dashboards (fleet status, mission metrics)   │
        │  - Alerts (CPU > 80%, mission failure rate)     │
        └─────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────┐
│                     Log Sources                                │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                     │
│  │ TVM      │  │ Vehicles │  │ Database │                     │
│  │ Server   │  │ (ROS 2)  │  │ (RDS)    │                     │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘                     │
│       │             │              │                            │
│       │ Filebeat    │ Filebeat     │ CloudWatch Logs            │
└───────┼─────────────┼──────────────┼─────────────────────────┘
        └─────────────┴──────────────┘
                     ↓
        ┌─────────────────────────────────────────────────┐
        │          Elasticsearch (Log Storage)            │
        │  - Index logs by date (daily indices)           │
        │  - 90-day retention (hot-warm-cold strategy)    │
        └─────────────────┬───────────────────────────────┘
                          │
                          ↓
        ┌─────────────────────────────────────────────────┐
        │              Kibana (Log Exploration)           │
        │  - Search logs, filter by vehicle/mission       │
        │  - Visualize error trends                       │
        └─────────────────────────────────────────────────┘
```

### 4.2 Deployment

**Monitoring Infrastructure:**
- **Prometheus Server:** ECS Fargate (1 vCPU, 2 GB RAM)
- **Grafana Server:** ECS Fargate (0.5 vCPU, 1 GB RAM)
- **Elasticsearch Cluster:** AWS Managed Elasticsearch (3 nodes, t3.medium)
- **Kibana:** Bundled with AWS Elasticsearch

**High Availability:**
- **Prometheus:** Single instance (acceptable, monitoring is not critical path)
- **Elasticsearch:** 3-node cluster (multi-AZ, automatic failover)

---

## 5. Metrics Collection

### 5.1 TVM Server Metrics

**Instrumentation:** Prometheus client library (Node.js: prom-client, Python: prometheus_client)

**Endpoint:** `GET /metrics` (Prometheus format)

**Example Metrics:**

```prometheus
# HTTP Request Count
http_requests_total{method="POST", endpoint="/api/missions", status="200"} 15342

# HTTP Request Duration (histogram)
http_request_duration_seconds_bucket{endpoint="/api/missions", le="0.1"} 12000
http_request_duration_seconds_bucket{endpoint="/api/missions", le="0.5"} 14500
http_request_duration_seconds_sum{endpoint="/api/missions"} 2453.2
http_request_duration_seconds_count{endpoint="/api/missions"} 15342

# Database Query Duration
db_query_duration_seconds{query="SELECT missions", table="missions"} 0.025

# Mission Metrics
missions_total{status="completed"} 4523
missions_total{status="failed"} 12
mission_success_rate 99.7  # Calculated metric

# Vehicle Metrics
vehicles_online 8
vehicles_total 10

# Error Count
errors_total{severity="critical", component="navigation"} 3
```

**Custom Metrics:**
```javascript
// Node.js example
const promClient = require('prom-client');

const missionCounter = new promClient.Counter({
  name: 'missions_total',
  help: 'Total number of missions',
  labelNames: ['status']
});

// Increment on mission completion
missionCounter.labels('completed').inc();
```

### 5.2 Vehicle Metrics

**Instrumentation:** ROS 2 node publishes metrics to TVM, TVM exposes on `/metrics`

**Key Metrics:**
```prometheus
# Vehicle Status
vehicle_status{vehicle_id="V123", status="idle"} 1

# Battery Level
vehicle_battery_level{vehicle_id="V123"} 67  # Percentage

# Location (for geofencing alerts)
vehicle_latitude{vehicle_id="V123"} 35.6
vehicle_longitude{vehicle_id="V123"} 139.7

# Sensor Health
vehicle_sensor_status{vehicle_id="V123", sensor="lidar", status="ok"} 1
vehicle_sensor_status{vehicle_id="V123", sensor="camera", status="error"} 1

# Error Count
vehicle_errors_total{vehicle_id="V123", error_code="VEH-NAV-001"} 2

# Network Connectivity
vehicle_last_heartbeat_seconds_ago{vehicle_id="V123"} 5
```

**Collection:** Vehicle pushes telemetry to TVM every 1 second, TVM aggregates and exposes on `/metrics`

### 5.3 Database Metrics

**Exporter:** `postgres_exporter` (Prometheus exporter for PostgreSQL)

**Deployment:** Sidecar container in ECS task, or standalone EC2

**Key Metrics:**
```prometheus
# Connection Count
pg_stat_database_numbackends{database="tvm"} 42

# Query Rate
pg_stat_database_xact_commit{database="tvm"} 15234  # Total commits (rate calculated)

# Slow Queries (custom query via postgres_exporter)
pg_slow_queries_count{query_hash="abc123"} 5

# Disk Usage
pg_database_size_bytes{database="tvm"} 10737418240  # 10 GB

# Replication Lag (for standby DB)
pg_replication_lag_seconds{application_name="standby"} 0.5
```

### 5.4 Infrastructure Metrics

**AWS CloudWatch → Prometheus:**
- Use `cloudwatch_exporter` to scrape CloudWatch metrics into Prometheus

**Key Metrics:**
```prometheus
# ECS Task Metrics
aws_ecs_service_desired_count{service="tvm-server"} 3
aws_ecs_service_running_count{service="tvm-server"} 3

# RDS Metrics
aws_rds_cpuutilization{dbinstance_identifier="tvm-production-primary"} 45.2
aws_rds_freeable_memory_bytes{dbinstance_identifier="tvm-production-primary"} 4294967296  # 4 GB

# ALB Metrics
aws_applicationelb_target_response_time_seconds{load_balancer="tvm-alb"} 0.12
aws_applicationelb_request_count_sum{load_balancer="tvm-alb"} 152345
```

---

## 6. Logging Architecture

### 6.1 Log Levels

| Level | Use Case | Example |
|-------|----------|---------|
| **DEBUG** | Detailed diagnostic info (disabled in production) | "Database query: SELECT * FROM missions WHERE id = '...'" |
| **INFO** | General informational messages | "Mission M456 assigned to Vehicle V5" |
| **WARN** | Potentially harmful situations | "Database query slow: took 2.3 seconds" |
| **ERROR** | Error events (recoverable) | "Mission creation failed: Invalid coordinates" |
| **CRITICAL** | Severe errors (require immediate action) | "Database connection lost" |

### 6.2 Log Format (JSON)

**Structured Logging:** All logs in JSON format (parseable, searchable)

```json
{
  "timestamp": "2025-12-17T10:30:00.123Z",
  "level": "ERROR",
  "service": "tvm-server",
  "component": "mission_service",
  "message": "Mission creation failed",
  "error": {
    "code": "TVM-VAL-001",
    "message": "Invalid pickup coordinates",
    "stack_trace": "..."
  },
  "context": {
    "request_id": "req-abc123",
    "user_id": "user-456",
    "mission_id": "mission-789"
  },
  "metadata": {
    "host": "tvm-server-1",
    "environment": "production",
    "version": "v1.2.3"
  }
}
```

**Benefits:**
- **Parseable:** Elasticsearch can index all fields
- **Searchable:** Filter by `request_id`, `user_id`, `error.code`
- **Contextual:** Full context for debugging

### 6.3 Log Aggregation (ELK Stack)

**Filebeat Configuration:**

```yaml
# filebeat.yml (on TVM server)
filebeat.inputs:
- type: log
  paths:
    - /var/log/tvm-server/*.log
  json.keys_under_root: true
  json.add_error_key: true

output.elasticsearch:
  hosts: ["elasticsearch.tvm.internal:9200"]
  index: "tvm-logs-%{+yyyy.MM.dd}"
```

**Elasticsearch Index Lifecycle:**

```
tvm-logs-2025-12-17 (hot)   →  7 days  → warm  →  30 days → cold  →  90 days → delete
```

**Index Mapping:**
```json
{
  "mappings": {
    "properties": {
      "timestamp": {"type": "date"},
      "level": {"type": "keyword"},
      "service": {"type": "keyword"},
      "message": {"type": "text"},
      "error.code": {"type": "keyword"},
      "context.request_id": {"type": "keyword"},
      "context.user_id": {"type": "keyword"}
    }
  }
}
```

### 6.4 Log Queries (Kibana)

**Example Queries:**

**Query 1: Find all errors for Mission M456**
```
context.mission_id:"mission-789" AND level:"ERROR"
```

**Query 2: Find slow database queries (>1 second)**
```
component:"database" AND message:"slow query" AND duration:>1000
```

**Query 3: Count errors by error code (aggregation)**
```
level:"ERROR" | agg: terms(error.code)
```

**Result:**
```
TVM-VAL-001: 42
TVM-DB-002: 15
VEH-NAV-001: 8
```

---

## 7. Distributed Tracing

### 7.1 Tracing Architecture (Optional)

**Use Case:** Debug latency issues in complex request flows

**Tool:** Jaeger (open-source, CNCF project)

**Deployment:** ECS Fargate (Jaeger all-in-one: collector + query + UI)

**Instrumentation:** OpenTelemetry SDKs

**Example Trace:**

```
Trace ID: req-abc123
Total Duration: 250ms

Span 1: API Gateway (10ms)
  └─ Span 2: TVM Server POST /api/missions (240ms)
      ├─ Span 3: Validate input (5ms)
      ├─ Span 4: Database INSERT missions (150ms)  ← SLOW!
      └─ Span 5: Send command to Vehicle V5 (80ms)
```

**Analysis:** Database INSERT is slow (150ms) → Investigate query, check indexing

**Adoption Strategy:** Implement tracing in Phase 2 (after MVP, if latency issues arise)

---

## 8. Alerting and Notifications

### 8.1 Alert Routing

```
┌─────────────────────────────────────────────────────────┐
│              Prometheus Alertmanager                    │
│  - Receives alerts from Prometheus                      │
│  - Deduplicates, groups, routes                         │
└─────────────────┬───────────────────────────────────────┘
                  │
       ┌──────────┴──────────┐
       │                     │
       ↓                     ↓
┌─────────────┐    ┌──────────────────┐
│  PagerDuty  │    │  Slack #alerts   │
│  (On-Call)  │    │  (Non-Critical)  │
└─────────────┘    └──────────────────┘
```

**Routing Rules:**
- **CRITICAL Alerts** → PagerDuty (page on-call engineer)
- **WARNING Alerts** → Slack #alerts channel
- **INFO Alerts** → Logged only (no notification)

### 8.2 Alert Definitions

**Alert 1: High Error Rate**

```yaml
# prometheus_alerts.yml
groups:
- name: tvm_alerts
  rules:
  - alert: HighErrorRate
    expr: |
      rate(http_requests_total{status=~"5.."}[5m])
      / rate(http_requests_total[5m]) > 0.05
    for: 5m
    labels:
      severity: critical
    annotations:
      summary: "High error rate detected"
      description: "Error rate is {{ $value | humanizePercentage }} (threshold: 5%)"
      runbook: "https://wiki.tvm.example.com/runbooks/high-error-rate"
```

**Alert Triggers:**
- If >5% of HTTP requests return 5xx errors for 5 consecutive minutes
- **Action:** Page on-call engineer, investigate immediately

**Alert 2: Vehicle Offline**

```yaml
- alert: VehicleOffline
  expr: |
    time() - vehicle_last_heartbeat_seconds_ago > 300
  labels:
    severity: warning
  annotations:
    summary: "Vehicle {{ $labels.vehicle_id }} is offline"
    description: "Last heartbeat received {{ $value }} seconds ago (threshold: 300s)"
    runbook: "https://wiki.tvm.example.com/runbooks/vehicle-offline"
```

**Alert Triggers:**
- If vehicle hasn't sent heartbeat in >5 minutes
- **Action:** Slack notification, dispatch technician

**Alert 3: Low Battery**

```yaml
- alert: VehicleLowBattery
  expr: |
    vehicle_battery_level < 20
  labels:
    severity: warning
  annotations:
    summary: "Vehicle {{ $labels.vehicle_id }} battery low"
    description: "Battery level is {{ $value }}% (threshold: 20%)"
    runbook: "https://wiki.tvm.example.com/runbooks/low-battery"
```

**Alert 4: Database Connection Pool Exhausted**

```yaml
- alert: DatabasePoolExhausted
  expr: |
    pg_stat_database_numbackends{database="tvm"} > 90
  labels:
    severity: critical
  annotations:
    summary: "Database connection pool near capacity"
    description: "Connection count: {{ $value }} (max: 100)"
    runbook: "https://wiki.tvm.example.com/runbooks/db-pool-exhausted"
```

### 8.3 Alert Fatigue Prevention

**Principles:**
1. **Actionable Alerts Only:** Every alert must require action (not just FYI)
2. **Runbooks:** Every alert links to runbook (what to do)
3. **Noise Reduction:** Aggregate similar alerts (e.g., "5 vehicles offline" → 1 alert)
4. **Tuning:** Review alerts quarterly, disable noisy alerts

**Alert Tuning:**
- **Too Many Alerts?** Increase threshold (e.g., error rate 5% → 10%)
- **Too Few Alerts?** Decrease threshold or add new alert

---

## 9. Dashboards

### 9.1 Dashboard Hierarchy

**1. Executive Dashboard (High-Level KPIs)**
- Audience: Leadership, non-technical stakeholders
- Update Frequency: 1 minute
- Panels: Mission success rate, vehicle uptime, fleet utilization

**2. Operations Dashboard (Real-Time Monitoring)**
- Audience: Operators, control room
- Update Frequency: 5 seconds
- Panels: Vehicle locations (map), active missions, alerts

**3. Engineering Dashboard (System Health)**
- Audience: Developers, DevOps
- Update Frequency: 15 seconds
- Panels: Request rate, error rate, latency, database performance

**4. Vehicle Dashboard (Per-Vehicle Metrics)**
- Audience: Maintenance team
- Update Frequency: 1 second
- Panels: Battery level, sensor status, error logs

### 9.2 Executive Dashboard (Grafana)

**Panels:**

**Panel 1: Mission Success Rate (Gauge)**
```prometheus
# Query
sum(rate(missions_total{status="completed"}[1h]))
/ sum(rate(missions_total[1h])) * 100
```
**Display:** Gauge, target: 99%, threshold: 95% (yellow), 90% (red)

**Panel 2: Fleet Availability (Stat)**
```prometheus
# Query
vehicles_online / vehicles_total * 100
```
**Display:** Stat, target: 100%, threshold: 80% (yellow)

**Panel 3: Missions Today (Graph)**
```prometheus
# Query
sum(increase(missions_total[1d]))
```
**Display:** Line graph (last 7 days)

**Panel 4: Average Mission Duration (Stat)**
```prometheus
# Query
avg(mission_duration_seconds) / 60
```
**Display:** Stat (in minutes)

### 9.3 Operations Dashboard

**Panels:**

**Panel 1: Vehicle Fleet Map (Worldmap Panel)**
- Datasource: PostgreSQL (query `vehicle_telemetry` table)
- Display: Interactive map, vehicle icons (color-coded by status)

**Panel 2: Active Missions (Table)**
```sql
SELECT
    mission_id,
    resident_name,
    vehicle_name,
    status,
    created_at
FROM missions
WHERE status IN ('pending', 'assigned', 'in_progress')
ORDER BY priority DESC, created_at ASC
LIMIT 50;
```

**Panel 3: Alert Summary (Alert List Panel)**
- Datasource: Alertmanager
- Display: List of active alerts (color-coded by severity)

### 9.4 Engineering Dashboard

**Panels:**

**Panel 1: Request Rate (Graph)**
```prometheus
# Query
sum(rate(http_requests_total[5m])) by (endpoint)
```
**Display:** Multi-line graph (one line per endpoint)

**Panel 2: Error Rate (Graph)**
```prometheus
# Query
sum(rate(http_requests_total{status=~"5.."}[5m]))
/ sum(rate(http_requests_total[5m])) * 100
```
**Display:** Line graph, threshold: 1% (yellow), 5% (red)

**Panel 3: Latency Percentiles (Graph)**
```prometheus
# Query (p50, p95, p99)
histogram_quantile(0.50, rate(http_request_duration_seconds_bucket[5m]))
histogram_quantile(0.95, rate(http_request_duration_seconds_bucket[5m]))
histogram_quantile(0.99, rate(http_request_duration_seconds_bucket[5m]))
```
**Display:** Multi-line graph

**Panel 4: Database Query Duration (Graph)**
```prometheus
# Query
rate(db_query_duration_seconds_sum[5m])
/ rate(db_query_duration_seconds_count[5m])
```
**Display:** Line graph

---

## 10. SLIs, SLOs, and SLAs

### 10.1 Definitions

- **SLI (Service Level Indicator):** Metric that measures service quality (e.g., request success rate)
- **SLO (Service Level Objective):** Target for SLI (e.g., 99.5% of requests succeed)
- **SLA (Service Level Agreement):** Contract with customer (e.g., 99% uptime or refund)

### 10.2 TVM System SLOs

| SLI | SLO Target | Measurement Window | Current (Example) |
|-----|------------|--------------------|-------------------|
| **Mission Success Rate** | ≥99% | Rolling 7 days | 99.3% ✅ |
| **API Availability** | ≥99.5% | Rolling 30 days | 99.7% ✅ |
| **API Latency (p95)** | <500ms | Rolling 1 hour | 320ms ✅ |
| **Vehicle Uptime** | ≥95% | Rolling 30 days | 97.1% ✅ |
| **Data Loss** | 0% | Per incident | 0% ✅ |

**Mission Success Rate Calculation:**
```prometheus
sum(increase(missions_total{status="completed"}[7d]))
/ sum(increase(missions_total[7d])) * 100
```

**API Availability Calculation:**
```prometheus
1 - (
  sum(rate(http_requests_total{status=~"5.."}[30d]))
  / sum(rate(http_requests_total[30d]))
)
```

### 10.3 Error Budget

**Error Budget:** Allowed failure rate before violating SLO

**Example:**
- **SLO:** 99% mission success rate (7 days)
- **Error Budget:** 1% = 0.01 × 10,000 missions = 100 failed missions allowed per week
- **Current Failures:** 70 missions failed this week
- **Remaining Error Budget:** 30 missions (30% budget remaining)

**Policy:**
- **If Error Budget >50%:** Ship new features (normal pace)
- **If Error Budget 20-50%:** Focus on reliability (slow down feature velocity)
- **If Error Budget <20%:** Freeze feature releases, focus on reliability only

---

## 11. Incident Detection and Response

### 11.1 Incident Lifecycle

```
1. Detection (MTTD: <1 minute)
   - Prometheus alert fires
   - Alertmanager routes to PagerDuty
   ↓
2. Triage (5 minutes)
   - On-call engineer acknowledges alert
   - Assess severity (P0/P1/P2/P3)
   ↓
3. Mitigation (MTTR: <1 hour)
   - Follow runbook
   - Apply immediate fix (rollback, restart, scale up)
   ↓
4. Resolution
   - Verify fix (metrics return to normal)
   - All clear
   ↓
5. Post-Mortem (within 48 hours)
   - Root cause analysis
   - Action items to prevent recurrence
```

### 11.2 Incident Severity Levels

| Severity | Definition | Example | Response Time |
|----------|------------|---------|---------------|
| **P0 (Critical)** | Service down, data loss, safety risk | TVM server down, vehicle collision | <15 min |
| **P1 (High)** | Major functionality impaired | Mission creation fails, 50% error rate | <1 hour |
| **P2 (Medium)** | Minor functionality impaired | Slow dashboard, 5% error rate | <4 hours |
| **P3 (Low)** | Cosmetic issue, no functional impact | Typo in UI, slow log indexing | <24 hours |

### 11.3 Runbooks

**Runbook Example: High Error Rate**

**Symptoms:**
- Alert: "HighErrorRate" firing
- Dashboard: Error rate >5% for 5 minutes
- Logs: Multiple 500 errors in Kibana

**Diagnosis:**
1. Check Grafana: Which endpoint is failing? (`/api/missions`?)
2. Check Kibana: Error logs for that endpoint
   - Query: `endpoint:"/api/missions" AND level:"ERROR"`
3. Identify error type: Database error? Validation error? External API timeout?

**Mitigation:**
- **If Database Error:** Check database health (CPU, connections), restart if needed
- **If Validation Error:** Recent code change? Rollback deployment
- **If External API Timeout:** Increase timeout, enable circuit breaker

**Resolution:**
- Verify error rate returns to <1%
- Monitor for 15 minutes
- All clear

**Post-Incident:**
- Create incident ticket (Jira)
- Schedule post-mortem meeting
- Update runbook with learnings

---

## 12. Performance Monitoring

### 12.1 Application Performance Metrics

**Key Metrics:**
- **Throughput:** Requests per second (RPS)
- **Latency:** Response time (p50, p95, p99)
- **Error Rate:** Percentage of failed requests
- **Saturation:** Resource utilization (CPU, memory, disk)

**RED Method (Rate, Errors, Duration):**
1. **Rate:** How many requests per second?
2. **Errors:** How many of those requests failed?
3. **Duration:** How long did those requests take?

**USE Method (Utilization, Saturation, Errors):**
1. **Utilization:** How busy is the resource? (CPU %)
2. **Saturation:** Is there queueing? (Request queue length)
3. **Errors:** Are there errors? (Error count)

### 12.2 Performance Bottleneck Identification

**Step 1: Identify Slow Requests**
```prometheus
# Query: p99 latency by endpoint
histogram_quantile(0.99, sum(rate(http_request_duration_seconds_bucket[5m])) by (endpoint, le))
```

**Result:** `/api/missions` has p99 latency of 2.5 seconds (slow!)

**Step 2: Drill Down with Tracing**
- Use Jaeger to trace a slow request
- Identify which span is slow (database query? external API call?)

**Step 3: Optimize**
- If database query slow: Add index, optimize query
- If external API slow: Increase timeout, cache response

### 12.3 Capacity Planning

**Monitor Trends:**
- **Request Rate:** Growing 10% month-over-month?
- **Database Size:** Growing 5 GB/month?
- **CPU Usage:** Consistently >70%?

**Forecast:**
- If trends continue, when will we hit capacity?
- Example: CPU usage at 70%, growing 5%/month → Reach 100% in 6 months

**Action:**
- Scale up resources proactively (increase instance size, add replicas)

---

## 13. Cost and Resource Monitoring

### 13.1 AWS Cost Monitoring

**AWS Cost Explorer:**
- Track cost by service (EC2, RDS, S3, CloudWatch)
- Identify cost anomalies (sudden spike)

**Cost Metrics (via CloudWatch):**
```prometheus
# Estimated monthly cost (from AWS Cost Explorer API)
aws_estimated_charges_usd{service="AmazonRDS"} 200.50
aws_estimated_charges_usd{service="AmazonEC2"} 150.30
```

**Alerts:**
- If estimated monthly cost >$600 → Alert (budget: $500)

### 13.2 Resource Utilization

**Right-Sizing:**
- **Over-Provisioned:** CPU <30% consistently → Downsize instance
- **Under-Provisioned:** CPU >80% consistently → Upsize instance

**Example:**
```prometheus
# Query: Average CPU usage over 7 days
avg_over_time(aws_ecs_cpuutilization{service="tvm-server"}[7d])
```
**Result:** 25% average CPU → Downsize from t3.large (2 vCPU) to t3.medium (1 vCPU)

---

## 14. Observability for Compliance

### 14.1 Audit Logging

**Requirement:** Log all access to PII (GDPR Art. 30, HIPAA)

**Implementation:**
- Every read/write of `residents` table logged to `audit_logs`
- Audit logs retained for 7 years (compliance)

**Query Example (Kibana):**
```
resource_type:"resident" AND action:"read" AND user_id:"user-456"
```
**Result:** All residents viewed by User 456 (audit trail)

### 14.2 Security Monitoring

**SIEM (Security Information and Event Management):**
- Use ELK Stack as SIEM
- Monitor for security events:
  - Failed login attempts (>5 from same IP → brute force attack?)
  - Unauthorized API access (403 errors)
  - Privilege escalation (user attempts admin action)

**Alert Example:**
```yaml
- alert: BruteForceAttempt
  expr: |
    sum(rate(http_requests_total{endpoint="/auth/login", status="401"}[5m])) by (ip_address) > 10
  labels:
    severity: warning
  annotations:
    summary: "Brute force attack detected from IP {{ $labels.ip_address }}"
    description: "{{ $value }} failed login attempts in last 5 minutes"
```

---

## 15. Observability Maturity Roadmap

### 15.1 Phase 1: MVP (Weeks 1-12)

**Goal:** Basic observability (detect outages, debug simple issues)

**Deliverables:**
- [ ] Prometheus + Grafana deployed
- [ ] TVM Server metrics endpoint (`/metrics`)
- [ ] Basic dashboards (request rate, error rate, latency)
- [ ] Logging to stdout (no ELK yet)
- [ ] Critical alerts (high error rate, service down)

### 15.2 Phase 2: Production (Weeks 13-24)

**Goal:** Comprehensive observability (proactive monitoring, fast debugging)

**Deliverables:**
- [ ] ELK Stack deployed (centralized logging)
- [ ] Vehicle metrics collection
- [ ] Database metrics (postgres_exporter)
- [ ] Advanced dashboards (operations, engineering, vehicle)
- [ ] Complete alert coverage (runbooks for all alerts)
- [ ] SLOs defined and tracked

### 15.3 Phase 3: Advanced (Months 7-12)

**Goal:** Observability excellence (predictive analytics, automated remediation)

**Deliverables:**
- [ ] Distributed tracing (Jaeger)
- [ ] Anomaly detection (ML-based, e.g., Elasticsearch ML)
- [ ] Automated remediation (e.g., auto-scale on high CPU)
- [ ] Capacity planning dashboard
- [ ] Cost optimization dashboard

---

## Approval

| Role | Name | Signature | Date |
|------|------|-----------|------|
| **Project Lead** | Maeda-san | __________ | _______ |
| **TVM Lead** | Unno-san | __________ | _______ |
| **DevOps/SRE** | TBD | __________ | _______ |

---

**Document Metadata:**
- **Version:** 1.0
- **Created:** 2025-12-17
- **Next Review:** 2026-03-17 (quarterly review)
- **Owner:** Team 2 - TVM Operations

---

**Related Documents:**
- `DEPLOYMENT_ARCHITECTURE.md` - Prometheus/Grafana/ELK deployment
- `ERROR_HANDLING_REQUIREMENTS.md` - Error logging requirements
- `SECURITY_ARCHITECTURE.md` - Security monitoring, audit logs
- `DISASTER_RECOVERY_PLAN.md` - Incident response procedures

---

*End of Document*
