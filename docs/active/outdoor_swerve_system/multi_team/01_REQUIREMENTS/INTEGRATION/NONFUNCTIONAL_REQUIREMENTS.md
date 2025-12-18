# Non-Functional Requirements

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Document Type:** Requirements Specification
**Status:** Active
**Version:** 2.0 (Expanded)
**Last Updated:** 2025-12-17
**Owner:** Integration Team (All Teams)

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Performance Requirements](#2-performance-requirements)
3. [Scalability Requirements](#3-scalability-requirements)
4. [Availability and Reliability](#4-availability-and-reliability)
5. [Maintainability Requirements](#5-maintainability-requirements)
6. [Usability Requirements](#6-usability-requirements)
7. [Security Requirements](#7-security-requirements)
8. [Privacy Requirements](#8-privacy-requirements)
9. [Compatibility and Interoperability](#9-compatibility-and-interoperability)
10. [Portability Requirements](#10-portability-requirements)
11. [Testability Requirements](#11-testability-requirements)
12. [Compliance Requirements](#12-compliance-requirements)
13. [Environmental Requirements](#13-environmental-requirements)
14. [Operational Requirements](#14-operational-requirements)

---

## 1. Introduction

### 1.1 Purpose

This document specifies the non-functional requirements (NFRs) for the outdoor wheelchair transport robot fleet system. NFRs define **how** the system should perform (quality attributes), as opposed to functional requirements which define **what** the system should do.

### 1.2 Scope

NFRs apply to all three teams:
- **Team 1 (Pankaj):** Vehicle software (performance, reliability, safety)
- **Team 2 (Unno):** TVM server (scalability, availability, security)
- **Team 3 (Maeda-san):** Hardware (environmental, durability, safety)

### 1.3 ISO 25010 Quality Model

Requirements organized using ISO/IEC 25010 quality model:

```
Quality Attributes
├── Performance Efficiency
│   ├── Time Behavior (response time, throughput)
│   ├── Resource Utilization (CPU, memory, bandwidth)
│   └── Capacity (scalability)
├── Reliability
│   ├── Availability (uptime)
│   ├── Fault Tolerance (error recovery)
│   └── Recoverability (disaster recovery)
├── Security (Confidentiality, Integrity, Availability)
├── Maintainability
│   ├── Modularity
│   ├── Reusability
│   └── Testability
├── Usability
│   ├── Learnability
│   ├── Accessibility
│   └── User Error Protection
└── Portability
    ├── Adaptability
    └── Installability
```

---

## 2. Performance Requirements

### 2.1 Response Time (Latency)

### REQ-PERF-001 [CRITICAL]
**API Response Time**
- TVM API SHALL respond within **500ms** for 95% of requests (p95)
- TVM API SHALL respond within **1000ms** for 99% of requests (p99)
- Measurement: End-to-end latency (client request → server response)

**Rationale:** Operators need responsive UI for mission management

### REQ-PERF-002 [CRITICAL]
**Database Query Performance**
- Database queries SHALL complete within **100ms** for 95% of queries
- Slow query threshold: >1 second (logged, investigated)

**Rationale:** Fast queries ensure responsive API

### REQ-PERF-003 [HIGH]
**Mission Creation Latency**
- Creating new mission SHALL complete within **2 seconds** (p95)
- Includes: Input validation, database insert, vehicle assignment

**Rationale:** Nurses create missions frequently, must be fast

### REQ-PERF-004 [HIGH]
**Vehicle Command Latency**
- Vehicle commands (e.g., abort mission, e-stop) SHALL be delivered within **1 second** (p95)
- Critical safety commands (e-stop) SHALL be delivered within **500ms** (p99)

**Rationale:** Safety-critical, must be near-instantaneous

### REQ-PERF-005 [MEDIUM]
**Dashboard Load Time**
- Dashboard SHALL load initial view within **3 seconds** on broadband connection (>5 Mbps)
- Dashboard SHALL load within **5 seconds** on slower connections (1-5 Mbps)

**Rationale:** User experience, operator efficiency

### REQ-PERF-006 [HIGH]
**Telemetry Update Frequency**
- Vehicle telemetry SHALL update at **1 Hz minimum** (once per second)
- Critical telemetry (battery, location, error codes) SHALL update at **10 Hz** (10 times/second)

**Rationale:** Real-time monitoring for operators

### 2.2 Throughput

### REQ-PERF-007 [HIGH]
**API Request Throughput**
- TVM Server SHALL handle **1000 requests/second** sustained load
- TVM Server SHALL handle **2000 requests/second** peak load (burst, 5 minutes)

**Rationale:** Support 50 vehicles × 10 req/s + 10 operators × 10 req/s = 600 req/s nominal, 2x headroom

### REQ-PERF-008 [MEDIUM]
**Mission Throughput**
- System SHALL support creating **500 missions/hour** sustained
- System SHALL support **100 missions/hour per operator** (10 operators = 1000 missions/hour peak)

**Rationale:** Peak usage during shift changes, emergencies

### REQ-PERF-009 [HIGH]
**Database Throughput**
- Database SHALL handle **10,000 read queries/second**
- Database SHALL handle **1,000 write queries/second**

**Rationale:** High read:write ratio (10:1), typical for fleet management

### 2.3 Resource Utilization

### REQ-PERF-010 [HIGH]
**TVM Server CPU Usage**
- TVM Server SHALL maintain CPU usage <70% under nominal load
- TVM Server SHALL maintain CPU usage <90% under peak load
- Auto-scaling SHALL trigger at 70% CPU

**Rationale:** Headroom for bursts, prevent performance degradation

### REQ-PERF-011 [HIGH]
**TVM Server Memory Usage**
- TVM Server SHALL maintain memory usage <80% of allocated RAM
- Memory leaks SHALL be detected and fixed (memory growth >10%/day)

**Rationale:** Prevent out-of-memory crashes

### REQ-PERF-012 [CRITICAL]
**Vehicle Compute Unit Resource Usage**
- ROS 2 navigation stack SHALL use ≤6 CPU cores (of 8 available, AMD Ryzen 7 7840HS)
- ROS 2 navigation stack SHALL use ≤16 GB RAM (of 32 GB available)
- Reserve 2 CPU cores and 16 GB RAM for safety-critical processes

**Rationale:** Reserve resources for safety PLC, watchdog, emergency systems

### REQ-PERF-013 [HIGH]
**Network Bandwidth**
- Vehicle-TVM communication SHALL use ≤1 Mbps per vehicle average
- Peak bandwidth (e.g., map download): ≤10 Mbps per vehicle
- Total fleet bandwidth: 50 vehicles × 1 Mbps = 50 Mbps nominal

**Rationale:** WiFi capacity planning (802.11ac: ~100 Mbps usable per AP)

### 2.4 Storage Performance

### REQ-PERF-014 [HIGH]
**Database Storage I/O**
- Database SHALL achieve **10,000 IOPS** (Input/Output Operations Per Second) sustained
- Database SHALL achieve **3000 IOPS write**, **7000 IOPS read** (read-heavy workload)

**Rationale:** RDS gp3 SSD provides 3000 IOPS baseline, scalable to 16,000 IOPS

### REQ-PERF-015 [MEDIUM]
**Vehicle SSD Performance**
- Vehicle SSD SHALL achieve **500 MB/s** sequential read (for map loading)
- Vehicle SSD SHALL achieve **200 MB/s** sequential write (for log recording)

**Rationale:** NVMe SSD typical performance, sufficient for ROS bags

---

## 3. Scalability Requirements

### 3.1 Horizontal Scalability

### REQ-SCALE-001 [HIGH]
**TVM Server Horizontal Scaling**
- System SHALL support scaling from 1 to 10 TVM server instances without code changes
- Load balancer SHALL distribute traffic evenly across instances (round-robin or least-connections)
- New instances SHALL auto-register with load balancer within 60 seconds

**Rationale:** Handle load growth, high availability

### REQ-SCALE-002 [MEDIUM]
**Database Read Replica Scaling**
- System SHALL support adding read replicas for read-heavy queries
- Application SHALL route read queries to replicas, write queries to primary

**Rationale:** Offload read traffic from primary database

### 3.2 Vertical Scalability

### REQ-SCALE-003 [HIGH]
**TVM Server Vertical Scaling**
- System SHALL support increasing instance size (e.g., t3.medium → t3.large) with <10 minutes downtime
- Performance SHALL scale linearly with instance size (2x CPU → 2x throughput)

**Rationale:** Simple scaling for early growth

### REQ-SCALE-004 [MEDIUM]
**Database Vertical Scaling**
- Database SHALL support increasing instance class (e.g., db.t3.large → db.m5.xlarge) with <10 minutes downtime
- Performance SHALL scale linearly with instance size

**Rationale:** AWS RDS supports vertical scaling with restart

### 3.3 Data Scalability

### REQ-SCALE-005 [HIGH]
**Database Size Growth**
- Database SHALL support storing **10 TB** of data without performance degradation
- Partitioning strategy SHALL be in place for high-volume tables (telemetry, audit_logs)

**Rationale:** Telemetry data grows rapidly (2.88M rows/day × 90 days = 259M rows)

### REQ-SCALE-006 [MEDIUM]
**S3 Storage Scalability**
- System SHALL support storing **100 TB** of backups, logs, and telemetry archives in S3
- S3 lifecycle policies SHALL automatically tier data (Standard → Glacier)

**Rationale:** S3 effectively unlimited, cost management via lifecycle policies

### 3.4 Fleet Scalability

### REQ-SCALE-007 [CRITICAL]
**Vehicle Fleet Size**
- System SHALL support **50 vehicles** in single facility (Phase 1 target)
- System SHALL support **500 vehicles** across multiple facilities (Phase 2 target, 1-2 years)
- System SHALL support **5,000 vehicles** globally (Phase 3 target, 3-5 years)

**Rationale:** Business growth projections

### REQ-SCALE-008 [HIGH]
**Concurrent Missions**
- System SHALL support **50 concurrent active missions** (one per vehicle)
- System SHALL support **500 concurrent missions** in queue (pending, scheduled)

**Rationale:** Peak usage during high-demand periods

### REQ-SCALE-009 [MEDIUM]
**User Scalability**
- System SHALL support **100 concurrent users** (operators, nurses, caregivers) per facility
- System SHALL support **1,000 concurrent users** across all facilities

**Rationale:** Large facilities (e.g., hospital campus, university)

---

## 4. Availability and Reliability

### 4.1 Availability Targets

### REQ-AVAIL-001 [CRITICAL]
**TVM Server Availability**
- TVM Server SHALL maintain **99.5% availability** (measured monthly)
- Downtime budget: 0.5% × 30 days = 3.6 hours/month = ~50 minutes/week
- Excludes planned maintenance windows (announced 48 hours in advance)

**Rationale:** Fleet operations depend on TVM availability

### REQ-AVAIL-002 [HIGH]
**Database Availability**
- Database SHALL maintain **99.9% availability** (measured monthly)
- Downtime budget: 43 minutes/month
- Automatic failover to standby within 5 minutes (Multi-AZ RDS)

**Rationale:** Database is single point of failure, must be highly available

### REQ-AVAIL-003 [HIGH]
**Vehicle Availability**
- Each vehicle SHALL maintain **95% availability** (measured monthly)
- Downtime includes: Charging (2 hours/day), maintenance (scheduled)
- Excludes: Unscheduled downtime (hardware failure, accidents)

**Rationale:** Some downtime acceptable per vehicle, but fleet should have spare capacity

### REQ-AVAIL-004 [MEDIUM]
**API Gateway Availability**
- API Gateway SHALL maintain **99.95% availability**
- Use managed service (AWS ALB: 99.99% SLA)

**Rationale:** API gateway is critical path, use highly available service

### 4.2 Fault Tolerance

### REQ-AVAIL-005 [CRITICAL]
**Single Point of Failure Elimination**
- System SHALL NOT have any single point of failure (SPOF) for critical components
- Critical components: TVM Server, Database, Load Balancer
- All critical components SHALL be redundant (multi-AZ, multiple instances)

**Rationale:** Resilience to component failures

### REQ-AVAIL-006 [CRITICAL]
**Automatic Failover**
- Database SHALL automatically failover to standby within **5 minutes** on primary failure
- TVM Server SHALL automatically replace failed instances within **3 minutes** (Auto Scaling Group)

**Rationale:** Minimize downtime from failures

### REQ-AVAIL-007 [HIGH]
**Graceful Degradation**
- On partial failure, system SHALL degrade gracefully (maintain partial functionality)
- Example: If database slow, return cached data with warning: "Data may be stale"
- Example: If vehicle offline, continue serving other vehicles

**Rationale:** Better to have partial service than complete outage

### REQ-AVAIL-008 [HIGH]
**Circuit Breaker Pattern**
- System SHALL implement circuit breaker for external dependencies (GPS service, weather API)
- Circuit opens after 5 consecutive failures (stop calling failing service)
- Circuit half-opens after 30 seconds (test recovery)

**Rationale:** Prevent cascading failures

### 4.3 Recoverability

### REQ-AVAIL-009 [CRITICAL]
**Recovery Time Objective (RTO)**
- TVM Server RTO: **4 hours** (time to restore service after disaster)
- Database RTO: **1 hour** (time to restore database from backup)
- Vehicle RTO: **8 hours** (time to repair/replace failed vehicle)

**Rationale:** See DISASTER_RECOVERY_PLAN.md

### REQ-AVAIL-010 [CRITICAL]
**Recovery Point Objective (RPO)**
- Database RPO: **1 hour** (max acceptable data loss)
- Implementation: Hourly backups + streaming replication (<1 min lag)

**Rationale:** Acceptable to lose up to 1 hour of missions in disaster

### REQ-AVAIL-011 [HIGH]
**Backup and Restore**
- Database backups SHALL be taken **daily** (automated)
- Backups SHALL be restored and tested **monthly** (DR drill)
- Backup restoration SHALL complete within **1 hour** (for 100 GB database)

**Rationale:** Verify backups are valid, practice DR procedures

---

## 5. Maintainability Requirements

### 5.1 Modularity

### REQ-MAINT-001 [HIGH]
**Modular Architecture**
- System SHALL be designed with loosely coupled modules
- Vehicle software: Separate ROS 2 packages (nav_control, nav_docking, swerve_drive_controller)
- TVM Server: Separate services (mission_service, vehicle_service, user_service)

**Rationale:** Enable independent development, testing, deployment

### REQ-MAINT-002 [MEDIUM]
**API Versioning**
- TVM API SHALL support versioning (e.g., `/api/v1/missions`, `/api/v2/missions`)
- Breaking changes SHALL increment major version
- Old API versions SHALL be maintained for 12 months after deprecation

**Rationale:** Allow clients to migrate gradually

### 5.2 Code Quality

### REQ-MAINT-003 [HIGH]
**Code Coverage**
- Unit test coverage SHALL be ≥**80%** for all new code
- Critical paths (mission creation, navigation, docking) SHALL have ≥**90%** coverage

**Rationale:** High test coverage catches bugs early

### REQ-MAINT-004 [HIGH]
**Static Analysis**
- Code SHALL pass static analysis (ESLint, Pylint, clang-tidy) with zero errors
- Warnings SHALL be addressed or explicitly suppressed with justification

**Rationale:** Catch common bugs, enforce code style

### REQ-MAINT-005 [MEDIUM]
**Code Review**
- All code changes SHALL be peer-reviewed before merge
- Pull requests SHALL require ≥1 approval from team member
- Critical changes (security, safety) SHALL require ≥2 approvals

**Rationale:** Catch bugs, share knowledge

### 5.3 Documentation

### REQ-MAINT-006 [HIGH]
**API Documentation**
- All API endpoints SHALL be documented using OpenAPI (Swagger) specification
- API documentation SHALL be auto-generated from code (e.g., using decorators)
- API documentation SHALL be accessible at `/api-docs`

**Rationale:** Enable frontend developers to integrate with API

### REQ-MAINT-007 [MEDIUM]
**Code Documentation**
- Complex algorithms SHALL have inline comments explaining logic
- All public functions SHALL have docstrings (JSDoc, Google-style Python docstrings)
- Architecture decisions SHALL be documented in ADR (Architecture Decision Records)

**Rationale:** Enable new developers to understand codebase

### 5.4 Deployment and Rollback

### REQ-MAINT-008 [CRITICAL]
**Zero-Downtime Deployment**
- TVM Server deployments SHALL have **zero downtime** (rolling update)
- During deployment, both old and new versions SHALL run simultaneously (blue-green or canary)

**Rationale:** Avoid service interruption during deployments

### REQ-MAINT-009 [CRITICAL]
**Rapid Rollback**
- Failed deployments SHALL be rolled back within **10 minutes**
- Rollback SHALL be automated (one-command or automatic on health check failure)

**Rationale:** Minimize impact of bad deployments

---

## 6. Usability Requirements

### 6.1 Learnability

### REQ-USE-001 [HIGH]
**Operator Training Time**
- New operators SHALL be proficient with TVM Dashboard within **4 hours** of training
- Dashboard SHALL include interactive tutorial (first-time user onboarding)

**Rationale:** Reduce training cost, enable rapid onboarding

### REQ-USE-002 [MEDIUM]
**Intuitive UI**
- Dashboard UI SHALL follow established design patterns (Material Design, Bootstrap)
- Critical actions (e.g., e-stop) SHALL be prominent (large red button)
- Similar actions SHALL be grouped together (navigation menu)

**Rationale:** Reduce cognitive load, prevent user errors

### 6.2 Accessibility

### REQ-USE-003 [HIGH]
**WCAG 2.1 Compliance**
- Dashboard SHALL comply with **WCAG 2.1 Level AA** (Web Content Accessibility Guidelines)
- All interactive elements SHALL be keyboard-accessible (no mouse required)
- Color contrast ratio SHALL be ≥4.5:1 (normal text), ≥3:1 (large text)

**Rationale:** Legal requirement (ADA, Section 508), inclusive design

### REQ-USE-004 [MEDIUM]
**Screen Reader Support**
- Dashboard SHALL be usable with screen readers (JAWS, NVDA, VoiceOver)
- All images SHALL have alt text
- Form fields SHALL have labels (not just placeholders)

**Rationale:** Accessibility for visually impaired users

### 6.3 User Error Protection

### REQ-USE-005 [CRITICAL]
**Destructive Action Confirmation**
- Destructive actions (e.g., delete resident, cancel mission) SHALL require confirmation
- Confirmation dialog SHALL clearly state consequences: "Are you sure? This cannot be undone."

**Rationale:** Prevent accidental data loss

### REQ-USE-006 [HIGH]
**Input Validation with Clear Errors**
- All form inputs SHALL be validated (client-side and server-side)
- Validation errors SHALL be specific: "Pickup location must be within facility bounds" (not "Invalid input")

**Rationale:** Help user fix errors quickly

### REQ-USE-007 [MEDIUM]
**Undo Functionality**
- Recent actions (e.g., create mission) SHALL be undoable within **5 minutes**
- Undo stack SHALL retain last 10 actions

**Rationale:** Allow users to fix mistakes

### 6.4 Responsiveness (UI)

### REQ-USE-008 [HIGH]
**Mobile-Friendly UI**
- Dashboard SHALL be usable on tablets (iPad, Android tablets)
- Responsive design: Layout adapts to screen size (≥768px width)
- Critical functions SHALL be accessible on mobile (e.g., e-stop, mission status)

**Rationale:** Operators may use tablets in facility

### REQ-USE-009 [MEDIUM]
**Offline Capability (Progressive Web App)**
- Dashboard SHALL function with limited offline capability (view cached data)
- When online, dashboard SHALL sync changes

**Rationale:** Handle temporary network outages

---

## 7. Security Requirements

*(See SECURITY_REQUIREMENTS.md for complete security requirements)*

**Cross-Reference:**
- REQ-SEC-001 to REQ-SEC-152: 152 security requirements
- Authentication, authorization, encryption, audit logging, incident response

**Summary:**
- **Confidentiality:** PII encrypted at rest (AES-256), in transit (TLS 1.3)
- **Integrity:** Input validation, digital signatures, audit logs
- **Availability:** DDoS protection, rate limiting, redundancy

---

## 8. Privacy Requirements

*(See PRIVACY_REQUIREMENTS.md for complete privacy requirements)*

**Cross-Reference:**
- REQ-PRIV-001 to REQ-PRIV-100: 100 privacy requirements
- GDPR, PIPEDA, HIPAA compliance

**Summary:**
- **Data Minimization:** Collect only necessary PII
- **Consent Management:** Explicit consent for medical data
- **Data Subject Rights:** Access, rectification, erasure, portability
- **Retention:** Data deleted after retention period (3 years for residents)

---

## 9. Compatibility and Interoperability

### 9.1 Browser Compatibility

### REQ-COMPAT-001 [HIGH]
**Supported Browsers**
- Dashboard SHALL support latest 2 versions of:
  - Google Chrome (Windows, macOS, Linux)
  - Mozilla Firefox (Windows, macOS, Linux)
  - Safari (macOS, iOS)
  - Microsoft Edge (Windows)
- Dashboard SHALL gracefully degrade on older browsers (show warning)

**Rationale:** Cover 99% of users

### 9.2 Operating System Compatibility

### REQ-COMPAT-002 [CRITICAL]
**Vehicle OS**
- Vehicle software SHALL run on **Ubuntu 22.04 LTS** (ROS 2 Humble)
- Vehicle software SHALL NOT depend on specific kernel version (use generic kernel)

**Rationale:** Standardized OS for all vehicles, LTS support until 2027

### REQ-COMPAT-003 [HIGH]
**TVM Server OS**
- TVM Server SHALL run on **Ubuntu 22.04 LTS** or **Amazon Linux 2**
- TVM Server SHALL be containerized (Docker), OS-agnostic

**Rationale:** Cloud deployment flexibility

### 9.3 API Interoperability

### REQ-COMPAT-004 [HIGH]
**REST API Standards**
- TVM API SHALL follow RESTful design principles
- API SHALL use standard HTTP methods (GET, POST, PUT, DELETE)
- API SHALL return standard HTTP status codes (200, 400, 404, 500)

**Rationale:** Enable third-party integrations

### REQ-COMPAT-005 [MEDIUM]
**API Backward Compatibility**
- API changes SHALL maintain backward compatibility within major version
- Breaking changes SHALL increment major version (v1 → v2)
- Old API versions SHALL be supported for 12 months after deprecation

**Rationale:** Don't break existing clients

---

## 10. Portability Requirements

### 10.1 Cloud Provider Portability

### REQ-PORT-001 [MEDIUM]
**Multi-Cloud Capability**
- System SHALL be designed to support multiple cloud providers (AWS, Azure, GCP)
- Infrastructure SHALL be defined using cloud-agnostic tools (Terraform, Kubernetes)
- Application SHALL NOT depend on AWS-specific services (where feasible)

**Rationale:** Avoid vendor lock-in, enable multi-region deployment

**Note:** Phase 1 (MVP) targets AWS only, multi-cloud support in Phase 3 (1-2 years)

### 10.2 Database Portability

### REQ-PORT-002 [MEDIUM]
**Database Abstraction**
- Application SHALL use ORM (Object-Relational Mapping) for database access (e.g., TypeORM, SQLAlchemy)
- Application SHALL NOT use database-specific SQL features (where feasible)

**Rationale:** Enable migration to different PostgreSQL-compatible databases (e.g., CockroachDB)

### 10.3 Deployment Portability

### REQ-PORT-003 [HIGH]
**Containerization**
- TVM Server SHALL be containerized (Docker)
- Deployment SHALL support multiple orchestration platforms (ECS, EKS, standalone Docker)

**Rationale:** Flexibility in deployment strategy

---

## 11. Testability Requirements

### 11.1 Unit Testing

### REQ-TEST-001 [HIGH]
**Unit Test Coverage**
- All business logic SHALL have unit tests (80% coverage target)
- Unit tests SHALL run in <1 minute (for fast feedback)

**Rationale:** Fast, isolated tests catch bugs early

### 11.2 Integration Testing

### REQ-TEST-002 [HIGH]
**Integration Test Coverage**
- All API endpoints SHALL have integration tests
- Integration tests SHALL use mocked external dependencies (database, vehicle)

**Rationale:** Verify component interactions

### 11.3 End-to-End Testing

### REQ-TEST-003 [MEDIUM]
**E2E Test Coverage**
- Critical user journeys SHALL have E2E tests:
  - Create mission → Assign vehicle → Complete mission
  - User login → Create resident → Request transport
- E2E tests SHALL run nightly (not on every commit, too slow)

**Rationale:** Verify system works end-to-end

### 11.4 Performance Testing

### REQ-TEST-004 [HIGH]
**Load Testing**
- System SHALL be load-tested before production deployment
- Load test SHALL simulate 1000 req/s sustained, 2000 req/s peak
- Load test SHALL identify bottlenecks (slow queries, CPU saturation)

**Rationale:** Ensure system meets performance requirements under load

### 11.5 Security Testing

### REQ-TEST-005 [HIGH]
**Vulnerability Scanning**
- Docker images SHALL be scanned for vulnerabilities (CVE) before deployment
- Critical vulnerabilities SHALL block deployment (fail CI/CD pipeline)

**Rationale:** Prevent deploying known vulnerabilities

### REQ-TEST-006 [MEDIUM]
**Penetration Testing**
- System SHALL undergo penetration testing **annually** by third-party security firm
- Findings SHALL be remediated within 90 days

**Rationale:** Identify security weaknesses

---

## 12. Compliance Requirements

*(See REGULATORY_COMPLIANCE_REQUIREMENTS.md for complete compliance requirements)*

### 12.1 Safety Standards

### REQ-COMP-001 [CRITICAL]
**ISO 13849 (Safety of Machinery)**
- Vehicle safety functions SHALL comply with **ISO 13849 Category 3, SIL 2**
- Safety functions: E-stop, obstacle detection, speed limiting

**Rationale:** Legal requirement for safety-critical systems

### 12.2 Cybersecurity Standards

### REQ-COMP-002 [HIGH]
**IEC 62443 (Industrial Cybersecurity)**
- Vehicle and TVM systems SHALL comply with **IEC 62443-4-2** (component security)
- Security Level: **SL 2** (Protection against intentional violation using simple means)

**Rationale:** Industry standard for industrial control systems

### 12.3 Data Protection Regulations

### REQ-COMP-003 [CRITICAL]
**GDPR Compliance (EU)**
- If operating in EU, system SHALL comply with **GDPR** (General Data Protection Regulation)
- Data subject rights: Access, rectification, erasure, portability

**Rationale:** Legal requirement, €20M fine for non-compliance

### REQ-COMP-004 [HIGH]
**PIPEDA Compliance (Canada)**
- If operating in Canada, system SHALL comply with **PIPEDA**
- Obtain consent before collecting personal information

**Rationale:** Legal requirement in Canada

### REQ-COMP-005 [HIGH]
**HIPAA Compliance (US, if handling PHI)**
- If handling Protected Health Information (PHI) in US, system SHALL comply with **HIPAA**
- Sign Business Associate Agreement (BAA) with healthcare providers

**Rationale:** Legal requirement for healthcare data in US

### 12.4 Accessibility Standards

### REQ-COMP-006 [HIGH]
**ADA / Section 508 Compliance (US)**
- Dashboard SHALL comply with **Section 508** (US federal accessibility standard)
- Equivalent to WCAG 2.0 Level AA

**Rationale:** Legal requirement for US government contracts

---

## 13. Environmental Requirements

### 13.1 Vehicle Operating Environment

### REQ-ENV-001 [CRITICAL]
**Outdoor Operating Temperature**
- Vehicle SHALL operate in temperature range: **-10°C to 40°C** (14°F to 104°F)
- Vehicle SHALL survive (non-operating) in temperature range: **-20°C to 50°C** (-4°F to 122°F)

**Rationale:** Outdoor operation in temperate climates (not Arctic/desert extremes)

### REQ-ENV-002 [CRITICAL]
**Weather Resistance (IP Rating)**
- Vehicle enclosure SHALL be rated **IP54** minimum (dust-protected, splash-resistant)
- Sensors (LiDAR, cameras) SHALL be rated **IP65** (dust-tight, water jet resistant)

**Rationale:** Operate in light rain, withstand dust

### REQ-ENV-003 [HIGH]
**Slope Capability**
- Vehicle SHALL navigate slopes up to **10 degrees** (17.6% grade)
- Vehicle SHALL navigate slopes up to **5 degrees** with passenger (safety factor)

**Rationale:** Outdoor terrain is not flat, need slope capability

### REQ-ENV-004 [MEDIUM]
**Surface Compatibility**
- Vehicle SHALL navigate: Asphalt, concrete, paved paths, compacted gravel
- Vehicle SHALL NOT navigate: Loose gravel, mud, sand, grass (outdoor-first, not all-terrain)

**Rationale:** Focus on paved outdoor environments (campuses, facilities)

### 13.2 Electromagnetic Compatibility (EMC)

### REQ-ENV-005 [HIGH]
**EMC Compliance**
- Vehicle SHALL comply with **FCC Part 15 Class B** (radiated emissions limit)
- Vehicle SHALL comply with **EN 55011** (EMC standard for industrial equipment)

**Rationale:** Prevent interference with other devices (WiFi, medical equipment)

### 13.3 Data Center Environment (TVM Server)

### REQ-ENV-006 [MEDIUM]
**Data Center Temperature**
- TVM Server (if on-premise) SHALL operate in data center temperature: **18°C to 27°C** (64°F to 81°F)
- Cloud deployment (AWS): AWS data center handles environmental conditions

**Rationale:** Typical data center ASHRAE standards

---

## 14. Operational Requirements

### 14.1 Monitoring and Alerting

### REQ-OPS-001 [CRITICAL]
**System Monitoring**
- All critical components SHALL be monitored 24/7 (Prometheus + Grafana)
- Alerts SHALL be sent to on-call engineer within **1 minute** of incident detection

**Rationale:** Proactive problem detection, rapid response

### REQ-OPS-002 [HIGH]
**SLO Tracking**
- System SHALL track SLOs (Service Level Objectives) in real-time dashboard
- SLO breaches SHALL trigger alerts (e.g., error rate >1%)

**Rationale:** Ensure service quality

### 14.2 Backup and Disaster Recovery

### REQ-OPS-003 [CRITICAL]
**Automated Backups**
- Database SHALL be backed up **daily** (automated, 7-day retention)
- Backups SHALL be tested **monthly** (restore drill)

**Rationale:** Prevent data loss, verify backups work

### REQ-OPS-004 [CRITICAL]
**Disaster Recovery Plan**
- System SHALL have documented disaster recovery plan (RTO: 4 hours, RPO: 1 hour)
- DR plan SHALL be tested **quarterly** (failover drill)

**Rationale:** See DISASTER_RECOVERY_PLAN.md

### 14.3 Capacity Planning

### REQ-OPS-005 [HIGH]
**Resource Forecasting**
- Operations team SHALL review resource usage **monthly**
- Forecast: When will current capacity be exhausted? (e.g., "Database will be full in 6 months")

**Rationale:** Proactive scaling, avoid capacity crises

### 14.4 Change Management

### REQ-OPS-006 [HIGH]
**Change Approval Process**
- Production changes SHALL be approved by change advisory board (CAB) for high-risk changes
- Emergency changes (P0 incidents) MAY bypass CAB (post-incident review required)

**Rationale:** Prevent unauthorized changes, reduce risk

### 14.5 On-Call Support

### REQ-OPS-007 [CRITICAL]
**24/7 On-Call Coverage**
- System SHALL have **24/7 on-call engineer** (primary + backup)
- On-call SHALL respond to pages within **15 minutes**

**Rationale:** Rapid response to critical incidents

---

## Summary

**Total Requirements:** 70 (CRITICAL: 25, HIGH: 37, MEDIUM: 8)

### Requirements by Category

| Category | Count | Critical | High | Medium |
|----------|-------|----------|------|--------|
| **Performance** | 15 | 4 | 8 | 3 |
| **Scalability** | 9 | 1 | 5 | 3 |
| **Availability & Reliability** | 11 | 5 | 5 | 1 |
| **Maintainability** | 9 | 1 | 5 | 3 |
| **Usability** | 9 | 1 | 4 | 4 |
| **Security** | (See SECURITY_REQUIREMENTS.md: 152 req) | - | - | - |
| **Privacy** | (See PRIVACY_REQUIREMENTS.md: 100 req) | - | - | - |
| **Compatibility** | 5 | 1 | 3 | 1 |
| **Portability** | 3 | 0 | 1 | 2 |
| **Testability** | 6 | 0 | 3 | 3 |
| **Compliance** | 6 | 2 | 4 | 0 |
| **Environmental** | 6 | 2 | 2 | 2 |
| **Operational** | 7 | 4 | 3 | 0 |

**Combined Total (including Security + Privacy):** **322 non-functional requirements**

---

## Approval

| Role | Name | Signature | Date |
|------|------|-----------|------|
| **Project Lead** | Maeda-san | __________ | _______ |
| **Vehicle Software Lead** | Pankaj | __________ | _______ |
| **TVM Lead** | Unno-san | __________ | _______ |

---

**Document Metadata:**
- **Version:** 2.0 (Expanded from v1.0)
- **Created:** 2025-12-17
- **Next Review:** 2026-03-17 (quarterly review)
- **Owner:** Integration Team (All Teams)

---

**Related Documents:**
- `SECURITY_REQUIREMENTS.md` - 152 security requirements
- `PRIVACY_REQUIREMENTS.md` - 100 privacy requirements
- `ERROR_HANDLING_REQUIREMENTS.md` - 91 error handling requirements
- `REGULATORY_COMPLIANCE_REQUIREMENTS.md` - Detailed compliance mapping
- `OBSERVABILITY_ARCHITECTURE.md` - Monitoring and alerting implementation
- `DISASTER_RECOVERY_PLAN.md` - DR procedures and RTO/RPO

---

*End of Document*
