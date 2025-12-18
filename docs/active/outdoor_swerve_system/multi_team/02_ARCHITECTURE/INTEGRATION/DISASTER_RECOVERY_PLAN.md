# Disaster Recovery Plan

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Document Type:** Operational Plan
**Status:** Active
**Version:** 1.0
**Last Updated:** 2025-12-17
**Owner:** Integration Team (All Teams)

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [DR Objectives and Targets](#2-dr-objectives-and-targets)
3. [Disaster Scenarios](#3-disaster-scenarios)
4. [Backup Architecture](#4-backup-architecture)
5. [Recovery Procedures](#5-recovery-procedures)
6. [Failover Architecture](#6-failover-architecture)
7. [Vehicle Recovery](#7-vehicle-recovery)
8. [Database Recovery](#8-database-recovery)
9. [TVM Server Recovery](#9-tvm-server-recovery)
10. [Network and Infrastructure Recovery](#10-network-and-infrastructure-recovery)
11. [DR Testing and Validation](#11-dr-testing-and-validation)
12. [Roles and Responsibilities](#12-roles-and-responsibilities)
13. [Communication Plan](#13-communication-plan)
14. [Business Continuity](#14-business-continuity)

---

## 1. Executive Summary

### 1.1 Purpose

This Disaster Recovery (DR) Plan ensures business continuity for the outdoor wheelchair transport robot fleet system in the event of catastrophic failure. The plan covers:
- **Technology disasters:** Data center outage, ransomware, hardware failure
- **Natural disasters:** Fire, flood, earthquake
- **Human-caused disasters:** Cyberattack, accidental deletion, sabotage

### 1.2 Scope

**In Scope:**
- TVM Server (fleet management backend)
- Database (PostgreSQL)
- Vehicle software (ROS 2 systems)
- Network infrastructure
- Data backups

**Out of Scope:**
- Vehicle hardware replacement (covered by hardware team's warranty/maintenance plan)
- Facility/building recovery (facility owner's responsibility)
- Employee safety (covered by facility emergency response plan)

### 1.3 DR Strategy

**Multi-Layer Protection:**
1. **Redundancy:** Hot standby database, load-balanced TVM servers
2. **Backups:** Daily automated backups, offsite storage
3. **Failover:** Automatic failover to secondary region (within 15 minutes)
4. **Rapid Recovery:** Pre-configured DR environment (infrastructure-as-code)

---

## 2. DR Objectives and Targets

### 2.1 Recovery Time Objective (RTO)

**RTO:** Maximum acceptable downtime

| System | RTO Target | Justification |
|--------|------------|---------------|
| **TVM Server (Critical)** | **4 hours** | Fleet cannot operate without TVM coordination |
| **Database** | **1 hour** | Data loss acceptable if <1 hour old (see RPO) |
| **Vehicle Software** | **8 hours** | Can operate degraded (manual mode) temporarily |
| **Dashboard (Web UI)** | **2 hours** | Operators can use backup manual processes |
| **Monitoring/Logging** | **24 hours** | Non-critical, can recover after core systems |

**Prioritization:** TVM Server + Database first, then Dashboard, then Monitoring.

### 2.2 Recovery Point Objective (RPO)

**RPO:** Maximum acceptable data loss

| Data Type | RPO Target | Backup Frequency | Justification |
|-----------|------------|------------------|---------------|
| **Database (Critical)** | **1 hour** | Streaming replication (real-time) + hourly snapshots | Missions created in last hour may be lost (acceptable) |
| **Vehicle Logs** | **24 hours** | Daily upload to TVM | Telemetry loss acceptable if vehicle can re-upload |
| **Configuration Files** | **0 minutes** | Version control (Git) | Infrastructure-as-code, no data loss |
| **Application Code** | **0 minutes** | Version control (Git) | No data loss |

**Result:** Database replication ensures <1 hour RPO. If primary fails, standby has data up to last committed transaction.

### 2.3 Maximum Tolerable Downtime (MTD)

**MTD:** Maximum downtime before business impact becomes unacceptable

- **Fleet Operations:** **12 hours** (after this, residents miss critical appointments, facility reputation damaged)
- **Individual Vehicle:** **4 hours** (fleet can operate with reduced capacity)

**Implication:** RTO of 4 hours (TVM) is well within MTD of 12 hours.

---

## 3. Disaster Scenarios

### 3.1 Scenario Matrix

| Scenario | Likelihood | Impact | RTO | Recovery Complexity |
|----------|------------|--------|-----|---------------------|
| **Single Server Failure** | High (monthly) | Low | 15 min | Low (auto-scaling) |
| **Database Corruption** | Medium (yearly) | Medium | 1 hour | Medium (restore from backup) |
| **Data Center Outage** | Low (every 5 years) | High | 4 hours | High (regional failover) |
| **Ransomware Attack** | Medium (yearly) | High | 8 hours | High (restore from offsite backup) |
| **Accidental Deletion** | High (quarterly) | Low | 30 min | Low (restore specific tables) |
| **Network Outage** | Medium (yearly) | Medium | 2 hours | Medium (reroute to backup ISP) |
| **Vehicle Hardware Failure** | High (weekly) | Low | 8 hours | Medium (swap vehicle) |
| **Natural Disaster** | Very Low (every 50 years) | Critical | 12 hours | Critical (rebuild entire system) |

**Focus Areas:**
- **High Likelihood + High Impact:** Ransomware (most critical to prepare for)
- **Low Likelihood + Critical Impact:** Natural disaster (worst-case scenario)

### 3.2 Scenario Details

#### Scenario 1: Single Server Failure
**Cause:** Hardware failure, OS crash, kernel panic
**Detection:** Monitoring alert (Prometheus/Grafana): "Server unresponsive"
**Impact:** Partial outage (other servers in load balancer continue serving)
**Recovery:** Auto-scaling replaces failed instance within 15 minutes (AWS Auto Scaling Group)

#### Scenario 2: Database Corruption
**Cause:** Software bug, disk corruption, accidental `DROP TABLE`
**Detection:** Application errors (500 errors), database health check fails
**Impact:** Full TVM outage (cannot read/write data)
**Recovery:** Restore from last backup (hourly snapshots, ~30 min to restore + 30 min validation)

#### Scenario 3: Data Center Outage
**Cause:** Power failure, fire, network outage, AWS region outage
**Detection:** All servers in primary region unreachable (monitoring alert)
**Impact:** Full TVM outage (entire primary region down)
**Recovery:** Failover to secondary region (standby database promoted to primary, DNS updated)

#### Scenario 4: Ransomware Attack
**Cause:** Malware encrypts all data (database, backups, servers)
**Detection:** Files encrypted, ransom note displayed, systems unresponsive
**Impact:** Full TVM outage, potential data loss if recent backups also encrypted
**Recovery:** Restore from offsite backup (S3 Glacier, immutable backups), rebuild servers

#### Scenario 5: Accidental Deletion
**Cause:** Operator runs `DELETE FROM residents` without `WHERE` clause
**Detection:** Missing data (resident count drops to 0)
**Impact:** Data loss (residents profiles deleted)
**Recovery:** Restore specific tables from last backup (point-in-time recovery)

---

## 4. Backup Architecture

### 4.1 Backup Strategy (3-2-1 Rule)

```
┌─────────────────────────────────────────────────────────┐
│              Production Data (Primary Copy)             │
│  - PostgreSQL Database (us-east-1)                      │
│  - TVM Server Files (logs, configs)                     │
│  - Vehicle Data (uploaded logs)                         │
└───────────────────┬─────────────────────────────────────┘
                    │
         ┌──────────┴──────────┐
         ↓                     ↓
┌─────────────────┐   ┌─────────────────┐
│ Copy 1: Standby │   │ Copy 2: Backup  │
│ Database        │   │ (S3)            │
│ (us-west-2)     │   │ (us-east-1)     │
│ - Streaming     │   │ - Daily full    │
│   replication   │   │ - Hourly incr.  │
│ - Hot standby   │   │ - 90 days ret.  │
└─────────────────┘   └────────┬────────┘
                               │
                               ↓
                      ┌─────────────────┐
                      │ Copy 3: Offsite │
                      │ (S3 Glacier)    │
                      │ (eu-west-1)     │
                      │ - Weekly full   │
                      │ - 7 years ret.  │
                      │ - Immutable     │
                      └─────────────────┘
```

**3-2-1 Rule:**
- **3 copies:** Production + Standby + Backup
- **2 media types:** Disk (production, standby) + Cloud (S3)
- **1 offsite:** S3 Glacier in different region (eu-west-1)

### 4.2 Backup Schedule

| Backup Type | Frequency | Retention | Storage | Purpose |
|-------------|-----------|-----------|---------|---------|
| **Database Streaming Replication** | Real-time | N/A (live standby) | us-west-2 (hot standby) | Zero data loss, instant failover |
| **Database Full Backup** | Daily (1 AM UTC) | 90 days | S3 (us-east-1) | Point-in-time recovery |
| **Database Incremental Backup** | Hourly | 7 days | S3 (us-east-1) | Recent changes, faster restore |
| **Database WAL Archives** | Continuous | 90 days | S3 (us-east-1) | Point-in-time recovery (PITR) |
| **Offsite Full Backup** | Weekly (Sunday) | 7 years | S3 Glacier (eu-west-1) | Disaster recovery, compliance |
| **TVM Server Config** | On change | Indefinite | Git (GitHub) | Infrastructure-as-code |
| **Vehicle Logs** | Daily upload | 90 days | S3 (us-east-1) | Diagnostics, audit trail |

### 4.3 Backup Verification

**Automated Verification:**
- **Daily:** Restore last backup to test environment (automated script)
- **Validation:** Run SQL queries to verify data integrity (row counts, checksums)
- **Alert:** If restore fails, alert ops team immediately

**Manual Verification:**
- **Monthly:** Full DR drill (restore entire system from backup, documented below)

### 4.4 Backup Security

**Encryption:**
- **At Rest:** All backups encrypted with AES-256 (AWS S3 SSE)
- **In Transit:** TLS 1.3 for all backup transfers

**Access Control:**
- **S3 Bucket Policy:** Only backup service account can write
- **IAM:** Only DR team can read/restore backups (MFA required)

**Immutability:**
- **S3 Object Lock:** Offsite backups (Glacier) are immutable (cannot be deleted or modified)
- **Retention:** 7 years (compliance requirement)
- **Protection:** Even if AWS credentials compromised, backups cannot be deleted

---

## 5. Recovery Procedures

### 5.1 Recovery Workflow

```
┌─────────────────────────────────────────────────────────┐
│ 1. DETECTION (Monitoring Alert or User Report)         │
│    - Severity assessment (P0/P1/P2/P3)                  │
│    - Incident ticket created (Jira/PagerDuty)           │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 2. TRIAGE (DR Coordinator)                              │
│    - Determine disaster type (server, DB, ransomware)   │
│    - Assess RTO/RPO (can we meet targets?)              │
│    - Activate DR team (page on-call)                    │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 3. DECISION (DR Commander)                              │
│    - Failover or restore? (decision tree below)         │
│    - Communicate to stakeholders (status update)        │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 4. EXECUTION (DR Team)                                  │
│    - Follow runbook (scenario-specific procedures)      │
│    - Parallel tasks (DB restore + server rebuild)       │
│    - Status updates every 30 minutes                    │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 5. VALIDATION (QA + Ops)                                │
│    - Test critical paths (login, create mission, etc.)  │
│    - Verify data integrity (spot checks)                │
│    - Monitor for errors (30 minutes)                    │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 6. ALL CLEAR (DR Commander)                             │
│    - Announce service restored                          │
│    - Deactivate DR team                                 │
│    - Schedule post-mortem (within 48 hours)             │
└─────────────────────────────────────────────────────────┘
```

### 5.2 Decision Tree: Failover vs. Restore

```
                    ┌─────────────────┐
                    │ Disaster Occurs │
                    └────────┬────────┘
                             │
                   ┌─────────┴─────────┐
                   │ Primary Region    │
                   │ Accessible?       │
                   └─────────┬─────────┘
                             │
                    ┌────────┴────────┐
                    │ YES         NO  │
                    ↓                 ↓
          ┌──────────────┐   ┌────────────────┐
          │ Single Server│   │ Regional Outage│
          │ or DB Issue? │   │ or Total Loss? │
          └──────┬───────┘   └────────┬───────┘
                 │                    │
        ┌────────┴────────┐           │
        │ YES         NO  │           │
        ↓                 ↓           ↓
  ┌──────────┐   ┌────────────┐  ┌────────────┐
  │ RESTORE  │   │ Investigate│  │ FAILOVER   │
  │ from     │   │ Root Cause │  │ to         │
  │ Backup   │   │ (unknown)  │  │ Secondary  │
  └──────────┘   └────────────┘  │ Region     │
                                 └────────────┘
```

**Decision Criteria:**
- **Failover (Scenario 3):** If primary region completely unreachable (network, power, AWS outage)
- **Restore (Scenario 2, 4, 5):** If specific component failed (database corruption, ransomware, accidental deletion)

---

## 6. Failover Architecture

### 6.1 Multi-Region Architecture

```
┌─────────────────────────────────────────────────────────┐
│                 PRIMARY REGION (us-east-1)              │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐       │
│  │ TVM Server │  │ PostgreSQL │  │   Redis    │       │
│  │ (Active)   │  │ (Primary)  │  │  (Active)  │       │
│  │ 3 instances│  │ Master     │  │            │       │
│  └────────────┘  └────────────┘  └────────────┘       │
│                        │ Streaming Replication         │
└────────────────────────┼───────────────────────────────┘
                         │
                         ↓
┌─────────────────────────────────────────────────────────┐
│              SECONDARY REGION (us-west-2)               │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐       │
│  │ TVM Server │  │ PostgreSQL │  │   Redis    │       │
│  │ (Standby)  │  │ (Standby)  │  │  (Standby) │       │
│  │ 1 instance │  │ Read-only  │  │            │       │
│  └────────────┘  └────────────┘  └────────────┘       │
│                                                         │
│  - Can serve read-only traffic (monitoring dashboard)  │
│  - Promoted to primary on failover                     │
└─────────────────────────────────────────────────────────┘

                    ┌──────────────┐
                    │  Route 53    │
                    │  (DNS)       │
                    │  Health Check│
                    └──────┬───────┘
                           │
              ┌────────────┴────────────┐
              │ If Primary Unhealthy:   │
              │ Switch to Secondary     │
              └─────────────────────────┘
```

### 6.2 Automatic Failover (Database)

**PostgreSQL Streaming Replication with Automatic Failover:**

```bash
# Primary Database (us-east-1)
postgresql://primary.tvm.internal:5432/tvm
  ↓ Streaming Replication (WAL shipping)
# Standby Database (us-west-2)
postgresql://standby.tvm.internal:5432/tvm (read-only)

# Failover Trigger (automated via Patroni or repmgr):
1. Health check detects primary down (3 consecutive failures, 30 seconds)
2. Standby promoted to primary (automatic)
3. DNS updated: primary.tvm.internal → standby IP
4. Application reconnects (retry logic, 60 seconds)
```

**Tools:**
- **Patroni:** Automatic failover for PostgreSQL (HA solution)
- **PgBouncer:** Connection pooler (reconnects to new primary seamlessly)

**Failover Time:** <5 minutes (detection 30s + promotion 1 min + DNS propagation 3 min)

### 6.3 Manual Failover (TVM Server)

**Procedure: Promote Secondary Region to Primary**

**Prerequisites:**
- Secondary region TVM servers are already running (1 instance, standby mode)
- Database failover already completed (standby DB now primary)

**Steps:**
1. **Verify Database Failover:**
   ```bash
   psql -h standby.tvm.internal -U tvm -c "SELECT pg_is_in_recovery();"
   # Expected: f (false, meaning not in recovery = primary)
   ```

2. **Scale Up TVM Servers in Secondary Region:**
   ```bash
   # AWS Auto Scaling Group
   aws autoscaling set-desired-capacity --auto-scaling-group-name tvm-secondary --desired-capacity 3
   # Wait for instances to start (2-3 minutes)
   ```

3. **Update DNS (Route 53):**
   ```bash
   # Switch api.tvm.example.com from us-east-1 to us-west-2
   aws route53 change-resource-record-sets --hosted-zone-id Z1234 --change-batch '{
     "Changes": [{
       "Action": "UPSERT",
       "ResourceRecordSet": {
         "Name": "api.tvm.example.com",
         "Type": "A",
         "AliasTarget": {
           "HostedZoneId": "Z5678",
           "DNSName": "tvm-secondary-lb-us-west-2.elb.amazonaws.com",
           "EvaluateTargetHealth": true
         }
       }
     }]
   }'
   # TTL: 60 seconds (clients switch within 1 minute)
   ```

4. **Update Vehicle Configuration:**
   - Vehicles connect to `api.tvm.example.com` (DNS automatically resolves to new region)
   - If hardcoded IPs: Push OTA update with new TVM server IP
   - Vehicles reconnect within 5 minutes (retry interval)

5. **Verify Service:**
   ```bash
   # Test login
   curl -X POST https://api.tvm.example.com/auth/login -d '{"email":"test@example.com","password":"test"}'
   # Test create mission
   curl -X POST https://api.tvm.example.com/api/missions -H "Authorization: Bearer $TOKEN" -d '{...}'
   ```

**Total Failover Time:** 10-15 minutes (manual steps)

---

## 7. Vehicle Recovery

### 7.1 Vehicle Software Failure

**Scenario:** Vehicle ROS 2 system crashes, unresponsive

**Recovery Procedure:**

1. **Remote Reboot:**
   ```bash
   # From TVM server, send reboot command to vehicle
   curl -X POST https://api.tvm.example.com/api/vehicles/{vehicle_id}/reboot
   # Vehicle reboots (2 minutes)
   ```

2. **If Reboot Fails, Physical Intervention:**
   - Dispatch technician to vehicle location
   - Power cycle compute unit (unplug battery, wait 30s, plug back in)
   - Manual boot verification (SSH into vehicle)

3. **If Boot Fails, Re-Image:**
   ```bash
   # Flash SD card with clean OS image (Ubuntu 22.04 + ROS 2)
   # Restore vehicle config from TVM (vehicle ID, certs, maps)
   # Total time: 1 hour (30 min re-image + 30 min testing)
   ```

### 7.2 Vehicle Hardware Failure

**Scenario:** LiDAR broken, motor failure, battery dead

**Recovery Procedure:**

1. **Swap Vehicle:**
   - Take failed vehicle out of service (TVM marks as "maintenance mode")
   - Deploy spare vehicle (if available)
   - Spare vehicle takes over remaining missions

2. **Hardware Repair:**
   - Diagnose issue (onsite or ship to hardware team)
   - Replace component (LiDAR, motor, battery)
   - Test in controlled environment before redeployment
   - **RTO:** 8 hours (includes diagnosis, repair, testing)

3. **Preventive Maintenance:**
   - Schedule quarterly maintenance (replace worn components before failure)
   - Maintain 10% spare vehicle fleet (e.g., 10 vehicles + 1 spare)

### 7.3 Lost Vehicle Communication

**Scenario:** Vehicle loses WiFi connection to TVM

**Recovery Procedure:**

1. **Automatic Behavior:**
   - Vehicle enters "offline mode" (continues current mission if safe)
   - Upon mission completion, vehicle parks and waits for reconnection
   - Retry connection every 60 seconds

2. **Manual Recovery:**
   - Dispatch technician to vehicle location
   - Check WiFi: Is AP reachable? Password changed?
   - Check compute unit: Network interface up? Firewall blocking?
   - If network hardware failed: Replace WiFi adapter (USB dongle backup)

**RTO:** 30 minutes (technician dispatch + diagnosis)

---

## 8. Database Recovery

### 8.1 Point-in-Time Recovery (PITR)

**Scenario:** Accidental data deletion at 14:30 UTC, need to restore to 14:25 UTC

**Procedure:**

1. **Stop Application (Prevent Further Writes):**
   ```bash
   # Scale down TVM servers to 0 (prevent new transactions)
   aws autoscaling set-desired-capacity --auto-scaling-group-name tvm-primary --desired-capacity 0
   ```

2. **Restore to PITR Target:**
   ```bash
   # PostgreSQL PITR using WAL archives
   # 1. Restore last full backup (1 AM UTC)
   aws s3 cp s3://tvm-backups/db-full-2025-12-17.dump /tmp/
   pg_restore -d tvm_recovery /tmp/db-full-2025-12-17.dump

   # 2. Replay WAL archives up to 14:25 UTC
   # recovery.conf:
   # restore_command = 'aws s3 cp s3://tvm-backups/wal/%f %p'
   # recovery_target_time = '2025-12-17 14:25:00 UTC'
   pg_ctl -D /var/lib/postgresql/data start
   # Wait for recovery to complete (10-20 minutes depending on WAL size)
   ```

3. **Validate Data:**
   ```bash
   psql -d tvm_recovery -c "SELECT COUNT(*) FROM residents;"
   # Verify resident count matches expected value before deletion
   ```

4. **Promote Recovery Database to Primary:**
   ```bash
   # Stop current primary (corrupted)
   pg_ctl -D /var/lib/postgresql/data stop
   # Start recovery DB as new primary
   pg_ctl promote -D /var/lib/postgresql/data_recovery
   # Update DNS: primary.tvm.internal → recovery DB IP
   ```

5. **Restart Application:**
   ```bash
   aws autoscaling set-desired-capacity --auto-scaling-group-name tvm-primary --desired-capacity 3
   ```

**Total Recovery Time:** 1 hour (30 min restore + 20 min replay + 10 min validation)

### 8.2 Full Database Restore

**Scenario:** Database completely corrupted, restore from last backup

**Procedure:**

1. **Download Latest Backup:**
   ```bash
   # Identify latest backup
   aws s3 ls s3://tvm-backups/ | grep db-full | tail -1
   # db-full-2025-12-17.dump (yesterday 1 AM UTC)

   # Download (5-10 minutes for 100 GB database)
   aws s3 cp s3://tvm-backups/db-full-2025-12-17.dump /tmp/
   ```

2. **Restore to New Database:**
   ```bash
   # Create new database
   createdb tvm_restored

   # Restore (20-30 minutes)
   pg_restore -d tvm_restored -j 4 /tmp/db-full-2025-12-17.dump
   # -j 4: Use 4 parallel jobs (faster restore)
   ```

3. **Validate:**
   ```bash
   psql -d tvm_restored -c "SELECT COUNT(*) FROM residents;"
   psql -d tvm_restored -c "SELECT COUNT(*) FROM missions;"
   psql -d tvm_restored -c "SELECT MAX(created_at) FROM missions;"
   # Check: Last mission timestamp should be ~1 AM UTC (backup time)
   ```

4. **Switch Application to Restored Database:**
   ```bash
   # Update connection string in TVM server config
   # DATABASE_URL=postgresql://tvm_restored:5432/tvm_restored
   # Restart TVM servers (rolling restart, 5 minutes)
   ```

**Total Recovery Time:** 45 minutes

**Data Loss (RPO):** Up to 24 hours if restoring from daily backup (missions created after 1 AM UTC lost)
- **Mitigation:** Use hourly incremental backups to reduce RPO to 1 hour

---

## 9. TVM Server Recovery

### 9.1 Single Server Failure (Auto-Recovery)

**Scenario:** 1 of 3 TVM servers crashes

**Automatic Recovery (AWS Auto Scaling):**
1. Health check fails (HTTP 200 not returned after 3 attempts, 30 seconds)
2. Auto Scaling Group terminates unhealthy instance
3. Auto Scaling Group launches new instance (2-3 minutes)
4. New instance joins load balancer pool (1 minute)
5. Service continues with 2→3 servers

**Manual Intervention:** None required (fully automated)

**Impact:** No downtime (load balancer routes traffic to healthy servers)

### 9.2 All Servers Failure (Manual Recovery)

**Scenario:** All TVM servers terminated (human error, security group misconfiguration)

**Procedure:**

1. **Launch New Instances from AMI:**
   ```bash
   # Launch 3 instances from latest TVM server AMI
   aws ec2 run-instances --image-id ami-tvm-v1.2.3 --count 3 --instance-type t3.large --key-name tvm-key --security-group-ids sg-12345 --subnet-id subnet-67890
   # Wait for instances to start (2-3 minutes)
   ```

2. **Verify Configuration:**
   ```bash
   # SSH into instances
   ssh ubuntu@<instance-ip>
   # Check: DATABASE_URL, REDIS_URL, environment variables
   # Check: TVM service running
   systemctl status tvm-server
   ```

3. **Add to Load Balancer:**
   ```bash
   aws elbv2 register-targets --target-group-arn <arn> --targets Id=<instance-id-1> Id=<instance-id-2> Id=<instance-id-3>
   # Wait for health checks to pass (2 minutes)
   ```

4. **Verify Service:**
   ```bash
   curl https://api.tvm.example.com/health
   # Expected: {"status": "healthy"}
   ```

**Total Recovery Time:** 10 minutes

### 9.3 Code Rollback

**Scenario:** Bad deployment causes TVM servers to crash repeatedly

**Procedure:**

1. **Identify Last Known Good Version:**
   ```bash
   git log --oneline
   # Identify commit hash of last stable version
   # e.g., abc1234 "v1.2.2 - Stable release"
   ```

2. **Rollback Deployment:**
   ```bash
   # Option A: CI/CD pipeline rollback
   # Trigger deployment of previous version (git tag v1.2.2)

   # Option B: Manual rollback
   git checkout abc1234
   docker build -t tvm-server:v1.2.2 .
   docker push tvm-server:v1.2.2
   # Update Auto Scaling Group launch template to use v1.2.2
   # Terminate current instances (Auto Scaling launches v1.2.2)
   ```

3. **Verify Stability:**
   - Monitor error rates (should drop to <1%)
   - Monitor crash rate (should be 0)

**Total Recovery Time:** 15 minutes

---

## 10. Network and Infrastructure Recovery

### 10.1 DNS Failure

**Scenario:** DNS provider (Route 53) outage

**Recovery:**

1. **Use Backup DNS Provider:**
   - Pre-configured secondary DNS: Cloudflare (backup)
   - Update nameservers at domain registrar: `ns1.route53.com` → `ns1.cloudflare.com`
   - TTL: 1 hour (clients switch within 1 hour)

2. **Emergency Hardcoded IP:**
   - Push OTA update to vehicles: Hardcode TVM server IP address
   - Vehicles bypass DNS (temporary workaround)

**RTO:** 1 hour (DNS propagation)

### 10.2 Load Balancer Failure

**Scenario:** AWS Application Load Balancer (ALB) failure

**Recovery:**

1. **Provision New Load Balancer:**
   ```bash
   aws elbv2 create-load-balancer --name tvm-lb-backup --subnets subnet-1 subnet-2 --security-groups sg-12345
   # Wait for provisioning (5 minutes)
   ```

2. **Register Targets:**
   ```bash
   aws elbv2 register-targets --target-group-arn <arn> --targets Id=<instance-1> Id=<instance-2> Id=<instance-3>
   ```

3. **Update DNS:**
   ```bash
   # Point api.tvm.example.com to new load balancer
   aws route53 change-resource-record-sets --hosted-zone-id Z1234 --change-batch '{...}'
   ```

**RTO:** 10 minutes

### 10.3 Internet Connectivity Loss

**Scenario:** ISP outage at facility

**Recovery:**

1. **Failover to Backup ISP:**
   - Dual-homed network: Primary ISP + Backup ISP (cellular)
   - Router failover: Automatic (if primary link down, switch to backup)
   - Bandwidth: Primary (1 Gbps) → Backup (100 Mbps, sufficient for TVM traffic)

2. **If Both ISPs Down:**
   - Vehicles operate in "offline mode" (autonomous, no TVM connection)
   - Missions queued locally on vehicles, uploaded when connectivity restored

**RTO:** 5 minutes (automatic ISP failover)

---

## 11. DR Testing and Validation

### 11.1 DR Testing Schedule

| Test Type | Frequency | Scope | Duration | Participants |
|-----------|-----------|-------|----------|--------------|
| **Backup Restore Test** | Daily (automated) | Restore last backup to test DB | 30 min | Automated script |
| **Failover Drill** | Monthly | Promote standby DB to primary | 1 hour | DR team (3 people) |
| **Full DR Exercise** | Quarterly | Simulate data center outage | 4 hours | All teams (10 people) |
| **Tabletop Exercise** | Bi-annually | Walk through scenarios (no actual failover) | 2 hours | Leadership + DR team |
| **Surprise Drill** | Annually | Unannounced DR activation (during business hours) | 4 hours | All teams |

### 11.2 DR Drill Procedure

**Quarterly Full DR Exercise:**

1. **Pre-Drill (Week Before):**
   - Schedule drill date/time (announce to all teams)
   - Prepare test plan (scenario, success criteria)
   - Backup production data (safety)

2. **Drill Execution (Day Of):**
   - **09:00 UTC:** Drill start, announcement: "Simulating data center outage in us-east-1"
   - **09:05 UTC:** Shut down primary region TVM servers (manually)
   - **09:10 UTC:** Failover procedure initiated (follow runbook)
   - **09:30 UTC:** Secondary region promoted to primary
   - **09:45 UTC:** Validation tests (create mission, vehicle control)
   - **10:00 UTC:** All clear, drill complete
   - **10:30 UTC:** Failback to primary region (reverse procedure)

3. **Post-Drill (Same Week):**
   - Debrief meeting (what went well, what didn't)
   - Update runbooks (fix outdated steps)
   - Track improvements (Jira tickets)

**Success Criteria:**
- ✅ Secondary region operational within 30 minutes
- ✅ No data loss (all missions created before failover intact)
- ✅ All critical functions working (login, create mission, vehicle telemetry)
- ✅ Failback to primary successful

### 11.3 Drill Metrics

**Track Over Time:**
- **RTO Actual vs. Target:** Did we meet 4-hour target?
- **Issues Encountered:** How many steps failed? Root causes?
- **Runbook Accuracy:** How many runbook steps were outdated?

**Goal:** Reduce RTO by 10% each quarter (e.g., 4h → 3.6h → 3.2h → 3h)

---

## 12. Roles and Responsibilities

### 12.1 DR Team Structure

| Role | Responsibility | Primary | Backup |
|------|----------------|---------|--------|
| **DR Commander** | Overall coordination, decision-making | Unno-san | Maeda-san |
| **Database Lead** | Database failover/restore | TBD (DBA) | Pankaj |
| **Infrastructure Lead** | Server/network recovery | TBD (DevOps) | Pankaj |
| **Application Lead** | TVM server recovery | Pankaj | Unno-san |
| **Vehicle Lead** | Vehicle recovery | Pankaj | TBD |
| **Communications Lead** | Stakeholder updates | Maeda-san | Unno-san |

### 12.2 On-Call Rotation

**24/7 On-Call Coverage:**
- **Primary On-Call:** Responds to pages (15 min SLA)
- **Secondary On-Call:** Backup if primary unavailable (30 min SLA)
- **Escalation:** DR Commander if both unavailable

**Rotation Schedule:**
- **Week 1:** Pankaj (primary), Unno-san (secondary)
- **Week 2:** Unno-san (primary), Pankaj (secondary)
- **Week 3:** TBD (primary), Pankaj (secondary)
- (Rotate weekly)

**On-Call Responsibilities:**
- Respond to pages within 15 minutes
- Assess severity (P0/P1/P2/P3)
- Activate DR team if P0 (critical disaster)

### 12.3 Contact Information

**DR Team Contacts:**

| Name | Role | Phone | Email | PagerDuty |
|------|------|-------|-------|-----------|
| Unno-san | DR Commander | +81-XXX | unno@example.com | @unno |
| Pankaj | Application Lead | +81-YYY | pankaj@example.com | @pankaj |
| Maeda-san | Communications Lead | +81-ZZZ | maeda@example.com | @maeda |

**External Contacts:**
- **AWS Support:** +1-XXX (Enterprise Support)
- **Database Vendor:** postgres-support@example.com
- **Facility Manager:** facility@example.com

---

## 13. Communication Plan

### 13.1 Communication Templates

#### Status Update (Every 30 Minutes During DR)

**Template:**
```
Subject: [DR] Status Update - <Timestamp>

Current Status: IN PROGRESS / RESOLVED
RTO Target: 4 hours
Elapsed Time: <elapsed>
ETA to Resolution: <eta>

Actions Taken:
- <action 1>
- <action 2>

Next Steps:
- <next step 1>
- <next step 2>

Impact:
- <affected systems>

Contact: <name>, <phone>, <email>
```

**Distribution:**
- Internal: All staff (Slack #incidents channel)
- External: Facility manager, key customers (email)

#### All Clear Announcement

**Template:**
```
Subject: [DR] Service Restored - <Timestamp>

RESOLVED: Disaster recovery complete. All systems operational.

Outage Duration: <duration>
Root Cause: <brief cause>
Data Loss: <none / X minutes>

Post-Mortem: Scheduled for <date>, <time>

Thank you for your patience.

Contact: <name>, <phone>, <email>
```

### 13.2 Communication Channels

**Internal:**
- **Slack:** #incidents channel (real-time updates)
- **Email:** all-staff@example.com (major updates only)
- **PagerDuty:** On-call escalation

**External:**
- **Email:** Facility manager, key customers
- **Status Page:** status.tvm.example.com (public status updates)

### 13.3 Stakeholder Notification Matrix

| Stakeholder | When to Notify | Method | SLA |
|-------------|----------------|--------|-----|
| **DR Team** | Immediately upon disaster detection | PagerDuty page | 5 min |
| **All Staff** | P0/P1 disasters | Slack + email | 15 min |
| **Facility Manager** | P0 disasters (service outage) | Phone call + email | 30 min |
| **Customers** | P0 disasters (>1 hour outage) | Email | 1 hour |
| **Public** | P0 disasters (>2 hour outage) | Status page update | 2 hours |

---

## 14. Business Continuity

### 14.1 Degraded Operations

**Scenario:** TVM server down, vehicles cannot receive new missions

**Continuity Plan:**

1. **Manual Dispatch:**
   - Operators use radio/phone to dispatch vehicles manually
   - Vehicles operate in "manual mode" (driver controls via joystick)
   - Low-tech fallback: Paper log of missions (pickup, dropoff, time)

2. **Partial Fleet Operations:**
   - If only some vehicles affected, operational vehicles take on additional missions
   - Prioritize urgent missions (medical appointments, time-sensitive)

3. **Alternative Transport:**
   - If fleet completely down, use facility's backup transport (manual wheelchairs, staff assistance)

**Maximum Duration:** 12 hours (within MTD)

### 14.2 Financial Impact

**Downtime Cost Estimation:**

| Downtime Duration | Lost Revenue | Reputation Damage | Total Cost |
|-------------------|--------------|-------------------|------------|
| **1 hour** | $100 (negligible) | None | $100 |
| **4 hours** | $400 | Low | $1,000 |
| **8 hours** | $800 | Medium | $5,000 |
| **12 hours** | $1,200 | High | $20,000 |
| **24 hours** | $2,400 | Critical (lose customers) | $100,000 |

**Justification for DR Investment:**
- Annual DR cost: $50,000 (infrastructure, testing, personnel)
- Expected disaster frequency: Once every 5 years
- Expected downtime without DR: 48 hours ($200,000 cost)
- Expected downtime with DR: 4 hours ($1,000 cost)
- **Savings:** $199,000 per disaster event
- **ROI:** 4:1 over 5 years

### 14.3 Insurance and Liability

**Cyber Insurance:**
- Coverage: Data breach, ransomware, business interruption
- Limit: $1,000,000
- Deductible: $10,000
- Requirements: Annual security audit, DR plan in place

**Business Interruption Insurance:**
- Coverage: Lost revenue during outages >24 hours
- Limit: $500,000
- Requirements: Documented DR plan, proof of testing

---

## 15. Post-DR Activities

### 15.1 Post-Mortem

**Timing:** Within 48 hours of incident resolution

**Agenda:**
1. **Timeline Review** (30 min)
   - When was disaster detected?
   - When was DR activated?
   - When was service restored?
   - Gaps: Where did we lose time?

2. **Root Cause Analysis** (30 min)
   - What caused the disaster? (5 Whys)
   - Could it have been prevented?

3. **What Went Well** (15 min)
   - Effective procedures
   - Good communication
   - Fast response

4. **What Went Wrong** (15 min)
   - Outdated runbooks
   - Missing tools
   - Communication breakdowns

5. **Action Items** (30 min)
   - Runbook updates (assign owner)
   - Tooling improvements
   - Training needs

**Deliverable:** Post-mortem document (shared with all teams)

### 15.2 Continuous Improvement

**Kaizen Approach:**
- After each DR event or drill, identify 3 improvements
- Track improvements in backlog (Jira)
- Review quarterly: Are we reducing RTO over time?

**Examples:**
- **Improvement 1:** Automate DNS failover (reduce manual step, save 5 minutes)
- **Improvement 2:** Pre-provision standby servers (eliminate cold start, save 10 minutes)
- **Improvement 3:** Improve runbook clarity (add screenshots, reduce confusion)

---

## 16. Document Maintenance

### 16.1 Review Schedule

**Quarterly Review:**
- Update contact information (phone numbers, emails)
- Update RTO/RPO targets (based on business needs)
- Review disaster scenarios (new threats?)

**Annual Review:**
- Full runbook walkthrough (identify outdated steps)
- Compliance check (insurance requirements, regulatory changes)
- Technology updates (new tools, cloud services)

### 16.2 Version Control

**Change Log:**

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0 | 2025-12-17 | Initial version | Pankaj |
| 1.1 | 2026-03-15 | Updated RTO targets after Q1 drill | Unno-san |

**Storage:** Git repository (version controlled, peer-reviewed changes)

---

## Appendix A: Runbook Checklist

### Database Failover Runbook

**Goal:** Promote standby database to primary (RTO: 15 minutes)

**Steps:**
- [ ] **Step 1:** Verify primary database unreachable
  ```bash
  psql -h primary.tvm.internal -c "SELECT 1"
  # Expected: Connection timeout
  ```
- [ ] **Step 2:** Check standby replication lag
  ```bash
  psql -h standby.tvm.internal -c "SELECT pg_last_wal_replay_lsn();"
  # Expected: <30 seconds lag
  ```
- [ ] **Step 3:** Promote standby to primary
  ```bash
  pg_ctl promote -D /var/lib/postgresql/data
  # Expected: "server promoted"
  ```
- [ ] **Step 4:** Update DNS
  ```bash
  aws route53 change-resource-record-sets --hosted-zone-id Z1234 --change-batch '{...}'
  ```
- [ ] **Step 5:** Verify application connectivity
  ```bash
  curl https://api.tvm.example.com/health
  # Expected: {"status": "healthy", "database": "connected"}
  ```
- [ ] **Step 6:** Announce service restored

**Time per Step:** 1 min, 2 min, 5 min, 3 min, 2 min, 2 min = **15 minutes total**

---

## Appendix B: Recovery Time Summary

| Disaster Scenario | Detection | Triage | Execution | Validation | Total RTO | Within Target? |
|-------------------|-----------|--------|-----------|------------|-----------|----------------|
| Single Server Failure | 1 min | 0 min | 3 min (auto) | 1 min | **5 min** | ✅ Yes (<15 min) |
| Database Corruption | 2 min | 5 min | 45 min | 10 min | **62 min** | ✅ Yes (<1 hour) |
| Data Center Outage | 5 min | 10 min | 15 min | 10 min | **40 min** | ✅ Yes (<4 hours) |
| Ransomware Attack | 10 min | 15 min | 6 hours | 30 min | **7 hours** | ❌ No (>4 hours target) |
| Accidental Deletion | 5 min | 10 min | 30 min | 15 min | **60 min** | ✅ Yes (<1 hour) |

**Note:** Ransomware exceeds RTO due to offsite backup retrieval from Glacier (slow). Mitigation: Keep recent backups in S3 Standard (not Glacier) for faster access.

---

## Approval

| Role | Name | Signature | Date |
|------|------|-----------|------|
| **Project Lead** | Maeda-san | __________ | _______ |
| **DR Commander** | Unno-san | __________ | _______ |
| **Application Lead** | Pankaj | __________ | _______ |

---

*End of Document*
