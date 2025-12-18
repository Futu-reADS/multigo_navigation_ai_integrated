# Security Architecture

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Document Type:** Architecture Specification
**Status:** Active
**Version:** 1.0
**Last Updated:** 2025-12-17
**Owner:** Integration Team (All Teams)

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Security Architecture Overview](#2-security-architecture-overview)
3. [Defense in Depth Architecture](#3-defense-in-depth-architecture)
4. [Identity and Access Management (IAM)](#4-identity-and-access-management-iam)
5. [Network Security Architecture](#5-network-security-architecture)
6. [API Security Architecture](#6-api-security-architecture)
7. [Data Protection Architecture](#7-data-protection-architecture)
8. [Secrets Management Architecture](#8-secrets-management-architecture)
9. [Vehicle Security Architecture](#9-vehicle-security-architecture)
10. [Security Monitoring and SIEM](#10-security-monitoring-and-siem)
11. [Incident Response Architecture](#11-incident-response-architecture)
12. [Physical Security Architecture](#12-physical-security-architecture)
13. [Compliance Mapping](#13-compliance-mapping)
14. [Security Testing Architecture](#14-security-testing-architecture)
15. [Disaster Recovery Architecture](#15-disaster-recovery-architecture)

---

## 1. Executive Summary

### 1.1 Purpose

This document defines the comprehensive security architecture for the outdoor wheelchair transport robot fleet system. It addresses security concerns across all three teams:
- **Team 1 (Pankaj):** Vehicle software security
- **Team 2 (Unno):** TVM server and fleet management security
- **Team 3 (Maeda-san):** Hardware and physical security

The architecture implements a **Defense in Depth** strategy with multiple overlapping security layers to protect:
- Patient safety and privacy
- Operational continuity
- Data integrity and confidentiality
- Physical assets
- Regulatory compliance (GDPR, PIPEDA, ISO 13849, IEC 62443)

### 1.2 Security Goals

**Primary Security Objectives:**
1. **Confidentiality:** Protect PII, medical data, and operational secrets
2. **Integrity:** Prevent unauthorized modification of data, commands, or software
3. **Availability:** Ensure 99.5% uptime (target SLA) with DDoS protection
4. **Authentication:** Verify identity of all users, vehicles, and systems
5. **Authorization:** Enforce least-privilege access control
6. **Auditability:** Maintain immutable audit logs for compliance
7. **Resilience:** Recover from security incidents within RTO/RPO targets

### 1.3 Threat Model Summary

**Key Threats Addressed:**
- **External Attackers:** DDoS, API exploitation, credential stuffing
- **Insider Threats:** Unauthorized data access, privilege escalation
- **Physical Tampering:** Vehicle component theft, hardware modification
- **Network Attacks:** MITM, eavesdropping, rogue access points
- **Supply Chain:** Compromised dependencies, malicious firmware
- **Operational:** Misconfiguration, weak passwords, unpatched vulnerabilities

**Out of Scope (Accepted Risks):**
- Nation-state APT attacks (beyond current scope)
- Quantum computing attacks (TLS 1.3 provides adequate near-term protection)
- Physical destruction of entire data center (insurance-covered)

### 1.4 Architecture Principles

1. **Zero Trust:** Never trust, always verify (even internal traffic)
2. **Least Privilege:** Minimum necessary permissions for all entities
3. **Defense in Depth:** Multiple overlapping security layers
4. **Fail Secure:** System defaults to safe state on security failures
5. **Secure by Default:** Secure configurations out-of-the-box
6. **Separation of Concerns:** Isolated security domains
7. **Auditability:** All security-relevant events logged

---

## 2. Security Architecture Overview

### 2.1 High-Level Security Layers

```
┌─────────────────────────────────────────────────────────────────┐
│                    LAYER 7: GOVERNANCE & COMPLIANCE             │
│         (Policies, Procedures, Training, Audits)                │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│              LAYER 6: SECURITY OPERATIONS (SecOps)              │
│    (SIEM, Incident Response, Threat Intelligence, Forensics)    │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│            LAYER 5: APPLICATION SECURITY                        │
│  (Input Validation, Output Encoding, Session Mgmt, Auth)       │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│              LAYER 4: DATA SECURITY                             │
│  (Encryption at Rest/Transit, Key Management, Tokenization)     │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│            LAYER 3: NETWORK SECURITY                            │
│     (Firewalls, IDS/IPS, VPN, Network Segmentation)            │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│          LAYER 2: HOST/ENDPOINT SECURITY                        │
│  (Hardening, Antivirus, EDR, Patch Management, File Integrity)  │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│            LAYER 1: PHYSICAL SECURITY                           │
│  (Facility Access, Vehicle Locks, TPM, Secure Boot)            │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 Security Zones

The system is divided into **5 security zones** with controlled traffic flow:

```
┌───────────────────────────────────────────────────────────────────┐
│                       ZONE 1: INTERNET                            │
│              (Untrusted, Public Internet)                         │
└───────────────┬───────────────────────────────────────────────────┘
                │ Firewall + WAF + DDoS Protection
                ↓
┌───────────────────────────────────────────────────────────────────┐
│                   ZONE 2: DMZ (Demilitarized Zone)                │
│  - API Gateway (HTTPS only)                                       │
│  - Load Balancer                                                  │
│  - Web Application Firewall (WAF)                                 │
└───────────────┬───────────────────────────────────────────────────┘
                │ Firewall (Stateful Inspection)
                ↓
┌───────────────────────────────────────────────────────────────────┐
│               ZONE 3: APPLICATION TIER                            │
│  - TVM Backend Server (Node.js/Python/Java)                       │
│  - Authentication Service (Keycloak or custom)                    │
│  - API Services (REST, WebSocket)                                 │
└───────────────┬───────────────────────────────────────────────────┘
                │ Firewall (Database ACLs)
                ↓
┌───────────────────────────────────────────────────────────────────┐
│                 ZONE 4: DATA TIER                                 │
│  - PostgreSQL Database (encrypted at rest)                        │
│  - Redis Cache (encrypted)                                        │
│  - Secrets Vault (HashiCorp Vault or AWS Secrets Manager)        │
└───────────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────────┐
│           ZONE 5: VEHICLE NETWORK (Isolated)                      │
│  - Fleet of Vehicles (WiFi to TVM via VPN or mTLS)               │
│  - Vehicle Internal Network (ROS 2 with SROS2)                    │
│  - Sensors, Actuators, Compute Unit                               │
└───────────────────────────────────────────────────────────────────┘
```

**Zone Traffic Rules:**
- Internet → DMZ: HTTPS (443) only
- DMZ → Application: HTTPS (443), controlled by API Gateway
- Application → Data: PostgreSQL (5432), Redis (6379), encrypted
- Vehicles → DMZ: HTTPS (443), authenticated via mTLS or JWT
- **No direct access** from Internet to Application/Data tiers
- **No direct access** from Vehicles to Data tier

---

## 3. Defense in Depth Architecture

### 3.1 Multi-Layer Security Strategy

**Defense in Depth** ensures that if one security layer is compromised, additional layers continue to protect the system.

```
     ┌─────────────────────────────────────────────────┐
     │    Attack Surface: Public API Endpoint          │
     └────────────────┬────────────────────────────────┘
                      ↓
     ┌─────────────────────────────────────────────────┐
     │ Layer 1: Rate Limiting (1000 req/min per IP)    │ ← Prevents brute force
     └────────────────┬────────────────────────────────┘
                      ↓
     ┌─────────────────────────────────────────────────┐
     │ Layer 2: WAF (OWASP Top 10 Protection)          │ ← Blocks SQL injection, XSS
     └────────────────┬────────────────────────────────┘
                      ↓
     ┌─────────────────────────────────────────────────┐
     │ Layer 3: TLS 1.3 Encryption                     │ ← Prevents MITM
     └────────────────┬────────────────────────────────┘
                      ↓
     ┌─────────────────────────────────────────────────┐
     │ Layer 4: API Authentication (JWT or mTLS)       │ ← Verifies identity
     └────────────────┬────────────────────────────────┘
                      ↓
     ┌─────────────────────────────────────────────────┐
     │ Layer 5: Authorization (RBAC)                   │ ← Checks permissions
     └────────────────┬────────────────────────────────┘
                      ↓
     ┌─────────────────────────────────────────────────┐
     │ Layer 6: Input Validation                       │ ← Sanitizes malicious input
     └────────────────┬────────────────────────────────┘
                      ↓
     ┌─────────────────────────────────────────────────┐
     │ Layer 7: Database ACLs                          │ ← Limits DB access
     └────────────────┬────────────────────────────────┘
                      ↓
     ┌─────────────────────────────────────────────────┐
     │ Layer 8: Audit Logging                          │ ← Detects anomalies
     └────────────────┬────────────────────────────────┘
                      ↓
     ┌─────────────────────────────────────────────────┐
     │ Layer 9: SIEM Monitoring                        │ ← Alerts on threats
     └─────────────────────────────────────────────────┘
```

**Result:** An attacker must successfully bypass **all 9 layers** to compromise data.

### 3.2 Security Controls by Layer

| Layer | Controls | Team Responsible |
|-------|----------|------------------|
| **Physical** | Facility access, vehicle locks, TPM, Secure Boot | Team 3 (Hardware) |
| **Host** | OS hardening, antivirus, patch management | Teams 1 & 2 (Vehicle & TVM) |
| **Network** | Firewalls, VPN, network segmentation, IDS/IPS | Team 2 (TVM Infrastructure) |
| **Data** | Encryption at rest/transit, key rotation, backups | Team 2 (TVM Backend) |
| **Application** | Input validation, session management, SAST/DAST | Teams 1 & 2 (All Software) |
| **SecOps** | SIEM, incident response, threat hunting | Team 2 (TVM Operations) |
| **Governance** | Policies, training, audits, compliance | Integration Team (All) |

---

## 4. Identity and Access Management (IAM)

### 4.1 IAM Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Identity Providers (IdP)                 │
│  - Internal: Keycloak or Auth0                              │
│  - External (Optional): SSO (SAML/OAuth2)                   │
└──────────────────────┬──────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────────────┐
│              Authentication Layer                           │
│  - Multi-Factor Authentication (MFA)                        │
│  - Password Policy Enforcement                              │
│  - Session Management (JWT)                                 │
└──────────────────────┬──────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────────────┐
│            Authorization Layer (RBAC)                       │
│  - Role Definitions (Admin, Operator, Nurse, etc.)          │
│  - Permission Mapping (API endpoints to roles)              │
│  - Policy Enforcement Point (PEP)                           │
└──────────────────────┬──────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────────────┐
│               Resource Access                               │
│  - TVM API (REST + WebSocket)                               │
│  - Vehicle Control Commands                                 │
│  - PII Data Access                                          │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 Authentication Mechanisms

#### 4.2.1 Human Users (TVM Dashboard)

**Authentication Flow:**
1. User enters username + password on TVM Dashboard
2. Password hashed and compared with bcrypt hash (cost factor 12)
3. If valid, TOTP code requested (MFA)
4. Upon successful MFA, JWT issued:
   - **Access Token:** 1 hour expiration
   - **Refresh Token:** 7 days expiration, stored in httpOnly cookie
5. Access token sent in `Authorization: Bearer <token>` header

**JWT Structure:**
```json
{
  "header": {
    "alg": "RS256",
    "typ": "JWT"
  },
  "payload": {
    "sub": "user-uuid-1234",
    "email": "operator@example.com",
    "role": "operator",
    "permissions": ["vehicle:view", "mission:create"],
    "iat": 1703001600,
    "exp": 1703005200
  },
  "signature": "RS256_signature..."
}
```

**Token Storage:**
- **Access Token:** In-memory (not localStorage to prevent XSS)
- **Refresh Token:** httpOnly cookie (prevents JS access)

#### 4.2.2 Vehicle-TVM Authentication

**Option A: Mutual TLS (mTLS) - RECOMMENDED**

```
Vehicle                                    TVM Server
  │                                             │
  │─────── TLS Handshake (Client Cert) ───────→│
  │                                             │ Verify client cert
  │                                             │ (issued by internal CA)
  │←────── TLS Handshake (Server Cert) ────────│
  │                                             │
  │ Verify server cert                          │
  │                                             │
  │──────── Encrypted API Requests ───────────→│
```

**mTLS Configuration:**
- Each vehicle has unique client certificate (X.509)
- Certificates issued by internal CA (offline root CA)
- Certificate validity: 1 year, automatic rotation at 6 months
- Certificate revocation list (CRL) checked on connection
- Private key stored in TPM 2.0 on vehicle compute unit

**Option B: JWT with Vehicle Credentials (Fallback if mTLS not feasible)**

```
Vehicle                                    TVM Server
  │                                             │
  │──── POST /auth/vehicle ───────────────────→│
  │     {vehicle_id, api_key}                   │
  │                                             │ Verify vehicle_id + api_key
  │                                             │ (hashed in database)
  │←────── JWT (1 hour expiration) ────────────│
  │                                             │
  │──── API Requests with JWT ────────────────→│
  │     Authorization: Bearer <jwt>             │
```

### 4.3 Authorization (RBAC)

**Role Definitions:**

| Role | Permissions | Use Case |
|------|-------------|----------|
| **Admin** | Full system access, user management, config changes | IT administrators |
| **Operator** | Monitor fleet, dispatch missions, view telemetry | Control room staff |
| **Nurse** | Request transport, view mission status, cancel missions | Healthcare staff |
| **Caregiver** | Request transport for assigned residents only | Care facility staff |
| **Maintenance** | View vehicle diagnostics, initiate maintenance mode | Technicians |
| **Guest** | View public status dashboard only | Visitors |

**Permission Matrix:**

| Resource | Admin | Operator | Nurse | Caregiver | Maintenance | Guest |
|----------|-------|----------|-------|-----------|-------------|-------|
| Create Mission | ✅ | ✅ | ✅ | ✅ | ❌ | ❌ |
| Cancel Mission | ✅ | ✅ | ✅ | Own only | ❌ | ❌ |
| View Telemetry | ✅ | ✅ | ✅ | Limited | ✅ | ❌ |
| E-Stop Vehicle | ✅ | ✅ | ❌ | ❌ | ✅ | ❌ |
| View PII | ✅ | Limited | Own patients | Own residents | ❌ | ❌ |
| Manage Users | ✅ | ❌ | ❌ | ❌ | ❌ | ❌ |
| Config Changes | ✅ | ❌ | ❌ | ❌ | ❌ | ❌ |

**RBAC Enforcement:**
- Checked on **every API request** (no caching of permissions)
- Implemented in middleware layer (before business logic)
- Denied by default (whitelist approach)

### 4.4 Session Management

**Session Security:**
- **Session Timeout:** 30 minutes inactivity, 8 hours absolute
- **Concurrent Sessions:** Max 3 per user
- **Session Revocation:** Immediate upon password change or logout
- **Session Storage:** Redis with encryption at rest

**Session Lifecycle:**
```
Login → Create Session (Redis) → Issue JWT
  ↓
Activity → Refresh Session TTL (sliding window)
  ↓
Logout → Delete Session (Redis) → Revoke JWT (blacklist)
```

---

## 5. Network Security Architecture

### 5.1 Network Topology

```
                        ┌──────────────────┐
                        │    Internet      │
                        └────────┬─────────┘
                                 │
                ┌────────────────┴────────────────┐
                │      DDoS Protection            │
                │  (Cloudflare or AWS Shield)     │
                └────────────────┬────────────────┘
                                 │
                ┌────────────────┴────────────────┐
                │    Firewall (Stateful)          │
                │  Allow: 443 (HTTPS)             │
                │  Block: All other ports         │
                └────────────────┬────────────────┘
                                 │
        ┌────────────────────────┴────────────────────────┐
        │               DMZ (10.0.1.0/24)                 │
        │  ┌──────────────┐      ┌──────────────┐        │
        │  │ Load Balancer│      │     WAF      │        │
        │  │  (HAProxy)   │      │  (ModSecurity)│        │
        │  └──────┬───────┘      └──────┬───────┘        │
        └─────────┼──────────────────────┼────────────────┘
                  │                      │
        ┌─────────┴──────────────────────┴────────────────┐
        │     Internal Firewall (ACLs)                    │
        └─────────┬───────────────────────────────────────┘
                  │
        ┌─────────┴─────────────────────────────────────┐
        │       Application Tier (10.0.2.0/24)          │
        │  ┌────────────┐  ┌────────────┐  ┌────────┐  │
        │  │ TVM Server │  │Auth Service│  │ SIEM   │  │
        │  │ (Node.js)  │  │(Keycloak)  │  │(ELK)   │  │
        │  └─────┬──────┘  └─────┬──────┘  └────────┘  │
        └────────┼───────────────┼──────────────────────┘
                 │               │
        ┌────────┴───────────────┴──────────────────────┐
        │         Data Tier (10.0.3.0/24)               │
        │  ┌────────────┐  ┌────────────┐  ┌────────┐  │
        │  │ PostgreSQL │  │   Redis    │  │ Vault  │  │
        │  │ (Primary)  │  │  (Cache)   │  │(Secrets)│  │
        │  └────────────┘  └────────────┘  └────────┘  │
        └───────────────────────────────────────────────┘

        ┌───────────────────────────────────────────────┐
        │      Vehicle Network (Dynamic IPs)            │
        │  ┌────────┐  ┌────────┐  ┌────────┐          │
        │  │Vehicle1│  │Vehicle2│  │Vehicle3│  ...     │
        │  │(WiFi)  │  │(WiFi)  │  │(WiFi)  │          │
        │  └────────┘  └────────┘  └────────┘          │
        │  Connect to TVM via HTTPS (443) with mTLS    │
        └───────────────────────────────────────────────┘
```

### 5.2 Firewall Rules

#### External Firewall (Internet → DMZ)

| Rule | Source | Destination | Port | Protocol | Action | Reason |
|------|--------|-------------|------|----------|--------|--------|
| 1 | Any | DMZ | 443 | TCP | ALLOW | HTTPS to TVM |
| 2 | Any | DMZ | 80 | TCP | ALLOW | HTTP redirect to HTTPS |
| 3 | Admin IPs | DMZ | 22 | TCP | ALLOW | SSH for management |
| 4 | Any | DMZ | Any | Any | DENY | Default deny |

#### Internal Firewall (DMZ → Application Tier)

| Rule | Source | Destination | Port | Protocol | Action | Reason |
|------|--------|-------------|------|----------|--------|--------|
| 1 | Load Balancer | TVM Server | 8080 | TCP | ALLOW | API traffic |
| 2 | WAF | TVM Server | 8080 | TCP | ALLOW | Filtered traffic |
| 3 | Any | Application | Any | Any | DENY | Default deny |

#### Database Firewall (Application → Data Tier)

| Rule | Source | Destination | Port | Protocol | Action | Reason |
|------|--------|-------------|------|----------|--------|--------|
| 1 | TVM Server | PostgreSQL | 5432 | TCP | ALLOW | DB queries |
| 2 | TVM Server | Redis | 6379 | TCP | ALLOW | Cache access |
| 3 | TVM Server | Vault | 8200 | TCP | ALLOW | Secrets retrieval |
| 4 | Backup Server | PostgreSQL | 5432 | TCP | ALLOW | DB backups |
| 5 | Any | Data Tier | Any | Any | DENY | Default deny |

### 5.3 Network Segmentation

**VLANs:**
- **VLAN 10:** DMZ (public-facing)
- **VLAN 20:** Application tier (internal)
- **VLAN 30:** Data tier (highly restricted)
- **VLAN 40:** Management network (SSH, monitoring)
- **VLAN 50:** Vehicle network (WiFi AP isolated)

**Inter-VLAN Routing:**
- All inter-VLAN traffic goes through firewall (no direct routing)
- Stateful inspection of all packets
- Deny by default, allow by exception

### 5.4 Vehicle-TVM Secure Communication

#### Option A: VPN Tunnel (OpenVPN or WireGuard)

```
Vehicle                          TVM Server
  │                                   │
  │─── VPN Handshake (WireGuard) ───→│
  │    (Pre-shared key per vehicle)   │
  │                                   │
  │←─── VPN Tunnel Established ──────│
  │    (Encrypted tunnel)             │
  │                                   │
  │─── API Requests over VPN ────────→│
  │    (Additional TLS layer)         │
```

**VPN Configuration:**
- **Protocol:** WireGuard (faster, more secure than OpenVPN)
- **Key Management:** Unique pre-shared key per vehicle
- **IP Assignment:** Static IP per vehicle within VPN (10.1.0.0/16)
- **Encryption:** ChaCha20-Poly1305
- **Key Rotation:** Every 90 days (automated)

#### Option B: mTLS over Public Internet (No VPN)

```
Vehicle                          TVM Server
  │                                   │
  │─── TLS 1.3 + Client Cert ───────→│
  │    (mTLS handshake)               │
  │                                   │
  │←─── Server Cert Verification ────│
  │                                   │
  │─── Encrypted API Requests ───────→│
```

**Advantages of mTLS:**
- Simpler infrastructure (no VPN server)
- Built-in mutual authentication
- TLS 1.3 provides strong encryption

**Recommendation:** Use **mTLS** for simplicity unless VPN already exists.

### 5.5 DDoS Protection

**Layer 3/4 DDoS (Network/Transport):**
- Use **Cloudflare** or **AWS Shield** for volumetric attacks
- Rate limiting: Max 1000 requests/min per IP
- SYN flood protection at firewall

**Layer 7 DDoS (Application):**
- WAF rate limiting per endpoint:
  - `/auth/login`: 10 attempts/min per IP
  - `/api/vehicles`: 100 req/min per user
  - `/api/missions`: 50 req/min per user
- CAPTCHA after 3 failed login attempts

---

## 6. API Security Architecture

### 6.1 API Gateway Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Internet                             │
└───────────────────────┬─────────────────────────────────┘
                        │ HTTPS (TLS 1.3)
                        ↓
┌─────────────────────────────────────────────────────────┐
│                  API Gateway                            │
│  ┌───────────────────────────────────────────────────┐ │
│  │ 1. Rate Limiting (1000 req/min per IP)           │ │
│  │ 2. WAF (OWASP Top 10 Protection)                 │ │
│  │ 3. Authentication (JWT Verification)             │ │
│  │ 4. Authorization (RBAC Check)                    │ │
│  │ 5. Input Validation (Schema Validation)          │ │
│  │ 6. Logging (Audit Trail)                         │ │
│  └───────────────────────────────────────────────────┘ │
└───────────────────────┬─────────────────────────────────┘
                        │ Internal HTTPS
                        ↓
┌─────────────────────────────────────────────────────────┐
│              TVM Backend Services                       │
│  - Vehicle Service                                      │
│  - Mission Service                                      │
│  - User Service                                         │
└─────────────────────────────────────────────────────────┘
```

### 6.2 API Security Controls

#### 6.2.1 Input Validation

**Validation Layers:**
1. **Schema Validation:** JSON schema validation (e.g., ajv library)
2. **Type Checking:** Enforce data types (string, number, enum)
3. **Length Limits:** Max string length, array size
4. **Format Validation:** Email, UUID, date formats
5. **Range Validation:** Min/max for numbers
6. **Whitelist Validation:** Allowed values for enums

**Example: Create Mission API**

```json
{
  "endpoint": "POST /api/missions",
  "schema": {
    "type": "object",
    "required": ["pickup_location", "dropoff_location", "resident_id"],
    "properties": {
      "pickup_location": {
        "type": "string",
        "minLength": 1,
        "maxLength": 100,
        "pattern": "^[a-zA-Z0-9\\s\\-]+$"
      },
      "dropoff_location": {
        "type": "string",
        "minLength": 1,
        "maxLength": 100,
        "pattern": "^[a-zA-Z0-9\\s\\-]+$"
      },
      "resident_id": {
        "type": "string",
        "format": "uuid"
      },
      "priority": {
        "type": "string",
        "enum": ["low", "normal", "high", "urgent"]
      }
    }
  }
}
```

#### 6.2.2 Output Encoding

**Prevent XSS:**
- HTML encode all user-generated content before rendering
- Use Content-Security-Policy (CSP) header:
  ```
  Content-Security-Policy: default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'
  ```
- Sanitize JSON responses (remove sensitive fields)

#### 6.2.3 Rate Limiting

**Rate Limits by Endpoint:**

| Endpoint | Limit | Window | Reason |
|----------|-------|--------|--------|
| `/auth/login` | 10 req | 1 min | Prevent brute force |
| `/auth/register` | 5 req | 1 hour | Prevent spam accounts |
| `/api/missions` | 50 req | 1 min | Normal usage |
| `/api/vehicles` | 100 req | 1 min | Frequent polling |
| `/api/telemetry` | 200 req | 1 min | Real-time updates |

**Rate Limiting Implementation:**
- Use **Redis** for distributed rate limiting
- Algorithm: **Sliding Window** (more accurate than fixed window)
- Response: HTTP 429 Too Many Requests

#### 6.2.4 API Versioning

**Versioning Strategy:**
- **URL Path Versioning:** `/api/v1/vehicles`, `/api/v2/vehicles`
- **Breaking Changes:** New major version (v1 → v2)
- **Backward Compatibility:** v1 maintained for 12 months after v2 release
- **Deprecation Notice:** 6 months before EOL

### 6.3 CORS (Cross-Origin Resource Sharing)

**CORS Configuration:**
```javascript
{
  "allowedOrigins": [
    "https://tvm-dashboard.example.com"  // Production dashboard
  ],
  "allowedMethods": ["GET", "POST", "PUT", "DELETE"],
  "allowedHeaders": ["Authorization", "Content-Type"],
  "exposedHeaders": ["X-Request-ID"],
  "maxAge": 86400,  // 24 hours
  "credentials": true  // Allow cookies
}
```

**Security:**
- Never use `Access-Control-Allow-Origin: *` in production
- Whitelist specific domains only

---

## 7. Data Protection Architecture

### 7.1 Encryption Architecture

```
┌─────────────────────────────────────────────────────────┐
│              ENCRYPTION AT REST                         │
│                                                         │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐   │
│  │  Database   │  │   Backups   │  │   Logs      │   │
│  │  (AES-256)  │  │  (AES-256)  │  │  (AES-256)  │   │
│  └─────────────┘  └─────────────┘  └─────────────┘   │
│                                                         │
│  Key Management:                                        │
│  - Master Key in HSM or KMS                            │
│  - Data Encryption Keys (DEKs) rotated annually        │
│  - Envelope Encryption (DEK encrypted by Master Key)   │
└─────────────────────────────────────────────────────────┘
                        ↕
┌─────────────────────────────────────────────────────────┐
│             ENCRYPTION IN TRANSIT                       │
│                                                         │
│  TLS 1.3 (all communication):                          │
│  - Client ↔ TVM Server: TLS 1.3                        │
│  - TVM Server ↔ Database: TLS 1.2+                     │
│  - Vehicle ↔ TVM Server: TLS 1.3 + mTLS               │
│                                                         │
│  Cipher Suites (Preferred Order):                      │
│  1. TLS_AES_256_GCM_SHA384                             │
│  2. TLS_CHACHA20_POLY1305_SHA256                       │
│  3. TLS_AES_128_GCM_SHA256                             │
└─────────────────────────────────────────────────────────┘
```

### 7.2 Key Management

#### 7.2.1 Key Hierarchy

```
┌─────────────────────────────────────────────────────────┐
│              Master Key (KMS or HSM)                    │
│  - Stored in AWS KMS or Hardware Security Module       │
│  - Never leaves KMS/HSM                                 │
│  - Rotated every 5 years (manual)                      │
└─────────────────┬───────────────────────────────────────┘
                  │ Encrypts
                  ↓
┌─────────────────────────────────────────────────────────┐
│        Data Encryption Keys (DEKs)                      │
│  - One DEK per data type (e.g., PII, telemetry)        │
│  - Stored encrypted in database                         │
│  - Rotated annually (automated)                         │
└─────────────────┬───────────────────────────────────────┘
                  │ Encrypts
                  ↓
┌─────────────────────────────────────────────────────────┐
│                 Data at Rest                            │
│  - Resident PII (name, contact, medical info)          │
│  - Vehicle telemetry (location, battery, errors)       │
│  - Audit logs                                           │
└─────────────────────────────────────────────────────────┘
```

**Key Rotation Schedule:**
- **Master Key:** Every 5 years (manual, planned downtime)
- **DEKs:** Every 12 months (automated, zero downtime)
- **TLS Certificates:** Every 12 months (automated via Let's Encrypt)
- **JWT Signing Keys:** Every 6 months (automated)

#### 7.2.2 Secrets Management

**HashiCorp Vault Architecture:**

```
┌─────────────────────────────────────────────────────────┐
│              Applications (TVM Server, Vehicles)        │
└───────────────────────┬─────────────────────────────────┘
                        │ Authenticate (AppRole)
                        ↓
┌─────────────────────────────────────────────────────────┐
│                  HashiCorp Vault                        │
│  ┌───────────────────────────────────────────────────┐ │
│  │ Secrets Engines:                                  │ │
│  │ - KV (Key-Value): DB passwords, API keys         │ │
│  │ - Database: Dynamic DB credentials                │ │
│  │ - PKI: TLS certificates for vehicles             │ │
│  └───────────────────────────────────────────────────┘ │
│                                                         │
│  ┌───────────────────────────────────────────────────┐ │
│  │ Audit Logging: All secret access logged          │ │
│  └───────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
```

**Secrets Stored in Vault:**
- Database passwords (PostgreSQL, Redis)
- API keys (external services)
- JWT signing keys
- Encryption keys (DEKs)
- Vehicle mTLS certificates

**Access Control:**
- TVM Server: Read-only access to `secret/tvm/*`
- Vehicles: Read-only access to `secret/vehicle/{vehicle_id}/*`
- Humans: No direct access (use Vault UI with MFA)

### 7.3 Data Classification

| Data Class | Examples | Encryption | Access Control | Retention |
|------------|----------|------------|----------------|-----------|
| **Public** | System status, weather | None | Public API | Indefinite |
| **Internal** | Vehicle telemetry (non-PII) | TLS only | Authenticated users | 90 days |
| **Confidential** | Resident names, mission details | At rest + transit | RBAC (need-to-know) | 3 years |
| **Restricted** | Medical info, SSN | At rest + transit + field-level | Admin only | 7 years (compliance) |

**Field-Level Encryption (FLE):**
- Restricted fields (e.g., SSN, medical notes) encrypted separately
- Each field has unique DEK
- Application decrypts only when needed (and user has permission)

---

## 8. Secrets Management Architecture

### 8.1 Secrets Lifecycle

```
┌─────────────────────────────────────────────────────────┐
│ 1. CREATION                                             │
│    - Generate strong random secret (256-bit entropy)    │
│    - Store in Vault with metadata (owner, expiration)   │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 2. DISTRIBUTION                                         │
│    - Application authenticates to Vault (AppRole)       │
│    - Vault issues time-limited token (TTL: 24 hours)    │
│    - Application retrieves secret (ephemeral)           │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 3. USAGE                                                │
│    - Secret used in memory only (never persisted)       │
│    - Audit log records access                           │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 4. ROTATION                                             │
│    - Automated rotation every 90 days (configurable)    │
│    - Old secret valid for 7 days (grace period)         │
│    - Applications automatically refresh                 │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 5. REVOCATION                                           │
│    - Immediate revocation on compromise                 │
│    - All active tokens invalidated                      │
│    - New secret generated                               │
└─────────────────────────────────────────────────────────┘
```

### 8.2 Vault Deployment

**High Availability Setup:**

```
┌─────────────────────────────────────────────────────────┐
│              Vault Cluster (HA)                         │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐    │
│  │  Vault 1    │  │  Vault 2    │  │  Vault 3    │    │
│  │  (Active)   │  │ (Standby)   │  │ (Standby)   │    │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘    │
│         │                │                │             │
│         └────────────────┴────────────────┘             │
│                    Raft Consensus                       │
└─────────────────────────────────────────────────────────┘
```

**Vault Unsealing:**
- Use **Shamir's Secret Sharing** (5 key shards, 3 required to unseal)
- Key shards held by 5 different personnel
- Auto-unseal with AWS KMS (recommended for production)

---

## 9. Vehicle Security Architecture

### 9.1 Vehicle Compute Unit Security

```
┌─────────────────────────────────────────────────────────┐
│              Vehicle Compute Unit                       │
│  ┌───────────────────────────────────────────────────┐ │
│  │ Layer 1: BIOS/UEFI Secure Boot                    │ │
│  │  - Only signed bootloader allowed                 │ │
│  │  - TPM 2.0 for key storage                        │ │
│  └───────────────────────────────────────────────────┘ │
│                      ↓                                  │
│  ┌───────────────────────────────────────────────────┐ │
│  │ Layer 2: OS Hardening (Ubuntu 22.04)              │ │
│  │  - Minimal install (no GUI)                       │ │
│  │  - AppArmor enabled                               │ │
│  │  - Automatic security updates                     │ │
│  └───────────────────────────────────────────────────┘ │
│                      ↓                                  │
│  ┌───────────────────────────────────────────────────┐ │
│  │ Layer 3: SROS2 (Secure ROS 2)                     │ │
│  │  - DDS security enabled                           │ │
│  │  - Encrypted ROS topics (AES-256-GCM)             │ │
│  │  - Access control per topic                       │ │
│  └───────────────────────────────────────────────────┘ │
│                      ↓                                  │
│  ┌───────────────────────────────────────────────────┐ │
│  │ Layer 4: Application Security                     │ │
│  │  - ROS 2 nodes run as non-root user              │ │
│  │  - File integrity monitoring (AIDE)               │ │
│  │  - Local firewall (ufw)                           │ │
│  └───────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
```

### 9.2 SROS2 Configuration

**Secure ROS 2 (SROS2) Architecture:**

```
┌─────────────────────────────────────────────────────────┐
│              ROS 2 Security Keystore                    │
│  - Root CA (offline, secured)                           │
│  - Vehicle identity certificates                        │
│  - Topic-level permissions                              │
└─────────────────┬───────────────────────────────────────┘
                  │
                  ↓
┌─────────────────────────────────────────────────────────┐
│            DDS Security (Secure Middleware)             │
│  - Authentication: Mutual TLS (vehicle nodes)           │
│  - Encryption: AES-256-GCM (all ROS topics)             │
│  - Access Control: Topic-level ACLs                     │
└─────────────────┬───────────────────────────────────────┘
                  │
                  ↓
┌─────────────────────────────────────────────────────────┐
│                  ROS 2 Topics                           │
│  /cmd_vel (Control Commands) ← HIGH SECURITY            │
│  /odometry (Telemetry)        ← MEDIUM SECURITY         │
│  /diagnostics                 ← LOW SECURITY            │
└─────────────────────────────────────────────────────────┘
```

**Topic-Level Permissions:**

| Topic | Publish | Subscribe | Reason |
|-------|---------|-----------|--------|
| `/cmd_vel` | nav_control node only | All nodes | Prevent command injection |
| `/e_stop` | safety node, dashboard | All nodes | Emergency stop |
| `/odometry` | localization node | nav_control, TVM bridge | Prevent spoofing |
| `/camera/image` | camera node | perception nodes | Bandwidth management |

### 9.3 Vehicle Physical Security

**Physical Tampering Prevention:**

1. **Enclosure Locks:**
   - Compute unit enclosure locked with key
   - Tamper-evident seals on enclosure screws
   - Intrusion detection switch (triggers alarm if opened)

2. **Component Security:**
   - Emergency stop button cannot be disabled (hardware-level)
   - Battery disconnect requires physical access (locked compartment)
   - LiDAR and cameras securely mounted (anti-theft bolts)

3. **Theft Prevention:**
   - Vehicle immobilization when parked (wheel brakes engaged)
   - GPS tracker (hidden, battery-backed)
   - Remote disable command from TVM (kills motor power)

**Tamper Detection:**
- If compute unit enclosure opened: Vehicle enters safe mode, sends alert to TVM
- If GPS tracker removed: Alert to TVM, last known location logged

### 9.4 Vehicle Software Updates

**Secure OTA (Over-The-Air) Updates:**

```
┌─────────────────────────────────────────────────────────┐
│ 1. Update Preparation (CI/CD Pipeline)                  │
│    - Build new ROS 2 packages                           │
│    - Sign with code signing certificate (offline key)   │
│    - Upload to S3 bucket (encrypted)                    │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 2. Update Distribution (TVM Server)                     │
│    - TVM notifies vehicles of available update          │
│    - Vehicles download update manifest (signed)         │
│    - Verify signature before download                   │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 3. Update Installation (Vehicle)                        │
│    - Download update package over TLS                   │
│    - Verify signature + checksum (SHA-256)              │
│    - Install in staging partition (A/B partitioning)    │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 4. Update Validation                                    │
│    - Reboot into new partition                          │
│    - Health check (ROS nodes, sensors, connectivity)    │
│    - If failed: Rollback to previous partition          │
│    - If success: Mark new partition as stable           │
└─────────────────────────────────────────────────────────┘
```

**Update Rollback:**
- Automatic rollback if vehicle fails health check 3 times
- Manual rollback from TVM dashboard
- Previous version retained for 30 days (emergency rollback)

---

## 10. Security Monitoring and SIEM

### 10.1 SIEM Architecture

**ELK Stack (Elasticsearch, Logstash, Kibana):**

```
┌─────────────────────────────────────────────────────────┐
│              Log Sources                                │
│  - TVM Server (application logs)                        │
│  - Vehicles (ROS logs, system logs)                     │
│  - Firewall (network logs)                              │
│  - WAF (security events)                                │
│  - Database (audit logs)                                │
└───────────────────┬─────────────────────────────────────┘
                    │ Forward logs
                    ↓
┌─────────────────────────────────────────────────────────┐
│              Logstash (Aggregation)                     │
│  - Parse logs (JSON, syslog)                            │
│  - Enrich with metadata (GeoIP, threat intel)           │
│  - Filter sensitive data (PII redaction)                │
└───────────────────┬─────────────────────────────────────┘
                    │ Index
                    ↓
┌─────────────────────────────────────────────────────────┐
│           Elasticsearch (Storage)                       │
│  - Store logs with 90-day retention                     │
│  - Hot-Warm-Cold architecture (performance)             │
│  - Index by date (daily indices)                        │
└───────────────────┬─────────────────────────────────────┘
                    │ Query
                    ↓
┌─────────────────────────────────────────────────────────┐
│              Kibana (Visualization)                     │
│  - Security dashboards (failed logins, anomalies)       │
│  - Real-time alerting (Watcher)                         │
│  - Incident investigation (search, filter)              │
└─────────────────────────────────────────────────────────┘
```

### 10.2 Security Alerts

**Alert Rules:**

| Alert | Trigger | Severity | Action |
|-------|---------|----------|--------|
| **Failed Login Attempts** | >5 from same IP in 5 min | HIGH | Block IP, notify admin |
| **Privilege Escalation** | Non-admin user attempts admin action | CRITICAL | Revoke session, alert SOC |
| **Data Exfiltration** | >100 MB data download by single user | HIGH | Throttle, alert admin |
| **Unauthorized API Access** | 401/403 HTTP codes >10 in 1 min | MEDIUM | Investigate user |
| **Vehicle Offline** | No heartbeat for >5 min | HIGH | Dispatch team |
| **E-Stop Triggered** | Emergency stop button pressed | CRITICAL | Alert control room immediately |
| **Tamper Detection** | Vehicle enclosure opened | CRITICAL | Immobilize vehicle, alert |

**Alert Destinations:**
- **Email:** Security team (for HIGH/CRITICAL)
- **Slack/PagerDuty:** On-call engineer (for CRITICAL)
- **Dashboard:** Real-time alert panel in TVM dashboard

### 10.3 Anomaly Detection

**Machine Learning-Based Anomaly Detection:**
- **Baseline:** Normal vehicle behavior (speed, routes, battery drain)
- **Anomalies:** Detect deviations from baseline
  - Unexpected route changes (possible hijacking)
  - Abnormal battery drain (possible hardware fault)
  - Unusual API call patterns (possible account takeover)

**Implementation:** Use Elasticsearch ML features or external tool (e.g., Splunk UBA)

---

## 11. Incident Response Architecture

### 11.1 Incident Response Plan

**IR Team Roles:**
- **Incident Commander:** Coordinates response (Unno-san)
- **Technical Lead:** Investigates root cause (Pankaj)
- **Communications Lead:** Stakeholder updates (Maeda-san)
- **Legal/Compliance:** Regulatory reporting (External counsel)

**Incident Severity Levels:**

| Severity | Definition | Response Time | Example |
|----------|------------|---------------|---------|
| **P0 (Critical)** | Patient safety risk, data breach | <15 min | Vehicle collision, PII leak |
| **P1 (High)** | Service outage, security exploit | <1 hour | TVM server down, API exploit |
| **P2 (Medium)** | Degraded performance, minor breach | <4 hours | Slow API, failed login alerts |
| **P3 (Low)** | Minor issues, no immediate impact | <24 hours | Log storage full, expired cert |

### 11.2 Incident Response Workflow

```
┌─────────────────────────────────────────────────────────┐
│ 1. DETECTION                                            │
│    - SIEM alert triggers                                │
│    - User reports issue                                 │
│    - Automated monitoring (Prometheus)                  │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 2. TRIAGE                                               │
│    - Incident Commander assesses severity               │
│    - Assign to appropriate team                         │
│    - Create incident ticket (Jira/PagerDuty)            │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 3. CONTAINMENT                                          │
│    - Isolate affected systems (e.g., quarantine vehicle)│
│    - Revoke compromised credentials                     │
│    - Block malicious IPs                                │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 4. INVESTIGATION                                        │
│    - Collect logs, forensic images                      │
│    - Root cause analysis (5 Whys, Fishbone)             │
│    - Timeline reconstruction                            │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 5. ERADICATION                                          │
│    - Remove malware, backdoors                          │
│    - Patch vulnerabilities                              │
│    - Reset compromised passwords                        │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 6. RECOVERY                                             │
│    - Restore from backups                               │
│    - Bring systems back online                          │
│    - Monitor for recurrence                             │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 7. POST-MORTEM                                          │
│    - Incident report (root cause, impact, lessons)      │
│    - Action items (prevent recurrence)                  │
│    - Update runbooks                                    │
└─────────────────────────────────────────────────────────┘
```

### 11.3 Security Incident Playbooks

**Playbook 1: Data Breach Response**

1. **Immediate Actions (<15 min):**
   - Isolate affected database server (revoke network access)
   - Revoke all active user sessions (invalidate JWTs)
   - Notify Incident Commander and Legal team

2. **Investigation (1-4 hours):**
   - Identify scope: Which data? How many records?
   - Review access logs: Who accessed? When?
   - Preserve evidence (forensic disk images)

3. **Containment (4-8 hours):**
   - Patch vulnerability (if identified)
   - Reset all admin passwords
   - Enable additional monitoring on database

4. **Notification (within 72 hours - GDPR requirement):**
   - Notify affected individuals
   - Report to data protection authority (if EU residents affected)
   - Public disclosure (if required by law)

5. **Remediation:**
   - Implement additional security controls (e.g., database firewall)
   - Security audit of all systems
   - Penetration testing

**Playbook 2: Vehicle Hijacking Attempt**

1. **Immediate Actions:**
   - Send remote immobilize command (kill motor power)
   - Alert control room and security team
   - If passenger on board: Send reassurance message on vehicle display

2. **Investigation:**
   - Review vehicle logs: Unauthorized commands?
   - Check TVM logs: API calls from unexpected IPs?
   - Verify vehicle certificate (not compromised)

3. **Recovery:**
   - Manually retrieve vehicle (if needed)
   - Re-image compute unit (clean install)
   - Reissue vehicle certificate

**Playbook 3: DDoS Attack**

1. **Immediate Actions:**
   - Enable DDoS protection (Cloudflare "Under Attack" mode)
   - Increase rate limits temporarily
   - Scale up infrastructure (auto-scaling)

2. **Mitigation:**
   - Identify attack type (volumetric, application-layer)
   - Block attack patterns (WAF rules)
   - Contact ISP/hosting provider for upstream filtering

3. **Recovery:**
   - Monitor traffic until attack subsides
   - Review attack patterns (lessons learned)
   - Adjust permanent rate limits if needed

---

## 12. Physical Security Architecture

### 12.1 Facility Physical Security

**Data Center / Server Room:**

```
┌─────────────────────────────────────────────────────────┐
│              Outer Perimeter                            │
│  - Fence with barbed wire                               │
│  - CCTV cameras (24/7 recording)                        │
│  - Security guard (access control)                      │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│              Building Entrance                          │
│  - Badge access (RFID)                                  │
│  - Visitor log                                          │
│  - Metal detector (optional)                            │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│              Server Room                                │
│  - Biometric access (fingerprint)                       │
│  - Two-person rule (for access)                         │
│  - CCTV inside room                                     │
│  - Environmental monitoring (temp, humidity)            │
└─────────────────────────────────────────────────────────┘
```

**Physical Access Control:**
- **Access Levels:**
  - Level 1 (Building): All employees
  - Level 2 (Server Room): IT staff only
  - Level 3 (Rack Access): Data center admins only

- **Access Logging:** All badge swipes logged with timestamp

### 12.2 Vehicle Physical Security

**Vehicle Locking Mechanisms:**
1. **Compute Unit Enclosure:** Keyed lock, tamper-evident seals
2. **Battery Compartment:** Padlock, only accessible by maintenance
3. **Emergency Stop:** Cannot be locked (must be always accessible)

**Anti-Theft Measures:**
- GPS tracker (hidden, battery-backed for 7 days)
- Remote immobilization (kill motor power via TVM command)
- Loud alarm if tampered (90 dB siren)

### 12.3 Hardware Security

**Trusted Platform Module (TPM 2.0):**
- Store mTLS private keys (cannot be extracted)
- Secure Boot measurements (verify OS integrity)
- Sealed storage (data encrypted, only accessible if OS unmodified)

**Secure Boot Chain:**
```
UEFI Firmware (signed by manufacturer)
   ↓
Bootloader (GRUB, signed by our CA)
   ↓
Linux Kernel (signed by our CA)
   ↓
Initramfs (verified by kernel)
   ↓
ROS 2 System (verified by file integrity monitoring)
```

**Result:** If any component tampered, boot fails (brick device until manual recovery)

---

## 13. Compliance Mapping

### 13.1 Security Controls to Compliance Standards

| Control | ISO 27001 | NIST CSF | IEC 62443 | GDPR | ISO 13849 |
|---------|-----------|----------|-----------|------|-----------|
| **Authentication (MFA)** | A.9.4.2 | PR.AC-1 | SR 1.1 | Art. 32 | - |
| **Encryption at Rest** | A.10.1.1 | PR.DS-1 | SR 4.1 | Art. 32 | - |
| **Encryption in Transit** | A.13.1.1 | PR.DS-2 | SR 4.2 | Art. 32 | - |
| **Access Control (RBAC)** | A.9.2.3 | PR.AC-4 | SR 1.5 | Art. 25 | - |
| **Audit Logging** | A.12.4.1 | DE.CM-1 | CR 2.1 | Art. 30 | - |
| **Incident Response** | A.16.1.1 | RS.RP-1 | CR 2.5 | Art. 33 | - |
| **Vulnerability Management** | A.12.6.1 | ID.RA-1 | SR 7.1 | Art. 32 | - |
| **Backup & Recovery** | A.12.3.1 | PR.IP-4 | CR 3.1 | Art. 17 | - |
| **E-Stop (Safety)** | - | - | SR 6.1 | - | Cat 3/SIL 2 |
| **Physical Security** | A.11.1.1 | PR.AC-2 | CR 1.1 | Art. 32 | - |

### 13.2 GDPR Compliance Architecture

**GDPR Requirements:**

| Requirement | Implementation | Architecture Component |
|-------------|----------------|------------------------|
| **Art. 25: Privacy by Design** | Minimize PII collection, pseudonymization | Data model design (only essential PII) |
| **Art. 30: Records of Processing** | Audit logs of all PII access | SIEM (Elasticsearch) |
| **Art. 32: Security of Processing** | Encryption, access control, MFA | IAM, Encryption architecture |
| **Art. 33: Breach Notification** | Incident response plan (72-hour notification) | IR playbooks |
| **Art. 15: Right to Access** | API endpoint to retrieve user data | `/api/gdpr/access/:user_id` |
| **Art. 17: Right to Erasure** | API endpoint to delete user data (with exceptions) | `/api/gdpr/delete/:user_id` |
| **Art. 20: Data Portability** | Export user data in JSON format | `/api/gdpr/export/:user_id` |

**GDPR API Endpoints:**

```http
GET /api/gdpr/access/:user_id
Authorization: Bearer <admin_jwt>
→ Returns all PII for user (JSON)

DELETE /api/gdpr/delete/:user_id
Authorization: Bearer <admin_jwt>
→ Anonymizes user data (cannot delete audit logs per legal hold)

GET /api/gdpr/export/:user_id
Authorization: Bearer <admin_jwt>
→ Returns user data in portable format (JSON)
```

### 13.3 ISO 13849 (Safety) Compliance

**Safety Integrity Level: SIL 2 (Category 3)**

**Safety Functions:**

| Function | Implementation | Redundancy | Validation |
|----------|----------------|------------|------------|
| **E-Stop** | Hardware button → kill motor power | Dual-channel (redundant circuits) | Manual testing monthly |
| **Obstacle Detection** | LiDAR + cameras | Dual-sensor (3D + 2D) | Automated testing daily |
| **Dead Man's Switch** | Operator must hold button (manual mode) | N/A (manual mode only) | Tested before each manual session |
| **Speed Limiting** | Software limit (1.5 m/s with passenger) | Hardware limiter (backup) | Automated testing daily |

**Safety Architecture:**

```
┌─────────────────────────────────────────────────────────┐
│         Safety-Critical Path (Redundant)                │
│                                                         │
│  E-Stop Button                                          │
│      ↓                                                  │
│  Safety Relay (Dual-Channel)                            │
│      ↓                                                  │
│  Motor Controller (Hardware Kill)                       │
│      ↓                                                  │
│  Motors (Power Cut)                                     │
│                                                         │
│  Safety PLC monitors ROS 2 system                       │
│  If ROS 2 fails → Safety PLC takes over                 │
└─────────────────────────────────────────────────────────┘
```

**Result:** If software fails, hardware safety layer remains functional (fail-safe).

---

## 14. Security Testing Architecture

### 14.1 Testing Strategy

**Security Testing Pyramid:**

```
                  ┌───────────────┐
                  │  Penetration  │  (Annually)
                  │    Testing    │  External firm
                  └───────┬───────┘
                          │
              ┌───────────┴───────────┐
              │   Security Scanning   │  (Monthly)
              │ DAST (ZAP, Burp Suite)│  Automated
              └───────────┬───────────┘
                          │
          ┌───────────────┴───────────────┐
          │    Static Analysis (SAST)     │  (Every commit)
          │ SonarQube, Semgrep, Bandit    │  CI/CD pipeline
          └───────────────┬───────────────┘
                          │
      ┌───────────────────┴───────────────────┐
      │  Dependency Scanning (SCA)            │  (Daily)
      │  Snyk, Dependabot, npm audit          │  Automated
      └───────────────────────────────────────┘
```

### 14.2 Testing Schedule

| Test Type | Frequency | Tool | Responsibility |
|-----------|-----------|------|----------------|
| **SAST (Static)** | Every commit | SonarQube, Semgrep | CI/CD pipeline (Team 2) |
| **Dependency Scan** | Daily | Snyk, npm audit | CI/CD pipeline (Team 2) |
| **DAST (Dynamic)** | Monthly | OWASP ZAP | Security team (Team 2) |
| **Penetration Test** | Annually | External firm | External vendor |
| **Code Review** | Every PR | Manual | All teams |
| **Threat Modeling** | Per feature | Manual (STRIDE) | Architecture review |

### 14.3 Vulnerability Management

**Vulnerability Lifecycle:**

```
┌─────────────────────────────────────────────────────────┐
│ 1. DISCOVERY                                            │
│    - Dependency scan (Snyk alert)                       │
│    - Penetration test findings                          │
│    - Researcher disclosure                              │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 2. TRIAGE                                               │
│    - Severity assessment (CVSS score)                   │
│    - Exploitability analysis (PoC available?)           │
│    - Assign priority (Critical → 7 days SLA)            │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 3. REMEDIATION                                          │
│    - Patch application (update dependency)              │
│    - Test fix (unit + integration tests)                │
│    - Deploy to production (emergency deployment)        │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 4. VERIFICATION                                         │
│    - Re-scan to confirm fix                             │
│    - Penetration test (if high severity)                │
│    - Close ticket                                       │
└─────────────────────────────────────────────────────────┘
```

**Patch SLAs:**
- **Critical:** 7 days
- **High:** 30 days
- **Medium:** 90 days
- **Low:** 180 days (or next release)

---

## 15. Disaster Recovery Architecture

### 15.1 Backup Architecture

**Backup Strategy (3-2-1 Rule):**
- **3 copies** of data: Production + 2 backups
- **2 different media**: Disk (primary backup) + Cloud (offsite backup)
- **1 offsite** copy: AWS S3 Glacier (different region)

```
┌─────────────────────────────────────────────────────────┐
│              Production Database (Primary)              │
│              PostgreSQL (us-east-1)                     │
└───────────────────┬─────────────────────────────────────┘
                    │
                    │ Streaming Replication (Real-time)
                    ↓
┌─────────────────────────────────────────────────────────┐
│            Standby Database (Hot Standby)               │
│           PostgreSQL (us-west-2)                        │
│           - Read-only queries allowed                   │
│           - Automatic failover (if primary fails)       │
└───────────────────┬─────────────────────────────────────┘
                    │
                    │ Daily Backup (1 AM UTC)
                    ↓
┌─────────────────────────────────────────────────────────┐
│              Backup Storage (S3)                        │
│           - Full backup: Weekly (Sunday)                │
│           - Incremental backup: Daily                   │
│           - Retention: 90 days                          │
└───────────────────┬─────────────────────────────────────┘
                    │
                    │ Replication (Cross-Region)
                    ↓
┌─────────────────────────────────────────────────────────┐
│           Offsite Backup (S3 Glacier)                   │
│           - Region: eu-west-1 (different continent)     │
│           - Retention: 7 years (compliance)             │
└─────────────────────────────────────────────────────────┘
```

### 15.2 Disaster Recovery Procedures

**RTO/RPO Targets:**
- **Recovery Time Objective (RTO):** 4 hours (time to restore service)
- **Recovery Point Objective (RPO):** 1 hour (max data loss acceptable)

**DR Scenarios:**

| Scenario | Impact | RTO | Recovery Procedure |
|----------|--------|-----|---------------------|
| **Database Corruption** | Data tier down | 1 hour | Restore from last backup |
| **Data Center Outage** | Full system down | 4 hours | Failover to standby region |
| **Ransomware Attack** | All systems encrypted | 8 hours | Restore from offsite backup (Glacier) |
| **Single Server Failure** | Partial outage | 15 min | Auto-scaling replaces instance |

**DR Runbook: Data Center Outage**

1. **Detection (0-5 min):**
   - Monitoring alerts: "Primary data center unreachable"
   - Incident Commander notified

2. **Decision (5-15 min):**
   - Assess: Is primary recovering? (wait 10 min)
   - If not: Initiate failover to standby region

3. **Failover (15-60 min):**
   - Promote standby database to primary (1 command)
   - Update DNS: Point `api.tvm.example.com` to standby region (5 min TTL)
   - Restart application servers in standby region
   - Update vehicle configuration (new TVM server URL)

4. **Verification (60-90 min):**
   - Test critical paths (login, create mission, vehicle telemetry)
   - Monitor error rates
   - All clear: Service restored

5. **Post-Incident (24 hours):**
   - Root cause analysis: Why did primary fail?
   - Rebuild primary data center
   - Sync data from standby back to primary

---

## 16. Security Roadmap

### 16.1 Phase 1: MVP Security (Weeks 1-12)

**Essential Security (Minimum Viable Security):**

| Component | Implementation | Status |
|-----------|----------------|--------|
| **Authentication** | JWT with bcrypt passwords | ✅ Required for MVP |
| **Authorization** | Basic RBAC (Admin, Operator, Nurse) | ✅ Required for MVP |
| **Encryption in Transit** | TLS 1.3 for all APIs | ✅ Required for MVP |
| **Encryption at Rest** | AES-256 for PII | ✅ Required for MVP |
| **Audit Logging** | Log all security events to file | ✅ Required for MVP |
| **Firewall** | Basic iptables rules | ✅ Required for MVP |

**Acceptable Risks for MVP:**
- No SIEM (manual log review)
- No WAF (API rate limiting only)
- No mTLS for vehicles (JWT authentication acceptable)
- No SROS2 (ROS 2 topics unencrypted)

### 16.2 Phase 2: Production Security (Weeks 13-24)

**Harden for Production:**

| Component | Implementation | Timeline |
|-----------|----------------|----------|
| **MFA** | TOTP for admin users | Week 14-15 |
| **SIEM** | ELK stack deployment | Week 16-18 |
| **WAF** | ModSecurity with OWASP rules | Week 19-20 |
| **mTLS** | Vehicle-TVM mutual TLS | Week 21-22 |
| **Secrets Management** | HashiCorp Vault | Week 23-24 |

### 16.3 Phase 3: Advanced Security (Months 7-12)

**Security Maturity:**

| Component | Implementation | Timeline |
|-----------|----------------|----------|
| **SROS2** | Secure ROS 2 with DDS security | Month 7-8 |
| **Penetration Testing** | External security audit | Month 9 |
| **SOC 2 Audit** | Third-party compliance audit | Month 10-11 |
| **Bug Bounty Program** | Public responsible disclosure | Month 12 |

---

## 17. Conclusion

This security architecture provides **comprehensive defense-in-depth** protection for the outdoor wheelchair transport robot fleet system. Key highlights:

**✅ Strong Foundation:**
- Multi-layer security (physical → network → application → data)
- Zero Trust architecture (verify everything)
- Industry-standard compliance (ISO 27001, GDPR, IEC 62443)

**✅ Practical Implementation:**
- Clear ownership (Team 1, 2, 3 responsibilities)
- Phased rollout (MVP → Production → Advanced)
- Automation where possible (CI/CD security scanning)

**✅ Operational Excellence:**
- SIEM for 24/7 monitoring
- Incident response playbooks
- Disaster recovery procedures (4-hour RTO)

**🚧 Phase 1 Focus (MVP):**
1. Implement JWT authentication + RBAC
2. Enable TLS 1.3 for all communication
3. Encrypt PII at rest (AES-256)
4. Set up basic audit logging
5. Configure firewalls

**Next Steps:**
1. Review this architecture with all three teams
2. Create security implementation tickets (Jira)
3. Assign security champions for each team
4. Schedule security training for developers

---

**Document Metadata:**
- **Version:** 1.0
- **Created:** 2025-12-17
- **Next Review:** 2026-03-17 (quarterly review)
- **Owner:** Integration Team (All Teams)

**Related Documents:**
- `SECURITY_REQUIREMENTS.md` (152 requirements implemented by this architecture)
- `PRIVACY_REQUIREMENTS.md` (to be created)
- `DISASTER_RECOVERY_PLAN.md` (to be created)
- `INCIDENT_RESPONSE_PLAN.md` (to be created)

---

**Approval Signatures:**

| Role | Name | Signature | Date |
|------|------|-----------|------|
| **Project Lead** | Maeda-san | __________ | _______ |
| **Vehicle Software Lead** | Pankaj | __________ | _______ |
| **TVM Lead** | Unno-san | __________ | _______ |
| **Security Officer** | TBD | __________ | _______ |

---

*End of Document*
