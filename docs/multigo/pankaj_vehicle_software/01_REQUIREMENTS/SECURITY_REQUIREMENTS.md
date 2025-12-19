# Security Requirements (Pilot Level)

**Document ID:** REQ-SEC-001
**Version:** 2.0 (Simplified for Pilot)
**Date:** December 19, 2025
**Status:** Active
**Owner:** Pankaj (Vehicle Software)
**Scope:** Vehicle software security ONLY

---

## Document Information

**Purpose:** Define pilot-level security requirements for wheelchair transport robot vehicle software

**Scope:**
- Vehicle-side software security
- Vehicle-TVM API communication security
- Basic safety-critical security features

**Out of Scope:**
- TVM server security (Unno's responsibility)
- Hardware physical security (Tsuchiya's responsibility)
- Advanced compliance (GDPR, HIPAA, ISO 27001)
- Enterprise features (MFA, SSO, penetration testing)

**Pilot Focus:**
- Essential security for safe operation
- Prevent common vulnerabilities
- Basic authentication and encryption
- Safety-critical protections

---

## Table of Contents

1. [Authentication](#1-authentication)
2. [API Security](#2-api-security)
3. [Network Security](#3-network-security)
4. [Data Protection](#4-data-protection)
5. [Physical Safety](#5-physical-safety)
6. [Logging](#6-logging)
7. [Secure Development](#7-secure-development)

---

## 1. Authentication

### 1.1 Vehicle-TVM Authentication

**REQ-SEC-001** [CRITICAL]
- Vehicle SHALL authenticate to TVM server using JWT (JSON Web Tokens)
- Token expiration: 1 hour
- Refresh token expiration: 24 hours
- **Implementation:** See TVM_API_SPECIFICATION.md

**REQ-SEC-002** [CRITICAL]
- Vehicle SHALL include unique device ID (UUID) in authentication
- Device ID provisioned during initial setup
- **Implementation:** Stored in `/etc/multigo/vehicle_id`

**REQ-SEC-003** [HIGH]
- Vehicle SHALL validate JWT signature on receiving commands from TVM
- Reject expired or invalid tokens
- **Implementation:** Use standard JWT library (PyJWT or cpp-jwt)

**REQ-SEC-004** [HIGH]
- Vehicle SHALL automatically refresh JWT before expiration
- Refresh 5 minutes before token expires
- **Implementation:** Background thread in TVM client

---

## 2. API Security

### 2.1 TVM API Communication

**REQ-SEC-005** [CRITICAL]
- All TVM API communication SHALL use HTTPS (TLS 1.2 or higher)
- No plaintext HTTP allowed
- **Implementation:** TVM API client enforces HTTPS URLs

**REQ-SEC-006** [CRITICAL]
- Vehicle SHALL validate TVM server TLS certificate
- Reject self-signed certificates in production
- Accept self-signed certificates in development (with warning)
- **Implementation:** Python requests library with `verify=True`

**REQ-SEC-007** [HIGH]
- Vehicle SHALL implement retry logic with exponential backoff
- Prevent API abuse from failed requests
- Max retries: 3, backoff: 1s, 2s, 4s
- **Implementation:** TVM client retry wrapper

### 2.2 Input Validation

**REQ-SEC-008** [CRITICAL]
- Vehicle SHALL validate all incoming WebSocket commands
- Check command type, parameters, data types
- Reject malformed commands
- **Implementation:** JSON schema validation

**REQ-SEC-009** [HIGH]
- Vehicle SHALL validate mission waypoints before executing
- Check coordinate bounds, velocity limits, steering angle limits
- **Implementation:** Waypoint validator in nav_goal package

---

## 3. Network Security

### 3.1 Network Communication

**REQ-SEC-010** [CRITICAL]
- All network communication SHALL use encryption:
  - TVM API: HTTPS/WSS (TLS)
  - ROS 2 internal: Localhost only (no external exposure)
- **Implementation:** Configure ROS 2 DDS to bind to localhost

**REQ-SEC-011** [HIGH]
- Vehicle WiFi connection SHALL use WPA2 or WPA3
- Minimum password length: 12 characters
- **Implementation:** Network configuration (out of software scope, document only)

**REQ-SEC-012** [MEDIUM]
- Vehicle SHALL NOT expose ROS 2 topics to public network
- ROS 2 DDS domain restricted to localhost or vehicle internal network
- **Implementation:** ROS_DOMAIN_ID environment variable, firewall rules

---

## 4. Data Protection

### 4.1 Data in Transit

**REQ-SEC-013** [CRITICAL]
- Telemetry data sent to TVM SHALL be encrypted (HTTPS)
- Includes: location, battery status, errors
- **Implementation:** Use TVM REST API with HTTPS

**REQ-SEC-014** [HIGH]
- Vehicle SHALL NOT send sensitive data in URLs
- Use POST body for all sensitive parameters
- **Implementation:** TVM client API design

### 4.2 Data at Rest

**REQ-SEC-015** [MEDIUM]
- Vehicle SHALL NOT store user PII locally
- Only store: vehicle ID, mission data, maps
- **Implementation:** Design principle - no user data persistence

**REQ-SEC-016** [MEDIUM]
- Vehicle logs SHALL NOT contain sensitive data
- Mask: API tokens, passwords, user names
- **Implementation:** Log filter/sanitizer

### 4.3 Secret Management

**REQ-SEC-017** [CRITICAL]
- API keys and credentials SHALL NOT be hardcoded in source code
- Store in configuration file or environment variables
- **Implementation:** Config file (`/etc/multigo/config.yaml`) with restricted permissions (chmod 600)

**REQ-SEC-018** [HIGH]
- Configuration files containing secrets SHALL have restricted permissions
- Readable only by vehicle software user (e.g., `multigo` user)
- **Implementation:** Deployment script sets `chmod 600 /etc/multigo/config.yaml`

---

## 5. Physical Safety

### 5.1 Emergency Stop

**REQ-SEC-019** [CRITICAL]
- Vehicle SHALL respond to hardware E-stop within 100ms
- Emergency stop has highest priority, cannot be overridden
- **Implementation:** Hardware E-stop circuit + software monitor

**REQ-SEC-020** [CRITICAL]
- Vehicle SHALL respond to software E-stop command from TVM within 500ms
- Stop all motors, cancel current mission
- **Implementation:** WebSocket E-stop handler in safety monitor

**REQ-SEC-021** [HIGH]
- Vehicle SHALL log all E-stop events
- Include: timestamp, trigger source (hardware/software), location
- **Implementation:** Safety monitor logs to `/var/log/multigo/safety.log`

### 5.2 Safety Boundaries

**REQ-SEC-022** [HIGH]
- Vehicle SHALL enforce safety limits on commands:
  - Max velocity: 1.5 m/s
  - Max steering angle: ±45°
  - Max acceleration: 0.5 m/s²
- **Implementation:** Swerve drive controller limits

**REQ-SEC-023** [MEDIUM]
- Vehicle SHALL reject commands from TVM if safety checks fail
- Example: Command to drive off map, exceed limits
- **Implementation:** Command validator in nav_control

---

## 6. Logging

### 6.1 Security Logging

**REQ-SEC-024** [HIGH]
- Vehicle SHALL log security events:
  - Authentication attempts (success/failure)
  - E-stop activations
  - Invalid commands received
  - Network connection failures
- **Implementation:** Centralized logging with security event markers

**REQ-SEC-025** [HIGH]
- Security logs SHALL be sent to TVM server
- For centralized monitoring
- **Implementation:** TVM API `/api/v1/errors` endpoint

**REQ-SEC-026** [MEDIUM]
- Vehicle SHALL retain logs locally for 7 days
- Rotate logs to prevent disk full
- **Implementation:** `logrotate` configuration

---

## 7. Secure Development

### 7.1 Code Security

**REQ-SEC-027** [HIGH]
- Code SHALL be reviewed for security vulnerabilities
- Peer review required for safety-critical code
- **Implementation:** GitHub pull request reviews

**REQ-SEC-028** [MEDIUM]
- Dependencies SHALL be kept up to date
- Check for known vulnerabilities monthly
- **Implementation:** `pip list --outdated`, `rosdep update`

**REQ-SEC-029** [MEDIUM]
- Secrets SHALL NOT be committed to version control
- Use `.gitignore` for config files with secrets
- **Implementation:** `.gitignore` includes `*.secret`, `config/*.yaml`

**REQ-SEC-030** [MEDIUM]
- Code SHALL follow secure coding practices
- OWASP Top 10 awareness (SQL injection, XSS, command injection)
- **Implementation:** Developer training, code review checklist

---

## Summary

**Total Requirements:** 30 (Simplified from 152)

### Requirements by Severity

| Severity | Count | Percentage |
|----------|-------|------------|
| CRITICAL | 11 | 37% |
| HIGH | 14 | 47% |
| MEDIUM | 5 | 17% |

### Requirements by Category

| Category | Count | Critical | High | Medium |
|----------|-------|----------|------|--------|
| Authentication | 4 | 2 | 2 | 0 |
| API Security | 5 | 2 | 2 | 1 |
| Network Security | 3 | 1 | 1 | 1 |
| Data Protection | 6 | 2 | 2 | 2 |
| Physical Safety | 5 | 3 | 2 | 0 |
| Logging | 3 | 0 | 2 | 1 |
| Secure Development | 4 | 1 | 3 | 0 |

---

## What Was Removed (Enterprise → Pilot)

**Removed 122 requirements including:**

### Authentication & Authorization
- ❌ Multi-factor authentication (MFA)
- ❌ Single Sign-On (SSO)
- ❌ Password complexity rules (12 chars, rotation, history)
- ❌ Client certificates (mTLS)
- ❌ Detailed RBAC roles (5 user roles)
- **Reason:** Pilot uses simple JWT auth, TVM server handles user management

### Data Protection
- ❌ AES-256 encryption at rest
- ❌ Key management service (AWS KMS, Vault)
- ❌ Key rotation (90 days)
- ❌ Data anonymization for analytics
- ❌ Secure data deletion (GDPR right to erasure)
- **Reason:** Vehicle doesn't store user PII, encryption handled by HTTPS in transit

### Network Security
- ❌ Network segmentation (DMZ, internal, management)
- ❌ Detailed firewall rules
- ❌ DDoS protection
- ❌ VPN for remote access
- ❌ Certificate pinning
- **Reason:** Pilot deployment, simple network setup

### Compliance
- ❌ GDPR compliance (9 requirements)
- ❌ PIPEDA compliance
- ❌ HIPAA compliance
- ❌ ISO 27001, ISO 13849, IEC 62443
- **Reason:** Pilot project, compliance added later if needed

### Incident Response
- ❌ Incident response plan
- ❌ 24/7 on-call rotation
- ❌ Forensics tools
- ❌ Breach notification (72 hours)
- **Reason:** Pilot doesn't need enterprise incident management

### Vulnerability Management
- ❌ Quarterly vulnerability scanning
- ❌ Annual penetration testing
- ❌ SBOM (Software Bill of Materials)
- ❌ 7-day critical patch SLA
- **Reason:** Manual security reviews sufficient for pilot

### Secure Development
- ❌ Static analysis (SAST) in CI/CD
- ❌ Dynamic analysis (DAST)
- ❌ Container security scanning
- ❌ Image signing
- **Reason:** Pilot uses manual code review, CI/CD security added later

---

## Implementation Priority

**Phase 1 (Week 1-2): Critical Security**
1. REQ-SEC-001: JWT authentication
2. REQ-SEC-005: HTTPS for TVM API
3. REQ-SEC-008: Command validation
4. REQ-SEC-017: Secret management
5. REQ-SEC-019, 020: E-stop implementation

**Phase 2 (Week 3-4): High Priority**
1. REQ-SEC-003: JWT validation
2. REQ-SEC-006: TLS certificate validation
3. REQ-SEC-009: Waypoint validation
4. REQ-SEC-022: Safety limits
5. REQ-SEC-024, 025: Security logging

**Phase 3 (Week 5+): Medium Priority**
1. REQ-SEC-012: ROS 2 network isolation
2. REQ-SEC-016: Log sanitization
3. REQ-SEC-028: Dependency updates
4. REQ-SEC-029: Secret .gitignore

---

## Testing Requirements

**Security testing SHALL include:**
1. JWT authentication test (valid/invalid/expired tokens)
2. HTTPS enforcement test (reject HTTP)
3. Command validation test (malformed JSON)
4. E-stop response time test (<100ms hardware, <500ms software)
5. Safety limit test (reject excessive velocity/steering)
6. Log sanitization test (no secrets in logs)

**Security review SHALL include:**
1. Peer review of authentication code
2. Peer review of command validation code
3. Manual test of E-stop functionality
4. Review of configuration file permissions

---

## Notes for Teams

**For Pankaj (Vehicle Software):**
- Implement these 30 requirements in vehicle software
- TVM API client must handle REQ-SEC-001 to REQ-SEC-007
- Safety monitor must handle REQ-SEC-019 to REQ-SEC-023
- Testing must verify all CRITICAL requirements

**For Unno (TVM Server):**
- Server-side security is YOUR responsibility (not in this document)
- Provide JWT tokens as per TVM_API_SPECIFICATION.md
- Implement server-side user authentication
- See your own security requirements document

**For Tsuchiya+Kiril (Hardware):**
- Hardware E-stop is YOUR responsibility
- Vehicle WiFi configuration is YOUR responsibility
- Physical security (locks, tampering detection) is YOUR responsibility
- See your own security requirements document

---

## Changes from v1.0

**v2.0 (December 19, 2025):**
- Reduced from 152 requirements to 30 (80% reduction)
- Removed enterprise features (MFA, SSO, compliance)
- Removed TVM server security (moved to Unno's scope)
- Removed hardware security (moved to Tsuchiya's scope)
- Focused on pilot-level vehicle software security
- Added implementation notes and testing requirements

**v1.0 (December 17, 2025):**
- Initial version with 152 enterprise-level requirements
- Status: Too advanced for pilot (senior feedback)

---

**Document Status:** ✅ Active (Pilot Level)
**Next Review:** After pilot completion (Week 18)
**Approved By:** Senior (December 19, 2025)

---

**END OF DOCUMENT**
