# User Role Management Requirements

**Document Control:**
- **Document ID:** FM-USER-REQ-001
- **Version:** 1.0
- **Date:** 2025-12-16
- **Status:** Draft
- **Owner:** Unno (Fleet Management Team)
- **Reviewers:** Security Team, System Administrator

---

## 1. Introduction

### 1.1 Purpose

This document specifies detailed requirements for User Role Management and Role-Based Access Control (RBAC) in the Fleet Management System. It defines user roles, permissions, authentication mechanisms, and access control policies to ensure secure and appropriate access to system features.

### 1.2 Scope

**In Scope:**
- User role definitions (Admin, Caregiver, Nurse, Operator)
- Permission matrix (granular feature-level permissions)
- User account lifecycle (creation, modification, deactivation)
- Authentication mechanisms (password-based, optional SSO)
- Session management and timeout policies
- Role-based UI customization
- Audit logging for security events
- Password policies and security controls
- Self-service features (password reset, profile update)

**Out of Scope:**
- Network-level security (firewall, VPN)
- Database security (covered in TVM_SERVER_REQUIREMENTS.md)
- Vehicle authentication (separate vehicle credentials)

### 1.3 Definitions and Acronyms

| Term | Definition |
|------|------------|
| RBAC | Role-Based Access Control |
| Admin | Administrator with full system access |
| Caregiver | 介護士 - Staff member providing walking assistance |
| Nurse | 看護士 - Medical staff for medicine delivery |
| Operator | Fleet operator monitoring vehicles |
| Permission | Granular access right to specific feature |
| Session | Authenticated user session with timeout |
| SSO | Single Sign-On (optional integration) |

### 1.4 References

- FLEET_MANAGEMENT_REQUIREMENTS.md - Parent document
- RESERVATION_SYSTEM_REQUIREMENTS.md - Reservation features
- TVM_SERVER_REQUIREMENTS.md - Authentication implementation

---

## 2. System Overview

### 2.1 User Roles

**Role Hierarchy:**
```
Admin (Full Access)
├── Operator (Fleet Management)
├── Nurse (Medical Services)
└── Caregiver (Resident Care)
```

**Role Purposes:**
- **Admin:** System configuration, user management, full data access
- **Operator:** Vehicle monitoring, dispatch override, queue management
- **Nurse:** Medicine delivery reservations, patient transport, urgent priority
- **Caregiver:** Walking assistance reservations, resident transport, routine priority

---

## 3. Functional Requirements

### 3.1 Role Definitions

#### 3.1.1 Admin Role

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| USER-ROLE-ADMIN-001 | CRITICAL | Admin role SHALL have full system access | • All features accessible<br>• No restrictions on data visibility<br>• Can manage all users |
| USER-ROLE-ADMIN-002 | HIGH | Admin SHALL manage user accounts (create, edit, deactivate) | • User management interface<br>• Can assign/change roles<br>• Can reset passwords |
| USER-ROLE-ADMIN-003 | HIGH | Admin SHALL access all audit logs and reports | • Full audit trail visibility<br>• Export logs to CSV/JSON<br>• Filter by user, action, date |
| USER-ROLE-ADMIN-004 | HIGH | Admin SHALL configure system settings | • Operational hours, waypoints, vehicle parameters<br>• Rate limits, notification templates<br>• Requires confirmation for critical changes |
| USER-ROLE-ADMIN-005 | MEDIUM | Admin SHALL override queue ordering | • Manually reorder reservations<br>• Requires justification note<br>• Logged to audit trail |

#### 3.1.2 Operator Role

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| USER-ROLE-OPR-001 | HIGH | Operator role SHALL monitor all vehicles in real-time | • Access to fleet dashboard<br>• Vehicle status, location, battery<br>• Cannot modify system settings |
| USER-ROLE-OPR-002 | HIGH | Operator SHALL dispatch vehicles manually | • Manual dispatch interface<br>• Select vehicle and mission<br>• Bypass automatic assignment |
| USER-ROLE-OPR-003 | HIGH | Operator SHALL cancel active missions | • Cancel button for in-progress missions<br>• Requires confirmation<br>• Logged to audit trail |
| USER-ROLE-OPR-004 | MEDIUM | Operator SHALL view all reservations | • Read-only access to reservation list<br>• Cannot create/modify reservations<br>• Can view queue status |
| USER-ROLE-OPR-005 | MEDIUM | Operator SHALL send emergency stop commands | • E-stop button for each vehicle<br>• Requires two-step confirmation<br>• Logged as critical event |

#### 3.1.3 Nurse Role

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| USER-ROLE-NURSE-001 | CRITICAL | Nurse role SHALL create medicine delivery reservations | • Access to reservation form<br>• Medicine delivery type<br>• Priority selection (routine, urgent, emergency) |
| USER-ROLE-NURSE-002 | HIGH | Nurse SHALL set emergency priority for critical deliveries | • Emergency priority option<br>• Requires confirmation dialog<br>• Immediate queue jump |
| USER-ROLE-NURSE-003 | HIGH | Nurse SHALL view own reservation history | • "My Reservations" page<br>• Filter by date, status<br>• Cannot see other nurses' reservations |
| USER-ROLE-NURSE-004 | MEDIUM | Nurse SHALL receive notifications for reservation status | • In-app notifications<br>• Email notifications (optional)<br>• Status changes: dispatched, completed, failed |
| USER-ROLE-NURSE-005 | MEDIUM | Nurse SHALL view vehicle locations (read-only) | • Map view of vehicle positions<br>• Helps estimate arrival time<br>• Cannot control vehicles |

#### 3.1.4 Caregiver Role

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| USER-ROLE-CARE-001 | CRITICAL | Caregiver role SHALL create walking assistance reservations | • Access to reservation form<br>• Walking assistance type<br>• Routine priority (no emergency option) |
| USER-ROLE-CARE-002 | HIGH | Caregiver SHALL view own reservation history | • Same as nurse (isolated view)<br>• Cannot see other caregivers' reservations<br>• Export to CSV |
| USER-ROLE-CARE-003 | MEDIUM | Caregiver SHALL modify own pending reservations | • Edit time, waypoints before dispatch<br>• Cannot modify after dispatch<br>• Validation same as creation |
| USER-ROLE-CARE-004 | MEDIUM | Caregiver SHALL cancel own pending reservations | • Cancel button<br>• Confirmation dialog<br>• Time slot released |

---

### 3.2 Permission Matrix

#### 3.2.1 Granular Permissions

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| USER-PERM-MATRIX-001 | CRITICAL | System SHALL enforce permission matrix for all features | • Server-side permission checks<br>• 403 Forbidden if insufficient permissions<br>• Client hides unauthorized features (UX) |
| USER-PERM-MATRIX-002 | HIGH | System SHALL define granular permissions | • Permissions: view_dashboard, create_reservation, cancel_mission, manage_users, emergency_stop, view_analytics, configure_system<br>• Each permission independently assignable<br>• Stored in database |
| USER-PERM-MATRIX-003 | HIGH | System SHALL map permissions to roles | • Admin: All permissions<br>• Operator: view_dashboard, cancel_mission, emergency_stop<br>• Nurse: create_reservation (medicine), view_dashboard (limited)<br>• Caregiver: create_reservation (walking), view_own_reservations |

**Permission Matrix Table:**

| Permission | Admin | Operator | Nurse | Caregiver |
|------------|-------|----------|-------|-----------|
| view_dashboard | ✅ | ✅ | ✅ (limited) | ✅ (limited) |
| view_all_vehicles | ✅ | ✅ | ✅ (read-only) | ❌ |
| create_reservation | ✅ | ❌ | ✅ (medicine) | ✅ (walking) |
| modify_own_reservation | ✅ | ❌ | ✅ | ✅ |
| modify_any_reservation | ✅ | ✅ | ❌ | ❌ |
| cancel_mission | ✅ | ✅ | ❌ | ❌ |
| emergency_stop | ✅ | ✅ | ❌ | ❌ |
| manual_dispatch | ✅ | ✅ | ❌ | ❌ |
| manage_users | ✅ | ❌ | ❌ | ❌ |
| configure_system | ✅ | ❌ | ❌ | ❌ |
| view_analytics | ✅ | ✅ | ❌ | ❌ |
| view_audit_logs | ✅ | ❌ | ❌ | ❌ |
| override_queue | ✅ | ✅ | ❌ | ❌ |

---

### 3.3 User Account Management

#### 3.3.1 User Account Creation (Admin Only)

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| USER-MGMT-CREATE-001 | HIGH | Admin SHALL create new user accounts | • Form: username, email, password, role, full name, department<br>• Validation: all fields required<br>• Confirmation displayed |
| USER-MGMT-CREATE-002 | HIGH | System SHALL enforce unique username and email | • Database constraint<br>• Error message if duplicate<br>• Case-insensitive check |
| USER-MGMT-CREATE-003 | HIGH | System SHALL enforce password complexity on creation | • Min 8 characters<br>• 1 uppercase, 1 lowercase, 1 number, 1 special char<br>• Cannot match common passwords (blacklist) |
| USER-MGMT-CREATE-004 | MEDIUM | System SHALL send welcome email to new user | • Email contains: username, temporary password, login link<br>• Force password change on first login<br>• Email within 5 minutes |

#### 3.3.2 User Account Modification (Admin Only)

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| USER-MGMT-EDIT-001 | HIGH | Admin SHALL edit existing user accounts | • Can change: email, role, full name, department, active status<br>• Cannot change username (immutable ID)<br>• Cannot edit own admin role (prevents lockout) |
| USER-MGMT-EDIT-002 | HIGH | System SHALL log all user account modifications | • Audit log entry: timestamp, admin_user, modified_user, field_changed, old_value, new_value<br>• Viewable by admins<br>• Exportable |
| USER-MGMT-EDIT-003 | MEDIUM | System SHALL notify user of account changes | • Email notification when role or email changed<br>• Security alert for critical changes<br>• User can contact admin if unauthorized |

#### 3.3.3 User Account Deactivation (Admin Only)

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| USER-MGMT-DEACT-001 | HIGH | Admin SHALL deactivate user accounts | • Deactivate button (not delete)<br>• Confirmation dialog<br>• User cannot login when deactivated |
| USER-MGMT-DEACT-002 | HIGH | System SHALL preserve data for deactivated users | • Reservation history retained<br>• Audit logs retained<br>• Can reactivate account later |
| USER-MGMT-DEACT-003 | MEDIUM | System SHALL cancel pending reservations on deactivation | • All pending reservations auto-cancelled<br>• Notification sent to other affected users<br>• Active missions allowed to complete |

---

### 3.4 Authentication

#### 3.4.1 Password-Based Authentication

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| USER-AUTH-PWD-001 | CRITICAL | System SHALL authenticate users via username and password | • Login form with validation<br>• Passwords hashed with bcrypt (cost ≥12)<br>• Never store plaintext passwords |
| USER-AUTH-PWD-002 | CRITICAL | System SHALL issue JWT token on successful authentication | • Token expiration: 24 hours<br>• Stored in httpOnly cookie<br>• Includes: user_id, role, permissions |
| USER-AUTH-PWD-003 | HIGH | System SHALL implement rate limiting for login attempts | • Max 5 failed attempts per username in 15 minutes<br>• 15-minute lockout after 5 failures<br>• Admin can unlock accounts |
| USER-AUTH-PWD-004 | HIGH | System SHALL log all authentication events | • Success: timestamp, user_id, IP address<br>• Failure: timestamp, username, IP address, reason<br>• Monitored for suspicious activity |

#### 3.4.2 Password Reset

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| USER-AUTH-RESET-001 | HIGH | System SHALL provide self-service password reset | • "Forgot Password" link on login page<br>• Email with reset link (token expires in 1 hour)<br>• User sets new password |
| USER-AUTH-RESET-002 | HIGH | System SHALL enforce password complexity on reset | • Same rules as creation<br>• Cannot reuse last 3 passwords<br>• Validation feedback in real-time |
| USER-AUTH-RESET-003 | MEDIUM | Admin SHALL reset user passwords manually | • Generate temporary password<br>• Email sent to user<br>• Force password change on next login |

#### 3.4.3 Session Management

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| USER-AUTH-SESSION-001 | HIGH | System SHALL implement session timeout | • Timeout: 8 hours of inactivity<br>• Warning 5 minutes before timeout<br>• "Extend Session" button |
| USER-AUTH-SESSION-002 | HIGH | System SHALL allow user to logout | • Logout button in navigation<br>• Invalidates JWT token<br>• Redirects to login page |
| USER-AUTH-SESSION-003 | MEDIUM | System SHALL refresh JWT token before expiration | • Auto-refresh at 80% of token lifetime (19.2h for 24h token)<br>• Transparent to user<br>• Logout if refresh fails |

---

### 3.5 Role-Based UI Customization

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| USER-UI-CUSTOM-001 | HIGH | System SHALL display role-appropriate dashboard on login | • Admin: Full dashboard with all widgets<br>• Operator: Fleet monitoring dashboard<br>• Nurse/Caregiver: Reservation-focused view |
| USER-UI-CUSTOM-002 | HIGH | System SHALL hide unauthorized features in UI | • Features without permission not displayed<br>• Menu items hidden/grayed out<br>• Prevents user confusion |
| USER-UI-CUSTOM-003 | MEDIUM | System SHALL show role badge in navigation | • Badge displays current role (e.g., "Admin", "Nurse")<br>• Color-coded (red=Admin, blue=Operator, green=Nurse/Caregiver)<br>• Visible at all times |
| USER-UI-CUSTOM-004 | LOW | System SHALL allow user to customize dashboard layout (if role permits) | • Drag-and-drop widgets<br>• Saved per user<br>• Admin and Operator only |

---

### 3.6 Audit Logging

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| USER-AUDIT-001 | CRITICAL | System SHALL log all security-relevant events | • Events: login, logout, failed login, password change, role change, permission change, user create/edit/deactivate<br>• Fields: timestamp, user_id, IP, action, result<br>• Immutable log entries |
| USER-AUDIT-002 | HIGH | System SHALL log all administrative actions | • User management, system config, queue override<br>• Includes before/after values<br>• Filterable by admin, action, date |
| USER-AUDIT-003 | MEDIUM | System SHALL retain audit logs for 90 days minimum | • Configurable retention (default 90 days)<br>• Auto-archive to cold storage<br>• Cannot be deleted by users |
| USER-AUDIT-004 | MEDIUM | System SHALL provide audit log search and export | • Search by: user, action, date range<br>• Export to CSV/JSON<br>• Admin-only access |

---

### 3.7 Password Policies

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| USER-PWD-POLICY-001 | HIGH | System SHALL enforce password complexity requirements | • Min 8 characters, max 128<br>• 1 uppercase, 1 lowercase, 1 number, 1 special (!@#$%^&*)<br>• Checked on client and server |
| USER-PWD-POLICY-002 | HIGH | System SHALL prevent common password usage | • Blacklist: password123, admin, qwerty, etc.<br>• Check against top 10,000 common passwords<br>• Error message: "Password too common" |
| USER-PWD-POLICY-003 | MEDIUM | System SHALL enforce password expiration (optional) | • Configurable expiration (default: disabled)<br>• If enabled: 90-day expiration<br>• Warning 7 days before expiration |
| USER-PWD-POLICY-004 | MEDIUM | System SHALL prevent password reuse | • Cannot reuse last 3 passwords<br>• Password history stored (hashed)<br>• Checked during password change |

---

### 3.8 Self-Service Features

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| USER-SELF-PROFILE-001 | MEDIUM | Users SHALL update own profile information | • Fields: full name, email, language preference, notification settings<br>• Cannot change: username, role<br>• Email change requires verification |
| USER-SELF-PROFILE-002 | MEDIUM | Users SHALL change own password | • Current password required<br>• New password complexity checked<br>• Confirmation email sent |
| USER-SELF-NOTIF-001 | MEDIUM | Users SHALL configure notification preferences | • Enable/disable: email notifications, in-app notifications, reminders<br>• Per-event preferences (reservation confirmed, dispatched, completed)<br>• Saved to user profile |

---

## 4. Non-Functional Requirements

### 4.1 Security

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| USER-SEC-001 | CRITICAL | System SHALL use HTTPS for all authentication requests | • TLS 1.2+ enforced<br>• HTTP redirects to HTTPS<br>• HSTS header enabled |
| USER-SEC-002 | CRITICAL | System SHALL protect against brute-force attacks | • Rate limiting (5 attempts per 15 min)<br>• CAPTCHA after 3 failures (optional)<br>• Account lockout mechanism |
| USER-SEC-003 | HIGH | System SHALL implement CSRF protection | • CSRF token in all forms<br>• SameSite cookie attribute<br>• Validated server-side |
| USER-SEC-004 | HIGH | System SHALL sanitize user inputs | • Prevent XSS (HTML escaping)<br>• Prevent SQL injection (parameterized queries)<br>• Input validation on all fields |

### 4.2 Performance

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| USER-PERF-001 | HIGH | Login SHALL complete within 2 seconds | • From credentials submit to dashboard load<br>• Includes authentication and token generation<br>• 95th percentile <2s |
| USER-PERF-002 | MEDIUM | Permission checks SHALL not degrade API performance | • Overhead <50ms per request<br>• Permissions cached in JWT<br>• No database query per permission check |

---

## 5. Testing Requirements

| Test Type | Coverage | Acceptance |
|-----------|----------|------------|
| Unit Tests | ≥80% code coverage | Permission logic, password validation, role checks |
| Security Tests | OWASP Top 10 | No critical vulnerabilities in authentication/authorization |
| Integration Tests | API endpoints with different roles | Correct permission enforcement |
| Penetration Tests | Login, session management, privilege escalation | No exploits found |

---

## 6. Traceability Matrix

| Requirement Category | Requirement Count | Priority Distribution |
|---------------------|-------------------|----------------------|
| Role Definitions | 18 | CRITICAL: 3, HIGH: 11, MEDIUM: 4 |
| Permission Matrix | 3 | CRITICAL: 1, HIGH: 2 |
| User Account Management | 10 | HIGH: 7, MEDIUM: 3 |
| Authentication | 11 | CRITICAL: 3, HIGH: 7, MEDIUM: 1 |
| Role-Based UI | 4 | HIGH: 2, MEDIUM: 1, LOW: 1 |
| Audit Logging | 4 | CRITICAL: 1, HIGH: 1, MEDIUM: 2 |
| Password Policies | 4 | HIGH: 2, MEDIUM: 2 |
| Self-Service | 3 | MEDIUM: 3 |
| Non-Functional (Security) | 4 | CRITICAL: 2, HIGH: 2 |
| Non-Functional (Performance) | 2 | HIGH: 1, MEDIUM: 1 |
| **TOTAL** | **63** | **CRITICAL: 10, HIGH: 33, MEDIUM: 19, LOW: 1** |

---

## 7. Assumptions and Dependencies

### 7.1 Assumptions

1. User accounts managed internally (not LDAP/Active Directory integration initially)
2. Email server available for notifications
3. Users have unique email addresses
4. Facility provides initial user list (names, roles)

### 7.2 Dependencies

1. TVM_SERVER_REQUIREMENTS.md (authentication implementation)
2. FLEET_MANAGEMENT_REQUIREMENTS.md (UI features)
3. Database server with user/role tables

---

## 8. Open Issues and Risks

### 8.1 Open Issues

| ID | Issue | Owner | Target Resolution |
|----|-------|-------|-------------------|
| USER-ISSUE-001 | SSO integration requirement (future enhancement) | Unno | Week 5 |
| USER-ISSUE-002 | Multi-factor authentication (MFA) requirement | Security Team | Week 5 |

### 8.2 Risks

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| Admin account lockout (forgotten password) | LOW | HIGH | Super-admin recovery process, admin can reset other admins |
| Privilege escalation vulnerability | LOW | CRITICAL | Security testing, code review, penetration testing |
| Session hijacking | LOW | HIGH | httpOnly cookies, HTTPS, CSRF protection |

---

## 9. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-16 | Unno | Initial draft - Week 3 documentation |

---

**END OF DOCUMENT**

**Total Requirements:** 63
**Document Status:** Draft for Review
**Next Review:** Week 4 (security audit and testing plan)
