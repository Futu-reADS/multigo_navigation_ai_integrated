# Fleet Management System Requirements

**Document Control:**
- **Document ID:** FM-REQ-001
- **Version:** 1.0
- **Date:** 2025-12-16
- **Status:** Draft
- **Owner:** Unno (Fleet Management Team)
- **Reviewers:** Pankaj (Integration), Project Management

---

## 1. Introduction

### 1.1 Purpose

This document specifies the requirements for the Fleet Management System (TVM - Total Vehicle Management), which provides centralized monitoring, control, and coordination of autonomous wheelchair transport robots. The system enables facility staff (caregivers, nurses, administrators) to dispatch vehicles, track missions, manage reservations, and monitor fleet status in real-time.

### 1.2 Scope

**In Scope:**
- Fleet dashboard (real-time monitoring, map visualization)
- Reservation system (walking assistance, medicine delivery)
- User role management (Admin, Caregiver, Nurse)
- Real-time vehicle tracking and status
- Mission dispatch and queue management
- Communication system (voice, alerts, notifications)
- Reporting and analytics
- Integration with TVM Server API (defined in TVM_API_SPECIFICATION.md)

**Out of Scope:**
- Vehicle-side software (covered in VEHICLE requirements)
- TVM server backend implementation (covered in TVM_SERVER_REQUIREMENTS.md)
- Hardware components (covered in HARDWARE requirements)
- Log upload system (separate tvm_upload daemon)

### 1.3 Definitions and Acronyms

| Term | Definition |
|------|------------|
| TVM | Total Vehicle Management - fleet management system |
| Admin | Administrator with full system access |
| Caregiver | 介護士 - Staff member providing walking assistance |
| Nurse | 看護士 - Medical staff delivering medicine/supplies |
| Mission | A single transport task (pickup → destination) |
| Reservation | Pre-scheduled mission request |
| Dispatch | Real-time mission assignment to vehicle |
| Fleet | Collection of all managed vehicles |
| Waypoint | Named location in facility map |

### 1.4 References

- TVM_API_SPECIFICATION.md - Vehicle ↔ TVM Server API contract
- TVM_DATA_MODELS.md - JSON schemas and data models
- TEAM_RESPONSIBILITIES.md - Team scope and ownership
- Excel Use Cases - Caregiver and nurse workflow scenarios

---

## 2. System Overview

### 2.1 System Context

The Fleet Management System is a web-based application accessible from PC and tablet devices. It connects to the TVM Server backend via REST API and WebSocket for real-time updates. The system serves three primary user roles:

1. **Administrators** - Full system access, user management, analytics
2. **Caregivers (介護士)** - Request walking assistance, track vehicle status
3. **Nurses (看護士)** - Request medicine delivery, emergency transport

### 2.2 Key Features

- Real-time vehicle tracking (1 Hz location updates)
- Interactive facility map with vehicle positions
- Reservation system with priority queue
- Role-based access control (RBAC)
- Voice communication (VoIP) between staff and passengers
- Emergency alert system
- Mission history and analytics
- Multi-language support (EN, JA)

---

## 3. Functional Requirements

### 3.1 Fleet Dashboard

#### 3.1.1 Real-Time Vehicle Monitoring

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-DASH-MON-001 | CRITICAL | System SHALL display real-time status of all vehicles in the fleet | • Vehicle list updates within 2 seconds of status change<br>• Display: vehicle ID, status, battery, location, current mission<br>• Color-coded status indicators (green=idle, blue=navigating, yellow=charging, red=error) |
| FM-DASH-MON-002 | CRITICAL | System SHALL update vehicle locations at 1 Hz when vehicle is in motion | • Map updates every 1 second<br>• Vehicle icon position accuracy ±0.5m on map<br>• Smooth animation between position updates |
| FM-DASH-MON-003 | HIGH | System SHALL display vehicle battery status with visual indicators | • Battery percentage shown (0-100%)<br>• Warning at <30% (yellow), critical at <15% (red)<br>• Charging indicator when vehicle is docking |
| FM-DASH-MON-004 | HIGH | System SHALL show vehicle state transitions in real-time | • States: idle, navigating, docking, charging, error, emergency_stop<br>• Transition timestamp displayed<br>• State history log (last 10 transitions) |
| FM-DASH-MON-005 | MEDIUM | System SHALL provide vehicle health summary dashboard | • Uptime percentage (last 24h, 7d, 30d)<br>• Mission success rate<br>• Average mission duration<br>• Maintenance alerts |
| FM-DASH-MON-006 | HIGH | System SHALL alert operators when vehicle enters error state | • Desktop notification + sound alert<br>• Error details displayed (code, severity, description)<br>• Acknowledge button to clear alert<br>• Escalation if not acknowledged within 5 minutes |
| FM-DASH-MON-007 | CRITICAL | System SHALL support emergency stop command from dashboard | • Prominent E-STOP button (red, requires confirmation)<br>• Command sent via WebSocket within 500ms<br>• Vehicle acknowledgment displayed<br>• Logged to audit trail |
| FM-DASH-MON-008 | MEDIUM | System SHALL display active mission details for each vehicle | • Mission ID, pickup/destination waypoints<br>• Progress percentage<br>• Estimated time to completion<br>• Passenger status (if applicable) |

#### 3.1.2 Interactive Map Visualization

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-DASH-MAP-001 | CRITICAL | System SHALL display interactive facility map with vehicle positions | • 2D floor plan overlay<br>• Vehicle icons at current GPS coordinates<br>• Zoom (100-500%), pan controls<br>• Multi-floor support |
| FM-DASH-MAP-002 | HIGH | System SHALL show vehicle heading/orientation on map | • Arrow indicator showing heading direction<br>• Rotation updates in real-time<br>• Different icons for different vehicle states |
| FM-DASH-MAP-003 | HIGH | System SHALL display waypoints and navigation paths on map | • Named waypoints (entrances, rooms, charging stations)<br>• Active mission path shown as line<br>• Clickable waypoints for info |
| FM-DASH-MAP-004 | MEDIUM | System SHALL highlight vehicle when selected from vehicle list | • Selected vehicle icon highlighted (larger, pulsing border)<br>• Auto-center map on selected vehicle<br>• Info panel shows detailed vehicle status |
| FM-DASH-MAP-005 | HIGH | System SHALL display obstacle/blocked path warnings on map | • Red zone overlay for blocked areas<br>• Vehicle route deviation alerts<br>• Alternative path suggestions |
| FM-DASH-MAP-006 | MEDIUM | System SHALL support custom map layers (toggle on/off) | • Layers: waypoints, charging stations, active missions, restricted areas<br>• Layer controls in map toolbar<br>• Layer state persisted in user preferences |
| FM-DASH-MAP-007 | LOW | System SHALL provide heatmap view of vehicle activity | • Historical path heatmap (last 24h, 7d)<br>• Color gradient (blue=low, red=high activity)<br>• Helps identify high-traffic areas |

#### 3.1.3 Dashboard Layout and Usability

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-DASH-UI-001 | HIGH | System SHALL provide responsive layout for PC (1920x1080) and tablet (1280x800) | • Automatic layout adjustment<br>• Touch-friendly controls (min 48px tap targets)<br>• No horizontal scrolling |
| FM-DASH-UI-002 | MEDIUM | System SHALL support dashboard customization | • Drag-and-drop widget placement<br>• Resizable panels<br>• Layout saved per user |
| FM-DASH-UI-003 | HIGH | System SHALL update dashboard widgets without full page reload | • WebSocket-based real-time updates<br>• Graceful degradation if WebSocket fails (fallback to polling)<br>• Update indicator (pulsing icon) |
| FM-DASH-UI-004 | MEDIUM | System SHALL provide keyboard shortcuts for common actions | • Ctrl+D: Dispatch mission<br>• Ctrl+R: Refresh dashboard<br>• Ctrl+F: Focus search<br>• Esc: Close modals |
| FM-DASH-UI-005 | LOW | System SHALL support dark mode theme | • Toggle in user settings<br>• High contrast for readability<br>• Preference saved |

---

### 3.2 Reservation System

#### 3.2.1 Reservation Creation

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-RES-CREATE-001 | CRITICAL | System SHALL allow caregivers to create walking assistance reservations | • Form: passenger name, pickup waypoint, destination waypoint, scheduled time<br>• Validation: all fields required, time ≥ current time + 10 minutes<br>• Confirmation displayed |
| FM-RES-CREATE-002 | CRITICAL | System SHALL allow nurses to create medicine delivery reservations | • Form: item description, pickup (pharmacy), destination (room), scheduled time, priority<br>• Priority levels: routine, urgent, emergency<br>• Emergency reservations skip queue |
| FM-RES-CREATE-003 | HIGH | System SHALL validate waypoint availability before accepting reservation | • Check waypoint exists in facility map<br>• Check waypoint not in restricted area<br>• Check waypoint accessible (not blocked) |
| FM-RES-CREATE-004 | HIGH | System SHALL estimate mission duration when creating reservation | • Duration = path distance / avg speed + docking time<br>• Display estimated completion time<br>• Update if path changes |
| FM-RES-CREATE-005 | MEDIUM | System SHALL support recurring reservations | • Daily, weekly patterns<br>• End date or occurrence count<br>• Edit/delete single or all occurrences |
| FM-RES-CREATE-006 | HIGH | System SHALL assign unique reservation ID upon creation | • Format: RES-YYYYMMDD-NNNN<br>• Sequential numbering<br>• Displayed in confirmation |
| FM-RES-CREATE-007 | MEDIUM | System SHALL send confirmation notification to requester | • Email/in-app notification<br>• Includes: reservation ID, time, waypoints, estimated duration<br>• Link to view/cancel |

#### 3.2.2 Reservation Queue Management

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-RES-QUEUE-001 | CRITICAL | System SHALL maintain priority queue of pending reservations | • Priority order: emergency > urgent > routine > scheduled time<br>• FIFO within same priority<br>• Queue visible to operators |
| FM-RES-QUEUE-002 | HIGH | System SHALL automatically dispatch vehicles for scheduled reservations | • Vehicle dispatched 5 minutes before scheduled time<br>• Selects nearest available vehicle<br>• Updates reservation status to "dispatched" |
| FM-RES-QUEUE-003 | HIGH | System SHALL handle vehicle unavailability gracefully | • If no vehicle available, queue reservation<br>• Notify requester of delay<br>• Assign vehicle as soon as one becomes idle |
| FM-RES-QUEUE-004 | MEDIUM | System SHALL optimize vehicle assignments for multiple reservations | • Batch nearby reservations to same vehicle<br>• Minimize total travel distance<br>• Respect time constraints |
| FM-RES-QUEUE-005 | HIGH | System SHALL update queue in real-time as vehicles complete missions | • Queue position recalculated<br>• ETA updates sent to waiting requesters<br>• WebSocket notifications |
| FM-RES-QUEUE-006 | MEDIUM | System SHALL allow operators to manually reorder queue | • Drag-and-drop interface<br>• Requires admin/operator role<br>• Logged to audit trail |

#### 3.2.3 Reservation Tracking and History

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-RES-TRACK-001 | HIGH | System SHALL display reservation status in real-time | • States: pending, queued, dispatched, in_progress, completed, cancelled<br>• Progress indicator (0-100%)<br>• Current location of assigned vehicle |
| FM-RES-TRACK-002 | MEDIUM | System SHALL provide reservation search and filtering | • Search by: reservation ID, requester, date range, status<br>• Multi-select filters<br>• Results sorted by time (newest first) |
| FM-RES-TRACK-003 | HIGH | System SHALL allow reservation cancellation before dispatch | • Cancel button (requires confirmation)<br>• Notification sent to requester<br>• Cannot cancel after vehicle dispatched |
| FM-RES-TRACK-004 | MEDIUM | System SHALL maintain complete reservation history | • All reservations stored (minimum 1 year)<br>• Includes: created_at, updated_at, completed_at, requester, vehicle_id, status_transitions<br>• Exportable to CSV |
| FM-RES-TRACK-005 | LOW | System SHALL generate reservation statistics | • Daily/weekly/monthly counts<br>• Avg duration, success rate<br>• Peak usage hours |

---

### 3.3 User Role Management

#### 3.3.1 Role Definitions

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-ROLE-DEF-001 | CRITICAL | System SHALL implement three user roles: Admin, Caregiver, Nurse | • Roles stored in database<br>• Each user assigned exactly one role<br>• Role-based UI elements (hide/show based on role) |
| FM-ROLE-DEF-002 | CRITICAL | Admin role SHALL have full system access | • Create/edit/delete users<br>• View all reservations and missions<br>• Access analytics and reports<br>• System configuration |
| FM-ROLE-DEF-003 | HIGH | Caregiver role SHALL access walking assistance features | • Create walking assistance reservations<br>• View own reservations<br>• Track assigned missions<br>• Voice communication with vehicle |
| FM-ROLE-DEF-004 | HIGH | Nurse role SHALL access medicine delivery features | • Create medicine delivery reservations<br>• Set priority (routine, urgent, emergency)<br>• View own reservations<br>• View vehicle locations (read-only) |
| FM-ROLE-DEF-005 | MEDIUM | System SHALL support role-based permissions matrix | • Permissions: view_dashboard, create_reservation, cancel_reservation, view_analytics, manage_users, emergency_stop<br>• Matrix stored in config file<br>• Enforced server-side |

#### 3.3.2 User Management (Admin Only)

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-ROLE-USER-001 | HIGH | Admin SHALL create new user accounts | • Form: username, email, password, role, full name<br>• Password requirements: ≥8 chars, 1 uppercase, 1 number, 1 special<br>• Email verification optional |
| FM-ROLE-USER-002 | HIGH | Admin SHALL edit existing user accounts | • Change: email, role, full name, active status<br>• Cannot change username (unique ID)<br>• Cannot edit own admin role |
| FM-ROLE-USER-003 | MEDIUM | Admin SHALL deactivate/reactivate user accounts | • Deactivated users cannot log in<br>• Data preserved (not deleted)<br>• Audit log entry |
| FM-ROLE-USER-004 | MEDIUM | Admin SHALL reset user passwords | • Generate temporary password<br>• Email sent to user<br>• Force password change on next login |
| FM-ROLE-USER-005 | HIGH | System SHALL enforce unique usernames and emails | • Validation on create/edit<br>• Error message if duplicate<br>• Case-insensitive check |

#### 3.3.3 Authentication and Authorization

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-ROLE-AUTH-001 | CRITICAL | System SHALL authenticate users via username/password | • Login form with validation<br>• JWT token issued on success (24h expiration)<br>• Token stored in httpOnly cookie |
| FM-ROLE-AUTH-002 | HIGH | System SHALL enforce role-based access control on all API endpoints | • Server validates JWT and role<br>• 403 Forbidden if insufficient permissions<br>• Client-side checks for UX (server is source of truth) |
| FM-ROLE-AUTH-003 | HIGH | System SHALL automatically refresh JWT tokens before expiration | • Refresh at 80% of token lifetime (19.2h)<br>• Transparent to user<br>• Logout if refresh fails |
| FM-ROLE-AUTH-004 | MEDIUM | System SHALL log all authentication events | • Login/logout timestamps<br>• Failed login attempts (rate limiting after 5 failures)<br>• IP address, user agent |
| FM-ROLE-AUTH-005 | MEDIUM | System SHALL support session timeout | • Configurable timeout (default 8 hours)<br>• Warning 5 minutes before timeout<br>• "Extend session" button |

---

### 3.4 Mission Management

#### 3.4.1 Mission Dispatch

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-MISSION-DISP-001 | CRITICAL | System SHALL dispatch missions to vehicles via WebSocket | • Command type: "dispatch"<br>• Payload: mission_id, pickup, destination, priority<br>• Vehicle acknowledgment within 2 seconds |
| FM-MISSION-DISP-002 | HIGH | System SHALL support manual mission dispatch from dashboard | • Operator selects: vehicle, pickup, destination<br>• Immediate dispatch (bypasses queue)<br>• Confirmation required |
| FM-MISSION-DISP-003 | HIGH | System SHALL validate mission before dispatch | • Vehicle must be in "idle" state<br>• Battery ≥ 20%<br>• No active errors<br>• Waypoints valid and reachable |
| FM-MISSION-DISP-004 | MEDIUM | System SHALL provide estimated mission duration before dispatch | • Based on: path distance, avg speed, docking time<br>• Display to operator<br>• Warn if duration > 10 minutes |
| FM-MISSION-DISP-005 | HIGH | System SHALL handle mission rejection by vehicle | • Reasons: low battery, obstacle blocked, error state<br>• Re-queue mission automatically<br>• Notify operator |

#### 3.4.2 Mission Monitoring

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-MISSION-MON-001 | CRITICAL | System SHALL track mission progress in real-time | • Phases: dispatched, navigating_to_pickup, docking_pickup, passenger_boarding, navigating_to_dest, docking_dest, passenger_alighting, completed<br>• Phase transitions logged<br>• Progress % displayed |
| FM-MISSION-MON-002 | HIGH | System SHALL display mission timeline | • Start time, phase transitions, completion time<br>• Actual vs. estimated duration<br>• Delay warnings (actual > estimated + 2 min) |
| FM-MISSION-MON-003 | HIGH | System SHALL allow mission cancellation during navigation | • Cancel button (admin/operator only)<br>• Sends "cancel" command via WebSocket<br>• Vehicle returns to idle state |
| FM-MISSION-MON-004 | MEDIUM | System SHALL alert if mission exceeds expected duration by 50% | • Desktop notification + dashboard alert<br>• Operator can: cancel mission, extend timeout, investigate<br>• Escalation to admin if not resolved |
| FM-MISSION-MON-005 | MEDIUM | System SHALL log all mission events to database | • Events: dispatched, started, waypoint_reached, completed, cancelled, failed<br>• Timestamp, vehicle_id, mission_id, event_data<br>• Queryable for analytics |

#### 3.4.3 Mission History and Analytics

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-MISSION-HIST-001 | MEDIUM | System SHALL maintain mission history for minimum 1 year | • All missions stored in database<br>• Includes: mission_id, vehicle_id, pickup, destination, start_time, end_time, status, requester<br>• Exportable to CSV/JSON |
| FM-MISSION-HIST-002 | MEDIUM | System SHALL provide mission search and filtering | • Search by: mission ID, vehicle ID, date range, status, requester<br>• Sort by: start time, duration, status<br>• Pagination (50 per page) |
| FM-MISSION-HIST-003 | LOW | System SHALL generate mission analytics dashboard | • Total missions (daily/weekly/monthly)<br>• Success rate %<br>• Avg duration<br>• Peak hours heatmap |
| FM-MISSION-HIST-004 | LOW | System SHALL identify frequently used routes | • Top 10 pickup-destination pairs<br>• Frequency count<br>• Avg duration per route |

---

### 3.5 Communication System

#### 3.5.1 Voice Communication (VoIP)

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-COMM-VOICE-001 | HIGH | System SHALL support voice communication between staff and vehicle passenger | • VoIP protocol (WebRTC recommended)<br>• Initiate call from dashboard<br>• Bidirectional audio |
| FM-COMM-VOICE-002 | HIGH | System SHALL display active calls in dashboard | • Call status: ringing, connected, ended<br>• Duration timer<br>• Mute/unmute controls |
| FM-COMM-VOICE-003 | MEDIUM | System SHALL support call recording (optional, configurable) | • Recording enabled per facility policy<br>• Stored in encrypted format<br>• Retention period configurable |
| FM-COMM-VOICE-004 | MEDIUM | System SHALL handle call failures gracefully | • Retry logic (3 attempts, 5s interval)<br>• Fallback to text alert<br>• Notify operator of failure |

#### 3.5.2 Alerts and Notifications

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-COMM-ALERT-001 | CRITICAL | System SHALL send emergency alerts to all online operators | • Alert types: vehicle error, passenger emergency button, system failure<br>• Desktop notification + sound<br>• Alert banner in dashboard (cannot be dismissed until acknowledged) |
| FM-COMM-ALERT-002 | HIGH | System SHALL support custom alert routing rules | • Route vehicle errors to maintenance team<br>• Route reservation requests to caregivers/nurses<br>• Admin can configure rules |
| FM-COMM-ALERT-003 | HIGH | System SHALL log all alerts to audit trail | • Timestamp, alert type, recipient(s), acknowledged_by, acknowledged_at<br>• Exportable to CSV<br>• Retention: 1 year |
| FM-COMM-ALERT-004 | MEDIUM | System SHALL support in-app notifications (non-urgent) | • Notification center icon with unread count<br>• List of recent notifications<br>• Mark as read/unread |
| FM-COMM-ALERT-005 | LOW | System SHALL support email notifications (configurable per user) | • User preferences: email on reservation confirmed, mission completed, vehicle error<br>• Email template with mission details<br>• Unsubscribe link |

---

### 3.6 Reporting and Analytics

#### 3.6.1 Standard Reports

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-REPORT-STD-001 | MEDIUM | System SHALL generate daily fleet utilization report | • Report includes: total missions, total km traveled, avg mission duration, vehicle uptime %<br>• Auto-generated at 23:59 daily<br>• Emailed to admins |
| FM-REPORT-STD-002 | MEDIUM | System SHALL generate weekly maintenance report | • Report includes: battery cycles, error count by type, low battery incidents<br>• Auto-generated Sunday 23:59<br>• Highlights vehicles needing maintenance |
| FM-REPORT-STD-003 | LOW | System SHALL generate monthly usage summary | • Missions per user role (caregiver vs nurse)<br>• Peak usage hours<br>• Top routes<br>• Cost estimate (km × rate) |

#### 3.6.2 Custom Analytics

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-REPORT-CUSTOM-001 | LOW | System SHALL provide interactive analytics dashboard | • Charts: line (missions over time), bar (missions by vehicle), pie (mission status)<br>• Date range selector<br>• Export to PNG/PDF |
| FM-REPORT-CUSTOM-002 | LOW | System SHALL support custom report builder | • Select metrics: mission count, avg duration, distance, battery usage<br>• Group by: vehicle, user, time period<br>• Filter by: date range, status, priority |

---

## 4. Non-Functional Requirements

### 4.1 Performance

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-PERF-001 | HIGH | System SHALL support up to 10 concurrent vehicles | • Real-time tracking for all vehicles<br>• Map updates <1s latency<br>• No performance degradation |
| FM-PERF-002 | HIGH | System SHALL handle up to 50 concurrent user sessions | • Login, dashboard load, API calls responsive<br>• WebSocket connections stable<br>• No memory leaks |
| FM-PERF-003 | MEDIUM | Dashboard page SHALL load within 3 seconds on 10 Mbps connection | • Initial page load <3s<br>• Time to interactive <5s<br>• Measured with Lighthouse |
| FM-PERF-004 | MEDIUM | API response time SHALL be <500ms for 95% of requests | • Measured at server<br>• Excludes WebSocket streaming<br>• Monitored with APM tool |

### 4.2 Reliability and Availability

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-REL-001 | HIGH | System SHALL have 99.5% uptime during operational hours (8:00-20:00) | • Max 36 minutes downtime per month<br>• Planned maintenance outside operational hours<br>• Monitored with uptime service |
| FM-REL-002 | HIGH | System SHALL gracefully handle WebSocket disconnections | • Auto-reconnect with exponential backoff (1s, 2s, 4s, ...)<br>• Queue updates during disconnect, sync on reconnect<br>• User notified if disconnect >10s |
| FM-REL-003 | MEDIUM | System SHALL backup database daily | • Automated backup at 02:00 daily<br>• Retention: 30 days<br>• Tested restore procedure monthly |

### 4.3 Security

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-SEC-001 | CRITICAL | System SHALL use HTTPS for all client-server communication | • TLS 1.2+ with strong ciphers<br>• Valid SSL certificate<br>• HTTP redirects to HTTPS |
| FM-SEC-002 | CRITICAL | System SHALL store passwords using bcrypt (cost factor ≥12) | • Never store plaintext passwords<br>• Salt unique per user<br>• Migration plan for algorithm updates |
| FM-SEC-003 | HIGH | System SHALL implement CSRF protection | • CSRF token in all forms<br>• SameSite cookie attribute<br>• Reject requests without valid token |
| FM-SEC-004 | HIGH | System SHALL sanitize all user inputs to prevent XSS | • Input validation server-side<br>• Output encoding (HTML entities)<br>• Content Security Policy headers |
| FM-SEC-005 | MEDIUM | System SHALL log security events (failed logins, permission denied, etc.) | • Events logged with timestamp, user, IP, action<br>• Monitored for anomalies<br>• Alerts on suspicious patterns |

### 4.4 Usability

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-UX-001 | HIGH | System SHALL support Japanese and English languages | • Language selector in user settings<br>• All UI text translated (no hardcoded strings)<br>• Date/time formatting per locale |
| FM-UX-002 | MEDIUM | System SHALL provide tooltips for all dashboard controls | • Hover tooltip on buttons, icons<br>• Keyboard accessible (focus → tooltip)<br>• Translated in both languages |
| FM-UX-003 | MEDIUM | System SHALL maintain UI state across sessions | • Dashboard layout, selected filters, language preference<br>• Stored in browser localStorage<br>• Synced to server for cross-device |
| FM-UX-004 | LOW | System SHALL support accessibility standards (WCAG 2.1 Level AA) | • Keyboard navigation (all features)<br>• Screen reader compatible (ARIA labels)<br>• Sufficient color contrast (4.5:1) |

### 4.5 Compatibility

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-COMPAT-001 | HIGH | System SHALL support modern web browsers | • Chrome 100+, Firefox 100+, Safari 15+, Edge 100+<br>• Tested on each browser<br>• Graceful degradation for older browsers |
| FM-COMPAT-002 | MEDIUM | System SHALL support tablet devices (iPad, Android tablets) | • Touch-friendly controls (min 48px tap targets)<br>• Responsive layout (portrait/landscape)<br>• Tested on iPad Air, Samsung Galaxy Tab |

---

## 5. Interface Requirements

### 5.1 TVM Server API Integration

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-INT-API-001 | CRITICAL | System SHALL consume vehicle telemetry via TVM Server API | • REST endpoints as defined in TVM_API_SPECIFICATION.md<br>• WebSocket connection for real-time updates<br>• Handle API errors gracefully |
| FM-INT-API-002 | CRITICAL | System SHALL send vehicle commands via TVM Server API | • Commands: dispatch, cancel, emergency_stop, configure<br>• WebSocket protocol as defined in specification<br>• Retry failed commands (max 3 attempts) |
| FM-INT-API-003 | HIGH | System SHALL validate all API requests/responses against JSON schemas | • Use schemas from TVM_DATA_MODELS.md<br>• Client-side validation before send<br>• Server response validation |

---

## 6. Data Requirements

### 6.1 Data Storage

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-DATA-STORE-001 | HIGH | System SHALL store user accounts, reservations, missions, vehicles in relational database | • Recommended: PostgreSQL 14+<br>• Schema versioning with migrations<br>• Foreign key constraints enforced |
| FM-DATA-STORE-002 | MEDIUM | System SHALL store real-time vehicle state in cache (optional) | • Redis/Memcached for fast read access<br>• Synchronized with database<br>• TTL 60 seconds |

### 6.2 Data Retention

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-DATA-RET-001 | MEDIUM | System SHALL retain mission history for 1 year | • Auto-archive missions older than 1 year<br>• Archived data exportable<br>• Configurable retention period |
| FM-DATA-RET-002 | MEDIUM | System SHALL retain audit logs for 90 days | • Login events, permission changes, mission edits<br>• Compressed after 30 days<br>• Exportable to SIEM system |

---

## 7. Operational Requirements

### 7.1 Deployment

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-OPS-DEPLOY-001 | MEDIUM | System SHALL support containerized deployment (Docker) | • Dockerfile provided<br>• Docker Compose for multi-container setup<br>• Environment variables for configuration |
| FM-OPS-DEPLOY-002 | MEDIUM | System SHALL provide installation and configuration guide | • Step-by-step setup instructions<br>• Configuration file examples<br>• Troubleshooting section |

### 7.2 Monitoring

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| FM-OPS-MON-001 | MEDIUM | System SHALL expose health check endpoint | • GET /health returns 200 if healthy<br>• Includes: database connection, API connectivity, WebSocket status<br>• Used by load balancer |
| FM-OPS-MON-002 | LOW | System SHALL log application errors to centralized logging system | • Structured logs (JSON format)<br>• Log levels: DEBUG, INFO, WARN, ERROR<br>• Forwarded to ELK/Splunk |

---

## 8. Acceptance Criteria

### 8.1 Testing Requirements

| Test Type | Coverage | Acceptance |
|-----------|----------|------------|
| Unit Tests | ≥80% code coverage | All tests pass |
| Integration Tests | API endpoints, database operations | All critical paths tested |
| UI Tests | Dashboard, reservation flow, user management | Key workflows automated (Selenium/Cypress) |
| Performance Tests | Load testing (50 users, 10 vehicles) | Meets performance requirements |
| Security Tests | OWASP Top 10 vulnerabilities | No critical/high vulnerabilities |
| Accessibility Tests | WCAG 2.1 Level AA | No major violations |

### 8.2 User Acceptance Testing

| Scenario | User Role | Pass Criteria |
|----------|-----------|---------------|
| Create walking assistance reservation | Caregiver | Reservation created, vehicle dispatched on time, mission completed |
| Create urgent medicine delivery | Nurse | Reservation prioritized, vehicle assigned immediately |
| Monitor fleet in real-time | Admin | All vehicles visible on map, status updates <2s latency |
| Emergency stop vehicle | Admin | Vehicle stops within 500ms, status updated in dashboard |
| Search mission history | Admin | Search filters work, results accurate |

---

## 9. Traceability Matrix

| Requirement Category | Requirement Count | Priority Distribution |
|---------------------|-------------------|----------------------|
| Fleet Dashboard | 21 | CRITICAL: 4, HIGH: 11, MEDIUM: 5, LOW: 1 |
| Reservation System | 19 | CRITICAL: 4, HIGH: 10, MEDIUM: 4, LOW: 1 |
| User Role Management | 14 | CRITICAL: 3, HIGH: 6, MEDIUM: 5 |
| Mission Management | 14 | CRITICAL: 3, HIGH: 7, MEDIUM: 4 |
| Communication System | 9 | CRITICAL: 1, HIGH: 4, MEDIUM: 3, LOW: 1 |
| Reporting and Analytics | 6 | MEDIUM: 3, LOW: 3 |
| Non-Functional (Performance) | 4 | HIGH: 2, MEDIUM: 2 |
| Non-Functional (Reliability) | 3 | HIGH: 2, MEDIUM: 1 |
| Non-Functional (Security) | 5 | CRITICAL: 2, HIGH: 2, MEDIUM: 1 |
| Non-Functional (Usability) | 4 | HIGH: 1, MEDIUM: 2, LOW: 1 |
| Non-Functional (Compatibility) | 2 | HIGH: 1, MEDIUM: 1 |
| Interface Requirements | 3 | CRITICAL: 2, HIGH: 1 |
| Data Requirements | 3 | HIGH: 1, MEDIUM: 2 |
| Operational Requirements | 3 | MEDIUM: 3, LOW: 1 |
| **TOTAL** | **110** | **CRITICAL: 19, HIGH: 45, MEDIUM: 36, LOW: 10** |

---

## 10. Assumptions and Dependencies

### 10.1 Assumptions

1. Facility has reliable Wi-Fi coverage for dashboard access
2. Users have basic computer/tablet proficiency
3. Facility map is provided in CAD/PNG format
4. Waypoint names standardized across facility

### 10.2 Dependencies

1. TVM Server backend implementation (TVM_SERVER_REQUIREMENTS.md)
2. Vehicle-side TVM client implementation (VEHICLE_TVM_CLIENT_REQUIREMENTS.md)
3. TVM API specification stable (TVM_API_SPECIFICATION.md)
4. Database server (PostgreSQL) available
5. SSL certificate for HTTPS

---

## 11. Open Issues and Risks

### 11.1 Open Issues

| ID | Issue | Owner | Target Resolution |
|----|-------|-------|-------------------|
| FM-ISSUE-001 | Map data format not finalized | Facility Ops | Week 3 |
| FM-ISSUE-002 | VoIP provider selection pending | Unno | Week 4 |
| FM-ISSUE-003 | Email server configuration TBD | IT Team | Week 5 |

### 11.2 Risks

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| Map data unavailable | LOW | HIGH | Use generic facility layout, add custom waypoints manually |
| WebSocket scalability issues | MEDIUM | MEDIUM | Load test early, consider message queue (Redis Pub/Sub) |
| User role requirements change | MEDIUM | LOW | Design flexible RBAC system, easy to add new roles |

---

## 12. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-16 | Unno | Initial draft - Week 2 documentation |

---

**END OF DOCUMENT**

**Total Requirements:** 110
**Document Status:** Draft for Review
**Next Review:** Week 3 (with Pankaj for integration alignment)
