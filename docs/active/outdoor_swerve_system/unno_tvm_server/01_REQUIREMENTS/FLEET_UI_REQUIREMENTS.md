# Fleet Management UI Requirements

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Document Type:** Fleet Management Requirements Specification
**Team Responsibility:** Unno (Fleet Management)
**Status:** Week 4 - Active Development
**Date:** December 16, 2025
**Version:** 1.0

---

## Document Control

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-16 | Unno Team | Initial fleet management UI requirements |

**Related Documents:**
- FLEET_MANAGEMENT_REQUIREMENTS.md (backend logic, mission management)
- TVM_SERVER_REQUIREMENTS.md (server architecture)
- USER_ROLE_MANAGEMENT_REQUIREMENTS.md (authentication, RBAC)
- TVM_API_SPECIFICATION.md (REST endpoints, WebSocket)
- RESERVATION_SYSTEM_REQUIREMENTS.md (booking UI integration)

---

## 1. Purpose and Scope

This document specifies requirements for the **Fleet Management Web UI**, a browser-based dashboard for monitoring and controlling the wheelchair transport robot fleet. The UI serves multiple user roles (Admin, Operator, Nurse, Caregiver) with different access levels.

**Design Principles:**
- **Responsive design** (desktop, tablet, mobile)
- **Real-time updates** (WebSocket for live telemetry)
- **Role-based views** (different dashboards for different users)
- **Accessibility** (WCAG 2.1 AA compliance for elderly users)

**Technology Stack:**
- Frontend: React 18 + TypeScript
- State Management: Redux Toolkit + RTK Query
- UI Framework: Material-UI (MUI) v5
- Real-time: Socket.io-client
- Maps: Leaflet.js for 2D map visualization

---

## 2. Dashboard and Overview Requirements

### 2.1 Main Dashboard

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-DASH-001 | System SHALL display fleet overview dashboard showing all vehicles on a map | CRITICAL | All vehicles visible on map |
| FLEET-UI-DASH-002 | Dashboard SHALL update vehicle positions in real-time (<5s latency) | CRITICAL | Position updates within 5s |
| FLEET-UI-DASH-003 | Dashboard SHALL display vehicle status: idle, navigating, docking, transporting, charging, error | CRITICAL | Status displayed with color coding |
| FLEET-UI-DASH-004 | Dashboard SHALL show fleet statistics: total vehicles, active missions, completed today, errors | HIGH | Statistics update in real-time |
| FLEET-UI-DASH-005 | Dashboard SHALL display battery levels for all vehicles with color warnings (<30% red, <50% yellow) | CRITICAL | Battery warnings displayed |
| FLEET-UI-DASH-006 | Dashboard SHALL support filtering vehicles by status (show only idle, only active, etc.) | MEDIUM | Filtering functional |
| FLEET-UI-DASH-007 | Dashboard SHALL display map layers: roads, buildings, POIs, geofences, charging stations | HIGH | Map layers toggle available |
| FLEET-UI-DASH-008 | Dashboard SHALL support zoom and pan on map (mouse wheel, touch gestures) | HIGH | Map navigation smooth |

**Success Criteria:** Dashboard loads within 2 seconds, real-time updates <5s latency

### 2.2 Vehicle Detail View

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-DETAIL-001 | Clicking a vehicle marker SHALL open detailed vehicle info panel | CRITICAL | Panel opens with vehicle data |
| FLEET-UI-DETAIL-002 | Vehicle detail panel SHALL display: name, battery %, speed, current task, ETA, errors | CRITICAL | All fields populated |
| FLEET-UI-DETAIL-003 | Vehicle detail panel SHALL show current navigation path on map (as colored line) | HIGH | Path displayed on map |
| FLEET-UI-DETAIL-004 | Vehicle detail panel SHALL display sensor status: LiDAR, cameras, IMU, GPS (green=OK, red=fail) | HIGH | Sensor status indicators shown |
| FLEET-UI-DETAIL-005 | Vehicle detail panel SHALL show recent activity log (last 10 events) | MEDIUM | Activity log displayed |
| FLEET-UI-DETAIL-006 | Vehicle detail panel SHALL provide quick actions: send to charging, cancel mission, emergency stop | CRITICAL | Quick actions functional |

---

## 3. Mission Management UI Requirements

### 3.1 Mission Creation

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-MISSION-001 | System SHALL provide "Create Mission" button to open mission creation wizard | CRITICAL | Button opens wizard |
| FLEET-UI-MISSION-002 | Mission wizard SHALL allow selecting mission type: walking_assistance or medicine_delivery | CRITICAL | Mission type selectable |
| FLEET-UI-MISSION-003 | Mission wizard SHALL allow clicking map to set pickup and dropoff locations | CRITICAL | Map click sets locations |
| FLEET-UI-MISSION-004 | Mission wizard SHALL display estimated travel time and distance | HIGH | ETA calculated and shown |
| FLEET-UI-MISSION-005 | Mission wizard SHALL allow selecting priority: routine, urgent, emergency | CRITICAL | Priority level selectable |
| FLEET-UI-MISSION-006 | Mission wizard SHALL allow scheduling mission for future time | MEDIUM | Scheduled missions supported |
| FLEET-UI-MISSION-007 | Mission wizard SHALL validate locations (must be within operational area) | CRITICAL | Invalid locations rejected |
| FLEET-UI-MISSION-008 | Mission wizard SHALL show available vehicles and auto-assign optimal vehicle | HIGH | Auto-assignment functional |

### 3.2 Mission Monitoring

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-MISSION-MON-001 | System SHALL display active missions list with status: pending, in_progress, completed, cancelled | CRITICAL | Missions list displayed |
| FLEET-UI-MISSION-MON-002 | Active missions SHALL show progress bar (e.g., "En route to pickup - 30% complete") | HIGH | Progress bar displayed |
| FLEET-UI-MISSION-MON-003 | System SHALL highlight vehicle and path on map when mission selected from list | HIGH | Selection highlights on map |
| FLEET-UI-MISSION-MON-004 | System SHALL allow cancelling active mission (confirmation dialog required) | CRITICAL | Mission cancellation functional |
| FLEET-UI-MISSION-MON-005 | System SHALL display ETA and distance remaining for active missions | HIGH | ETA and distance shown |
| FLEET-UI-MISSION-MON-006 | System SHALL alert operator on mission failure (visual + audio notification) | CRITICAL | Failure alerts displayed |

### 3.3 Mission History

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-MISSION-HIST-001 | System SHALL provide mission history view with date range filter | MEDIUM | History view with filters |
| FLEET-UI-MISSION-HIST-002 | Mission history SHALL display: mission ID, date/time, type, vehicle, pickup/dropoff, duration, outcome | MEDIUM | All fields in history |
| FLEET-UI-MISSION-HIST-003 | Mission history SHALL support searching by vehicle ID, date, mission type | LOW | Search functional |
| FLEET-UI-MISSION-HIST-004 | Mission history SHALL allow exporting to CSV for reporting | LOW | CSV export functional |

---

## 4. Reservation System UI Requirements

### 4.1 Reservation Calendar View

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-RESV-001 | System SHALL display reservation calendar showing daily, weekly, monthly views | HIGH | Calendar views available |
| FLEET-UI-RESV-002 | Calendar SHALL show reservations color-coded by type: walking (blue), medicine (green) | HIGH | Color coding applied |
| FLEET-UI-RESV-003 | Calendar SHALL display recurring reservations with repeat icon | MEDIUM | Recurring icon shown |
| FLEET-UI-RESV-004 | Clicking calendar slot SHALL open reservation creation form | HIGH | Form opens on click |

### 4.2 Reservation Management

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-RESV-MGT-001 | Reservation form SHALL include fields: patient name, type, pickup/dropoff, date/time, priority, recurring pattern | CRITICAL | All fields in form |
| FLEET-UI-RESV-MGT-002 | System SHALL validate reservation conflicts (same patient at same time) | CRITICAL | Conflicts detected and flagged |
| FLEET-UI-RESV-MGT-003 | System SHALL allow modifying existing reservations (with confirmation) | HIGH | Modification functional |
| FLEET-UI-RESV-MGT-004 | System SHALL allow cancelling reservations (cancel single or series for recurring) | HIGH | Cancellation functional |
| FLEET-UI-RESV-MGT-005 | System SHALL notify affected users on reservation changes via email/SMS | MEDIUM | Notifications sent |

---

## 5. Vehicle Management UI Requirements

### 5.1 Vehicle Fleet List

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-VEHICLE-001 | System SHALL display vehicle list table with columns: ID, name, status, battery, location, last seen | CRITICAL | Vehicle list displayed |
| FLEET-UI-VEHICLE-002 | Vehicle list SHALL allow sorting by any column (ascending/descending) | MEDIUM | Sorting functional |
| FLEET-UI-VEHICLE-003 | Vehicle list SHALL allow filtering by status: online, offline, error, charging | HIGH | Filtering functional |
| FLEET-UI-VEHICLE-004 | Vehicle list SHALL display offline vehicles in gray with "Last seen X min ago" | HIGH | Offline status clear |

### 5.2 Vehicle Configuration

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-VEHICLE-CFG-001 | Admin SHALL be able to add new vehicle to fleet (enter vehicle ID, name, capabilities) | HIGH | Add vehicle functional |
| FLEET-UI-VEHICLE-CFG-002 | Admin SHALL be able to edit vehicle settings: name, operational area, max speed, charging schedule | MEDIUM | Edit settings functional |
| FLEET-UI-VEHICLE-CFG-003 | Admin SHALL be able to mark vehicle as out-of-service (exclude from mission assignment) | HIGH | Out-of-service status functional |
| FLEET-UI-VEHICLE-CFG-004 | Admin SHALL be able to delete vehicle from fleet (confirmation required) | LOW | Delete functional with confirmation |

### 5.3 Vehicle Diagnostics

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-VEHICLE-DIAG-001 | System SHALL display vehicle diagnostics page with sensor health, error codes, logs | HIGH | Diagnostics page displayed |
| FLEET-UI-VEHICLE-DIAG-002 | Diagnostics SHALL show error codes with severity: info, warning, error, critical | HIGH | Error codes color-coded |
| FLEET-UI-VEHICLE-DIAG-003 | Diagnostics SHALL display recent logs (last 100 entries) with timestamp, level, message | MEDIUM | Logs displayed |
| FLEET-UI-VEHICLE-DIAG-004 | Diagnostics SHALL allow downloading full logs as ZIP file | LOW | Log download functional |

---

## 6. User Management UI Requirements

### 6.1 User List and Roles

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-USER-001 | Admin SHALL access user management page showing all users with columns: name, email, role, status | HIGH | User list displayed |
| FLEET-UI-USER-002 | Admin SHALL be able to create new user account (enter name, email, role, initial password) | HIGH | User creation functional |
| FLEET-UI-USER-003 | Admin SHALL be able to edit user role: Admin, Operator, Nurse, Caregiver | HIGH | Role editing functional |
| FLEET-UI-USER-004 | Admin SHALL be able to deactivate user account (disable login, preserve data) | HIGH | Deactivation functional |
| FLEET-UI-USER-005 | Admin SHALL be able to reset user password (send reset link via email) | MEDIUM | Password reset functional |

### 6.2 User Activity Log

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-USER-AUDIT-001 | Admin SHALL access audit log showing user actions: login, logout, mission created, settings changed | HIGH | Audit log displayed |
| FLEET-UI-USER-AUDIT-002 | Audit log SHALL include: timestamp, user, action, IP address, result (success/failure) | HIGH | All fields in audit log |
| FLEET-UI-USER-AUDIT-003 | Audit log SHALL support filtering by user, action type, date range | MEDIUM | Filtering functional |

---

## 7. Alerts and Notifications UI Requirements

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-ALERT-001 | System SHALL display notifications panel showing recent alerts (last 20) | CRITICAL | Notifications panel displayed |
| FLEET-UI-ALERT-002 | Alerts SHALL be categorized: info (blue), warning (yellow), error (red), critical (flashing red) | CRITICAL | Color coding applied |
| FLEET-UI-ALERT-003 | Critical alerts SHALL trigger browser notification (if permission granted) and audio beep | CRITICAL | Browser notifications triggered |
| FLEET-UI-ALERT-004 | System SHALL display alert count badge on notification icon (e.g., "🔔 5") | HIGH | Badge count displayed |
| FLEET-UI-ALERT-005 | Clicking alert SHALL navigate to relevant page (e.g., vehicle detail, mission detail) | HIGH | Navigation functional |
| FLEET-UI-ALERT-006 | Operator SHALL be able to acknowledge/dismiss alerts | MEDIUM | Acknowledge action functional |
| FLEET-UI-ALERT-007 | System SHALL support alert filtering by severity and category | LOW | Filtering functional |

**Alert Types:**
- Vehicle offline (>2 minutes)
- Battery low (<30%)
- Mission failed (navigation, docking, timeout)
- Emergency stop pressed
- Sensor failure detected
- Geofence violation

---

## 8. Settings and Configuration UI Requirements

### 8.1 System Settings

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-SETTINGS-001 | Admin SHALL access system settings page with sections: general, map, notifications, integrations | MEDIUM | Settings page accessible |
| FLEET-UI-SETTINGS-002 | General settings SHALL include: system name, timezone, language (EN/JP), theme (light/dark) | MEDIUM | Settings editable |
| FLEET-UI-SETTINGS-003 | Map settings SHALL include: default zoom level, map provider (OpenStreetMap/Google), center coordinates | LOW | Map settings editable |
| FLEET-UI-SETTINGS-004 | Notification settings SHALL include: enable email alerts, enable SMS alerts, alert thresholds | MEDIUM | Notification settings editable |

### 8.2 Geofence Management

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-GEOFENCE-001 | Admin SHALL be able to draw geofences on map (polygon tool) | HIGH | Geofence drawing functional |
| FLEET-UI-GEOFENCE-002 | Geofences SHALL be categorized: operational_area (allowed), keep_out (forbidden) | HIGH | Geofence types supported |
| FLEET-UI-GEOFENCE-003 | System SHALL display all geofences on map with labels | HIGH | Geofences displayed |
| FLEET-UI-GEOFENCE-004 | Admin SHALL be able to edit/delete existing geofences | MEDIUM | Editing functional |

---

## 9. Analytics and Reporting UI Requirements

### 9.1 Fleet Analytics Dashboard

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-ANALYTICS-001 | System SHALL provide analytics dashboard with time range selector (today, week, month, custom) | MEDIUM | Analytics dashboard displayed |
| FLEET-UI-ANALYTICS-002 | Dashboard SHALL display charts: missions per day, vehicle utilization %, average mission duration | MEDIUM | Charts rendered correctly |
| FLEET-UI-ANALYTICS-003 | Dashboard SHALL display KPIs: total missions completed, success rate %, total distance traveled | MEDIUM | KPIs calculated correctly |
| FLEET-UI-ANALYTICS-004 | Dashboard SHALL show vehicle efficiency: battery cycles, charging time, operational hours | LOW | Efficiency metrics shown |

### 9.2 Reporting

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-REPORT-001 | Admin SHALL be able to generate PDF reports: daily summary, weekly summary, monthly summary | LOW | PDF generation functional |
| FLEET-UI-REPORT-002 | Reports SHALL include: mission statistics, vehicle status, errors encountered, user activity | LOW | Report content comprehensive |
| FLEET-UI-REPORT-003 | System SHALL allow scheduling automatic report generation (email delivery) | LOW | Scheduled reports functional |

---

## 10. Mobile Responsive Design Requirements

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-MOBILE-001 | UI SHALL be fully responsive for desktop (1920×1080), tablet (1024×768), mobile (375×667) | HIGH | Responsive breakpoints functional |
| FLEET-UI-MOBILE-002 | Mobile view SHALL display simplified dashboard (vehicle count, map, mission list) | HIGH | Mobile dashboard optimized |
| FLEET-UI-MOBILE-003 | Mobile view SHALL support touch gestures: tap, swipe, pinch-zoom on map | HIGH | Touch gestures functional |
| FLEET-UI-MOBILE-004 | Mobile view SHALL hide non-critical UI elements (analytics, detailed logs) | MEDIUM | Mobile UI simplified |

---

## 11. Accessibility Requirements (WCAG 2.1 AA)

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-A11Y-001 | UI SHALL support keyboard navigation (tab, arrow keys, enter, esc) | HIGH | Keyboard navigation functional |
| FLEET-UI-A11Y-002 | UI SHALL provide sufficient color contrast (4.5:1 for text, 3:1 for UI components) | HIGH | Color contrast verified |
| FLEET-UI-A11Y-003 | UI SHALL include ARIA labels for screen readers on all interactive elements | MEDIUM | ARIA labels present |
| FLEET-UI-A11Y-004 | UI SHALL support font size adjustment (browser zoom 100%-200%) | MEDIUM | Zoom support verified |
| FLEET-UI-A11Y-005 | UI SHALL avoid flashing content (seizure risk for <3 flashes per second) | HIGH | No flashing content |

---

## 12. Performance Requirements

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-PERF-001 | Initial page load SHALL complete within 2 seconds on broadband (10 Mbps) | HIGH | Load time <2s |
| FLEET-UI-PERF-002 | Dashboard updates (vehicle positions) SHALL occur with <5s latency | CRITICAL | Update latency <5s |
| FLEET-UI-PERF-003 | UI SHALL remain responsive during high load (50+ vehicles, 100+ missions) | HIGH | No UI freezing |
| FLEET-UI-PERF-004 | Map rendering SHALL support 100+ vehicle markers without lag | HIGH | Map renders smoothly |
| FLEET-UI-PERF-005 | WebSocket reconnection SHALL occur automatically within 10s on disconnect | CRITICAL | Reconnection <10s |

---

## 13. Security Requirements

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-SEC-001 | UI SHALL require login (username/password or SSO) before accessing any page | CRITICAL | Login enforced |
| FLEET-UI-SEC-002 | UI SHALL enforce session timeout (30 minutes of inactivity) | HIGH | Session timeout functional |
| FLEET-UI-SEC-003 | UI SHALL display different views based on user role (RBAC enforcement) | CRITICAL | Role-based views verified |
| FLEET-UI-SEC-004 | UI SHALL sanitize all user inputs to prevent XSS attacks | CRITICAL | XSS protection verified |
| FLEET-UI-SEC-005 | UI SHALL use HTTPS for all communication (TLS 1.3) | CRITICAL | HTTPS enforced |
| FLEET-UI-SEC-006 | UI SHALL not store sensitive data (passwords, tokens) in browser localStorage | HIGH | Sensitive data in secure storage |

---

## 14. Internationalization (i18n) Requirements

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| FLEET-UI-I18N-001 | UI SHALL support English (EN) and Japanese (JA) languages | HIGH | Both languages available |
| FLEET-UI-I18N-002 | User SHALL be able to switch language via settings menu | HIGH | Language switch functional |
| FLEET-UI-I18N-003 | All UI text SHALL be externalized in translation files (i18next) | MEDIUM | Translation files used |
| FLEET-UI-I18N-004 | Date/time format SHALL adapt to selected language (EN: MM/DD/YYYY, JA: YYYY/MM/DD) | MEDIUM | Date formats localized |

---

## 15. Requirements Summary

| Category | Count | Priority Breakdown |
|----------|-------|-------------------|
| Dashboard and Overview | 14 | Critical: 9, High: 4, Medium: 1 |
| Mission Management | 19 | Critical: 9, High: 7, Medium: 2, Low: 1 |
| Reservation System | 9 | Critical: 3, High: 5, Medium: 1 |
| Vehicle Management | 13 | Critical: 1, High: 8, Medium: 3, Low: 1 |
| User Management | 8 | High: 6, Medium: 2 |
| Alerts and Notifications | 7 | Critical: 3, High: 3, Medium: 1, Low: 1 |
| Settings and Configuration | 8 | High: 3, Medium: 4, Low: 1 |
| Analytics and Reporting | 7 | Medium: 4, Low: 3 |
| Mobile Responsive Design | 4 | High: 3, Medium: 1 |
| Accessibility | 5 | High: 3, Medium: 2 |
| Performance | 5 | Critical: 2, High: 3 |
| Security | 6 | Critical: 4, High: 2 |
| Internationalization | 4 | High: 2, Medium: 2 |
| **TOTAL** | **109** | **Critical: 31, High: 48, Medium: 23, Low: 7** |

---

## 16. UI Wireframe References

**Main Dashboard Layout:**
```
┌────────────────────────────────────────────────────────────┐
│ [Logo] Fleet Management Dashboard    [🔔 5] [👤 Admin] [⚙️] │
├────────────────────────────────────────────────────────────┤
│  📊 Fleet Stats:  Total: 10  |  Active: 7  |  Charging: 2  │
│                   Battery Avg: 65%  |  Errors: 1           │
├──────────────────────────────┬─────────────────────────────┤
│                              │  🔴 VEH-03 - Battery Low    │
│        MAP VIEW              │  🟡 VEH-07 - Mission Delayed │
│    (Vehicle positions,       │  ⚪ VEH-01 - Charging       │
│     geofences, paths)        │                             │
│                              │  [Vehicle List] [Missions]  │
│  🚙 🚙 🚙                    │  [Reservations] [Users]     │
│                              │                             │
└──────────────────────────────┴─────────────────────────────┘
```

---

## 17. Traceability Matrix

| UI Requirement Category | Backend Requirements | API Endpoints |
|------------------------|---------------------|---------------|
| Dashboard | FLEET_MANAGEMENT_REQUIREMENTS.md | GET /api/v1/vehicles, WebSocket /vehicles/telemetry |
| Mission Management | FLEET_MANAGEMENT_REQUIREMENTS.md | POST /api/v1/missions, GET /missions |
| Reservation System | RESERVATION_SYSTEM_REQUIREMENTS.md | POST /api/v1/reservations, GET /reservations |
| User Management | USER_ROLE_MANAGEMENT_REQUIREMENTS.md | POST /api/v1/users, GET /users |

---

## 18. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-16 | Unno Team | Initial fleet management UI requirements (109 requirements) |

---

**Document End**
