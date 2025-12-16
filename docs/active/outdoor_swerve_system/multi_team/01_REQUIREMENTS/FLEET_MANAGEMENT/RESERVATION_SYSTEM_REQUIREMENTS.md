# Reservation System Requirements

**Document Control:**
- **Document ID:** FM-RES-REQ-001
- **Version:** 1.0
- **Date:** 2025-12-16
- **Status:** Draft
- **Owner:** Unno (Fleet Management Team)
- **Reviewers:** Pankaj (Integration), Facility Operations Team

---

## 1. Introduction

### 1.1 Purpose

This document specifies detailed requirements for the Reservation System, a core component of the Fleet Management System. The Reservation System enables facility staff (caregivers and nurses) to schedule and manage wheelchair transport missions for residents, including walking assistance and medicine delivery services.

### 1.2 Scope

**In Scope:**
- Reservation creation workflows (caregiver and nurse interfaces)
- Reservation scheduling and time slot management
- Priority queue management (routine, urgent, emergency)
- Reservation modification and cancellation
- Recurring reservation patterns
- Conflict detection and resolution
- Notification system (confirmation, reminders, status updates)
- Integration with vehicle dispatch system
- Reservation history and reporting

**Out of Scope:**
- Real-time vehicle tracking (covered in FLEET_MANAGEMENT_REQUIREMENTS.md)
- TVM Server backend implementation (covered in TVM_SERVER_REQUIREMENTS.md)
- User authentication and role management (covered in USER_ROLE_MANAGEMENT_REQUIREMENTS.md)

### 1.3 Definitions and Acronyms

| Term | Definition |
|------|------------|
| Reservation | Pre-scheduled transport mission request |
| Caregiver | 介護士 - Staff member requesting walking assistance |
| Nurse | 看護士 - Medical staff requesting medicine delivery |
| Priority | Urgency level (routine, urgent, emergency) |
| Time Slot | 15-minute booking window |
| Recurring Reservation | Repeating pattern (daily, weekly) |
| Conflict | Overlapping reservations for same vehicle/time |

### 1.4 References

- FLEET_MANAGEMENT_REQUIREMENTS.md - Parent document
- TVM_SERVER_REQUIREMENTS.md - Backend API
- USER_ROLE_MANAGEMENT_REQUIREMENTS.md - User roles and permissions
- Excel Use Cases - Caregiver and nurse workflows

---

## 2. System Overview

### 2.1 Use Cases

**Primary Use Cases:**
1. **Walking Assistance (Caregiver):**
   - Resident needs transportation from room to lobby
   - Caregiver books wheelchair robot for specific time
   - Robot picks up resident, transports safely, returns

2. **Medicine Delivery (Nurse):**
   - Pharmacy has prepared medication for patient in room 201
   - Nurse books robot for urgent delivery
   - Robot picks up medicine, delivers to patient room

3. **Scheduled Activities:**
   - Daily rehabilitation session at 10:00 AM
   - Recurring reservation every weekday
   - Automatic dispatch 5 minutes before scheduled time

### 2.2 Reservation Lifecycle

```
Create → Schedule → Queue → Dispatch → In Progress → Complete
   ↓         ↓         ↓         ↓
 Modify   Cancel   Conflict   Failed
```

---

## 3. Functional Requirements

### 3.1 Reservation Creation

#### 3.1.1 Walking Assistance Reservation (Caregiver)

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-CREATE-WALK-001 | CRITICAL | System SHALL provide reservation form for walking assistance | • Form fields: passenger name, pickup waypoint, destination waypoint, scheduled time, notes<br>• All fields required except notes<br>• Accessible from caregiver dashboard |
| RES-CREATE-WALK-002 | HIGH | System SHALL validate passenger name against resident database | • Autocomplete from resident list<br>• Prevents typos<br>• Shows resident photo (optional) |
| RES-CREATE-WALK-003 | HIGH | System SHALL validate waypoint selection | • Pickup/destination from dropdown (facility waypoints)<br>• Cannot be same waypoint<br>• Waypoint descriptions shown (e.g., "Room 201 - 2nd Floor East") |
| RES-CREATE-WALK-004 | CRITICAL | System SHALL enforce minimum advance booking time | • Scheduled time ≥ current time + 10 minutes<br>• Error message if too soon<br>• Exception: Emergency priority (immediate booking) |
| RES-CREATE-WALK-005 | MEDIUM | System SHALL suggest optimal time slots | • Show next 5 available slots<br>• Based on vehicle availability<br>• Green (available), yellow (limited), red (full) |
| RES-CREATE-WALK-006 | MEDIUM | System SHALL allow notes/special instructions | • Free text field (max 500 characters)<br>• Examples: "Passenger has mobility aid", "Return trip needed"<br>• Displayed to vehicle operator (if present) |

#### 3.1.2 Medicine Delivery Reservation (Nurse)

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-CREATE-MED-001 | CRITICAL | System SHALL provide reservation form for medicine delivery | • Form fields: item description, pickup (pharmacy), destination (room), scheduled time, priority<br>• Priority: routine, urgent, emergency<br>• Accessible from nurse dashboard |
| RES-CREATE-MED-002 | HIGH | System SHALL default pickup to pharmacy waypoint | • Pickup auto-filled with "Pharmacy - 1st Floor"<br>• Can be changed (e.g., different storage location)<br>• Destination required (patient room) |
| RES-CREATE-MED-003 | CRITICAL | System SHALL support priority levels for medicine delivery | • Routine: Normal queue (e.g., daily medication)<br>• Urgent: Jump queue (e.g., pain relief)<br>• Emergency: Immediate dispatch (e.g., critical medication)<br>• Priority shown with color coding |
| RES-CREATE-MED-004 | HIGH | System SHALL enforce priority-based time constraints | • Routine: ≥ 30 minutes advance<br>• Urgent: ≥ 10 minutes advance<br>• Emergency: Immediate (0 minutes)<br>• Error if constraints violated |
| RES-CREATE-MED-005 | MEDIUM | System SHALL require confirmation for emergency priority | • Confirmation dialog: "Emergency delivery will interrupt current missions. Confirm?"<br>• Requires explicit confirmation<br>• Logged to audit trail |

#### 3.1.3 Reservation Confirmation

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-CONFIRM-001 | CRITICAL | System SHALL generate unique reservation ID on creation | • Format: RES-YYYYMMDD-NNNN (e.g., RES-20251216-0042)<br>• Sequential numbering per day<br>• Displayed in confirmation |
| RES-CONFIRM-002 | HIGH | System SHALL display reservation summary after creation | • Summary includes: reservation ID, passenger/item, waypoints, scheduled time, priority<br>• Estimated duration shown<br>• Print/email option |
| RES-CONFIRM-003 | HIGH | System SHALL send confirmation notification to requester | • In-app notification + email (if configured)<br>• Contains reservation details + cancellation link<br>• Sent within 5 seconds of creation |
| RES-CONFIRM-004 | MEDIUM | System SHALL calculate estimated completion time | • Based on: pickup distance, transport distance, docking time<br>• Display: "Scheduled 10:00 AM, Est. Complete 10:12 AM"<br>• Updated if route changes |

---

### 3.2 Reservation Scheduling

#### 3.2.1 Time Slot Management

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-SCHED-SLOT-001 | HIGH | System SHALL use 15-minute time slots for scheduling | • Slots: 00:00, 00:15, 00:30, 00:45 each hour<br>• User selects nearest slot to desired time<br>• Simplifies conflict detection |
| RES-SCHED-SLOT-002 | HIGH | System SHALL show time slot availability in real-time | • Calendar/timeline view<br>• Available (green), limited (yellow), full (red)<br>• Updates as reservations created/cancelled |
| RES-SCHED-SLOT-003 | MEDIUM | System SHALL enforce operational hours | • Configurable operational hours (e.g., 8:00-20:00)<br>• Prevent bookings outside hours<br>• Exception: Emergency priority |
| RES-SCHED-SLOT-004 | MEDIUM | System SHALL support buffer time between reservations | • Configurable buffer (default 5 minutes)<br>• Prevents back-to-back missions<br>• Allows vehicle to reach pickup on time |

#### 3.2.2 Vehicle Assignment

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-SCHED-VEHICLE-001 | CRITICAL | System SHALL automatically assign vehicle to reservation at dispatch time | • Assignment algorithm: nearest available vehicle with sufficient battery<br>• No manual assignment during creation (auto at dispatch)<br>• Displayed in reservation status |
| RES-SCHED-VEHICLE-002 | HIGH | System SHALL consider vehicle battery level in assignment | • Require ≥30% battery for assignment<br>• Prefer vehicle with >50% battery<br>• Reject if all vehicles <20% battery |
| RES-SCHED-VEHICLE-003 | HIGH | System SHALL re-assign vehicle if initially assigned vehicle fails | • If vehicle enters error state before dispatch<br>• Automatic re-assignment to next available vehicle<br>• Notification sent to requester |
| RES-SCHED-VEHICLE-004 | MEDIUM | System SHALL optimize vehicle assignments for multiple reservations | • Batch nearby reservations to same vehicle (route optimization)<br>• Minimize total travel distance<br>• Consider reservation priorities |

---

### 3.3 Reservation Modification

#### 3.3.1 Time/Waypoint Changes

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-MOD-CHANGE-001 | HIGH | System SHALL allow reservation modification before dispatch | • Edit button in reservation details<br>• Can change: time, waypoints, notes, priority<br>• Cannot change after vehicle dispatched |
| RES-MOD-CHANGE-002 | HIGH | System SHALL validate modifications same as creation | • Same validation rules apply<br>• Check time slot availability<br>• Prevent conflicts |
| RES-MOD-CHANGE-003 | MEDIUM | System SHALL notify requester of successful modification | • Confirmation notification<br>• Shows old vs new details<br>• Updated estimated completion time |
| RES-MOD-CHANGE-004 | MEDIUM | System SHALL log all modifications to audit trail | • Timestamp, user, field changed, old value, new value<br>• Viewable by admins<br>• Exportable to CSV |

#### 3.3.2 Priority Changes

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-MOD-PRIORITY-001 | HIGH | System SHALL allow nurses to upgrade priority (routine → urgent → emergency) | • Upgrade button in reservation details<br>• Requires confirmation for emergency<br>• Re-queues reservation with new priority |
| RES-MOD-PRIORITY-002 | MEDIUM | System SHALL restrict priority downgrade | • Only admin can downgrade emergency → urgent<br>• Caregiver/nurse cannot downgrade own reservations<br>• Prevents gaming the queue |

---

### 3.4 Reservation Cancellation

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-CANCEL-001 | CRITICAL | System SHALL allow reservation cancellation before dispatch | • Cancel button in reservation details<br>• Confirmation dialog: "Cancel reservation RES-...?"<br>• Cannot cancel after vehicle dispatched (use mission cancel instead) |
| RES-CANCEL-002 | HIGH | System SHALL release time slot on cancellation | • Cancelled reservation removed from queue<br>• Time slot becomes available immediately<br>• Other queued reservations may move up |
| RES-CANCEL-003 | HIGH | System SHALL notify requester of cancellation | • Confirmation notification: "Reservation RES-... cancelled"<br>• Email notification (if configured)<br>• Cancellation timestamp recorded |
| RES-CANCEL-004 | MEDIUM | System SHALL log cancellation reason (optional) | • Optional field: "Reason for cancellation"<br>• Helps analyze cancellation patterns<br>• Viewable by admins |
| RES-CANCEL-005 | MEDIUM | System SHALL support bulk cancellation (admin only) | • Select multiple reservations<br>• Cancel all button<br>• Confirmation with count: "Cancel 5 reservations?" |

---

### 3.5 Recurring Reservations

#### 3.5.1 Recurrence Patterns

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-RECUR-PATTERN-001 | MEDIUM | System SHALL support daily recurring reservations | • "Repeat daily" checkbox on creation form<br>• End date or occurrence count (e.g., "Repeat 30 times")<br>• Creates series of linked reservations |
| RES-RECUR-PATTERN-002 | MEDIUM | System SHALL support weekly recurring reservations | • "Repeat weekly" with weekday selection<br>• Example: "Every Monday, Wednesday, Friday"<br>• Flexible for therapy schedules |
| RES-RECUR-PATTERN-003 | MEDIUM | System SHALL support custom intervals | • "Repeat every N days" (e.g., every 3 days)<br>• Max 30 days interval<br>• For irregular schedules |
| RES-RECUR-PATTERN-004 | LOW | System SHALL show recurrence pattern in reservation details | • Badge: "🔁 Daily", "🔁 Mon/Wed/Fri"<br>• Helps identify recurring reservations<br>• Clickable to view series |

#### 3.5.2 Recurring Reservation Management

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-RECUR-MGMT-001 | HIGH | System SHALL allow editing single occurrence vs all occurrences | • Edit dialog: "Edit this occurrence only" or "Edit all future occurrences"<br>• Changes propagate to selected scope<br>• Past occurrences unaffected |
| RES-RECUR-MGMT-002 | HIGH | System SHALL allow cancelling single occurrence vs all occurrences | • Cancel dialog: same options as edit<br>• "Cancel this occurrence" removes one instance<br>• "Cancel all" removes entire series |
| RES-RECUR-MGMT-003 | MEDIUM | System SHALL detect and skip holidays/facility closure days | • Admin configures holiday calendar<br>• Recurring reservations auto-skip these days<br>• Notification to requester |
| RES-RECUR-MGMT-004 | MEDIUM | System SHALL limit recurring reservation series to 3 months | • Max 90 days into future<br>• Prevents excessive long-term bookings<br>• Can extend series after 2 months |

---

### 3.6 Conflict Detection and Resolution

#### 3.6.1 Conflict Detection

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-CONFLICT-DETECT-001 | HIGH | System SHALL detect time slot conflicts | • Conflict: >5 reservations in same 15-min slot<br>• Warning shown during creation: "Time slot nearly full"<br>• Suggest alternative slots |
| RES-CONFLICT-DETECT-002 | HIGH | System SHALL detect waypoint conflicts (same pickup/destination at same time) | • Warning if same waypoint requested by multiple reservations within 5 minutes<br>• May cause pickup delays<br>• Not blocked, just warned |
| RES-CONFLICT-DETECT-003 | MEDIUM | System SHALL detect vehicle capacity conflicts | • If fleet has N vehicles, limit concurrent reservations to N<br>• Warning at N-1 reservations<br>• Block at N reservations (unless emergency) |

#### 3.6.2 Conflict Resolution

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-CONFLICT-RESOLVE-001 | HIGH | System SHALL prioritize emergency reservations | • Emergency reservations always accepted (even if conflicts)<br>• May delay lower-priority reservations<br>• Requester of delayed reservation notified |
| RES-CONFLICT-RESOLVE-002 | MEDIUM | System SHALL suggest alternative time slots for conflicts | • "Requested time 10:00 is full. Alternatives: 10:15, 10:30, 10:45"<br>• Up to 5 suggestions<br>• One-click rebook |
| RES-CONFLICT-RESOLVE-003 | MEDIUM | System SHALL allow admin to override capacity limits | • Admin can force-add reservation beyond limit<br>• Requires justification note<br>• Logged to audit trail |

---

### 3.7 Reservation Queue Management

#### 3.7.1 Queue Prioritization

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-QUEUE-PRIORITY-001 | CRITICAL | System SHALL maintain priority queue ordered by priority and time | • Order: Emergency > Urgent > Routine > Scheduled time<br>• Within same priority: FIFO (first in, first out)<br>• Queue updated real-time |
| RES-QUEUE-PRIORITY-002 | HIGH | System SHALL display queue position to requesters | • "Your reservation is #3 in queue"<br>• Updates as queue changes<br>• Estimated wait time shown |
| RES-QUEUE-PRIORITY-003 | HIGH | System SHALL allow admin to manually reorder queue | • Drag-and-drop interface<br>• Requires admin role<br>• Logged to audit trail with reason |

#### 3.7.2 Queue Status Updates

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-QUEUE-STATUS-001 | HIGH | System SHALL update reservation status in real-time | • States: pending, queued, dispatched, in_progress, completed, cancelled<br>• Status badge color-coded<br>• Timestamp for each state transition |
| RES-QUEUE-STATUS-002 | MEDIUM | System SHALL send status update notifications | • Notification when status changes to "dispatched"<br>• Notification when completed<br>• In-app + email (if configured) |
| RES-QUEUE-STATUS-003 | MEDIUM | System SHALL provide estimated dispatch time | • "Estimated dispatch: 10:05 AM (in 3 minutes)"<br>• Based on queue position and vehicle availability<br>• Updates dynamically |

---

### 3.8 Notification System

#### 3.8.1 Confirmation Notifications

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-NOTIF-CONFIRM-001 | HIGH | System SHALL send confirmation notification on reservation creation | • In-app notification (desktop popup)<br>• Email notification (if user enabled)<br>• Contains reservation ID, time, waypoints |
| RES-NOTIF-CONFIRM-002 | MEDIUM | System SHALL include cancellation link in confirmation | • Link format: https://.../reservations/{id}/cancel?token=...<br>• One-click cancellation (with confirmation)<br>• Token expires after reservation time |

#### 3.8.2 Reminder Notifications

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-NOTIF-REMIND-001 | MEDIUM | System SHALL send reminder notification 10 minutes before scheduled time | • Configurable reminder time (default 10 min)<br>• Notification: "Reminder: Your reservation RES-... starts in 10 minutes"<br>• User can disable reminders in settings |
| RES-NOTIF-REMIND-002 | LOW | System SHALL send reminder to passenger (if contact info available) | • SMS or phone call reminder (if facility has system)<br>• "Your wheelchair transport will arrive at 10:00 AM"<br>• Optional feature |

#### 3.8.3 Status Change Notifications

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-NOTIF-STATUS-001 | HIGH | System SHALL notify requester when vehicle is dispatched | • Notification: "Vehicle dispatched for RES-..."<br>• Includes vehicle ID and estimated arrival<br>• Sent within 5 seconds of dispatch |
| RES-NOTIF-STATUS-002 | HIGH | System SHALL notify requester when mission is completed | • Notification: "Reservation RES-... completed successfully"<br>• Timestamp of completion<br>• Option to provide feedback |
| RES-NOTIF-STATUS-003 | CRITICAL | System SHALL notify requester if mission fails | • Notification: "Reservation RES-... failed. Reason: {error}"<br>• Offer to rebook<br>• Escalation to admin |

---

### 3.9 Reservation History and Reporting

#### 3.9.1 Reservation History

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-HISTORY-001 | MEDIUM | System SHALL maintain complete reservation history | • All reservations stored (minimum 1 year)<br>• Fields: created_at, scheduled_time, actual_start, actual_end, status, requester, vehicle_id<br>• Searchable and filterable |
| RES-HISTORY-002 | MEDIUM | System SHALL allow requester to view own reservation history | • "My Reservations" page<br>• Filter by: date range, status, priority<br>• Export to CSV |
| RES-HISTORY-003 | MEDIUM | System SHALL allow admin to view all reservation history | • Full access to all reservations<br>• Advanced filters: requester, waypoint, vehicle<br>• Audit trail visible |

#### 3.9.2 Reservation Analytics

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-ANALYTICS-001 | MEDIUM | System SHALL generate reservation usage reports | • Daily/weekly/monthly reservation counts<br>• Breakdown by type (walking assistance vs medicine delivery)<br>• Breakdown by priority |
| RES-ANALYTICS-002 | MEDIUM | System SHALL calculate average lead time (booking time to scheduled time) | • Metric: "Average lead time: 2 hours 15 minutes"<br>• Helps plan vehicle capacity<br>• Trend analysis (improving/declining) |
| RES-ANALYTICS-003 | LOW | System SHALL identify peak reservation times | • Heatmap: day of week × hour of day<br>• Helps optimize vehicle deployment<br>• Example: "Peak: Mon-Fri 10:00-11:00" |
| RES-ANALYTICS-004 | LOW | System SHALL calculate cancellation rate | • Cancellation rate % (cancelled / total)<br>• Breakdown by requester<br>• Identify problematic patterns |

---

## 4. Non-Functional Requirements

### 4.1 Performance

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-PERF-001 | HIGH | System SHALL create reservation within 2 seconds | • From form submit to confirmation<br>• Includes validation and database insert<br>• 95th percentile <2s |
| RES-PERF-002 | MEDIUM | System SHALL update queue position within 1 second of reservation change | • Real-time updates via WebSocket<br>• No full page reload<br>• Smooth UI updates |

### 4.2 Usability

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-UX-001 | HIGH | Reservation form SHALL be completable in <60 seconds | • Measured in usability testing<br>• Minimal required fields<br>• Autocomplete and suggestions |
| RES-UX-002 | MEDIUM | System SHALL provide contextual help for each form field | • Tooltip on hover/tap<br>• Example: "Pickup: Where passenger is currently located"<br>• Japanese and English |

### 4.3 Reliability

| Req ID | Priority | Requirement | Acceptance Criteria |
|--------|----------|-------------|---------------------|
| RES-REL-001 | HIGH | System SHALL prevent double-booking of same reservation | • Database constraint: unique reservation ID<br>• Transaction isolation level: SERIALIZABLE<br>• Retry logic on conflict |
| RES-REL-002 | HIGH | System SHALL persist reservation queue across system restarts | • Queue stored in database (not just in-memory)<br>• Recovered on startup<br>• No reservations lost |

---

## 5. Interface Requirements

### 5.1 API Endpoints

| Endpoint | Method | Purpose |
|----------|--------|---------|
| /api/v1/reservations | POST | Create reservation |
| /api/v1/reservations/{id} | GET | Get reservation details |
| /api/v1/reservations/{id} | PUT | Update reservation |
| /api/v1/reservations/{id} | DELETE | Cancel reservation |
| /api/v1/reservations | GET | List reservations (with filters) |
| /api/v1/reservations/{id}/recurrence | POST | Create recurring reservation |
| /api/v1/reservations/queue | GET | Get current queue status |

### 5.2 Database Schema

**Table: reservations**
- id (UUID, primary key)
- reservation_id (VARCHAR, unique, e.g., RES-20251216-0042)
- type (ENUM: walking_assistance, medicine_delivery)
- requester_id (foreign key → users)
- passenger_name (VARCHAR) OR item_description (VARCHAR)
- pickup_waypoint_id (foreign key → waypoints)
- destination_waypoint_id (foreign key → waypoints)
- scheduled_time (TIMESTAMP)
- priority (ENUM: routine, urgent, emergency)
- status (ENUM: pending, queued, dispatched, in_progress, completed, cancelled)
- vehicle_id (foreign key → vehicles, nullable)
- created_at, updated_at (TIMESTAMP)
- notes (TEXT, nullable)
- recurrence_pattern_id (foreign key, nullable)

**Table: reservation_recurrence_patterns**
- id (UUID, primary key)
- type (ENUM: daily, weekly, custom)
- interval_days (INT) - for custom
- weekdays (JSON) - [1,3,5] for Mon/Wed/Fri
- end_date (DATE) OR occurrence_count (INT)
- created_at (TIMESTAMP)

**Table: reservation_status_history**
- id (UUID, primary key)
- reservation_id (foreign key)
- status (ENUM)
- timestamp (TIMESTAMP)
- notes (TEXT, nullable)

---

## 6. Testing Requirements

| Test Type | Coverage | Acceptance |
|-----------|----------|------------|
| Unit Tests | ≥80% code coverage | Reservation creation, validation, queue logic |
| Integration Tests | API endpoints, database operations | All CRUD operations tested |
| UI Tests | Reservation form, queue display, notifications | Key workflows automated (Cypress) |
| Load Tests | 100 concurrent reservation creations | No errors, response time <2s |
| Conflict Tests | Multiple users booking same slot | Correct conflict detection and resolution |

---

## 7. Traceability Matrix

| Requirement Category | Requirement Count | Priority Distribution |
|---------------------|-------------------|----------------------|
| Reservation Creation | 17 | CRITICAL: 5, HIGH: 7, MEDIUM: 5 |
| Reservation Scheduling | 8 | CRITICAL: 1, HIGH: 5, MEDIUM: 2 |
| Reservation Modification | 6 | HIGH: 3, MEDIUM: 3 |
| Reservation Cancellation | 5 | CRITICAL: 1, HIGH: 2, MEDIUM: 2 |
| Recurring Reservations | 8 | MEDIUM: 6, LOW: 2 |
| Conflict Detection/Resolution | 6 | HIGH: 4, MEDIUM: 2 |
| Queue Management | 6 | CRITICAL: 1, HIGH: 4, MEDIUM: 1 |
| Notification System | 8 | CRITICAL: 1, HIGH: 4, MEDIUM: 2, LOW: 1 |
| History and Reporting | 7 | MEDIUM: 5, LOW: 2 |
| Non-Functional | 6 | HIGH: 4, MEDIUM: 2 |
| **TOTAL** | **77** | **CRITICAL: 9, HIGH: 33, MEDIUM: 30, LOW: 5** |

---

## 8. Assumptions and Dependencies

### 8.1 Assumptions

1. Facility has defined waypoints (room numbers, common areas)
2. Resident database available with names and contact info
3. Staff trained on reservation system usage
4. Operational hours configured (e.g., 8:00-20:00)

### 8.2 Dependencies

1. FLEET_MANAGEMENT_REQUIREMENTS.md (parent system)
2. TVM_SERVER_REQUIREMENTS.md (backend API implementation)
3. USER_ROLE_MANAGEMENT_REQUIREMENTS.md (user authentication and roles)
4. Vehicle dispatch system operational

---

## 9. Open Issues and Risks

### 9.1 Open Issues

| ID | Issue | Owner | Target Resolution |
|----|-------|-------|-------------------|
| RES-ISSUE-001 | Recurring reservation holiday calendar data source | Unno | Week 4 |
| RES-ISSUE-002 | Notification delivery method (email vs SMS) | Facility IT | Week 4 |

### 9.2 Risks

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| Over-booking due to concurrent requests | MEDIUM | HIGH | Database-level locking, transaction isolation |
| Users bypass queue by creating multiple reservations | LOW | MEDIUM | Limit: max 3 pending reservations per user |
| Emergency priority misuse | LOW | MEDIUM | Audit trail, admin review, usage reports |

---

## 10. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-16 | Unno | Initial draft - Week 3 documentation |

---

**END OF DOCUMENT**

**Total Requirements:** 77
**Document Status:** Draft for Review
**Next Review:** Week 4 (integration with queue management and dispatch)
