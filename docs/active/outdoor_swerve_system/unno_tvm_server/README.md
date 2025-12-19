# TVM Server (Fleet Management) - Unno's Scope
# TVMサーバー（フリート管理） - Unnoの範囲

**Team Lead:** Unno
**Scope:** TVM Server (Backend + Dashboard) ONLY
**Timeline:** 20 weeks (15 weeks MVP + 5 weeks advanced features)
**Status:** Ready for implementation

---

## 🎯 What You Build

**Fleet Management System (Server-Side):**
- ✅ **Backend Server** - REST API + WebSocket server
- ✅ **Database** - Vehicle data, missions, users, telemetry
- ✅ **Fleet Dashboard** - Web application (PC/Tablet)
- ✅ **User Authentication** - JWT, role-based access control (RBAC)
- ✅ **Reservation System** - Mission scheduling and dispatch
- ✅ **Real-time Monitoring** - Vehicle location, status, battery
- ✅ **Teleoperation** - Manual control interface (optional)
- ✅ **Voice Communication** - VoIP integration (optional)

---

## ❌ What You DON'T Build

- ❌ **Vehicle Software** (Pankaj's responsibility)
  - Not ROS 2 nodes, navigation, docking, perception
  - Not vehicle-side client code

- ❌ **Hardware Platform** (Tsuchiya + Kiril's responsibility)
  - Not mechanical, electrical, sensors
  - Not physical robot

**Your Focus:** Server-side backend and fleet management dashboard

---

## 🔗 Your Interface

### Interface to Vehicle Software (Pankaj)

**Documents (CRITICAL):**
- `04_INTERFACES/TVM_API_SPECIFICATION.md` - **YOU MUST IMPLEMENT THIS**
- `04_INTERFACES/TVM_DATA_MODELS.md` - JSON schemas you must follow

**What Pankaj Sends to You:**
- Vehicle telemetry (location, status, battery, errors) via REST API
- Heartbeat every 1 second
- Mission status updates

**What You Send to Pankaj:**
- Mission dispatch commands via WebSocket
- Emergency stop commands
- Configuration updates
- Cancel mission commands

**Implementation Guide:**
- See `TVM_API_IMPLEMENTATION_GUIDE_FOR_UNNO.md` (will be provided)
- Use mock vehicle client for testing (will be provided)

---

## 🛠️ Technology Stack (YOUR CHOICE)

**Backend Options:**
- Node.js + Express/NestJS
- Python + FastAPI/Django
- Java + Spring Boot
- **Your decision** - choose based on team expertise

**Database Options:**
- PostgreSQL (relational, recommended)
- MySQL (relational)
- MongoDB (document, flexible)
- **Your decision** - see DATABASE_DESIGN.md for schema reference

**Frontend Options:**
- React (most popular)
- Vue (lightweight)
- Angular (enterprise)
- **Your decision** - must be web-based (PC/Tablet)

**Real-Time:**
- WebSocket / Socket.io (required for vehicle commands)

**VoIP (Optional):**
- Twilio (commercial)
- WebRTC (open standard)
- Asterisk (open-source PBX)

---

## 📁 Folder Structure

```
unno_tvm_server/
├── 01_REQUIREMENTS/          ← What the server must do
│   ├── FLEET_MANAGEMENT_REQUIREMENTS.md
│   ├── TVM_SERVER_REQUIREMENTS.md
│   ├── RESERVATION_SYSTEM_REQUIREMENTS.md
│   ├── USER_ROLE_MANAGEMENT_REQUIREMENTS.md
│   ├── FLEET_UI_REQUIREMENTS.md
│   └── TELEOPERATION_REQUIREMENTS.md
│
├── 02_ARCHITECTURE/          ← How it's organized
│   ├── TVM_SERVER_ARCHITECTURE.md
│   ├── DATABASE_ARCHITECTURE.md
│   ├── DATABASE_DESIGN.md (PostgreSQL schema reference)
│   └── FLEET_UI_ARCHITECTURE.md
│
├── 03_DESIGN/                ← Detailed designs
│   ├── TVM_SERVER_DESIGN.md
│   ├── DATABASE_SCHEMA_DESIGN.md
│   ├── AUTHENTICATION_DESIGN.md
│   └── FLEET_UI_COMPONENTS_DESIGN.md
│
├── 04_INTERFACES/            ← Interface to vehicle (Pankaj)
│   ├── TVM_API_SPECIFICATION.md ← IMPLEMENT THIS (critical!)
│   └── TVM_DATA_MODELS.md ← Follow these schemas
│
├── 05_DEVELOPMENT/           ← How to develop
│   └── FLEET_SOFTWARE_DEVELOPMENT_GUIDE.md
│
└── README.md                 ← This file
```

---

## 🚀 Quick Start

### 1. Read Interface Documents First

**CRITICAL - Read these before coding:**
1. `04_INTERFACES/TVM_API_SPECIFICATION.md` - **What APIs you must implement**
2. `04_INTERFACES/TVM_DATA_MODELS.md` - **JSON schemas you must follow**
3. `TVM_API_IMPLEMENTATION_GUIDE_FOR_UNNO.md` - Implementation guide (will be provided by Pankaj)

### 2. Review Requirements

Start with these to understand what to build:
1. `01_REQUIREMENTS/TVM_SERVER_REQUIREMENTS.md`
2. `01_REQUIREMENTS/FLEET_MANAGEMENT_REQUIREMENTS.md`
3. `01_REQUIREMENTS/RESERVATION_SYSTEM_REQUIREMENTS.md`

### 3. Review Database Design

See: `02_ARCHITECTURE/DATABASE_DESIGN.md` (PostgreSQL schema reference)

**11 Tables:**
- users, vehicles, missions, mission_waypoints
- vehicle_telemetry, vehicle_errors, reservations
- floors, routes, route_waypoints, audit_logs

### 4. Choose Technology Stack

**Decisions needed:**
- Backend framework (Node.js/Python/Java)
- Database (PostgreSQL recommended)
- Frontend framework (React/Vue/Angular)

### 5. Begin Implementation

**Suggested order (MVP first):**

**Phase 1: MVP (15 weeks)**
1. **Week 1-2:** Authentication + User Management
2. **Week 3-5:** Database schema + basic CRUD
3. **Week 6-8:** Vehicle telemetry ingestion (REST API)
4. **Week 9-11:** Mission dispatch (WebSocket)
5. **Week 12-13:** Fleet dashboard (basic UI)
6. **Week 14-15:** Integration testing with Pankaj

**Phase 2: Advanced Features (5 weeks)**
7. **Week 16-17:** Reservation system
8. **Week 18-19:** Advanced dashboard features
9. **Week 20:** Voice communication (optional)

---

## 🔑 Key Design Principles

### 1. API-First Development
- Implement TVM_API_SPECIFICATION.md exactly as specified
- Use mock vehicle client for testing (provided by Pankaj)
- Don't change API without coordinating with Pankaj

### 2. Pilot-Level Simplicity
- Single-server deployment (not multi-region)
- Basic high availability (not enterprise 99.9%+ uptime)
- No disaster recovery (senior said exclude)
- Focus on functionality over scalability

### 3. Real-Time Vehicle Communication
- WebSocket server must handle:
  - Multiple concurrent vehicle connections
  - Heartbeat every 1 second from each vehicle
  - Command dispatch with <1 second latency
  - Reconnection handling (vehicles go offline)

### 4. User Roles
- **Admin:** Full access, system configuration
- **Operator:** Dispatch missions, monitor fleet
- **Caregiver:** Request transport for residents
- **Nurse:** Medical transport requests (optional)

---

## 📋 API Implementation Checklist

### REST API Endpoints (Vehicle → Server)

**Must implement these:**
- [ ] `POST /api/v1/auth/login` - Vehicle authentication
- [ ] `POST /api/v1/telemetry` - Upload telemetry (location, status, battery)
- [ ] `POST /api/v1/missions/{id}/status` - Update mission status
- [ ] `POST /api/v1/errors` - Report errors
- [ ] `GET /api/v1/vehicles/{id}/config` - Get vehicle configuration

### WebSocket (Server → Vehicle)

**Must implement these:**
- [ ] `mission.dispatch` - Send new mission to vehicle
- [ ] `mission.cancel` - Cancel current mission
- [ ] `emergency.stop` - Emergency stop command
- [ ] `config.update` - Configuration update

### Dashboard API Endpoints (Dashboard → Server)

**Must implement these:**
- [ ] User authentication and authorization
- [ ] Vehicle list and status
- [ ] Mission creation and dispatch
- [ ] Real-time location tracking
- [ ] Historical telemetry queries

---

## 🧪 Testing Strategy

### 1. Unit Testing
- Test each API endpoint independently
- Test database operations
- Test WebSocket connection handling

### 2. Integration Testing with Mock Vehicle
- Pankaj will provide mock vehicle client
- Use it to test:
  - Telemetry ingestion
  - Mission dispatch
  - Command delivery
  - Error handling

### 3. Load Testing (Optional)
- Simulate multiple vehicles (10-20)
- Test WebSocket connection limits
- Test database query performance

### 4. Integration with Real Vehicle
- Week 14-15: Integration testing with Pankaj
- Test real vehicle telemetry
- Test real mission dispatch
- Test reconnection scenarios

---

## 📊 Database Schema Overview

**Reference:** `02_ARCHITECTURE/DATABASE_DESIGN.md`

**Key Tables:**

1. **users** - User accounts (admin, operator, caregiver)
2. **vehicles** - Vehicle registry and configuration
3. **missions** - Mission queue and history
4. **mission_waypoints** - Waypoints for each mission
5. **vehicle_telemetry** - Real-time vehicle data (location, status, battery)
6. **vehicle_errors** - Error logs from vehicles
7. **reservations** - Scheduled transport requests
8. **floors** - Building floor definitions
9. **routes** - Predefined routes
10. **route_waypoints** - Waypoints for routes
11. **audit_logs** - User action audit trail

**Time-Series Data:**
- `vehicle_telemetry` can grow large (1Hz updates)
- Consider partitioning or time-series database (TimescaleDB)
- Implement data retention policy (keep 30 days)

---

## 🔒 Security Requirements

**Reference:** Simplified from Pankaj's scope

**Authentication:**
- JWT tokens for vehicles and users
- Token expiration: 24 hours (vehicles), 8 hours (users)
- Refresh token mechanism

**Authorization:**
- Role-based access control (RBAC)
- Permissions per user role
- API endpoint access control

**Data Protection:**
- HTTPS/TLS for all communication
- Encrypt sensitive data in database (passwords, tokens)
- API rate limiting (prevent abuse)

**Privacy:**
- Coordinate with Li San (legal review)
- Implement data retention policies
- User consent for data collection

---

## 📞 Contacts

**Your Team:** Unno (+ team members TBD)

**Vehicle Software Team:** Pankaj
- **Interface:** TVM_API_SPECIFICATION.md
- **Coordination:** API review meetings, integration testing (Week 14-15)
- **Mock Client:** Will be provided by Pankaj for testing

**Hardware Team:** Tsuchiya + Kiril
- **No direct interface** - communicate via vehicle software
- **Coordination:** Not required

**Legal:** Li San
- **Review:** Privacy requirements
- **Status:** Pending review

**Senior:** [Name]
- **Approval:** Final architecture and scope

---

## ✅ Success Criteria

### MVP (Week 15)

- [ ] All REST API endpoints implemented
- [ ] All WebSocket commands implemented
- [ ] Database schema complete
- [ ] Basic fleet dashboard working
- [ ] User authentication working
- [ ] Integration test with Pankaj passed

### Advanced Features (Week 20)

- [ ] Reservation system working
- [ ] Advanced dashboard features complete
- [ ] Voice communication (if implemented)
- [ ] Load testing passed (10+ vehicles)

### Quality

- [ ] API follows TVM_API_SPECIFICATION.md exactly
- [ ] All JSON schemas validated against TVM_DATA_MODELS.md
- [ ] Code coverage ≥70%
- [ ] Documentation complete

---

## ⚠️ Important Notes

### API Contract is Frozen

**DO NOT change the API without coordinating with Pankaj:**
- REST endpoints are defined in TVM_API_SPECIFICATION.md
- WebSocket commands are defined
- JSON schemas are defined in TVM_DATA_MODELS.md

**If you need API changes:**
1. Discuss with Pankaj
2. Update TVM_API_SPECIFICATION.md (both agree)
3. Use semantic versioning (v1, v2, etc.)

### Offline Vehicle Handling

**Vehicles may go offline:**
- WiFi disconnection
- Battery drain
- Network issues

**Your server must:**
- Queue commands for offline vehicles
- Retry command delivery when vehicle reconnects
- Mark missions as "failed" if vehicle offline too long

### Real-Time Dashboard Updates

**Users expect real-time updates:**
- Vehicle locations update every 1 second
- Mission status updates immediately
- Use WebSocket or Server-Sent Events (SSE) to dashboard

---

**Document Version:** 1.0
**Last Updated:** December 19, 2025
**Status:** Ready for implementation
**Next Action:** Wait for TVM_API_IMPLEMENTATION_GUIDE_FOR_UNNO.md from Pankaj

---

**Happy Coding! 🚀**
