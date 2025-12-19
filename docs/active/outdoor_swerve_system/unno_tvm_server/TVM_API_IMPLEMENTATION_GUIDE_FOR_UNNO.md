# TVM API Implementation Guide for Unno

**Document Type:** Implementation Guide
**Target Audience:** Unno (TVM Server Team)
**Status:** Active
**Version:** 1.0
**Date:** December 19, 2025
**Prepared By:** Pankaj (Vehicle Software Team)

---

## Table of Contents

1. [Quick Start](#1-quick-start)
2. [API Overview](#2-api-overview)
3. [Step-by-Step Implementation](#3-step-by-step-implementation)
4. [Code Examples](#4-code-examples)
5. [Testing Guide](#5-testing-guide)
6. [Mock Vehicle Client](#6-mock-vehicle-client)
7. [Integration Testing](#7-integration-testing)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. Quick Start

### 1.1 What You Need to Implement

**Your responsibility (Unno):**
- ✅ TVM server backend (REST API + WebSocket)
- ✅ Database (PostgreSQL/MySQL/MongoDB - your choice)
- ✅ User authentication (JWT for vehicles and users)
- ✅ Fleet dashboard (React/Vue/Angular - your choice)

**What Pankaj provides:**
- ✅ API specification (TVM_API_SPECIFICATION.md)
- ✅ Data models (TVM_DATA_MODELS.md)
- ✅ Mock vehicle client for testing (this guide)
- ✅ Vehicle software that calls your APIs

### 1.2 Technology Choices

**You decide:**
- Backend framework: Node.js (Express/NestJS), Python (FastAPI/Django), Java (Spring Boot)
- Database: PostgreSQL (recommended), MySQL, MongoDB
- Frontend: React (recommended), Vue, Angular

**This guide uses:**
- Python + FastAPI (examples)
- PostgreSQL (database schema)
- But you can adapt to your chosen stack

### 1.3 Timeline

**Week 1-2:** Authentication + basic endpoints
**Week 3-5:** Database schema + CRUD operations
**Week 6-8:** WebSocket for mission commands
**Week 9-11:** Fleet dashboard (basic)
**Week 12-15:** Testing + integration with Pankaj

---

## 2. API Overview

### 2.1 What Your Server Must Provide

**REST API (Vehicle → Server):**
```
POST /api/v1/auth/login          - Vehicle authentication
POST /api/v1/telemetry           - Upload telemetry (1Hz from vehicle)
POST /api/v1/missions/{id}/status - Update mission status
POST /api/v1/errors              - Report errors
GET  /api/v1/vehicles/{id}/config - Get vehicle configuration
```

**WebSocket (Server → Vehicle):**
```
mission.dispatch   - Send new mission to vehicle
mission.cancel     - Cancel current mission
emergency.stop     - Emergency stop command
config.update      - Update vehicle configuration
```

**Dashboard API (Dashboard → Server):**
```
POST /api/v1/auth/login          - User login
GET  /api/v1/vehicles            - List all vehicles
POST /api/v1/missions            - Create new mission
GET  /api/v1/missions/{id}       - Get mission details
GET  /api/v1/telemetry/latest    - Get latest telemetry
```

### 2.2 Authentication Flow

```
Vehicle Startup:
1. Vehicle sends: POST /api/v1/auth/login
   Body: {"vehicle_id": "VH-001", "api_key": "secret_key"}
2. Server responds: {"access_token": "jwt_token", "expires_in": 3600}
3. Vehicle stores token, uses in all subsequent requests
4. Vehicle refreshes token before expiration

User Login:
1. User enters username/password on dashboard
2. Dashboard sends: POST /api/v1/auth/login
   Body: {"username": "operator1", "password": "password"}
3. Server validates, responds: {"access_token": "jwt_token", "role": "operator"}
4. Dashboard stores token, uses in all requests
```

---

## 3. Step-by-Step Implementation

### 3.1 Phase 1: Authentication (Week 1-2)

**Step 1: Set up project structure**

```bash
# Example for FastAPI (Python)
mkdir tvm_server
cd tvm_server
python3 -m venv venv
source venv/bin/activate
pip install fastapi uvicorn pyjwt psycopg2-binary python-jose
```

**Step 2: Implement JWT authentication**

See [Code Examples](#4-code-examples) Section 4.1

**Step 3: Create vehicle login endpoint**

```python
# POST /api/v1/auth/login (for vehicles)
@app.post("/api/v1/auth/login")
async def vehicle_login(credentials: VehicleCredentials):
    # 1. Validate vehicle_id and api_key
    vehicle = db.get_vehicle_by_id(credentials.vehicle_id)
    if not vehicle or vehicle.api_key != credentials.api_key:
        raise HTTPException(401, "Invalid credentials")

    # 2. Generate JWT token
    payload = {
        "vehicle_id": credentials.vehicle_id,
        "exp": datetime.utcnow() + timedelta(hours=1)
    }
    token = jwt.encode(payload, SECRET_KEY, algorithm="HS256")

    # 3. Return token
    return {"access_token": token, "token_type": "bearer", "expires_in": 3600}
```

**Step 4: Test with curl**

```bash
curl -X POST http://localhost:8000/api/v1/auth/login \
  -H "Content-Type: application/json" \
  -d '{"vehicle_id": "VH-001", "api_key": "test_key_123"}'

# Expected response:
# {"access_token": "eyJ0eXAiOiJKV1...", "token_type": "bearer", "expires_in": 3600}
```

---

### 3.2 Phase 2: Database Setup (Week 3-5)

**Step 1: Create PostgreSQL database**

```sql
CREATE DATABASE tvm_fleet;
CREATE USER tvm_user WITH PASSWORD 'secure_password';
GRANT ALL PRIVILEGES ON DATABASE tvm_fleet TO tvm_user;
```

**Step 2: Create tables**

See TVM_SERVER_REQUIREMENTS.md Section 2 for complete schema.

**Key tables (minimum for MVP):**

```sql
-- Vehicles table
CREATE TABLE vehicles (
    id SERIAL PRIMARY KEY,
    vehicle_id VARCHAR(50) UNIQUE NOT NULL,
    model VARCHAR(100),
    api_key VARCHAR(255) NOT NULL,
    status VARCHAR(20) DEFAULT 'offline',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Missions table
CREATE TABLE missions (
    id SERIAL PRIMARY KEY,
    vehicle_id VARCHAR(50) REFERENCES vehicles(vehicle_id),
    status VARCHAR(20) DEFAULT 'pending',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    started_at TIMESTAMP,
    completed_at TIMESTAMP
);

-- Mission waypoints table
CREATE TABLE mission_waypoints (
    id SERIAL PRIMARY KEY,
    mission_id INTEGER REFERENCES missions(id),
    sequence INTEGER NOT NULL,
    x FLOAT NOT NULL,
    y FLOAT NOT NULL,
    heading FLOAT,
    action VARCHAR(50)
);

-- Telemetry table (time-series data)
CREATE TABLE vehicle_telemetry (
    id SERIAL PRIMARY KEY,
    vehicle_id VARCHAR(50) REFERENCES vehicles(vehicle_id),
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    x FLOAT,
    y FLOAT,
    heading FLOAT,
    battery_percent INTEGER,
    status VARCHAR(20),
    cpu_usage FLOAT,
    ram_usage FLOAT
);

-- Users table (for dashboard)
CREATE TABLE users (
    id SERIAL PRIMARY KEY,
    username VARCHAR(50) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    role VARCHAR(20) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

**Step 3: Insert test data**

```sql
-- Test vehicle
INSERT INTO vehicles (vehicle_id, model, api_key, status)
VALUES ('VH-001', 'MultiGo-Outdoor-v1', 'test_key_123', 'offline');

-- Test user
INSERT INTO users (username, password_hash, role)
VALUES ('admin', '<bcrypt_hash>', 'admin');
```

---

### 3.3 Phase 3: Telemetry Endpoint (Week 6-8)

**Implementation:**

```python
# POST /api/v1/telemetry
@app.post("/api/v1/telemetry")
async def upload_telemetry(
    telemetry: TelemetryData,
    vehicle_id: str = Depends(get_current_vehicle)  # From JWT
):
    # 1. Validate telemetry data
    if not validate_telemetry(telemetry):
        raise HTTPException(400, "Invalid telemetry data")

    # 2. Store in database
    db.insert_telemetry(vehicle_id, telemetry)

    # 3. Update vehicle status
    db.update_vehicle_status(vehicle_id, telemetry.status)

    # 4. Return success
    return {"status": "ok", "timestamp": datetime.utcnow().isoformat()}
```

**Expected request from vehicle (every 1 second):**

```json
POST /api/v1/telemetry
Authorization: Bearer <jwt_token>

{
  "timestamp": "2025-12-19T10:30:45Z",
  "location": {
    "x": 12.5,
    "y": 8.3,
    "heading": 45.0
  },
  "status": "NAVIGATING",
  "battery": {
    "percent": 75,
    "voltage": 48.2,
    "current": 5.5
  },
  "system": {
    "cpu_usage": 45.2,
    "ram_usage": 62.1,
    "disk_usage": 38.5
  }
}
```

---

### 3.4 Phase 4: WebSocket for Commands (Week 9-11)

**Step 1: Set up WebSocket server**

```python
from fastapi import WebSocket

# WebSocket endpoint
@app.websocket("/ws/{vehicle_id}")
async def websocket_endpoint(websocket: WebSocket, vehicle_id: str):
    # 1. Authenticate vehicle (check JWT in query params or first message)
    await websocket.accept()

    # 2. Register connection
    connection_manager.connect(vehicle_id, websocket)

    try:
        while True:
            # 3. Receive messages from vehicle (heartbeat, status)
            data = await websocket.receive_json()

            if data["type"] == "heartbeat":
                # Update last_seen timestamp
                db.update_vehicle_heartbeat(vehicle_id)
                await websocket.send_json({"type": "ack"})

            elif data["type"] == "mission_status":
                # Update mission status in database
                db.update_mission_status(data["mission_id"], data["status"])

    except WebSocketDisconnect:
        connection_manager.disconnect(vehicle_id)
```

**Step 2: Send mission dispatch command**

```python
# When operator creates mission via dashboard
@app.post("/api/v1/missions")
async def create_mission(mission: MissionCreate, user_id: str = Depends(get_current_user)):
    # 1. Create mission in database
    mission_id = db.create_mission(mission.vehicle_id, mission.waypoints)

    # 2. Send command to vehicle via WebSocket
    command = {
        "type": "mission.dispatch",
        "mission_id": mission_id,
        "waypoints": mission.waypoints
    }

    await connection_manager.send_to_vehicle(mission.vehicle_id, command)

    return {"mission_id": mission_id, "status": "dispatched"}
```

**Vehicle receives:**

```json
{
  "type": "mission.dispatch",
  "mission_id": 12345,
  "waypoints": [
    {"x": 10.0, "y": 5.0, "heading": 0.0, "action": "navigate"},
    {"x": 15.0, "y": 8.0, "heading": 90.0, "action": "navigate"},
    {"x": 15.0, "y": 8.0, "heading": 180.0, "action": "dock"}
  ]
}
```

---

## 4. Code Examples

### 4.1 JWT Authentication Helper

```python
# auth.py
from jose import JWTError, jwt
from datetime import datetime, timedelta
from fastapi import HTTPException, Depends
from fastapi.security import HTTPBearer

SECRET_KEY = "your-secret-key-here"  # Use environment variable!
ALGORITHM = "HS256"

security = HTTPBearer()

def create_access_token(data: dict, expires_delta: timedelta = timedelta(hours=1)):
    to_encode = data.copy()
    expire = datetime.utcnow() + expires_delta
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

def verify_token(token: str):
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        return payload
    except JWTError:
        raise HTTPException(status_code=401, detail="Invalid token")

def get_current_vehicle(credentials = Depends(security)):
    token = credentials.credentials
    payload = verify_token(token)
    vehicle_id = payload.get("vehicle_id")
    if vehicle_id is None:
        raise HTTPException(status_code=401, detail="Invalid authentication")
    return vehicle_id
```

### 4.2 Database Connection (PostgreSQL)

```python
# database.py
import psycopg2
from psycopg2.extras import RealDictCursor

class Database:
    def __init__(self):
        self.conn = psycopg2.connect(
            host="localhost",
            database="tvm_fleet",
            user="tvm_user",
            password="secure_password"
        )

    def get_vehicle_by_id(self, vehicle_id: str):
        cursor = self.conn.cursor(cursor_factory=RealDictCursor)
        cursor.execute("SELECT * FROM vehicles WHERE vehicle_id = %s", (vehicle_id,))
        return cursor.fetchone()

    def insert_telemetry(self, vehicle_id: str, telemetry: dict):
        cursor = self.conn.cursor()
        cursor.execute("""
            INSERT INTO vehicle_telemetry
            (vehicle_id, x, y, heading, battery_percent, status, cpu_usage, ram_usage)
            VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
        """, (
            vehicle_id,
            telemetry["location"]["x"],
            telemetry["location"]["y"],
            telemetry["location"]["heading"],
            telemetry["battery"]["percent"],
            telemetry["status"],
            telemetry["system"]["cpu_usage"],
            telemetry["system"]["ram_usage"]
        ))
        self.conn.commit()

    def create_mission(self, vehicle_id: str, waypoints: list):
        cursor = self.conn.cursor()
        # Insert mission
        cursor.execute("""
            INSERT INTO missions (vehicle_id, status)
            VALUES (%s, 'pending') RETURNING id
        """, (vehicle_id,))
        mission_id = cursor.fetchone()[0]

        # Insert waypoints
        for seq, wp in enumerate(waypoints):
            cursor.execute("""
                INSERT INTO mission_waypoints (mission_id, sequence, x, y, heading, action)
                VALUES (%s, %s, %s, %s, %s, %s)
            """, (mission_id, seq, wp["x"], wp["y"], wp.get("heading", 0), wp.get("action", "navigate")))

        self.conn.commit()
        return mission_id
```

### 4.3 WebSocket Connection Manager

```python
# websocket_manager.py
from fastapi import WebSocket
from typing import Dict

class ConnectionManager:
    def __init__(self):
        self.active_connections: Dict[str, WebSocket] = {}

    def connect(self, vehicle_id: str, websocket: WebSocket):
        self.active_connections[vehicle_id] = websocket

    def disconnect(self, vehicle_id: str):
        if vehicle_id in self.active_connections:
            del self.active_connections[vehicle_id]

    async def send_to_vehicle(self, vehicle_id: str, message: dict):
        if vehicle_id in self.active_connections:
            await self.active_connections[vehicle_id].send_json(message)
        else:
            raise Exception(f"Vehicle {vehicle_id} not connected")

    async def broadcast(self, message: dict):
        for connection in self.active_connections.values():
            await connection.send_json(message)

manager = ConnectionManager()
```

---

## 5. Testing Guide

### 5.1 Unit Testing

**Test authentication:**

```python
# test_auth.py
import pytest
from fastapi.testclient import TestClient
from main import app

client = TestClient(app)

def test_vehicle_login_success():
    response = client.post("/api/v1/auth/login", json={
        "vehicle_id": "VH-001",
        "api_key": "test_key_123"
    })
    assert response.status_code == 200
    assert "access_token" in response.json()

def test_vehicle_login_invalid_credentials():
    response = client.post("/api/v1/auth/login", json={
        "vehicle_id": "VH-999",
        "api_key": "wrong_key"
    })
    assert response.status_code == 401
```

**Test telemetry endpoint:**

```python
def test_upload_telemetry():
    # First, get auth token
    login_response = client.post("/api/v1/auth/login", json={
        "vehicle_id": "VH-001",
        "api_key": "test_key_123"
    })
    token = login_response.json()["access_token"]

    # Upload telemetry
    response = client.post("/api/v1/telemetry",
        headers={"Authorization": f"Bearer {token}"},
        json={
            "timestamp": "2025-12-19T10:30:45Z",
            "location": {"x": 10.0, "y": 5.0, "heading": 0.0},
            "status": "IDLE",
            "battery": {"percent": 80, "voltage": 48.5, "current": 0.5},
            "system": {"cpu_usage": 25.0, "ram_usage": 50.0, "disk_usage": 30.0}
        }
    )
    assert response.status_code == 200
```

### 5.2 Manual Testing with Postman

**Collection structure:**

```
TVM API Tests/
├── Auth/
│   ├── Vehicle Login
│   └── User Login
├── Telemetry/
│   ├── Upload Telemetry
│   └── Get Latest Telemetry
├── Missions/
│   ├── Create Mission
│   ├── Get Mission
│   └── Update Mission Status
└── WebSocket/
    └── Test Connection
```

**Environment variables:**
```
base_url: http://localhost:8000
vehicle_token: {{vehicle_access_token}}
user_token: {{user_access_token}}
```

---

## 6. Mock Vehicle Client

### 6.1 Purpose

This mock client simulates a vehicle for testing your TVM server **without needing Pankaj's actual vehicle software**.

### 6.2 Mock Client Code

```python
# mock_vehicle_client.py
import requests
import websocket
import json
import time
from datetime import datetime

class MockVehicleClient:
    def __init__(self, vehicle_id, api_key, server_url):
        self.vehicle_id = vehicle_id
        self.api_key = api_key
        self.server_url = server_url
        self.access_token = None
        self.ws = None

    def login(self):
        """Authenticate and get JWT token"""
        response = requests.post(
            f"{self.server_url}/api/v1/auth/login",
            json={"vehicle_id": self.vehicle_id, "api_key": self.api_key}
        )
        if response.status_code == 200:
            self.access_token = response.json()["access_token"]
            print(f"✅ Logged in as {self.vehicle_id}")
            return True
        else:
            print(f"❌ Login failed: {response.text}")
            return False

    def send_telemetry(self):
        """Send telemetry to server (simulate 1Hz)"""
        telemetry = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "location": {"x": 10.0, "y": 5.0, "heading": 0.0},
            "status": "IDLE",
            "battery": {"percent": 80, "voltage": 48.5, "current": 0.5},
            "system": {"cpu_usage": 25.0, "ram_usage": 50.0, "disk_usage": 30.0}
        }

        response = requests.post(
            f"{self.server_url}/api/v1/telemetry",
            headers={"Authorization": f"Bearer {self.access_token}"},
            json=telemetry
        )

        if response.status_code == 200:
            print(f"✅ Telemetry sent at {telemetry['timestamp']}")
        else:
            print(f"❌ Telemetry failed: {response.text}")

    def connect_websocket(self):
        """Connect to WebSocket and listen for commands"""
        ws_url = self.server_url.replace("http", "ws") + f"/ws/{self.vehicle_id}"
        self.ws = websocket.WebSocketApp(
            ws_url,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )
        self.ws.on_open = self.on_open
        self.ws.run_forever()

    def on_open(self, ws):
        print(f"✅ WebSocket connected")
        # Send heartbeat every 5 seconds
        def send_heartbeat():
            while True:
                ws.send(json.dumps({"type": "heartbeat"}))
                time.sleep(5)

        import threading
        threading.Thread(target=send_heartbeat, daemon=True).start()

    def on_message(self, ws, message):
        data = json.loads(message)
        print(f"📨 Received command: {data['type']}")

        if data["type"] == "mission.dispatch":
            print(f"  Mission ID: {data['mission_id']}")
            print(f"  Waypoints: {len(data['waypoints'])}")
            # Simulate accepting mission
            ws.send(json.dumps({
                "type": "mission_status",
                "mission_id": data["mission_id"],
                "status": "accepted"
            }))

        elif data["type"] == "emergency.stop":
            print(f"  🚨 EMERGENCY STOP!")

    def on_error(self, ws, error):
        print(f"❌ WebSocket error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        print(f"⚠️  WebSocket closed: {close_msg}")

    def run(self):
        """Main loop: login, send telemetry, connect WebSocket"""
        if not self.login():
            return

        # Send telemetry in background thread
        import threading
        def telemetry_loop():
            while True:
                self.send_telemetry()
                time.sleep(1)  # 1Hz

        threading.Thread(target=telemetry_loop, daemon=True).start()

        # Connect WebSocket (blocking)
        self.connect_websocket()

# Usage
if __name__ == "__main__":
    client = MockVehicleClient(
        vehicle_id="VH-001",
        api_key="test_key_123",
        server_url="http://localhost:8000"
    )
    client.run()
```

### 6.3 Running Mock Client

```bash
# Install dependencies
pip install requests websocket-client

# Run mock client
python mock_vehicle_client.py

# Expected output:
# ✅ Logged in as VH-001
# ✅ Telemetry sent at 2025-12-19T10:30:45Z
# ✅ WebSocket connected
# ✅ Telemetry sent at 2025-12-19T10:30:46Z
# 📨 Received command: mission.dispatch
#   Mission ID: 12345
#   Waypoints: 3
```

---

## 7. Integration Testing

### 7.1 Test Scenario 1: Vehicle Connects and Sends Telemetry

**Steps:**
1. Start TVM server: `uvicorn main:app --reload`
2. Start mock vehicle: `python mock_vehicle_client.py`
3. Verify in server logs: Authentication success, telemetry received
4. Check database: `SELECT * FROM vehicle_telemetry ORDER BY timestamp DESC LIMIT 10;`

**Expected result:**
- Vehicle authenticated successfully
- Telemetry inserted into database every 1 second
- WebSocket connection established

### 7.2 Test Scenario 2: Dispatch Mission to Vehicle

**Steps:**
1. Mock vehicle running and connected
2. Use Postman or dashboard to create mission:
   ```
   POST /api/v1/missions
   Authorization: Bearer <user_token>
   {
     "vehicle_id": "VH-001",
     "waypoints": [
       {"x": 10.0, "y": 5.0, "heading": 0.0, "action": "navigate"},
       {"x": 15.0, "y": 8.0, "heading": 90.0, "action": "dock"}
     ]
   }
   ```
3. Check mock vehicle console for command reception
4. Verify mission status in database

**Expected result:**
- Mission created in database
- WebSocket command sent to vehicle
- Mock vehicle receives and acknowledges mission

### 7.3 Test Scenario 3: Emergency Stop

**Steps:**
1. Mock vehicle running
2. Send emergency stop via dashboard or API:
   ```
   POST /api/v1/vehicles/VH-001/emergency-stop
   ```
3. Check mock vehicle console

**Expected result:**
- Emergency stop command sent via WebSocket
- Mock vehicle receives `emergency.stop` message

---

## 8. Troubleshooting

### 8.1 Common Issues

**Issue: Vehicle login fails with 401**
```
Possible causes:
- api_key in database doesn't match
- vehicle_id not found in database
- JWT secret key mismatch

Solution:
1. Check database: SELECT * FROM vehicles WHERE vehicle_id = 'VH-001';
2. Verify api_key matches
3. Check server logs for detailed error
```

**Issue: WebSocket connection fails**
```
Possible causes:
- CORS not configured
- WebSocket not enabled in server
- Authentication failed

Solution:
1. Enable CORS in FastAPI:
   from fastapi.middleware.cors import CORSMiddleware
   app.add_middleware(CORSMiddleware, allow_origins=["*"])
2. Check WebSocket endpoint is registered
3. Verify JWT token is valid
```

**Issue: Telemetry not appearing in database**
```
Possible causes:
- Database connection failed
- Table doesn't exist
- SQL error

Solution:
1. Check server logs for SQL errors
2. Verify table exists: \dt in psql
3. Test database connection manually
```

---

## 9. Deliverables Checklist

### 9.1 Week 15 Deliverables (Integration Testing with Pankaj)

- [ ] All REST endpoints implemented and tested
- [ ] All WebSocket commands implemented
- [ ] Database schema complete with test data
- [ ] Mock vehicle client can connect and operate
- [ ] Basic fleet dashboard showing vehicle status
- [ ] Authentication working (vehicle + user)
- [ ] API documentation (Swagger/OpenAPI)
- [ ] Server deployed and accessible from network

### 9.2 Integration Testing Requirements

- [ ] Pankaj can connect real vehicle to your server
- [ ] Real vehicle can authenticate successfully
- [ ] Real vehicle telemetry appears in your database
- [ ] Dashboard shows real vehicle location/status
- [ ] You can dispatch mission to real vehicle
- [ ] Real vehicle receives and executes mission
- [ ] Mission status updates appear in dashboard

---

## 10. Contact & Support

**For questions about API specification:**
- Contact: Pankaj (Vehicle Software Team)
- Reference: TVM_API_SPECIFICATION.md, TVM_DATA_MODELS.md

**For technology choices (backend/database/frontend):**
- Your decision - choose what your team knows best
- Recommendation: Python + FastAPI + PostgreSQL + React

**Integration testing schedule:**
- Week 14-15: Joint testing with Pankaj
- Coordinate schedule via Slack/email

---

## Summary

**This guide provides:**
- ✅ Step-by-step implementation instructions
- ✅ Code examples (Python + FastAPI)
- ✅ Database schema (PostgreSQL)
- ✅ Mock vehicle client for testing
- ✅ Integration testing scenarios
- ✅ Troubleshooting guide

**Your next steps:**
1. Choose your technology stack
2. Set up project structure
3. Implement authentication (Week 1-2)
4. Implement REST endpoints (Week 3-8)
5. Implement WebSocket (Week 9-11)
6. Test with mock vehicle
7. Integrate with Pankaj's real vehicle (Week 14-15)

**Success criteria:**
- Mock vehicle client can connect and operate
- All API endpoints return correct responses
- Database stores telemetry and missions
- Dashboard shows vehicle status
- Ready for integration with real vehicle

---

**Document Status:** ✅ Complete
**Version:** 1.0
**Date:** December 19, 2025
**Prepared By:** Pankaj

**Good luck with implementation! 🚀**
