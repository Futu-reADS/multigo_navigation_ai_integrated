# TVM Server Design

**Team:** Unno (Fleet Management)  
**Date:** 2025-12-16  
**Version:** 1.0

## API Routes (Express.js)

### Authentication Routes
```javascript
POST /api/v1/auth/login          → JWT token
POST /api/v1/auth/refresh        → Refresh token
POST /api/v1/auth/logout         → Invalidate token
```

### Vehicle Routes
```javascript
GET    /api/v1/vehicles           → List all vehicles
GET    /api/v1/vehicles/:id       → Vehicle details
POST   /api/v1/vehicles/:id/command → Send command
```

### Mission Routes
```javascript
POST   /api/v1/missions           → Create mission
GET    /api/v1/missions           → List missions (filtered)
GET    /api/v1/missions/:id       → Mission details
PATCH  /api/v1/missions/:id       → Update mission
DELETE /api/v1/missions/:id       → Cancel mission
```

## Middleware Stack

1. **helmet** - Security headers
2. **cors** - Cross-origin resource sharing
3. **express-rate-limit** - Rate limiting (100 req/15min per IP)
4. **morgan** - HTTP request logging
5. **express-validator** - Request validation
6. **auth-middleware** - JWT verification
7. **rbac-middleware** - Role-based access control

## WebSocket Events (Socket.io)

```javascript
// Client → Server
socket.emit('subscribe_vehicle', {vehicle_id})
socket.emit('send_command', {vehicle_id, command})

// Server → Client
socket.emit('vehicle_telemetry', {vehicle_id, location, battery, status})
socket.emit('mission_update', {mission_id, status, progress})
socket.emit('alert', {severity, message})
```

## Service Layer

```javascript
// services/VehicleService.js
class VehicleService {
  async getVehicleStatus(vehicleId) {
    // Check Redis cache first
    // If miss, query PostgreSQL + cache result
  }
  
  async sendCommand(vehicleId, command) {
    // Validate command
    // Publish to vehicle via WebSocket
    // Log to audit_logs table
  }
}
```

## Background Workers (Bull Queue)

```javascript
// workers/mission-scheduler.js
missionQueue.process(async (job) => {
  const {reservation_id} = job.data;
  // Convert reservation → mission at scheduled time
  // Assign optimal vehicle
  // Send dispatch command
});

// workers/analytics.js
analyticsQueue.process(async (job) => {
  // Compute daily/weekly statistics
  // Store in analytics tables
});
```

---
**References:** TVM_SERVER_ARCHITECTURE.md, TVM_API_SPECIFICATION.md
