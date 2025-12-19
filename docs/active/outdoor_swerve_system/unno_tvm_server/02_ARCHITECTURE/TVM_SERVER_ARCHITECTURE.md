# TVM Server Architecture

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Document Type:** Fleet Management Architecture Specification
**Team:** Unno (Fleet Management)
**Date:** December 16, 2025
**Version:** 1.0

---

## System Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    TVM Server (Node.js)                     │
│                                                             │
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────────┐  │
│  │   REST API  │  │  WebSocket   │  │  Background     │  │
│  │   (Express) │  │  (Socket.io) │  │  Workers (Bull) │  │
│  └──────┬──────┘  └──────┬───────┘  └────────┬────────┘  │
│         │                 │                    │           │
│         └─────────────────┴────────────────────┘           │
│                           │                                │
│                  ┌────────▼─────────┐                      │
│                  │  Business Logic  │                      │
│                  │   (Services)     │                      │
│                  └────────┬─────────┘                      │
│                           │                                │
│         ┌─────────────────┼──────────────────┐            │
│         │                 │                  │             │
│    ┌────▼─────┐    ┌─────▼──────┐    ┌─────▼──────┐     │
│    │PostgreSQL│    │   Redis    │    │  S3 (logs) │     │
│    │  (main)  │    │  (cache)   │    │  (storage) │     │
│    └──────────┘    └────────────┘    └────────────┘     │
└─────────────────────────────────────────────────────────────┘
```

## Technology Stack

- **Runtime:** Node.js 20 LTS
- **Framework:** Express.js 4.x
- **Database:** PostgreSQL 15
- **Cache:** Redis 7.x
- **Queue:** Bull (Redis-based)
- **WebSocket:** Socket.io 4.x
- **Authentication:** JWT (jsonwebtoken)
- **Validation:** Joi
- **ORM:** Prisma

## Component Details

### REST API Layer
- Authentication middleware (JWT verification)
- Rate limiting (express-rate-limit)
- Request validation (Joi schemas)
- Error handling middleware
- API versioning (/api/v1/)

### WebSocket Layer
- Real-time vehicle telemetry
- Mission status updates
- Fleet dashboard updates
- Authentication via JWT

### Background Workers
- Mission scheduling
- Battery health monitoring
- Log aggregation
- Analytics computation

### Database Schema
- vehicles, missions, reservations, users, roles, logs
- Optimized indexes for queries
- Partitioning for large tables

### Caching Strategy
- Vehicle status: 5s TTL
- User sessions: 24h TTL
- Map data: 1h TTL

## Deployment Architecture

- Docker containers on AWS ECS
- Application Load Balancer
- Auto-scaling (2-10 instances)
- Multi-AZ deployment

---

**Total:** 35-40 requirements covered in TVM_SERVER_REQUIREMENTS.md
