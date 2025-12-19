# Fleet UI Architecture

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Document Type:** Fleet Management UI Architecture
**Team:** Unno (Fleet Management)
**Date:** December 16, 2025
**Version:** 1.0

---

## Frontend Architecture

```
┌─────────────────────────────────────────────────────────┐
│              React 18 SPA (TypeScript)                  │
│                                                         │
│  ┌─────────────────────────────────────────────────┐  │
│  │           Component Hierarchy                   │  │
│  │                                                 │  │
│  │  App                                            │  │
│  │   ├─ AuthProvider (context)                    │  │
│  │   ├─ Router (react-router-dom)                 │  │
│  │   │   ├─ Dashboard (map + vehicle list)        │  │
│  │   │   ├─ Missions (active + history)           │  │
│  │   │   ├─ Reservations (calendar view)          │  │
│  │   │   ├─ Vehicles (management)                 │  │
│  │   │   ├─ Users (admin only)                    │  │
│  │   │   └─ Settings                              │  │
│  │   └─ WebSocketProvider (real-time updates)     │  │
│  └─────────────────────────────────────────────────┘  │
│                                                         │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────┐ │
│  │ State Mgmt   │  │  API Layer   │  │  UI Library │ │
│  │ Redux Toolkit│  │  RTK Query   │  │  Material-UI│ │
│  └──────────────┘  └──────────────┘  └─────────────┘ │
└─────────────────────────────────────────────────────────┘
```

## Technology Stack

- **Framework:** React 18 (TypeScript)
- **State Management:** Redux Toolkit + RTK Query
- **Routing:** react-router-dom v6
- **UI Framework:** Material-UI (MUI) v5
- **Maps:** Leaflet.js + react-leaflet
- **Charts:** Recharts
- **Real-time:** Socket.io-client
- **Build:** Vite
- **Testing:** Vitest + React Testing Library

## Key Features

### Dashboard
- Real-time vehicle positions on map
- Fleet statistics widgets
- Recent alerts panel
- Quick actions toolbar

### Map Component
- Vehicle markers with status colors
- Navigation paths visualization
- Geofence boundaries
- Zoom/pan controls
- Layer toggles

### Mission Management
- Create mission wizard
- Active missions list with progress
- Mission history with filters
- Mission detail modal

### Responsive Design
- Desktop: 1920×1080 (primary)
- Tablet: 1024×768
- Mobile: 375×667 (limited functionality)

## State Management

### Redux Slices
- `auth` - User authentication state
- `vehicles` - Vehicle list and status
- `missions` - Mission data
- `reservations` - Reservation calendar
- `ui` - UI state (modals, loading, etc.)

### RTK Query APIs
- `vehiclesApi` - Vehicle CRUD operations
- `missionsApi` - Mission management
- `usersApi` - User management (admin)

## Build and Deployment

- Vite build (optimized bundle)
- Static hosting on AWS S3 + CloudFront
- CI/CD via GitHub Actions
- Environment-specific configs (.env files)

---

**Total:** 109 requirements covered in FLEET_UI_REQUIREMENTS.md
