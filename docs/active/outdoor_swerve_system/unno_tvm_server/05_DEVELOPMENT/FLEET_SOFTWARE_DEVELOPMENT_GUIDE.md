# Fleet Software Development Guide

**Team:** Unno (Fleet Management)
**Date:** 2025-12-16
**Version:** 1.0

## Project Structure

```
fleet_management/
├── tvm-server/          # Node.js backend
│   ├── src/
│   │   ├── routes/      # Express routes
│   │   ├── services/    # Business logic
│   │   ├── models/      # Prisma models
│   │   └── middleware/  # Auth, RBAC, etc.
│   ├── prisma/
│   │   └── schema.prisma
│   └── tests/
├── fleet-ui/            # React frontend
│   ├── src/
│   │   ├── components/
│   │   ├── features/    # Redux slices
│   │   ├── pages/
│   │   └── hooks/
│   └── tests/
```

## Development Commands

### Backend (TVM Server)
```bash
npm run dev          # Start dev server (nodemon)
npm run test         # Run tests (Jest)
npm run lint         # ESLint
npm run prisma:migrate  # Run database migrations
```

### Frontend (Fleet UI)
```bash
npm run dev          # Start dev server (Vite)
npm run test         # Run tests (Vitest)
npm run build        # Production build
npm run lint         # ESLint
```

## Code Standards

- **TypeScript:** Strict mode enabled
- **Linting:** ESLint with Airbnb config
- **Formatting:** Prettier
- **Testing:** Jest (backend), Vitest (frontend)

---
**References:** TVM_SERVER_ARCHITECTURE.md, FLEET_UI_ARCHITECTURE.md
