# Authentication Design

**Team:** Unno (Fleet Management)
**Date:** 2025-12-16
**Version:** 1.0

## JWT Token Structure

```json
{
  "header": {
    "alg": "HS256",
    "typ": "JWT"
  },
  "payload": {
    "user_id": "uuid",
    "email": "user@example.com",
    "role": "operator",
    "permissions": ["view_dashboard", "create_mission"],
    "iat": 1702723200,
    "exp": 1702809600
  },
  "signature": "..."
}
```

## Authentication Flow

```
1. User submits login (email + password)
2. Server verifies credentials (bcrypt hash comparison)
3. Server generates JWT token (24h expiration)
4. Server returns token + refresh token
5. Client stores token (memory, not localStorage for security)
6. Client includes token in Authorization header: "Bearer {token}"
7. Server middleware validates token on protected routes
8. Token expires → Client uses refresh token → Get new access token
```

## RBAC Implementation

```typescript
// middleware/rbac.ts
const rbacMiddleware = (requiredPermission: string) => {
  return (req, res, next) => {
    const {permissions} = req.user; // From JWT payload
    
    if (!permissions.includes(requiredPermission)) {
      return res.status(403).json({error: 'Forbidden'});
    }
    
    next();
  };
};

// Usage
app.post('/api/v1/missions', 
  authMiddleware,  // Verify JWT
  rbacMiddleware('create_mission'),  // Check permission
  createMissionHandler
);
```

## Password Security

- **Hashing:** bcrypt (cost factor 12)
- **Min length:** 12 characters
- **Complexity:** Uppercase, lowercase, number, special char
- **Password reset:** Email link with 1-hour expiration token

---
**References:** USER_ROLE_MANAGEMENT_REQUIREMENTS.md (63 req)
