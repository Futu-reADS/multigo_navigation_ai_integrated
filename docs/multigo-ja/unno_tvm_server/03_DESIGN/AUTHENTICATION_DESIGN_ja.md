# 認証設計

**チーム:** Unno（フリート管理）
**日付:** 2025-12-16
**バージョン:** 1.0

## JWTトークン構造

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

## 認証フロー

```
1. ユーザーがログイン送信（メール + パスワード）
2. サーバーが認証情報を検証（bcryptハッシュ比較）
3. サーバーがJWTトークン生成（24時間有効期限）
4. サーバーがトークン + リフレッシュトークン返却
5. クライアントがトークンを保存（セキュリティのためメモリ、localStorageは不可）
6. クライアントがAuthorizationヘッダーにトークン含める: "Bearer {token}"
7. サーバーミドルウェアが保護されたルートでトークン検証
8. トークン期限切れ → クライアントがリフレッシュトークン使用 → 新しいアクセストークン取得
```

## RBAC実装

```typescript
// middleware/rbac.ts
const rbacMiddleware = (requiredPermission: string) => {
  return (req, res, next) => {
    const {permissions} = req.user; // JWTペイロードから

    if (!permissions.includes(requiredPermission)) {
      return res.status(403).json({error: 'Forbidden'});
    }

    next();
  };
};

// 使用法
app.post('/api/v1/missions',
  authMiddleware,  // JWT検証
  rbacMiddleware('create_mission'),  // 権限チェック
  createMissionHandler
);
```

## パスワードセキュリティ

- **ハッシュ:** bcrypt（コストファクター12）
- **最小長:** 12文字
- **複雑性:** 大文字、小文字、数字、特殊文字
- **パスワードリセット:** 1時間有効期限トークン付きメールリンク

---
**参照:** USER_ROLE_MANAGEMENT_REQUIREMENTS.md（63要件）
