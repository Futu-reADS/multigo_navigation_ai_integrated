# フリートソフトウェア開発ガイド

**チーム:** Unno（フリート管理）
**日付:** 2025-12-16
**バージョン:** 1.0

## プロジェクト構造

```
fleet_management/
├── tvm-server/          # Node.jsバックエンド
│   ├── src/
│   │   ├── routes/      # Expressルート
│   │   ├── services/    # ビジネスロジック
│   │   ├── models/      # Prismaモデル
│   │   └── middleware/  # Auth、RBACなど
│   ├── prisma/
│   │   └── schema.prisma
│   └── tests/
├── fleet-ui/            # Reactフロントエンド
│   ├── src/
│   │   ├── components/
│   │   ├── features/    # Reduxスライス
│   │   ├── pages/
│   │   └── hooks/
│   └── tests/
```

## 開発コマンド

### バックエンド（TVMサーバー）
```bash
npm run dev          # 開発サーバー起動（nodemon）
npm run test         # テスト実行（Jest）
npm run lint         # ESLint
npm run prisma:migrate  # データベースマイグレーション実行
```

### フロントエンド（フリートUI）
```bash
npm run dev          # 開発サーバー起動（Vite）
npm run test         # テスト実行（Vitest）
npm run build        # 本番ビルド
npm run lint         # ESLint
```

## コード標準

- **TypeScript:** ストリクトモード有効
- **リンティング:** Airbnb構成のESLint
- **フォーマット:** Prettier
- **テスト:** Jest（バックエンド）、Vitest（フロントエンド）

---
**参照:** TVM_SERVER_ARCHITECTURE.md、FLEET_UI_ARCHITECTURE.md
