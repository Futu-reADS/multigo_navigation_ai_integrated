# データベースアーキテクチャ

**プロジェクト:** 屋外車椅子輸送ロボット - マルチチームシステム
**文書タイプ:** データベースアーキテクチャ仕様
**チーム:** Unno（フリート管理）
**日付:** 2025-12-16
**バージョン:** 1.0

---

## データベーステクノロジー

**プライマリデータベース:** PostgreSQL 15
- ミッションクリティカルデータのACID準拠
- 柔軟なスキーマのためのJSON/JSONBサポート
- 全文検索機能
- 地理空間クエリのためのPostGIS拡張

**キャッシュ/セッションストア:** Redis 7.x
- セッション管理（JWTトークンブラックリスト）
- リアルタイム車両ステータスキャッシュ
- レート制限カウンター
- バックグラウンドジョブキュー（Bull）

---

## スキーマ概要

### コアテーブル

**users**
```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    full_name VARCHAR(255) NOT NULL,
    role_id UUID REFERENCES roles(id),
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW(),
    last_login TIMESTAMP,
    status VARCHAR(20) DEFAULT 'active'
);
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_role ON users(role_id);
```

**roles**
```sql
CREATE TABLE roles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    name VARCHAR(50) UNIQUE NOT NULL, -- admin, operator, nurse, caregiver
    permissions JSONB NOT NULL,
    created_at TIMESTAMP DEFAULT NOW()
);
```

**vehicles**
```sql
CREATE TABLE vehicles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    vehicle_id VARCHAR(20) UNIQUE NOT NULL, -- VEH-001
    name VARCHAR(100),
    status VARCHAR(20) DEFAULT 'offline', -- idle, navigating, charging, error
    battery_percent INTEGER,
    location GEOMETRY(POINT, 4326), -- PostGIS for lat/lon
    last_seen TIMESTAMP,
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);
CREATE INDEX idx_vehicles_status ON vehicles(status);
CREATE INDEX idx_vehicles_location ON vehicles USING GIST(location);
```

**missions**
```sql
CREATE TABLE missions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    vehicle_id UUID REFERENCES vehicles(id),
    type VARCHAR(50) NOT NULL, -- walking_assistance, medicine_delivery
    priority VARCHAR(20) DEFAULT 'routine', -- routine, urgent, emergency
    status VARCHAR(50) DEFAULT 'pending',
    pickup_location GEOMETRY(POINT, 4326),
    dropoff_location GEOMETRY(POINT, 4326),
    created_by UUID REFERENCES users(id),
    created_at TIMESTAMP DEFAULT NOW(),
    started_at TIMESTAMP,
    completed_at TIMESTAMP,
    metadata JSONB -- flexible additional data
);
CREATE INDEX idx_missions_vehicle ON missions(vehicle_id);
CREATE INDEX idx_missions_status ON missions(status);
CREATE INDEX idx_missions_created_at ON missions(created_at DESC);
```

**reservations**
```sql
CREATE TABLE reservations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    patient_name VARCHAR(255) NOT NULL,
    type VARCHAR(50) NOT NULL,
    priority VARCHAR(20) DEFAULT 'routine',
    scheduled_time TIMESTAMP NOT NULL,
    pickup_location GEOMETRY(POINT, 4326),
    dropoff_location GEOMETRY(POINT, 4326),
    recurring_pattern VARCHAR(50), -- null, daily, weekly
    recurring_until DATE,
    created_by UUID REFERENCES users(id),
    created_at TIMESTAMP DEFAULT NOW(),
    status VARCHAR(20) DEFAULT 'scheduled' -- scheduled, completed, cancelled
);
CREATE INDEX idx_reservations_scheduled_time ON reservations(scheduled_time);
CREATE INDEX idx_reservations_status ON reservations(status);
```

**telemetry_logs**（月次パーティション）
```sql
CREATE TABLE telemetry_logs (
    id BIGSERIAL,
    vehicle_id UUID REFERENCES vehicles(id),
    timestamp TIMESTAMP NOT NULL,
    location GEOMETRY(POINT, 4326),
    battery_percent INTEGER,
    speed FLOAT,
    status VARCHAR(50),
    metadata JSONB
) PARTITION BY RANGE (timestamp);

CREATE TABLE telemetry_logs_2025_12 PARTITION OF telemetry_logs
    FOR VALUES FROM ('2025-12-01') TO ('2026-01-01');
```

**audit_logs**
```sql
CREATE TABLE audit_logs (
    id BIGSERIAL PRIMARY KEY,
    user_id UUID REFERENCES users(id),
    action VARCHAR(100) NOT NULL,
    resource_type VARCHAR(50),
    resource_id UUID,
    ip_address INET,
    timestamp TIMESTAMP DEFAULT NOW(),
    details JSONB
);
CREATE INDEX idx_audit_user ON audit_logs(user_id);
CREATE INDEX idx_audit_timestamp ON audit_logs(timestamp DESC);
```

---

## Redisスキーマ

### キャッシュキー

```
vehicle:{vehicle_id}:status → JSON（5秒TTL）
vehicle:{vehicle_id}:location → JSON（5秒TTL）
user:session:{token} → user_id（24時間TTL）
rate_limit:{ip}:{endpoint} → count（1分TTL）
mission:{mission_id}:progress → JSON（10秒TTL）
```

### キュー（Bull）

```
queue:mission_scheduler → ミッションスケジューリングジョブ
queue:analytics → 分析計算ジョブ
queue:notifications → メール/SMS通知ジョブ
```

---

## バックアップと復旧

- **日次バックアップ:** UTC 02:00に完全PostgreSQLバックアップ
- **ポイントインタイムリカバリー:** WALアーカイブが有効
- **保持期間:** 30日間のバックアップ保持
- **目標復旧時間（RTO）:** <1時間
- **目標復旧時点（RPO）:** <15分

---

## 性能最適化

- **接続プーリング:** pgBouncer（100接続）
- **クエリ最適化:** 低速クエリにEXPLAIN ANALYZE
- **インデックス戦略:** 頻繁にクエリされる列の複合インデックス
- **パーティショニング:** telemetry_logsを月次でパーティション
- **バキューム:** 自動バキュームが有効

---

## セキュリティ

- **保存時暗号化:** AWS RDS暗号化
- **転送時暗号化:** SSL/TLS接続
- **行レベルセキュリティ:** マルチテナンシー用に有効（将来）
- **アクセス制御:** IAMベースのデータベースアクセス
- **監査ログ:** すべてのDDL/DML操作をログ記録

---

**文書終了**
