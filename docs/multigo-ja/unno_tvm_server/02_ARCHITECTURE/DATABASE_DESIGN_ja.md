# データベース設計

**プロジェクト:** 屋外車椅子搬送ロボット - マルチチームシステム
**文書タイプ:** アーキテクチャ仕様
**ステータス:** アクティブ
**バージョン:** 1.0
**最終更新:** 2025-12-17
**オーナー:** チーム2 - TVMバックエンド

---

## 目次

1. [エグゼクティブサマリー](#1-エグゼクティブサマリー)
2. [データベーステクノロジー](#2-データベーステクノロジー)
3. [スキーマ概要](#3-スキーマ概要)
4. [コアテーブル](#4-コアテーブル)
5. [リレーションシップと制約](#5-リレーションシップと制約)
6. [インデックス戦略](#6-インデックス戦略)
7. [データタイプと検証](#7-データタイプと検証)
8. [クエリパターン](#8-クエリパターン)
9. [パーティショニングとシャーディング](#9-パーティショニングとシャーディング)
10. [データライフサイクル管理](#10-データライフサイクル管理)
11. [パフォーマンス最適化](#11-パフォーマンス最適化)
12. [バックアップとリカバリー](#12-バックアップとリカバリー)
13. [セキュリティとアクセス制御](#13-セキュリティとアクセス制御)
14. [マイグレーション戦略](#14-マイグレーション戦略)

---

## 1. エグゼクティブサマリー

### 1.1 目的

この文書は、TVM（Transport Vehicle Management）システムのデータベース設計を定義します。データベースには以下が格納されます：
- **居住者:** 車椅子搬送サービスを利用する人々
- **車両:** 自律ロボットのフリート
- **ミッション:** 搬送リクエスト（ピックアップ/ドロップオフ）
- **テレメトリ:** 車両センサーデータ、バッテリー、位置情報
- **ユーザー:** オペレーター、看護師、介護者
- **監査ログ:** セキュリティとコンプライアンスログ

### 1.2 データベース要件

**機能要件:**
- 居住者プロファイルの保存（PII、医療情報、設定）
- 車両の追跡（ステータス、位置、バッテリー、メンテナンス）
- ミッションの管理（キュー、実行、履歴）
- テレメトリのログ記録（車両からのリアルタイム更新）
- ユーザー認証と認可（RBAC）
- 監査証跡（全データアクセス、変更）

**非機能要件:**
- **可用性:** 99.5%稼働率（マルチAZデプロイ）
- **パフォーマンス:**
  - 読み取りレイテンシ: <50ms（95パーセンタイル）
  - 書き込みレイテンシ: <100ms（95パーセンタイル）
  - スループット: 1000クエリ/秒
- **スケーラビリティ:** 50車両、500居住者、10,000ミッション/日をサポート
- **データ整合性:** ACIDトランザクション、外部キー制約
- **セキュリティ:** 保存時の暗号化（AES-256）、転送時の暗号化（TLS 1.3）
- **コンプライアンス:** GDPR、PIPEDA、HIPAA（データ保持、プライバシー）

### 1.3 設計原則

1. **正規化:** 冗長性を避けるため第3正規形（3NF）
2. **非正規化（選択的）:** 読み取り集約的なクエリ用（例：居住者名付きミッションリスト）
3. **論理削除:** ハード削除の代わりに`deleted_at`タイムスタンプを使用（監査証跡）
4. **タイムスタンプ:** すべてのテーブルに`created_at`、`updated_at`カラムを持つ
5. **UUID:** プライマリキーにUUIDを使用（分散、非連続）
6. **制約:** データベースレベルでデータ整合性を強制（外部キー、チェック制約）

---

## 2. データベーステクノロジー

### 2.1 RDBMS選択: PostgreSQL

**なぜPostgreSQLか?**
- **ACIDコンプライアンス:** 強い整合性、トランザクション
- **JSONサポート:** ネイティブJSONBタイプ（テレメトリ用の柔軟なスキーマ）
- **高度な機能:** CTE、ウィンドウ関数、全文検索
- **地理空間:** PostGIS拡張（位置クエリ）
- **成熟:** 25年以上、実績豊富
- **オープンソース:** ライセンスコストなし

**検討した代替案:**
| データベース | 長所 | 短所 | 決定 |
|----------|------|------|----------|
| **MySQL** | 人気、良好なパフォーマンス | 高度なSQL機能が少ない | ❌ PostgreSQLの方が機能豊富 |
| **MongoDB** | 柔軟なスキーマ、水平スケーリング | ACID非対応（単一ドキュメントのみ）、JOIN高コスト | ❌ 強い整合性が必要 |
| **CockroachDB** | 分散、PostgreSQL互換 | 複雑な運用、高コスト | ❌ MVP用には単一リージョンで十分 |

**決定:** PostgreSQL 15+（最新安定版）

### 2.2 デプロイ

**AWS RDS PostgreSQL:**
- **インスタンス:** db.t3.large（2 vCPU、8 GB RAM）、MVP用、必要に応じてスケールアップ
- **ストレージ:** 100 GB gp3 SSD（最大1 TBまで自動スケール）
- **マルチAZ:** はい（高可用性、自動フェイルオーバー）
- **暗号化:** 保存時（AES-256）、転送時（TLS 1.3）
- **バックアップ:** 日次自動バックアップ（7日間保持）、手動スナップショット

---

## 3. スキーマ概要

### 3.1 エンティティリレーションシップ図

```
┌────────────┐         ┌─────────────┐
│  Residents │         │  Vehicles   │
│  (PII)     │         │  (Fleet)    │
└─────┬──────┘         └──────┬──────┘
      │                       │
      │ 1:N                   │ 1:N
      ↓                       ↓
┌──────────────────────────────────┐
│           Missions               │
│  (Transport Requests)            │
└───────────┬──────────────────────┘
            │ 1:N
            ↓
┌──────────────────────────────────┐
│        Mission Events            │
│  (State Changes)                 │
└──────────────────────────────────┘

┌────────────┐         ┌─────────────┐
│   Users    │         │   Roles     │
│  (Operators│◄────────┤  (RBAC)     │
│   Nurses)  │   N:M   │             │
└────────────┘         └─────────────┘

┌──────────────────────────────────┐
│         Telemetry                │
│  (Vehicle Sensor Data)           │
│  - Time-series data              │
└──────────────────────────────────┘

┌──────────────────────────────────┐
│         Audit Logs               │
│  (Security, Compliance)          │
└──────────────────────────────────┘
```

### 3.2 テーブルカテゴリー

| カテゴリー | テーブル | 目的 |
|----------|--------|---------|
| **コアエンティティ** | residents, vehicles, missions | ビジネスオブジェクト |
| **運用** | mission_events, vehicle_status | リアルタイム状態 |
| **テレメトリ** | telemetry, vehicle_logs | センサーデータ、診断 |
| **ユーザー管理** | users, roles, permissions, user_roles | 認証、RBAC |
| **設定** | facilities, locations, maps | システム設定 |
| **監査とコンプライアンス** | audit_logs, data_access_logs | セキュリティ、GDPR |

---

## 4. コアテーブル

### 4.1 `residents` テーブル

**目的:** 居住者（乗客）プロファイルの保存

```sql
CREATE TABLE residents (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    facility_id UUID NOT NULL REFERENCES facilities(id),

    -- Personal Information (PII)
    first_name VARCHAR(100) NOT NULL,
    last_name VARCHAR(100) NOT NULL,
    date_of_birth DATE NOT NULL,
    gender VARCHAR(20),  -- 'male', 'female', 'non-binary', 'prefer-not-to-say'

    -- Contact Information
    phone VARCHAR(20),
    emergency_contact_name VARCHAR(200),
    emergency_contact_phone VARCHAR(20),

    -- Medical Information (encrypted, HIPAA if US)
    medical_notes TEXT,  -- Encrypted at app layer (e.g., allergies, mobility restrictions)
    wheelchair_type VARCHAR(50),  -- 'manual', 'powered', 'transport_chair'
    requires_assistance BOOLEAN DEFAULT FALSE,

    -- Preferences
    preferred_language VARCHAR(10) DEFAULT 'en',  -- 'en', 'ja', 'fr', etc.

    -- Photo (optional, for staff identification)
    photo_url VARCHAR(500),

    -- Privacy & Consent
    gdpr_consent BOOLEAN DEFAULT FALSE,
    gdpr_consent_date TIMESTAMP WITH TIME ZONE,

    -- Metadata
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    deleted_at TIMESTAMP WITH TIME ZONE,  -- Soft delete

    -- Constraints
    CONSTRAINT check_gender CHECK (gender IN ('male', 'female', 'non-binary', 'prefer-not-to-say', NULL))
);

-- Indexes
CREATE INDEX idx_residents_facility ON residents(facility_id) WHERE deleted_at IS NULL;
CREATE INDEX idx_residents_last_name ON residents(last_name) WHERE deleted_at IS NULL;
CREATE INDEX idx_residents_deleted ON residents(deleted_at);  -- For soft delete queries

-- Trigger: Update updated_at on row change
CREATE TRIGGER update_residents_updated_at BEFORE UPDATE ON residents
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
```

**注記:**
- `medical_notes`はアプリケーション層で暗号化（フィールドレベルの粒度のため、データベース暗号化ではない）
- `gdpr_consent`はGDPRコンプライアンスに必要（PII処理の明示的同意）
- 論理削除: アクティブな居住者には`deleted_at IS NULL`

### 4.2 `vehicles` テーブル

**目的:** 車両フリート情報の保存

```sql
CREATE TABLE vehicles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    facility_id UUID NOT NULL REFERENCES facilities(id),

    -- Vehicle Identification
    name VARCHAR(100) NOT NULL UNIQUE,  -- Human-readable (e.g., "Vehicle Alpha")
    serial_number VARCHAR(100) UNIQUE,  -- Hardware serial number

    -- Vehicle Type
    model VARCHAR(100),  -- e.g., "Swerve-Drive-V1"
    firmware_version VARCHAR(50),  -- e.g., "v1.2.3"

    -- Status
    status VARCHAR(50) NOT NULL DEFAULT 'idle',
    -- 'idle', 'navigating', 'docking', 'charging', 'maintenance', 'error'

    -- Current State
    current_mission_id UUID REFERENCES missions(id),
    current_location JSONB,  -- {"lat": 35.6, "lon": 139.7, "zone": "Building A"}
    battery_level INTEGER CHECK (battery_level >= 0 AND battery_level <= 100),  -- Percentage
    last_heartbeat TIMESTAMP WITH TIME ZONE,  -- Last communication with TVM

    -- Capabilities
    max_payload_kg INTEGER DEFAULT 150,  -- Max weight capacity
    max_speed_mps NUMERIC(3,1) DEFAULT 2.0,  -- Max speed in m/s

    -- Maintenance
    last_maintenance_date DATE,
    next_maintenance_due DATE,
    total_distance_km NUMERIC(10,2) DEFAULT 0.0,  -- Odometer
    total_missions INTEGER DEFAULT 0,

    -- Metadata
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    deleted_at TIMESTAMP WITH TIME ZONE,

    -- Constraints
    CONSTRAINT check_status CHECK (status IN ('idle', 'navigating', 'docking', 'charging', 'maintenance', 'error'))
);

-- Indexes
CREATE INDEX idx_vehicles_facility ON vehicles(facility_id) WHERE deleted_at IS NULL;
CREATE INDEX idx_vehicles_status ON vehicles(status) WHERE deleted_at IS NULL;
CREATE INDEX idx_vehicles_last_heartbeat ON vehicles(last_heartbeat);  -- For offline detection
CREATE INDEX idx_vehicles_name ON vehicles(name);

-- Trigger: Update updated_at
CREATE TRIGGER update_vehicles_updated_at BEFORE UPDATE ON vehicles
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
```

**注記:**
- `status`は列挙型風（柔軟性のため真の列挙型ではなくチェック制約）
- `current_location`はJSONB（柔軟、マイグレーションなしでフィールド追加可能）
- `last_heartbeat`はオフライン車両の検出に使用（5分以上古い場合にアラート）

### 4.3 `missions` テーブル

**目的:** 搬送リクエスト（ピックアップ/ドロップオフ）の保存

```sql
CREATE TABLE missions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    facility_id UUID NOT NULL REFERENCES facilities(id),

    -- Mission Participants
    resident_id UUID NOT NULL REFERENCES residents(id),
    vehicle_id UUID REFERENCES vehicles(id),  -- Assigned vehicle (NULL if unassigned)
    created_by_user_id UUID NOT NULL REFERENCES users(id),  -- Who created the mission

    -- Mission Details
    pickup_location VARCHAR(200) NOT NULL,  -- Human-readable (e.g., "Room 101")
    pickup_coordinates POINT,  -- PostGIS point (lat, lon)
    dropoff_location VARCHAR(200) NOT NULL,
    dropoff_coordinates POINT,

    -- Priority
    priority VARCHAR(20) NOT NULL DEFAULT 'normal',
    -- 'low', 'normal', 'high', 'urgent'

    -- Status
    status VARCHAR(50) NOT NULL DEFAULT 'pending',
    -- 'pending', 'assigned', 'in_progress', 'completed', 'cancelled', 'failed'

    -- Timestamps
    scheduled_at TIMESTAMP WITH TIME ZONE,  -- Requested pickup time (NULL for ASAP)
    assigned_at TIMESTAMP WITH TIME ZONE,  -- When vehicle assigned
    started_at TIMESTAMP WITH TIME ZONE,  -- When vehicle started mission
    completed_at TIMESTAMP WITH TIME ZONE,  -- When mission completed

    -- Notes
    notes TEXT,  -- Additional instructions (e.g., "Bring oxygen tank")
    failure_reason TEXT,  -- If status='failed', reason

    -- Metadata
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    deleted_at TIMESTAMP WITH TIME ZONE,

    -- Constraints
    CONSTRAINT check_priority CHECK (priority IN ('low', 'normal', 'high', 'urgent')),
    CONSTRAINT check_status CHECK (status IN ('pending', 'assigned', 'in_progress', 'completed', 'cancelled', 'failed'))
);

-- Indexes
CREATE INDEX idx_missions_facility ON missions(facility_id) WHERE deleted_at IS NULL;
CREATE INDEX idx_missions_resident ON missions(resident_id);
CREATE INDEX idx_missions_vehicle ON missions(vehicle_id);
CREATE INDEX idx_missions_status ON missions(status) WHERE status IN ('pending', 'assigned', 'in_progress');
CREATE INDEX idx_missions_created_at ON missions(created_at DESC);  -- For recent missions query
CREATE INDEX idx_missions_scheduled_at ON missions(scheduled_at) WHERE status = 'pending';

-- GiST index for geospatial queries (PostGIS)
CREATE INDEX idx_missions_pickup_location ON missions USING GIST (pickup_coordinates);
CREATE INDEX idx_missions_dropoff_location ON missions USING GIST (dropoff_coordinates);

-- Trigger: Update updated_at
CREATE TRIGGER update_missions_updated_at BEFORE UPDATE ON missions
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
```

**注記:**
- `status`ライフサイクル: pending → assigned → in_progress → completed（またはcancelled/failed）
- `pickup_coordinates`と`dropoff_coordinates`は地理空間クエリ用のPostGIS `POINT`
- `created_at DESC`のインデックスは「最近のミッション」クエリ用

### 4.4 `mission_events` テーブル

**目的:** すべてのミッション状態変更をログ記録（監査証跡、タイムライン）

```sql
CREATE TABLE mission_events (
    id BIGSERIAL PRIMARY KEY,  -- Use BIGINT for high-volume inserts
    mission_id UUID NOT NULL REFERENCES missions(id) ON DELETE CASCADE,
    vehicle_id UUID REFERENCES vehicles(id),

    -- Event Type
    event_type VARCHAR(50) NOT NULL,
    -- 'created', 'assigned', 'started', 'pickup_arrived', 'pickup_completed',
    -- 'in_transit', 'dropoff_arrived', 'dropoff_completed', 'completed', 'cancelled', 'failed'

    -- Event Details
    details JSONB,  -- Flexible field for event-specific data
    -- Example: {"reason": "resident not ready", "retry_count": 2}

    -- Metadata
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    created_by_user_id UUID REFERENCES users(id)  -- Who triggered event (if manual)
);

-- Indexes
CREATE INDEX idx_mission_events_mission ON mission_events(mission_id);
CREATE INDEX idx_mission_events_created_at ON mission_events(created_at DESC);
CREATE INDEX idx_mission_events_type ON mission_events(event_type);
```

**注記:**
- 大量挿入用に`BIGSERIAL`（ミッションごとに5-10イベント生成）
- イベント固有データ用の柔軟な`details` JSONB
- 論理削除なし（不変の監査証跡）

### 4.5 `telemetry` テーブル

**目的:** 時系列車両センサーデータの保存

```sql
CREATE TABLE telemetry (
    id BIGSERIAL PRIMARY KEY,
    vehicle_id UUID NOT NULL REFERENCES vehicles(id),

    -- Timestamp
    timestamp TIMESTAMP WITH TIME ZONE NOT NULL,

    -- Location
    latitude NUMERIC(10,7),  -- -90 to 90
    longitude NUMERIC(10,7),  -- -180 to 180
    heading NUMERIC(5,2),  -- 0-360 degrees

    -- Motion
    speed_mps NUMERIC(4,2),  -- meters per second
    acceleration_mps2 NUMERIC(4,2),

    -- Battery
    battery_level INTEGER,  -- Percentage 0-100
    battery_voltage NUMERIC(5,2),  -- Volts
    battery_current NUMERIC(6,2),  -- Amperes

    -- Sensors
    lidar_status VARCHAR(20),  -- 'ok', 'degraded', 'error'
    camera_status VARCHAR(20),
    imu_status VARCHAR(20),

    -- System
    cpu_usage INTEGER,  -- Percentage 0-100
    memory_usage INTEGER,  -- MB
    disk_usage INTEGER,  -- Percentage 0-100

    -- Errors/Warnings
    error_codes VARCHAR(100)[],  -- Array of error codes (e.g., ['VEH-NAV-001', 'VEH-LID-001'])

    -- Metadata
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Partitioning: By month (TimescaleDB or native partitioning)
-- See Section 9.1 for partitioning strategy

-- Indexes
CREATE INDEX idx_telemetry_vehicle_timestamp ON telemetry(vehicle_id, timestamp DESC);
CREATE INDEX idx_telemetry_timestamp ON telemetry(timestamp DESC);

-- Retention policy: Delete telemetry older than 90 days (see Section 10)
```

**注記:**
- 大容量テーブル: 10 Hz × 10車両 × 8時間/日 = 2.88M行/日
- **パーティショニングが必要**（月または週単位、セクション9参照）
- 最適化された時系列クエリ用にTimescaleDB拡張を検討
- **保持期間:** 90日（PRIVACY_REQUIREMENTS.md REQ-PRIV-012）

### 4.6 `users` テーブル

**目的:** ユーザーアカウント（オペレーター、看護師、介護者、管理者）の保存

```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    facility_id UUID NOT NULL REFERENCES facilities(id),

    -- Credentials
    email VARCHAR(255) NOT NULL UNIQUE,
    password_hash VARCHAR(255) NOT NULL,  -- bcrypt hash

    -- Personal Information
    first_name VARCHAR(100) NOT NULL,
    last_name VARCHAR(100) NOT NULL,
    phone VARCHAR(20),

    -- Account Status
    status VARCHAR(20) NOT NULL DEFAULT 'active',
    -- 'active', 'inactive', 'locked', 'pending_verification'
    email_verified BOOLEAN DEFAULT FALSE,
    email_verification_token VARCHAR(255),

    -- MFA (Multi-Factor Authentication)
    mfa_enabled BOOLEAN DEFAULT FALSE,
    mfa_secret VARCHAR(255),  -- TOTP secret (encrypted)

    -- Password Reset
    password_reset_token VARCHAR(255),
    password_reset_expires TIMESTAMP WITH TIME ZONE,

    -- Last Login
    last_login_at TIMESTAMP WITH TIME ZONE,
    last_login_ip INET,

    -- Metadata
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    deleted_at TIMESTAMP WITH TIME ZONE,

    -- Constraints
    CONSTRAINT check_status CHECK (status IN ('active', 'inactive', 'locked', 'pending_verification'))
);

-- Indexes
CREATE UNIQUE INDEX idx_users_email ON users(LOWER(email)) WHERE deleted_at IS NULL;
CREATE INDEX idx_users_facility ON users(facility_id) WHERE deleted_at IS NULL;

-- Trigger: Update updated_at
CREATE TRIGGER update_users_updated_at BEFORE UPDATE ON users
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
```

**注記:**
- bcryptを使用した`password_hash`（コストファクター12、SECURITY_REQUIREMENTS.md REQ-SEC-006参照）
- `email`大文字小文字を区別しないユニークインデックス（LOWER(email)）
- `mfa_secret`はアプリケーション層で暗号化（MFA有効時）

### 4.7 `roles` テーブル（RBAC）

**目的:** ユーザーロールの定義（管理者、オペレーター、看護師、介護者）

```sql
CREATE TABLE roles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    name VARCHAR(50) NOT NULL UNIQUE,  -- 'admin', 'operator', 'nurse', 'caregiver', 'guest'
    description TEXT,

    -- Metadata
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Seed data (insert default roles)
INSERT INTO roles (name, description) VALUES
    ('admin', 'Full system access, user management, config changes'),
    ('operator', 'Monitor fleet, dispatch missions, view telemetry'),
    ('nurse', 'Request transport, view mission status, cancel own missions'),
    ('caregiver', 'Request transport for assigned residents only'),
    ('maintenance', 'View vehicle diagnostics, initiate maintenance mode'),
    ('guest', 'View public status dashboard only');
```

### 4.8 `permissions` テーブル

**目的:** 詳細な権限の定義（例："mission:create"、"vehicle:view"）

```sql
CREATE TABLE permissions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    resource VARCHAR(50) NOT NULL,  -- 'mission', 'vehicle', 'resident', 'user'
    action VARCHAR(50) NOT NULL,  -- 'create', 'read', 'update', 'delete', 'list'
    description TEXT,

    -- Metadata
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),

    CONSTRAINT unique_permission UNIQUE (resource, action)
);

-- Seed data
INSERT INTO permissions (resource, action, description) VALUES
    ('mission', 'create', 'Create new missions'),
    ('mission', 'read', 'View mission details'),
    ('mission', 'update', 'Update mission (e.g., cancel)'),
    ('mission', 'delete', 'Delete mission'),
    ('mission', 'list', 'List all missions'),
    ('vehicle', 'read', 'View vehicle details'),
    ('vehicle', 'control', 'Control vehicle (e-stop, reboot)'),
    ('resident', 'create', 'Add new residents'),
    ('resident', 'read', 'View resident profiles (PII access)'),
    ('user', 'create', 'Create new users'),
    ('user', 'delete', 'Delete users');
```

### 4.9 `role_permissions` テーブル（多対多）

**目的:** ロールに権限を割り当て

```sql
CREATE TABLE role_permissions (
    role_id UUID NOT NULL REFERENCES roles(id) ON DELETE CASCADE,
    permission_id UUID NOT NULL REFERENCES permissions(id) ON DELETE CASCADE,

    PRIMARY KEY (role_id, permission_id)
);

-- Seed data: Example for 'operator' role
INSERT INTO role_permissions (role_id, permission_id)
SELECT
    (SELECT id FROM roles WHERE name = 'operator'),
    id
FROM permissions
WHERE (resource = 'mission' AND action IN ('create', 'read', 'list'))
   OR (resource = 'vehicle' AND action = 'read');
```

### 4.10 `user_roles` テーブル（多対多）

**目的:** ユーザーにロールを割り当て

```sql
CREATE TABLE user_roles (
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    role_id UUID NOT NULL REFERENCES roles(id) ON DELETE CASCADE,

    -- Metadata
    assigned_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    assigned_by_user_id UUID REFERENCES users(id),  -- Who assigned this role

    PRIMARY KEY (user_id, role_id)
);

-- Indexes
CREATE INDEX idx_user_roles_user ON user_roles(user_id);
CREATE INDEX idx_user_roles_role ON user_roles(role_id);
```

### 4.11 `audit_logs` テーブル

**目的:** セキュリティとコンプライアンスのための監査証跡

```sql
CREATE TABLE audit_logs (
    id BIGSERIAL PRIMARY KEY,

    -- Who
    user_id UUID REFERENCES users(id),  -- NULL for system events
    user_email VARCHAR(255),  -- Denormalized (user might be deleted)

    -- What
    action VARCHAR(100) NOT NULL,  -- 'login', 'mission_create', 'resident_view', 'config_change'
    resource_type VARCHAR(50),  -- 'mission', 'resident', 'vehicle', 'user'
    resource_id UUID,  -- ID of affected resource

    -- Details
    details JSONB,  -- Flexible field for action-specific data
    -- Example: {"old_status": "pending", "new_status": "in_progress"}

    -- When & Where
    timestamp TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    ip_address INET,
    user_agent TEXT,

    -- Result
    success BOOLEAN NOT NULL,  -- TRUE = success, FALSE = failure (e.g., unauthorized)
    failure_reason TEXT  -- If success=FALSE, reason
);

-- Indexes
CREATE INDEX idx_audit_logs_user ON audit_logs(user_id);
CREATE INDEX idx_audit_logs_timestamp ON audit_logs(timestamp DESC);
CREATE INDEX idx_audit_logs_action ON audit_logs(action);
CREATE INDEX idx_audit_logs_resource ON audit_logs(resource_type, resource_id);

-- Retention: 7 years (PRIVACY_REQUIREMENTS.md REQ-PRIV-048)
```

**注記:**
- `user_email`は非正規化（ユーザーが削除された場合でもアクションを実行したユーザーがわかる）
- 柔軟な監査データ用の`details` JSONB
- 大容量: 保持ポリシーが必要（パーティションまたは古いデータのアーカイブ）

---

## 5. リレーションシップと制約

### 5.1 外部キーリレーションシップ

```sql
-- Mission → Resident (who is being transported)
missions.resident_id → residents.id

-- Mission → Vehicle (which vehicle assigned)
missions.vehicle_id → vehicles.id

-- Mission → User (who created mission)
missions.created_by_user_id → users.id

-- Mission Events → Mission
mission_events.mission_id → missions.id (CASCADE on delete)

-- Telemetry → Vehicle
telemetry.vehicle_id → vehicles.id

-- Users → Facility
users.facility_id → facilities.id

-- User Roles → Users, Roles
user_roles.user_id → users.id (CASCADE on delete)
user_roles.role_id → roles.id (CASCADE on delete)
```

### 5.2 チェック制約

```sql
-- Battery level: 0-100%
ALTER TABLE vehicles ADD CONSTRAINT check_battery_level
    CHECK (battery_level >= 0 AND battery_level <= 100);

-- Mission priority: enum-like
ALTER TABLE missions ADD CONSTRAINT check_priority
    CHECK (priority IN ('low', 'normal', 'high', 'urgent'));

-- Vehicle status: enum-like
ALTER TABLE vehicles ADD CONSTRAINT check_status
    CHECK (status IN ('idle', 'navigating', 'docking', 'charging', 'maintenance', 'error'));
```

### 5.3 ユニーク制約

```sql
-- Vehicle name unique per facility
CREATE UNIQUE INDEX idx_vehicles_name_unique ON vehicles(facility_id, name) WHERE deleted_at IS NULL;

-- User email unique (case-insensitive)
CREATE UNIQUE INDEX idx_users_email_unique ON users(LOWER(email)) WHERE deleted_at IS NULL;
```

---

## 6. インデックス戦略

### 6.1 インデックスタイプ

| インデックスタイプ | ユースケース | 例 |
|------------|----------|---------|
| **B-Tree（デフォルト）** | 等価、範囲クエリ | `CREATE INDEX idx_missions_created_at ON missions(created_at)` |
| **GiST（地理空間）** | PostGISポイント/ジオメトリクエリ | `CREATE INDEX idx_missions_pickup USING GIST (pickup_coordinates)` |
| **GIN（JSONB）** | JSONBキー/値クエリ | `CREATE INDEX idx_telemetry_details ON telemetry USING GIN (details)` |
| **パーシャルインデックス** | 行のサブセットをインデックス化 | `CREATE INDEX idx_active_missions ON missions(status) WHERE status='in_progress'` |

### 6.2 クエリパフォーマンス目標

**目標:**
- **ポイントクエリ（IDによる）:** <5ms
- **リストクエリ（ページネーション）:** <50ms
- **集計クエリ（ダッシュボード）:** <200ms

**ツール:**
- スロークエリのプロファイルに`EXPLAIN ANALYZE`を使用
- スロークエリログの監視（>1秒のクエリ）
- クエリレイテンシ用のGrafanaダッシュボード（p50、p95、p99）

---

## 7. データタイプと検証

### 7.1 データタイプの選択

| ドメイン | PostgreSQLタイプ | 根拠 |
|--------|----------------|-----------|
| **プライマリキー** | UUID | 分散フレンドリー、非連続 |
| **タイムスタンプ** | TIMESTAMP WITH TIME ZONE | タイムゾーン対応、DST問題回避 |
| **金額** | NUMERIC(10,2) | 正確な小数、浮動小数点エラーなし |
| **地理位置** | POINT (PostGIS) | 地理空間クエリ（距離、境界ボックス） |
| **列挙型風** | VARCHAR + CHECK | 柔軟性（マイグレーションなしで値追加可能） |
| **柔軟なデータ** | JSONB | 半構造化データ、インデックス化 |

### 7.2 検証ルール

**アプリケーション層検証:**
- メールフォーマット: 正規表現`/^[^\s@]+@[^\s@]+\.[^\s@]+$/`
- 電話フォーマット: E.164（例: +1-555-123-4567）
- 座標: 緯度-90〜90、経度-180〜180

**データベース層検証:**
- チェック制約（列挙型、範囲）
- 外部キー制約（参照整合性）
- NOT NULL制約（必須フィールド）

---

## 8. クエリパターン

### 8.1 一般的なクエリ

#### クエリ1: アクティブミッションのリスト（ダッシュボード）

```sql
SELECT
    m.id,
    m.status,
    m.priority,
    m.pickup_location,
    m.dropoff_location,
    r.first_name || ' ' || r.last_name AS resident_name,
    v.name AS vehicle_name,
    m.created_at
FROM missions m
JOIN residents r ON m.resident_id = r.id
LEFT JOIN vehicles v ON m.vehicle_id = v.id
WHERE m.status IN ('pending', 'assigned', 'in_progress')
  AND m.deleted_at IS NULL
ORDER BY
    CASE m.priority
        WHEN 'urgent' THEN 1
        WHEN 'high' THEN 2
        WHEN 'normal' THEN 3
        WHEN 'low' THEN 4
    END,
    m.created_at ASC
LIMIT 50;
```

**パフォーマンス:** <50ms（`status`、`created_at`にインデックス）

#### クエリ2: 車両フリートステータス

```sql
SELECT
    v.id,
    v.name,
    v.status,
    v.battery_level,
    v.current_location->>'zone' AS current_zone,
    v.last_heartbeat,
    CASE
        WHEN v.last_heartbeat > NOW() - INTERVAL '5 minutes' THEN 'online'
        ELSE 'offline'
    END AS connectivity_status
FROM vehicles v
WHERE v.deleted_at IS NULL
ORDER BY v.name;
```

**パフォーマンス:** <10ms（小規模テーブル、<100車両）

#### クエリ3: 車両の最近のテレメトリ

```sql
SELECT
    timestamp,
    latitude,
    longitude,
    speed_mps,
    battery_level,
    error_codes
FROM telemetry
WHERE vehicle_id = $1  -- Parameter: vehicle UUID
  AND timestamp > NOW() - INTERVAL '1 hour'
ORDER BY timestamp DESC
LIMIT 1000;
```

**パフォーマンス:** <100ms（`vehicle_id, timestamp`にインデックス）

#### クエリ4: ミッションタイムライン（監査証跡）

```sql
SELECT
    me.event_type,
    me.details,
    me.created_at,
    u.email AS triggered_by
FROM mission_events me
LEFT JOIN users u ON me.created_by_user_id = u.id
WHERE me.mission_id = $1  -- Parameter: mission UUID
ORDER BY me.created_at ASC;
```

**パフォーマンス:** <20ms（`mission_id`にインデックス）

### 8.2 最適化テクニック

**1. 非正規化（選択的）:**
- `missions`テーブルに`resident_name`を保存（すべてのクエリでのJOINを回避）
- トレードオフ: 更新の複雑さ vs. 読み取りパフォーマンス

**2. マテリアライズドビュー:**
```sql
CREATE MATERIALIZED VIEW mission_summary AS
SELECT
    m.id,
    m.status,
    m.priority,
    r.first_name || ' ' || r.last_name AS resident_name,
    v.name AS vehicle_name,
    m.created_at
FROM missions m
JOIN residents r ON m.resident_id = r.id
LEFT JOIN vehicles v ON m.vehicle_id = v.id
WHERE m.deleted_at IS NULL;

-- Refresh periodically (e.g., every 5 minutes)
REFRESH MATERIALIZED VIEW CONCURRENTLY mission_summary;
```

**3. コネクションプーリング:**
- PgBouncerを使用してコネクションをプール化（コネクションオーバーヘッド削減）
- プールサイズ: 50コネクション（1000クライアントコネクション用）

---

## 9. パーティショニングとシャーディング

### 9.1 テーブルパーティショニング（テレメトリ）

**問題:** `telemetry`テーブルが1億行以上に成長（2.88M行/日 × 90日保持 = 259M行）

**解決策:** 月単位でパーティショニング（PostgreSQLネイティブパーティショニング）

```sql
-- Parent table (partitioned)
CREATE TABLE telemetry (
    id BIGSERIAL,
    vehicle_id UUID NOT NULL,
    timestamp TIMESTAMP WITH TIME ZONE NOT NULL,
    ...
    PRIMARY KEY (id, timestamp)
) PARTITION BY RANGE (timestamp);

-- Child partitions (one per month)
CREATE TABLE telemetry_2025_12 PARTITION OF telemetry
    FOR VALUES FROM ('2025-12-01') TO ('2026-01-01');

CREATE TABLE telemetry_2026_01 PARTITION OF telemetry
    FOR VALUES FROM ('2026-01-01') TO ('2026-02-01');

-- Automatically create partitions (using pg_partman extension)
SELECT partman.create_parent('public.telemetry', 'timestamp', 'native', 'monthly');
```

**利点:**
- クエリパフォーマンス: 関連パーティションのみスキャン（例: 先月）
- 保持: 古いパーティションを削除（高速、DELETEスキャンなし）

### 9.2 シャーディング（将来、必要な場合）

**シャーディング戦略:** `facility_id`による（マルチテナント）

**シャーディングのタイミング:**
- 単一データベースが負荷を処理できない（>10,000書き込み/秒）
- データサイズが5 TBを超える

**実装:**
- **Citus（PostgreSQL拡張）:** 分散PostgreSQL
- **アプリケーションレベルシャーディング:** `facility_id`に基づいてクエリをルーティング

**複雑性:** 高い運用オーバーヘッド、必要でない限り回避

---

## 10. データライフサイクル管理

### 10.1 保持ポリシー

| テーブル | 保持期間 | アクション |
|-------|-----------|--------|
| `missions` | 3年 | 論理削除、1年後にS3 Glacierにアーカイブ |
| `mission_events` | 3年 | ミッションと共にアーカイブ |
| `telemetry` | 90日 | 削除（パーティション削除） |
| `vehicle_logs` | 90日 | 削除 |
| `audit_logs` | 7年 | 年単位でパーティション、1年後にアーカイブ |
| `residents` | アクティブ + 3年 | 論理削除、3年後に匿名化 |

### 10.2 自動クリーンアップジョブ

```sql
-- Cron job (using pg_cron extension) or external scheduler
-- Run daily at 2 AM UTC

-- Delete telemetry older than 90 days
DELETE FROM telemetry WHERE timestamp < NOW() - INTERVAL '90 days';

-- Or drop partition (faster)
DROP TABLE telemetry_2025_09;  -- If current month is December, drop September

-- Anonymize old residents (GDPR right to erasure)
UPDATE residents
SET
    first_name = 'Deleted',
    last_name = 'User #' || id,
    email = NULL,
    phone = NULL,
    medical_notes = NULL
WHERE deleted_at < NOW() - INTERVAL '3 years';
```

---

## 11. パフォーマンス最適化

### 11.1 クエリ最適化

**スロークエリ分析:**
```sql
-- Enable slow query log (log queries >1 second)
ALTER SYSTEM SET log_min_duration_statement = 1000;  -- milliseconds

-- View slow queries
SELECT query, calls, mean_exec_time, max_exec_time
FROM pg_stat_statements
ORDER BY mean_exec_time DESC
LIMIT 20;
```

**一般的な修正:**
1. **インデックスの欠落:** フィルター/結合カラムにインデックスを追加
2. **シーケンシャルスキャン:** `EXPLAIN ANALYZE`をチェック、インデックスが使用されていることを確認
3. **大きなOFFSET:** OFFSETの代わりにカーソルベースのページネーションを使用

### 11.2 VacuumとAnalyze

**Auto-Vacuum:** デフォルトで有効だが、高トラフィックテーブル用に調整

```sql
-- Tune auto-vacuum for telemetry table (high insert rate)
ALTER TABLE telemetry SET (
    autovacuum_vacuum_scale_factor = 0.05,  -- Vacuum when 5% of rows are dead tuples
    autovacuum_analyze_scale_factor = 0.02  -- Analyze when 2% of rows changed
);
```

**手動Vacuum:**
```sql
-- Full vacuum (reclaims disk space, locks table)
VACUUM FULL telemetry;  -- Run during maintenance window

-- Analyze (updates statistics for query planner)
ANALYZE missions;
```

### 11.3 コネクションプーリング

**PgBouncer設定:**
```ini
[databases]
tvm = host=tvm-db.internal port=5432 dbname=tvm

[pgbouncer]
pool_mode = transaction  # Transaction-level pooling
max_client_conn = 1000   # Max client connections
default_pool_size = 50   # Max connections to PostgreSQL
```

**結果:** 1000クライアントコネクション → 50 PostgreSQLコネクション（オーバーヘッド削減）

---

## 12. バックアップとリカバリー

### 12.1 バックアップ戦略

**AWS RDS自動バックアップ:**
- **頻度:** 日次（午前1時UTC）
- **保持:** 7日（RDSデフォルト）
- **タイプ:** フルスナップショット + トランザクションログ（ポイントインタイムリカバリー）

**手動スナップショット:**
- **重要な変更前:** スキーママイグレーション前に手動スナップショットを取得
- **保持:** 無期限（手動削除まで）

**オフサイトバックアップ:**
- **S3 Glacier:** 週次フルバックアップを別リージョンに複製（eu-west-1）
- **保持:** 7年（コンプライアンス）

### 12.2 リカバリー手順

**ポイントインタイムリカバリー（PITR）:**
```bash
# Restore to specific time (e.g., before accidental DELETE)
aws rds restore-db-instance-to-point-in-time \
  --source-db-instance-identifier tvm-production-primary \
  --target-db-instance-identifier tvm-restored-2025-12-17-14-25 \
  --restore-time 2025-12-17T14:25:00Z
```

**災害復旧:** DISASTER_RECOVERY_PLAN.md参照

---

## 13. セキュリティとアクセス制御

### 13.1 データベースユーザー

| ユーザー | 権限 | 目的 |
|------|------------|---------|
| `tvm_admin` | SUPERUSER | データベース管理（マイグレーション、メンテナンス） |
| `tvm_app` | CONNECT、SELECT、INSERT、UPDATE、DELETE（全テーブル） | アプリケーションユーザー（TVMサーバー） |
| `tvm_readonly` | CONNECT、SELECT（全テーブル） | 読み取り専用ユーザー（分析、レポート） |

**アプリケーションユーザーの作成:**
```sql
CREATE USER tvm_app WITH PASSWORD 'secure-password-xyz123';
GRANT CONNECT ON DATABASE tvm TO tvm_app;
GRANT SELECT, INSERT, UPDATE, DELETE ON ALL TABLES IN SCHEMA public TO tvm_app;
GRANT USAGE, SELECT ON ALL SEQUENCES IN SCHEMA public TO tvm_app;

-- Future tables (grant automatically)
ALTER DEFAULT PRIVILEGES IN SCHEMA public GRANT SELECT, INSERT, UPDATE, DELETE ON TABLES TO tvm_app;
```

### 13.2 行レベルセキュリティ（RLS）

**ユースケース:** マルチテナント分離（ユーザーは自施設のデータのみアクセス可能）

```sql
-- Enable RLS on residents table
ALTER TABLE residents ENABLE ROW LEVEL SECURITY;

-- Policy: Users can only see residents in their facility
CREATE POLICY residents_facility_isolation ON residents
    USING (facility_id = current_setting('app.current_facility_id')::UUID);

-- Application sets facility_id at session start
SET app.current_facility_id = '123e4567-e89b-12d3-a456-426614174000';
```

**注記:** オーバーヘッドを追加、厳密な分離が必要な場合のみ使用（例: SaaSマルチテナント）

---

## 14. マイグレーション戦略

### 14.1 スキーマバージョニング

**ツール:** FlywayまたはLiquibase（またはPython用Alembic、Node.js用TypeORMなどのネイティブORMマイグレーション）

**マイグレーションファイル:**
```
migrations/
├── V001__create_residents_table.sql
├── V002__create_vehicles_table.sql
├── V003__create_missions_table.sql
├── V004__add_mission_priority.sql
└── V005__add_telemetry_partitions.sql
```

**命名規則:** `V<version>__<description>.sql`

### 14.2 マイグレーションベストプラクティス

**1. 後方互換性:**
- DEFAULT値でカラムを追加（古いコードが動作し続ける）
- 別のマイグレーションでカラムを削除（コード更新後）

**2. ステージングでテスト:**
- ステージングで最初にマイグレーションを実行
- アプリケーションが正常に動作することを確認

**3. ロールバック計画:**
- ロールバック手順を文書化（例: `V004_rollback.sql`）
- 複雑なマイグレーションにはトランザクションを使用

**4. ゼロダウンタイムマイグレーション:**
- 新しいカラムを追加（デフォルト値付き）
- 古いカラムと新しいカラムの両方に書き込むコードをデプロイ
- 新しいカラムをバックフィル（バックグラウンドジョブ）
- 新しいカラムのみから読み取るコードをデプロイ
- 古いカラムを削除

---

## 15. 監視とアラート

### 15.1 データベースメトリクス

**主要メトリクス（Prometheus）:**
- **コネクション数:** `pg_stat_database_numbackends`
- **クエリレート:** `pg_stat_database_xact_commit` / 時間
- **スロークエリ:** カスタムエクスポーター（>1秒のクエリ）
- **ディスク使用量:** `pg_database_size`
- **レプリケーションラグ:** `pg_stat_replication_replay_lag`

**アラート:**
- コネクション数が最大値の>80% → スケールアップまたはコネクションリーク調査
- スロークエリレート >10/分 → クエリパフォーマンス調査
- ディスク使用量 >90% → ストレージ拡張またはクリーンアップ
- レプリケーションラグ >5分 → ネットワーク確認、スタンバイパフォーマンス調査

### 15.2 Grafanaダッシュボード

**パネル:**
1. **クエリレート（QPS）:** ラインチャート、過去24時間
2. **スロークエリ:** テーブル、上位10スロークエリ
3. **コネクションプール:** ゲージ、アクティブ/アイドル/待機コネクション
4. **ディスク使用量:** ゲージ、使用率パーセンテージ
5. **レプリケーションラグ:** ラインチャート、セカンダリリージョンラグ

---

## 付録A: ヘルパー関数

### A.1 `update_updated_at_column()` 関数

```sql
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;
```

**使用法:** 行更新時に`updated_at`カラムを自動更新するトリガー

---

## 付録B: シードデータスクリプト

```sql
-- Create default facility
INSERT INTO facilities (id, name, address, timezone)
VALUES ('123e4567-e89b-12d3-a456-426614174000', 'Main Campus', '123 Healthcare Ave', 'America/New_York');

-- Create default admin user
INSERT INTO users (facility_id, email, password_hash, first_name, last_name, status)
VALUES (
    '123e4567-e89b-12d3-a456-426614174000',
    'admin@example.com',
    '$2a$12$examplehash',  -- bcrypt hash of 'changeme'
    'Admin',
    'User',
    'active'
);

-- Assign admin role
INSERT INTO user_roles (user_id, role_id)
SELECT
    (SELECT id FROM users WHERE email = 'admin@example.com'),
    (SELECT id FROM roles WHERE name = 'admin');
```

---

## 承認

| 役割 | 氏名 | 署名 | 日付 |
|------|------|-----------|------|
| **プロジェクトリード** | 前田さん | __________ | _______ |
| **TVMリード** | 海野さん | __________ | _______ |
| **データベースアーキテクト** | TBD | __________ | _______ |

---

**文書メタデータ:**
- **バージョン:** 1.0
- **作成日:** 2025-12-17
- **次回レビュー:** 2026-03-17（四半期レビュー）
- **オーナー:** チーム2 - TVMバックエンド

---

**関連文書:**
- `DEPLOYMENT_ARCHITECTURE.md` - RDSデプロイ、バックアップ
- `SECURITY_ARCHITECTURE.md` - データベースセキュリティ、アクセス制御
- `PRIVACY_REQUIREMENTS.md` - データ保持、GDPRコンプライアンス
- `DISASTER_RECOVERY_PLAN.md` - バックアップとリカバリー手順

---

*文書の終わり*
