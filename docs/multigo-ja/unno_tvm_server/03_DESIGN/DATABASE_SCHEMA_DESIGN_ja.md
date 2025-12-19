# データベーススキーマ設計

**チーム:** Unno（フリート管理）
**日付:** 2025-12-16
**バージョン:** 1.0

## ER図

```
┌─────────┐       ┌──────────┐       ┌──────────────┐
│  users  │───────│  roles   │       │ reservations │
└────┬────┘   1:N └──────────┘       └──────┬───────┘
     │                                       │
     │ created_by                            │ created_by
     │                                       │
     └────────────┬──────────────────────────┘
                  │
                  │ 1:N
              ┌───▼────────┐
              │  missions  │
              └───┬────────┘
                  │ N:1
                  │
              ┌───▼──────┐
              │ vehicles │
              └──────────┘
```

## インデックス戦略

### 高トラフィッククエリ

```sql
-- クエリ: 車両のアクティブミッション取得
SELECT * FROM missions
WHERE vehicle_id = ? AND status IN ('pending', 'in_progress');

-- 最適インデックス:
CREATE INDEX idx_missions_vehicle_status ON missions(vehicle_id, status);

-- クエリ: ユーザーの最近の予約
SELECT * FROM reservations
WHERE created_by = ?
ORDER BY scheduled_time DESC
LIMIT 20;

-- 最適インデックス:
CREATE INDEX idx_reservations_user_time ON reservations(created_by, scheduled_time DESC);
```

### パーティショニング戦略

```sql
-- telemetry_logs: 月次パーティション
-- 3ヶ月の詳細データを保持、古いデータは集約

CREATE TABLE telemetry_logs_2025_12 PARTITION OF telemetry_logs
    FOR VALUES FROM ('2025-12-01') TO ('2026-01-01');

-- 自動パーティション作成（cronジョブ）
-- アーカイブ戦略: 3ヶ月後、日次サマリーに集約
```

---
**参照:** DATABASE_ARCHITECTURE.md
