# TVMサーバー設計

**チーム:** Unno（フリート管理）
**日付:** 2025-12-16
**バージョン:** 1.0

## APIルート（Express.js）

### 認証ルート
```javascript
POST /api/v1/auth/login          → JWTトークン
POST /api/v1/auth/refresh        → リフレッシュトークン
POST /api/v1/auth/logout         → トークン無効化
```

### 車両ルート
```javascript
GET    /api/v1/vehicles           → 全車両リスト
GET    /api/v1/vehicles/:id       → 車両詳細
POST   /api/v1/vehicles/:id/command → コマンド送信
```

### ミッションルート
```javascript
POST   /api/v1/missions           → ミッション作成
GET    /api/v1/missions           → ミッションリスト（フィルター付き）
GET    /api/v1/missions/:id       → ミッション詳細
PATCH  /api/v1/missions/:id       → ミッション更新
DELETE /api/v1/missions/:id       → ミッションキャンセル
```

## ミドルウェアスタック

1. **helmet** - セキュリティヘッダー
2. **cors** - クロスオリジンリソース共有
3. **express-rate-limit** - レート制限（IPあたり15分に100リクエスト）
4. **morgan** - HTTPリクエストログ
5. **express-validator** - リクエスト検証
6. **auth-middleware** - JWT検証
7. **rbac-middleware** - ロールベースアクセス制御

## WebSocketイベント（Socket.io）

```javascript
// クライアント → サーバー
socket.emit('subscribe_vehicle', {vehicle_id})
socket.emit('send_command', {vehicle_id, command})

// サーバー → クライアント
socket.emit('vehicle_telemetry', {vehicle_id, location, battery, status})
socket.emit('mission_update', {mission_id, status, progress})
socket.emit('alert', {severity, message})
```

## サービスレイヤー

```javascript
// services/VehicleService.js
class VehicleService {
  async getVehicleStatus(vehicleId) {
    // 最初にRedisキャッシュを確認
    // ミスの場合、PostgreSQLクエリ + 結果をキャッシュ
  }

  async sendCommand(vehicleId, command) {
    // コマンド検証
    // WebSocket経由で車両に公開
    // audit_logsテーブルにログ
  }
}
```

## バックグラウンドワーカー（Bull Queue）

```javascript
// workers/mission-scheduler.js
missionQueue.process(async (job) => {
  const {reservation_id} = job.data;
  // 予約時刻に予約 → ミッション変換
  // 最適車両割り当て
  // ディスパッチコマンド送信
});

// workers/analytics.js
analyticsQueue.process(async (job) => {
  // 日次/週次統計計算
  // 分析テーブルに保存
});
```

---
**参照:** TVM_SERVER_ARCHITECTURE.md、TVM_API_SPECIFICATION.md
