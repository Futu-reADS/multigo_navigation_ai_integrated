# TVM API仕様 - 車両 ↔ フリート管理サーバー

**ドキュメントID:** INT-TVM-API-001
**バージョン:** 1.0
**日付:** 2025年12月16日
**ステータス:** ドラフト - 重要（第1週優先度1）
**分類:** 内部

**責任チーム:**
- **車両クライアント:** Pankaj（ソフトウェア - 車両側）
- **TVMサーバー:** Unno（ソフトウェア - フリート管理側）

**目的:** 自律車両とTVM（Total Vehicle Management）サーバー間の正確なAPI契約を定義し、モックインターフェースを使用した並行開発を可能にする。

---

## ドキュメント管理

| バージョン | 日付 | 著者 | 変更内容 |
|---------|------|---------|---------|
| 1.0 | 2025-12-16 | Pankaj + Unno | マルチチーム開発のための初期API仕様 |

---

## 目次

1. [はじめに](#1-はじめに)
2. [API概要](#2-api概要)
3. [認証と認可](#3-認証と認可)
4. [車両 → TVMエンドポイント（REST）](#4-車両--tvmエンドポイントrest)
5. [TVM → 車両コマンド（WebSocket）](#5-tvm--車両コマンドwebsocket)
6. [データモデル](#6-データモデル)
7. [エラー処理](#7-エラー処理)
8. [オフライン動作](#8-オフライン動作)
9. [並行開発用モックインターフェース](#9-並行開発用モックインターフェース)
10. [テストと検証](#10-テストと検証)
11. [デプロイメントとバージョニング](#11-デプロイメントとバージョニング)

---

## 1. はじめに

### 1.1 目的

このドキュメントは以下の間の**完全なAPI契約**を定義します:
- **車両クライアント**（自律車椅子輸送ロボット - ROS 2ベース）
- **TVMサーバー**（Total Vehicle Managementフリート管理システム）

**重要な依存関係:**
- 車両ソフトウェアはこの仕様なしにTVMサーバーと通信できない
- TVMサーバーはこの仕様なしに車両データを受信できない
- 両チームは指定された通りに正確に実装する必要がある

**承認必須:** 開発開始前にPankaj（車両）とUnno（TVMサーバー）の両方がこのドキュメントを承認する必要があります。

---

### 1.2 範囲

**範囲内:**
- ✅ 車両テレメトリーアップロード用REST API（位置、ステータス、バッテリー、エラー）
- ✅ リアルタイムTVM→車両コマンド用WebSocket API（派遣、キャンセル、緊急停止）
- ✅ 認証メカニズム（JWTトークン）
- ✅ データモデル（すべてのメッセージのJSONスキーマ）
- ✅ エラーコードと処理
- ✅ オフライン動作の振る舞い
- ✅ 並行開発用モックインターフェース

**範囲外:**
- ❌ TVMサーバー内部実装の詳細
- ❌ 車両ソフトウェア内部アーキテクチャ
- ❌ データベーススキーマ（TVMサーバー内部）
- ❌ ROS 2トピック（車両内部）
- ❌ ログアップロードシステム（別個のtvm_uploadデーモン）

---

### 1.3 関連ドキュメント

- **TVM_DATA_MODELS.md** - すべてのメッセージの詳細なJSONスキーマ
- **HARDWARE_SOFTWARE_INTERFACES.md** - ROS 2インターフェース仕様
- **VEHICLE_TVM_CLIENT_REQUIREMENTS.md** - 車両側クライアント要件
- **TVM_SERVER_REQUIREMENTS.md** - サーバー側要件
- **TVM_INTEGRATION_REQUIREMENTS.md** - 統合要件と受け入れ基準

---

## 2. API概要

### 2.1 技術スタック

**通信プロトコル:**
- **REST API (HTTPS):** 車両 → TVMテレメトリーアップロード（単方向プッシュ）
- **WebSocket (WSS):** TVM → 車両コマンド（双方向、リアルタイム）

**データ形式:**
- **JSON:** すべてのメッセージペイロード
- **UTF-8エンコーディング:** すべてのテキスト
- **ISO 8601 (UTC):** すべてのタイムスタンプ

**認証:**
- **JWT（JSON Webトークン）:** Bearerトークン認証
- **トークン有効期限:** 24時間（設定可能）
- **トークン更新:** 有効期限前の自動更新

**APIバージョニング:**
- **URLバージョニング:** `/api/v1/...`
- **現在のバージョン:** v1
- **下位互換性:** バージョンアップグレード後6ヶ月間維持

---

### 2.2 ベースURL

**本番環境:**
```
REST APIベースURL: https://tvm.example.com/api/v1
WebSocket URL:     wss://tvm.example.com/ws/v1
```

**ステージング:**
```
REST APIベースURL: https://tvm-staging.example.com/api/v1
WebSocket URL:     wss://tvm-staging.example.com/ws/v1
```

**開発（ローカル）:**
```
REST APIベースURL: http://localhost:8000/api/v1
WebSocket URL:     ws://localhost:8000/ws/v1
```

---

### 2.3 レート制限

| エンドポイントカテゴリ | レート制限 | バースト | 注記 |
|-------------------|------------|-------|-------|
| 位置更新 | 車両あたり1リクエスト/秒 | 5 | 通常のテレメトリー |
| ステータス更新 | 車両あたり1リクエスト/秒 | 5 | 状態変更 |
| バッテリー更新 | 車両あたり1リクエスト/5秒 | 2 | より低い頻度 |
| エラー報告 | 車両あたり10リクエスト/秒 | 20 | 高優先度、レート制限をバイパス |
| コマンド（WebSocket） | 制限なし | N/A | リアルタイムクリティカル |

**レート制限ヘッダー:**
```
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1702742400
```

**レート制限超過レスポンス:**
```json
{
  "error": "rate_limit_exceeded",
  "message": "リクエストが多すぎます。15秒お待ちください。",
  "retry_after": 15
}
```

---

## 3. 認証と認可

### 3.1 車両登録

**初期セットアップ（1回限り）:**
1. 車両がTVMサーバーデータベースに登録される（手動プロセス）
2. 車両に一意の`vehicle_id`が割り当てられる（例: `wheelchair_001`）
3. 車両にAPI認証情報が割り当てられる（client_id + client_secret）

**車両認証情報:**
```json
{
  "vehicle_id": "wheelchair_001",
  "client_id": "veh_abc123xyz456",
  "client_secret": "secret_789def012ghi"
}
```

**保存:** 認証情報は車両に安全に保存される（暗号化された設定ファイル）

---

### 3.2 トークン取得

**エンドポイント:** `POST /api/v1/auth/token`

**リクエスト:**
```json
{
  "client_id": "veh_abc123xyz456",
  "client_secret": "secret_789def012ghi",
  "grant_type": "client_credentials"
}
```

**レスポンス（成功）:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 86400,
  "vehicle_id": "wheelchair_001"
}
```

**レスポンス（エラー）:**
```json
{
  "error": "invalid_client",
  "error_description": "無効なクライアント認証情報"
}
```

**トークン有効期限:** 24時間（86400秒）

---

### 3.3 アクセストークンの使用

**すべてのREST APIリクエストに以下を含める必要があります:**
```
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**例:**
```bash
curl -X POST https://tvm.example.com/api/v1/vehicle/wheelchair_001/location \
  -H "Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..." \
  -H "Content-Type: application/json" \
  -d '{
    "latitude": 35.681236,
    "longitude": 139.767125,
    "timestamp": "2025-12-16T10:30:45.123Z"
  }'
```

---

### 3.4 トークン更新

**車両クライアントは以下を行う必要があります:**
- トークン有効期限を監視（`expires_in`をチェック）
- 有効期限の1時間前にトークンを更新（プロアクティブ）
- トークンが期限切れの場合は即座に再認証（リアクティブ）

**更新ロジック:**
```python
# 疑似コード
def ensure_valid_token():
    if current_time > (token_acquired_at + expires_in - 3600):
        # トークンが1時間以内に期限切れ、プロアクティブに更新
        new_token = acquire_token(client_id, client_secret)
        update_token(new_token)
    return current_token
```

---

### 3.5 WebSocket認証

**WebSocket接続はURLにトークンを含める必要があります:**
```
wss://tvm.example.com/ws/v1/vehicle/wheelchair_001?token=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**接続拒否（無効なトークン）:**
```json
{
  "type": "error",
  "code": "auth_failed",
  "message": "無効または期限切れのトークン"
}
```

**接続クローズコード:** 4001（認証失敗）

---

## 4. 車両 → TVMエンドポイント（REST）

### 4.1 位置更新

**エンドポイント:** `POST /api/v1/vehicle/{vehicle_id}/location`

**頻度:** ナビゲーション中1 Hz（1秒に1回）、アイドル時0.2 Hz（5秒に1回）

**リクエストヘッダー:**
```
Authorization: Bearer {access_token}
Content-Type: application/json
```

**リクエストボディ:**
```json
{
  "timestamp": "2025-12-16T10:30:45.123Z",
  "location": {
    "latitude": 35.681236,
    "longitude": 139.767125,
    "altitude": 45.5,
    "heading": 90.5,
    "speed": 0.8,
    "accuracy": 0.15
  }
}
```

**フィールド説明:**
| フィールド | 型 | 単位 | 範囲 | 必須 | 説明 |
|-------|------|------|-------|----------|-------------|
| timestamp | string (ISO 8601 UTC) | - | - | ✅ | 測定時刻 |
| latitude | float | 度 | -90〜90 | ✅ | WGS84緯度 |
| longitude | float | 度 | -180〜180 | ✅ | WGS84経度 |
| altitude | float | メートル | - | ⚠️ オプション | 海抜高度（利用可能な場合） |
| heading | float | 度 | 0-360 | ✅ | 真北方位（0=北、90=東） |
| speed | float | m/s | 0-2.0 | ✅ | 現在の速度 |
| accuracy | float | メートル | 0-1.0 | ✅ | 位置精度（±メートル） |

**レスポンス（成功）:**
```json
{
  "status": "ok",
  "received_at": "2025-12-16T10:30:45.234Z"
}
```

**レスポンス（エラー）:**
```json
{
  "error": "invalid_location",
  "message": "緯度が範囲外",
  "field": "location.latitude",
  "received_value": 95.123
}
```

**HTTPステータスコード:**
- `200 OK` - 位置更新成功
- `400 Bad Request` - 無効なデータ形式または範囲外の値
- `401 Unauthorized` - 無効または期限切れのトークン
- `429 Too Many Requests` - レート制限超過
- `500 Internal Server Error` - サーバーエラー

---

### 4.2 ステータス更新

**エンドポイント:** `POST /api/v1/vehicle/{vehicle_id}/status`

**頻度:** 状態変更時（イベント駆動）、または10秒ごと（ハートビート）

**リクエストボディ:**
```json
{
  "timestamp": "2025-12-16T10:30:45.123Z",
  "state": "navigating",
  "mission_id": "mission_12345",
  "current_waypoint": "room_201",
  "passengers_onboard": true,
  "operational_hours": true,
  "details": {
    "progress_percent": 65.5,
    "eta_seconds": 180,
    "distance_remaining_meters": 145.2
  }
}
```

**車両状態（列挙型）:**
| 状態 | 説明 | 遷移元 | 遷移先 |
|-------|-------------|------------------|----------------|
| `idle` | 車両停止、ミッションなし | charging, error | navigating, charging |
| `navigating` | 目的地へ積極的に移動中 | idle, docking | docking, idle, error |
| `docking` | 車椅子とドッキング中 | navigating | idle, error |
| `charging` | 充電ステーションで充電中 | idle | idle |
| `error` | システムエラー、注意が必要 | any | idle（回復後） |
| `emergency_stop` | 緊急停止が作動 | any | idle（リセット後） |

---

### 4.3 バッテリー更新

**エンドポイント:** `POST /api/v1/vehicle/{vehicle_id}/battery`

**頻度:** 5秒ごと（負荷を減らすため低頻度）

**リクエストボディ:**
```json
{
  "timestamp": "2025-12-16T10:30:45.123Z",
  "battery": {
    "soc_percent": 85.5,
    "voltage": 52.3,
    "current": -8.2,
    "temperature": 32.5,
    "charging": false,
    "estimated_runtime_minutes": 180,
    "health_percent": 95.0
  }
}
```

---

### 4.4 エラー報告

**エンドポイント:** `POST /api/v1/vehicle/{vehicle_id}/error`

**頻度:** イベント駆動（エラー検出時即座）

**優先度:** 高（レート制限をバイパス）

**リクエストボディ:**
```json
{
  "timestamp": "2025-12-16T10:35:12.456Z",
  "error_code": "NAV_OBSTACLE_BLOCKED",
  "severity": "warning",
  "subsystem": "navigation",
  "location": {
    "latitude": 35.681236,
    "longitude": 139.767125
  },
  "description": "障害物によって経路がブロックされました、再ルートを試行中",
  "recovery_action": "rerouting",
  "user_action_required": false
}
```

**重大度レベル:**
| 重大度 | 説明 | スタッフ警告 | 車両の動作 |
|----------|-------------|-------------|------------------|
| `info` | 情報提供、対応不要 | いいえ | 動作を継続 |
| `warning` | 軽微な問題、車両は回復可能 | オプション | 自動回復を試行 |
| `error` | 重大な問題、ミッションに影響の可能性 | はい（メール/ダッシュボード） | 停止または基地に戻る |
| `critical` | 安全上重要、即時対応が必要 | はい（SMS + 音声通話） | 緊急停止、手動介入が必要 |

---

## 5. TVM → 車両コマンド（WebSocket）

### 5.1 WebSocket接続

**接続URL:**
```
wss://tvm.example.com/ws/v1/vehicle/{vehicle_id}?token={access_token}
```

---

### 5.2 ウェルカムメッセージ（サーバー → 車両）

**接続確立直後に送信**

**メッセージ:**
```json
{
  "type": "welcome",
  "vehicle_id": "wheelchair_001",
  "server_time": "2025-12-16T10:30:00.000Z",
  "protocol_version": "1.0",
  "ping_interval_seconds": 30
}
```

---

### 5.3 ハートビート（Ping/Pong）

**車両は30秒ごとにPingを送信する必要があります**

**Ping（車両 → サーバー）:**
```json
{
  "type": "ping",
  "timestamp": "2025-12-16T10:30:30.123Z"
}
```

**Pong（サーバー → 車両）:**
```json
{
  "type": "pong",
  "timestamp": "2025-12-16T10:30:30.234Z"
}
```

---

### 5.4 派遣コマンド

**目的:** ロボットを特定の場所に送る（オプションで乗客ピックアップあり）

**コマンド（サーバー → 車両）:**
```json
{
  "type": "dispatch",
  "command_id": "cmd_12345",
  "timestamp": "2025-12-16T10:40:00.000Z",
  "priority": "normal",
  "mission": {
    "mission_id": "mission_67890",
    "pickup": {
      "waypoint_id": "lobby_entrance",
      "latitude": 35.681100,
      "longitude": 139.767000,
      "action": "dock_wheelchair"
    },
    "destination": {
      "waypoint_id": "room_201",
      "latitude": 35.682456,
      "longitude": 139.768234,
      "action": "undock_wheelchair"
    },
    "scheduled_time": "2025-12-16T11:00:00.000Z"
  }
}
```

**優先度レベル:**
| 優先度 | 説明 | 動作 |
|----------|-------------|----------|
| `low` | 緊急ではないタスク | 通常にキュー |
| `normal` | 標準タスク | デフォルト優先度 |
| `high` | 緊急タスク | 通常より先にキュー |
| `urgent` | 緊急タスク | 安全であれば現在のミッションを中断 |

**車両確認応答（車両 → サーバー）:**
```json
{
  "type": "command_ack",
  "command_id": "cmd_12345",
  "status": "accepted",
  "estimated_arrival": "2025-12-16T10:55:00.000Z",
  "message": "ミッション受理、ナビゲーション開始"
}
```

---

### 5.5 ミッションキャンセルコマンド

**目的:** 現在のミッションをキャンセルしてアイドルに戻る

**コマンド（サーバー → 車両）:**
```json
{
  "type": "cancel",
  "command_id": "cmd_23456",
  "timestamp": "2025-12-16T10:45:00.000Z",
  "mission_id": "mission_67890",
  "reason": "user_requested",
  "safe_stop": true
}
```

---

### 5.6 緊急停止コマンド

**目的:** 即座の停止（安全上重要）

**優先度:** クリティカル（最高優先度、即座の実行が必要）

**コマンド（サーバー → 車両）:**
```json
{
  "type": "emergency_stop",
  "command_id": "cmd_ESTOP_34567",
  "timestamp": "2025-12-16T10:50:00.000Z",
  "reason": "staff_initiated",
  "message": "オペレーターによって緊急停止が要求されました"
}
```

**車両は以下を行う必要があります:**
1. 1秒以内にすべてのモーターを停止
2. ハザードライト/音声警告を作動
3. 即座に確認応答を送信
4. 緊急停止イベントを記録

---

### 5.7 設定更新コマンド

**目的:** 車両設定をリモートで更新

**コマンド（サーバー → 車両）:**
```json
{
  "type": "configure",
  "command_id": "cmd_56789",
  "timestamp": "2025-12-16T11:00:00.000Z",
  "config_updates": {
    "max_speed": 0.8,
    "audio_volume": 75,
    "ehmi_language": "ja"
  }
}
```

**設定可能なパラメータ:**
| パラメータ | 型 | 範囲 | デフォルト | 説明 |
|-----------|------|-------|---------|-------------|
| `max_speed` | float | 0.1-1.5 m/s | 1.0 | 最大ナビゲーション速度 |
| `audio_volume` | int | 0-100 | 80 | eHMI音声ボリューム（%） |
| `ehmi_language` | string | en/ja/zh | ja | eHMI音声言語 |
| `telemetry_frequency` | float | 0.1-2.0 Hz | 1.0 | 位置更新頻度 |
| `operational_hours_start` | string | HH:MM | 09:00 | 日次動作開始時刻 |
| `operational_hours_end` | string | HH:MM | 17:00 | 日次動作終了時刻 |

---

## 6. データモデル

**完全なJSONスキーマについてはTVM_DATA_MODELS.mdを参照**

### 6.1 コアデータ型

**位置:**
```typescript
interface Location {
  latitude: number;      // -90〜90度（WGS84）
  longitude: number;     // -180〜180度（WGS84）
  altitude?: number;     // 海抜メートル（オプション）
  heading: number;       // 0-360度（真北）
  speed: number;         // m/s（0-2.0）
  accuracy: number;      // ±メートル（0-1.0）
}
```

**タイムスタンプ:**
```typescript
type Timestamp = string; // ISO 8601 UTC形式: "2025-12-16T10:30:45.123Z"
```

**車両状態:**
```typescript
type VehicleState =
  | "idle"
  | "navigating"
  | "docking"
  | "charging"
  | "error"
  | "emergency_stop";
```

---

## 7. エラー処理

### 7.1 HTTPエラーレスポンス

**標準エラー形式:**
```json
{
  "error": "error_code",
  "message": "人間が読めるエラー説明",
  "field": "field_name",
  "received_value": "invalid_value",
  "timestamp": "2025-12-16T10:30:45.123Z"
}
```

**一般的なHTTPステータスコード:**
| コード | エラー | 説明 |
|------|-------|-------------|
| 400 | Bad Request | 無効なデータ形式または検証エラー |
| 401 | Unauthorized | 無効または期限切れの認証トークン |
| 403 | Forbidden | このアクションに対して車両が認可されていない |
| 404 | Not Found | エンドポイントまたはリソースが見つからない |
| 429 | Too Many Requests | レート制限超過 |
| 500 | Internal Server Error | サーバー側エラー |
| 503 | Service Unavailable | サーバーが一時的に利用不可 |

---

### 7.2 再試行ロジック

**車両クライアントは指数バックオフを実装する必要があります:**

```python
# 疑似コード
def retry_with_backoff(request_func, max_retries=5):
    base_delay = 1  # 秒
    max_delay = 60  # 秒

    for attempt in range(max_retries):
        try:
            return request_func()
        except (ConnectionError, Timeout) as e:
            if attempt == max_retries - 1:
                raise  # 最大再試行後に諦める

            delay = min(base_delay * (2 ** attempt), max_delay)
            wait_with_jitter(delay)
```

---

## 8. オフライン動作

### 8.1 TVMサーバーに到達できない場合の車両の動作

**車両は以下を行う必要があります:**
1. ✅ 自律動作を継続（ローカルナビゲーションを使用）
2. ✅ テレメトリーメッセージをローカルにキュー（最大1000メッセージまたは10 MB）
3. ✅ 30秒ごとに再接続を試行
4. ✅ すべてのオフラインイベントをログ
5. ❌ 新しいミッションを受け入れない（現在のミッションのみ完了）
6. ✅ バッテリーが30%未満でステータスをアップロードできない場合は基地に戻る

---

### 8.2 再接続後の一括アップロード

**エンドポイント:** `POST /api/v1/vehicle/{vehicle_id}/bulk`

**リクエスト（一括アップロード）:**
```json
{
  "vehicle_id": "wheelchair_001",
  "offline_period": {
    "start": "2025-12-16T10:30:00.000Z",
    "end": "2025-12-16T10:35:00.000Z"
  },
  "messages": [
    {
      "type": "location",
      "timestamp": "2025-12-16T10:30:00.123Z",
      "data": { /* 位置データ */ }
    },
    {
      "type": "battery",
      "timestamp": "2025-12-16T10:30:05.123Z",
      "data": { /* バッテリーデータ */ }
    }
    // ... 最大1000メッセージ
  ]
}
```

---

## 9. 並行開発用モックインターフェース

### 9.1 モックTVMサーバー（Pankaj用 - 車両開発）

**目的:** 実際のTVMサーバーを待たずに車両ソフトウェア開発を可能にする

**実装:** Python FastAPIモックサーバー

**機能:**
- ✅ すべてのREST APIエンドポイントを受け入れ
- ✅ 成功レスポンスを返す
- ✅ 受信したすべてのメッセージをログ
- ✅ WebSocket経由でランダムなテストコマンドを送信
- ✅ ネットワーク遅延をシミュレート（設定可能）

---

### 9.2 モック車両クライアント（Unno用 - TVMサーバー開発）

**目的:** 実際の車両を待たずにTVMサーバー開発を可能にする

**実装:** 車両の動作をシミュレートするPythonスクリプト

**機能:**
- ✅ 位置更新を送信（1 Hz）
- ✅ ステータス更新を送信（状態変更時）
- ✅ バッテリー更新を送信（5秒ごと）
- ✅ WebSocket経由でコマンドに応答
- ✅ 現実的な車両動作をシミュレート（ナビゲーション、ドッキング、エラー）

---

## 10. テストと検証

### 10.1 API契約テスト

**両チームは以下を行う必要があります:**
1. ✅ 契約としてAPI仕様を使用
2. ✅ 仕様に対する統合テストを書く
3. ✅ 開発中にモックインターフェースを使用
4. ✅ 本番前にエンドツーエンドテストを実行

**テストカテゴリ:**
| テストタイプ | 責任 | ツール |
|-----------|----------------|-------|
| 単体テスト（APIエンドポイント） | Unno（TVMサーバー） | pytest、FastAPI TestClient |
| 単体テスト（TVMクライアントlib） | Pankaj（車両） | pytest、requests-mock |
| 統合テスト | 両方（共同） | Postman、pytest + 実エンドポイント |
| 負荷テスト | Unno（TVMサーバー） | Locust、Apache Bench |
| WebSocketテスト | 両方（共同） | ws（Node.js）、websockets（Python） |

---

### 10.2 受け入れ基準

**API仕様が受け入れられる条件:**
- ✅ すべてのエンドポイントが例付きで文書化されている
- ✅ すべてのデータモデルがJSONスキーマで定義されている
- ✅ エラー処理が指定されている
- ✅ 両チーム用のモックインターフェースが利用可能
- ✅ 統合テストに合格（成功率>95%）
- ✅ 負荷テスト: 10台の車両 @ 1 Hzテレメトリー（10分間持続）
- ✅ WebSocketコマンド: <500msレイテンシー（95パーセンタイル）
- ✅ オフライン動作: キューと一括アップロードがテスト済み

---

## 11. デプロイメントとバージョニング

### 11.1 APIバージョニング

**URLバージョニング:** `/api/v1/...` → `/api/v2/...`

**バージョンサポート:**
- 現在のバージョン（v1）: 完全サポート
- 前のバージョン（v0）: v1リリース後6ヶ月間サポート
- 非推奨バージョン: 削除前に3ヶ月の通知

**破壊的変更（新バージョンが必要）:**
- エンドポイントの削除
- 必須フィールドの変更
- データ型の変更
- フィールド名の変更

**非破壊的変更（同じバージョン）:**
- オプションフィールドの追加
- 新しいエンドポイントの追加
- フィールドの非推奨化（警告期間あり）

---

### 11.2 デプロイメントチェックリスト

**本番デプロイメント前:**
- ✅ API仕様が両チームによってレビューされ承認されている
- ✅ 統合テストに合格（>95%）
- ✅ 負荷テスト完了（10台の車両、1 Hz、10分間）
- ✅ セキュリティ監査完了（JWT実装、HTTPS）
- ✅ 監視とロギングが設定されている（CloudWatch、Prometheus）
- ✅ エラーアラートが設定されている（PagerDuty、メール）
- ✅ ドキュメントが公開されている（APIリファレンス、開発者ガイド）
- ✅ ロールバック計画が準備されている

---

## ドキュメントステータス

**ステータス:** ドラフト - レビュー待ち
**次回レビュー:** 2025年12月17日（キックオフミーティング - Pankaj + Unno）
**承認必須:** 開発開始前に両チームが承認する必要があります

---

## 付録

### 付録A: クイックリファレンス - REST APIエンドポイント

| メソッド | エンドポイント | 頻度 | 目的 |
|--------|----------|-----------|---------|
| POST | `/api/v1/auth/token` | 起動時/トークン期限切れ時 | JWTトークン取得 |
| POST | `/api/v1/vehicle/{id}/location` | 1 Hz | 位置テレメトリー |
| POST | `/api/v1/vehicle/{id}/status` | イベント駆動/10秒 | ステータス更新 |
| POST | `/api/v1/vehicle/{id}/battery` | 5秒ごと | バッテリーテレメトリー |
| POST | `/api/v1/vehicle/{id}/error` | イベント駆動 | エラー報告 |
| POST | `/api/v1/vehicle/{id}/docking` | イベント駆動 | ドッキングステータス |
| POST | `/api/v1/vehicle/{id}/bulk` | オフライン後 | 一括メッセージアップロード |

---

### 付録B: クイックリファレンス - WebSocketコマンド

| コマンドタイプ | 方向 | 優先度 | 目的 |
|--------------|-----------|----------|---------|
| `welcome` | サーバー → 車両 | N/A | 接続確認 |
| `ping` / `pong` | 双方向 | N/A | ハートビート |
| `dispatch` | サーバー → 車両 | Normal/High/Urgent | 目的地へ送る |
| `cancel` | サーバー → 車両 | High | ミッションキャンセル |
| `emergency_stop` | サーバー → 車両 | CRITICAL | 即座の停止 |
| `resume` | サーバー → 車両 | High | 緊急停止から再開 |
| `configure` | サーバー → 車両 | Normal | 設定更新 |
| `command_ack` | 車両 → サーバー | N/A | コマンド確認応答 |

---

**ドキュメント終了**

**重要な次のステップ:**
1. ⚠️ **第1週1-2日目:** PankajとUnnoがこの仕様をレビュー
2. ⚠️ **第1週3-4日目:** API設計について議論し確定（会議が必要）
3. ⚠️ **第1週5日目:** 両チームによる承認
4. ⚠️ **第1週6-10日目:** モックインターフェースを実装
5. ⚠️ **第2週以降:** 並行開発を開始（車両クライアント + TVMサーバー）

**質問/clarifications:** Pankaj（車両）またはUnno（TVMサーバー）に連絡
