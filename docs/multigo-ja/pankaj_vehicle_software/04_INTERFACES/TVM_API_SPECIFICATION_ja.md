# TVM API仕様 - 車両 ↔ フリート管理サーバー

**ドキュメントID:** INT-TVM-API-001
**バージョン:** 1.0
**日付:** 2025-12-16
**ステータス:** ドラフト - クリティカル（第1週優先度1）
**分類:** 内部

**責任チーム:**
- **車両クライアント:** Pankaj（ソフトウェア - 車両側）
- **TVMサーバー:** Unno（ソフトウェア - フリート管理側）

**目的:** 自律走行車両とTVM（Total Vehicle Management）サーバー間の正確なAPI契約を定義し、モックインターフェースによる並行開発を可能にする。

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
7. [エラーハンドリング](#7-エラーハンドリング)
8. [オフライン動作](#8-オフライン動作)
9. [並行開発用モックインターフェース](#9-並行開発用モックインターフェース)
10. [テストと検証](#10-テストと検証)
11. [デプロイメントとバージョニング](#11-デプロイメントとバージョニング)

---

## 1. はじめに

### 1.1 目的

本ドキュメントは、以下の間の**完全なAPI契約**を定義します:
- **車両クライアント**（自律車椅子輸送ロボット - ROS 2ベース）
- **TVMサーバー**（Total Vehicle Managementフリート管理システム）

**クリティカルな依存関係:**
- 車両ソフトウェアは、この仕様なしではTVMサーバーと通信できない
- TVMサーバーは、この仕様なしでは車両データを受信できない
- 両チームは仕様通りに正確に実装しなければならない

**署名が必要:** Pankaj（車両）とUnno（TVMサーバー）の両者が開発開始前に本ドキュメントを承認する必要があります。

---

### 1.2 スコープ

**スコープ内:**
- ✅ 車両テレメトリアップロード用REST API（位置、ステータス、バッテリー、エラー）
- ✅ リアルタイムTVM→車両コマンド用WebSocket API（派遣、キャンセル、緊急停止）
- ✅ 認証メカニズム（JWTトークン）
- ✅ データモデル（すべてのメッセージのJSONスキーマ）
- ✅ エラーコードとハンドリング
- ✅ オフライン動作動作
- ✅ 並行開発用モックインターフェース

**スコープ外:**
- ❌ 内部TVMサーバー実装詳細
- ❌ 内部車両ソフトウェアアーキテクチャ
- ❌ データベーススキーマ（TVMサーバー内部）
- ❌ ROS 2トピック（車両内部）
- ❌ ログアップロードシステム（別個のtvm_uploadデーモン）

---

### 1.3 関連ドキュメント

- **TVM_DATA_MODELS.md** - すべてのメッセージの詳細JSONスキーマ
- **HARDWARE_SOFTWARE_INTERFACES.md** - ROS 2インターフェース仕様
- **VEHICLE_TVM_CLIENT_REQUIREMENTS.md** - 車両側クライアント要件
- **TVM_SERVER_REQUIREMENTS.md** - サーバー側要件
- **TVM_INTEGRATION_REQUIREMENTS.md** - 統合要件と受入基準

---

## 2. API概要

### 2.1 技術スタック

**通信プロトコル:**
- **REST API（HTTPS）:** 車両 → TVMテレメトリアップロード（単方向プッシュ）
- **WebSocket（WSS）:** TVM → 車両コマンド（双方向、リアルタイム）

**データフォーマット:**
- **JSON:** すべてのメッセージペイロード
- **UTF-8エンコーディング:** すべてのテキスト
- **ISO 8601（UTC）:** すべてのタイムスタンプ

**認証:**
- **JWT（JSON Web Tokens）:** Bearerトークン認証
- **トークン有効期限:** 24時間（構成可能）
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

| エンドポイントカテゴリ | レート制限 | バースト | 備考 |
|-------------------|------------|-------|-------|
| 位置更新 | 車両あたり1 req/秒 | 5 | 通常テレメトリ |
| ステータス更新 | 車両あたり1 req/秒 | 5 | 状態変更 |
| バッテリー更新 | 車両あたり1 req/5秒 | 2 | 低頻度 |
| エラー報告 | 車両あたり10 req/秒 | 20 | 高優先度、レート制限バイパス |
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
  "message": "Too many requests. Please wait 15 seconds.",
  "retry_after": 15
}
```

---

### 2.4 通信アーキテクチャ

```
┌─────────────────────────────────────────────────────────────┐
│                   自律走行車両                              │
│                                                             │
│  ┌────────────────────────────────────────────────────┐    │
│  │           車両ソフトウェア（ROS 2）                │    │
│  │                                                    │    │
│  │  ┌──────────┐  ┌──────────┐  ┌────────────┐      │    │
│  │  │ナビゲー  │  │ドッキング│  │   安全     │      │    │
│  │  │ション    │  │サブシス  │  │   監視     │      │    │
│  │  │サブシス  │  │テム      │  │            │      │    │
│  │  │テム      │  │          │  │            │      │    │
│  │  └─────┬────┘  └─────┬────┘  └──────┬─────┘      │    │
│  │        │             │              │             │    │
│  │        └─────────────┼──────────────┘             │    │
│  │                      │                            │    │
│  │              ┌───────▼────────┐                   │    │
│  │              │  TVMクライア   │                   │    │
│  │              │   ントライブラ │                   │    │
│  │              │   リ           │                   │    │
│  │              │ ┌────────────┐ │                   │    │
│  │              │ │RESTクライア│ │ ← 位置、        │    │
│  │              │ │ント        │ │   ステータス、  │    │
│  │              │ └─────┬──────┘ │   バッテリー、  │    │
│  │              │       │        │   エラー        │    │
│  │              │       │ HTTPS  │                 │    │
│  │              │ ┌─────▼──────┐ │                 │    │
│  │              │ │WebSocket   │ │ ← 派遣、        │    │
│  │              │ │クライアント│ │   キャンセル、  │    │
│  │              │ └─────┬──────┘ │   E-Stop        │    │
│  │              └───────┼────────┘                   │    │
│  └──────────────────────┼────────────────────────────┘    │
└─────────────────────────┼──────────────────────────────────┘
                          │
                    ══════╪══════ インターネット ══════
                          │
┌─────────────────────────┼──────────────────────────────────┐
│                         │                                  │
│              ┌──────────▼───────────┐                      │
│              │   ロードバランサー   │                      │
│              │  (NGINX/HAProxy)     │                      │
│              └──────────┬───────────┘                      │
│                         │                                  │
│         ┌───────────────┴────────────────┐                │
│         │                                 │                │
│   ┌─────▼──────┐                  ┌──────▼─────┐         │
│   │ REST API   │                  │ WebSocket  │         │
│   │  サーバー  │                  │   サーバー │         │
│   └─────┬──────┘                  └──────┬─────┘         │
│         │                                 │                │
│         └─────────────┬─────────────────┘                │
│                       │                                    │
│              ┌────────▼─────────┐                         │
│              │  TVMサーバー     │                         │
│              │  (フリート管理)  │                         │
│              │                  │                         │
│              │  ┌────────────┐  │                         │
│              │  │ダッシュ    │  │  ← PC/タブレットアプリ │
│              │  │ボード      │  │                         │
│              │  │サービス    │  │                         │
│              │  └────────────┘  │                         │
│              │  ┌────────────┐  │                         │
│              │  │予約        │  │  ← 予約システム        │
│              │  │サービス    │  │                         │
│              │  └────────────┘  │                         │
│              │  ┌────────────┐  │                         │
│              │  │  データベー│  │  ← PostgreSQL          │
│              │  │  ス        │  │                         │
│              │  │(Postgres)  │  │                         │
│              │  └────────────┘  │                         │
│              └──────────────────┘                         │
│                                                            │
│              TVMサーバー（Unnoチーム）                    │
└────────────────────────────────────────────────────────────┘
```

---

## 3. 認証と認可

### 3.1 車両登録

**初期セットアップ（1回のみ）:**
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

**保存:** 認証情報は車両上に安全に保存される（暗号化された構成ファイル）

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
  "error_description": "Invalid client credentials"
}
```

**トークン有効期限:** 24時間（86400秒）

---

### 3.3 アクセストークンの使用

**すべてのREST APIリクエストに含める必要があります:**
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

**車両クライアントは以下を実施する必要があります:**
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

**WebSocket接続にはURLにトークンを含める必要があります:**
```
wss://tvm.example.com/ws/v1/vehicle/wheelchair_001?token=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**接続拒否（無効なトークン）:**
```json
{
  "type": "error",
  "code": "auth_failed",
  "message": "Invalid or expired token"
}
```

**接続クローズコード:** 4001（認証失敗）

---

## 4. 車両 → TVMエンドポイント（REST）

### 4.1 位置更新

**エンドポイント:** `POST /api/v1/vehicle/{vehicle_id}/location`

**頻度:** ナビゲーション中は1 Hz（1秒に1回）、アイドル時は0.2 Hz（5秒に1回）

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
| フィールド | タイプ | 単位 | 範囲 | 必須 | 説明 |
|-------|------|------|-------|----------|-------------|
| timestamp | string（ISO 8601 UTC） | - | - | ✅ | 測定時刻 |
| latitude | float | 度 | -90～90 | ✅ | WGS84緯度 |
| longitude | float | 度 | -180～180 | ✅ | WGS84経度 |
| altitude | float | メートル | - | ⚠️ オプション | 海抜高度（利用可能な場合） |
| heading | float | 度 | 0-360 | ✅ | 真北方位（0=北、90=東） |
| speed | float | m/s | 0-2.0 | ✅ | 現在速度 |
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
  "message": "Latitude out of range",
  "field": "location.latitude",
  "received_value": 95.123
}
```

**HTTPステータスコード:**
- `200 OK` - 位置更新成功
- `400 Bad Request` - 無効なデータフォーマットまたは範囲外の値
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
| `idle` | 車両停止、ミッションなし | charging、error | navigating、charging |
| `navigating` | 目的地へ積極的に移動中 | idle、docking | docking、idle、error |
| `docking` | 車椅子とドッキング中 | navigating | idle、error |
| `charging` | ステーションで充電中 | idle | idle |
| `error` | システムエラー、注意が必要 | any | idle（回復後） |
| `emergency_stop` | 緊急停止作動 | any | idle（リセット後） |

**フィールド説明:**
| フィールド | タイプ | 必須 | 説明 |
|-------|------|----------|-------------|
| timestamp | string（ISO 8601 UTC） | ✅ | ステータス更新時刻 |
| state | string（enum） | ✅ | 現在の車両状態 |
| mission_id | string | ⚠️ オプション | 現在のミッションID（ナビゲーション中の場合） |
| current_waypoint | string | ⚠️ オプション | 現在の目標ウェイポイント（ナビゲーション中の場合） |
| passengers_onboard | boolean | ✅ | 車椅子/乗客が接続されている場合はtrue |
| operational_hours | boolean | ✅ | 運用時間ウィンドウ内の場合はtrue |
| details | object | ⚠️ オプション | 状態固有の詳細 |

**レスポンス（成功）:**
```json
{
  "status": "ok",
  "received_at": "2025-12-16T10:30:45.234Z"
}
```

---

### 4.3 バッテリー更新

**エンドポイント:** `POST /api/v1/vehicle/{vehicle_id}/battery`

**頻度:** 5秒ごと（負荷軽減のため低頻度）

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

**フィールド説明:**
| フィールド | タイプ | 単位 | 必須 | 説明 |
|-------|------|------|----------|-------------|
| timestamp | string（ISO 8601 UTC） | - | ✅ | 測定時刻 |
| soc_percent | float | % | ✅ | 充電状態（0-100%） |
| voltage | float | V | ✅ | バッテリー電圧 |
| current | float | A | ✅ | 電流（負=放電、正=充電） |
| temperature | float | °C | ✅ | バッテリー温度 |
| charging | boolean | - | ✅ | 充電中の場合はtrue |
| estimated_runtime_minutes | integer | 分 | ⚠️ オプション | 現在の負荷での残り稼働時間 |
| health_percent | float | % | ⚠️ オプション | バッテリー健全性（0-100%） |

**レスポンス（成功）:**
```json
{
  "status": "ok",
  "received_at": "2025-12-16T10:30:45.234Z",
  "warning": null
}
```

**レスポンス（低バッテリー警告）:**
```json
{
  "status": "ok",
  "received_at": "2025-12-16T10:30:45.234Z",
  "warning": "low_battery",
  "message": "Battery below 30%. Consider returning to charging station.",
  "recommended_action": "return_to_base"
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
  "description": "Path blocked by obstacle, attempting reroute",
  "recovery_action": "rerouting",
  "user_action_required": false
}
```

**重大度レベル:**
| 重大度 | 説明 | スタッフ警告 | 車両動作 |
|----------|-------------|-------------|------------------|
| `info` | 情報、アクション不要 | なし | 動作継続 |
| `warning` | 軽微な問題、車両は回復可能 | オプション | 自動回復を試行 |
| `error` | 重大な問題、ミッションに影響の可能性 | はい（メール/ダッシュボード） | 停止または基地に戻る |
| `critical` | 安全クリティカル、即座の注意が必要 | はい（SMS + 音声通話） | 緊急停止、手動介入が必要 |

**一般的なエラーコード:**
| コード | 重大度 | 説明 |
|------|----------|-------------|
| `NAV_OBSTACLE_BLOCKED` | warning | 経路がブロック、再ルーティング中 |
| `NAV_LOCALIZATION_LOST` | error | ローカライゼーション信頼度が低すぎる |
| `DOCK_ALIGNMENT_FAILED` | warning | ドッキングアライメント失敗、再試行中 |
| `DOCK_MARKER_NOT_FOUND` | error | ArUcoマーカー未検出 |
| `SAFE_ESTOP_ACTIVATED` | critical | 緊急停止トリガー |
| `SAFE_COLLISION_DETECTED` | critical | バンパーによる衝突検出 |
| `POWER_BATTERY_CRITICAL` | critical | バッテリー10%未満 |
| `SENSOR_LIDAR_FAILURE` | error | LiDARセンサー障害 |

**レスポンス（成功）:**
```json
{
  "status": "acknowledged",
  "received_at": "2025-12-16T10:35:12.567Z",
  "alert_id": "alert_67890",
  "action": "alert_staff",
  "staff_notified": true
}
```

---

### 4.5 ドッキングステータス更新

**エンドポイント:** `POST /api/v1/vehicle/{vehicle_id}/docking`

**頻度:** ドッキングシーケンス中のイベント駆動

**リクエストボディ:**
```json
{
  "timestamp": "2025-12-16T10:40:15.789Z",
  "phase": "visual_servoing",
  "marker_id": 42,
  "distance_to_marker_meters": 0.85,
  "alignment_error_degrees": 2.3,
  "docking_success": false,
  "attempts": 1,
  "details": {
    "marker_visible": true,
    "confidence": 0.95
  }
}
```

**ドッキングフェーズ:**
| フェーズ | 説明 | 予想期間 |
|-------|-------------|-------------------|
| `approaching` | ドッキングゾーンへナビゲート | 10-30秒 |
| `marker_detection` | ArUcoマーカーを検索 | 5-10秒 |
| `visual_servoing` | ビジュアルフィードバックを使用した精密アライメント | 15-30秒 |
| `final_approach` | ドッキングへのゆっくりとした最終接近 | 10-15秒 |
| `attachment` | 物理的接続メカニズム | 5-10秒 |
| `success` | ドッキング完了、車椅子固定 | - |
| `failed` | ドッキング失敗、アイドルに戻る | - |

**レスポンス（成功）:**
```json
{
  "status": "ok",
  "received_at": "2025-12-16T10:40:15.890Z"
}
```

---

## 5. TVM → 車両コマンド（WebSocket）

### 5.1 WebSocket接続

**接続URL:**
```
wss://tvm.example.com/ws/v1/vehicle/{vehicle_id}?token={access_token}
```

**接続ライフサイクル:**
```
車両                            TVMサーバー
  │                                   │
  ├─── WebSocket接続 ────────────────>│
  │    （URLにJWTトークン）           │
  │                                   │
  │<─── 接続承認 ──────────────────────┤
  │    （200 WebSocketアップグレード） │
  │                                   │
  │<─── ウェルカムメッセージ ──────────┤
  │    { "type": "welcome", ... }     │
  │                                   │
  ├─── Ping ─────────────────────────>│ (30秒ごと)
  │<─── Pong ──────────────────────────┤
  │                                   │
  │<─── 派遣コマンド ─────────────────┤
  ├─── コマンド確認 ─────────────────>│
  │                                   │
  │<─── 緊急停止 ─────────────────────┤
  ├─── 実行確認 ─────────────────────>│
  │                                   │
  ├─── 切断 ─────────────────────────>│ (正常)
  │    (クローズコード1000)            │
  │<─── クローズ確認 ──────────────────┤
  │                                   │
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

**接続タイムアウト:**
- 10秒以内にPongを受信しない場合 → 再接続
- サーバーが60秒以内にPingを受信しない場合 → 車両を切断

---

### 5.4 派遣コマンド

**目的:** ロボットを特定の場所に送る（乗客ピックアップオプション付き）

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
| `low` | 緊急ではないタスク | 通常通りキュー |
| `normal` | 標準タスク | デフォルト優先度 |
| `high` | 緊急タスク | 通常より優先してキュー |
| `urgent` | 緊急タスク | 安全な場合は現在のミッションを中断 |

**車両確認（車両 → サーバー）:**
```json
{
  "type": "command_ack",
  "command_id": "cmd_12345",
  "status": "accepted",
  "estimated_arrival": "2025-12-16T10:55:00.000Z",
  "message": "Mission accepted, starting navigation"
}
```

**拒否（車両 → サーバー）:**
```json
{
  "type": "command_ack",
  "command_id": "cmd_12345",
  "status": "rejected",
  "reason": "battery_too_low",
  "message": "Battery at 15%, insufficient for mission"
}
```

**拒否理由:**
| 理由 | 説明 |
|--------|-------------|
| `battery_too_low` | バッテリー20%未満 |
| `already_on_mission` | 車両が既にミッション実行中 |
| `in_error_state` | 車両がエラー状態 |
| `outside_operational_hours` | 構成された運用ウィンドウ外 |
| `invalid_destination` | 目的地ウェイポイントがマップに見つからない |

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

**safe_stop:**
- `true` → 現在の場所で安全に停止、乗客が乗車している場合はドッキング解除
- `false` → 即座に停止（緊急キャンセル）

**車両確認:**
```json
{
  "type": "command_ack",
  "command_id": "cmd_23456",
  "status": "executed",
  "stopped_at": {
    "latitude": 35.681500,
    "longitude": 139.767500
  },
  "message": "Mission canceled, returned to idle"
}
```

---

### 5.6 緊急停止コマンド

**目的:** 即座の停止（安全クリティカル）

**優先度:** クリティカル（最高優先度、即座の実行が必要）

**コマンド（サーバー → 車両）:**
```json
{
  "type": "emergency_stop",
  "command_id": "cmd_ESTOP_34567",
  "timestamp": "2025-12-16T10:50:00.000Z",
  "reason": "staff_initiated",
  "message": "Emergency stop requested by operator"
}
```

**車両は以下を実行する必要があります:**
1. 1秒以内にすべてのモーターを停止
2. 危険灯/音声警告を作動
3. 即座に確認を送信
4. 緊急停止イベントをログ

**車両確認:**
```json
{
  "type": "command_ack",
  "command_id": "cmd_ESTOP_34567",
  "status": "executed",
  "stopped_at": "2025-12-16T10:50:00.567Z",
  "location": {
    "latitude": 35.681600,
    "longitude": 139.767600
  }
}
```

**緊急停止からの再開:**
```json
{
  "type": "resume",
  "command_id": "cmd_45678",
  "timestamp": "2025-12-16T10:55:00.000Z"
}
```

---

### 5.7 構成更新コマンド

**目的:** 車両構成をリモート更新

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

**構成可能なパラメータ:**
| パラメータ | タイプ | 範囲 | デフォルト | 説明 |
|-----------|------|-------|---------|-------------|
| `max_speed` | float | 0.1-1.5 m/s | 1.0 | 最大ナビゲーション速度 |
| `audio_volume` | int | 0-100 | 80 | eHMI音声ボリューム（%） |
| `ehmi_language` | string | en/ja/zh | ja | eHMI音声言語 |
| `telemetry_frequency` | float | 0.1-2.0 Hz | 1.0 | 位置更新頻度 |
| `operational_hours_start` | string | HH:MM | 09:00 | 日次運用開始時刻 |
| `operational_hours_end` | string | HH:MM | 17:00 | 日次運用終了時刻 |

**車両確認:**
```json
{
  "type": "command_ack",
  "command_id": "cmd_56789",
  "status": "applied",
  "updated_config": {
    "max_speed": 0.8,
    "audio_volume": 75,
    "ehmi_language": "ja"
  }
}
```

---

## 6. データモデル

**完全なJSONスキーマについてはTVM_DATA_MODELS.mdを参照**

### 6.1 コアデータタイプ

**位置:**
```typescript
interface Location {
  latitude: number;      // -90～90度（WGS84）
  longitude: number;     // -180～180度（WGS84）
  altitude?: number;     // 海抜メートル（オプション）
  heading: number;       // 0-360度（真北）
  speed: number;         // m/s（0-2.0）
  accuracy: number;      // ±メートル（0-1.0）
}
```

**タイムスタンプ:**
```typescript
type Timestamp = string; // ISO 8601 UTCフォーマット: "2025-12-16T10:30:45.123Z"
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

**エラー重大度:**
```typescript
type ErrorSeverity = "info" | "warning" | "error" | "critical";
```

**コマンド優先度:**
```typescript
type CommandPriority = "low" | "normal" | "high" | "urgent";
```

---

## 7. エラーハンドリング

### 7.1 HTTPエラーレスポンス

**標準エラーフォーマット:**
```json
{
  "error": "error_code",
  "message": "Human-readable error description",
  "field": "field_name",
  "received_value": "invalid_value",
  "timestamp": "2025-12-16T10:30:45.123Z"
}
```

**一般的なHTTPステータスコード:**
| コード | エラー | 説明 |
|------|-------|-------------|
| 400 | Bad Request | 無効なデータフォーマットまたは検証エラー |
| 401 | Unauthorized | 無効または期限切れの認証トークン |
| 403 | Forbidden | このアクションに対して車両が認可されていない |
| 404 | Not Found | エンドポイントまたはリソースが見つからない |
| 429 | Too Many Requests | レート制限超過 |
| 500 | Internal Server Error | サーバー側エラー |
| 503 | Service Unavailable | サーバーが一時的に利用不可 |

---

### 7.2 WebSocketエラーメッセージ

**フォーマット:**
```json
{
  "type": "error",
  "code": "error_code",
  "message": "Human-readable error description",
  "timestamp": "2025-12-16T10:30:45.123Z"
}
```

**WebSocketクローズコード:**
| コード | 理由 | 説明 |
|------|--------|-------------|
| 1000 | Normal Closure | 正常な切断 |
| 1001 | Going Away | サーバーシャットダウン中 |
| 4001 | Authentication Failed | 無効または期限切れのトークン |
| 4002 | Invalid Message | 不正なメッセージを受信 |
| 4003 | Rate Limit Exceeded | コマンドが多すぎる |

---

### 7.3 リトライロジック

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
                raise  # 最大リトライ回数後に諦める

            delay = min(base_delay * (2 ** attempt), max_delay)
            wait_with_jitter(delay)
```

**リトライポリシー:**
| エラータイプ | リトライ | 最大リトライ | バックオフ |
|------------|-------|-------------|---------|
| 接続タイムアウト | はい | 5 | 指数（1秒、2秒、4秒、8秒、16秒） |
| 500 Internal Server Error | はい | 3 | 指数 |
| 503 Service Unavailable | はい | 3 | 指数 |
| 429 Too Many Requests | はい | 無制限 | `retry_after`秒待機 |
| 400 Bad Request | いいえ | 0 | N/A（データを修正して手動でリトライ） |
| 401 Unauthorized | はい（再認証） | 1 | 即座（新しいトークンを取得） |

---

## 8. オフライン動作

### 8.1 TVMサーバーに到達できない場合の車両動作

**車両は以下を実施する必要があります:**
1. ✅ 自律動作を継続（ローカルナビゲーションを使用）
2. ✅ テレメトリメッセージをローカルにキュー（最大1000メッセージまたは10 MB）
3. ✅ 30秒ごとに再接続を試行
4. ✅ すべてのオフラインイベントをログ
5. ❌ 新しいミッションを受け入れない（現在のミッションのみ完了）
6. ✅ バッテリー<30%でステータスをアップロードできない場合は基地に戻る

**メッセージキューイング:**
```
┌────────────────────────────────────┐
│ 車両（オフラインモード）           │
│                                    │
│ ┌────────────────────────────────┐ │
│ │  ローカルメッセージキュー      │ │
│ │                                │ │
│ │  1. 位置（t=10:30:00）         │ │
│ │  2. 位置（t=10:30:01）         │ │
│ │  3. バッテリー（t=10:30:05）   │ │
│ │  4. エラー（t=10:30:10）       │ │
│ │  ... (最大1000メッセージ)      │ │
│ └────────────────────────────────┘ │
│                                    │
│ 再接続タイマー: 30秒               │
└────────────────────────────────────┘
         │
         │ 再接続成功
         ▼
┌────────────────────────────────────┐
│ TVMサーバー                        │
│                                    │
│ 一括アップロードエンドポイント:    │
│ POST /api/v1/vehicle/{id}/bulk     │
│                                    │
│ 最大1000メッセージを受け入れ       │
└────────────────────────────────────┘
```

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

**レスポンス:**
```json
{
  "status": "ok",
  "messages_received": 247,
  "messages_processed": 247,
  "offline_duration_seconds": 300
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
- ✅ ネットワーク遅延をシミュレート（構成可能）

**モックサーバー例（Python）:**
```python
# mock_tvm_server.py
from fastapi import FastAPI, WebSocket
import uvicorn

app = FastAPI()

@app.post("/api/v1/vehicle/{vehicle_id}/location")
async def location_update(vehicle_id: str, data: dict):
    print(f"[MOCK] Received location from {vehicle_id}: {data}")
    return {"status": "ok", "received_at": "2025-12-16T10:30:45.234Z"}

@app.post("/api/v1/vehicle/{vehicle_id}/status")
async def status_update(vehicle_id: str, data: dict):
    print(f"[MOCK] Received status from {vehicle_id}: {data}")
    return {"status": "ok", "received_at": "2025-12-16T10:30:45.234Z"}

@app.websocket("/ws/v1/vehicle/{vehicle_id}")
async def websocket_endpoint(websocket: WebSocket, vehicle_id: str):
    await websocket.accept()
    await websocket.send_json({
        "type": "welcome",
        "vehicle_id": vehicle_id,
        "server_time": "2025-12-16T10:30:00.000Z"
    })

    # 10秒後にテスト派遣コマンドを送信
    await asyncio.sleep(10)
    await websocket.send_json({
        "type": "dispatch",
        "command_id": "cmd_test_001",
        "mission": {
            "destination": {
                "waypoint_id": "test_waypoint",
                "latitude": 35.681236,
                "longitude": 139.767125
            }
        }
    })

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

**モックサーバーの実行:**
```bash
pip install fastapi uvicorn
python mock_tvm_server.py
```

**車両の接続先:** `http://localhost:8000`

---

### 9.2 モック車両クライアント（Unno用 - TVMサーバー開発）

**目的:** 実際の車両を待たずにTVMサーバー開発を可能にする

**実装:** 車両動作をシミュレートするPythonスクリプト

**機能:**
- ✅ 位置更新を送信（1 Hz）
- ✅ ステータス更新を送信（状態変更時）
- ✅ バッテリー更新を送信（5秒ごと）
- ✅ WebSocket経由でコマンドに応答
- ✅ リアルな車両動作をシミュレート（ナビゲーション、ドッキング、エラー）

**モック車両例（Python）:**
```python
# mock_vehicle.py
import requests
import asyncio
import websockets
import json
from datetime import datetime

VEHICLE_ID = "wheelchair_001"
TVM_BASE_URL = "https://tvm.example.com/api/v1"
TVM_WS_URL = f"wss://tvm.example.com/ws/v1/vehicle/{VEHICLE_ID}"

async def send_location_updates():
    """位置更新を1秒ごとに送信"""
    while True:
        location_data = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "location": {
                "latitude": 35.681236,
                "longitude": 139.767125,
                "heading": 90.0,
                "speed": 0.5,
                "accuracy": 0.10
            }
        }
        response = requests.post(
            f"{TVM_BASE_URL}/vehicle/{VEHICLE_ID}/location",
            json=location_data,
            headers={"Authorization": f"Bearer {TOKEN}"}
        )
        print(f"Location update: {response.status_code}")
        await asyncio.sleep(1)

async def websocket_command_listener():
    """TVMサーバーからのコマンドをリッスン"""
    async with websockets.connect(TVM_WS_URL) as websocket:
        async for message in websocket:
            cmd = json.loads(message)
            print(f"Received command: {cmd['type']}")

            if cmd['type'] == 'dispatch':
                # 確認を送信
                ack = {
                    "type": "command_ack",
                    "command_id": cmd['command_id'],
                    "status": "accepted"
                }
                await websocket.send(json.dumps(ack))

# 両方のタスクを同時実行
asyncio.run(asyncio.gather(
    send_location_updates(),
    websocket_command_listener()
))
```

---

## 10. テストと検証

### 10.1 API契約テスト

**両チームは以下を実施する必要があります:**
1. ✅ API仕様を契約として使用
2. ✅ 仕様に対する統合テストを作成
3. ✅ 開発中にモックインターフェースを使用
4. ✅ 本番環境前にエンドツーエンドテストを実行

**テストカテゴリ:**
| テストタイプ | 責任 | ツール |
|-----------|----------------|-------|
| ユニットテスト（APIエンドポイント） | Unno（TVMサーバー） | pytest、FastAPI TestClient |
| ユニットテスト（TVMクライアントライブラリ） | Pankaj（車両） | pytest、requests-mock |
| 統合テスト | 両者（共同） | Postman、pytest + 実際のエンドポイント |
| 負荷テスト | Unno（TVMサーバー） | Locust、Apache Bench |
| WebSocketテスト | 両者（共同） | ws（Node.js）、websockets（Python） |

---

### 10.2 受入基準

**API仕様は以下の場合に受入:**
- ✅ すべてのエンドポイントが例付きで文書化
- ✅ すべてのデータモデルがJSONスキーマで定義
- ✅ エラーハンドリングが指定
- ✅ 両チーム用のモックインターフェースが利用可能
- ✅ 統合テストが合格（>95%成功率）
- ✅ 負荷テスト: 1 Hzテレメトリで10台の車両（10分間持続）
- ✅ WebSocketコマンド: <500msレイテンシ（95パーセンタイル）
- ✅ オフライン動作: キューと一括アップロードがテスト済み

---

## 11. デプロイメントとバージョニング

### 11.1 APIバージョニング

**URLバージョニング:** `/api/v1/...` → `/api/v2/...`

**バージョンサポート:**
- 現在のバージョン（v1）: 完全サポート
- 以前のバージョン（v0）: v1リリース後6ヶ月間サポート
- 非推奨バージョン: 削除前に3ヶ月の通知

**破壊的変更（新バージョンが必要）:**
- エンドポイントの削除
- 必須フィールドの変更
- データタイプの変更
- フィールド名の変更

**非破壊的変更（同じバージョン）:**
- オプションフィールドの追加
- 新しいエンドポイントの追加
- フィールドの非推奨化（警告期間付き）

---

### 11.2 デプロイメントチェックリスト

**本番デプロイメント前:**
- ✅ API仕様が両チームによってレビューおよび承認済み
- ✅ 統合テストが合格（>95%）
- ✅ 負荷テストが完了（10台の車両、1 Hz、10分）
- ✅ セキュリティ監査が完了（JWT実装、HTTPS）
- ✅ 監視とロギングが構成済み（CloudWatch、Prometheus）
- ✅ エラーアラートが構成済み（PagerDuty、メール）
- ✅ ドキュメントが公開済み（APIリファレンス、開発者ガイド）
- ✅ ロールバック計画が準備済み

---

## ドキュメントステータス

**ステータス:** ドラフト - レビュー待ち
**次回レビュー:** 2025-12-17（キックオフミーティング - Pankaj + Unno）
**署名が必要:** 両チームが開発開始前に承認する必要があります

---

## 付録

### 付録A: クイックリファレンス - REST APIエンドポイント

| メソッド | エンドポイント | 頻度 | 目的 |
|--------|----------|-----------|---------|
| POST | `/api/v1/auth/token` | 起動時/トークン期限切れ時 | JWTトークンを取得 |
| POST | `/api/v1/vehicle/{id}/location` | 1 Hz | 位置テレメトリ |
| POST | `/api/v1/vehicle/{id}/status` | イベント駆動/10秒 | ステータス更新 |
| POST | `/api/v1/vehicle/{id}/battery` | 5秒ごと | バッテリーテレメトリ |
| POST | `/api/v1/vehicle/{id}/error` | イベント駆動 | エラー報告 |
| POST | `/api/v1/vehicle/{id}/docking` | イベント駆動 | ドッキングステータス |
| POST | `/api/v1/vehicle/{id}/bulk` | オフライン後 | 一括メッセージアップロード |

---

### 付録B: クイックリファレンス - WebSocketコマンド

| コマンドタイプ | 方向 | 優先度 | 目的 |
|--------------|-----------|----------|---------|
| `welcome` | サーバー → 車両 | N/A | 接続確認 |
| `ping` / `pong` | 双方向 | N/A | ハートビート |
| `dispatch` | サーバー → 車両 | Normal/High/Urgent | 目的地に送る |
| `cancel` | サーバー → 車両 | High | ミッションをキャンセル |
| `emergency_stop` | サーバー → 車両 | CRITICAL | 即座に停止 |
| `resume` | サーバー → 車両 | High | E-Stopから再開 |
| `configure` | サーバー → 車両 | Normal | 構成を更新 |
| `command_ack` | 車両 → サーバー | N/A | コマンド確認 |

---

### 付録C: cURLコマンド例

**トークンを取得:**
```bash
curl -X POST https://tvm.example.com/api/v1/auth/token \
  -H "Content-Type: application/json" \
  -d '{
    "client_id": "veh_abc123xyz456",
    "client_secret": "secret_789def012ghi",
    "grant_type": "client_credentials"
  }'
```

**位置を送信:**
```bash
curl -X POST https://tvm.example.com/api/v1/vehicle/wheelchair_001/location \
  -H "Authorization: Bearer eyJhbG..." \
  -H "Content-Type: application/json" \
  -d '{
    "timestamp": "2025-12-16T10:30:45.123Z",
    "location": {
      "latitude": 35.681236,
      "longitude": 139.767125,
      "heading": 90.0,
      "speed": 0.5,
      "accuracy": 0.10
    }
  }'
```

---

**ドキュメント終了**

**クリティカルな次のステップ:**
1. ⚠️ **第1週1-2日目:** PankajとUnnoがこの仕様をレビュー
2. ⚠️ **第1週3-4日目:** API設計を議論して確定（ミーティングが必要）
3. ⚠️ **第1週5日目:** 両チームによる署名
4. ⚠️ **第1週6-10日目:** モックインターフェースを実装
5. ⚠️ **第2週以降:** 並行開発を開始（車両クライアント + TVMサーバー）

**質問/明確化:** Pankaj（車両）またはUnno（TVMサーバー）に連絡
