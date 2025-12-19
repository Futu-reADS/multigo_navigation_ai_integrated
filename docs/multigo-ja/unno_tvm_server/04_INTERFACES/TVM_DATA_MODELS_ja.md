# TVMデータモデル - 共有データ構造

**文書ID:** INT-TVM-DATA-001
**バージョン:** 1.0
**日付:** 2025-12-16
**ステータス:** ドラフト - 重要（第1週優先度2）
**分類:** 内部

**担当チーム:**
- **車両クライアント:** Pankaj（ソフトウェア - 車両側）
- **TVMサーバー:** Unno（ソフトウェア - フリート管理側）

**目的:** 車両とTVMサーバー間で交換されるすべてのメッセージの正確なデータ構造、JSONスキーマ、検証ルールを定義する。

---

## 文書管理

| バージョン | 日付 | 作成者 | 変更内容 |
|---------|------|---------|---------|
| 1.0 | 2025-12-16 | Pankaj + Unno | データモデル仕様の初版 |

---

## 目次

1. [はじめに](#1-はじめに)
2. [共通データ型](#2-共通データ型)
3. [車両テレメトリーモデル](#3-車両テレメトリーモデル)
4. [コマンドモデル](#4-コマンドモデル)
5. [エラーモデル](#5-エラーモデル)
6. [JSONスキーマ定義](#6-jsonスキーマ定義)
7. [検証ルール](#7-検証ルール)
8. [列挙型定義](#8-列挙型定義)

---

## 1. はじめに

### 1.1 目的

本文書はTVM APIのすべてのメッセージの**完全なデータモデル**を定義します：
- 各メッセージタイプのJSON構造
- フィールドタイプ、範囲、制約
- 必須フィールドとオプションフィールド
- 検証ルール
- 列挙型値の定義

**すべての実装はこれらのスキーマに対してメッセージを検証する必要があります。**

---

### 1.2 JSONスキーマ標準

**フォーマット:** JSON Schema Draft 07
**検証ライブラリ:**
- Python: `jsonschema`、`pydantic`
- JavaScript: `ajv`、`joi`
- Go: `gojsonschema`

---

### 1.3 データフォーマット規約

**タイムスタンプ:**
- フォーマット: ISO 8601 UTC
- 例: `"2025-12-16T10:30:45.123Z"`
- 精度: ミリ秒（小数点以下3桁）
- タイムゾーン: 常にUTC（Z接尾辞）

**座標:**
- システム: WGS84
- 緯度: -90.0～+90.0度
- 経度: -180.0～+180.0度
- 精度: 小数点以下6桁（約0.1メートル）

**単位:**
- 距離: メートル（m）
- 速度: メートル毎秒（m/s）
- 角度: 度（0-360、真北から時計回り）
- 温度: 摂氏（°C）
- 電圧: ボルト（V）
- 電流: アンペア（A）
- バッテリー: パーセント（0-100%）

---

## 2. 共通データ型

### 2.1 位置情報

**TypeScriptインターフェース:**
```typescript
interface Location {
  latitude: number;      // -90.0～+90.0度（WGS84）
  longitude: number;     // -180.0～+180.0度（WGS84）
  altitude?: number;     // 海抜メートル（オプション）
  heading: number;       // 0-360度（真北 = 0、時計回り）
  speed: number;         // m/s（0.0～2.0）
  accuracy: number;      // ±メートル（位置精度、0.0～1.0）
}
```

**JSONスキーマ:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "latitude": {
      "type": "number",
      "minimum": -90.0,
      "maximum": 90.0,
      "description": "WGS84度での緯度"
    },
    "longitude": {
      "type": "number",
      "minimum": -180.0,
      "maximum": 180.0,
      "description": "WGS84度での経度"
    },
    "altitude": {
      "type": "number",
      "description": "海抜メートルでの高度（オプション）"
    },
    "heading": {
      "type": "number",
      "minimum": 0.0,
      "maximum": 360.0,
      "description": "度での方位（0=北、90=東、時計回り）"
    },
    "speed": {
      "type": "number",
      "minimum": 0.0,
      "maximum": 2.0,
      "description": "m/sでの速度"
    },
    "accuracy": {
      "type": "number",
      "minimum": 0.0,
      "maximum": 1.0,
      "description": "±メートルでの位置精度"
    }
  },
  "required": ["latitude", "longitude", "heading", "speed", "accuracy"],
  "additionalProperties": false
}
```

**例:**
```json
{
  "latitude": 35.681236,
  "longitude": 139.767125,
  "altitude": 45.5,
  "heading": 90.5,
  "speed": 0.8,
  "accuracy": 0.15
}
```

---

### 2.2 タイムスタンプ

**型:** 文字列（ISO 8601 UTC）

**フォーマット:** `YYYY-MM-DDTHH:MM:SS.sssZ`

**JSONスキーマ:**
```json
{
  "type": "string",
  "format": "date-time",
  "pattern": "^\\d{4}-\\d{2}-\\d{2}T\\d{2}:\\d{2}:\\d{2}\\.\\d{3}Z$",
  "description": "ミリ秒精度のISO 8601 UTCタイムスタンプ"
}
```

**例:**
```json
"2025-12-16T10:30:45.123Z"  ✅ 有効
"2025-12-16T10:30:45Z"      ❌ 無効（ミリ秒なし）
"2025-12-16 10:30:45"       ❌ 無効（フォーマット違い）
"2025-12-16T10:30:45+09:00" ❌ 無効（UTCでない、Zを使用）
```

---

### 2.3 車両状態（列挙型）

**型:** 文字列（列挙型）

**許可値:**
```typescript
type VehicleState =
  | "idle"            // 停止、アクティブミッションなし
  | "navigating"      // 目的地に向かって能動的に移動中
  | "docking"         // 車椅子とドッキング中
  | "charging"        // ステーションで充電中
  | "error"           // システムエラー
  | "emergency_stop"; // 緊急停止作動
```

**JSONスキーマ:**
```json
{
  "type": "string",
  "enum": ["idle", "navigating", "docking", "charging", "error", "emergency_stop"],
  "description": "車両の現在の動作状態"
}
```

**状態遷移:**
```
idle ──────────> navigating ──────────> docking ──────────> idle
  │                   │                    │                    │
  │                   │                    │                    │
  └──────> charging <─┴────────────────────┴────────────────────┘
           │
           │
      error / emergency_stop（どの状態からでも）
           │
           └──────> idle（回復後）
```

---

### 2.4 エラー重大度（列挙型）

**型:** 文字列（列挙型）

**許可値:**
```typescript
type ErrorSeverity =
  | "info"     // 情報、アクション不要
  | "warning"  // 軽微な問題、車両は回復可能
  | "error"    // 重大な問題、ミッションに影響する可能性
  | "critical" // 安全上重要、即座の注意が必要
```

**JSONスキーマ:**
```json
{
  "type": "string",
  "enum": ["info", "warning", "error", "critical"],
  "description": "エラーの重大度レベル"
}
```

**重大度の動作:**
| 重大度 | スタッフ警告 | 車両動作 | 例 |
|----------|-------------|------------------|----------|
| `info` | なし | 続行 | ウェイポイント到達、タスク完了 |
| `warning` | ダッシュボードのみ | 回復を試行 | 経路遮断（再ルート中） |
| `error` | メール + ダッシュボード | 停止または帰還 | 位置推定喪失、センサー故障 |
| `critical` | SMS + 音声通話 | 緊急停止 | 衝突、緊急停止、バッテリー重要 |

---

### 2.5 コマンド優先度（列挙型）

**型:** 文字列（列挙型）

**許可値:**
```typescript
type CommandPriority =
  | "low"    // 緊急でない、通常にキュー
  | "normal" // 標準優先度（デフォルト）
  | "high"   // 緊急、優先してキュー
  | "urgent" // 非常事態、安全なら中断
```

**JSONスキーマ:**
```json
{
  "type": "string",
  "enum": ["low", "normal", "high", "urgent"],
  "default": "normal",
  "description": "ミッションディスパッチの優先度レベル"
}
```

---

## 3. 車両テレメトリーモデル

### 3.1 位置更新

**メッセージタイプ:** `POST /api/v1/vehicle/{id}/location`

**TypeScriptインターフェース:**
```typescript
interface LocationUpdate {
  timestamp: string;    // ISO 8601 UTC
  location: Location;   // 2.1参照
}
```

**JSONスキーマ:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "timestamp": {
      "$ref": "#/definitions/Timestamp"
    },
    "location": {
      "$ref": "#/definitions/Location"
    }
  },
  "required": ["timestamp", "location"],
  "additionalProperties": false
}
```

**例:**
```json
{
  "timestamp": "2025-12-16T10:30:45.123Z",
  "location": {
    "latitude": 35.681236,
    "longitude": 139.767125,
    "heading": 90.5,
    "speed": 0.8,
    "accuracy": 0.15
  }
}
```

---

### 3.2 ステータス更新

**メッセージタイプ:** `POST /api/v1/vehicle/{id}/status`

**TypeScriptインターフェース:**
```typescript
interface StatusUpdate {
  timestamp: string;
  state: VehicleState;
  mission_id?: string;
  current_waypoint?: string;
  passengers_onboard: boolean;
  operational_hours: boolean;
  details?: {
    progress_percent?: number;
    eta_seconds?: number;
    distance_remaining_meters?: number;
  };
}
```

**JSONスキーマ:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "timestamp": {
      "$ref": "#/definitions/Timestamp"
    },
    "state": {
      "$ref": "#/definitions/VehicleState"
    },
    "mission_id": {
      "type": "string",
      "pattern": "^mission_[a-zA-Z0-9]+$",
      "description": "現在のミッションID（アクティブの場合）"
    },
    "current_waypoint": {
      "type": "string",
      "description": "現在のターゲットウェイポイント（ナビゲーション中の場合）"
    },
    "passengers_onboard": {
      "type": "boolean",
      "description": "車椅子/乗客が装着されている場合true"
    },
    "operational_hours": {
      "type": "boolean",
      "description": "運用時間内の場合true"
    },
    "details": {
      "type": "object",
      "properties": {
        "progress_percent": {
          "type": "number",
          "minimum": 0,
          "maximum": 100
        },
        "eta_seconds": {
          "type": "integer",
          "minimum": 0
        },
        "distance_remaining_meters": {
          "type": "number",
          "minimum": 0
        }
      },
      "additionalProperties": false
    }
  },
  "required": ["timestamp", "state", "passengers_onboard", "operational_hours"],
  "additionalProperties": false
}
```

**例:**
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

---

### 3.3 バッテリー更新

**メッセージタイプ:** `POST /api/v1/vehicle/{id}/battery`

**TypeScriptインターフェース:**
```typescript
interface BatteryUpdate {
  timestamp: string;
  battery: {
    soc_percent: number;                // 充電状態（0-100%）
    voltage: number;                    // ボルト
    current: number;                    // アンペア（負=放電）
    temperature: number;                // 摂氏
    charging: boolean;                  // 能動的に充電中の場合true
    estimated_runtime_minutes?: number; // 残り実行時間（オプション）
    health_percent?: number;            // バッテリー健全性0-100%（オプション）
  };
}
```

**JSONスキーマ:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "timestamp": {
      "$ref": "#/definitions/Timestamp"
    },
    "battery": {
      "type": "object",
      "properties": {
        "soc_percent": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "充電状態パーセント"
        },
        "voltage": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "ボルトでのバッテリー電圧"
        },
        "current": {
          "type": "number",
          "minimum": -100,
          "maximum": 100,
          "description": "アンペアでの電流（負=放電）"
        },
        "temperature": {
          "type": "number",
          "minimum": -20,
          "maximum": 80,
          "description": "摂氏でのバッテリー温度"
        },
        "charging": {
          "type": "boolean",
          "description": "能動的に充電中の場合true"
        },
        "estimated_runtime_minutes": {
          "type": "integer",
          "minimum": 0,
          "description": "分での推定実行時間"
        },
        "health_percent": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "バッテリー健全性パーセント"
        }
      },
      "required": ["soc_percent", "voltage", "current", "temperature", "charging"],
      "additionalProperties": false
    }
  },
  "required": ["timestamp", "battery"],
  "additionalProperties": false
}
```

**例:**
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

### 3.4 エラーレポート

**メッセージタイプ:** `POST /api/v1/vehicle/{id}/error`

**TypeScriptインターフェース:**
```typescript
interface ErrorReport {
  timestamp: string;
  error_code: string;
  severity: ErrorSeverity;
  subsystem: string;
  location?: Location;
  description: string;
  recovery_action?: string;
  user_action_required: boolean;
}
```

**JSONスキーマ:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "timestamp": {
      "$ref": "#/definitions/Timestamp"
    },
    "error_code": {
      "type": "string",
      "pattern": "^[A-Z_]+$",
      "description": "エラーコード（例: NAV_OBSTACLE_BLOCKED）"
    },
    "severity": {
      "$ref": "#/definitions/ErrorSeverity"
    },
    "subsystem": {
      "type": "string",
      "enum": ["navigation", "docking", "perception", "power", "safety", "hardware"],
      "description": "エラーを生成したサブシステム"
    },
    "location": {
      "$ref": "#/definitions/Location"
    },
    "description": {
      "type": "string",
      "maxLength": 500,
      "description": "人間が読めるエラー説明"
    },
    "recovery_action": {
      "type": "string",
      "maxLength": 200,
      "description": "試行中の自動回復アクション"
    },
    "user_action_required": {
      "type": "boolean",
      "description": "手動介入が必要な場合true"
    }
  },
  "required": ["timestamp", "error_code", "severity", "subsystem", "description", "user_action_required"],
  "additionalProperties": false
}
```

**例:**
```json
{
  "timestamp": "2025-12-16T10:35:12.456Z",
  "error_code": "NAV_OBSTACLE_BLOCKED",
  "severity": "warning",
  "subsystem": "navigation",
  "location": {
    "latitude": 35.681236,
    "longitude": 139.767125,
    "heading": 90.0,
    "speed": 0.0,
    "accuracy": 0.10
  },
  "description": "障害物により経路遮断、再ルートを試行中",
  "recovery_action": "rerouting",
  "user_action_required": false
}
```

---

### 3.5 ドッキングステータス更新

**メッセージタイプ:** `POST /api/v1/vehicle/{id}/docking`

**TypeScriptインターフェース:**
```typescript
interface DockingUpdate {
  timestamp: string;
  phase: DockingPhase;
  marker_id?: number;
  distance_to_marker_meters?: number;
  alignment_error_degrees?: number;
  docking_success: boolean;
  attempts: number;
  details?: {
    marker_visible?: boolean;
    confidence?: number;
  };
}

type DockingPhase =
  | "approaching"
  | "marker_detection"
  | "visual_servoing"
  | "final_approach"
  | "attachment"
  | "success"
  | "failed";
```

**JSONスキーマ:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "timestamp": {
      "$ref": "#/definitions/Timestamp"
    },
    "phase": {
      "type": "string",
      "enum": ["approaching", "marker_detection", "visual_servoing", "final_approach", "attachment", "success", "failed"]
    },
    "marker_id": {
      "type": "integer",
      "minimum": 0,
      "maximum": 999,
      "description": "ArUcoマーカーID"
    },
    "distance_to_marker_meters": {
      "type": "number",
      "minimum": 0,
      "maximum": 10,
      "description": "メートルでのマーカーまでの距離"
    },
    "alignment_error_degrees": {
      "type": "number",
      "minimum": 0,
      "maximum": 180,
      "description": "度での整列誤差"
    },
    "docking_success": {
      "type": "boolean",
      "description": "ドッキングが成功した場合true"
    },
    "attempts": {
      "type": "integer",
      "minimum": 0,
      "description": "ドッキング試行回数"
    },
    "details": {
      "type": "object",
      "properties": {
        "marker_visible": {
          "type": "boolean"
        },
        "confidence": {
          "type": "number",
          "minimum": 0,
          "maximum": 1
        }
      }
    }
  },
  "required": ["timestamp", "phase", "docking_success", "attempts"],
  "additionalProperties": false
}
```

**例:**
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

---

## 4. コマンドモデル

### 4.1 ディスパッチコマンド

**WebSocketメッセージタイプ:** `dispatch`

**TypeScriptインターフェース:**
```typescript
interface DispatchCommand {
  type: "dispatch";
  command_id: string;
  timestamp: string;
  priority: CommandPriority;
  mission: {
    mission_id: string;
    pickup?: {
      waypoint_id: string;
      latitude: number;
      longitude: number;
      action: "dock_wheelchair" | "wait";
    };
    destination: {
      waypoint_id: string;
      latitude: number;
      longitude: number;
      action: "undock_wheelchair" | "park";
    };
    scheduled_time?: string;
  };
}
```

**JSONスキーマ:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "type": {
      "const": "dispatch"
    },
    "command_id": {
      "type": "string",
      "pattern": "^cmd_[a-zA-Z0-9]+$"
    },
    "timestamp": {
      "$ref": "#/definitions/Timestamp"
    },
    "priority": {
      "$ref": "#/definitions/CommandPriority"
    },
    "mission": {
      "type": "object",
      "properties": {
        "mission_id": {
          "type": "string",
          "pattern": "^mission_[a-zA-Z0-9]+$"
        },
        "pickup": {
          "type": "object",
          "properties": {
            "waypoint_id": {
              "type": "string"
            },
            "latitude": {
              "type": "number",
              "minimum": -90,
              "maximum": 90
            },
            "longitude": {
              "type": "number",
              "minimum": -180,
              "maximum": 180
            },
            "action": {
              "type": "string",
              "enum": ["dock_wheelchair", "wait"]
            }
          },
          "required": ["waypoint_id", "latitude", "longitude", "action"]
        },
        "destination": {
          "type": "object",
          "properties": {
            "waypoint_id": {
              "type": "string"
            },
            "latitude": {
              "type": "number",
              "minimum": -90,
              "maximum": 90
            },
            "longitude": {
              "type": "number",
              "minimum": -180,
              "maximum": 180
            },
            "action": {
              "type": "string",
              "enum": ["undock_wheelchair", "park"]
            }
          },
          "required": ["waypoint_id", "latitude", "longitude", "action"]
        },
        "scheduled_time": {
          "$ref": "#/definitions/Timestamp"
        }
      },
      "required": ["mission_id", "destination"]
    }
  },
  "required": ["type", "command_id", "timestamp", "priority", "mission"],
  "additionalProperties": false
}
```

**例:**
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

---

### 4.2 コマンド確認応答

**WebSocketメッセージタイプ:** `command_ack`

**TypeScriptインターフェース:**
```typescript
interface CommandAcknowledgment {
  type: "command_ack";
  command_id: string;
  status: "accepted" | "rejected" | "executed";
  reason?: string;
  message?: string;
  estimated_arrival?: string;
  stopped_at?: Location;
}
```

**JSONスキーマ:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "type": {
      "const": "command_ack"
    },
    "command_id": {
      "type": "string"
    },
    "status": {
      "type": "string",
      "enum": ["accepted", "rejected", "executed"]
    },
    "reason": {
      "type": "string",
      "enum": ["battery_too_low", "already_on_mission", "in_error_state", "outside_operational_hours", "invalid_destination"]
    },
    "message": {
      "type": "string",
      "maxLength": 200
    },
    "estimated_arrival": {
      "$ref": "#/definitions/Timestamp"
    },
    "stopped_at": {
      "$ref": "#/definitions/Location"
    }
  },
  "required": ["type", "command_id", "status"],
  "additionalProperties": false
}
```

**例（受理）:**
```json
{
  "type": "command_ack",
  "command_id": "cmd_12345",
  "status": "accepted",
  "estimated_arrival": "2025-12-16T10:55:00.000Z",
  "message": "ミッション受理、ナビゲーション開始"
}
```

**例（拒否）:**
```json
{
  "type": "command_ack",
  "command_id": "cmd_12345",
  "status": "rejected",
  "reason": "battery_too_low",
  "message": "バッテリー15%、ミッションに不十分"
}
```

---

### 4.3 緊急停止コマンド

**WebSocketメッセージタイプ:** `emergency_stop`

**TypeScriptインターフェース:**
```typescript
interface EmergencyStopCommand {
  type: "emergency_stop";
  command_id: string;
  timestamp: string;
  reason: string;
  message?: string;
}
```

**JSONスキーマ:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "type": {
      "const": "emergency_stop"
    },
    "command_id": {
      "type": "string",
      "pattern": "^cmd_ESTOP_[a-zA-Z0-9]+$"
    },
    "timestamp": {
      "$ref": "#/definitions/Timestamp"
    },
    "reason": {
      "type": "string",
      "maxLength": 200
    },
    "message": {
      "type": "string",
      "maxLength": 500
    }
  },
  "required": ["type", "command_id", "timestamp", "reason"],
  "additionalProperties": false
}
```

**例:**
```json
{
  "type": "emergency_stop",
  "command_id": "cmd_ESTOP_34567",
  "timestamp": "2025-12-16T10:50:00.000Z",
  "reason": "staff_initiated",
  "message": "オペレーターによる緊急停止要求"
}
```

---

## 5. エラーモデル

### 5.1 HTTPエラーレスポンス

**TypeScriptインターフェース:**
```typescript
interface HTTPError {
  error: string;
  message: string;
  field?: string;
  received_value?: any;
  timestamp: string;
}
```

**JSONスキーマ:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "error": {
      "type": "string",
      "description": "機械可読エラーコード"
    },
    "message": {
      "type": "string",
      "description": "人間可読エラーメッセージ"
    },
    "field": {
      "type": "string",
      "description": "エラーの原因となったフィールド（該当する場合）"
    },
    "received_value": {
      "description": "受信した無効な値（任意の型）"
    },
    "timestamp": {
      "$ref": "#/definitions/Timestamp"
    }
  },
  "required": ["error", "message", "timestamp"],
  "additionalProperties": false
}
```

**例:**
```json
{
  "error": "invalid_location",
  "message": "緯度が範囲外",
  "field": "location.latitude",
  "received_value": 95.123,
  "timestamp": "2025-12-16T10:30:45.234Z"
}
```

---

## 6. JSONスキーマ定義

### 6.1 完全スキーマバンドル

**ファイル名:** `tvm_api_schemas.json`

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "definitions": {
    "Timestamp": {
      "type": "string",
      "format": "date-time",
      "pattern": "^\\d{4}-\\d{2}-\\d{2}T\\d{2}:\\d{2}:\\d{2}\\.\\d{3}Z$"
    },
    "Location": {
      "type": "object",
      "properties": {
        "latitude": { "type": "number", "minimum": -90, "maximum": 90 },
        "longitude": { "type": "number", "minimum": -180, "maximum": 180 },
        "altitude": { "type": "number" },
        "heading": { "type": "number", "minimum": 0, "maximum": 360 },
        "speed": { "type": "number", "minimum": 0, "maximum": 2 },
        "accuracy": { "type": "number", "minimum": 0, "maximum": 1 }
      },
      "required": ["latitude", "longitude", "heading", "speed", "accuracy"]
    },
    "VehicleState": {
      "type": "string",
      "enum": ["idle", "navigating", "docking", "charging", "error", "emergency_stop"]
    },
    "ErrorSeverity": {
      "type": "string",
      "enum": ["info", "warning", "error", "critical"]
    },
    "CommandPriority": {
      "type": "string",
      "enum": ["low", "normal", "high", "urgent"]
    }
  }
}
```

---

## 7. 検証ルール

### 7.1 フィールド検証

**すべての実装は以下を検証する必要があります:**

| フィールド | 検証ルール | 無効な場合のエラー |
|-------|----------------|------------------|
| `latitude` | -90.0 ≤ 値 ≤ 90.0 | `invalid_location` |
| `longitude` | -180.0 ≤ 値 ≤ 180.0 | `invalid_location` |
| `heading` | 0.0 ≤ 値 ≤ 360.0 | `invalid_heading` |
| `speed` | 0.0 ≤ 値 ≤ 2.0 | `invalid_speed` |
| `timestamp` | 有効なISO 8601 UTC、未来ではない | `invalid_timestamp` |
| `soc_percent` | 0.0 ≤ 値 ≤ 100.0 | `invalid_battery` |
| `vehicle_id` | 登録済み車両と一致 | `unknown_vehicle` |
| `command_id` | 一意、再利用されない | `duplicate_command` |

---

### 7.2 タイムスタンプ検証

**ルール:**
1. 有効なISO 8601 UTCフォーマットである必要がある
2. ミリ秒精度（小数点以下3桁）が必要
3. 'Z'接尾辞（UTCタイムゾーン）が必要
4. 未来であってはならない（5秒のクロックスキューを許容）
5. 最近のものであるべき（1時間以上古い場合は警告）

**Python検証例:**
```python
from datetime import datetime, timedelta

def validate_timestamp(timestamp_str: str) -> bool:
    try:
        ts = datetime.fromisoformat(timestamp_str.replace('Z', '+00:00'))
        now = datetime.utcnow()

        # 未来ではない（5秒のスキューを許容）
        if ts > now + timedelta(seconds=5):
            raise ValueError("タイムスタンプが未来")

        # 古すぎない（警告のみ）
        if ts < now - timedelta(hours=1):
            print("警告: タイムスタンプが1時間以上古い")

        return True
    except Exception as e:
        raise ValueError(f"無効なタイムスタンプ: {e}")
```

---

### 7.3 位置検証

**ルール:**
1. 緯度: -90～+90度
2. 経度: -180～+180度
3. 精度: 0～1.0メートル（屋外NDT位置推定に妥当）
4. 速度: 0～2.0 m/s（最大車両速度）

**Python検証例:**
```python
def validate_location(location: dict) -> bool:
    if not (-90 <= location['latitude'] <= 90):
        raise ValueError("無効な緯度")
    if not (-180 <= location['longitude'] <= 180):
        raise ValueError("無効な経度")
    if not (0 <= location['speed'] <= 2.0):
        raise ValueError("無効な速度")
    if not (0 <= location['accuracy'] <= 1.0):
        raise ValueError("無効な精度")
    return True
```

---

## 8. 列挙型定義

### 8.1 車両状態

```typescript
enum VehicleState {
  IDLE = "idle",
  NAVIGATING = "navigating",
  DOCKING = "docking",
  CHARGING = "charging",
  ERROR = "error",
  EMERGENCY_STOP = "emergency_stop"
}
```

---

### 8.2 エラー重大度

```typescript
enum ErrorSeverity {
  INFO = "info",
  WARNING = "warning",
  ERROR = "error",
  CRITICAL = "critical"
}
```

---

### 8.3 コマンド優先度

```typescript
enum CommandPriority {
  LOW = "low",
  NORMAL = "normal",
  HIGH = "high",
  URGENT = "urgent"
}
```

---

### 8.4 ドッキングフェーズ

```typescript
enum DockingPhase {
  APPROACHING = "approaching",
  MARKER_DETECTION = "marker_detection",
  VISUAL_SERVOING = "visual_servoing",
  FINAL_APPROACH = "final_approach",
  ATTACHMENT = "attachment",
  SUCCESS = "success",
  FAILED = "failed"
}
```

---

### 8.5 エラーコード

**一般的なエラーコード:**

| コード | サブシステム | 重大度 | 説明 |
|------|-----------|----------|-------------|
| `NAV_OBSTACLE_BLOCKED` | navigation | warning | 障害物により経路遮断 |
| `NAV_LOCALIZATION_LOST` | navigation | error | 位置推定信頼度が低すぎる |
| `NAV_PATH_NOT_FOUND` | navigation | error | 目的地への有効な経路がない |
| `DOCK_ALIGNMENT_FAILED` | docking | warning | ドッキング整列失敗 |
| `DOCK_MARKER_NOT_FOUND` | docking | error | ArUcoマーカー未検出 |
| `DOCK_ATTACHMENT_FAILED` | docking | error | 物理的装着失敗 |
| `SAFE_ESTOP_ACTIVATED` | safety | critical | 緊急停止トリガー |
| `SAFE_COLLISION_DETECTED` | safety | critical | バンパーによる衝突検出 |
| `POWER_BATTERY_CRITICAL` | power | critical | バッテリーが10%未満 |
| `SENSOR_LIDAR_FAILURE` | hardware | error | LiDARセンサー故障 |
| `SENSOR_CAMERA_FAILURE` | hardware | error | カメラセンサー故障 |
| `SENSOR_IMU_FAILURE` | hardware | error | IMUセンサー故障 |

---

## 文書ステータス

**ステータス:** ドラフト - レビュー待ち
**次回レビュー:** 2025-12-17（TVM_API_SPECIFICATION.mdと合同レビュー）
**承認必要:** Pankaj（車両）とUnno（TVMサーバー）の両方

---

## 付録

### 付録A: Python Pydanticモデル

**Python検証用のPydanticモデル例:**

```python
from pydantic import BaseModel, Field, validator
from datetime import datetime
from typing import Optional, Literal

class Location(BaseModel):
    latitude: float = Field(..., ge=-90, le=90)
    longitude: float = Field(..., ge=-180, le=180)
    altitude: Optional[float] = None
    heading: float = Field(..., ge=0, le=360)
    speed: float = Field(..., ge=0, le=2.0)
    accuracy: float = Field(..., ge=0, le=1.0)

class LocationUpdate(BaseModel):
    timestamp: datetime
    location: Location

    @validator('timestamp')
    def timestamp_not_future(cls, v):
        if v > datetime.utcnow():
            raise ValueError('タイムスタンプは未来にできません')
        return v

# 使用方法:
try:
    update = LocationUpdate.parse_obj({
        "timestamp": "2025-12-16T10:30:45.123Z",
        "location": {
            "latitude": 35.681236,
            "longitude": 139.767125,
            "heading": 90.0,
            "speed": 0.8,
            "accuracy": 0.15
        }
    })
    print("有効!")
except Exception as e:
    print(f"無効: {e}")
```

---

### 付録B: TypeScript検証

**TypeScript用のZodスキーマ例:**

```typescript
import { z } from 'zod';

const LocationSchema = z.object({
  latitude: z.number().min(-90).max(90),
  longitude: z.number().min(-180).max(180),
  altitude: z.number().optional(),
  heading: z.number().min(0).max(360),
  speed: z.number().min(0).max(2.0),
  accuracy: z.number().min(0).max(1.0),
});

const LocationUpdateSchema = z.object({
  timestamp: z.string().datetime(),
  location: LocationSchema,
});

// 使用方法:
try {
  const update = LocationUpdateSchema.parse({
    timestamp: "2025-12-16T10:30:45.123Z",
    location: {
      latitude: 35.681236,
      longitude: 139.767125,
      heading: 90.0,
      speed: 0.8,
      accuracy: 0.15,
    },
  });
  console.log("有効!");
} catch (e) {
  console.error("無効:", e.errors);
}
```

---

**文書終了**

**次のステップ:**
1. TVM_API_SPECIFICATION.mdと併せてレビュー
2. 車両クライアントとTVMサーバーの両方でJSONスキーマを検証
3. コードで検証を実装（Pydantic/Zod）
4. モックデータでメッセージ検証をテスト
