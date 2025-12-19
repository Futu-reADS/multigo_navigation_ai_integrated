# TVM API実装ガイド（海野さん向け）

**文書種類:** 実装ガイド
**対象読者:** 海野（TVMサーバーチーム）
**ステータス:** アクティブ
**バージョン:** 1.0
**日付:** 2025年12月19日
**作成者:** パンカジ（車両ソフトウェアチーム）

---

## 目次

1. [クイックスタート](#1-クイックスタート)
2. [API概要](#2-api概要)
3. [ステップバイステップ実装](#3-ステップバイステップ実装)
4. [コード例](#4-コード例)
5. [テストガイド](#5-テストガイド)
6. [モック車両クライアント](#6-モック車両クライアント)
7. [統合テスト](#7-統合テスト)
8. [トラブルシューティング](#8-トラブルシューティング)

---

## 1. クイックスタート

### 1.1 実装する必要があるもの

**あなたの責任（海野さん）:**
- ✅ TVMサーバーバックエンド（REST API + WebSocket）
- ✅ データベース（PostgreSQL/MySQL/MongoDB - あなたの選択）
- ✅ ユーザー認証（車両とユーザー向けのJWT）
- ✅ フリートダッシュボード（React/Vue/Angular - あなたの選択）

**パンカジが提供するもの:**
- ✅ API仕様（TVM_API_SPECIFICATION.md）
- ✅ データモデル（TVM_DATA_MODELS.md）
- ✅ テスト用モック車両クライアント（このガイド）
- ✅ あなたのAPIを呼び出す車両ソフトウェア

### 1.2 技術選択

**あなたが決定:**
- バックエンドフレームワーク: Node.js（Express/NestJS）、Python（FastAPI/Django）、Java（Spring Boot）
- データベース: PostgreSQL（推奨）、MySQL、MongoDB
- フロントエンド: React（推奨）、Vue、Angular

**このガイドで使用:**
- Python + FastAPI（例）
- PostgreSQL（データベーススキーマ）
- ただし、選択したスタックに適応できます

### 1.3 タイムライン

**1-2週目:** 認証 + 基本エンドポイント
**3-5週目:** データベーススキーマ + CRUD操作
**6-8週目:** ミッションコマンド用WebSocket
**9-11週目:** フリートダッシュボード（基本）
**12-15週目:** テスト + パンカジとの統合

---

## 2. API概要

### 2.1 サーバーが提供する必要があるもの

**REST API（車両 → サーバー）:**
```
POST /api/v1/auth/login          - 車両認証
POST /api/v1/telemetry           - テレメトリーアップロード（車両から1Hz）
POST /api/v1/missions/{id}/status - ミッションステータス更新
POST /api/v1/errors              - エラー報告
GET  /api/v1/vehicles/{id}/config - 車両設定取得
```

**WebSocket（サーバー → 車両）:**
```
mission.dispatch   - 車両に新しいミッションを送信
mission.cancel     - 現在のミッションをキャンセル
emergency.stop     - 緊急停止コマンド
config.update      - 車両設定を更新
```

**ダッシュボードAPI（ダッシュボード → サーバー）:**
```
POST /api/v1/auth/login          - ユーザーログイン
GET  /api/v1/vehicles            - すべての車両をリスト
POST /api/v1/missions            - 新しいミッションを作成
GET  /api/v1/missions/{id}       - ミッション詳細取得
GET  /api/v1/telemetry/latest    - 最新テレメトリー取得
```

### 2.2 認証フロー

```
車両起動:
1. 車両送信: POST /api/v1/auth/login
   ボディ: {"vehicle_id": "VH-001", "api_key": "secret_key"}
2. サーバー応答: {"access_token": "jwt_token", "expires_in": 3600}
3. 車両がトークンを保存し、以降のすべてのリクエストで使用
4. 車両は有効期限前にトークンを更新

ユーザーログイン:
1. ユーザーがダッシュボードでユーザー名/パスワードを入力
2. ダッシュボード送信: POST /api/v1/auth/login
   ボディ: {"username": "operator1", "password": "password"}
3. サーバーが検証し、応答: {"access_token": "jwt_token", "role": "operator"}
4. ダッシュボードがトークンを保存し、すべてのリクエストで使用
```

---

## 3. ステップバイステップ実装

### 3.1 フェーズ1: 認証（1-2週目）

**ステップ1: プロジェクト構造のセットアップ**

```bash
# FastAPI（Python）の例
mkdir tvm_server
cd tvm_server
python3 -m venv venv
source venv/bin/activate
pip install fastapi uvicorn pyjwt psycopg2-binary python-jose
```

**ステップ2: JWT認証の実装**

[コード例](#4-コード例)セクション4.1を参照

**ステップ3: 車両ログインエンドポイントの作成**

```python
# POST /api/v1/auth/login（車両用）
@app.post("/api/v1/auth/login")
async def vehicle_login(credentials: VehicleCredentials):
    # 1. vehicle_idとapi_keyを検証
    vehicle = db.get_vehicle_by_id(credentials.vehicle_id)
    if not vehicle or vehicle.api_key != credentials.api_key:
        raise HTTPException(401, "Invalid credentials")

    # 2. JWTトークンを生成
    payload = {
        "vehicle_id": credentials.vehicle_id,
        "exp": datetime.utcnow() + timedelta(hours=1)
    }
    token = jwt.encode(payload, SECRET_KEY, algorithm="HS256")

    # 3. トークンを返す
    return {"access_token": token, "token_type": "bearer", "expires_in": 3600}
```

**ステップ4: curlでテスト**

```bash
curl -X POST http://localhost:8000/api/v1/auth/login \
  -H "Content-Type: application/json" \
  -d '{"vehicle_id": "VH-001", "api_key": "test_key_123"}'

# 期待される応答:
# {"access_token": "eyJ0eXAiOiJKV1...", "token_type": "bearer", "expires_in": 3600}
```

---

### 3.2 フェーズ2: データベースセットアップ（3-5週目）

**ステップ1: PostgreSQLデータベースの作成**

```sql
CREATE DATABASE tvm_fleet;
CREATE USER tvm_user WITH PASSWORD 'secure_password';
GRANT ALL PRIVILEGES ON DATABASE tvm_fleet TO tvm_user;
```

**ステップ2: テーブルの作成**

完全なスキーマについてはTVM_SERVER_REQUIREMENTS.mdのセクション2を参照。

**MVP用の主要テーブル（最小限）:**

```sql
-- 車両テーブル
CREATE TABLE vehicles (
    id SERIAL PRIMARY KEY,
    vehicle_id VARCHAR(50) UNIQUE NOT NULL,
    model VARCHAR(100),
    api_key VARCHAR(255) NOT NULL,
    status VARCHAR(20) DEFAULT 'offline',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- ミッションテーブル
CREATE TABLE missions (
    id SERIAL PRIMARY KEY,
    vehicle_id VARCHAR(50) REFERENCES vehicles(vehicle_id),
    status VARCHAR(20) DEFAULT 'pending',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    started_at TIMESTAMP,
    completed_at TIMESTAMP
);

-- ミッションウェイポイントテーブル
CREATE TABLE mission_waypoints (
    id SERIAL PRIMARY KEY,
    mission_id INTEGER REFERENCES missions(id),
    sequence INTEGER NOT NULL,
    x FLOAT NOT NULL,
    y FLOAT NOT NULL,
    heading FLOAT,
    action VARCHAR(50)
);

-- テレメトリーテーブル（時系列データ）
CREATE TABLE vehicle_telemetry (
    id SERIAL PRIMARY KEY,
    vehicle_id VARCHAR(50) REFERENCES vehicles(vehicle_id),
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    x FLOAT,
    y FLOAT,
    heading FLOAT,
    battery_percent INTEGER,
    status VARCHAR(20),
    cpu_usage FLOAT,
    ram_usage FLOAT
);

-- ユーザーテーブル（ダッシュボード用）
CREATE TABLE users (
    id SERIAL PRIMARY KEY,
    username VARCHAR(50) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    role VARCHAR(20) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

**ステップ3: テストデータの挿入**

```sql
-- テスト車両
INSERT INTO vehicles (vehicle_id, model, api_key, status)
VALUES ('VH-001', 'MultiGo-Outdoor-v1', 'test_key_123', 'offline');

-- テストユーザー
INSERT INTO users (username, password_hash, role)
VALUES ('admin', '<bcrypt_hash>', 'admin');
```

---

### 3.3 フェーズ3: テレメトリーエンドポイント（6-8週目）

**実装:**

```python
# POST /api/v1/telemetry
@app.post("/api/v1/telemetry")
async def upload_telemetry(
    telemetry: TelemetryData,
    vehicle_id: str = Depends(get_current_vehicle)  # JWTから
):
    # 1. テレメトリーデータを検証
    if not validate_telemetry(telemetry):
        raise HTTPException(400, "Invalid telemetry data")

    # 2. データベースに保存
    db.insert_telemetry(vehicle_id, telemetry)

    # 3. 車両ステータスを更新
    db.update_vehicle_status(vehicle_id, telemetry.status)

    # 4. 成功を返す
    return {"status": "ok", "timestamp": datetime.utcnow().isoformat()}
```

**車両からの期待されるリクエスト（1秒ごと）:**

```json
POST /api/v1/telemetry
Authorization: Bearer <jwt_token>

{
  "timestamp": "2025-12-19T10:30:45Z",
  "location": {
    "x": 12.5,
    "y": 8.3,
    "heading": 45.0
  },
  "status": "NAVIGATING",
  "battery": {
    "percent": 75,
    "voltage": 48.2,
    "current": 5.5
  },
  "system": {
    "cpu_usage": 45.2,
    "ram_usage": 62.1,
    "disk_usage": 38.5
  }
}
```

---

### 3.4 フェーズ4: コマンド用WebSocket（9-11週目）

**ステップ1: WebSocketサーバーのセットアップ**

```python
from fastapi import WebSocket

# WebSocketエンドポイント
@app.websocket("/ws/{vehicle_id}")
async def websocket_endpoint(websocket: WebSocket, vehicle_id: str):
    # 1. 車両を認証（クエリパラメータまたは最初のメッセージでJWTをチェック）
    await websocket.accept()

    # 2. 接続を登録
    connection_manager.connect(vehicle_id, websocket)

    try:
        while True:
            # 3. 車両からメッセージを受信（ハートビート、ステータス）
            data = await websocket.receive_json()

            if data["type"] == "heartbeat":
                # last_seenタイムスタンプを更新
                db.update_vehicle_heartbeat(vehicle_id)
                await websocket.send_json({"type": "ack"})

            elif data["type"] == "mission_status":
                # データベースのミッションステータスを更新
                db.update_mission_status(data["mission_id"], data["status"])

    except WebSocketDisconnect:
        connection_manager.disconnect(vehicle_id)
```

**ステップ2: ミッション配信コマンドの送信**

```python
# オペレーターがダッシュボード経由でミッションを作成したとき
@app.post("/api/v1/missions")
async def create_mission(mission: MissionCreate, user_id: str = Depends(get_current_user)):
    # 1. データベースにミッションを作成
    mission_id = db.create_mission(mission.vehicle_id, mission.waypoints)

    # 2. WebSocket経由で車両にコマンドを送信
    command = {
        "type": "mission.dispatch",
        "mission_id": mission_id,
        "waypoints": mission.waypoints
    }

    await connection_manager.send_to_vehicle(mission.vehicle_id, command)

    return {"mission_id": mission_id, "status": "dispatched"}
```

**車両が受信:**

```json
{
  "type": "mission.dispatch",
  "mission_id": 12345,
  "waypoints": [
    {"x": 10.0, "y": 5.0, "heading": 0.0, "action": "navigate"},
    {"x": 15.0, "y": 8.0, "heading": 90.0, "action": "navigate"},
    {"x": 15.0, "y": 8.0, "heading": 180.0, "action": "dock"}
  ]
}
```

---

## 4. コード例

### 4.1 JWT認証ヘルパー

```python
# auth.py
from jose import JWTError, jwt
from datetime import datetime, timedelta
from fastapi import HTTPException, Depends
from fastapi.security import HTTPBearer

SECRET_KEY = "your-secret-key-here"  # 環境変数を使用してください！
ALGORITHM = "HS256"

security = HTTPBearer()

def create_access_token(data: dict, expires_delta: timedelta = timedelta(hours=1)):
    to_encode = data.copy()
    expire = datetime.utcnow() + expires_delta
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

def verify_token(token: str):
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        return payload
    except JWTError:
        raise HTTPException(status_code=401, detail="Invalid token")

def get_current_vehicle(credentials = Depends(security)):
    token = credentials.credentials
    payload = verify_token(token)
    vehicle_id = payload.get("vehicle_id")
    if vehicle_id is None:
        raise HTTPException(status_code=401, detail="Invalid authentication")
    return vehicle_id
```

### 4.2 データベース接続（PostgreSQL）

```python
# database.py
import psycopg2
from psycopg2.extras import RealDictCursor

class Database:
    def __init__(self):
        self.conn = psycopg2.connect(
            host="localhost",
            database="tvm_fleet",
            user="tvm_user",
            password="secure_password"
        )

    def get_vehicle_by_id(self, vehicle_id: str):
        cursor = self.conn.cursor(cursor_factory=RealDictCursor)
        cursor.execute("SELECT * FROM vehicles WHERE vehicle_id = %s", (vehicle_id,))
        return cursor.fetchone()

    def insert_telemetry(self, vehicle_id: str, telemetry: dict):
        cursor = self.conn.cursor()
        cursor.execute("""
            INSERT INTO vehicle_telemetry
            (vehicle_id, x, y, heading, battery_percent, status, cpu_usage, ram_usage)
            VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
        """, (
            vehicle_id,
            telemetry["location"]["x"],
            telemetry["location"]["y"],
            telemetry["location"]["heading"],
            telemetry["battery"]["percent"],
            telemetry["status"],
            telemetry["system"]["cpu_usage"],
            telemetry["system"]["ram_usage"]
        ))
        self.conn.commit()

    def create_mission(self, vehicle_id: str, waypoints: list):
        cursor = self.conn.cursor()
        # ミッションを挿入
        cursor.execute("""
            INSERT INTO missions (vehicle_id, status)
            VALUES (%s, 'pending') RETURNING id
        """, (vehicle_id,))
        mission_id = cursor.fetchone()[0]

        # ウェイポイントを挿入
        for seq, wp in enumerate(waypoints):
            cursor.execute("""
                INSERT INTO mission_waypoints (mission_id, sequence, x, y, heading, action)
                VALUES (%s, %s, %s, %s, %s, %s)
            """, (mission_id, seq, wp["x"], wp["y"], wp.get("heading", 0), wp.get("action", "navigate")))

        self.conn.commit()
        return mission_id
```

### 4.3 WebSocket接続マネージャー

```python
# websocket_manager.py
from fastapi import WebSocket
from typing import Dict

class ConnectionManager:
    def __init__(self):
        self.active_connections: Dict[str, WebSocket] = {}

    def connect(self, vehicle_id: str, websocket: WebSocket):
        self.active_connections[vehicle_id] = websocket

    def disconnect(self, vehicle_id: str):
        if vehicle_id in self.active_connections:
            del self.active_connections[vehicle_id]

    async def send_to_vehicle(self, vehicle_id: str, message: dict):
        if vehicle_id in self.active_connections:
            await self.active_connections[vehicle_id].send_json(message)
        else:
            raise Exception(f"Vehicle {vehicle_id} not connected")

    async def broadcast(self, message: dict):
        for connection in self.active_connections.values():
            await connection.send_json(message)

manager = ConnectionManager()
```

---

## 5. テストガイド

### 5.1 ユニットテスト

**認証のテスト:**

```python
# test_auth.py
import pytest
from fastapi.testclient import TestClient
from main import app

client = TestClient(app)

def test_vehicle_login_success():
    response = client.post("/api/v1/auth/login", json={
        "vehicle_id": "VH-001",
        "api_key": "test_key_123"
    })
    assert response.status_code == 200
    assert "access_token" in response.json()

def test_vehicle_login_invalid_credentials():
    response = client.post("/api/v1/auth/login", json={
        "vehicle_id": "VH-999",
        "api_key": "wrong_key"
    })
    assert response.status_code == 401
```

**テレメトリーエンドポイントのテスト:**

```python
def test_upload_telemetry():
    # まず、認証トークンを取得
    login_response = client.post("/api/v1/auth/login", json={
        "vehicle_id": "VH-001",
        "api_key": "test_key_123"
    })
    token = login_response.json()["access_token"]

    # テレメトリーをアップロード
    response = client.post("/api/v1/telemetry",
        headers={"Authorization": f"Bearer {token}"},
        json={
            "timestamp": "2025-12-19T10:30:45Z",
            "location": {"x": 10.0, "y": 5.0, "heading": 0.0},
            "status": "IDLE",
            "battery": {"percent": 80, "voltage": 48.5, "current": 0.5},
            "system": {"cpu_usage": 25.0, "ram_usage": 50.0, "disk_usage": 30.0}
        }
    )
    assert response.status_code == 200
```

### 5.2 Postmanでの手動テスト

**コレクション構造:**

```
TVM API Tests/
├── Auth/
│   ├── Vehicle Login
│   └── User Login
├── Telemetry/
│   ├── Upload Telemetry
│   └── Get Latest Telemetry
├── Missions/
│   ├── Create Mission
│   ├── Get Mission
│   └── Update Mission Status
└── WebSocket/
    └── Test Connection
```

**環境変数:**
```
base_url: http://localhost:8000
vehicle_token: {{vehicle_access_token}}
user_token: {{user_access_token}}
```

---

## 6. モック車両クライアント

### 6.1 目的

このモッククライアントは、**パンカジの実際の車両ソフトウェアを必要とせずに**、TVMサーバーをテストするために車両をシミュレートします。

### 6.2 モッククライアントコード

```python
# mock_vehicle_client.py
import requests
import websocket
import json
import time
from datetime import datetime

class MockVehicleClient:
    def __init__(self, vehicle_id, api_key, server_url):
        self.vehicle_id = vehicle_id
        self.api_key = api_key
        self.server_url = server_url
        self.access_token = None
        self.ws = None

    def login(self):
        """認証してJWTトークンを取得"""
        response = requests.post(
            f"{self.server_url}/api/v1/auth/login",
            json={"vehicle_id": self.vehicle_id, "api_key": self.api_key}
        )
        if response.status_code == 200:
            self.access_token = response.json()["access_token"]
            print(f"✅ {self.vehicle_id}としてログイン")
            return True
        else:
            print(f"❌ ログイン失敗: {response.text}")
            return False

    def send_telemetry(self):
        """テレメトリーをサーバーに送信（1Hzをシミュレート）"""
        telemetry = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "location": {"x": 10.0, "y": 5.0, "heading": 0.0},
            "status": "IDLE",
            "battery": {"percent": 80, "voltage": 48.5, "current": 0.5},
            "system": {"cpu_usage": 25.0, "ram_usage": 50.0, "disk_usage": 30.0}
        }

        response = requests.post(
            f"{self.server_url}/api/v1/telemetry",
            headers={"Authorization": f"Bearer {self.access_token}"},
            json=telemetry
        )

        if response.status_code == 200:
            print(f"✅ {telemetry['timestamp']}にテレメトリー送信")
        else:
            print(f"❌ テレメトリー失敗: {response.text}")

    def connect_websocket(self):
        """WebSocketに接続してコマンドをリッスン"""
        ws_url = self.server_url.replace("http", "ws") + f"/ws/{self.vehicle_id}"
        self.ws = websocket.WebSocketApp(
            ws_url,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )
        self.ws.on_open = self.on_open
        self.ws.run_forever()

    def on_open(self, ws):
        print(f"✅ WebSocket接続完了")
        # 5秒ごとにハートビートを送信
        def send_heartbeat():
            while True:
                ws.send(json.dumps({"type": "heartbeat"}))
                time.sleep(5)

        import threading
        threading.Thread(target=send_heartbeat, daemon=True).start()

    def on_message(self, ws, message):
        data = json.loads(message)
        print(f"📨 コマンド受信: {data['type']}")

        if data["type"] == "mission.dispatch":
            print(f"  ミッションID: {data['mission_id']}")
            print(f"  ウェイポイント: {len(data['waypoints'])}")
            # ミッション受諾をシミュレート
            ws.send(json.dumps({
                "type": "mission_status",
                "mission_id": data["mission_id"],
                "status": "accepted"
            }))

        elif data["type"] == "emergency.stop":
            print(f"  🚨 緊急停止！")

    def on_error(self, ws, error):
        print(f"❌ WebSocketエラー: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        print(f"⚠️  WebSocket切断: {close_msg}")

    def run(self):
        """メインループ: ログイン、テレメトリー送信、WebSocket接続"""
        if not self.login():
            return

        # バックグラウンドスレッドでテレメトリーを送信
        import threading
        def telemetry_loop():
            while True:
                self.send_telemetry()
                time.sleep(1)  # 1Hz

        threading.Thread(target=telemetry_loop, daemon=True).start()

        # WebSocketに接続（ブロッキング）
        self.connect_websocket()

# 使用方法
if __name__ == "__main__":
    client = MockVehicleClient(
        vehicle_id="VH-001",
        api_key="test_key_123",
        server_url="http://localhost:8000"
    )
    client.run()
```

### 6.3 モッククライアントの実行

```bash
# 依存関係をインストール
pip install requests websocket-client

# モッククライアントを実行
python mock_vehicle_client.py

# 期待される出力:
# ✅ VH-001としてログイン
# ✅ 2025-12-19T10:30:45Zにテレメトリー送信
# ✅ WebSocket接続完了
# ✅ 2025-12-19T10:30:46Zにテレメトリー送信
# 📨 コマンド受信: mission.dispatch
#   ミッションID: 12345
#   ウェイポイント: 3
```

---

## 7. 統合テスト

### 7.1 テストシナリオ1: 車両が接続してテレメトリーを送信

**ステップ:**
1. TVMサーバーを起動: `uvicorn main:app --reload`
2. モック車両を起動: `python mock_vehicle_client.py`
3. サーバーログで検証: 認証成功、テレメトリー受信
4. データベースをチェック: `SELECT * FROM vehicle_telemetry ORDER BY timestamp DESC LIMIT 10;`

**期待される結果:**
- 車両が正常に認証された
- テレメトリーが1秒ごとにデータベースに挿入された
- WebSocket接続が確立された

### 7.2 テストシナリオ2: 車両にミッションを配信

**ステップ:**
1. モック車両が実行され接続されている
2. Postmanまたはダッシュボードでミッションを作成:
   ```
   POST /api/v1/missions
   Authorization: Bearer <user_token>
   {
     "vehicle_id": "VH-001",
     "waypoints": [
       {"x": 10.0, "y": 5.0, "heading": 0.0, "action": "navigate"},
       {"x": 15.0, "y": 8.0, "heading": 90.0, "action": "dock"}
     ]
   }
   ```
3. モック車両コンソールでコマンド受信を確認
4. データベースでミッションステータスを確認

**期待される結果:**
- ミッションがデータベースに作成された
- WebSocketコマンドが車両に送信された
- モック車両がミッションを受信して承認した

### 7.3 テストシナリオ3: 緊急停止

**ステップ:**
1. モック車両が実行中
2. ダッシュボードまたはAPI経由で緊急停止を送信:
   ```
   POST /api/v1/vehicles/VH-001/emergency-stop
   ```
3. モック車両コンソールを確認

**期待される結果:**
- 緊急停止コマンドがWebSocket経由で送信された
- モック車両が`emergency.stop`メッセージを受信した

---

## 8. トラブルシューティング

### 8.1 よくある問題

**問題: 車両ログインが401で失敗**
```
考えられる原因:
- データベースのapi_keyが一致しない
- vehicle_idがデータベースに見つからない
- JWTシークレットキーの不一致

解決策:
1. データベースをチェック: SELECT * FROM vehicles WHERE vehicle_id = 'VH-001';
2. api_keyが一致することを確認
3. サーバーログで詳細なエラーを確認
```

**問題: WebSocket接続が失敗**
```
考えられる原因:
- CORSが設定されていない
- サーバーでWebSocketが有効になっていない
- 認証が失敗した

解決策:
1. FastAPIでCORSを有効化:
   from fastapi.middleware.cors import CORSMiddleware
   app.add_middleware(CORSMiddleware, allow_origins=["*"])
2. WebSocketエンドポイントが登録されているか確認
3. JWTトークンが有効か確認
```

**問題: テレメトリーがデータベースに表示されない**
```
考えられる原因:
- データベース接続が失敗した
- テーブルが存在しない
- SQLエラー

解決策:
1. サーバーログでSQLエラーを確認
2. テーブルが存在することを確認: psqlで\dt
3. データベース接続を手動でテスト
```

---

## 9. 成果物チェックリスト

### 9.1 15週目の成果物（パンカジとの統合テスト）

- [ ] すべてのRESTエンドポイントが実装され、テストされている
- [ ] すべてのWebSocketコマンドが実装されている
- [ ] テストデータを含む完全なデータベーススキーマ
- [ ] モック車両クライアントが接続して動作可能
- [ ] 車両ステータスを表示する基本的なフリートダッシュボード
- [ ] 認証が動作している（車両+ユーザー）
- [ ] APIドキュメント（Swagger/OpenAPI）
- [ ] サーバーがデプロイされ、ネットワークからアクセス可能

### 9.2 統合テスト要件

- [ ] パンカジが実際の車両をあなたのサーバーに接続できる
- [ ] 実際の車両が正常に認証できる
- [ ] 実際の車両のテレメトリーがあなたのデータベースに表示される
- [ ] ダッシュボードが実際の車両の位置/ステータスを表示する
- [ ] 実際の車両にミッションを配信できる
- [ ] 実際の車両がミッションを受信して実行する
- [ ] ミッションステータスの更新がダッシュボードに表示される

---

## 10. 連絡先とサポート

**API仕様に関する質問:**
- 連絡先: パンカジ（車両ソフトウェアチーム）
- 参照: TVM_API_SPECIFICATION.md、TVM_DATA_MODELS.md

**技術選択について（バックエンド/データベース/フロントエンド）:**
- あなたの決定 - チームが最もよく知っているものを選択してください
- 推奨: Python + FastAPI + PostgreSQL + React

**統合テストスケジュール:**
- 14-15週目: パンカジとの共同テスト
- Slack/メールでスケジュールを調整

---

## まとめ

**このガイドが提供するもの:**
- ✅ ステップバイステップの実装手順
- ✅ コード例（Python + FastAPI）
- ✅ データベーススキーマ（PostgreSQL）
- ✅ テスト用モック車両クライアント
- ✅ 統合テストシナリオ
- ✅ トラブルシューティングガイド

**次のステップ:**
1. 技術スタックを選択
2. プロジェクト構造をセットアップ
3. 認証を実装（1-2週目）
4. RESTエンドポイントを実装（3-8週目）
5. WebSocketを実装（9-11週目）
6. モック車両でテスト
7. パンカジの実際の車両と統合（14-15週目）

**成功基準:**
- モック車両クライアントが接続して動作できる
- すべてのAPIエンドポイントが正しい応答を返す
- データベースがテレメトリーとミッションを保存する
- ダッシュボードが車両ステータスを表示する
- 実際の車両との統合の準備ができている

---

**文書ステータス:** ✅ 完成
**バージョン:** 1.0
**日付:** 2025年12月19日
**作成者:** パンカジ

**実装頑張ってください！ 🚀**
