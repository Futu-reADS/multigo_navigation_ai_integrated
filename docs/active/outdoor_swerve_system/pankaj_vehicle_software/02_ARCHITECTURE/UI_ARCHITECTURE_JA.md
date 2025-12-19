# UIアーキテクチャ（タッチスクリーンインターフェース）

**文書ID:** ARCH-UI-001
**バージョン:** 1.0
**日付:** 2025年12月15日
**ステータス:** ドラフト

---

## 1. 概要

**UIサブシステム**は、7-10インチタッチスクリーンを介して乗客とオペレーターの主要なヒューマンマシンインターフェースを提供します。ミッション制御、ステータス監視、緊急操作を可能にします。

**主要機能:**
- ミッションステータス表示（バッテリー、位置、進捗、ETA）
- ウェイポイントリストからの目的地選択
- 緊急停止ボタン（ソフトウェアトリガー）
- リアルタイム診断とエラーメッセージ
- 手動ローカライゼーション初期化
- 多言語サポート（EN、JP、ZH）

**技術スタック:**
- **フロントエンド:** React 18 + Next.js 14（SSR/SSG）
- **言語:** TypeScript（型安全性）
- **ROS 2ブリッジ:** rosbridge_suite（WebSocket）
- **デプロイ:** Chromiumキオスクモード（全画面、ブラウザUIなし）

---

## 2. システムアーキテクチャ

### 2.1 コンポーネント図

```
┌──────────────────────────────────────────────────────────────────┐
│                    タッチスクリーンUIサブシステム                  │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │               Reactアプリ（Next.js 14）                    │ │
│  │                                                            │ │
│  │  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐│ │
│  │  │ ミッション   │    │ 目的地       │    │ 緊急         ││ │
│  │  │ ステータス   │    │ 選択         │    │ 停止         ││ │
│  │  │ コンポーネント│    │ コンポーネント│    │ コンポーネント││ │
│  │  │              │    │              │    │              ││ │
│  │  │ - バッテリー │    │ - ウェイポイント│   │ - E-Stopボタン││ │
│  │  │ - 位置       │    │   リスト     │    │ - キャンセルボタン││ │
│  │  │ - 進捗       │    │ - マップビュー│   │              ││ │
│  │  │ - ETA        │    │              │    │              ││ │
│  │  └──────────────┘    └──────────────┘    └──────────────┘│ │
│  │                                                            │ │
│  │  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐│ │
│  │  │ 診断         │    │ローカライゼーション│    │ 設定         ││ │
│  │  │ コンポーネント│    │ 初期化       │    │ コンポーネント││ │
│  │  │              │    │ コンポーネント│    │              ││ │
│  │  │ - エラー     │    │ - 2D姿勢     │    │ - 言語       ││ │
│  │  │   メッセージ │    │   推定       │    │ - 明るさ     ││ │
│  │  │ - サブシステム│   │ - 手動       │    │ - 音量       ││ │
│  │  │   ヘルス     │    │   姿勢入力   │    │              ││ │
│  │  └──────────────┘    └──────────────┘    └──────────────┘│ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                  状態管理                                  │ │
│  │                  (React Context)                           │ │
│  │                                                            │ │
│  │  - ロボットステータス（バッテリー、姿勢、速度）            │ │
│  │  - ミッションステータス（現在のウェイポイント、ETA、距離） │ │
│  │  - 安全ステータス（E-Stop、診断）                         │ │
│  │  - ユーザー設定（言語、テーマ）                            │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│                             ↕ WebSocket                          │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                  rosbridge_suite                           │ │
│  │                  (WebSocketサーバー)                       │ │
│  │                                                            │ │
│  │  - トピック購読（ロボットステータス、診断）                │ │
│  │  - サービス呼び出し（ナビゲーションゴール、E-Stop）       │ │
│  │  - アクションクライアント（ルート実行）                    │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│                             ↕ ROS 2                              │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                  ROS 2ナビゲーションスタック               │ │
│  │                                                            │ │
│  │  - /battery_statusトピック                                │ │
│  │  - /ndt_poseトピック                                      │ │
│  │  - /route_statusトピック                                  │ │
│  │  - /emergency_stopトピック                                │ │
│  │  - /execute_routeアクション                               │ │
│  └────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────┘
```

---

## 3. 技術スタック

### 3.1 フロントエンドフレームワーク

**React 18 + Next.js 14:**
- **SSR/SSG:** 高速初期ロードのためのサーバーサイドレンダリング
- **ファイルベースルーティング:** シンプルなページ組織
- **APIルート:** ROSブリッジ統合用バックエンド
- **TypeScript:** ロバスト性のための型安全性

**パッケージ構造:**
```
ui/
├── package.json
├── tsconfig.json
├── next.config.js
├── src/
│   ├── app/                      # Next.js 14 App Router
│   │   ├── layout.tsx            # ルートレイアウト
│   │   ├── page.tsx              # ホームページ（ミッションステータス）
│   │   ├── destination/          # 目的地選択
│   │   ├── diagnostics/          # 診断ビュー
│   │   └── settings/             # 設定ビュー
│   ├── components/               # Reactコンポーネント
│   │   ├── MissionStatus.tsx
│   │   ├── DestinationList.tsx
│   │   ├── EmergencyStop.tsx
│   │   ├── DiagnosticsPanel.tsx
│   │   └── LocalizationInit.tsx
│   ├── hooks/                    # カスタムReactフック
│   │   ├── useROS.ts             # rosbridge WebSocketフック
│   │   ├── useRobotStatus.ts
│   │   └── useMissionStatus.ts
│   ├── context/                  # React Contextプロバイダー
│   │   ├── ROSContext.tsx
│   │   └── LanguageContext.tsx
│   ├── types/                    # TypeScript型定義
│   │   ├── ros_msgs.ts           # ROSメッセージ型
│   │   └── app_state.ts
│   └── utils/                    # ユーティリティ関数
│       ├── rosbridge.ts          # rosbridgeクライアント
│       └── i18n.ts               # 国際化
├── public/                       # 静的アセット
│   ├── icons/
│   └── locales/                  # 翻訳ファイル
│       ├── en.json
│       ├── ja.json
│       └── zh.json
└── styles/                       # CSS/SCSSスタイル
    └── globals.css
```

---

### 3.2 ROS 2ブリッジ（rosbridge_suite）

**rosbridge_server:**
- **プロトコル:** WebSocket（ws://localhost:9090）
- **メッセージフォーマット:** JSON
- **機能:** トピックpub/sub、サービス呼び出し、アクションクライアント

**インストール:**
```bash
sudo apt install ros-humble-rosbridge-suite
```

**起動:**
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**設定:**
```yaml
# rosbridge_websocket.launch.xml
<launch>
  <arg name="port" default="9090"/>
  <arg name="address" default=""/>
  <arg name="ssl" default="false"/>
  <arg name="certfile" default=""/>
  <arg name="keyfile" default=""/>
  
  <node pkg="rosbridge_server" exec="rosbridge_websocket" name="rosbridge_websocket">
    <param name="port" value="$(var port)"/>
    <param name="address" value="$(var address)"/>
    <param name="ssl" value="$(var ssl)"/>
    <param name="certfile" value="$(var certfile)"/>
    <param name="keyfile" value="$(var keyfile)"/>
  </node>
</launch>
```

---

## 4. Reactコンポーネント

### 4.1 ミッションステータスコンポーネント

**MissionStatus.tsx:**
```tsx
import React from 'react';
import { useRobotStatus, useMissionStatus } from '@/hooks';

export const MissionStatus: React.FC = () => {
  const robotStatus = useRobotStatus();
  const missionStatus = useMissionStatus();

  return (
    <div className="mission-status">
      {/* バッテリーレベル */}
      <div className="status-item">
        <BatteryIcon level={robotStatus.battery_percentage} />
        <span>{robotStatus.battery_percentage}%</span>
        <span>{robotStatus.battery_voltage.toFixed(1)}V</span>
      </div>

      {/* 現在地 */}
      <div className="status-item">
        <LocationIcon />
        <span>{robotStatus.current_location}</span>
      </div>

      {/* ミッション進捗 */}
      {missionStatus.active && (
        <>
          <div className="status-item">
            <ProgressBar 
              current={missionStatus.waypoints_completed} 
              total={missionStatus.waypoints_total} 
            />
            <span>
              {missionStatus.waypoints_completed} / {missionStatus.waypoints_total} 停止
            </span>
          </div>

          {/* ETA */}
          <div className="status-item">
            <ClockIcon />
            <span>ETA: {formatETA(missionStatus.eta_seconds)}</span>
            <span>{missionStatus.distance_remaining.toFixed(0)}m 残り</span>
          </div>
        </>
      )}
    </div>
  );
};
```

---

### 4.2 目的地選択コンポーネント

**DestinationList.tsx:**
```tsx
import React, { useState } from 'react';
import { useROS } from '@/hooks';
import type { Waypoint } from '@/types/ros_msgs';

export const DestinationList: React.FC = () => {
  const ros = useROS();
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [selectedWaypoint, setSelectedWaypoint] = useState<string | null>(null);

  // ROSパラメータサーバーからウェイポイント読み込み
  React.useEffect(() => {
    ros.getParam('/waypoints', (waypointData) => {
      setWaypoints(waypointData);
    });
  }, [ros]);

  const handleNavigate = () => {
    if (!selectedWaypoint) return;

    // /execute_routeアクション呼び出し
    const goal = {
      waypoint_ids: [selectedWaypoint],
    };

    ros.callAction('/execute_route', 'custom_msgs/ExecuteRoute', goal, {
      onResult: (result) => {
        if (result.success) {
          alert(`ナビゲーション完了: ${result.waypoints_completed} 停止`);
        } else {
          alert(`ナビゲーション失敗: ${result.message}`);
        }
      },
      onFeedback: (feedback) => {
        console.log(`現在のウェイポイント: ${feedback.current_waypoint_id}`);
        console.log(`残り距離: ${feedback.distance_remaining}m`);
      },
    });
  };

  return (
    <div className="destination-list">
      <h2>目的地選択</h2>
      <ul>
        {waypoints.map((wp) => (
          <li 
            key={wp.id}
            onClick={() => setSelectedWaypoint(wp.id)}
            className={selectedWaypoint === wp.id ? 'selected' : ''}
          >
            <span>{wp.name}</span>
            <span>{wp.type}</span>
          </li>
        ))}
      </ul>
      <button 
        onClick={handleNavigate}
        disabled={!selectedWaypoint}
        className="navigate-btn"
      >
        ナビゲーション開始
      </button>
    </div>
  );
};
```

---

### 4.3 緊急停止コンポーネント

**EmergencyStop.tsx:**
```tsx
import React, { useState } from 'react';
import { useROS } from '@/hooks';

export const EmergencyStop: React.FC = () => {
  const ros = useROS();
  const [estopActive, setEstopActive] = useState(false);

  React.useEffect(() => {
    // /estop_status購読
    ros.subscribe('/estop_status', 'custom_msgs/EStopStatus', (msg) => {
      setEstopActive(msg.active);
    });
  }, [ros]);

  const handleEmergencyStop = () => {
    // /emergency_stopトピックに配信
    ros.publish('/emergency_stop', 'std_msgs/Bool', { data: true });
    setEstopActive(true);
  };

  const handleReset = () => {
    // /reset_estopサービス呼び出し（手動検証必要）
    ros.callService('/reset_estop', 'std_srvs/Trigger', {}, (response) => {
      if (response.success) {
        setEstopActive(false);
      } else {
        alert(`E-Stopリセット不可: ${response.message}`);
      }
    });
  };

  return (
    <div className="emergency-stop">
      {estopActive ? (
        <>
          <div className="estop-active">
            <WarningIcon size={64} color="red" />
            <h2>緊急停止作動中</h2>
            <p>ロボットは停止中。リセット前に安全条件を確認してください。</p>
          </div>
          <button onClick={handleReset} className="reset-btn">
            E-Stopリセット
          </button>
        </>
      ) : (
        <button onClick={handleEmergencyStop} className="estop-btn">
          <StopIcon size={48} />
          緊急停止
        </button>
      )}
    </div>
  );
};
```

---

## 5. 状態管理

### 5.1 ROSコンテキストプロバイダー

**ROSContext.tsx:**
```tsx
import React, { createContext, useContext, useEffect, useState } from 'react';
import ROSLIB from 'roslib';

interface ROSContextType {
  ros: ROSLIB.Ros | null;
  connected: boolean;
}

const ROSContext = createContext<ROSContextType>({ ros: null, connected: false });

export const ROSProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [ros, setRos] = useState<ROSLIB.Ros | null>(null);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({
      url: 'ws://localhost:9090',  // rosbridge WebSocket
    });

    rosInstance.on('connection', () => {
      console.log('ROSブリッジに接続');
      setConnected(true);
    });

    rosInstance.on('error', (error) => {
      console.error('ROSブリッジエラー:', error);
      setConnected(false);
    });

    rosInstance.on('close', () => {
      console.log('ROSブリッジ接続クローズ');
      setConnected(false);
    });

    setRos(rosInstance);

    return () => {
      rosInstance.close();
    };
  }, []);

  return (
    <ROSContext.Provider value={{ ros, connected }}>
      {children}
    </ROSContext.Provider>
  );
};

export const useROS = () => {
  const context = useContext(ROSContext);
  if (!context.ros) {
    throw new Error('useROSはROSProvider内で使用する必要があります');
  }
  return context.ros;
};
```

---

## 6. デプロイ

### 6.1 キオスクモード（Chromium）

**キオスクセットアップ:**
```bash
# Chromiumインストール
sudo apt install chromium-browser

# キオスク起動スクリプト作成
cat > ~/start_ui_kiosk.sh << 'SCRIPT'
#!/bin/bash
chromium-browser \
  --kiosk \
  --noerrdialogs \
  --disable-infobars \
  --no-first-run \
  --disable-session-crashed-bubble \
  --disable-features=TranslateUI \
  --start-fullscreen \
  http://localhost:3000
SCRIPT

chmod +x ~/start_ui_kiosk.sh

# 起動時自動開始（systemdサービス）
sudo tee /etc/systemd/system/ui-kiosk.service << 'SERVICE'
[Unit]
Description=タッチスクリーンUIキオスク
After=network.target

[Service]
User=robot
Environment=DISPLAY=:0
ExecStart=/home/robot/start_ui_kiosk.sh
Restart=always

[Install]
WantedBy=multi-user.target
SERVICE

sudo systemctl enable ui-kiosk.service
sudo systemctl start ui-kiosk.service
```

---

### 6.2 Next.js本番ビルド

**ビルドとデプロイ:**
```bash
# 開発モード
npm run dev

# 本番ビルド
npm run build
npm run start

# またはスタンドアロン出力使用（小さい）
# next.config.js:
module.exports = {
  output: 'standalone',
};
```

**Systemdサービス（Next.js）:**
```bash
sudo tee /etc/systemd/system/ui-server.service << 'SERVICE'
[Unit]
Description=タッチスクリーンUIサーバー
After=network.target rosbridge.service

[Service]
User=robot
WorkingDirectory=/home/robot/ui
ExecStart=/usr/bin/npm start
Restart=always

[Install]
WantedBy=multi-user.target
SERVICE

sudo systemctl enable ui-server.service
sudo systemctl start ui-server.service
```

---

## 7. 国際化（i18n）

### 7.1 翻訳ファイル

**public/locales/en.json:**
```json
{
  "mission_status": "Mission Status",
  "battery": "Battery",
  "current_location": "Current Location",
  "eta": "Estimated Time of Arrival",
  "select_destination": "Select Destination",
  "emergency_stop": "Emergency Stop",
  "diagnostics": "Diagnostics",
  "settings": "Settings",
  "language": "Language",
  "start_navigation": "Start Navigation"
}
```

**public/locales/ja.json:**
```json
{
  "mission_status": "ミッション状態",
  "battery": "バッテリー",
  "current_location": "現在地",
  "eta": "到着予定時刻",
  "select_destination": "目的地選択",
  "emergency_stop": "緊急停止",
  "diagnostics": "診断",
  "settings": "設定",
  "language": "言語",
  "start_navigation": "ナビゲーション開始"
}
```

---

## 8. テスト戦略

### 8.1 ユニットテスト（Jest + React Testing Library）
- コンポーネントレンダリング
- ユーザーインタラクション（ボタンクリック、フォーム入力）
- 状態管理

### 8.2 統合テスト
- rosbridge接続
- トピック購読
- サービス呼び出し
- アクションクライアント

### 8.3 ユーザー受入テスト
- タッチターゲットサイズ（≥44×44ピクセル）
- 日光下での可読性（≥400ニット画面）
- 多言語切り替え
- 緊急停止応答性（<500ms）

---

## 9. 性能目標

| メトリック | 目標 | 検証 |
|--------|--------|------------|
| 初期ロード時間 | <2秒 | Lighthouse |
| UI応答性 | <100ms | ユーザーテスト |
| WebSocket遅延 | <50ms | ネットワーク監視 |
| タッチターゲットサイズ | ≥44×44ピクセル | UI検査 |
| 画面明るさ | ≥400ニット | ハードウェア仕様 |

---

## 10. 実装フェーズ

### フェーズ1: 基本UI（スプリント1-2）
- ✅ React + Next.jsセットアップ
- ✅ rosbridge統合
- ✅ ミッションステータス表示
- ✅ 緊急停止ボタン

### フェーズ2: ナビゲーションUI（スプリント3-4）
- ✅ 目的地選択
- ✅ ウェイポイントリスト
- ✅ ナビゲーション進捗表示

### フェーズ3: 高度な機能（スプリント5-6）
- ✅ 診断パネル
- ✅ ローカライゼーション初期化
- ✅ 設定ページ
- ✅ 多言語サポート

### フェーズ4: デプロイ（スプリント7-8）
- ✅ キオスクモードセットアップ
- ✅ 本番ビルド
- ✅ Systemd統合
- ✅ ユーザー受入テスト

---

**文書ステータス:** ドラフト
**実装ステータス:** 未開始
**承認必要:** UI/UXリード、システムアーキテクト
