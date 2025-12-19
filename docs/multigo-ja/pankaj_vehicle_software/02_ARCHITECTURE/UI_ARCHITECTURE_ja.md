# UIアーキテクチャ（タッチスクリーンインターフェース）

**文書ID:** ARCH-UI-001
**バージョン:** 1.0
**日付:** 2025年12月15日
**ステータス:** ドラフト

---

## 1. 概要

**UIサブシステム**は、7〜10インチのタッチスクリーンを介して、乗客とオペレーターのための主要な人間-機械インターフェースを提供します。ミッション制御、ステータス監視、緊急操作を可能にします。

**主要機能:**
- ミッションステータス表示（バッテリー、位置、進捗、到着予定時刻）
- ウェイポイントリストから目的地選択
- 緊急停止ボタン（ソフトウェアトリガー）
- リアルタイム診断とエラーメッセージ
- 手動ローカライゼーション初期化
- 多言語サポート（EN、JP、ZH）

**技術スタック:**
- **フロントエンド:** React 18 + Next.js 14（SSR/SSG）
- **言語:** TypeScript（型安全性）
- **ROS 2ブリッジ:** rosbridge_suite（WebSocket）
- **デプロイメント:** Chromiumキオスクモード（フルスクリーン、ブラウザUIなし）

---

## 2. システムアーキテクチャ

### 2.1 コンポーネント図

```
┌──────────────────────────────────────────────────────────────────┐
│                    Touch Screen UI Subsystem                     │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │               React App (Next.js 14)                       │ │
│  │                                                            │ │
│  │  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐│ │
│  │  │ Mission      │    │ Destination  │    │ Emergency    ││ │
│  │  │ Status       │    │ Selection    │    │ Stop         ││ │
│  │  │ Component    │    │ Component    │    │ Component    ││ │
│  │  │              │    │              │    │              ││ │
│  │  │ - Battery    │    │ - Waypoint   │    │ - E-Stop btn ││ │
│  │  │ - Location   │    │   list       │    │ - Cancel btn ││ │
│  │  │ - Progress   │    │ - Map view   │    │              ││ │
│  │  │ - ETA        │    │              │    │              ││ │
│  │  └──────────────┘    └──────────────┘    └──────────────┘│ │
│  │                                                            │ │
│  │  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐│ │
│  │  │ Diagnostics  │    │ Localization │    │ Settings     ││ │
│  │  │ Component    │    │ Init         │    │ Component    ││ │
│  │  │              │    │ Component    │    │              ││ │
│  │  │ - Error      │    │ - 2D Pose    │    │ - Language   ││ │
│  │  │   messages   │    │   Estimate   │    │ - Brightness ││ │
│  │  │ - Subsystem  │    │ - Manual     │    │ - Volume     ││ │
│  │  │   health     │    │   pose input │    │              ││ │
│  │  └──────────────┘    └──────────────┘    └──────────────┘│ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                  State Management                          │ │
│  │                  (React Context)                           │ │
│  │                                                            │ │
│  │  - Robot status (battery, pose, velocity)                 │ │
│  │  - Mission status (current waypoint, ETA, distance)       │ │
│  │  - Safety status (E-Stop, diagnostics)                    │ │
│  │  - User preferences (language, theme)                     │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│                             ↕ WebSocket                          │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                  rosbridge_suite                           │ │
│  │                  (WebSocket Server)                        │ │
│  │                                                            │ │
│  │  - Topic subscription (robot status, diagnostics)         │ │
│  │  - Service calls (navigation goals, E-Stop)               │ │
│  │  - Action clients (route execution)                       │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│                             ↕ ROS 2                              │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                  ROS 2 Navigation Stack                    │ │
│  │                                                            │ │
│  │  - /battery_status topic                                  │ │
│  │  - /ndt_pose topic                                        │ │
│  │  - /route_status topic                                    │ │
│  │  - /emergency_stop topic                                  │ │
│  │  - /execute_route action                                  │ │
│  └────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────┘
```

---

## 3. 技術スタック

### 3.1 フロントエンドフレームワーク

**React 18 + Next.js 14:**
- **SSR/SSG:** 高速な初期ロードのためのサーバーサイドレンダリング
- **ファイルベースルーティング:** シンプルなページ構成
- **APIルート:** ROSブリッジ統合のためのバックエンド
- **TypeScript:** 堅牢性のための型安全性

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
- **メッセージ形式:** JSON
- **機能:** トピックのパブリッシュ/サブスクライブ、サービス呼び出し、アクションクライアント

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
      {/* Battery Level */}
      <div className="status-item">
        <BatteryIcon level={robotStatus.battery_percentage} />
        <span>{robotStatus.battery_percentage}%</span>
        <span>{robotStatus.battery_voltage.toFixed(1)}V</span>
      </div>

      {/* Current Location */}
      <div className="status-item">
        <LocationIcon />
        <span>{robotStatus.current_location}</span>
      </div>

      {/* Mission Progress */}
      {missionStatus.active && (
        <>
          <div className="status-item">
            <ProgressBar
              current={missionStatus.waypoints_completed}
              total={missionStatus.waypoints_total}
            />
            <span>
              {missionStatus.waypoints_completed} / {missionStatus.waypoints_total} stops
            </span>
          </div>

          {/* ETA */}
          <div className="status-item">
            <ClockIcon />
            <span>ETA: {formatETA(missionStatus.eta_seconds)}</span>
            <span>{missionStatus.distance_remaining.toFixed(0)}m remaining</span>
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

  // Load waypoints from ROS parameter server
  React.useEffect(() => {
    ros.getParam('/waypoints', (waypointData) => {
      setWaypoints(waypointData);
    });
  }, [ros]);

  const handleNavigate = () => {
    if (!selectedWaypoint) return;

    // Call /execute_route action
    const goal = {
      waypoint_ids: [selectedWaypoint],
    };

    ros.callAction('/execute_route', 'custom_msgs/ExecuteRoute', goal, {
      onResult: (result) => {
        if (result.success) {
          alert(`Navigation complete: ${result.waypoints_completed} stops`);
        } else {
          alert(`Navigation failed: ${result.message}`);
        }
      },
      onFeedback: (feedback) => {
        console.log(`Current waypoint: ${feedback.current_waypoint_id}`);
        console.log(`Distance remaining: ${feedback.distance_remaining}m`);
      },
    });
  };

  return (
    <div className="destination-list">
      <h2>Select Destination</h2>
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
        Start Navigation
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
    // Subscribe to /estop_status
    ros.subscribe('/estop_status', 'custom_msgs/EStopStatus', (msg) => {
      setEstopActive(msg.active);
    });
  }, [ros]);

  const handleEmergencyStop = () => {
    // Publish to /emergency_stop topic
    ros.publish('/emergency_stop', 'std_msgs/Bool', { data: true });
    setEstopActive(true);
  };

  const handleReset = () => {
    // Call /reset_estop service (requires manual verification)
    ros.callService('/reset_estop', 'std_srvs/Trigger', {}, (response) => {
      if (response.success) {
        setEstopActive(false);
      } else {
        alert(`Cannot reset E-Stop: ${response.message}`);
      }
    });
  };

  return (
    <div className="emergency-stop">
      {estopActive ? (
        <>
          <div className="estop-active">
            <WarningIcon size={64} color="red" />
            <h2>EMERGENCY STOP ACTIVE</h2>
            <p>Robot is stopped. Check safety conditions before reset.</p>
          </div>
          <button onClick={handleReset} className="reset-btn">
            Reset E-Stop
          </button>
        </>
      ) : (
        <button onClick={handleEmergencyStop} className="estop-btn">
          <StopIcon size={48} />
          EMERGENCY STOP
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
      console.log('Connected to ROS bridge');
      setConnected(true);
    });

    rosInstance.on('error', (error) => {
      console.error('ROS bridge error:', error);
      setConnected(false);
    });

    rosInstance.on('close', () => {
      console.log('Connection to ROS bridge closed');
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
    throw new Error('useROS must be used within ROSProvider');
  }
  return context.ros;
};
```

---

### 5.2 カスタムフック

**useRobotStatus.ts:**
```tsx
import { useState, useEffect } from 'react';
import { useROS } from './useROS';

interface RobotStatus {
  battery_percentage: number;
  battery_voltage: number;
  current_location: string;
  velocity: { x: number; y: number; theta: number };
}

export const useRobotStatus = (): RobotStatus => {
  const ros = useROS();
  const [status, setStatus] = useState<RobotStatus>({
    battery_percentage: 0,
    battery_voltage: 0,
    current_location: 'Unknown',
    velocity: { x: 0, y: 0, theta: 0 },
  });

  useEffect(() => {
    // Subscribe to /battery_status
    const batteryTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/battery_status',
      messageType: 'sensor_msgs/BatteryState',
    });

    batteryTopic.subscribe((message: any) => {
      setStatus((prev) => ({
        ...prev,
        battery_percentage: message.percentage * 100,
        battery_voltage: message.voltage,
      }));
    });

    // Subscribe to /ndt_pose for location
    const poseTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/ndt_pose',
      messageType: 'geometry_msgs/PoseStamped',
    });

    poseTopic.subscribe((message: any) => {
      // Convert pose to waypoint name (lookup in waypoint database)
      const location = lookupWaypointFromPose(message.pose);
      setStatus((prev) => ({
        ...prev,
        current_location: location,
      }));
    });

    return () => {
      batteryTopic.unsubscribe();
      poseTopic.unsubscribe();
    };
  }, [ros]);

  return status;
};
```

---

## 6. デプロイメント

### 6.1 キオスクモード（Chromium）

**キオスクセットアップ:**
```bash
# Chromiumをインストール
sudo apt install chromium-browser

# キオスク起動スクリプトを作成
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

# ブート時に自動起動（systemdサービス）
sudo tee /etc/systemd/system/ui-kiosk.service << 'SERVICE'
[Unit]
Description=Touch Screen UI Kiosk
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

# またはスタンドアロン出力を使用（より小さい）
# next.config.js:
module.exports = {
  output: 'standalone',
};
```

**Systemdサービス（Next.js）:**
```bash
sudo tee /etc/systemd/system/ui-server.service << 'SERVICE'
[Unit]
Description=Touch Screen UI Server
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

### 7.2 i18nフック

**utils/i18n.ts:**
```tsx
import { useState, useEffect } from 'react';

type Locale = 'en' | 'ja' | 'zh';

const translations: Record<Locale, any> = {};

export const useTranslation = () => {
  const [locale, setLocale] = useState<Locale>('en');
  const [t, setT] = useState<any>({});

  useEffect(() => {
    // Load translation file
    fetch(`/locales/${locale}.json`)
      .then((res) => res.json())
      .then((data) => setT(data));
  }, [locale]);

  return { t, setLocale, locale };
};
```

---

## 8. テスト戦略

### 8.1 ユニットテスト（Jest + React Testing Library）
- コンポーネントレンダリング
- ユーザーインタラクション（ボタンクリック、フォーム入力）
- 状態管理

### 8.2 統合テスト
- rosbridge接続
- トピックサブスクリプション
- サービス呼び出し
- アクションクライアント

### 8.3 ユーザー受け入れテスト
- タッチターゲットサイズ（≥44×44ピクセル）
- 日中の可読性（≥400nitsスクリーン）
- 多言語切り替え
- 緊急停止の応答性（<500ms）

---

## 9. パフォーマンス目標

| 指標 | 目標 | 検証方法 |
|--------|--------|------------|
| 初期ロード時間 | <2秒 | Lighthouse |
| UI応答性 | <100ms | ユーザーテスト |
| WebSocketレイテンシ | <50ms | ネットワーク監視 |
| タッチターゲットサイズ | ≥44×44ピクセル | UI検査 |
| 画面輝度 | ≥400nits | ハードウェア仕様 |

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

### フェーズ4: デプロイメント（スプリント7-8）
- ✅ キオスクモードセットアップ
- ✅ 本番ビルド
- ✅ Systemd統合
- ✅ ユーザー受け入れテスト

---

**文書ステータス:** ドラフト
**実装ステータス:** 未開始
**必要な承認:** UI/UXリード、システムアーキテクト
