# UIコンポーネント詳細設計

**ドキュメントID:** DESIGN-UI-001
**バージョン:** 1.0
**日付:** 2025-12-15
**ステータス:** ドラフト

---

## 1. 概要

本ドキュメントは、ROS 2通信のためのrosbridge WebSocket統合を備えたReact 18 + Next.js 14アプリケーションを実装するタッチスクリーンUIの詳細設計仕様を提供します。

**技術スタック:**
- React 18 + Next.js 14 + TypeScript
- rosbridge-client (WebSocket ROS 2統合)
- TailwindCSS (スタイリング)
- Zustand (状態管理)
- i18next (多言語: EN, JP, ZH)

**デプロイ:** Ubuntu 22.04上のChromiumキオスクモード

---

## 2. コンポーネントアーキテクチャ

```
┌────────────────────────────────────────────────────────────┐
│                    UIアプリケーション                       │
│                 (Next.js 14 アプリ)                        │
├────────────────────────────────────────────────────────────┤
│                                                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │
│  │   ミッション │  │   緊急       │  │   システム   │   │
│  │   ステータス │  │   停止       │  │   ステータス │   │
│  └──────────────┘  └──────────────┘  └──────────────┘   │
│                                                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │
│  │   ルート     │  │   マップ     │  │   言語       │   │
│  │   進捗       │  │   ビューア   │  │   選択       │   │
│  └──────────────┘  └──────────────┘  └──────────────┘   │
│                                                            │
├────────────────────────────────────────────────────────────┤
│                  ROS Bridgeレイヤー                        │
│           (WebSocket: ws://localhost:9090)                 │
└────────────────────────────────────────────────────────────┘
         │
         ↓
    ┌────────────┐
    │  rosbridge │
    │  サーバー  │
    └────────────┘
```

---

## 3. 主要コンポーネント

### MissionStatusコンポーネント

```typescript
// components/MissionStatus.tsx
import { useRobotStatus } from '@/hooks/useRobotStatus';
import { useMissionStatus } from '@/hooks/useMissionStatus';

export const MissionStatus: React.FC = () => {
  const robotStatus = useRobotStatus();
  const missionStatus = useMissionStatus();

  return (
    <div className="mission-status">
      <h2>ミッションステータス</h2>

      <div className="status-grid">
        <StatusCard
          label="バッテリー"
          value={`${robotStatus.battery_percent}%`}
          status={getBatteryStatus(robotStatus.battery_percent)}
        />

        <StatusCard
          label="位置"
          value={missionStatus.current_waypoint}
          status="info"
        />

        <StatusCard
          label="進捗"
          value={`${missionStatus.waypoints_completed}/${missionStatus.total_waypoints}`}
          status="info"
        />

        <StatusCard
          label="到着予定"
          value={formatETA(missionStatus.estimated_arrival)}
          status="info"
        />
      </div>
    </div>
  );
};
```

---

### EmergencyStopコンポーネント

```typescript
// components/EmergencyStop.tsx
import { useROS } from '@/hooks/useROS';
import { useState } from 'react';

export const EmergencyStop: React.FC = () => {
  const ros = useROS();
  const [isPressed, setIsPressed] = useState(false);

  const handleEmergencyStop = () => {
    // /emergency_stopトピックにパブリッシュ
    ros.publish('/emergency_stop', 'std_msgs/Bool', { data: true });
    setIsPressed(true);
  };

  const handleReset = () => {
    ros.publish('/emergency_stop', 'std_msgs/Bool', { data: false });
    setIsPressed(false);
  };

  return (
    <div className="emergency-stop">
      {!isPressed ? (
        <button
          className="estop-button"
          onClick={handleEmergencyStop}
        >
          <span className="estop-icon">⚠️</span>
          <span className="estop-text">緊急停止</span>
        </button>
      ) : (
        <div className="estop-active">
          <p>緊急停止作動中</p>
          <button onClick={handleReset}>リセット</button>
        </div>
      )}
    </div>
  );
};
```

---

### ROS Bridgeフック

```typescript
// hooks/useROS.ts
import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { useROSStore } from '@/store/rosStore';

export const useROS = () => {
  const { ros, setROS, setConnected } = useROSStore();

  useEffect(() => {
    if (!ros) {
      const newROS = new ROSLIB.Ros({
        url: 'ws://localhost:9090'
      });

      newROS.on('connection', () => {
        console.log('rosbridgeに接続しました');
        setConnected(true);
      });

      newROS.on('error', (error) => {
        console.error('rosbridgeエラー:', error);
        setConnected(false);
      });

      newROS.on('close', () => {
        console.log('rosbridgeから切断しました');
        setConnected(false);
      });

      setROS(newROS);
    }
  }, [ros, setROS, setConnected]);

  const publish = (topic: string, messageType: string, message: any) => {
    if (!ros) return;

    const rosTopic = new ROSLIB.Topic({
      ros: ros,
      name: topic,
      messageType: messageType
    });

    const rosMessage = new ROSLIB.Message(message);
    rosTopic.publish(rosMessage);
  };

  const subscribe = (
    topic: string,
    messageType: string,
    callback: (message: any) => void
  ) => {
    if (!ros) return;

    const rosTopic = new ROSLIB.Topic({
      ros: ros,
      name: topic,
      messageType: messageType
    });

    rosTopic.subscribe(callback);

    return () => rosTopic.unsubscribe();
  };

  return { ros, publish, subscribe };
};
```

---

### Robot Statusフック

```typescript
// hooks/useRobotStatus.ts
import { useEffect, useState } from 'react';
import { useROS } from './useROS';

interface RobotStatus {
  battery_percent: number;
  battery_voltage: number;
  localization_quality: number;
  safety_state: string;
}

export const useRobotStatus = () => {
  const { subscribe } = useROS();
  const [status, setStatus] = useState<RobotStatus>({
    battery_percent: 0,
    battery_voltage: 0,
    localization_quality: 0,
    safety_state: 'UNKNOWN'
  });

  useEffect(() => {
    const unsubscribe = subscribe(
      '/robot_status',
      'custom_msgs/RobotStatus',
      (message: any) => {
        setStatus({
          battery_percent: message.battery_percent,
          battery_voltage: message.battery_voltage,
          localization_quality: message.localization_quality,
          safety_state: message.safety_state
        });
      }
    );

    return unsubscribe;
  }, [subscribe]);

  return status;
};
```

---

## 4. 状態管理

**Zustandストア:**
```typescript
// store/rosStore.ts
import create from 'zustand';
import ROSLIB from 'roslib';

interface ROSStore {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  setROS: (ros: ROSLIB.Ros) => void;
  setConnected: (connected: boolean) => void;
}

export const useROSStore = create<ROSStore>((set) => ({
  ros: null,
  connected: false,
  setROS: (ros) => set({ ros }),
  setConnected: (connected) => set({ connected })
}));
```

---

## 5. 多言語サポート

**i18n設定:**
```typescript
// i18n/config.ts
import i18n from 'i18next';
import { initReactI18next } from 'react-i18next';

const resources = {
  en: {
    translation: {
      "mission_status": "Mission Status",
      "emergency_stop": "Emergency Stop",
      "battery": "Battery",
      "location": "Location",
      "eta": "ETA"
    }
  },
  ja: {
    translation: {
      "mission_status": "ミッションステータス",
      "emergency_stop": "緊急停止",
      "battery": "バッテリー",
      "location": "位置",
      "eta": "到着予定時刻"
    }
  },
  zh: {
    translation: {
      "mission_status": "任务状态",
      "emergency_stop": "紧急停止",
      "battery": "电池",
      "location": "位置",
      "eta": "预计到达时间"
    }
  }
};

i18n
  .use(initReactI18next)
  .init({
    resources,
    lng: 'ja',  // デフォルト日本語
    fallbackLng: 'en',
    interpolation: {
      escapeValue: false
    }
  });

export default i18n;
```

---

## 6. キオスクモードデプロイ

**Chromiumキオスクスクリプト:**
```bash
#!/bin/bash
# launch-ui.sh

# rosbridgeを起動
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

# rosbridgeを待つ
sleep 5

# Next.js UIを起動 (本番ビルド)
cd /opt/wheelchair_robot/ui
npm run start &

# UIサーバーを待つ
sleep 10

# Chromiumをキオスクモードで起動
chromium-browser \
  --kiosk \
  --no-first-run \
  --disable-infobars \
  --disable-session-crashed-bubble \
  --disable-translate \
  --touch-events=enabled \
  http://localhost:3000
```

---

## 7. テスト

**コンポーネントテスト例:**
```typescript
// __tests__/MissionStatus.test.tsx
import { render, screen } from '@testing-library/react';
import { MissionStatus } from '@/components/MissionStatus';

jest.mock('@/hooks/useRobotStatus', () => ({
  useRobotStatus: () => ({
    battery_percent: 85,
    battery_voltage: 48.5,
    localization_quality: 0.95,
    safety_state: 'NORMAL'
  })
}));

test('バッテリー残量を含むミッションステータスをレンダリング', () => {
  render(<MissionStatus />);

  expect(screen.getByText('85%')).toBeInTheDocument();
});
```

---

**ドキュメントステータス:** ドラフト
**実装ステータス:** 開発準備完了
**必要な承認:** UIリード、ROS統合リード
