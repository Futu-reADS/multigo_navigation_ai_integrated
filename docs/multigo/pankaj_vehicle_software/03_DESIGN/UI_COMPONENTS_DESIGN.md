# UI Components Detailed Design

**Document ID:** DESIGN-UI-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft

---

## 1. Overview

This document provides detailed design specifications for the Touch Screen UI, implementing a React 18 + Next.js 14 application with rosbridge WebSocket integration for ROS 2 communication.

**Tech Stack:**
- React 18 + Next.js 14 + TypeScript
- rosbridge-client (WebSocket ROS 2 integration)
- TailwindCSS (styling)
- Zustand (state management)
- i18next (multi-language: EN, JP, ZH)

**Deployment:** Chromium kiosk mode on Ubuntu 22.04

---

## 2. Component Architecture

```
┌────────────────────────────────────────────────────────────┐
│                    UI Application                          │
│                 (Next.js 14 App)                           │
├────────────────────────────────────────────────────────────┤
│                                                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │
│  │   Mission    │  │   Emergency  │  │   System     │   │
│  │   Status     │  │   Stop       │  │   Status     │   │
│  └──────────────┘  └──────────────┘  └──────────────┘   │
│                                                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │
│  │   Route      │  │   Map        │  │   Language   │   │
│  │   Progress   │  │   Viewer     │  │   Selector   │   │
│  └──────────────┘  └──────────────┘  └──────────────┘   │
│                                                            │
├────────────────────────────────────────────────────────────┤
│                  ROS Bridge Layer                          │
│           (WebSocket: ws://localhost:9090)                 │
└────────────────────────────────────────────────────────────┘
         │
         ↓
    ┌────────────┐
    │  rosbridge │
    │  server    │
    └────────────┘
```

---

## 3. Key Components

### MissionStatus Component

```typescript
// components/MissionStatus.tsx
import { useRobotStatus } from '@/hooks/useRobotStatus';
import { useMissionStatus } from '@/hooks/useMissionStatus';

export const MissionStatus: React.FC = () => {
  const robotStatus = useRobotStatus();
  const missionStatus = useMissionStatus();

  return (
    <div className="mission-status">
      <h2>Mission Status</h2>

      <div className="status-grid">
        <StatusCard
          label="Battery"
          value={`${robotStatus.battery_percent}%`}
          status={getBatteryStatus(robotStatus.battery_percent)}
        />

        <StatusCard
          label="Location"
          value={missionStatus.current_waypoint}
          status="info"
        />

        <StatusCard
          label="Progress"
          value={`${missionStatus.waypoints_completed}/${missionStatus.total_waypoints}`}
          status="info"
        />

        <StatusCard
          label="ETA"
          value={formatETA(missionStatus.estimated_arrival)}
          status="info"
        />
      </div>
    </div>
  );
};
```

---

### Emergency Stop Component

```typescript
// components/EmergencyStop.tsx
import { useROS } from '@/hooks/useROS';
import { useState } from 'react';

export const EmergencyStop: React.FC = () => {
  const ros = useROS();
  const [isPressed, setIsPressed] = useState(false);

  const handleEmergencyStop = () => {
    // Publish to /emergency_stop topic
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
          <span className="estop-text">EMERGENCY STOP</span>
        </button>
      ) : (
        <div className="estop-active">
          <p>Emergency Stop Active</p>
          <button onClick={handleReset}>RESET</button>
        </div>
      )}
    </div>
  );
};
```

---

### ROS Bridge Hook

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
        console.log('Connected to rosbridge');
        setConnected(true);
      });

      newROS.on('error', (error) => {
        console.error('rosbridge error:', error);
        setConnected(false);
      });

      newROS.on('close', () => {
        console.log('Disconnected from rosbridge');
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

### Robot Status Hook

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

## 4. State Management

**Zustand Store:**
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

## 5. Multi-Language Support

**i18n Configuration:**
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
    lng: 'en',
    fallbackLng: 'en',
    interpolation: {
      escapeValue: false
    }
  });

export default i18n;
```

---

## 6. Kiosk Mode Deployment

**Chromium Kiosk Script:**
```bash
#!/bin/bash
# launch-ui.sh

# Start rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

# Wait for rosbridge
sleep 5

# Start Next.js UI (production build)
cd /opt/wheelchair_robot/ui
npm run start &

# Wait for UI server
sleep 10

# Launch Chromium in kiosk mode
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

## 7. Testing

**Component Test Example:**
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

test('renders mission status with battery percentage', () => {
  render(<MissionStatus />);

  expect(screen.getByText('85%')).toBeInTheDocument();
});
```

---

**Document Status:** Draft
**Implementation Status:** Ready for development
**Approvals Required:** UI Lead, ROS Integration Lead
