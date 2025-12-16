# UI Architecture (Touch Screen Interface)

**Document ID:** ARCH-UI-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft

---

## 1. Overview

The **UI Subsystem** provides the primary human-machine interface for passengers and operators via a 7-10 inch touchscreen. It enables mission control, status monitoring, and emergency operations.

**Key Capabilities:**
- Mission status display (battery, location, progress, ETA)
- Destination selection from waypoint list
- Emergency stop button (software trigger)
- Real-time diagnostics and error messages
- Manual localization initialization
- Multi-language support (EN, JP, ZH)

**Technology Stack:**
- **Frontend:** React 18 + Next.js 14 (SSR/SSG)
- **Language:** TypeScript (type safety)
- **ROS 2 Bridge:** rosbridge_suite (WebSocket)
- **Deployment:** Chromium kiosk mode (fullscreen, no browser UI)

---

## 2. System Architecture

### 2.1 Component Diagram

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

## 3. Technology Stack

### 3.1 Frontend Framework

**React 18 + Next.js 14:**
- **SSR/SSG:** Server-side rendering for fast initial load
- **File-based routing:** Simple page organization
- **API routes:** Backend for ROS bridge integration
- **TypeScript:** Type safety for robustness

**Package Structure:**
```
ui/
├── package.json
├── tsconfig.json
├── next.config.js
├── src/
│   ├── app/                      # Next.js 14 App Router
│   │   ├── layout.tsx            # Root layout
│   │   ├── page.tsx              # Home page (mission status)
│   │   ├── destination/          # Destination selection
│   │   ├── diagnostics/          # Diagnostics view
│   │   └── settings/             # Settings view
│   ├── components/               # React components
│   │   ├── MissionStatus.tsx
│   │   ├── DestinationList.tsx
│   │   ├── EmergencyStop.tsx
│   │   ├── DiagnosticsPanel.tsx
│   │   └── LocalizationInit.tsx
│   ├── hooks/                    # Custom React hooks
│   │   ├── useROS.ts             # rosbridge WebSocket hook
│   │   ├── useRobotStatus.ts
│   │   └── useMissionStatus.ts
│   ├── context/                  # React Context providers
│   │   ├── ROSContext.tsx
│   │   └── LanguageContext.tsx
│   ├── types/                    # TypeScript type definitions
│   │   ├── ros_msgs.ts           # ROS message types
│   │   └── app_state.ts
│   └── utils/                    # Utility functions
│       ├── rosbridge.ts          # rosbridge client
│       └── i18n.ts               # Internationalization
├── public/                       # Static assets
│   ├── icons/
│   └── locales/                  # Translation files
│       ├── en.json
│       ├── ja.json
│       └── zh.json
└── styles/                       # CSS/SCSS styles
    └── globals.css
```

---

### 3.2 ROS 2 Bridge (rosbridge_suite)

**rosbridge_server:**
- **Protocol:** WebSocket (ws://localhost:9090)
- **Message format:** JSON
- **Capabilities:** Topic pub/sub, service calls, action clients

**Installation:**
```bash
sudo apt install ros-humble-rosbridge-suite
```

**Launch:**
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**Configuration:**
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

## 4. React Components

### 4.1 Mission Status Component

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

### 4.2 Destination Selection Component

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

### 4.3 Emergency Stop Component

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

## 5. State Management

### 5.1 ROS Context Provider

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

### 5.2 Custom Hooks

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

## 6. Deployment

### 6.1 Kiosk Mode (Chromium)

**Kiosk Setup:**
```bash
# Install Chromium
sudo apt install chromium-browser

# Create kiosk startup script
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

# Auto-start on boot (systemd service)
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

### 6.2 Next.js Production Build

**Build and Deploy:**
```bash
# Development mode
npm run dev

# Production build
npm run build
npm run start

# Or use standalone output (smaller)
# next.config.js:
module.exports = {
  output: 'standalone',
};
```

**Systemd Service (Next.js):**
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

## 7. Internationalization (i18n)

### 7.1 Translation Files

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

### 7.2 i18n Hook

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

## 8. Testing Strategy

### 8.1 Unit Tests (Jest + React Testing Library)
- Component rendering
- User interactions (button clicks, form inputs)
- State management

### 8.2 Integration Tests
- rosbridge connection
- Topic subscription
- Service calls
- Action clients

### 8.3 User Acceptance Tests
- Touch target size (≥44×44 pixels)
- Daylight readability (≥400 nits screen)
- Multi-language switching
- Emergency stop responsiveness (<500ms)

---

## 9. Performance Targets

| Metric | Target | Validation |
|--------|--------|------------|
| Initial Load Time | <2s | Lighthouse |
| UI Responsiveness | <100ms | User testing |
| WebSocket Latency | <50ms | Network monitoring |
| Touch Target Size | ≥44×44 pixels | UI inspection |
| Screen Brightness | ≥400 nits | Hardware spec |

---

## 10. Implementation Phases

### Phase 1: Basic UI (Sprints 1-2)
- ✅ React + Next.js setup
- ✅ rosbridge integration
- ✅ Mission status display
- ✅ Emergency stop button

### Phase 2: Navigation UI (Sprints 3-4)
- ✅ Destination selection
- ✅ Waypoint list
- ✅ Navigation progress display

### Phase 3: Advanced Features (Sprints 5-6)
- ✅ Diagnostics panel
- ✅ Localization initialization
- ✅ Settings page
- ✅ Multi-language support

### Phase 4: Deployment (Sprints 7-8)
- ✅ Kiosk mode setup
- ✅ Production build
- ✅ Systemd integration
- ✅ User acceptance testing

---

**Document Status:** Draft
**Implementation Status:** Not Started
**Approvals Required:** UI/UX Lead, System Architect
