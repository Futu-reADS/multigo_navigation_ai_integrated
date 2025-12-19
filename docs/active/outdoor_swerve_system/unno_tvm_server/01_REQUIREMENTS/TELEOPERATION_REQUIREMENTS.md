# Teleoperation Requirements

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Document Type:** Fleet Management + Vehicle Requirements Specification
**Team Responsibility:** Pankaj (Vehicle Software) + Unno (Teleop UI)
**Status:** Week 5 - Active Development
**Date:** December 16, 2025
**Version:** 1.0

---

## Document Control

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-16 | Multi-Team | Initial teleoperation requirements |

**Related Documents:**
- COMMUNICATION_SYSTEM_REQUIREMENTS.md (WiFi, 4G LTE latency)
- FLEET_UI_REQUIREMENTS.md (web-based teleop interface)
- SAFETY_REQUIREMENTS.md (teleoperation safety constraints)
- SWERVE_DRIVE_REQUIREMENTS.md (vehicle control commands)

---

## 1. Purpose and Scope

This document specifies requirements for **remote teleoperation** of wheelchair transport robots, allowing operators to manually control vehicles when autonomous operation is not feasible (e.g., recovery from errors, navigation in unmapped areas, testing).

**Design Principles:**
- Safety-first: Reduced speed limits, operator confirmation
- Low-bandwidth operation (functional over 4G LTE)
- Multi-modal control (gamepad, keyboard, touch)
- Latency-tolerant interface (visual feedback, predictive indicators)

**Use Cases:**
- Emergency vehicle recovery (stuck, navigation failure)
- Mapping new areas (manual exploration + SLAM)
- Vehicle testing and commissioning
- Operator training and demonstration

---

## 2. Teleoperation Modes

### 2.1 Mode Definitions

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| TELEOP-MODE-001 | System SHALL support three teleoperation modes: disabled, assisted, manual | CRITICAL | All modes implemented |
| TELEOP-MODE-002 | **Disabled mode**: Teleoperation not available (normal autonomous operation) | CRITICAL | Mode transition functional |
| TELEOP-MODE-003 | **Assisted mode**: Operator sends high-level goals, vehicle navigates autonomously | HIGH | Assisted mode functional |
| TELEOP-MODE-004 | **Manual mode**: Operator directly controls vehicle motion (joystick/keyboard input) | CRITICAL | Manual mode functional |
| TELEOP-MODE-005 | System SHALL require operator confirmation to enter manual mode (prevent accidental activation) | CRITICAL | Confirmation dialog required |

**Success Criteria:** Mode transitions clear and fail-safe

### 2.2 Mode Transitions

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| TELEOP-TRANS-001 | Admin or Operator role SHALL be required to activate teleoperation | CRITICAL | RBAC enforced |
| TELEOP-TRANS-002 | System SHALL automatically exit manual mode on operator inactivity (>10 seconds no input) | CRITICAL | Auto-exit functional |
| TELEOP-TRANS-003 | System SHALL exit manual mode immediately on emergency stop | CRITICAL | E-stop exits manual mode |
| TELEOP-TRANS-004 | System SHALL display prominent warning when entering manual mode (UI alert + vehicle audio) | HIGH | Warning displayed |
| TELEOP-TRANS-005 | System SHALL log all teleoperation sessions: operator, start time, duration, mode | HIGH | Teleoperation logging functional |

---

## 3. Manual Teleoperation Requirements

### 3.1 Control Interface

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| TELEOP-CTRL-001 | System SHALL support gamepad control (Xbox/PS4 compatible, USB/Bluetooth) | HIGH | Gamepad input functional |
| TELEOP-CTRL-002 | System SHALL support keyboard control (W/A/S/D for motion, arrow keys for camera) | MEDIUM | Keyboard input functional |
| TELEOP-CTRL-003 | System SHALL support touch control on tablet/mobile (virtual joystick) | LOW | Touch control functional |
| TELEOP-CTRL-004 | Gamepad layout SHALL be: left stick = translation (forward/backward/strafe), right stick = rotation | HIGH | Gamepad mapping correct |
| TELEOP-CTRL-005 | System SHALL display current control method in UI (gamepad icon, keyboard icon) | MEDIUM | Control method indicator shown |

**Gamepad Mapping:**
- Left stick Y-axis: Forward/backward velocity
- Left stick X-axis: Left/right strafe
- Right stick X-axis: Rotation (yaw)
- Right trigger: Speed boost (max speed increase)
- A button: Honk/warning sound
- B button: Emergency stop
- Start button: Enable/disable teleoperation

### 3.2 Velocity Control

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| TELEOP-VEL-001 | Manual mode maximum speed SHALL be 0.5 m/s (50% of normal autonomous speed) | CRITICAL | Speed limit enforced |
| TELEOP-VEL-002 | System SHALL apply velocity smoothing (0.5 m/s² acceleration limit) | HIGH | Smooth acceleration |
| TELEOP-VEL-003 | System SHALL support speed boost (right trigger pressed = 0.8 m/s max) | MEDIUM | Speed boost functional |
| TELEOP-VEL-004 | System SHALL apply deadzone to joystick input (±10% stick deflection = zero velocity) | HIGH | Deadzone prevents drift |
| TELEOP-VEL-005 | System SHALL display current commanded velocity vs actual velocity in UI | MEDIUM | Velocity display functional |

### 3.3 Teleoperation Safety

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| TELEOP-SAFE-001 | System SHALL maintain obstacle detection during teleoperation (emergency stop on collision) | CRITICAL | Obstacle detection active |
| TELEOP-SAFE-002 | System SHALL enforce geofencing during teleoperation (prevent leaving operational area) | CRITICAL | Geofencing enforced |
| TELEOP-SAFE-003 | System SHALL display collision warning overlay in video feed (red bounding boxes) | HIGH | Collision warnings visible |
| TELEOP-SAFE-004 | System SHALL require operator to hold "dead-man switch" (button held = motion enabled) | CRITICAL | Dead-man switch implemented |
| TELEOP-SAFE-005 | Vehicle SHALL stop immediately (within 0.5s) when dead-man switch released | CRITICAL | Immediate stop on release |
| TELEOP-SAFE-006 | System SHALL prohibit teleoperation when passenger onboard (safety policy) | CRITICAL | Passenger detection blocks teleop |

---

## 4. Video Streaming Requirements

### 4.1 Camera Configuration

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| TELEOP-VIDEO-001 | System SHALL stream video from front camera (primary view) at 720p, 15 fps | CRITICAL | Video stream functional |
| TELEOP-VIDEO-002 | System SHALL optionally stream rear camera (when reversing) at 480p, 10 fps | MEDIUM | Rear camera stream functional |
| TELEOP-VIDEO-003 | Video SHALL include status overlay: battery %, connection strength, speed, mode | HIGH | Status overlay displayed |
| TELEOP-VIDEO-004 | System SHALL support camera PTZ control (pan-tilt-zoom, if camera supports) | LOW | PTZ control functional |

### 4.2 Video Encoding and Transmission

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| TELEOP-VIDEO-ENC-001 | Video SHALL use H.264 codec (hardware encoding on Jetson Orin Nano) | CRITICAL | H.264 encoding functional |
| TELEOP-VIDEO-ENC-002 | Video bitrate SHALL adapt to network bandwidth: 2 Mbps (WiFi), 500 kbps (LTE) | CRITICAL | Adaptive bitrate functional |
| TELEOP-VIDEO-ENC-003 | Video SHALL use WebRTC for low-latency streaming (P2P when possible) | HIGH | WebRTC streaming implemented |
| TELEOP-VIDEO-ENC-004 | Video latency SHALL be <500ms glass-to-glass (WiFi), <1000ms (LTE) | HIGH | Latency measured and acceptable |
| TELEOP-VIDEO-ENC-005 | System SHALL display video latency indicator in UI (green <500ms, yellow <1000ms, red >1000ms) | MEDIUM | Latency indicator functional |

### 4.3 Video Quality Degradation

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| TELEOP-VIDEO-DEG-001 | System SHALL reduce video resolution on poor network (720p → 480p → 360p) | HIGH | Resolution adaptation functional |
| TELEOP-VIDEO-DEG-002 | System SHALL reduce frame rate on poor network (15fps → 10fps → 5fps) | HIGH | Frame rate adaptation functional |
| TELEOP-VIDEO-DEG-003 | System SHALL display "Low Bandwidth" warning when video quality degraded | MEDIUM | Warning displayed |
| TELEOP-VIDEO-DEG-004 | System SHALL maintain teleoperation functionality even without video (sensor overlay only) | MEDIUM | Text-only teleop possible |

---

## 5. Operator UI Requirements

### 5.1 Teleoperation Dashboard

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| TELEOP-UI-001 | Teleop UI SHALL display main video feed (占70% of screen area) | CRITICAL | Video feed prominent |
| TELEOP-UI-002 | UI SHALL display vehicle status panel: battery, speed, mode, errors | HIGH | Status panel visible |
| TELEOP-UI-003 | UI SHALL display LiDAR overlay on video (top-down 2D scan, semi-transparent) | HIGH | LiDAR overlay functional |
| TELEOP-UI-004 | UI SHALL display map with vehicle position and teleoperation path trace | MEDIUM | Map display functional |
| TELEOP-UI-005 | UI SHALL provide emergency stop button (large, red, always visible) | CRITICAL | E-stop button accessible |
| TELEOP-UI-006 | UI SHALL display network latency: video latency, control latency, connection quality | HIGH | Latency metrics displayed |

### 5.2 Control Feedback

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| TELEOP-FEEDBACK-001 | UI SHALL display joystick input visualization (virtual joystick indicator) | MEDIUM | Input visualization shown |
| TELEOP-FEEDBACK-002 | UI SHALL provide haptic feedback on gamepad (vibration on collision warning) | LOW | Haptic feedback functional |
| TELEOP-FEEDBACK-003 | UI SHALL display vehicle orientation arrow (compass heading) | MEDIUM | Orientation arrow shown |
| TELEOP-FEEDBACK-004 | UI SHALL highlight active sensors (green=OK, yellow=degraded, red=failed) | HIGH | Sensor status indicators |

---

## 6. Assisted Teleoperation Requirements

### 6.1 Goal-Based Control

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| TELEOP-ASSIST-001 | Assisted mode SHALL allow operator to click map to set waypoint goal | HIGH | Waypoint setting functional |
| TELEOP-ASSIST-002 | Vehicle SHALL autonomously navigate to clicked goal using Nav2 planner | HIGH | Autonomous navigation to goal |
| TELEOP-ASSIST-003 | Operator SHALL be able to cancel current goal and set new goal | HIGH | Goal cancellation functional |
| TELEOP-ASSIST-004 | System SHALL display planned path to goal on map (dashed line) | MEDIUM | Path visualization functional |

### 6.2 Assisted Safety

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| TELEOP-ASSIST-SAFE-001 | Assisted mode SHALL use normal autonomous safety constraints (obstacle avoidance, geofencing) | CRITICAL | Safety constraints active |
| TELEOP-ASSIST-SAFE-002 | Operator SHALL be able to emergency stop vehicle during assisted navigation | CRITICAL | E-stop functional in assisted mode |

---

## 7. Multi-Vehicle Teleoperation Requirements

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| TELEOP-MULTI-001 | System SHALL support teleoperating one vehicle at a time (no simultaneous multi-vehicle control) | CRITICAL | Single-vehicle control enforced |
| TELEOP-MULTI-002 | Operator SHALL be able to switch control between vehicles (with confirmation dialog) | HIGH | Vehicle switching functional |
| TELEOP-MULTI-003 | System SHALL prevent two operators from controlling same vehicle simultaneously | CRITICAL | Control locking functional |
| TELEOP-MULTI-004 | UI SHALL display which vehicle is currently under teleoperation control | HIGH | Active vehicle highlighted |

---

## 8. Teleoperation Session Management

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| TELEOP-SESSION-001 | System SHALL require operator login before accessing teleoperation (authentication) | CRITICAL | Login enforced |
| TELEOP-SESSION-002 | Teleoperation session SHALL timeout after 30 minutes (require re-authentication) | HIGH | Session timeout functional |
| TELEOP-SESSION-003 | System SHALL log teleoperation sessions with: operator, vehicle, start/end time, distance traveled | HIGH | Session logging functional |
| TELEOP-SESSION-004 | System SHALL display teleoperation history in fleet UI (audit trail) | MEDIUM | History display functional |

---

## 9. Network and Latency Requirements

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| TELEOP-NET-001 | Teleoperation SHALL be functional over WiFi (preferred, <100ms latency) | CRITICAL | WiFi teleop functional |
| TELEOP-NET-002 | Teleoperation SHALL be functional over 4G LTE (fallback, <500ms latency acceptable) | HIGH | LTE teleop functional |
| TELEOP-NET-003 | System SHALL measure round-trip latency (operator input → vehicle response) | HIGH | Latency measurement functional |
| TELEOP-NET-004 | System SHALL warn operator when latency >500ms (yellow warning) or >1000ms (red warning) | HIGH | Latency warnings displayed |
| TELEOP-NET-005 | System SHALL automatically exit manual mode if network disconnected >5 seconds | CRITICAL | Auto-exit on disconnect |

---

## 10. Teleoperation Performance Requirements

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| TELEOP-PERF-001 | Control command latency SHALL be <100ms (operator input → vehicle motion) over WiFi | HIGH | Latency <100ms measured |
| TELEOP-PERF-002 | Video latency SHALL be <500ms (camera → operator screen) over WiFi | HIGH | Latency <500ms measured |
| TELEOP-PERF-003 | System SHALL maintain 15 fps video streaming minimum (acceptable operator experience) | MEDIUM | 15 fps minimum maintained |
| TELEOP-PERF-004 | Teleoperation SHALL consume <5 Mbps bandwidth total (video + control + telemetry) | MEDIUM | Bandwidth usage measured |

---

## 11. Emergency and Fault Handling

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| TELEOP-FAULT-001 | System SHALL exit teleoperation mode on vehicle emergency stop | CRITICAL | E-stop exits teleop |
| TELEOP-FAULT-002 | System SHALL exit teleoperation mode on critical sensor failure (LiDAR, cameras) | CRITICAL | Sensor failure exits teleop |
| TELEOP-FAULT-003 | System SHALL notify operator on vehicle fault during teleoperation (visual + audio alert) | HIGH | Fault notifications displayed |
| TELEOP-FAULT-004 | System SHALL record teleoperation session video for post-incident review | LOW | Session recording functional |

---

## 12. Teleoperation Training and Testing

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| TELEOP-TRAIN-001 | System SHALL provide teleoperation training mode (limited speed, safe area only) | MEDIUM | Training mode functional |
| TELEOP-TRAIN-002 | Training mode SHALL be accessible to operators with "Trainee" role | MEDIUM | Role-based training access |
| TELEOP-TRAIN-003 | System SHALL provide teleoperation tutorial UI (on-screen instructions) | LOW | Tutorial UI available |

---

## 13. Requirements Summary

| Category | Count | Priority Breakdown |
|----------|-------|-------------------|
| Teleoperation Modes | 10 | Critical: 7, High: 3 |
| Manual Teleoperation | 16 | Critical: 8, High: 5, Medium: 2, Low: 1 |
| Video Streaming | 15 | Critical: 5, High: 7, Medium: 3 |
| Operator UI | 10 | Critical: 2, High: 5, Medium: 2, Low: 1 |
| Assisted Teleoperation | 6 | Critical: 2, High: 4 |
| Multi-Vehicle Teleoperation | 4 | Critical: 2, High: 2 |
| Session Management | 4 | Critical: 1, High: 2, Medium: 1 |
| Network and Latency | 5 | Critical: 2, High: 3 |
| Performance | 4 | High: 2, Medium: 2 |
| Emergency and Fault Handling | 4 | Critical: 2, High: 1, Low: 1 |
| Training and Testing | 3 | Medium: 2, Low: 1 |
| **TOTAL** | **81** | **Critical: 31, High: 34, Medium: 13, Low: 3** |

---

## 14. Teleoperation System Diagram

```
┌──────────────────────────────────────────────────────────┐
│               Operator Workstation                       │
│                                                          │
│  ┌──────────────┐      ┌───────────────┐               │
│  │   Web UI     │      │   Gamepad     │               │
│  │  (Browser)   │◄─────┤  (USB/BT)     │               │
│  │              │      └───────────────┘               │
│  │ • Video feed │                                       │
│  │ • LiDAR overlay                                      │
│  │ • Status panel                                       │
│  │ • Map view   │                                       │
│  └──────┬───────┘                                       │
│         │ WebRTC (video)                                │
│         │ WebSocket (control + telemetry)               │
└─────────┼──────────────────────────────────────────────┘
          │
          │ WiFi (preferred, <100ms) or LTE (fallback, <500ms)
          │
┌─────────▼──────────────────────────────────────────────┐
│              Vehicle (Jetson Orin Nano)                │
│                                                        │
│  ┌───────────────┐    ┌────────────────┐             │
│  │ Teleop Node   │───>│ Swerve Drive   │             │
│  │ (ROS 2)       │    │ Controller     │             │
│  └───────┬───────┘    └────────────────┘             │
│          │                                            │
│          ▼                                            │
│  ┌───────────────┐    ┌────────────────┐             │
│  │ Video Encoder │    │ Safety Monitor │             │
│  │ (H.264)       │    │ (obstacles,    │             │
│  │               │    │  geofencing)   │             │
│  └───────────────┘    └────────────────┘             │
│                                                        │
└────────────────────────────────────────────────────────┘
```

---

## 15. Traceability Matrix

| Teleoperation Requirement Category | Related System Components |
|-----------------------------------|---------------------------|
| Control Interface | ROS 2 teleop node, swerve drive controller, gamepad driver |
| Video Streaming | Front/rear cameras, H.264 encoder (Jetson), WebRTC server |
| Operator UI | Fleet management web UI, WebSocket server, video player |
| Safety | Obstacle detection, geofencing, emergency stop, dead-man switch |

---

## 16. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-16 | Multi-Team | Initial teleoperation requirements (81 requirements) |

---

**Document End**
