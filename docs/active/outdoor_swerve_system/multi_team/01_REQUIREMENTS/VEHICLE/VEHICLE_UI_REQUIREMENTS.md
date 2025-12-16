# Vehicle UI Requirements

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Document Type:** Vehicle Requirements Specification
**Team Responsibility:** Pankaj (Vehicle Software)
**Status:** Week 5 - Active Development
**Date:** December 16, 2025
**Version:** 1.0

---

## Document Control

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-16 | Pankaj Team | Initial vehicle UI requirements |

**Related Documents:**
- UI_EHMI_REQUIREMENTS.md (external display, eHMI ESP32)
- EXTERIOR_REQUIREMENTS.md (physical touchscreen mounting, IP rating)
- FLEET_UI_REQUIREMENTS.md (fleet management web dashboard)
- SAFETY_REQUIREMENTS.md (emergency stop UI integration)

---

## 1. Purpose and Scope

This document specifies requirements for the **on-vehicle touchscreen UI**, a 10.1-inch display mounted on the robot for:
- **Local status monitoring** (operators, caregivers checking vehicle state)
- **Mission control** (start/stop missions, select destinations)
- **Diagnostics** (error codes, sensor status, logs)
- **Configuration** (WiFi setup, vehicle settings - admin only)
- **Emergency operations** (manual e-stop, call for help)

**Design Principles:**
- Large touch targets (minimum 60mm) for elderly users with gloves
- High contrast UI (readable in bright sunlight)
- Simple, icon-based interface (minimize text)
- Bilingual support (English/Japanese)

**Technology Stack:**
- Display: 10.1" 1920×1200 industrial touchscreen (IP65, 1000 nits brightness)
- Software: Qt 6 C++ application (native, low latency)
- Communication: ROS 2 topics for vehicle status, commands

---

## 2. Hardware Requirements

### 2.1 Touchscreen Display Specifications

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-HW-001 | Vehicle SHALL have 10.1" touchscreen display (1920×1200 resolution, 16:10 aspect) | CRITICAL | Display installed and functional |
| VEH-UI-HW-002 | Display brightness SHALL be ≥1000 nits (readable in direct sunlight) | HIGH | Brightness measured |
| VEH-UI-HW-003 | Display SHALL be IP65 rated (waterproof for outdoor use) | CRITICAL | IP65 certification verified |
| VEH-UI-HW-004 | Touchscreen SHALL support capacitive touch (glove mode enabled) | HIGH | Glove touch functional |
| VEH-UI-HW-005 | Display SHALL be bonded (no air gap) to reduce glare and improve sunlight readability | MEDIUM | Optical bonding verified |
| VEH-UI-HW-006 | Display viewing angle SHALL be ≥170° horizontal, ≥160° vertical | MEDIUM | Viewing angle measured |

### 2.2 Physical Mounting

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-MOUNT-001 | Display SHALL be mounted on robot side panel at 1200mm height (accessible to standing adult) | HIGH | Mounting height verified |
| VEH-UI-MOUNT-002 | Display SHALL be angled 15° upward for ergonomic viewing | MEDIUM | Angle measured |
| VEH-UI-MOUNT-003 | Display enclosure SHALL have rubber gasket seal (IP65 protection) | CRITICAL | Waterproofing verified |
| VEH-UI-MOUNT-004 | Display SHALL have protective acrylic shield (prevent scratching, 3mm thick) | MEDIUM | Shield installed |

---

## 3. Home Screen Requirements

### 3.1 Home Screen Layout

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-HOME-001 | Home screen SHALL display large vehicle status: IDLE, NAVIGATING, DOCKING, TRANSPORTING, CHARGING, ERROR | CRITICAL | Status displayed prominently |
| VEH-UI-HOME-002 | Home screen SHALL display battery icon with percentage (color-coded: green >50%, yellow 30-50%, red <30%) | CRITICAL | Battery display functional |
| VEH-UI-HOME-003 | Home screen SHALL display WiFi signal strength (4-bar indicator) | HIGH | WiFi indicator functional |
| VEH-UI-HOME-004 | Home screen SHALL display current mission (if active): "Walking Assistance to Building A" | HIGH | Mission description shown |
| VEH-UI-HOME-005 | Home screen SHALL display ETA to destination (minutes remaining) | MEDIUM | ETA calculation and display |
| VEH-UI-HOME-006 | Home screen SHALL provide large buttons: "View Map", "Diagnostics", "Settings", "Emergency" | CRITICAL | Navigation buttons functional |

**Home Screen Layout:**
```
┌─────────────────────────────────────────┐
│  Vehicle VEH-03          🔋 65%  📶 ●●●○│
├─────────────────────────────────────────┤
│                                         │
│          STATUS: IDLE                   │
│           (Large, centered)             │
│                                         │
│  Current Mission: None                  │
│                                         │
├─────────────────────────────────────────┤
│  [📍 View Map]     [🔧 Diagnostics]    │
│                                         │
│  [⚙️ Settings]      [🚨 Emergency]      │
└─────────────────────────────────────────┘
```

### 3.2 Status Indicators

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-STATUS-001 | Status text SHALL be large (minimum 80mm height, bold font) | HIGH | Text size verified |
| VEH-UI-STATUS-002 | Status SHALL be color-coded: green (OK states), blue (active), yellow (caution), red (error) | CRITICAL | Color coding applied |
| VEH-UI-STATUS-003 | Error status SHALL include brief error message (e.g., "Navigation Failed - Obstacle") | HIGH | Error messages displayed |
| VEH-UI-STATUS-004 | Home screen SHALL auto-refresh every 2 seconds (live status updates) | HIGH | Refresh rate verified |

---

## 4. Map View Requirements

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-MAP-001 | Map view SHALL display 2D occupancy grid map with vehicle position (red arrow) | HIGH | Map displayed correctly |
| VEH-UI-MAP-002 | Map SHALL display current navigation path (blue line) if mission active | HIGH | Path visualization functional |
| VEH-UI-MAP-003 | Map SHALL support pinch-zoom gesture (zoom in/out) | MEDIUM | Pinch-zoom functional |
| VEH-UI-MAP-004 | Map SHALL display nearby obstacles (LiDAR scan overlay, semi-transparent red) | MEDIUM | Obstacle overlay functional |
| VEH-UI-MAP-005 | Map view SHALL have "Back to Home" button (return to home screen) | HIGH | Navigation button functional |
| VEH-UI-MAP-006 | Map SHALL display destination marker (green flag icon) when mission active | MEDIUM | Destination marker shown |

---

## 5. Diagnostics View Requirements

### 5.1 Sensor Status

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-DIAG-001 | Diagnostics SHALL display sensor status grid: LiDAR, Front Camera, Rear Camera, IMU, BMS, Motors | CRITICAL | Sensor grid displayed |
| VEH-UI-DIAG-002 | Each sensor SHALL show status icon: ✅ (OK), ⚠️ (Warning), ❌ (Failed) | CRITICAL | Status icons correct |
| VEH-UI-DIAG-003 | Tapping sensor SHALL show detailed info: sensor name, data rate, last update time, error code | HIGH | Detail view functional |
| VEH-UI-DIAG-004 | Diagnostics SHALL display motor controller status (8 controllers, temperature, current) | HIGH | Motor status displayed |

### 5.2 Error Logs

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-DIAG-LOG-001 | Diagnostics SHALL display recent error log (last 20 entries) | HIGH | Error log displayed |
| VEH-UI-DIAG-LOG-002 | Error log entries SHALL include: timestamp, severity (INFO/WARN/ERROR), message | HIGH | Log fields complete |
| VEH-UI-DIAG-LOG-003 | Error log SHALL support scrolling (swipe up/down) | MEDIUM | Scrolling functional |
| VEH-UI-DIAG-LOG-004 | Error log SHALL auto-refresh every 5 seconds | MEDIUM | Auto-refresh functional |

### 5.3 System Information

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-DIAG-SYS-001 | Diagnostics SHALL display system info: vehicle ID, software version, IP address, uptime | MEDIUM | System info displayed |
| VEH-UI-DIAG-SYS-002 | Diagnostics SHALL display resource usage: CPU %, memory usage, disk usage | MEDIUM | Resource usage displayed |

---

## 6. Settings View Requirements

### 6.1 Network Settings

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-SET-NET-001 | Admin SHALL be able to configure WiFi SSID and password (on-screen keyboard) | HIGH | WiFi config functional |
| VEH-UI-SET-NET-002 | Settings SHALL display current WiFi connection status and IP address | HIGH | WiFi status displayed |
| VEH-UI-SET-NET-003 | Settings SHALL allow testing WiFi connection (ping TVM server) | MEDIUM | Connection test functional |

### 6.2 Vehicle Settings

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-SET-VEH-001 | Admin SHALL be able to configure vehicle ID (alphanumeric, max 10 chars) | MEDIUM | Vehicle ID editable |
| VEH-UI-SET-VEH-002 | Admin SHALL be able to set maximum speed limit (0.5 - 1.5 m/s) | MEDIUM | Speed limit configurable |
| VEH-UI-SET-VEH-003 | Settings SHALL display battery health (SOH %, cycle count, last service date) | MEDIUM | Battery health displayed |

### 6.3 Display Settings

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-SET-DISP-001 | Settings SHALL allow adjusting display brightness (10% - 100%) | MEDIUM | Brightness adjustment functional |
| VEH-UI-SET-DISP-002 | Settings SHALL allow selecting language: English or Japanese | HIGH | Language selection functional |
| VEH-UI-SET-DISP-003 | Settings SHALL allow toggling auto-brightness (adapt to ambient light) | LOW | Auto-brightness toggle functional |

### 6.4 Access Control

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-SET-ACCESS-001 | Settings view SHALL require admin PIN (4-digit code) to access | CRITICAL | PIN protection enforced |
| VEH-UI-SET-ACCESS-002 | Settings SHALL lock automatically after 2 minutes of inactivity | HIGH | Auto-lock functional |
| VEH-UI-SET-ACCESS-003 | Failed PIN attempts SHALL be logged (security audit) | MEDIUM | Failed attempts logged |

---

## 7. Emergency View Requirements

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-EMERG-001 | Emergency view SHALL display large "EMERGENCY STOP" button (300mm × 150mm, red) | CRITICAL | E-stop button prominent |
| VEH-UI-EMERG-002 | Pressing E-stop button SHALL immediately stop vehicle (same as physical e-stop button) | CRITICAL | UI e-stop functional |
| VEH-UI-EMERG-003 | Emergency view SHALL display "Call Operator" button (initiate voice/video call) | HIGH | Call button functional |
| VEH-UI-EMERG-004 | Emergency view SHALL display current GPS coordinates (for emergency services) | MEDIUM | GPS coordinates displayed |
| VEH-UI-EMERG-005 | Emergency view SHALL have "Cancel Emergency" button (restore normal operation) | HIGH | E-stop release functional |
| VEH-UI-EMERG-006 | Emergency activation SHALL send alert to TVM server immediately | CRITICAL | Alert transmission functional |

---

## 8. Touch Interaction Requirements

### 8.1 Touch Targets and Gestures

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-TOUCH-001 | All interactive elements SHALL be minimum 60mm × 60mm (accessible with gloves) | CRITICAL | Touch target size verified |
| VEH-UI-TOUCH-002 | Touch targets SHALL have 10mm spacing (prevent accidental adjacent presses) | HIGH | Spacing verified |
| VEH-UI-TOUCH-003 | System SHALL provide tactile feedback (button press animation, 100ms highlight) | MEDIUM | Visual feedback functional |
| VEH-UI-TOUCH-004 | System SHALL support multi-touch gestures: pinch-zoom, swipe | MEDIUM | Gestures functional |
| VEH-UI-TOUCH-005 | Touch response latency SHALL be <100ms (perceived as instant) | HIGH | Latency measured |

### 8.2 On-Screen Keyboard

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-KB-001 | System SHALL provide on-screen keyboard for text entry (alphanumeric, symbols) | HIGH | Keyboard functional |
| VEH-UI-KB-002 | Keyboard keys SHALL be large (40mm × 40mm minimum) | HIGH | Key size verified |
| VEH-UI-KB-003 | Keyboard SHALL support English and Japanese input (character switching) | HIGH | Bilingual input functional |

---

## 9. Visual Design Requirements

### 9.1 Color Scheme and Contrast

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-VISUAL-001 | UI SHALL use high contrast colors (minimum 7:1 contrast ratio per WCAG AAA) | HIGH | Contrast ratio measured |
| VEH-UI-VISUAL-002 | UI SHALL use dark background (reduce eye strain in bright sunlight) | MEDIUM | Dark theme applied |
| VEH-UI-VISUAL-003 | Status colors SHALL be consistent: green=OK, blue=active, yellow=caution, red=error | CRITICAL | Color coding consistent |
| VEH-UI-VISUAL-004 | Text SHALL be sans-serif font (Roboto or Noto Sans), minimum 18pt | HIGH | Font verified |

### 9.2 Icons and Symbols

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-ICON-001 | UI SHALL use internationally recognized icons (ISO 7000 standard where applicable) | MEDIUM | Icons intuitive |
| VEH-UI-ICON-002 | Critical icons (emergency, battery, errors) SHALL be ≥40mm size | HIGH | Icon size verified |
| VEH-UI-ICON-003 | Icons SHALL have text labels below (bilingual: English/Japanese) | HIGH | Labels present |

---

## 10. Localization and Language Support

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-L10N-001 | UI SHALL support English and Japanese languages | CRITICAL | Both languages functional |
| VEH-UI-L10N-002 | Language SHALL be selectable via Settings view (persistent across reboots) | HIGH | Language selection persistent |
| VEH-UI-L10N-003 | All UI text SHALL be externalized in translation files (no hardcoded strings) | MEDIUM | Translation files used |
| VEH-UI-L10N-004 | Date/time format SHALL adapt to selected language (EN: MM/DD/YYYY, JA: YYYY/MM/DD) | MEDIUM | Date format localized |

---

## 11. Performance Requirements

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-PERF-001 | UI application startup time SHALL be <5 seconds (from power-on to home screen) | HIGH | Startup time measured |
| VEH-UI-PERF-002 | UI frame rate SHALL be ≥30 fps (smooth animations) | MEDIUM | Frame rate measured |
| VEH-UI-PERF-003 | Touch input latency SHALL be <100ms (responsive feel) | HIGH | Latency measured |
| VEH-UI-PERF-004 | UI CPU usage SHALL be <20% on Jetson Orin Nano (leave resources for navigation) | MEDIUM | CPU usage profiled |
| VEH-UI-PERF-005 | UI memory usage SHALL be <500 MB | LOW | Memory usage measured |

---

## 12. Power Management

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-PWR-001 | Display SHALL dim to 30% brightness after 60 seconds of no touch input (power save) | MEDIUM | Auto-dim functional |
| VEH-UI-PWR-002 | Display SHALL turn off after 5 minutes of no touch input (screen saver) | MEDIUM | Screen saver functional |
| VEH-UI-PWR-003 | Touching screen SHALL wake display immediately (within 500ms) | HIGH | Wake-up functional |
| VEH-UI-PWR-004 | Display power consumption SHALL be <15W at full brightness | LOW | Power consumption measured |

---

## 13. Accessibility Requirements

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-A11Y-001 | UI SHALL support large touch targets (60mm minimum, suitable for elderly users) | CRITICAL | Touch targets accessible |
| VEH-UI-A11Y-002 | UI SHALL use large fonts (minimum 18pt body text, 32pt headings) | HIGH | Font sizes accessible |
| VEH-UI-A11Y-003 | UI SHALL provide audio feedback option (spoken status updates) | LOW | Audio feedback available |
| VEH-UI-A11Y-004 | UI SHALL support glove mode (capacitive touch with gloves) | HIGH | Glove touch functional |

---

## 14. Safety and Fault Tolerance

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| VEH-UI-SAFE-001 | UI SHALL maintain vehicle control functions if display fails (e.g., external e-stop still works) | CRITICAL | Display failure doesn't affect safety |
| VEH-UI-SAFE-002 | UI SHALL display critical warnings with flashing red border (impossible to miss) | CRITICAL | Warning display prominent |
| VEH-UI-SAFE-003 | UI SHALL have watchdog timer (restart UI app if frozen >5 seconds) | HIGH | Watchdog functional |
| VEH-UI-SAFE-004 | UI SHALL log all user interactions for security audit (who pressed what, when) | MEDIUM | Interaction logging functional |

---

## 15. Requirements Summary

| Category | Count | Priority Breakdown |
|----------|-------|-------------------|
| Hardware Requirements | 10 | Critical: 4, High: 4, Medium: 2 |
| Home Screen | 10 | Critical: 4, High: 5, Medium: 1 |
| Map View | 6 | High: 3, Medium: 3 |
| Diagnostics View | 10 | Critical: 2, High: 6, Medium: 2 |
| Settings View | 13 | Critical: 1, High: 4, Medium: 7, Low: 1 |
| Emergency View | 6 | Critical: 3, High: 2, Medium: 1 |
| Touch Interaction | 8 | Critical: 1, High: 5, Medium: 2 |
| Visual Design | 7 | Critical: 1, High: 4, Medium: 2 |
| Localization | 4 | Critical: 1, High: 1, Medium: 2 |
| Performance | 5 | High: 2, Medium: 2, Low: 1 |
| Power Management | 4 | High: 1, Medium: 2, Low: 1 |
| Accessibility | 4 | Critical: 1, High: 2, Low: 1 |
| Safety and Fault Tolerance | 4 | Critical: 2, High: 1, Medium: 1 |
| **TOTAL** | **91** | **Critical: 20, High: 40, Medium: 25, Low: 6** |

---

## 16. Vehicle UI System Architecture

```
┌────────────────────────────────────────────────────┐
│        10.1" Touchscreen Display (Qt 6 App)        │
│                                                    │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐        │
│  │  Home    │  │   Map    │  │Diagnostics│        │
│  │  Screen  │  │   View   │  │   View   │         │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘        │
│       │             │             │               │
│  ┌────┴─────┐  ┌────┴─────┐  ┌────┴─────┐        │
│  │ Settings │  │Emergency │  │Touch Input│        │
│  │   View   │  │   View   │  │ Handler  │         │
│  └──────────┘  └──────────┘  └──────────┘        │
│                                                    │
│       ROS 2 Qt Integration (rclcpp, Qt signals)   │
└─────────────────────┬──────────────────────────────┘
                      │
                      ▼
         ┌────────────────────────────┐
         │   ROS 2 Topics / Services  │
         │                            │
         │  • /vehicle_status         │
         │  • /battery_status         │
         │  • /navigation/goal        │
         │  • /diagnostics            │
         │  • /emergency_stop         │
         └────────────────────────────┘
```

---

## 17. Traceability Matrix

| Vehicle UI Requirement Category | Related System Components |
|--------------------------------|---------------------------|
| Hardware | 10.1" industrial touchscreen, mounting bracket, power supply |
| Home Screen | ROS 2 status topics, battery monitoring, mission planner |
| Diagnostics | ROS 2 diagnostics aggregator, sensor health monitors |
| Settings | Network configuration, vehicle parameters, user authentication |
| Emergency | Emergency stop controller, TVM alert system |

---

## 18. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-16 | Pankaj Team | Initial vehicle UI requirements (91 requirements) |

---

**Document End**
