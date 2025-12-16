# UI & eHMI Requirements

**Document ID:** REQ-UI-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft

---

## 1. Introduction

This document specifies requirements for the **User Interface (Touch Screen)** and **External Human-Machine Interface (eHMI)** subsystems.

---

## 2. Touch Screen UI Requirements

### 2.1 Hardware (UI-HW)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| UI-HW-001 | System SHALL have 7-10 inch touch screen | Critical | Hardware BOM |
| UI-HW-002 | Touch screen SHALL be viewable in daylight (≥400 nits) | High | Field test |
| UI-HW-003 | Touch screen SHALL have IP65+ front panel | Critical | IP test |
| UI-HW-004 | Touch screen SHALL support capacitive multi-touch | High | Hardware test |

**Total: 4**

### 2.2 Software Stack (UI-SW)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| UI-SW-001 | UI SHALL be built with React 18 + Next.js 14 | Critical | Code inspection |
| UI-SW-002 | UI SHALL use TypeScript for type safety | High | Code inspection |
| UI-SW-003 | UI SHALL communicate with ROS 2 via rosbridge | Critical | Integration test |
| UI-SW-004 | UI SHALL run in Chromium/Electron kiosk mode | High | Deployment test |

**Total: 4**

### 2.3 User Flows (UI-FLOW)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| UI-FLOW-001 | UI SHALL display robot status (battery, location, mission) | Critical | UI test |
| UI-FLOW-002 | UI SHALL allow destination selection from waypoint list | Critical | UI test |
| UI-FLOW-003 | UI SHALL display mission progress (ETA, distance) | High | UI test |
| UI-FLOW-004 | UI SHALL provide emergency stop button | Critical | UI test |
| UI-FLOW-005 | UI SHALL display diagnostics and error messages | High | UI test |
| UI-FLOW-006 | UI SHALL support manual localization initialization | High | UI test |

**Total: 6**

### 2.4 Accessibility (UI-ACCESS)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| UI-ACCESS-001 | UI SHALL have large touch targets (≥44×44 pixels) | High | Ergonomic test |
| UI-ACCESS-002 | UI SHALL support multiple languages (EN, JP, ZH) | High | Localization test |
| UI-ACCESS-003 | UI SHALL have high contrast mode for low vision | Medium | Accessibility test |
| UI-ACCESS-004 | UI SHALL provide audio feedback for actions | Medium | Audio test |

**Total: 4**

---

## 3. eHMI Requirements

### 3.1 Hardware Components (EHMI-HW)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| EHMI-HW-001 | System SHALL have ESP32-S3 microcontroller | Critical | Hardware BOM |
| EHMI-HW-002 | System SHALL have WS2812B LED strip (≥60 LEDs) | Critical | Hardware BOM |
| EHMI-HW-003 | System SHALL have HUB75 LED matrix (64×32 pixels) | High | Hardware BOM |
| EHMI-HW-004 | System SHALL have I2S audio amplifier + speaker | High | Hardware BOM |
| EHMI-HW-005 | eHMI SHALL communicate via serial (115200 baud) | Critical | Interface test |

**Total: 5**

### 3.2 eHMI States (EHMI-STATE)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| EHMI-STATE-001 | eHMI SHALL implement states 0-20 (standard robot states) | Critical | State test |
| EHMI-STATE-002 | eHMI SHALL implement states 21-28 (wheelchair-specific) | Critical | State test |
| EHMI-STATE-003 | State 21: Approaching Wheelchair | High | Integration test |
| EHMI-STATE-004 | State 22: Docking in Progress | High | Integration test |
| EHMI-STATE-005 | State 23: Docking Complete | High | Integration test |
| EHMI-STATE-006 | State 24: Passenger Onboard | High | Integration test |
| EHMI-STATE-007 | State 25: Transport in Progress | High | Integration test |
| EHMI-STATE-008 | State 26: Destination Reached | High | Integration test |
| EHMI-STATE-009 | State 27: Passenger Disembark | High | Integration test |
| EHMI-STATE-010 | State 28: Docking Failed | High | Integration test |

**Total: 10**

### 3.3 LED Patterns (EHMI-LED)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| EHMI-LED-001 | LED strip SHALL display state-specific colors/patterns | Critical | Visual test |
| EHMI-LED-002 | LED matrix SHALL display arrows/icons for robot intent | High | Visual test |
| EHMI-LED-003 | Emergency state SHALL use red flashing pattern | Critical | Safety test |
| EHMI-LED-004 | LED brightness SHALL adjust based on ambient light | Medium | Integration test |

**Total: 4**

### 3.4 Audio Feedback (EHMI-AUDIO)

| Requirement ID | Requirement | Priority | Validation |
|----------------|-------------|----------|------------|
| EHMI-AUDIO-001 | System SHALL play audio announcements for key events | High | Audio test |
| EHMI-AUDIO-002 | Audio SHALL support multiple languages (EN, JP, ZH) | High | Localization test |
| EHMI-AUDIO-003 | Audio volume SHALL be adjustable (0-10) | High | Configuration test |
| EHMI-AUDIO-004 | Emergency state SHALL play siren sound | Critical | Safety test |

**Total: 4**

---

## 4. Requirements Summary

| Category | Total | Critical | High | Medium | Low |
|----------|-------|----------|------|--------|-----|
| **Touch Screen UI** | **18** | 7 | 10 | 1 | 0 |
| **eHMI Hardware** | **5** | 4 | 1 | 0 | 0 |
| **eHMI States** | **10** | 2 | 8 | 0 | 0 |
| **eHMI LED** | **4** | 2 | 1 | 1 | 0 |
| **eHMI Audio** | **4** | 1 | 3 | 0 | 0 |
| **TOTAL** | **41** | **16 (39%)** | **23 (56%)** | **2 (5%)** | **0** |

---

## 5. Acceptance Criteria

### MVP UI/eHMI
- ✅ Basic touch screen with mission status
- ✅ LED strip with basic states
- ✅ Emergency stop button (physical + UI)

### Production UI/eHMI
- ✅ All MVP + all high-priority
- ✅ Full LED matrix with icons
- ✅ Audio announcements
- ✅ Multi-language support
- ✅ All wheelchair states 21-28

---

**Document Status:** Draft
**Approvals Required:** UI/UX Lead, eHMI Engineer, System Architect
