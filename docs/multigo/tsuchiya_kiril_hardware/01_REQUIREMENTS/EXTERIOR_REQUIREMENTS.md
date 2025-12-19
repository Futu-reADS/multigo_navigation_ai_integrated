# Exterior and Physical Design Requirements

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Document Type:** Hardware Requirements Specification
**Team Responsibility:** Tsuchiya + Kiril (Hardware/Mechanical)
**Status:** Week 4 - Active Development
**Date:** December 16, 2025
**Version:** 1.0

---

## Document Control

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-16 | Tsuchiya Team | Initial exterior and physical design requirements |

**Related Documents:**
- MECHANICAL_REQUIREMENTS.md (chassis structure, swerve drive mechanics)
- ELECTRICAL_REQUIREMENTS.md (lighting power supply)
- SAFETY_REQUIREMENTS.md (collision safety, emergency systems)
- UI_EHMI_REQUIREMENTS.md (external human-machine interface)

---

## 1. Purpose and Scope

This document specifies requirements for the **physical exterior design** of the wheelchair transport robot, including:
- **Weatherproofing** (IP ratings, material selection)
- **Exterior lighting** (status LEDs, safety lights)
- **Branding and identification** (logos, vehicle ID, QR codes)
- **Physical dimensions and clearances**
- **Accessibility features** (for elderly users and caregivers)

**Design Goals:**
- Outdoor-first design (IP54+ weatherproofing)
- Friendly, approachable appearance (not industrial/threatening)
- Clear status communication (LED patterns, displays)
- Easy maintenance (tool-free access to key components)

---

## 2. Physical Dimensions and Clearances

### 2.1 Overall Dimensions

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| EXT-DIM-001 | Robot base SHALL NOT exceed 1200mm length × 900mm width × 600mm height (without wheelchair) | CRITICAL | Dimensions verified with measurements |
| EXT-DIM-002 | Robot with docked wheelchair SHALL NOT exceed 2500mm length (stays within path width limits) | CRITICAL | Combined dimensions measured |
| EXT-DIM-003 | Robot ground clearance SHALL be ≥150mm to handle outdoor terrain (curbs, rough surfaces) | HIGH | Ground clearance measured |
| EXT-DIM-004 | Robot footprint SHALL fit within standard accessibility pathways (minimum 1.5m wide) | HIGH | Fits in pathway |
| EXT-DIM-005 | Robot weight SHALL NOT exceed 150kg (without wheelchair, with battery) | HIGH | Weight measured on scale |

**Success Criteria:** Robot navigates standard wheelchair pathways without obstruction

### 2.2 Clearances and Accessibility

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| EXT-CLEAR-001 | Robot SHALL maintain minimum 300mm clearance from obstacles during navigation | CRITICAL | Clearance verified in tests |
| EXT-CLEAR-002 | Wheelchair docking mechanism SHALL provide ≥400mm approach width (standard wheelchair width) | CRITICAL | Wheelchair fits in docking area |
| EXT-CLEAR-003 | Robot SHALL have low center of gravity (CoG height <400mm) for stability on slopes | CRITICAL | CoG measured and verified |
| EXT-CLEAR-004 | Robot corners SHALL have rounded edges (radius ≥20mm) to prevent injury on contact | HIGH | Edge radius measured |

---

## 3. Weatherproofing and Environmental Protection

### 3.1 IP Rating Requirements

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| EXT-WEATHER-001 | Robot exterior SHALL achieve IP54 rating minimum (dust protected, water splash resistant) | CRITICAL | IP54 certification obtained |
| EXT-WEATHER-002 | Critical electronics enclosures SHALL achieve IP65 rating (dust-tight, water jet protected) | CRITICAL | IP65 certification for enclosures |
| EXT-WEATHER-003 | Motor housings SHALL achieve IP67 rating (dust-tight, temporary immersion) | HIGH | IP67 certification for motors |
| EXT-WEATHER-004 | All cable entry points SHALL use sealed grommets or IP-rated connectors | CRITICAL | No water ingress in spray test |
| EXT-WEATHER-005 | Battery compartment SHALL be sealed IP65 minimum (protect against rain) | CRITICAL | Battery stays dry in rain test |

**Success Criteria:** Robot operates in light rain (5mm/hour) for 2 hours without failure

### 3.2 Material Selection

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| EXT-MAT-001 | Exterior panels SHALL use UV-resistant materials (ABS plastic, powder-coated aluminum) | HIGH | UV testing shows <5% degradation after 1000 hours |
| EXT-MAT-002 | Chassis frame SHALL use corrosion-resistant materials (stainless steel, anodized aluminum) | CRITICAL | Corrosion testing passes |
| EXT-MAT-003 | Exterior screws/fasteners SHALL be stainless steel or zinc-coated (prevent rust) | HIGH | No rust after salt spray test |
| EXT-MAT-004 | Plastic components SHALL be impact-resistant (withstand 10J impact without cracking) | HIGH | Impact test passed |
| EXT-MAT-005 | Rubber/silicone seals SHALL be rated for -10°C to +50°C operating temperature | HIGH | Seals remain flexible at temperature extremes |

### 3.3 Environmental Operating Range

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| EXT-ENV-001 | Robot SHALL operate in temperature range: -5°C to +45°C | CRITICAL | Operation verified at extremes |
| EXT-ENV-002 | Robot SHALL operate in humidity range: 10% to 90% RH (non-condensing) | HIGH | Operation verified at humidity extremes |
| EXT-ENV-003 | Robot SHALL withstand wind speeds up to 15 m/s (54 km/h, strong breeze) | MEDIUM | Stability verified in wind tunnel |
| EXT-ENV-004 | Robot SHALL operate in light rain up to 5mm/hour precipitation | HIGH | Rain chamber testing passed |
| EXT-ENV-005 | Robot SHALL survive (not operate) in temperature range: -20°C to +60°C | MEDIUM | Survival testing passed |

---

## 4. Exterior Lighting Requirements

### 4.1 Status LED Indicators

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| EXT-LIGHT-001 | Robot SHALL have front-facing status LED strip (RGB, 300mm wide, visible from 10m) | CRITICAL | LED visible at 10m in daylight |
| EXT-LIGHT-002 | Status LED SHALL display patterns: green (ready), blue (navigating), yellow (caution), red (error), flashing red (emergency) | CRITICAL | All patterns display correctly |
| EXT-LIGHT-003 | Status LED brightness SHALL be adjustable: 100% (daylight), 30% (twilight), 10% (night) | HIGH | Brightness adjustment functional |
| EXT-LIGHT-004 | LED strip SHALL be IP65 rated (protected from rain) | CRITICAL | LED waterproof verified |

**LED Pattern Definitions:**
- **Solid Green:** Idle, ready for mission
- **Solid Blue:** Navigating to destination
- **Pulsing Blue:** Docking in progress
- **Solid Yellow:** Caution (obstacle detected, waiting)
- **Solid Red:** Error (needs attention)
- **Flashing Red (1Hz):** Emergency stop active

### 4.2 Safety Lighting

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| EXT-LIGHT-SAFE-001 | Robot SHALL have front white running lights (2×, 500 lumens each, visible from 50m) | HIGH | Lights visible at 50m |
| EXT-LIGHT-SAFE-002 | Robot SHALL have rear red lights (2×, constant when moving, visible from 50m) | HIGH | Rear lights visible at 50m |
| EXT-LIGHT-SAFE-003 | Robot SHALL have amber turn signal lights (left/right, flash 1Hz when turning) | MEDIUM | Turn signals flash correctly |
| EXT-LIGHT-SAFE-004 | Robot SHALL have emergency flashers (all amber lights flash 2Hz) when emergency stop active | CRITICAL | Flashers activate on e-stop |
| EXT-LIGHT-SAFE-005 | All safety lights SHALL be automotive-grade (vibration-resistant, IP67) | HIGH | Lights survive vibration test |

### 4.3 Interior Lighting (for nighttime use)

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| EXT-LIGHT-INT-001 | Robot SHALL have downward-facing path lights (illuminate ground 2m ahead, 200 lumens) | MEDIUM | Path illuminated at night |
| EXT-LIGHT-INT-002 | Path lights SHALL activate automatically in low-light conditions (<50 lux) | MEDIUM | Auto-activation functional |
| EXT-LIGHT-INT-003 | Interior (under robot) lights SHALL be warm white LED (3000K) for comfort | LOW | Color temperature verified |

---

## 5. Branding and Identification

### 5.1 Visual Identity

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| EXT-BRAND-001 | Robot exterior SHALL display facility logo/branding on front and side panels (300mm × 150mm area) | MEDIUM | Logo placement verified |
| EXT-BRAND-002 | Robot SHALL display vehicle ID number on front, rear, and sides (minimum 80mm height, high-contrast) | CRITICAL | Vehicle ID clearly visible from 10m |
| EXT-BRAND-003 | Logo/branding SHALL be applied via vinyl decal or UV-printed directly on panels | MEDIUM | Branding durable (survives 6 months outdoor) |
| EXT-BRAND-004 | Robot color scheme SHALL use friendly, non-threatening colors (avoid black/military colors) | MEDIUM | Color scheme approved by facility |

**Recommended Color Schemes:**
- Option 1: White body with blue accents (healthcare theme)
- Option 2: Light gray body with green accents (eco-friendly theme)
- Option 3: Beige body with orange accents (warm, approachable theme)

### 5.2 QR Code and Information Display

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| EXT-QR-001 | Robot SHALL display QR code on front and rear panels (100mm × 100mm, links to vehicle info) | HIGH | QR code scannable from 1m |
| EXT-QR-002 | QR code SHALL encode: vehicle ID, fleet URL, emergency contact number | HIGH | QR code data correct |
| EXT-QR-003 | Robot SHALL display emergency contact phone number in large text (minimum 60mm height) | CRITICAL | Phone number visible from 5m |
| EXT-QR-004 | Emergency contact info SHALL be displayed in both English and Japanese | HIGH | Bilingual text present |

---

## 6. External Display and eHMI Requirements

### 6.1 Front Display Screen

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| EXT-DISPLAY-001 | Robot SHALL have front-facing LCD display (7-inch minimum, 1024×600 resolution) | HIGH | Display readable in sunlight |
| EXT-DISPLAY-002 | Display SHALL show robot status text: "Ready", "In Service", "Out of Service", "Emergency Stop" | HIGH | Status messages displayed |
| EXT-DISPLAY-003 | Display SHALL show friendly graphics (smiley face, icons) to indicate robot state | MEDIUM | Graphics displayed correctly |
| EXT-DISPLAY-004 | Display brightness SHALL auto-adjust based on ambient light (500 nits max daylight, 50 nits night) | MEDIUM | Auto-brightness functional |
| EXT-DISPLAY-005 | Display SHALL be impact-resistant (polycarbonate cover, withstand 5J impact) | HIGH | Impact test passed |
| EXT-DISPLAY-006 | Display SHALL be IP65 rated (waterproof for outdoor use) | CRITICAL | Display waterproof verified |

### 6.2 eHMI Audio Output

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| EXT-AUDIO-001 | Robot SHALL have external speaker (5W minimum, 85 dB @ 1m) for status announcements | HIGH | Audio audible at 5m |
| EXT-AUDIO-002 | Speaker SHALL play sounds: startup chime, movement warning beep, emergency tone | HIGH | All sounds play correctly |
| EXT-AUDIO-003 | Audio volume SHALL be adjustable (quiet indoor mode, loud outdoor mode) | MEDIUM | Volume adjustment functional |
| EXT-AUDIO-004 | Robot SHALL announce in Japanese: "動きます" (Moving), "停止します" (Stopping) | HIGH | Japanese audio clear |
| EXT-AUDIO-005 | Speaker SHALL be weatherproof (IP65 rating) | HIGH | Speaker waterproof verified |

---

## 7. Physical Access and Maintenance

### 7.1 Panel Access

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| EXT-ACCESS-001 | Battery compartment SHALL be accessible via tool-free latches (no screws required) | HIGH | Battery removal without tools |
| EXT-ACCESS-002 | Top panel SHALL open to access compute unit and electronics (4 quarter-turn fasteners) | MEDIUM | Top panel opens easily |
| EXT-ACCESS-003 | All exterior panels SHALL be removable for maintenance (standard hex bolts, M6) | MEDIUM | Panels removable with hex key |
| EXT-ACCESS-004 | Cable routing SHALL allow easy replacement of wiring harnesses | MEDIUM | Cables replaceable |

### 7.2 Emergency Access

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| EXT-EMERG-001 | Emergency stop button SHALL be accessible from front, rear, and sides (3× buttons minimum) | CRITICAL | E-stop accessible from all sides |
| EXT-EMERG-002 | Emergency stop buttons SHALL be large (80mm diameter), red, mushroom-head style | CRITICAL | E-stop buttons meet standards |
| EXT-EMERG-003 | Emergency battery disconnect SHALL be accessible via external key switch | HIGH | Battery disconnect functional |

---

## 8. Ergonomics and User Interaction

### 8.1 Physical Interaction Points

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| EXT-ERGO-001 | Robot SHALL have physical handle/handhold on sides for manual pushing (emergency) | MEDIUM | Handle supports 20kg push force |
| EXT-ERGO-002 | Handle height SHALL be 900-1100mm (comfortable for standing adult) | MEDIUM | Handle height measured |
| EXT-ERGO-003 | Robot SHALL have clearly marked "Push Here" labels near handles (bilingual) | LOW | Labels visible |

### 8.2 Wheelchair Interface

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| EXT-WHEEL-001 | Wheelchair docking mechanism SHALL be clearly marked with alignment guides (yellow chevrons) | HIGH | Guides visible from 5m |
| EXT-WHEEL-002 | Docking area SHALL have visual status indicator (green=ready, red=not ready) | HIGH | Status indicator functional |
| EXT-WHEEL-003 | Docking mechanism SHALL have physical stops to prevent over-insertion | CRITICAL | Stops prevent over-insertion |

---

## 9. Safety Markings and Labels

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| EXT-LABEL-001 | Robot SHALL display "CAUTION: AUTONOMOUS VEHICLE" in English and Japanese on all sides | CRITICAL | Warning labels present |
| EXT-LABEL-002 | Robot SHALL have reflective strips on corners (front, rear, sides) for nighttime visibility | HIGH | Reflective strips visible at 50m |
| EXT-LABEL-003 | Battery compartment SHALL have "HIGH VOLTAGE" warning label (48V) | CRITICAL | Warning label present |
| EXT-LABEL-004 | All labels SHALL be waterproof and UV-resistant (survive 5 years outdoor) | HIGH | Label durability tested |
| EXT-LABEL-005 | Robot SHALL display maximum speed limit label (1.0 m/s with passenger) | MEDIUM | Speed limit label present |

---

## 10. Aesthetics and Design Language

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| EXT-AEST-001 | Robot design SHALL convey friendliness and approachability (rounded shapes, soft edges) | MEDIUM | Design review approved |
| EXT-AEST-002 | Robot SHALL NOT resemble military or industrial equipment (avoid angular, aggressive styling) | MEDIUM | Design review approved |
| EXT-AEST-003 | Exterior surfaces SHALL be smooth and easy to clean (no dirt traps) | MEDIUM | Surface cleanability verified |
| EXT-AEST-004 | Panel gaps SHALL be ≤5mm (professional appearance) | LOW | Gap measurements taken |

---

## 11. Mounting Points and Attachments

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| EXT-MOUNT-001 | Robot top surface SHALL have T-slot mounting rails for sensor accessories (LiDAR, cameras) | HIGH | Mounting rails functional |
| EXT-MOUNT-002 | Mounting rails SHALL support up to 10kg distributed load | HIGH | Load test passed |
| EXT-MOUNT-003 | Robot SHALL have mounting points for optional rain cover/canopy | LOW | Mounting points present |
| EXT-MOUNT-004 | Sensor mounting points SHALL provide cable routing channels to interior | MEDIUM | Cable routing functional |

---

## 12. Cleaning and Maintenance

| ID | Requirement | Priority | Acceptance Criteria |
|----|-------------|----------|---------------------|
| EXT-CLEAN-001 | Exterior surfaces SHALL be cleanable with mild detergent and water | MEDIUM | Cleaning does not damage finish |
| EXT-CLEAN-002 | Robot SHALL have drain holes in bottom panels to prevent water accumulation | HIGH | Water drains completely |
| EXT-CLEAN-003 | Exterior finish SHALL resist common cleaning agents (isopropyl alcohol, bleach solution) | MEDIUM | Finish undamaged by cleaners |

---

## 13. Requirements Summary

| Category | Count | Priority Breakdown |
|----------|-------|-------------------|
| Physical Dimensions and Clearances | 9 | Critical: 5, High: 4 |
| Weatherproofing and Environmental | 18 | Critical: 9, High: 7, Medium: 2 |
| Exterior Lighting | 13 | Critical: 4, High: 7, Medium: 1, Low: 1 |
| Branding and Identification | 8 | Critical: 2, High: 3, Medium: 3 |
| External Display and eHMI | 11 | Critical: 1, High: 7, Medium: 3 |
| Physical Access and Maintenance | 7 | Critical: 2, High: 3, Medium: 2 |
| Ergonomics and User Interaction | 6 | Critical: 1, High: 2, Medium: 2, Low: 1 |
| Safety Markings and Labels | 5 | Critical: 2, High: 2, Medium: 1 |
| Aesthetics and Design Language | 4 | Medium: 3, Low: 1 |
| Mounting Points and Attachments | 4 | High: 2, Medium: 1, Low: 1 |
| Cleaning and Maintenance | 3 | High: 1, Medium: 2 |
| **TOTAL** | **88** | **Critical: 26, High: 38, Medium: 20, Low: 4** |

---

## 14. Traceability Matrix

| Exterior Requirement Category | Related Hardware Components |
|------------------------------|----------------------------|
| Weatherproofing | All electronic enclosures, battery compartment, motor housings |
| Lighting | 48V→12V DC-DC converter, eHMI ESP32 controller |
| Display and eHMI | Jetson Orin Nano, ESP32 via UART |
| Emergency Stop | Safety controller, CAN bus emergency broadcast |

---

## 15. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-16 | Tsuchiya Team | Initial exterior and physical design requirements (88 requirements) |

---

**Document End**
