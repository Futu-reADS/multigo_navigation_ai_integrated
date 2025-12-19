# Mechanical System Architecture

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System  
**Document Type:** Hardware Architecture Specification
**Team:** Tsuchiya (Hardware)
**Date:** December 16, 2025
**Version:** 1.0

---

## Overall Mechanical Layout

```
                    Top View
        ┌──────────────────────────┐
        │    Front (Docking Side)  │
        │                          │
        │  [Camera] [LiDAR]        │
        │     ┌──────────┐         │
        │     │ Compute  │         │
        │     │ Battery  │         │
        │     └──────────┘         │
        │  ⚙️       ⚙️              │ 
        │  Motor   Motor           │
        │                          │
        │  ⚙️       ⚙️              │
        │  Motor   Motor           │
        │                          │
        │    Rear (Charging Port)  │
        └──────────────────────────┘
```

## Chassis Structure

**Frame Material:** Aluminum 6061-T6 extrusion (40mm × 40mm)
- Lightweight: 15kg frame weight
- High strength-to-weight ratio
- Corrosion-resistant (anodized finish)
- Modular design (bolted assembly)

**Dimensions:**
- Length: 1200mm
- Width: 900mm
- Height: 600mm (without wheelchair)
- Ground clearance: 150mm
- Wheelbase: 800mm
- Track width: 700mm

**Weight Distribution:**
- Front: 45% (sensors, compute unit)
- Rear: 55% (battery, charging port)
- Center of Gravity: 350mm height, centered laterally
- Total weight: 150kg (including battery)

## Swerve Drive Module (×4)

### Module Components

```
     ┌─────────────┐
     │ Steer Motor │ (12V, 50W, encoder)
     └──────┬──────┘
            │ Belt drive (3:1 reduction)
            ▼
     ┌─────────────┐
     │ Swivel Plate│ (Bearing-mounted)
     └──────┬──────┘
            │ 
            ▼
     ┌─────────────┐
     │ Drive Motor │ (48V, 200W, in-wheel)
     │  + Wheel    │ (6.5" diameter)
     └─────────────┘
```

**Specifications per module:**
- Drive motor: 200W, 48V, hall-effect encoder
- Steer motor: 50W, 12V, absolute encoder
- Wheel diameter: 165mm (6.5")
- Tire: Pneumatic, outdoor tread pattern
- Swivel bearing: Deep-groove ball bearing, 30mm ID
- Steering range: 360° continuous rotation
- Max torque (drive): 15 Nm
- Max speed: 2 m/s (no load)

## Wheelchair Docking Mechanism

### Mechanical Design

```
Side View (Rear of Robot)
┌──────────────────────────────────┐
│  Robot Chassis                   │
│                                  │
│         ┌────────────┐           │
│         │  Docking   │           │
│         │   Arms     │◄──────────┼─ Servo actuators
│         │  (folded)  │           │
│         └────────────┘           │
│              │                   │
│              ▼                   │
│         [Engagement              │
│          Mechanism]              │
│              │                   │
└──────────────┼───────────────────┘
               │
        Wheelchair caster slots
```

**Components:**
- 2× Folding arms (aluminum, servo-actuated)
- Engagement mechanism: Spring-loaded pins
- Alignment guides: Tapered funnels (±50mm tolerance)
- Sensors: Hall-effect switches (engagement confirmation)
- ArUco marker: 100mm × 100mm, mounted at 500mm height

**Docking Sequence:**
1. Visual alignment (ArUco marker detection)
2. Arms unfold (servo actuation, 2s)
3. Robot reverses to engagement position
4. Spring pins engage wheelchair caster slots
5. Confirmation via hall-effect sensors

## Sensor Mounting

### LiDAR (2D, front-mounted)
- Model: SICK TiM571 or similar
- Mounting height: 400mm
- Mounting angle: 0° (horizontal)
- Field of view: 270°
- Vibration isolation: Rubber dampers

### Cameras (×3)
- Front camera: 1920×1080, 90° FOV, 1200mm height
- Rear camera: 1280×720, 120° FOV (fisheye), 800mm height
- Docking camera: 1280×720, 60° FOV, 500mm height (ArUco detection)
- Mounting: Adjustable brackets, IP65 enclosures

### IMU
- Mounting: Chassis center, rigidly bolted
- Orientation: Aligned with robot coordinate frame

## Enclosures

### Compute Unit Enclosure
- Material: Powder-coated steel, IP54
- Dimensions: 400mm × 300mm × 200mm
- Mounting: Vibration-isolated (rubber grommets)
- Cooling: Forced-air (40mm fan), filtered intake
- Access: Hinged door, quarter-turn fasteners

### Battery Enclosure
- Material: Aluminum, IP65
- Dimensions: 500mm × 350mm × 150mm
- Mounting: Center-mounted, low CoG
- Cooling: Passive (finned aluminum)
- Access: Tool-free latches (quick battery swap)
- Fire suppression: Thermal fuse (85°C)

### External Display Enclosure
- Mounting: Side panel, 1200mm height
- Material: Polycarbonate shield (3mm), IP65 gasket
- Angle: 15° upward tilt (ergonomic viewing)

## Suspension and Compliance

- No active suspension (rigid chassis)
- Compliance via pneumatic tires (40 PSI)
- Swerve modules provide some compliance (rubber bushings)
- Ground clearance: 150mm (avoids most obstacles)

## Weatherproofing

- All enclosures: IP54 minimum (IP65 for critical components)
- Cable entry: IP-rated cable glands
- Vents: Gore-Tex membrane vents (pressure equalization, moisture barrier)
- Seals: Silicone gaskets, EPDM rubber
- Drainage: Sloped floors in enclosures, drain holes at low points

## Material Summary

| Component | Material | Justification |
|-----------|----------|---------------|
| Frame | Aluminum 6061-T6 | Lightweight, strong, corrosion-resistant |
| Panels | ABS plastic or aluminum | Easy to fabricate, weather-resistant |
| Wheels | Rubber (pneumatic) | Outdoor traction, shock absorption |
| Bearings | Stainless steel | Corrosion-resistant, low friction |
| Fasteners | Stainless steel (A4) | Corrosion-resistant |
| Gaskets | Silicone/EPDM | Weather-resistant, durable |

---

**Total:** 99 requirements covered in MECHANICAL_REQUIREMENTS.md  
**Related:** EXTERIOR_REQUIREMENTS.md (88 req)

**Document End**
