# Hardware Platform - Tsuchiya + Kiril's Scope
# ハードウェアプラットフォーム - Tsuchiya + Kirilの範囲

**Team Leads:**
- **Tsuchiya** - Mechanical, Electrical, Sensors, Power
- **Kiril** - Exterior, eHMI Hardware, Aesthetics

**Scope:** Physical Hardware Platform ONLY
**Timeline:** 16 weeks (5 weeks prototype, 11 weeks complete platform)
**Status:** Ready for implementation

---

## 🎯 What You Build

### Tsuchiya's Scope (Mechanical + Electrical + Sensors)

**Mechanical System:**
- ✅ **Chassis Design** - Welded aluminum frame, load capacity 100kg
- ✅ **Swerve Drive Modules** - 4x independent steering and drive modules
- ✅ **Suspension System** - For outdoor rough terrain
- ✅ **Docking Mechanism** - Physical attachment to wheelchair
- ✅ **Sensor Mounting** - LiDAR, cameras, IMU, bumpers
- ✅ **Cable Management** - Internal routing, weatherproof connectors

**Electrical System:**
- ✅ **Power Distribution** - 48V bus, DC-DC converters (12V, 5V)
- ✅ **Battery Management System (BMS)** - 48V 20Ah LiFePO4 with CAN interface
- ✅ **Motor Controllers** - 4x drive + 4x steering (CAN bus)
- ✅ **Compute Platform** - GMKtec Nucbox K6 (AMD Ryzen 7 7840HS, 32GB RAM)
- ✅ **Safety Systems** - Hardware E-stop, bumpers, watchdog
- ✅ **Charging System** - 48V charging port

**Sensor Integration:**
- ✅ **3D LiDAR** - Outdoor-grade IP65+, USB/Ethernet interface
- ✅ **Cameras** - 2x RGB cameras (ArUco detection, obstacle awareness)
- ✅ **IMU** - 9-DOF (MPU-9250 or similar)
- ✅ **Encoders** - 4x drive encoders + 4x steer encoders
- ✅ **Bumpers** - Contact sensors for collision detection

### Kiril's Scope (Exterior + eHMI)

**Exterior Design:**
- ✅ **Body Panels** - Weatherproof (IP54+), aesthetic design
- ✅ **Lighting** - Headlights, taillights, turn signals
- ✅ **Protective Covers** - LiDAR cover, camera covers
- ✅ **Branding** - Logo, color scheme, visual identity

**eHMI Hardware:**
- ✅ **ESP32-S3 Controller** - Microcontroller for LED control
- ✅ **LED System** - WS2812B LED strips (2-3 meters, 60+ LEDs/meter)
- ✅ **Audio System** - I2S amplifier (MAX98357A), speakers (5W+, weatherproof)
- ✅ **Display (Optional)** - Small OLED for status messages
- ✅ **Serial Communication** - UART to main compute

---

## ❌ What You DON'T Build

- ❌ **Vehicle Software** (Pankaj's responsibility)
  - Not ROS 2 nodes, navigation algorithms, control software
  - Not UI, not docking controller, not perception

- ❌ **TVM Server** (Unno's responsibility)
  - Not backend server, database, fleet dashboard
  - Not server-side logic

**Your Focus:** Physical hardware that Pankaj's software controls

---

## 🔗 Your Interface

### Interface to Vehicle Software (Pankaj)

**Document (CRITICAL):**
- `04_INTERFACES/HARDWARE_SOFTWARE_INTERFACE.md` - **YOU MUST PROVIDE THIS**

**What Pankaj Needs from You:**

**ROS 2 Topics (You Publish):**
- `/sensors/lidar/points` - 3D LiDAR point cloud (10 Hz)
- `/sensors/camera/front/image` - Front camera (10 Hz)
- `/sensors/camera/rear/image` - Rear camera (10 Hz)
- `/sensors/imu/data` - IMU (50 Hz)
- `/sensors/battery/status` - Battery (1 Hz)
- `/safety/bumpers` - Bumper contact status (50 Hz)
- `/safety/estop` - E-stop status (50 Hz)

**ROS 2 Topics (You Subscribe):**
- `/control/cmd_vel` - Velocity commands from software
- `/control/swerve/drive_velocities` - Drive motor speeds (4 motors)
- `/control/swerve/steer_angles` - Steering angles (4 motors)

**CAN Bus:**
- Motor controllers respond to commands
- BMS publishes battery data

**Serial (eHMI):**
- ESP32 receives LED/sound commands from software
- ESP32 sends status back to software

---

## 🛠️ Hardware Specifications

### Compute Platform

**Model:** GMKtec Nucbox K6
- **CPU:** AMD Ryzen 7 7840HS (8 cores, 16 threads, 3.8-5.1 GHz)
- **RAM:** 32GB DDR5
- **GPU:** AMD Radeon 780M (integrated)
- **Storage:** 1TB NVMe SSD
- **OS:** Ubuntu 22.04 LTS (installed by Pankaj)
- **Ports:** USB 3.2, USB-C, Ethernet, HDMI

### Power System

**Battery:**
- **Type:** LiFePO4 (safer, longer life)
- **Voltage:** 48V nominal
- **Capacity:** 20Ah (960Wh)
- **BMS:** CAN bus interface, over-current/over-voltage protection
- **Runtime:** ≥4 hours with passenger

**Power Distribution:**
- 48V → Motor controllers (direct)
- 48V → 12V (DC-DC converter, 20A) → Compute, sensors
- 48V → 5V (DC-DC converter, 10A) → ESP32, minor electronics

### Motors

**Drive Motors (4x):**
- **Type:** 6.5" hoverboard in-wheel motors
- **Power:** 80W per wheel (320W total)
- **Voltage:** 48V
- **Control:** CAN bus motor controller
- **Encoders:** Hall effect sensors

**Steering Motors (4x):**
- **Option 1:** Digital servo (3.5Nm torque)
- **Option 2:** Power window motor (2.9Nm torque)
- **Control:** PWM or CAN bus

### Sensors

**3D LiDAR:**
- **Range:** 30-100 meters
- **FOV:** 360° horizontal, ±15° vertical (minimum)
- **Accuracy:** ±3cm
- **Protection:** IP65+ (outdoor-grade)
- **Interface:** Ethernet or USB
- **Cost:** $1000-3000

**Cameras (2x):**
- **Resolution:** 1920x1080 (minimum)
- **FPS:** 30 FPS
- **Interface:** USB 3.0
- **Protection:** IP54+ weatherproof housing
- **Purpose:** ArUco detection (docking), obstacle awareness

**IMU:**
- **Type:** 9-DOF (accelerometer, gyroscope, magnetometer)
- **Model:** MPU-9250 or similar
- **Interface:** I2C or SPI
- **Sample Rate:** 50 Hz minimum

**Encoders:**
- **Drive Encoders:** 4x (built into motors or separate)
- **Steer Encoders:** 4x absolute encoders for steering angle
- **Resolution:** 360 ticks/revolution minimum

**Bumpers:**
- **Type:** Contact switches or pressure sensors
- **Locations:** Front, rear, sides
- **Interface:** GPIO or CAN bus

**E-Stop:**
- **Type:** Physical emergency stop button
- **Circuit:** Hardwired to motor controllers (safety-critical)
- **Redundancy:** Dual-channel (two independent circuits)

### eHMI Hardware (Kiril)

**ESP32-S3:**
- **Purpose:** Control LEDs, speakers, display
- **Communication:** UART to main compute (115200 baud)
- **Power:** 5V from DC-DC converter

**LED System:**
- **Type:** WS2812B addressable LED strip
- **Length:** 2-3 meters
- **Density:** 60+ LEDs/meter
- **Control:** SPI from ESP32

**Audio System:**
- **Amplifier:** MAX98357A (I2S)
- **Speakers:** 5W+ weatherproof speakers
- **Audio Files:** Stored on SD card (WAV format)

---

## 📁 Folder Structure

```
tsuchiya_kiril_hardware/
├── 01_REQUIREMENTS/          ← What the hardware must do
│   ├── MECHANICAL_REQUIREMENTS.md (Tsuchiya)
│   ├── ELECTRICAL_REQUIREMENTS.md (Tsuchiya)
│   ├── COMMUNICATION_SYSTEM_REQUIREMENTS.md (Tsuchiya)
│   ├── EXTERIOR_REQUIREMENTS.md (Kiril)
│   └── CHARGING_INFRASTRUCTURE_REQUIREMENTS.md (Tsuchiya)
│
├── 02_ARCHITECTURE/          ← How it's organized
│   ├── MECHANICAL_SYSTEM_ARCHITECTURE.md
│   ├── ELECTRICAL_SYSTEM_ARCHITECTURE.md
│   └── CHARGING_SYSTEM_ARCHITECTURE.md
│
├── 03_DESIGN/                ← Detailed designs
│   ├── MECHANICAL_CAD_DESIGN.md (CAD models, drawings)
│   ├── ELECTRICAL_SCHEMATICS.md (Wiring diagrams)
│   └── PCB_DESIGN.md (Custom PCBs if any)
│
├── 04_INTERFACES/            ← Interface to software (Pankaj)
│   └── HARDWARE_SOFTWARE_INTERFACE.md ← PROVIDE THIS (critical!)
│
├── 05_DEVELOPMENT/           ← How to build
│   └── HARDWARE_DEVELOPMENT_GUIDE.md
│
└── README.md                 ← This file
```

---

## 🚀 Quick Start

### 1. Read Interface Document First

**CRITICAL - Read this before building:**
1. `04_INTERFACES/HARDWARE_SOFTWARE_INTERFACE.md` - What Pankaj needs from you

**Key sections:**
- ROS 2 topics you must publish (sensors)
- ROS 2 topics you must subscribe to (commands)
- CAN bus protocol
- Serial protocol (eHMI)

### 2. Review Requirements

Start with these to understand what to build:
1. `01_REQUIREMENTS/MECHANICAL_REQUIREMENTS.md` (Tsuchiya)
2. `01_REQUIREMENTS/ELECTRICAL_REQUIREMENTS.md` (Tsuchiya)
3. `01_REQUIREMENTS/EXTERIOR_REQUIREMENTS.md` (Kiril)

### 3. Begin Implementation

**Suggested order:**

**Phase 1: Fast Prototype (5 weeks)**
- **Week 1-2:** Select sensors, define architecture, procure parts
- **Week 3-4:** Assemble basic chassis, install drive motors
- **Week 5:** Add LiDAR + cameras + CAN communication

**Milestone:** "Walking Skeleton" - Platform can drive, sensors publishing data

**Phase 2: Refinement (11 weeks)**
- **Week 6-8:** Add steering motors (complete swerve drive)
- **Week 9-10:** Install suspension system
- **Week 11-12:** Add remaining sensors (bumpers, IMU)
- **Week 13-14:** Implement docking mechanism
- **Week 15:** Complete control system integration
- **Week 16:** Final weatherproofing

**Phase 3: Exterior (Parallel with Kiril)**
- **Week 1-8:** Exterior CAD design
- **Week 9-12:** Fabricate body panels
- **Week 13-16:** eHMI hardware assembly

---

## 🔧 Assembly Checklist

### Mechanical (Tsuchiya)

- [ ] Chassis frame welded/assembled
- [ ] Swerve drive modules installed (4x)
- [ ] Suspension system installed
- [ ] Docking mechanism installed
- [ ] Sensor mounting brackets installed
- [ ] Cable routing complete
- [ ] Ground clearance ≥20mm verified

### Electrical (Tsuchiya)

- [ ] Battery installed and secured
- [ ] BMS installed and configured
- [ ] Power distribution wired
- [ ] DC-DC converters installed (48V→12V, 48V→5V)
- [ ] Motor controllers installed (8x)
- [ ] CAN bus wired and terminated
- [ ] Compute platform installed
- [ ] E-stop circuit wired and tested
- [ ] All connectors weatherproofed

### Sensors (Tsuchiya)

- [ ] 3D LiDAR mounted and connected
- [ ] Front camera mounted and connected
- [ ] Rear camera mounted and connected
- [ ] IMU installed and calibrated
- [ ] Drive encoders installed (4x)
- [ ] Steer encoders installed (4x)
- [ ] Bumpers installed
- [ ] Battery monitoring connected

### eHMI (Kiril)

- [ ] ESP32-S3 installed
- [ ] LED strips installed and wired
- [ ] Speakers installed
- [ ] Audio amplifier installed
- [ ] Serial connection to compute
- [ ] Power supply verified (5V)
- [ ] eHMI firmware uploaded (by Pankaj)

### Exterior (Kiril)

- [ ] Body panels installed
- [ ] Headlights/taillights installed
- [ ] LiDAR protective cover installed
- [ ] Camera covers installed
- [ ] Branding/logo applied
- [ ] IP54+ weatherproofing verified

---

## 🧪 Testing & Validation

### Unit Testing (Component-Level)

**Power System:**
- [ ] Battery voltage correct (48V ±2V)
- [ ] DC-DC converters output correct (12V, 5V)
- [ ] BMS reporting data via CAN
- [ ] Charging system working

**Motors:**
- [ ] All 4 drive motors respond to CAN commands
- [ ] All 4 steering motors respond to commands
- [ ] Encoders reporting correct values
- [ ] No overheating under load

**Sensors:**
- [ ] LiDAR publishing point clouds
- [ ] Cameras capturing images
- [ ] IMU reporting orientation
- [ ] Bumpers detecting contact
- [ ] E-stop functioning correctly

**eHMI:**
- [ ] LEDs controllable via serial commands
- [ ] Speakers playing audio
- [ ] Status messages sent back to compute

### Integration Testing (with Pankaj's Software)

**Coordination with Pankaj:**
- [ ] ROS 2 topics verified (correct message types, frequencies)
- [ ] CAN bus commands working
- [ ] Serial protocol tested
- [ ] Latency measured (<100ms for safety-critical)

### Safety Testing

- [ ] E-stop stops all motors <100ms
- [ ] Bumpers trigger emergency stop
- [ ] Battery BMS protects against over-current
- [ ] Thermal shutdown working
- [ ] Failsafe behavior verified

### Environmental Testing

- [ ] IP54+ weatherproofing verified (water spray test)
- [ ] Outdoor operation tested (rain, sun, wind)
- [ ] Temperature range tested (-10°C to +40°C)
- [ ] Vibration testing (rough terrain)

---

## 📋 Dependencies

### From Pankaj (Software Team)

**What you need from him:**
- [ ] ROS 2 topic definitions (HARDWARE_SOFTWARE_INTERFACE.md)
- [ ] Control command formats (velocity, steering)
- [ ] eHMI serial protocol specification
- [ ] Ubuntu 22.04 + ROS 2 Humble installed on compute
- [ ] ROS 2 driver nodes for sensors

**Coordination:**
- Review `shared/INTERFACES/HARDWARE_SOFTWARE_INTERFACE.md` together
- Test each interface incrementally
- Weekly integration meetings

### From Unno (TVM Server Team)

**No direct dependencies** - communicate via Pankaj's software

---

## ⚠️ Important Notes

### Safety is Critical

**This robot transports people:**
- Hardware E-stop must be fail-safe (dual-channel)
- Bumpers must trigger emergency stop
- Battery must have over-current protection
- All safety circuits must be tested and validated

### Interface Contract is Frozen

**DO NOT change the interface without coordinating with Pankaj:**
- ROS topic names, message types, frequencies are defined
- CAN bus protocol is defined
- Serial protocol is defined

**If you need interface changes:**
1. Discuss with Pankaj
2. Update HARDWARE_SOFTWARE_INTERFACE.md (both agree)
3. Test updated interface

### Weatherproofing

**Robot must operate outdoors:**
- All electronics IP54+ rated (dust + water spray protection)
- All connectors weatherproof (automotive-grade)
- Battery protected from rain
- Sensors protected (but functional)

### Cable Management

**Professional cable routing:**
- Internal routing (not external dangling cables)
- Strain relief at all connectors
- Color-coded wires (power, signal, ground)
- Labeled connectors for maintenance

---

## 📞 Contacts

**Your Team:**
- **Tsuchiya** - Mechanical, Electrical, Sensors, Power
- **Kiril** - Exterior, eHMI Hardware

**Software Team:** Pankaj
- **Interface:** HARDWARE_SOFTWARE_INTERFACE.md
- **Coordination:** Weekly integration meetings
- **Testing:** Integration testing after prototype complete

**Server Team:** Unno
- **No direct interface** - communicate via Pankaj's software

**Senior:** [Name]
- **Approval:** Final hardware design and procurement

---

## ✅ Success Criteria

### Prototype (Week 5)

- [ ] Platform can drive forward/backward
- [ ] LiDAR publishing data
- [ ] Cameras publishing images
- [ ] Power system functional
- [ ] Pankaj can begin software development on real hardware

### Complete Platform (Week 16)

- [ ] Swerve drive fully functional (omnidirectional motion)
- [ ] All sensors operational
- [ ] Docking mechanism working
- [ ] eHMI functional (LEDs, speakers)
- [ ] Weatherproof (IP54+)
- [ ] Safety systems validated

### Integration with Software

- [ ] All ROS topics working correctly
- [ ] Control latency <100ms
- [ ] No communication errors
- [ ] Safety systems tested with software

---

**Document Version:** 1.0
**Last Updated:** December 19, 2025
**Status:** Ready for implementation
**Next Action:** Wait for HARDWARE_REQUIREMENTS_FOR_TSUCHIYA.md from Pankaj

---

**Happy Building! 🔧**
