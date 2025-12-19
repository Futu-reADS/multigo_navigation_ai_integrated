# Hardware Requirements Package for Tsuchiya + Kiril

**Document Type:** Implementation Guide
**Target Audience:** Tsuchiya (Mechanical/Electrical/Sensors) + Kiril (Exterior/eHMI)
**Status:** Active
**Version:** 1.0
**Date:** December 19, 2025
**Prepared By:** Pankaj (Vehicle Software Team)

---

## Table of Contents

1. [Quick Start](#1-quick-start)
2. [What You Must Provide](#2-what-you-must-provide)
3. [Hardware-Software Interface](#3-hardware-software-interface)
4. [Component Specifications](#4-component-specifications)
5. [Assembly Guide](#5-assembly-guide)
6. [Testing Procedures](#6-testing-procedures)
7. [Integration with Software](#7-integration-with-software)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. Quick Start

### 1.1 Your Scope

**Tsuchiya (Mechanical + Electrical + Sensors):**
- ✅ Chassis design and fabrication
- ✅ Swerve drive modules (4x steering + 4x drive motors)
- ✅ Power system (48V battery, DC-DC converters, BMS)
- ✅ Sensor installation (LiDAR, cameras, IMU, encoders, bumpers)
- ✅ Compute platform installation (GMKtec Nucbox K6)
- ✅ Cable management and weatherproofing

**Kiril (Exterior + eHMI):**
- ✅ Body panels and exterior design
- ✅ eHMI hardware (ESP32, LEDs, speakers)
- ✅ Lighting (headlights, taillights, turn signals)
- ✅ Protective covers (LiDAR, cameras)

**Out of Scope:**
- ❌ Vehicle software (Pankaj's responsibility)
- ❌ TVM server (Unno's responsibility)
- ❌ Software configuration (Pankaj's responsibility)

### 1.2 What Pankaj Needs from You

**Critical deliverables:**
1. **ROS 2 topics publishing sensor data** (see Section 3.2)
2. **ROS 2 topics subscribing to motor commands** (see Section 3.3)
3. **CAN bus for motor control** (see Section 3.4)
4. **Serial interface for eHMI** (see Section 3.5)
5. **E-stop hardware circuit** (see Section 4.7)

**Timeline:** 16 weeks (5 weeks prototype + 11 weeks complete)

---

## 2. What You Must Provide

### 2.1 Physical Platform (Tsuchiya)

**Mechanical System:**
- [x] Welded aluminum chassis frame
- [x] 4x swerve drive modules (independent steering + drive)
- [x] Suspension system (for outdoor rough terrain)
- [x] Docking mechanism (physical attachment to wheelchair)
- [x] Sensor mounting brackets (LiDAR, cameras, IMU)
- [x] Cable routing (internal, weatherproof)

**Electrical System:**
- [x] 48V 20Ah LiFePO4 battery + BMS
- [x] Power distribution (48V bus, DC-DC converters to 12V, 5V)
- [x] 4x drive motor controllers (CAN bus)
- [x] 4x steering motor controllers (PWM or CAN bus)
- [x] Compute platform (GMKtec Nucbox K6) installed
- [x] Hardware E-stop circuit (dual-channel)
- [x] Charging port (48V)

**Sensors:**
- [x] 3D LiDAR (outdoor-grade IP65+, 30-100m range, USB/Ethernet)
- [x] 2x RGB cameras (1920x1080, 30 FPS, USB 3.0)
- [x] IMU (9-DOF: MPU-9250 or similar, I2C/SPI)
- [x] 4x drive encoders (360 ticks/rev minimum)
- [x] 4x steer encoders (absolute, for steering angle)
- [x] Contact bumpers (front, rear, sides - GPIO or CAN)

### 2.2 Exterior & eHMI (Kiril)

**Exterior:**
- [x] Weatherproof body panels (IP54+)
- [x] Headlights and taillights
- [x] Turn signals
- [x] LiDAR protective cover (transparent/mesh)
- [x] Camera protective covers

**eHMI Hardware:**
- [x] ESP32-S3 microcontroller
- [x] WS2812B LED strips (2-3 meters, 60+ LEDs/meter)
- [x] I2S audio amplifier (MAX98357A)
- [x] Weatherproof speakers (5W+)
- [x] Serial connection (UART) to main compute

---

## 3. Hardware-Software Interface

### 3.1 Interface Document

**CRITICAL:** Read this first!
- **File:** `shared/INTERFACES/HARDWARE_SOFTWARE_INTERFACE.md`
- **Length:** 1,010 lines
- **Status:** ✅ Complete, production-ready

**This is the contract between you and Pankaj.**
- DO NOT change topics/protocols without coordinating with Pankaj
- All ROS 2 topic names, message types, frequencies are defined
- CAN bus protocol is specified
- Serial protocol for eHMI is specified

### 3.2 ROS 2 Topics You Must Publish (Sensor Data)

**Your hardware drivers must publish these:**

```yaml
# LiDAR point cloud
Topic: /sensors/lidar/points
Type: sensor_msgs/msg/PointCloud2
Frequency: 10 Hz
Publisher: LiDAR driver (you provide)

# Front camera image
Topic: /sensors/camera/front/image
Type: sensor_msgs/msg/Image
Frequency: 10 Hz
Publisher: Camera driver (you provide)

# Rear camera image
Topic: /sensors/camera/rear/image
Type: sensor_msgs/msg/Image
Frequency: 10 Hz
Publisher: Camera driver (you provide)

# IMU data
Topic: /sensors/imu/data
Type: sensor_msgs/msg/Imu
Frequency: 50 Hz
Publisher: IMU driver (you provide)

# Battery status
Topic: /sensors/battery/status
Type: sensor_msgs/msg/BatteryState
Frequency: 1 Hz
Publisher: BMS driver via CAN (you provide)

# Bumper contact status
Topic: /safety/bumpers
Type: std_msgs/msg/Bool (or custom message)
Frequency: 50 Hz
Publisher: Bumper GPIO reader (you provide)

# E-stop status
Topic: /safety/estop
Type: std_msgs/msg/Bool
Frequency: 50 Hz
Publisher: E-stop GPIO reader (you provide)
```

**How to implement:**
- Install ROS 2 Humble on compute platform
- Write ROS 2 driver nodes (C++ or Python)
- Launch drivers on startup (systemd service)

**Example driver node (bumpers):**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO  # Or appropriate GPIO library

class BumperNode(Node):
    def __init__(self):
        super().__init__('bumper_node')
        self.publisher = self.create_publisher(Bool, '/safety/bumpers', 10)
        self.timer = self.create_timer(0.02, self.publish_bumper_status)  # 50Hz

        # Configure GPIO pins for bumpers
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Front bumper
        # ... configure other bumpers

    def publish_bumper_status(self):
        msg = Bool()
        msg.data = GPIO.input(17) == GPIO.LOW  # Active low
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = BumperNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

### 3.3 ROS 2 Topics You Must Subscribe (Motor Commands)

**Your hardware drivers must subscribe to these:**

```yaml
# Velocity commands (high-level)
Topic: /control/cmd_vel
Type: geometry_msgs/msg/Twist
Frequency: 10 Hz (from software)
Subscriber: Swerve drive controller (you provide)

# Drive motor velocities (low-level)
Topic: /control/swerve/drive_velocities
Type: std_msgs/msg/Float32MultiArray (4 values)
Frequency: 20 Hz
Subscriber: Drive motor CAN controller (you provide)

# Steering angles (low-level)
Topic: /control/swerve/steer_angles
Type: std_msgs/msg/Float32MultiArray (4 values)
Frequency: 20 Hz
Subscriber: Steer motor PWM/CAN controller (you provide)
```

**Example subscriber node (drive motors):**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import can  # python-can library

class DriveMotorNode(Node):
    def __init__(self):
        super().__init__('drive_motor_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/control/swerve/drive_velocities',
            self.velocity_callback,
            10
        )
        # Initialize CAN bus
        self.can_bus = can.interface.Bus(channel='can0', bustype='socketcan')

    def velocity_callback(self, msg):
        velocities = msg.data  # [v1, v2, v3, v4]
        for i, vel in enumerate(velocities):
            # Send CAN message to motor controller i
            can_msg = can.Message(
                arbitration_id=0x100 + i,
                data=self.velocity_to_bytes(vel),
                is_extended_id=False
            )
            self.can_bus.send(can_msg)

    def velocity_to_bytes(self, vel):
        # Convert velocity (m/s) to motor controller format
        # Example: scale to 0-255 for 8-bit controller
        scaled = int((vel / 1.5) * 255)  # Max velocity 1.5 m/s
        return [scaled & 0xFF]

def main():
    rclpy.init()
    node = DriveMotorNode()
    rclpy.spin(node)
```

### 3.4 CAN Bus Protocol

**Network topology:**
```
CAN Bus (500 kbps)
├── Drive Motor 1 (ID: 0x101)
├── Drive Motor 2 (ID: 0x102)
├── Drive Motor 3 (ID: 0x103)
├── Drive Motor 4 (ID: 0x104)
├── Steer Motor 1 (ID: 0x111)
├── Steer Motor 2 (ID: 0x112)
├── Steer Motor 3 (ID: 0x113)
├── Steer Motor 4 (ID: 0x114)
└── BMS (ID: 0x200)
```

**Message format (drive motors):**
```
CAN ID: 0x101-0x104
Data Length: 2 bytes
Byte 0: Velocity command (signed, -127 to +127, scaled to ±1.5 m/s)
Byte 1: Checksum (optional)
```

**Message format (BMS):**
```
CAN ID: 0x200
Data Length: 8 bytes
Byte 0-1: Voltage (uint16, mV, big-endian)
Byte 2-3: Current (int16, mA, big-endian)
Byte 4: State of Charge (uint8, percentage 0-100)
Byte 5: Temperature (int8, °C)
Byte 6-7: Reserved
```

**Your responsibility:**
- Configure motor controllers with correct CAN IDs
- Implement CAN protocol in motor firmware/controllers
- Provide CAN bus termination resistors (120Ω at each end)
- Document any deviations from standard protocol

### 3.5 Serial Protocol (eHMI - ESP32)

**Connection:**
```
Main Compute (Nucbox K6) UART ←→ ESP32-S3 UART
Baud rate: 115200
Data bits: 8
Parity: None
Stop bits: 1
```

**Command format (Compute → ESP32):**
```json
{
  "type": "led_pattern",
  "pattern": "idle",
  "color": [0, 255, 0],
  "brightness": 128
}

{
  "type": "play_sound",
  "sound": "docking_start",
  "volume": 80
}

{
  "type": "emergency_blink",
  "enabled": true
}
```

**Status format (ESP32 → Compute):**
```json
{
  "type": "status",
  "led_operational": true,
  "audio_operational": true,
  "cpu_temp": 45.2
}
```

**Your responsibility (Kiril):**
- Wire UART connection (TX/RX/GND) between Nucbox and ESP32
- Write ESP32 firmware to parse JSON commands
- Control WS2812B LEDs via SPI
- Control audio playback via I2S
- Send status updates every 5 seconds

**Example ESP32 code (partial):**
```cpp
// ESP32 firmware (Arduino)
#include <ArduinoJson.h>
#include <FastLED.h>

#define LED_PIN 5
#define NUM_LEDS 180
CRGB leds[NUM_LEDS];

void setup() {
  Serial.begin(115200);  // UART to Nucbox
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
}

void loop() {
  if (Serial.available()) {
    String json_str = Serial.readStringUntil('\n');
    StaticJsonDocument<256> doc;
    deserializeJson(doc, json_str);

    String type = doc["type"];
    if (type == "led_pattern") {
      String pattern = doc["pattern"];
      setLEDPattern(pattern);
    } else if (type == "play_sound") {
      String sound = doc["sound"];
      playSound(sound);
    }
  }
}

void setLEDPattern(String pattern) {
  if (pattern == "idle") {
    fill_solid(leds, NUM_LEDS, CRGB::Green);
  } else if (pattern == "navigating") {
    fill_solid(leds, NUM_LEDS, CRGB::Blue);
  } else if (pattern == "docking") {
    fill_solid(leds, NUM_LEDS, CRGB::Yellow);
  } else if (pattern == "error") {
    fill_solid(leds, NUM_LEDS, CRGB::Red);
  }
  FastLED.show();
}
```

---

## 4. Component Specifications

### 4.1 Compute Platform (GMKtec Nucbox K6)

**Specs:**
- CPU: AMD Ryzen 7 7840HS (8 cores, 3.8-5.1 GHz)
- RAM: 32GB DDR5
- Storage: 1TB NVMe SSD
- GPU: AMD Radeon 780M (integrated)
- Ports: 2x USB 3.2, 2x USB-C, 1x Ethernet, 1x HDMI

**Mounting:**
- Location: Inside chassis (protected from weather)
- Cooling: Ensure adequate airflow, fans not blocked
- Vibration: Use rubber grommets to isolate from chassis

**Power:**
- Input: 12V DC (from DC-DC converter)
- Consumption: ~65W typical, 120W peak
- Connector: DC barrel jack (Nucbox standard)

**Software installation:**
- Ubuntu 22.04 LTS (Pankaj installs)
- ROS 2 Humble (Pankaj installs)
- Vehicle software (Pankaj installs)

**Your responsibility:**
- Mount securely inside chassis
- Connect 12V power supply
- Connect Ethernet/WiFi for TVM server communication
- Route cables cleanly

### 4.2 3D LiDAR

**Requirements:**
- Range: 30-100 meters
- FOV: 360° horizontal, ±15° vertical minimum
- Accuracy: ±3cm
- Protection: IP65+ (outdoor-grade, dust + water resistant)
- Interface: USB 3.0 or Ethernet

**Recommended models:**
- Ouster OS1-64 (~$3,500, Ethernet, 64 channels)
- Velodyne VLP-16 (~$4,000, Ethernet, 16 channels)
- Livox Horizon (~$1,500, proprietary protocol, good for robotics)

**Mounting:**
- Location: Top-center of vehicle (360° unobstructed view)
- Height: ~1.2m above ground
- Stabilization: Rigid mount (no wobble)
- Protection: Weatherproof cover (transparent or mesh, provided by Kiril)

**Wiring:**
- Power: 12V DC (from DC-DC converter)
- Data: Ethernet (CAT6) or USB 3.0 to Nucbox
- Cable length: <5 meters to avoid signal degradation

**Driver installation:**
- Pankaj installs ROS 2 driver (e.g., `ouster_driver`, `velodyne_driver`)
- You provide LiDAR model/spec to Pankaj for driver config

### 4.3 Cameras (2x RGB)

**Requirements:**
- Resolution: 1920x1080 (Full HD) minimum
- Frame rate: 30 FPS
- Interface: USB 3.0
- Protection: IP54+ weatherproof housing

**Recommended:**
- ELP USB Camera Module (USB 3.0, 1080p, wide-angle lens, ~$100)
- Logitech C920 (USB 2.0, 1080p, but needs weatherproof enclosure)

**Mounting:**
- Front camera: Center-front, aimed forward/down (for ArUco docking)
- Rear camera: Center-rear, aimed backward (for obstacle awareness)
- Field of view: Wide angle (90°+) preferred

**Wiring:**
- Data: USB 3.0 to Nucbox
- Cable length: <3 meters (USB 3.0 length limit)

**Driver:**
- Standard USB Video Class (UVC) - works out-of-box with ROS 2
- Pankaj configures camera calibration

### 4.4 IMU (9-DOF)

**Requirements:**
- Type: 9-DOF (accelerometer, gyroscope, magnetometer)
- Model: MPU-9250, BNO055, or similar
- Interface: I2C or SPI
- Sample rate: 50 Hz minimum

**Recommended:**
- Adafruit BNO055 Breakout (~$35, I2C, pre-calibrated)
- SparkFun MPU-9250 Breakout (~$15, I2C/SPI)

**Mounting:**
- Location: Center of vehicle chassis (near center of mass)
- Orientation: Align axes with vehicle frame (X=forward, Y=left, Z=up)
- Stabilization: Rigid mount (no vibration isolation needed)

**Wiring:**
- I2C: SDA, SCL, VCC (3.3V or 5V), GND
- Connect to Nucbox GPIO or USB-to-I2C adapter

**Driver:**
- Pankaj uses ROS 2 IMU driver (e.g., `imu_tools`, `bno055_driver`)
- You provide IMU model/I2C address

### 4.5 Encoders

**Drive Encoders (4x):**
- Type: Incremental or absolute
- Resolution: 360 ticks/revolution minimum
- Interface: Hall effect sensors (3-wire: VCC, GND, signal)
- Mounting: On drive motor shaft or wheel hub

**Steer Encoders (4x):**
- Type: Absolute (required for steering angle feedback)
- Resolution: 10-bit (1024 positions) minimum
- Interface: Analog (0-5V), I2C, or SPI
- Mounting: On steering axis

**Your responsibility:**
- Install encoders on each motor/axis
- Wire to motor controllers or directly to Nucbox GPIO
- Provide encoder specs (PPR, interface) to Pankaj

### 4.6 Bumpers

**Requirements:**
- Type: Contact switches or pressure sensors
- Locations: Front, rear, left side, right side (4 minimum)
- Interface: GPIO (digital input, active low preferred)

**Recommended:**
- Mechanical micro-switches (waterproof, IP65+)
- Mounted on flexible bumper strip

**Wiring:**
- Connect to Nucbox GPIO pins
- Use pull-up resistors (10kΩ internal pull-up okay)
- Active low: Pressed = LOW (0V), Released = HIGH (3.3V/5V)

**Example wiring:**
```
Bumper switch ──┬── GND
                └── GPIO pin (with pull-up to 3.3V)
```

### 4.7 Emergency Stop (E-stop)

**CRITICAL SAFETY REQUIREMENT:**

**Hardware circuit:**
```
Battery 48V ──┬── E-stop button (dual-channel) ──┬── Motor Controller Power
              │                                   │
              └── (normally closed contacts)     └── Software monitor (GPIO)
```

**Requirements:**
- Dual-channel emergency stop button (Eaton/Pilz/Schneider)
- Normally closed (NC) contacts (breaks circuit when pressed)
- Hardwired to motor controller power (cannot be overridden by software)
- Redundant monitoring: Hardware AND software

**E-stop button spec:**
- Type: Mushroom head, red, twist-to-release
- Protection: IP65+ (weatherproof)
- Mounting: Easily accessible (top or side of vehicle)

**Software monitor:**
- E-stop status connected to Nucbox GPIO
- Publishes `/safety/estop` topic (Bool, 50 Hz)
- Pankaj's safety monitor reads this topic

**Testing:**
- Press E-stop → ALL motors stop within 100ms
- Software receives E-stop signal
- Vehicle cannot move until E-stop is released

---

## 5. Assembly Guide

### 5.1 Assembly Order (Recommended)

**Phase 1: Mechanical Frame (Week 1-2)**
1. Fabricate chassis frame (welded aluminum)
2. Install swerve drive modules (4x, at corners)
3. Mount suspension system
4. Install docking mechanism (front/rear)

**Phase 2: Power System (Week 3)**
5. Install battery (48V 20Ah LiFePO4) in protected compartment
6. Install BMS (connect to battery)
7. Install DC-DC converters (48V→12V, 48V→5V)
8. Wire power distribution (48V bus, 12V bus, 5V bus)
9. Install E-stop circuit (hardwired to motor power)
10. Install charging port

**Phase 3: Compute & Sensors (Week 4-5)**
11. Mount Nucbox K6 inside chassis
12. Connect 12V power to Nucbox
13. Mount LiDAR on top (center)
14. Mount cameras (front + rear)
15. Install IMU (center of chassis)
16. Install encoders (drive + steer)
17. Install bumpers (perimeter)

**Phase 4: Motor Control (Week 6-8)**
18. Install drive motor controllers (4x, CAN bus)
19. Install steer motor controllers (4x, PWM or CAN)
20. Wire CAN bus (twisted pair, 120Ω termination at ends)
21. Connect motors to controllers
22. Test motors individually (with CAN tool or ROS)

**Phase 5: eHMI (Week 9-10, Kiril)**
23. Install ESP32-S3 inside chassis
24. Wire LED strips (WS2812B) around perimeter
25. Install speakers (weatherproof)
26. Connect audio amplifier (I2S)
27. Wire UART from ESP32 to Nucbox
28. Connect 5V power to ESP32

**Phase 6: Cable Management (Week 11)**
29. Route all cables internally (no external dangling)
30. Use cable ties and mounts
31. Label all connectors
32. Weatherproof all external connectors (IP54+)

**Phase 7: Exterior (Week 12-16, Kiril)**
33. Fabricate body panels
34. Install headlights and taillights
35. Install protective covers (LiDAR, cameras)
36. Apply branding/logo
37. Final weatherproofing check

### 5.2 Power Distribution Diagram

```
48V Battery (20Ah LiFePO4)
    │
    ├──[BMS]──[E-stop]──┬── Drive Motor Controllers (4x) ─── Drive Motors (4x)
    │                   │
    │                   └── Steer Motor Controllers (4x) ─── Steer Motors (4x)
    │
    ├──[DC-DC 48V→12V, 20A]──┬── Nucbox K6 (compute)
    │                        ├── LiDAR
    │                        ├── Cameras (2x)
    │                        └── Motor controllers (logic power)
    │
    └──[DC-DC 48V→5V, 10A]───┬── ESP32-S3
                             ├── IMU
                             ├── Encoders
                             └── Bumpers
```

### 5.3 CAN Bus Wiring

```
Nucbox K6 (CAN interface)
    │
    ├── CAN_H (twisted pair, yellow)
    ├── CAN_L (twisted pair, green)
    │
    ├─[120Ω termination]
    │
    ├── Motor 1 (ID: 0x101)
    ├── Motor 2 (ID: 0x102)
    ├── Motor 3 (ID: 0x103)
    ├── Motor 4 (ID: 0x104)
    ├── Steer 1 (ID: 0x111)
    ├── Steer 2 (ID: 0x112)
    ├── Steer 3 (ID: 0x113)
    ├── Steer 4 (ID: 0x114)
    ├── BMS (ID: 0x200)
    │
    └─[120Ω termination]
```

**Wiring tips:**
- Use twisted pair cable for CAN_H and CAN_L
- Keep cable length <40 meters total
- 120Ω termination resistors at BOTH ends of bus
- Avoid stubs (T-junctions) longer than 300mm

---

## 6. Testing Procedures

### 6.1 Power System Testing

**Test 1: Battery voltage**
```
Procedure:
1. Disconnect all loads
2. Measure battery voltage with multimeter
Expected: 48V ±2V (51.2V fully charged, 40V empty for LiFePO4)
```

**Test 2: DC-DC converters**
```
Procedure:
1. Connect load (Nucbox, LiDAR)
2. Measure output voltage
Expected: 12V ±0.5V, 5V ±0.25V
```

**Test 3: E-stop**
```
Procedure:
1. Power on system
2. Press E-stop button
Expected: ALL motor power cuts off within 100ms
         E-stop GPIO reads LOW
```

### 6.2 Sensor Testing

**Test 1: LiDAR**
```bash
# On Nucbox (after Pankaj installs driver)
ros2 topic echo /sensors/lidar/points

# Expected: Point cloud data streaming at ~10Hz
# If no data: Check power (12V), Ethernet connection, driver config
```

**Test 2: Cameras**
```bash
ros2 topic echo /sensors/camera/front/image --no-arr

# Expected: Image messages at 30Hz
# If no data: Check USB connection, camera power
```

**Test 3: IMU**
```bash
ros2 topic echo /sensors/imu/data

# Expected: IMU data at 50Hz, orientation quaternion, angular velocity
# If no data: Check I2C connection, I2C address, driver config
```

**Test 4: Bumpers**
```bash
ros2 topic echo /safety/bumpers

# Press each bumper → value should change to True
# Expected: Responsive within 20ms
```

### 6.3 Motor Control Testing

**Test 1: CAN bus communication**
```bash
# Install can-utils on Nucbox
sudo apt install can-utils

# Bring up CAN interface
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Listen for messages
candump can0

# Expected: See BMS messages (ID 0x200) broadcasting
```

**Test 2: Drive motors**
```bash
# Send test command via CAN
cansend can0 101#7F  # Motor 1, 50% forward (127 decimal = 0x7F)

# Expected: Motor 1 rotates forward at half speed
# Repeat for motors 2, 3, 4 (IDs 102, 103, 104)
```

**Test 3: Steering motors**
```
# Via PWM or CAN (depends on your setup)
# Send steering command to each motor
# Expected: Wheel rotates to commanded angle
```

### 6.4 eHMI Testing (Kiril)

**Test 1: Serial communication**
```bash
# On Nucbox
screen /dev/ttyUSB0 115200  # Or /dev/ttyACM0

# Send test command
{"type":"led_pattern","pattern":"idle","color":[0,255,0],"brightness":128}

# Expected: ESP32 receives command, LEDs turn green
```

**Test 2: LED control**
```
# Cycle through patterns: idle, navigating, docking, error
# Expected: LEDs change color accordingly
```

**Test 3: Audio**
```
# Send sound command
{"type":"play_sound","sound":"startup","volume":80}

# Expected: Speaker plays startup sound
```

---

## 7. Integration with Software

### 7.1 Handoff to Pankaj

**Before Pankaj can start:**
- [x] All hardware assembled
- [x] Power system tested and operational
- [x] All sensors mounted and powered
- [x] Ubuntu 22.04 installed on Nucbox (or Pankaj installs)
- [x] Network configuration (WiFi or Ethernet) working

**What you provide to Pankaj:**
1. **Hardware spec sheet:**
   - LiDAR model and connection type (USB/Ethernet)
   - Camera model and USB ports
   - IMU model and I2C address
   - Encoder specs (PPR, interface)
   - CAN bus baud rate and device IDs
   - ESP32 serial port (/dev/ttyUSB0 or /dev/ttyACM0)

2. **Wiring diagram:**
   - Which USB port is LiDAR
   - Which USB ports are cameras (front/rear)
   - CAN interface device name (can0)
   - GPIO pin assignments (bumpers, E-stop)

3. **Test results:**
   - Power system voltages verified
   - All sensors tested individually
   - Motor control tested
   - E-stop tested

**What Pankaj does:**
- Installs ROS 2 Humble
- Installs sensor drivers (LiDAR, cameras, IMU)
- Writes motor control nodes (CAN bus)
- Writes eHMI node (serial to ESP32)
- Configures ROS 2 topics as specified
- Tests integration with hardware

### 7.2 Joint Testing (Week 12-15)

**Test 1: Sensor data pipeline**
- Pankaj verifies all sensor topics publishing correctly
- Adjusts driver configs if needed
- Calibrates cameras (intrinsic/extrinsic)

**Test 2: Motor control**
- Pankaj sends test commands via ROS topics
- You verify motors respond correctly
- Tune PID controllers if needed

**Test 3: Safety systems**
- Test E-stop (hardware and software)
- Test bumpers trigger software response
- Verify battery monitoring

**Test 4: eHMI**
- Pankaj sends LED/sound commands
- You verify ESP32 receives and executes
- Test all LED patterns and sounds

### 7.3 Troubleshooting Checklist

**Issue: LiDAR not publishing data**
```
Your checks:
- Is LiDAR powered? (12V supply)
- Is Ethernet/USB cable connected?
- Is LiDAR LED blinking (indicates operation)?

Pankaj checks:
- Is driver installed and running?
- Correct topic name configured?
- Network settings (if Ethernet LiDAR)?
```

**Issue: Motors not responding to commands**
```
Your checks:
- Is CAN bus powered?
- Are termination resistors installed (120Ω at both ends)?
- Are motor controllers configured with correct IDs?
- Can you see CAN messages with `candump`?

Pankaj checks:
- Is ROS node sending commands?
- Correct CAN message format?
- Correct motor IDs in code?
```

**Issue: E-stop not stopping motors**
```
Your checks:
- Is E-stop wired correctly (normally closed)?
- Does pressing E-stop cut motor power?
- Is E-stop GPIO connected?

Critical: This is a safety issue, must be fixed immediately!
```

---

## 8. Deliverables Checklist

### 8.1 Week 5 Deliverables (Prototype)

- [ ] Chassis frame assembled
- [ ] Swerve drive modules installed (basic motion only)
- [ ] Power system operational (battery, DC-DC, BMS)
- [ ] Nucbox K6 mounted and powered
- [ ] LiDAR mounted and publishing data (test with `ros2 topic echo`)
- [ ] Cameras mounted and streaming (test with `ros2 topic echo`)
- [ ] E-stop circuit working
- [ ] Documentation: wiring diagram, sensor specs

**Milestone:** "Walking Skeleton" - Platform can drive, sensors publishing

### 8.2 Week 16 Deliverables (Complete Platform)

- [ ] All 4 swerve modules fully operational (steering + drive)
- [ ] All sensors installed and tested (LiDAR, cameras, IMU, encoders, bumpers)
- [ ] Power system finalized with charging port
- [ ] Docking mechanism installed
- [ ] eHMI complete (ESP32, LEDs, speakers)
- [ ] Exterior panels and protective covers (IP54+)
- [ ] All cable management complete (weatherproof connectors)
- [ ] Safety systems validated (E-stop <100ms response)
- [ ] Integration testing with Pankaj complete
- [ ] Final documentation package delivered

**Milestone:** Ready for field testing with Pankaj's software

---

## 9. Summary

**Your deliverables to Pankaj:**
1. ✅ Physical platform (chassis, motors, power, sensors)
2. ✅ ROS 2 topics publishing sensor data (7 topics)
3. ✅ ROS 2 topics subscribing to motor commands (3 topics)
4. ✅ CAN bus for motor control
5. ✅ Serial interface for eHMI (ESP32)
6. ✅ E-stop hardware circuit
7. ✅ Hardware spec sheet and wiring diagram
8. ✅ Test results documentation

**Timeline:**
- Week 5: Prototype (basic motion + sensors)
- Week 16: Complete platform (all features)
- Week 12-15: Joint testing with Pankaj

**Critical success factors:**
- Follow HARDWARE_SOFTWARE_INTERFACE.md exactly
- Test each component individually before integration
- Coordinate with Pankaj on any interface changes
- Prioritize safety (E-stop, bumpers, battery protection)

**Next steps:**
1. Review this guide with your team
2. Review HARDWARE_SOFTWARE_INTERFACE.md
3. Order components (LiDAR, cameras, IMU, motors)
4. Begin chassis fabrication
5. Set up weekly sync meetings with Pankaj

---

**Document Status:** ✅ Complete
**Version:** 1.0
**Date:** December 19, 2025
**Prepared By:** Pankaj

**Good luck with the build! 🔧**
