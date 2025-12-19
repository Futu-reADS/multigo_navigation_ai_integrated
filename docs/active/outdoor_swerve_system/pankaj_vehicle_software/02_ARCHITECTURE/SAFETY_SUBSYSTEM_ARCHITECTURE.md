# Safety Subsystem Architecture

**Document ID:** ARCH-SAFE-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft

---

## 1. Overview

The **Safety Subsystem** is the **highest priority subsystem**, ensuring safe operation for passengers and bystanders. It implements a **defense-in-depth** strategy with four independent safety layers.

**Key Capabilities:**
- Emergency stop (hardware-level, <100ms response)
- Passenger safety monitoring (secure attachment, speed limits, smooth motion)
- Operational safety (obstacle avoidance, slope limits, weather limits)
- System health monitoring (watchdog, diagnostics, fail-safe)
- Compliance with ISO 13482 (safety for personal care robots)

**Safety Philosophy (Defense-in-Depth):**
```
Layer 1: Emergency Stop (Hardware)     ← <100ms, independent circuit
Layer 2: System Monitoring (Watchdog)  ← Software watchdog, health checks
Layer 3: Collision Avoidance (Perception + Planning) ← Predictive safety
Layer 4: Operational Safety (Constraints) ← Speed limits, slope limits
```

---

## 2. System Architecture

### 2.1 Component Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                    Safety Subsystem                              │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │              Layer 1: Emergency Stop (Hardware)            │ │
│  │                                                            │ │
│  │  Physical E-Stop Button ──┐                               │ │
│  │  UI E-Stop Button ────────┼──▶ Relay ──▶ Motor Power OFF │ │
│  │  Safety Monitor Trigger ──┘      (Hardware Circuit)       │ │
│  │                                  <100ms response           │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │          Layer 2: System Monitoring (Software)             │ │
│  │                                                            │ │
│  │  ┌──────────────────┐                                      │ │
│  │  │ Safety Monitor   │  Watchdog, health checks             │ │
│  │  │   (Supervisor)   │  All subsystems report status        │ │
│  │  │                  │  Trigger E-Stop on failure           │ │
│  │  └────────┬─────────┘                                      │ │
│  │           │                                                 │ │
│  │           │ health status                                  │ │
│  │           ↓                                                 │ │
│  │  ┌────────────────────────────────────────┐               │ │
│  │  │ Subsystem Health Monitors:             │               │ │
│  │  │ - Navigation health                    │               │ │
│  │  │ - Localization health (NDT score)      │               │ │
│  │  │ - Perception health (LiDAR)            │               │ │
│  │  │ - Swerve drive health (wheel encoders) │               │ │
│  │  │ - Docking health (ArUco detection)     │               │ │
│  │  │ - Battery health (voltage, current)    │               │ │
│  │  └────────────────────────────────────────┘               │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │     Layer 3: Collision Avoidance (Perception + Planning)   │ │
│  │                                                            │ │
│  │  3D LiDAR → Perception → Costmap → Nav2 → Avoid obstacles │ │
│  │  Minimum distance: 0.5m from obstacles (inflation)        │ │
│  │  Dynamic obstacle tracking and prediction                 │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │        Layer 4: Operational Safety (Constraints)           │ │
│  │                                                            │ │
│  │  - Speed limits: 1.0 m/s (passenger), 1.5 m/s (empty)     │ │
│  │  - Acceleration limits: 0.5 m/s²                          │ │
│  │  - Slope limits: 10° max                                  │ │
│  │  - Weather limits: No heavy rain (IP54+)                  │ │
│  │  - Operational hours: Daylight only (no autonomous night) │ │
│  └────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────┘
```

---

## 3. ROS 2 Node Architecture

### 3.1 Safety Monitor Node

**Node Name:** `safety_monitor`
**Language:** C++
**Lifecycle:** Managed lifecycle node (highest priority)

**Subscribed Topics:**
- `/odom` (nav_msgs/Odometry): Robot velocity (speed limiting)
- `/ndt_score` (std_msgs/Float32): Localization quality
- `/points_filtered` (sensor_msgs/PointCloud2): Obstacle detection
- `/battery_status` (sensor_msgs/BatteryState): Battery health
- `/diagnostics` (diagnostic_msgs/DiagnosticArray): Subsystem health
- `/passenger_attached` (std_msgs/Bool): Wheelchair docking status

**Published Topics:**
- `/safety_status` (custom_msgs/SafetyStatus): Overall safety state
- `/emergency_stop` (std_msgs/Bool): Emergency stop trigger
- `/cmd_vel_limited` (geometry_msgs/Twist): Velocity commands after safety limiting

**Subscribed Services (Health Checks):**
- `/navigation/health` (custom_srvs/GetHealth)
- `/perception/health` (custom_srvs/GetHealth)
- `/swerve_drive/health` (custom_srvs/GetHealth)

**Published Actions:**
- `/safe_stop` (custom_msgs/action/SafeStop): Controlled deceleration to stop

**Parameters:**
```yaml
safety_monitor:
  ros__parameters:
    # Watchdog timeouts
    watchdog:
      navigation_timeout: 1.0     # seconds (no updates → E-Stop)
      localization_timeout: 1.0   # seconds
      perception_timeout: 0.5     # seconds (critical!)
      control_timeout: 0.2        # seconds (very critical!)
    
    # Health check thresholds
    health:
      min_ndt_score: 0.3          # Localization quality
      min_battery_voltage: 42.0   # Volts (80% of 48V nominal)
      max_cpu_usage: 90.0         # Percent
      max_temperature: 80.0       # Celsius (CPU/battery)
    
    # Operational limits
    limits:
      max_speed_passenger: 1.0    # m/s
      max_speed_empty: 1.5        # m/s
      max_acceleration: 0.5       # m/s²
      max_slope: 10.0             # degrees (17.6% grade)
      min_obstacle_distance: 0.5  # meters
    
    # Emergency stop
    estop:
      hardware_relay_topic: "/estop_relay"  # GPIO output
      deceleration_rate: 1.0      # m/s² (controlled stop)
```

---

### 3.2 Emergency Stop Controller Node

**Node Name:** `estop_controller`
**Language:** C++
**Lifecycle:** Standard node

**Subscribed Topics:**
- `/estop_button_physical` (std_msgs/Bool): Physical E-Stop button (GPIO input)
- `/estop_button_ui` (std_msgs/Bool): UI E-Stop button
- `/emergency_stop` (std_msgs/Bool): Software E-Stop trigger (from safety_monitor)

**Published Topics:**
- `/estop_relay` (std_msgs/Bool): Relay control (GPIO output, cuts motor power)
- `/estop_status` (custom_msgs/EStopStatus): E-Stop state and reason

**Hardware Interface:**
- GPIO input: Physical E-Stop button (active-low, pull-up resistor)
- GPIO output: Relay control (NO relay, cuts 48V to motor controllers)

**Configuration:**
```yaml
estop_controller:
  ros__parameters:
    # GPIO pins
    gpio:
      estop_button_pin: 17        # BCM pin (Raspberry Pi / similar)
      relay_pin: 27               # BCM pin
    
    # Debouncing
    debounce_ms: 50               # Ignore bounces for 50ms
    
    # Auto-reset
    auto_reset_enabled: false     # Require manual reset
    auto_reset_delay: 5.0         # seconds (if enabled)
```

---

### 3.3 Velocity Governor Node

**Node Name:** `velocity_governor`
**Language:** C++
**Lifecycle:** Standard node

**Subscribed Topics:**
- `/cmd_vel` (geometry_msgs/Twist): Raw velocity commands (from Nav2)
- `/passenger_attached` (std_msgs/Bool): Passenger status
- `/safety_status` (custom_msgs/SafetyStatus): Current safety state

**Published Topics:**
- `/cmd_vel_limited` (geometry_msgs/Twist): Limited velocity commands (to swerve drive)

**Algorithm:**
```cpp
geometry_msgs::msg::Twist limitVelocity(
    const geometry_msgs::msg::Twist& cmd_vel,
    bool passenger_onboard,
    const SafetyStatus& safety_status) {

    geometry_msgs::msg::Twist limited = cmd_vel;

    // 1. Speed limits
    double max_linear = passenger_onboard ? max_speed_passenger : max_speed_empty;
    double linear_mag = std::sqrt(cmd_vel.linear.x * cmd_vel.linear.x +
                                   cmd_vel.linear.y * cmd_vel.linear.y);
    if (linear_mag > max_linear) {
        double scale = max_linear / linear_mag;
        limited.linear.x *= scale;
        limited.linear.y *= scale;
    }

    // 2. Angular velocity limit
    limited.angular.z = std::clamp(cmd_vel.angular.z, -max_angular_vel, max_angular_vel);

    // 3. Acceleration limiting (compare to previous command)
    double dt = 0.05;  // 20 Hz controller
    double accel_x = (limited.linear.x - prev_cmd_.linear.x) / dt;
    double accel_y = (limited.linear.y - prev_cmd_.linear.y) / dt;
    
    if (std::abs(accel_x) > max_acceleration) {
        limited.linear.x = prev_cmd_.linear.x + std::copysign(max_acceleration * dt, accel_x);
    }
    if (std::abs(accel_y) > max_acceleration) {
        limited.linear.y = prev_cmd_.linear.y + std::copysign(max_acceleration * dt, accel_y);
    }

    // 4. Safety overrides
    if (safety_status.emergency_stop) {
        // Immediate stop
        limited.linear.x = 0.0;
        limited.linear.y = 0.0;
        limited.angular.z = 0.0;
    } else if (safety_status.degraded_mode) {
        // Reduced speed in degraded mode (e.g., low battery, poor localization)
        limited.linear.x *= 0.5;
        limited.linear.y *= 0.5;
        limited.angular.z *= 0.5;
    }

    prev_cmd_ = limited;
    return limited;
}
```

---

## 4. Safety Layers (Defense-in-Depth)

### 4.1 Layer 1: Emergency Stop (Hardware)

**Purpose:** Immediate power cutoff to motors (<100ms response)

**Implementation:**
```
Physical E-Stop Button (NC, normally closed)
    │
    ├──▶ GPIO Input (estop_controller)
    │
    └──▶ Hardware Relay (NO, normally open)
            │
            └──▶ 48V Power to Motor Controllers
```

**Hardware Circuit:**
- Physical E-Stop button wired in series with relay coil
- Button pressed → relay opens → motor power cut
- Independent of software (fail-safe)

**Software Interface:**
```cpp
void handlePhysicalEStop(bool button_pressed) {
    if (button_pressed) {
        // 1. Open relay (cut motor power)
        gpio_->setPin(relay_pin, false);
        
        // 2. Publish E-Stop status
        estop_status.active = true;
        estop_status.reason = "Physical button pressed";
        estop_publisher_->publish(estop_status);
        
        // 3. Log event
        RCLCPP_ERROR(get_logger(), "EMERGENCY STOP ACTIVATED: Physical button");
    }
}

void resetEStop() {
    // Manual reset required
    // 1. Verify safe conditions
    if (!verifySafeToReset()) {
        RCLCPP_WARN(get_logger(), "Cannot reset E-Stop: unsafe conditions");
        return;
    }
    
    // 2. Close relay (restore motor power)
    gpio_->setPin(relay_pin, true);
    
    // 3. Publish reset status
    estop_status.active = false;
    estop_status.reason = "Reset";
    estop_publisher_->publish(estop_status);
    
    RCLCPP_INFO(get_logger(), "E-Stop reset");
}
```

---

### 4.2 Layer 2: System Monitoring (Watchdog)

**Purpose:** Detect software failures and trigger safe shutdown

**Watchdog Algorithm:**
```cpp
class SafetyMonitor {
private:
    struct SubsystemHealth {
        std::string name;
        rclcpp::Time last_update;
        double timeout;
        bool healthy;
    };
    
    std::map<std::string, SubsystemHealth> subsystems_;

public:
    void checkWatchdog() {
        rclcpp::Time now = this->now();
        
        for (auto& [name, health] : subsystems_) {
            double elapsed = (now - health.last_update).seconds();
            
            if (elapsed > health.timeout) {
                RCLCPP_ERROR(get_logger(), "Watchdog timeout: %s (%.2fs)", 
                             name.c_str(), elapsed);
                health.healthy = false;
                
                // Trigger safe stop (NOT emergency stop, controlled deceleration)
                triggerSafeStop(name + " watchdog timeout");
            }
        }
    }
    
    void updateSubsystemHealth(const std::string& name, bool healthy) {
        if (subsystems_.count(name)) {
            subsystems_[name].last_update = this->now();
            subsystems_[name].healthy = healthy;
        }
    }
    
    void triggerSafeStop(const std::string& reason) {
        // Controlled deceleration to stop (NOT emergency cut power)
        auto goal = custom_msgs::action::SafeStop::Goal();
        goal.reason = reason;
        goal.deceleration_rate = deceleration_rate_;
        
        safe_stop_client_->async_send_goal(goal);
        
        // Publish degraded mode
        safety_status_.emergency_stop = false;
        safety_status_.degraded_mode = true;
        safety_status_.reason = reason;
        status_publisher_->publish(safety_status_);
    }
};
```

**Health Checks:**
```cpp
void performHealthChecks() {
    // 1. Localization health (NDT score)
    if (ndt_score_ < min_ndt_score) {
        updateSubsystemHealth("localization", false);
        RCLCPP_WARN(get_logger(), "Poor localization: NDT score %.2f < %.2f", 
                    ndt_score_, min_ndt_score);
    }
    
    // 2. Battery health
    if (battery_voltage_ < min_battery_voltage) {
        updateSubsystemHealth("battery", false);
        RCLCPP_ERROR(get_logger(), "Low battery: %.1fV < %.1fV", 
                     battery_voltage_, min_battery_voltage);
        triggerSafeStop("Low battery");
    }
    
    // 3. CPU temperature
    double cpu_temp = getCPUTemperature();
    if (cpu_temp > max_temperature) {
        RCLCPP_ERROR(get_logger(), "CPU overheating: %.1f°C", cpu_temp);
        triggerSafeStop("CPU overheating");
    }
    
    // 4. Perception health (LiDAR data rate)
    double lidar_rate = getLidarDataRate();
    if (lidar_rate < 5.0) {  // Expect 10 Hz, tolerate down to 5 Hz
        updateSubsystemHealth("perception", false);
        RCLCPP_WARN(get_logger(), "LiDAR data rate low: %.1f Hz", lidar_rate);
    }
}
```

---

### 4.3 Layer 3: Collision Avoidance

**Purpose:** Predictive obstacle avoidance (prevent collisions before they happen)

**Integration:**
- Perception subsystem detects obstacles (3D LiDAR → filtered point cloud)
- Nav2 costmap marks obstacles with inflation (0.5m buffer)
- Nav2 controller avoids obstacles during path following

**Minimum Distance Enforcement:**
```yaml
# In local_costmap.yaml
inflation_layer:
  inflation_radius: 0.5           # meters (safety buffer)
  cost_scaling_factor: 3.0        # Aggressive cost increase near obstacles
```

**Dynamic Obstacle Handling:**
- Voxel layer clears and marks obstacles at 5 Hz (real-time tracking)
- DWB controller samples trajectories to avoid obstacles
- Recovery behaviors if path blocked (spin, backup, wait)

---

### 4.4 Layer 4: Operational Safety (Constraints)

**Purpose:** Enforce operational limits to prevent hazardous conditions

**Speed Limits:**
- **Passenger onboard:** 1.0 m/s (3.6 km/h, walking pace)
- **Empty:** 1.5 m/s (5.4 km/h, brisk walk)
- **Acceleration:** 0.5 m/s² (gentle, avoid passenger discomfort)

**Slope Limits:**
- **Max slope:** 10° (17.6% grade, ~100kg load capacity)
- **Monitoring:** IMU pitch/roll
- **Action:** Refuse navigation goals on excessive slopes

**Weather Limits:**
- **IP54+ rating:** Light rain operation
- **NO heavy rain:** Water intrusion risk
- **NO night operation:** Autonomous navigation daylight only (manual override for indoor)

**Operational Hours:**
```yaml
operational_hours:
  enabled: true
  daylight_only: true             # No autonomous operation at night
  override_indoor: true           # Indoor operation allowed anytime
  sunrise_offset: 30              # minutes (wait 30min after sunrise)
  sunset_offset: -30              # minutes (stop 30min before sunset)
```

---

## 5. Passenger Safety Features

### 5.1 Secure Attachment Verification

**Docking Confirmation:**
```cpp
bool verifySecureAttachment() {
    // 1. Check docking precision (must be within ±5mm)
    double docking_error = computeDockingError();
    if (docking_error > 0.005) {  // 5mm
        RCLCPP_ERROR(get_logger(), "Docking error too large: %.1fmm", docking_error * 1000);
        return false;
    }
    
    // 2. Check wheelchair lock status (future: CAN/serial interface)
    bool locked = checkWheelchairLock();
    if (!locked) {
        RCLCPP_ERROR(get_logger(), "Wheelchair not locked");
        return false;
    }
    
    // 3. Verify IMU stability (passenger settled, no excessive motion)
    double imu_accel_mag = getIMUAccelMagnitude();
    if (imu_accel_mag > 0.5) {  // m/s² (excluding gravity)
        RCLCPP_WARN(get_logger(), "Excessive motion detected, wait for passenger to settle");
        return false;
    }
    
    return true;
}
```

---

### 5.2 Smooth Motion Control

**Jerk Limiting:**
```cpp
// Limit rate of acceleration change (jerk)
double max_jerk = 1.0;  // m/s³

double jerk_x = (current_accel_x - prev_accel_x) / dt;
if (std::abs(jerk_x) > max_jerk) {
    current_accel_x = prev_accel_x + std::copysign(max_jerk * dt, jerk_x);
}
```

**Smooth Starts and Stops:**
- Ramp acceleration from 0 to target over 2 seconds
- Controlled deceleration (1.0 m/s²) for safe stops
- NO sudden direction changes (limit angular acceleration)

---

## 6. Fault Detection and Recovery

### 6.1 Fault Categories

| Fault Type | Example | Response | Recovery |
|------------|---------|----------|----------|
| Critical | E-Stop button, collision | Emergency stop | Manual reset |
| Major | Localization lost, LiDAR failure | Safe stop | Auto-retry or manual |
| Minor | Low battery warning, high CPU | Degraded mode | Continue with limits |
| Warning | Poor WiFi, high temperature | Log only | Monitor |

---

### 6.2 Recovery Actions

**Safe Stop:**
```cpp
void executeSafeStop(double deceleration_rate) {
    // 1. Controlled deceleration
    double current_speed = getCurrentSpeed();
    double target_speed = 0.0;
    
    rclcpp::Rate rate(20);  // 20 Hz
    while (current_speed > 0.01) {
        double delta_v = deceleration_rate * 0.05;  // dt = 50ms
        target_speed = std::max(0.0, current_speed - delta_v);
        
        publishVelocityCommand(target_speed);
        rate.sleep();
        current_speed = getCurrentSpeed();
    }
    
    // 2. Engage brakes (if available)
    engageBrakes();
    
    // 3. Publish stopped status
    RCLCPP_INFO(get_logger(), "Safe stop complete");
}
```

**Return to Safe State:**
- Navigate to nearest waypoint (if localization OK)
- Return to charging station (if battery low)
- Stay in place and request manual intervention (if major fault)

---

## 7. Testing Strategy

### 7.1 Unit Tests
- Emergency stop response time (<100ms)
- Velocity limiting correctness
- Health check thresholds
- Watchdog timeout detection

### 7.2 Integration Tests
- Full safety stack with all subsystems
- E-Stop triggered from each source (physical, UI, software)
- Safe stop from various speeds
- Recovery from degraded mode

### 7.3 Field Tests (Safety-Critical)
- Emergency stop during motion (various speeds)
- Obstacle avoidance (static and dynamic obstacles)
- Slope limit enforcement (10° slope test)
- Passenger safety (smooth motion, secure attachment)
- Battery depletion handling (low battery safe return)
- Localization loss recovery

---

## 8. Performance Targets

| Metric | Target | Validation |
|--------|--------|------------|
| Emergency Stop Response Time | <100ms | Hardware testing |
| Watchdog Detection Time | <1s | Fault injection |
| Velocity Limiting Latency | <20ms | Timestamp analysis |
| Health Check Rate | 10 Hz | Timestamp analysis |
| Safe Stop Deceleration | 1.0 m/s² | Accelerometer |
| Collision Avoidance Distance | >0.5m | Obstacle tests |

---

## 9. Implementation Phases

### Phase 1: Emergency Stop (Sprints 1-2)
- ✅ Hardware relay circuit
- ✅ Physical E-Stop button integration
- ✅ UI E-Stop button
- ✅ E-Stop controller node

### Phase 2: System Monitoring (Sprints 3-4)
- ✅ Safety monitor node
- ✅ Watchdog implementation
- ✅ Health checks (all subsystems)
- ✅ Diagnostics integration

### Phase 3: Operational Safety (Sprints 5-6)
- ✅ Velocity governor
- ✅ Acceleration limiting
- ✅ Slope monitoring (IMU)
- ✅ Operational constraints

### Phase 4: Passenger Safety (Sprints 7-8)
- ✅ Secure attachment verification
- ✅ Smooth motion control (jerk limiting)
- ✅ Passenger onboard detection
- ✅ Safety validation testing

---

**Document Status:** Draft
**Implementation Status:** Not Started
**Approvals Required:** Safety Engineer, System Architect, Regulatory Compliance Lead
