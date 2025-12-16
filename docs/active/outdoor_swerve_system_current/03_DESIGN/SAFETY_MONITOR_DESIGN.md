# Safety Monitor Detailed Design

**Document ID:** DESIGN-SAFETY-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft

---

## 1. Overview

This document provides detailed design specifications for the Safety Monitor, implementing defense-in-depth safety with 4 independent layers.

**Key Components:**
- EmergencyStopController (hardware relay + software monitor)
- VelocityGovernor (speed limiting based on context)
- HealthMonitor (watchdog for all subsystems)
- CollisionPredictor (predictive safety)

**Safety Architecture:** 4 Independent Layers
1. **Emergency Stop (Hardware)** - <100ms relay cutoff
2. **System Monitoring (Watchdog)** - Software health checks
3. **Collision Avoidance (Perception + Planning)** - Predictive safety
4. **Operational Safety (Constraints)** - Speed/slope/weather limits

---

## 2. Safety Layers

### Layer 1: Emergency Stop (Hardware)

```
Physical E-Stop Button → Hardware Relay → Motor Power Cutoff
        │                                      (<100ms)
        ├→ Software E-Stop Topic (/emergency_stop)
        └→ Safety Status LED
```

**Implementation:**
```cpp
class EmergencyStopController {
public:
    void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            // Trigger hardware relay via GPIO
            gpio_->setPin(ESTOP_RELAY_PIN, GPIO_HIGH);

            // Publish zero velocity
            publishZeroVelocity();

            // Set safety state
            safety_state_ = SafetyState::EMERGENCY_STOP;

            RCLCPP_ERROR(logger_, "EMERGENCY STOP ACTIVATED");
        }
    }

private:
    void publishZeroVelocity() {
        geometry_msgs::msg::Twist zero_cmd;
        // All velocities = 0
        cmd_vel_pub_->publish(zero_cmd);
    }
};
```

---

### Layer 2: Watchdog Monitoring

```cpp
class HealthMonitor {
public:
    struct SubsystemHealth {
        std::string name;
        rclcpp::Time last_heartbeat;
        double timeout_threshold;  // seconds
        HealthStatus status;
    };

    void monitorLoop() {
        auto now = node_->now();

        for (auto& subsystem : monitored_subsystems_) {
            double elapsed = (now - subsystem.last_heartbeat).seconds();

            if (elapsed > subsystem.timeout_threshold) {
                RCLCPP_ERROR(logger_, "Subsystem %s timeout (%.1fs)",
                             subsystem.name.c_str(), elapsed);

                handleSubsystemFailure(subsystem.name);
            }
        }
    }

private:
    void handleSubsystemFailure(const std::string& subsystem_name) {
        // Critical subsystems trigger safe stop
        if (isCriticalSubsystem(subsystem_name)) {
            triggerSafeStop("Critical subsystem failure: " + subsystem_name);
        } else {
            // Non-critical: log warning, enter degraded mode
            enterDegradedMode(subsystem_name);
        }
    }

    std::vector<SubsystemHealth> monitored_subsystems_ = {
        {"swerve_controller", now(), 1.0, HealthStatus::OK},
        {"ndt_localization", now(), 2.0, HealthStatus::OK},
        {"perception", now(), 1.0, HealthStatus::OK},
        {"navigation", now(), 5.0, HealthStatus::OK}
    };
};
```

---

### Layer 3: Collision Avoidance

**Predictive Collision Detection:**
```cpp
class CollisionPredictor {
public:
    bool predictCollision(
        const geometry_msgs::msg::Twist& cmd_vel,
        const sensor_msgs::msg::PointCloud2& obstacle_cloud,
        double prediction_horizon)  // seconds
    {
        // Simulate robot trajectory
        auto trajectory = simulateTrajectory(cmd_vel, prediction_horizon);

        // Check for collisions along trajectory
        for (const auto& pose : trajectory) {
            if (checkCollision(pose, obstacle_cloud)) {
                return true;  // Collision predicted
            }
        }

        return false;  // Safe
    }

private:
    std::vector<Pose> simulateTrajectory(
        const geometry_msgs::msg::Twist& cmd_vel,
        double horizon)
    {
        std::vector<Pose> trajectory;
        Pose current_pose = getCurrentPose();

        double dt = 0.1;  // 100ms timestep
        for (double t = 0; t < horizon; t += dt) {
            current_pose = integrateMotion(current_pose, cmd_vel, dt);
            trajectory.push_back(current_pose);
        }

        return trajectory;
    }
};
```

---

### Layer 4: Velocity Governor

**Context-Aware Speed Limiting:**
```cpp
class VelocityGovernor {
public:
    geometry_msgs::msg::Twist limitVelocity(
        const geometry_msgs::msg::Twist& cmd_vel,
        const SafetyContext& context)
    {
        geometry_msgs::msg::Twist limited_cmd = cmd_vel;

        // 1. Passenger onboard check
        double max_linear_vel = context.passenger_onboard ?
            params_.max_vel_with_passenger : params_.max_vel_empty;

        // 2. Slope limit
        if (std::abs(context.slope_angle) > params_.max_slope) {
            RCLCPP_WARN(logger_, "Slope %.1f° exceeds max %.1f° - reducing speed",
                        context.slope_angle * 180.0 / M_PI,
                        params_.max_slope * 180.0 / M_PI);
            max_linear_vel *= 0.5;  // 50% speed on steep slopes
        }

        // 3. Weather conditions
        if (context.weather == Weather::LIGHT_RAIN) {
            max_linear_vel *= 0.7;  // 70% speed in rain
        }

        // 4. Proximity to obstacles
        if (context.min_obstacle_distance < 2.0) {
            double proximity_factor = context.min_obstacle_distance / 2.0;
            max_linear_vel *= proximity_factor;
        }

        // Apply limits
        double linear_vel = std::sqrt(
            cmd_vel.linear.x * cmd_vel.linear.x +
            cmd_vel.linear.y * cmd_vel.linear.y);

        if (linear_vel > max_linear_vel) {
            double scale = max_linear_vel / linear_vel;
            limited_cmd.linear.x *= scale;
            limited_cmd.linear.y *= scale;
        }

        // Angular velocity limit
        limited_cmd.angular.z = std::clamp(
            cmd_vel.angular.z,
            -params_.max_angular_vel,
            params_.max_angular_vel);

        return limited_cmd;
    }

private:
    struct Parameters {
        double max_vel_empty = 1.5;          // m/s
        double max_vel_with_passenger = 1.0; // m/s
        double max_angular_vel = 2.0;        // rad/s
        double max_slope = 0.175;            // radians (10°)
    };

    Parameters params_;
};
```

---

## 3. Safety State Machine

```
┌────────────────────────────────────────────────────────────┐
│              Safety State Machine                          │
└────────────────────────────────────────────────────────────┘

    ┌──────────┐
    │  NORMAL  │ ← All systems operational
    └────┬─────┘
         │
         ├─→ SUBSYSTEM_FAILURE → ┌─────────────┐
         │                        │ DEGRADED    │
         │                        └─────────────┘
         │
         ├─→ OBSTACLE_DETECTED → ┌──────────────┐
         │                       │ COLLISION    │
         │                       │ AVOIDANCE    │
         │                       └──────────────┘
         │
         └─→ E_STOP_PRESSED → ┌────────────┐
                              │ EMERGENCY  │
                              │ STOP       │
                              └────────────┘

Recovery:
    EMERGENCY_STOP → (manual reset) → NORMAL
    DEGRADED → (subsystem recovery) → NORMAL
    COLLISION_AVOIDANCE → (obstacle cleared) → NORMAL
```

---

## 4. Configuration

**safety_params.yaml:**
```yaml
safety_monitor:
  ros__parameters:
    # Velocity limits
    max_linear_velocity_empty: 1.5        # m/s
    max_linear_velocity_passenger: 1.0    # m/s
    max_angular_velocity: 2.0             # rad/s
    max_acceleration: 0.5                 # m/s²

    # Slope limits
    max_slope_angle: 0.175                # radians (10°)
    max_slope_angle_passenger: 0.087      # radians (5°)

    # Collision prediction
    prediction_horizon: 3.0               # seconds
    min_obstacle_distance: 0.5            # meters (emergency stop)
    warning_obstacle_distance: 2.0        # meters (slow down)

    # Watchdog timeouts
    subsystem_timeouts:
      swerve_controller: 1.0              # seconds
      ndt_localization: 2.0
      perception: 1.0
      navigation: 5.0

    # Weather operations
    allow_light_rain: true
    allow_heavy_rain: false
    max_wind_speed: 10.0                  # m/s
```

---

## 5. Testing Strategy

**Safety Test Example:**
```cpp
TEST(SafetyMonitorTest, EmergencyStopActivation) {
    SafetyMonitor monitor;

    // Simulate emergency stop
    std_msgs::msg::Bool estop_msg;
    estop_msg.data = true;
    monitor.emergencyStopCallback(
        std::make_shared<std_msgs::msg::Bool>(estop_msg));

    // Verify state
    EXPECT_EQ(monitor.getSafetyState(), SafetyState::EMERGENCY_STOP);

    // Verify zero velocity published
    auto cmd_vel = monitor.getLastCommandVelocity();
    EXPECT_DOUBLE_EQ(cmd_vel.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(cmd_vel.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(cmd_vel.angular.z, 0.0);
}

TEST(VelocityGovernorTest, PassengerSpeedLimit) {
    VelocityGovernor governor;

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 2.0;  // Request 2.0 m/s

    SafetyContext context;
    context.passenger_onboard = true;

    auto limited_cmd = governor.limitVelocity(cmd_vel, context);

    // Should be limited to 1.0 m/s with passenger
    EXPECT_LE(limited_cmd.linear.x, 1.0);
}
```

---

**Document Status:** Draft
**Implementation Status:** Ready for development
**Approvals Required:** Safety Lead, Systems Engineer
