# Swerve Drive Subsystem Architecture

**Document ID:** ARCH-SWERVE-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft

---

## 1. Overview

The **Swerve Drive Subsystem** provides omnidirectional motion control for the wheelchair transport robot. Unlike ParcelPal's differential drive, this subsystem enables holonomic motion (independent vx, vy, ω control) for precise maneuvering in tight spaces.

**Key Capabilities:**
- 4-wheel independent swerve modules (drive + steering per wheel)
- Omnidirectional motion (move in any direction without rotation)
- Nav2 controller integration
- Real-time inverse kinematics
- Safety-compliant velocity limiting

---

## 2. Component Architecture

### 2.1 System Components

```
┌─────────────────────────────────────────────────────────┐
│              Swerve Drive Subsystem                     │
│                                                         │
│  ┌──────────────────┐      ┌──────────────────┐       │
│  │  Nav2 Controller │─────▶│  Swerve Drive    │       │
│  │  (DWB Planner)   │      │  Controller Node │       │
│  └──────────────────┘      │  (C++)           │       │
│         │                   │                  │       │
│         │ /cmd_vel          │  - IK Solver     │       │
│         │ (Twist)           │  - Wheel Coord   │       │
│         └──────────────────▶│  - Velocity Gov  │       │
│                             └────────┬─────────┘       │
│                                      │                  │
│                                      │ Individual       │
│                                      │ wheel commands   │
│                                      ↓                  │
│         ┌─────────────┬──────────────┴──────────┬──────────────┐
│         ▼             ▼                         ▼              ▼
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐      │
│  │ Module   │  │ Module   │  │ Module   │  │ Module   │      │
│  │ FL       │  │ FR       │  │ RL       │  │ RR       │      │
│  │ (Driver) │  │ (Driver) │  │ (Driver) │  │ (Driver) │      │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘      │
│       │             │              │             │             │
│       ↓             ↓              ↓             ↓             │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐      │
│  │ Motor    │  │ Motor    │  │ Motor    │  │ Motor    │      │
│  │ Controller│  │ Controller│  │ Controller│  │ Controller│     │
│  │ + Encoder│  │ + Encoder│  │ + Encoder│  │ + Encoder│      │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘      │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**Components:**
1. **Swerve Drive Controller Node** (C++): Main controller implementing nav2_core::Controller interface
2. **Module Driver Nodes** (4×): Per-wheel control nodes (drive motor + steering motor)
3. **Motor Controllers** (8×): Hardware controllers (4 drive + 4 steering)
4. **Encoders** (8×): Position feedback (4 wheel speed + 4 steering angle)

---

## 3. ROS 2 Node Architecture

### 3.1 Swerve Drive Controller Node

**Node Name:** `swerve_drive_controller`
**Language:** C++ (performance-critical)
**Lifecycle:** Managed lifecycle node

**Subscribed Topics:**
- `/cmd_vel` (geometry_msgs/Twist): Desired body velocities (vx, vy, ω)
- `/odom` (nav_msgs/Odometry): Current robot odometry
- `/joint_states` (sensor_msgs/JointState): Wheel positions and velocities

**Published Topics:**
- `/wheel_commands` (custom_msgs/SwerveModuleCommand): Per-wheel drive/steering commands
- `/swerve/status` (custom_msgs/SwerveStatus): Controller status and diagnostics
- `/tf` (tf2_msgs/TFMessage): odom → base_link transform

**Services:**
- `/swerve/reset_odometry` (std_srvs/Trigger): Reset odometry to (0,0,0)

**Parameters:**
```yaml
wheel_base: 0.6       # meters (front-rear distance)
track_width: 0.5      # meters (left-right distance)
wheel_radius: 0.0825  # meters (6.5" wheels)
max_linear_velocity: 1.5   # m/s
max_angular_velocity: 2.0  # rad/s
max_linear_acceleration: 0.5  # m/s²
```

---

### 3.2 Module Driver Nodes (×4)

**Node Names:** `swerve_module_fl`, `swerve_module_fr`, `swerve_module_rl`, `swerve_module_rr`
**Language:** C++ or Python
**Lifecycle:** Standard nodes

**Subscribed Topics:**
- `/wheel_commands` (custom_msgs/SwerveModuleCommand): Desired module state

**Published Topics:**
- `/joint_states` (sensor_msgs/JointState): Current wheel state (position, velocity)

**Hardware Interface:**
- CAN bus or serial to motor controllers
- Reads encoder values
- Sends velocity commands

---

## 4. Inverse Kinematics Algorithm

### 4.1 Mathematical Model

**Input:** Body velocity command `(vx, vy, ω)` in robot frame

**Output:** Individual wheel velocities and steering angles for 4 modules

**Equations:**

For each wheel `i` at position `(xi, yi)`:

```
# Wheel velocity contribution from translation and rotation
vxi = vx - ω * yi
vyi = vy + ω * xi

# Wheel speed (magnitude)
vi = sqrt(vxi² + vyi²)

# Steering angle (direction)
θi = atan2(vyi, vxi)
```

**Module Positions:**
```
FL: ( L/2,  W/2)
FR: ( L/2, -W/2)
RL: (-L/2,  W/2)
RR: (-L/2, -W/2)

Where:
L = wheel_base = 0.6m
W = track_width = 0.5m
```

---

### 4.2 Wheel Coordination

**Challenge:** Avoid module "fight" during steering transitions

**Solution:** Coordinate all modules to reach target angles before applying drive force

**Algorithm:**
```cpp
void updateModules(SwerveCommand cmd) {
    // 1. Compute target angles for all modules
    for (int i = 0; i < 4; i++) {
        target_angles[i] = computeSteeringAngle(cmd, module_pos[i]);
    }

    // 2. Check if any module needs >90° rotation
    //    If yes, flip angle and reverse drive direction (optimization)
    for (int i = 0; i < 4; i++) {
        double angle_error = target_angles[i] - current_angles[i];
        if (abs(angle_error) > M_PI/2) {
            target_angles[i] += M_PI;  // Flip 180°
            drive_directions[i] *= -1;  // Reverse drive
        }
    }

    // 3. Wait for all modules to reach target angles (within tolerance)
    bool all_aligned = true;
    for (int i = 0; i < 4; i++) {
        if (abs(target_angles[i] - current_angles[i]) > angle_tolerance) {
            all_aligned = false;
            break;
        }
    }

    // 4. Apply drive velocities only when aligned
    if (all_aligned) {
        for (int i = 0; i < 4; i++) {
            sendDriveCommand(i, target_velocities[i] * drive_directions[i]);
        }
    } else {
        // Ramp down drive while steering
        for (int i = 0; i < 4; i++) {
            sendDriveCommand(i, 0.0);
        }
    }
}
```

---

## 5. Nav2 Integration

### 5.1 Controller Plugin

**Interface:** `nav2_core::Controller`

**Implementation:**
```cpp
class SwerveDriveController : public nav2_core::Controller {
public:
    void configure(/* ... */) override;
    void cleanup() override;
    void activate() override;
    void deactivate() override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;

    void setPlan(const nav_msgs::msg::Path & path) override;
};
```

**Key Methods:**
- `computeVelocityCommands()`: Compute body velocities to follow path
- `setPlan()`: Receive path from global planner

---

### 5.2 Configuration (nav2_params.yaml)

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "swerve_drive_controller::SwerveDriveController"
      # Swerve-specific parameters
      wheel_base: 0.6
      track_width: 0.5
      # DWB trajectory parameters
      max_vel_x: 1.5
      max_vel_y: 1.5  # Omnidirectional!
      max_vel_theta: 2.0
      min_vel_x: -1.5
      min_vel_y: -1.5
      min_vel_theta: -2.0
      # Trajectory scoring
      goal_distance_bias: 20.0
      path_distance_bias: 32.0
      occdist_scale: 0.02
```

---

## 6. Odometry Computation

### 6.1 Forward Kinematics

**Input:** Individual wheel velocities and steering angles

**Output:** Robot body velocity `(vx, vy, ω)`

**Method:** Least-squares solution (overdetermined system with 4 wheels)

```cpp
Eigen::Vector3d computeOdometry(
    const std::array<double, 4>& wheel_speeds,
    const std::array<double, 4>& steering_angles) {

    // Build Jacobian matrix (4 wheels × 3 DOF)
    Eigen::Matrix<double, 4, 3> J;
    for (int i = 0; i < 4; i++) {
        double xi = module_positions[i].x;
        double yi = module_positions[i].y;
        double theta = steering_angles[i];

        J(i, 0) = cos(theta);           // vx contribution
        J(i, 1) = sin(theta);           // vy contribution
        J(i, 2) = -yi*cos(theta) + xi*sin(theta);  // ω contribution
    }

    // Measured wheel velocities
    Eigen::Vector4d v_wheels(wheel_speeds.data());

    // Least-squares solution: v_body = (J^T J)^-1 J^T v_wheels
    Eigen::Vector3d v_body = (J.transpose() * J).inverse() * J.transpose() * v_wheels;

    return v_body;  // (vx, vy, ω)
}
```

---

### 6.2 Odometry Integration

**Update Rate:** 50 Hz (from wheel encoders)

**Integration:**
```cpp
void updateOdometry(double dt) {
    Eigen::Vector3d v_body = computeOdometry(wheel_speeds, steering_angles);

    double vx = v_body(0);
    double vy = v_body(1);
    double omega = v_body(2);

    // Update pose (in odom frame)
    theta += omega * dt;
    x += (vx * cos(theta) - vy * sin(theta)) * dt;
    y += (vx * sin(theta) + vy * cos(theta)) * dt;

    // Publish odometry
    publishOdometry(x, y, theta, vx, vy, omega);
}
```

---

## 7. Safety Integration

### 7.1 Velocity Governor

**Purpose:** Enforce safety limits on commanded velocities

**Limits:**
- Max linear speed: 1.5 m/s (no passenger), 1.0 m/s (with passenger)
- Max angular speed: 2.0 rad/s
- Max acceleration: 0.5 m/s²

**Implementation:**
```cpp
geometry_msgs::msg::Twist limitVelocity(
    const geometry_msgs::msg::Twist& cmd_vel,
    bool passenger_onboard) {

    geometry_msgs::msg::Twist limited = cmd_vel;

    // Speed limits
    double max_linear = passenger_onboard ? 1.0 : 1.5;
    double linear_mag = sqrt(cmd_vel.linear.x * cmd_vel.linear.x +
                             cmd_vel.linear.y * cmd_vel.linear.y);
    if (linear_mag > max_linear) {
        double scale = max_linear / linear_mag;
        limited.linear.x *= scale;
        limited.linear.y *= scale;
    }

    // Angular limit
    limited.angular.z = std::clamp(cmd_vel.angular.z, -2.0, 2.0);

    // Acceleration limit (compare to previous command)
    double accel_dt = 0.05;  // 20 Hz controller
    double max_delta_v = max_accel * accel_dt;
    // ... (limit delta from previous command)

    return limited;
}
```

---

### 7.2 Emergency Stop

**Behavior:** Immediately stop all wheel motion

**Implementation:**
```cpp
void emergencyStop() {
    // Send zero velocity to all modules
    for (int i = 0; i < 4; i++) {
        sendDriveCommand(i, 0.0);
        // Keep current steering angle (don't reset to avoid sudden motion)
    }

    // Engage brakes
    engageBrakes();
}
```

---

## 8. Testing Strategy

### 8.1 Unit Tests
- Inverse kinematics correctness (known inputs → expected outputs)
- Forward kinematics (wheel velocities → body velocity)
- Velocity limiting logic
- Module coordination algorithm

### 8.2 Integration Tests
- Nav2 controller interface
- ROS 2 topic communication
- Emergency stop response time
- Odometry accuracy (drive square, measure error)

### 8.3 Field Tests
- Omnidirectional motion (strafe left/right, diagonal)
- Precision positioning (±10cm accuracy)
- Smooth transitions (no jerky motion during steering changes)
- Performance under load (100kg payload)

---

## 9. Performance Targets

| Metric | Target | Validation |
|--------|--------|------------|
| Control Loop Rate | 20 Hz | Timestamp analysis |
| IK Computation Time | <5ms | Profiling |
| Module Coordination Time | <200ms | Motion capture |
| Odometry Drift | <2% distance | Square path test |
| Max Linear Speed | 1.5 m/s | Speed test |
| Max Angular Speed | 2.0 rad/s | Rotation test |
| Acceleration | 0.5 m/s² | Acceleration test |

---

## 10. Implementation Phases

### Phase 1: Basic Controller (Sprints 1-2)
- ✅ Inverse kinematics implementation
- ✅ Basic module drivers (simulated hardware)
- ✅ Odometry computation
- ✅ Unit tests

### Phase 2: Nav2 Integration (Sprints 3-4)
- ✅ Controller plugin interface
- ✅ DWB planner integration
- ✅ Path following tests
- ✅ Integration with localization

### Phase 3: Safety & Polish (Sprints 5-6)
- ✅ Velocity governor
- ✅ Emergency stop
- ✅ Module coordination refinement
- ✅ Field testing and tuning

---

**Document Status:** Draft
**Implementation Status:** Not Started
**Approvals Required:** Navigation Lead, Swerve Drive Engineer, Safety Engineer
