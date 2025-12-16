# Navigation Subsystem Architecture

**Document ID:** ARCH-NAV-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft

---

## 1. Overview

The **Navigation Subsystem** provides autonomous navigation capabilities for 500m-1km routes without GPS. This architecture is **60-70% reused from ParcelPal**, adapted for swerve drive and wheelchair transport requirements.

**Key Capabilities:**
- Two-phase navigation (offline mapping + online localization)
- NDT scan matching for GPS-free localization (±10cm over 500m)
- Multi-waypoint route execution (4-8 stops per route)
- Omnidirectional path planning (swerve drive capability)
- Dynamic obstacle avoidance
- Recovery behaviors for failure handling

**Localization Strategy (ParcelPal Proven):**
- **Phase 1 (Offline):** Manual SLAM mapping once with loop closure → save static map
- **Phase 2 (Online):** NDT localization on saved map → NO drift accumulation!

---

## 2. System Architecture

### 2.1 Component Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                    Navigation Subsystem                          │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                  Nav2 Stack (ROS 2)                        │ │
│  │                                                            │ │
│  │  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐│ │
│  │  │ BT Navigator │───▶│   Planner    │───▶│  Controller  ││ │
│  │  │ (Behavior    │    │   Server     │    │   Server     ││ │
│  │  │  Tree)       │    │              │    │              ││ │
│  │  │              │    │ - NavFn      │    │ - DWB        ││ │
│  │  │ - Waypoint   │    │ - Smac       │    │ - Swerve     ││ │
│  │  │   Following  │    │   Hybrid-A*  │    │   Controller ││ │
│  │  │ - Recovery   │    │              │    │              ││ │
│  │  │   Behaviors  │    └──────┬───────┘    └──────┬───────┘│ │
│  │  └──────────────┘           │                   │        │ │
│  │                              │ global path       │ /cmd_vel│
│  │                              ↓                   ↓        │ │
│  └──────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────┐         ┌────────────────┐                 │
│  │  Localization  │────────▶│   Costmap 2D   │                 │
│  │  (NDT from     │         │   (Perception  │                 │
│  │   Autoware)    │  pose   │    Integration)│                 │
│  │                │         │                │                 │
│  │ - NDT Scan     │         │ - Global Map   │                 │
│  │   Matching     │         │ - Local Costmap│                 │
│  │ - Particle     │         │ - Inflation    │                 │
│  │   Filter       │         │                │                 │
│  └────────┬───────┘         └────────┬───────┘                 │
│           │                          │                          │
│           │ /tf (map→odom→base_link) │ /map, /points           │
│           ↓                          ↓                          │
│  ┌────────────────┐         ┌────────────────┐                 │
│  │ 3D LiDAR       │         │  Perception    │                 │
│  │ (NDT source)   │         │  (Obstacles)   │                 │
│  └────────────────┘         └────────────────┘                 │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                  Route Manager (NEW)                       │ │
│  │                                                            │ │
│  │  - Multi-waypoint route planning (500m-1km)               │ │
│  │  - Waypoint queue management (4-8 stops)                  │ │
│  │  - Re-routing on failure                                  │ │
│  │  - Mission progress tracking                              │ │
│  └────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────┘
```

---

## 3. ROS 2 Node Architecture

### 3.1 Nav2 Stack Nodes

**Standard Nav2 Nodes (from ros-humble-navigation2):**

1. **BT Navigator** (`bt_navigator`)
   - Manages behavior tree execution
   - Coordinates planner, controller, recovery servers
   - Handles waypoint following

2. **Planner Server** (`planner_server`)
   - Generates global path from start to goal
   - Plugin architecture: NavFn, Smac Planner Hybrid-A*
   - Publishes path visualization

3. **Controller Server** (`controller_server`)
   - Follows global path with local trajectory optimization
   - Plugin architecture: DWB, TEB, **Swerve Controller (custom)**
   - Publishes `/cmd_vel` commands

4. **Recovery Server** (`recoveries_server`)
   - Executes recovery behaviors on failure
   - Behaviors: spin, backup, wait

5. **Waypoint Follower** (`waypoint_follower`)
   - Navigates through list of waypoints
   - Action interface: `nav2_msgs/action/FollowWaypoints`

6. **Lifecycle Manager** (`lifecycle_manager`)
   - Manages lifecycle of Nav2 nodes
   - Startup, shutdown, recovery

---

### 3.2 Localization Node (NDT from Autoware)

**Node Name:** `ndt_localization`
**Package:** `autoware_localization` (or custom wrapper)
**Language:** C++

**Subscribed Topics:**
- `/points` (sensor_msgs/PointCloud2): 3D LiDAR point cloud (10 Hz)
- `/imu/data` (sensor_msgs/Imu): IMU for orientation (50 Hz)
- `/odom` (nav_msgs/Odometry): Wheel odometry (50 Hz)

**Published Topics:**
- `/tf` (tf2_msgs/TFMessage): map → odom transform
- `/ndt_pose` (geometry_msgs/PoseStamped): Current pose estimate
- `/ndt_score` (std_msgs/Float32): NDT alignment score (convergence quality)

**Services:**
- `/ndt/set_initial_pose` (geometry_msgs/PoseWithCovarianceStamped): Manual initialization

**Parameters:**
```yaml
# NDT configuration (Autoware defaults)
ndt:
  trans_epsilon: 0.01             # Convergence threshold (translation)
  step_size: 0.1                  # Newton optimization step size
  resolution: 1.0                 # Voxel grid resolution (meters)
  max_iterations: 30              # Max optimization iterations
  
  # Performance
  num_threads: 4                  # Parallel execution
  
  # Initialization
  initial_pose_topic: "/initialpose"  # From RViz or UI
  use_imu: true                   # IMU-aided initialization
  use_odom: true                  # Odom-aided prediction

# Map
map_topic: "/map"                 # Pre-built static map
map_frame: "map"
base_frame: "base_link"
odom_frame: "odom"
```

---

### 3.3 Route Manager Node (NEW)

**Node Name:** `route_manager`
**Language:** C++ or Python
**Lifecycle:** Standard node

**Subscribed Topics:**
- `/ndt_pose` (geometry_msgs/PoseStamped): Current robot pose
- `/waypoint_follower/feedback` (nav2_msgs/action/FollowWaypoints/Feedback): Navigation progress

**Published Topics:**
- `/route_status` (custom_msgs/RouteStatus): Route progress (waypoints completed, ETA)
- `/mission_markers` (visualization_msgs/MarkerArray): Waypoint visualization

**Action Servers:**
- `/execute_route` (custom_msgs/action/ExecuteRoute): Execute multi-stop route

**Action Clients:**
- `/follow_waypoints` (nav2_msgs/action/FollowWaypoints): Nav2 waypoint follower

**Parameters:**
```yaml
# Route configuration
max_waypoints_per_route: 8        # Maximum stops per mission
waypoint_tolerance: 0.3           # meters (reached threshold)
reroute_on_failure: true          # Attempt alternate path on failure
max_reroute_attempts: 3           # Give up after N failures

# Waypoint database
waypoint_file: "config/waypoints.yaml"  # Predefined waypoint locations
```

---

## 4. Two-Phase Navigation Strategy

### 4.1 Phase 1: Offline Mapping (Manual, One-Time)

**Goal:** Create high-quality static map with loop closure

**Procedure:**
```bash
# 1. Launch SLAM Toolbox in mapping mode
ros2 launch slam_toolbox online_async_launch.py

# 2. Manually drive robot through entire operational area (500m-1km)
#    - Drive slowly (0.5 m/s) for high-quality scans
#    - Revisit start location for loop closure
#    - Cover all paths, docking stations, waypoints

# 3. Save map after loop closure converges
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: {data: 'campus_map'}"

# 4. Map files generated:
#    - campus_map.yaml (map metadata)
#    - campus_map.pgm (occupancy grid image)
```

**Map Quality Verification:**
- Loop closure error: <10cm (check SLAM Toolbox logs)
- No ghost walls or duplicate features
- All docking stations visible on map
- Waypoints correspond to known locations

---

### 4.2 Phase 2: Online Localization (Operational Mode)

**Goal:** Localize robot on saved map with <±10cm accuracy

**Procedure:**
```bash
# 1. Load pre-built map
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=campus_map.yaml

# 2. Launch NDT localization
ros2 launch autoware_localization ndt_localization.launch.py

# 3. Initialize robot pose (one of these methods):
#    a) Manual initialization via RViz "2D Pose Estimate" tool
#    b) Automatic initialization from last known pose (config/last_pose.yaml)
#    c) Multi-hypothesis particle filter initialization (if very uncertain)

# 4. NDT converges to accurate pose (usually <5 seconds)
#    - Monitor /ndt_score topic (should be >0.5 for good alignment)
#    - Verify pose in RViz (LiDAR scan overlays map)
```

**Localization Monitoring:**
```cpp
void monitorLocalization() {
    // Subscribe to /ndt_score
    double ndt_score = getCurrentNDTScore();
    
    if (ndt_score < 0.3) {
        // Poor alignment - localization may be lost
        triggerRelocalization();
    }
    
    // Also monitor pose covariance
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    double position_std = sqrt(pose.pose.covariance[0]);  // x variance
    
    if (position_std > 0.5) {
        // High uncertainty - localization degraded
        reduceSpeed();  // Safety measure
    }
}
```

---

## 5. Path Planning

### 5.1 Global Planner (NavFn / Smac Hybrid-A*)

**Algorithm Selection:**
- **NavFn (Dijkstra):** Fast, simple, holonomic assumption (good for swerve drive)
- **Smac Hybrid-A*:** Better for constrained spaces, considers robot footprint

**Configuration (nav2_params.yaml):**
```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5                # Goal tolerance (meters)
      use_astar: false              # Dijkstra (faster for outdoor open spaces)
      allow_unknown: true           # Plan through unknown space (outdoor expansion)
```

**Alternative (for complex indoor/outdoor transitions):**
```yaml
    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      tolerance: 0.5
      downsample_costmap: false     # Use full resolution
      allow_unknown: true
      max_planning_time: 5.0        # seconds
```

---

### 5.2 Local Controller (DWB / Swerve Controller)

**DWB Controller (Standard Nav2):**
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0    # Hz
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      # Velocity limits (omnidirectional!)
      max_vel_x: 1.5
      max_vel_y: 1.5              # Swerve drive can strafe
      max_vel_theta: 2.0
      min_vel_x: -1.5
      min_vel_y: -1.5
      
      # Trajectory generation
      sim_time: 1.7               # Seconds to simulate ahead
      vx_samples: 20
      vy_samples: 20              # Sample lateral velocities
      vtheta_samples: 40
      
      # Trajectory scoring
      path_distance_bias: 32.0    # Prefer staying on global path
      goal_distance_bias: 24.0    # Prefer heading toward goal
      occdist_scale: 0.01         # Avoid obstacles
```

**Custom Swerve Controller (from SWERVE_DRIVE_ARCHITECTURE.md):**
```yaml
    FollowPath:
      plugin: "swerve_drive_controller::SwerveDriveController"
      # See SWERVE_DRIVE_ARCHITECTURE.md for full configuration
```

---

## 6. Multi-Waypoint Navigation

### 6.1 Waypoint Definition

**Waypoint File (config/waypoints.yaml):**
```yaml
waypoints:
  - id: "building_a_entrance"
    pose:
      position: {x: 10.5, y: 20.3, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}  # Facing east
    type: "pickup"
    
  - id: "building_b_entrance"
    pose:
      position: {x: 150.2, y: 35.8, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}  # Facing north
    type: "dropoff"
    
  - id: "building_c_entrance"
    pose:
      position: {x: 280.7, y: -12.4, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: -0.707, w: 0.707}  # Facing west
    type: "dropoff"
```

---

### 6.2 Route Execution

**Action Interface:**
```cpp
// custom_msgs/action/ExecuteRoute.action
# Goal
string[] waypoint_ids     # Ordered list of waypoint IDs to visit
---
# Result
bool success
string message
int32 waypoints_completed
---
# Feedback
int32 current_waypoint_index
string current_waypoint_id
float32 distance_remaining  # meters to current waypoint
float32 eta_seconds
```

**Route Manager Implementation:**
```cpp
void executeRoute(const ExecuteRoute::Goal& goal) {
    // 1. Load waypoints from config
    std::vector<geometry_msgs::msg::PoseStamped> waypoint_poses;
    for (const auto& waypoint_id : goal.waypoint_ids) {
        waypoint_poses.push_back(loadWaypoint(waypoint_id));
    }

    // 2. Call Nav2 waypoint follower
    nav2_msgs::action::FollowWaypoints::Goal nav2_goal;
    nav2_goal.poses = waypoint_poses;
    
    auto nav2_result = waypoint_follower_client_->sendGoal(nav2_goal);

    // 3. Monitor progress and publish feedback
    while (!nav2_result->is_finished()) {
        ExecuteRoute::Feedback feedback;
        feedback.current_waypoint_index = getCurrentWaypointIndex();
        feedback.distance_remaining = computeDistanceToWaypoint();
        feedback.eta_seconds = estimateETA();
        sendFeedback(feedback);
    }

    // 4. Handle result
    if (nav2_result->success) {
        ExecuteRoute::Result result;
        result.success = true;
        result.waypoints_completed = goal.waypoint_ids.size();
        sendResult(result);
    } else {
        // Re-routing or failure handling
        handleNavigationFailure(nav2_result->message);
    }
}
```

---

## 7. Costmap Configuration

### 7.1 Global Costmap (Static Map)

**Purpose:** Long-range path planning on pre-built map

**Configuration:**
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0       # Hz (static map, infrequent updates)
      publish_frequency: 1.0
      robot_base_frame: base_link
      global_frame: map
      
      # Map plugin
      plugins: ["static_layer", "inflation_layer"]
      
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: /map
        
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.5     # meters (safety buffer)
        cost_scaling_factor: 3.0
```

---

### 7.2 Local Costmap (Dynamic Obstacles)

**Purpose:** Real-time obstacle avoidance from LiDAR

**Configuration:**
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0       # Hz
      publish_frequency: 2.0
      robot_base_frame: base_link
      global_frame: odom
      
      width: 5                    # meters (local window)
      height: 5
      resolution: 0.05            # meters/pixel (5cm)
      
      # Plugins
      plugins: ["voxel_layer", "inflation_layer"]
      
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.2         # meters (vertical resolution)
        z_voxels: 16              # 3.2m vertical range
        max_obstacle_height: 2.0  # meters (ignore overhead obstacles)
        min_obstacle_height: 0.1  # meters (ignore ground)
        mark_threshold: 2         # Min points to mark obstacle
        
        # Point cloud source
        observation_sources: scan
        scan:
          topic: /points
          sensor_frame: lidar_frame
          data_type: PointCloud2
          marking: true
          clearing: true
          min_obstacle_height: 0.1
          max_obstacle_height: 2.0
      
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.3     # meters (tight for indoor, wider for outdoor)
        cost_scaling_factor: 3.0
```

---

## 8. Recovery Behaviors

### 8.1 Failure Scenarios

**Common Failure Modes:**
1. **Path blocked:** Temporary obstacle on path
2. **Localization lost:** NDT score drops, high covariance
3. **Goal unreachable:** No valid path to goal
4. **Robot stuck:** Wheels slipping, no progress

---

### 8.2 Recovery Actions

**Configuration:**
```yaml
recoveries_server:
  ros__parameters:
    recovery_plugins: ["spin", "backup", "wait"]
    
    spin:
      plugin: "nav2_recoveries/Spin"
      simulate_ahead_time: 2.0
      
    backup:
      plugin: "nav2_recoveries/BackUp"
      simulate_ahead_time: 2.0
      
    wait:
      plugin: "nav2_recoveries/Wait"
      wait_duration: 5.0          # seconds
```

**Behavior Tree Integration:**
```xml
<!-- Nav2 default behavior tree (excerpt) -->
<BehaviorTree>
  <PipelineSequence name="NavigateWithReplanning">
    <RateController hz="1.0">
      <RecoveryNode number_of_retries="6" name="NavigateRecovery">
        <PipelineSequence name="NavigateWithReplanning">
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
          <FollowPath path="{path}" controller_id="FollowPath"/>
        </PipelineSequence>
        <ReactiveFallback name="RecoveryFallback">
          <Spin spin_dist="1.57"/>        <!-- 90° spin -->
          <Wait wait_duration="5.0"/>
          <BackUp backup_dist="0.3" backup_speed="0.1"/>
        </ReactiveFallback>
      </RecoveryNode>
    </RateController>
  </PipelineSequence>
</BehaviorTree>
```

---

## 9. Integration with Other Subsystems

### 9.1 Localization Integration

**Data Flow:**
- NDT localization publishes `map → odom` TF transform
- Nav2 receives robot pose via TF lookup (`map → base_link`)
- Odometry from swerve drive publishes `odom → base_link` TF

**TF Tree:**
```
map (from NDT, static map frame)
 └─ odom (from NDT, corrected by NDT)
     └─ base_link (from odometry, wheel encoders)
         └─ lidar_frame (static, from URDF)
         └─ camera_1_frame (static, from URDF)
```

---

### 9.2 Perception Integration

**Obstacle Detection:**
- Perception subsystem publishes `/points` (filtered 3D LiDAR)
- Local costmap's `voxel_layer` consumes `/points`
- Obstacles marked in costmap → controller avoids

**Ground Removal:**
- Perception performs ground plane removal (see PERCEPTION_SUBSYSTEM_ARCHITECTURE.md)
- Only obstacles (height 0.1m - 2.0m) reach costmap

---

### 9.3 Swerve Drive Integration

**Command Interface:**
- Controller server publishes `/cmd_vel` (geometry_msgs/Twist)
- Swerve drive controller subscribes to `/cmd_vel`
- Swerve drive executes omnidirectional motion (vx, vy, ω)

**Advantages:**
- Lateral motion (vy) enables tight cornering without rotation
- DWB controller can sample vy trajectories

---

## 10. Testing Strategy

### 10.1 Unit Tests
- Path planning correctness (known map → expected path)
- NDT localization accuracy (simulated scans → known pose)
- Waypoint loading and parsing

### 10.2 Integration Tests
- Nav2 stack integration (all nodes running)
- Localization + navigation (simulated map + LiDAR)
- Multi-waypoint execution (3-5 waypoints)

### 10.3 Field Tests
- Mapping quality (loop closure error <10cm)
- Localization accuracy over distance (±10cm over 500m)
- Path following accuracy (cross-track error <20cm)
- Obstacle avoidance (dynamic obstacles)
- Multi-waypoint routes (4-8 stops, 500m-1km total)
- Recovery behavior effectiveness (blocked path, stuck robot)

---

## 11. Performance Targets

| Metric | Target | Validation |
|--------|--------|------------|
| Localization Accuracy | ±10cm over 500m | GPS ground truth or surveyed waypoints |
| Path Following Accuracy | <20cm cross-track error | Logged path vs. planned path |
| Planning Time (Global) | <1s for 500m path | Timestamp analysis |
| Control Loop Rate | 20 Hz | Timestamp analysis |
| Obstacle Reaction Time | <500ms | Dynamic obstacle test |
| Waypoint Reached Threshold | ±30cm | Field tests |
| Multi-Waypoint Success Rate | >90% (500m-1km routes) | Field tests (50 routes) |

---

## 12. Implementation Phases

### Phase 1: Localization (Sprints 1-2)
- ✅ NDT localization integration (Autoware)
- ✅ Manual SLAM mapping procedure
- ✅ TF tree configuration
- ✅ Localization monitoring

### Phase 2: Nav2 Integration (Sprints 3-4)
- ✅ Nav2 stack configuration
- ✅ Global/local costmap setup
- ✅ Path planning (NavFn)
- ✅ Local control (DWB or Swerve)

### Phase 3: Multi-Waypoint Navigation (Sprints 5-6)
- ✅ Route manager node
- ✅ Waypoint database
- ✅ Nav2 waypoint follower integration
- ✅ Re-routing on failure

### Phase 4: Field Validation (Sprints 7-8)
- ✅ Campus mapping (500m-1km)
- ✅ Localization accuracy testing
- ✅ Multi-waypoint route execution
- ✅ Recovery behavior tuning

---

**Document Status:** Draft
**Implementation Status:** Not Started
**Approvals Required:** Navigation Lead, Localization Engineer, Route Planning Engineer, System Architect
