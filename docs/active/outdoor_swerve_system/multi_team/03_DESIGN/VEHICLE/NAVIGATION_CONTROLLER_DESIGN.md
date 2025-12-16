# Navigation Controller Detailed Design

**Document ID:** DESIGN-NAV-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft

---

## 1. Overview

This document provides detailed design specifications for the Navigation Controller, implementing the proven ParcelPal 2-phase navigation strategy: offline SLAM mapping + online NDT localization for GPS-free long-range navigation (500m-1km).

**Key Components:**
- NavigationManager (mission coordinator)
- NDTLocalizationNode (map-based localization)
- RouteManager (multi-waypoint route execution)
- Nav2 Integration (global/local planning, recovery behaviors)
- MapManager (map loading, switching, validation)

**Key Features:**
- NO GPS/GNSS (cost savings: $1,500-3,000)
- NO continuous SLAM (no drift accumulation)
- ±10cm localization accuracy over 500m
- Multi-waypoint route support (4-8 stops per route)
- Proven ParcelPal architecture (60-70% reuse)

---

## 2. Architecture Overview

### 2.1 Two-Phase Navigation Strategy

**Phase 1: Offline Mapping (Manual, One-Time)**
```
Human operator drives robot through area with joystick
    ↓
SLAM algorithm (Cartographer or SLAM Toolbox)
    ↓
Generate 2D occupancy grid map + loop closure
    ↓
Save map to file (map.pgm + map.yaml)
    ↓
DONE - Never run SLAM again for this area
```

**Phase 2: Online Localization (Autonomous)**
```
Load saved map from Phase 1
    ↓
NDT scan matching (current LiDAR scan vs saved map)
    ↓
Estimate robot pose (x, y, θ) in map frame
    ↓
Publish /odom → /map transform
    ↓
Nav2 uses map for global planning
```

**Why This Works Without Drift:**
- SLAM is run ONCE offline with loop closure (NO drift in saved map)
- NDT localizes against FIXED saved map (NO drift accumulation)
- Result: ±10cm accuracy over 500m (proven in ParcelPal)

---

## 3. Class Diagram

```
┌─────────────────────────────────────────────────────────────┐
│              NavigationManager                              │
│              (Mission Coordinator)                          │
├─────────────────────────────────────────────────────────────┤
│ - route_manager_: unique_ptr<RouteManager>                 │
│ - nav2_client_: ActionClient<NavigateToPose>              │
│ - map_manager_: unique_ptr<MapManager>                     │
│ - current_mission_: Mission                                 │
├─────────────────────────────────────────────────────────────┤
│ + startMission(mission)                                     │
│ + pauseMission()                                            │
│ + resumeMission()                                           │
│ + abortMission()                                            │
│ - executeMission()                                          │
│ - handleWaypointComplete()                                  │
│ - handleNavigationFailure()                                 │
└─────────────────────────────────────────────────────────────┘
                        │ uses
                        ↓
┌─────────────────────────────────────────────────────────────┐
│              RouteManager                                   │
│              (Multi-Waypoint Management)                    │
├─────────────────────────────────────────────────────────────┤
│ - waypoints_: vector<Waypoint>                             │
│ - current_waypoint_idx_: int                               │
│ - route_status_: RouteStatus                               │
├─────────────────────────────────────────────────────────────┤
│ + loadRoute(route_file): bool                              │
│ + getNextWaypoint(): Waypoint                              │
│ + markWaypointComplete()                                    │
│ + isRouteComplete(): bool                                   │
│ + getRemainingDistance(): double                            │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│              NDTLocalizationNode                            │
│              (Map-Based Localization)                       │
├─────────────────────────────────────────────────────────────┤
│ - ndt_: pcl::NormalDistributionsTransform                  │
│ - target_cloud_: pcl::PointCloud (saved map)               │
│ - current_pose_: Eigen::Matrix4f                           │
│ - scan_sub_: Subscription<PointCloud2>                     │
├─────────────────────────────────────────────────────────────┤
│ + initialize(map_file): bool                               │
│ + localize(scan): PoseStamped                              │
│ - scanCallback(scan)                                        │
│ - publishTransform()                                        │
│ - checkConvergence(): bool                                  │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│              MapManager                                     │
├─────────────────────────────────────────────────────────────┤
│ - maps_: map<string, OccupancyGrid>                        │
│ - current_map_id_: string                                   │
├─────────────────────────────────────────────────────────────┤
│ + loadMap(map_file): bool                                  │
│ + switchMap(map_id): bool                                  │
│ + validateMap(map): bool                                    │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│              Nav2 Stack Integration                         │
├─────────────────────────────────────────────────────────────┤
│ - BT Navigator (behavior tree executor)                    │
│ - Planner Server (global path planning)                    │
│   - NavFn / Dijkstra planner                               │
│ - Controller Server (local trajectory execution)           │
│   - SwerveDriveController (custom plugin)                  │
│ - Smoother Server (path smoothing)                         │
│ - Recovery Behaviors (stuck recovery, spin, backup)        │
└─────────────────────────────────────────────────────────────┘
```

---

## 4. Detailed Component Specifications

### 4.1 NDTLocalizationNode

**Header (ndt_localization_node.hpp):**
```cpp
#ifndef NDT_LOCALIZATION_NODE_HPP_
#define NDT_LOCALIZATION_NODE_HPP_

#include <memory>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace navigation
{

struct NDTParameters {
  // NDT algorithm parameters
  double resolution;               // meters (e.g., 1.0)
  double step_size;                // meters (e.g., 0.1)
  double transformation_epsilon;   // (e.g., 0.01)
  int max_iterations;             // (e.g., 30)

  // Convergence criteria
  double fitness_score_threshold;  // (e.g., 1.0)
  double max_correspondence_distance; // meters (e.g., 2.0)

  // Performance
  int num_threads;                 // (e.g., 4)

  // Initial pose uncertainty
  Eigen::Vector3d initial_position;  // [x, y, z]
  Eigen::Vector3d initial_orientation; // [roll, pitch, yaw]
};

class NDTLocalizationNode : public rclcpp::Node {
public:
  explicit NDTLocalizationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~NDTLocalizationNode() = default;

private:
  void configure();
  bool loadMapPointCloud(const std::string& map_pcd_file);

  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  bool performNDTAlignment(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
    Eigen::Matrix4f& transformation);

  void publishTransform(const Eigen::Matrix4f& transformation);
  void publishPose(const Eigen::Matrix4f& transformation);
  bool checkConvergence(double fitness_score);

  // NDT solver
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;

  // Map point cloud (target for NDT)
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;

  // Current pose estimate
  Eigen::Matrix4f current_pose_;
  bool pose_initialized_{false};

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Parameters
  NDTParameters params_;
};

}  // namespace navigation

#endif  // NDT_LOCALIZATION_NODE_HPP_
```

**Key Method Implementation:**

**performNDTAlignment:**
```cpp
bool NDTLocalizationNode::performNDTAlignment(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
  Eigen::Matrix4f& transformation)
{
  // Set input cloud (current LiDAR scan)
  ndt_.setInputSource(input_cloud);

  // Initial guess (use previous pose or initial pose)
  Eigen::Matrix4f guess = pose_initialized_ ? current_pose_ : getInitialPoseMatrix();

  // Perform NDT alignment
  pcl::PointCloud<pcl::PointXYZ> output_cloud;
  ndt_.align(output_cloud, guess);

  // Check convergence
  bool converged = ndt_.hasConverged();
  double fitness_score = ndt_.getFitnessScore();

  if (!converged || !checkConvergence(fitness_score)) {
    RCLCPP_WARN(logger_, "NDT did not converge: fitness=%.3f, converged=%d",
                fitness_score, converged);
    return false;
  }

  // Get final transformation
  transformation = ndt_.getFinalTransformation();

  RCLCPP_DEBUG(logger_, "NDT converged: fitness=%.3f, iterations=%d",
               fitness_score, ndt_.getFinalNumIteration());

  return true;
}

bool NDTLocalizationNode::checkConvergence(double fitness_score) {
  // Lower fitness score = better alignment
  // Typical good fitness: <1.0 for outdoor environments
  return fitness_score < params_.fitness_score_threshold;
}
```

**loadMapPointCloud:**
```cpp
bool NDTLocalizationNode::loadMapPointCloud(const std::string& map_pcd_file) {
  target_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_pcd_file, *target_cloud_) == -1) {
    RCLCPP_ERROR(logger_, "Failed to load map point cloud: %s", map_pcd_file.c_str());
    return false;
  }

  RCLCPP_INFO(logger_, "Loaded map point cloud: %zu points", target_cloud_->size());

  // Set target cloud for NDT
  ndt_.setInputTarget(target_cloud_);

  // Configure NDT parameters
  ndt_.setResolution(params_.resolution);
  ndt_.setStepSize(params_.step_size);
  ndt_.setTransformationEpsilon(params_.transformation_epsilon);
  ndt_.setMaximumIterations(params_.max_iterations);

  // Enable multi-threading for performance
  ndt_.setNumThreads(params_.num_threads);

  return true;
}
```

---

### 4.2 RouteManager

**Route File Format (YAML):**
```yaml
route:
  id: "warehouse_to_cafeteria"
  name: "Main Building Route"
  map_id: "main_building_floor_1"

  waypoints:
    - id: 0
      name: "Wheelchair Pickup"
      position:
        x: 10.5
        y: 3.2
        theta: 0.0  # radians
      action: "DOCK_WHEELCHAIR"
      dwell_time: 30.0  # seconds

    - id: 1
      name: "Hallway Checkpoint"
      position:
        x: 25.0
        y: 3.5
        theta: 0.0
      action: "PASS_THROUGH"
      dwell_time: 0.0

    - id: 2
      name: "Elevator Waiting Area"
      position:
        x: 42.0
        y: 8.0
        theta: 1.57  # 90° right
      action: "WAIT_FOR_ELEVATOR"
      dwell_time: 60.0  # max wait time

    - id: 3
      name: "Destination (Cafeteria)"
      position:
        x: 55.0
        y: 15.0
        theta: 3.14  # 180°
      action: "UNDOCK_WHEELCHAIR"
      dwell_time: 30.0
```

**Implementation:**
```cpp
bool RouteManager::loadRoute(const std::string& route_file) {
  // Parse YAML file
  YAML::Node config = YAML::LoadFile(route_file);

  route_id_ = config["route"]["id"].as<std::string>();
  map_id_ = config["route"]["map_id"].as<std::string>();

  waypoints_.clear();
  for (const auto& wp_node : config["route"]["waypoints"]) {
    Waypoint wp;
    wp.id = wp_node["id"].as<int>();
    wp.name = wp_node["name"].as<std::string>();
    wp.position.x = wp_node["position"]["x"].as<double>();
    wp.position.y = wp_node["position"]["y"].as<double>();
    wp.position.theta = wp_node["position"]["theta"].as<double>();
    wp.action = parseAction(wp_node["action"].as<std::string>());
    wp.dwell_time = wp_node["dwell_time"].as<double>();

    waypoints_.push_back(wp);
  }

  current_waypoint_idx_ = 0;
  route_status_ = RouteStatus::LOADED;

  RCLCPP_INFO(logger_, "Loaded route '%s' with %zu waypoints",
              route_id_.c_str(), waypoints_.size());

  return true;
}

Waypoint RouteManager::getNextWaypoint() {
  if (current_waypoint_idx_ >= waypoints_.size()) {
    throw std::runtime_error("No more waypoints in route");
  }

  return waypoints_[current_waypoint_idx_];
}

void RouteManager::markWaypointComplete() {
  if (current_waypoint_idx_ < waypoints_.size()) {
    RCLCPP_INFO(logger_, "Waypoint %d '%s' complete",
                waypoints_[current_waypoint_idx_].id,
                waypoints_[current_waypoint_idx_].name.c_str());

    current_waypoint_idx_++;

    if (isRouteComplete()) {
      route_status_ = RouteStatus::COMPLETED;
      RCLCPP_INFO(logger_, "Route '%s' complete!", route_id_.c_str());
    }
  }
}

double RouteManager::getRemainingDistance() {
  if (isRouteComplete()) return 0.0;

  double total_distance = 0.0;
  for (size_t i = current_waypoint_idx_; i < waypoints_.size() - 1; ++i) {
    double dx = waypoints_[i+1].position.x - waypoints_[i].position.x;
    double dy = waypoints_[i+1].position.y - waypoints_[i].position.y;
    total_distance += std::sqrt(dx*dx + dy*dy);
  }

  return total_distance;
}
```

---

### 4.3 NavigationManager

**Mission Execution State Machine:**
```
┌──────────────────────────────────────────────────────────────┐
│             Mission Execution State Machine                  │
└──────────────────────────────────────────────────────────────┘

    ┌────────┐
    │  IDLE  │
    └───┬────┘
        │ START_MISSION
        ↓
    ┌──────────┐
    │LOAD_ROUTE│ ← Load route YAML, validate waypoints
    └────┬─────┘
         │ ROUTE_LOADED
         ↓
    ┌──────────┐
    │LOAD_MAP  │ ← Load saved map for this route
    └────┬─────┘
         │ MAP_LOADED
         ↓
    ┌──────────┐
    │LOCALIZE  │ ← Wait for NDT convergence
    └────┬─────┘
         │ LOCALIZED
         ↓
    ┌──────────┐
    │NAVIGATE  │ ← Execute waypoints sequentially
    └────┬─────┘
         │ loop: for each waypoint
         │   ↓
         │  ┌─────────────────┐
         │  │ Nav2 NavigateToPose(waypoint)
         │  └─────────────────┘
         │   ↓
         │  ┌─────────────────┐
         │  │ WAYPOINT_REACHED
         │  │ Execute action (dock, undock, wait)
         │  └─────────────────┘
         │   ↓
         │  ┌─────────────────┐
         │  │ Next waypoint
         │  └─────────────────┘
         │
         │ ALL_WAYPOINTS_COMPLETE
         ↓
    ┌──────────┐
    │COMPLETE  │
    └──────────┘

    Error Handling:
    ┌─────────┐
    │ FAILED  │ ← Localization failure / Navigation timeout / Obstacle blocked
    └──────────┘
```

**Implementation:**
```cpp
void NavigationManager::executeMission() {
  auto mission = current_mission_;

  // 1. Load route
  if (!route_manager_->loadRoute(mission.route_file)) {
    RCLCPP_ERROR(logger_, "Failed to load route: %s", mission.route_file.c_str());
    missionFailed("Route load failure");
    return;
  }

  // 2. Load map
  std::string map_file = getMapPath(route_manager_->getMapId());
  if (!map_manager_->loadMap(map_file)) {
    RCLCPP_ERROR(logger_, "Failed to load map: %s", map_file.c_str());
    missionFailed("Map load failure");
    return;
  }

  // 3. Wait for localization convergence
  if (!waitForLocalization(30.0)) {  // 30s timeout
    RCLCPP_ERROR(logger_, "Localization timeout");
    missionFailed("Localization failure");
    return;
  }

  // 4. Execute waypoints
  while (!route_manager_->isRouteComplete()) {
    Waypoint wp = route_manager_->getNextWaypoint();

    RCLCPP_INFO(logger_, "Navigating to waypoint %d: '%s'", wp.id, wp.name.c_str());

    // Send navigation goal to Nav2
    auto goal = createNavigationGoal(wp);
    auto result_future = nav2_client_->async_send_goal(goal);

    // Wait for result
    auto result = result_future.get();

    if (result->code != NavigationResult::SUCCESS) {
      RCLCPP_ERROR(logger_, "Navigation to waypoint %d failed: %d", wp.id, result->code);
      handleNavigationFailure(wp, result->code);
      continue;  // Retry or skip based on policy
    }

    // Execute waypoint action
    executeWaypointAction(wp);

    // Dwell time
    if (wp.dwell_time > 0.0) {
      RCLCPP_INFO(logger_, "Dwelling at waypoint %d for %.1fs", wp.id, wp.dwell_time);
      rclcpp::sleep_for(std::chrono::duration<double>(wp.dwell_time));
    }

    // Mark complete
    route_manager_->markWaypointComplete();
  }

  // Mission complete
  RCLCPP_INFO(logger_, "Mission '%s' complete!", mission.id.c_str());
  publishMissionStatus(MissionStatus::COMPLETED);
}
```

---

## 5. Nav2 Integration

### 5.1 Nav2 Configuration (nav2_params.yaml)

```yaml
bt_navigator:
  ros__parameters:
    default_bt_xml_filename: "navigate_w_replanning.xml"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5                # meters
      use_astar: false              # Dijkstra is fine for outdoor open spaces
      allow_unknown: false          # Don't plan through unknown areas
      use_final_approach_orientation: true

controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "swerve_drive_controller::SwerveDriveController"
      # (See SWERVE_DRIVE_CONTROLLER_DESIGN.md for full config)

smoother_server:
  ros__parameters:
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 0.01
      max_its: 1000
      do_refinement: true

recovery_server:
  ros__parameters:
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
      simulate_ahead_time: 2.0
    backup:
      plugin: "nav2_behaviors/BackUp"
      simulate_ahead_time: 2.0
    wait:
      plugin: "nav2_behaviors/Wait"
      wait_duration: 5.0
```

---

## 6. Map Management

### 6.1 Map File Structure

```
maps/
├── main_building_floor_1/
│   ├── map.pgm                # Occupancy grid image
│   ├── map.yaml               # Map metadata
│   └── map.pcd                # Point cloud for NDT
│
├── outdoor_courtyard/
│   ├── map.pgm
│   ├── map.yaml
│   └── map.pcd
│
└── parking_lot/
    ├── map.pgm
    ├── map.yaml
    └── map.pcd
```

**map.yaml Example:**
```yaml
image: map.pgm
resolution: 0.05      # meters/pixel
origin: [-50.0, -30.0, 0.0]  # [x, y, theta]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

**map.pcd:** Generated from SLAM session's point cloud data, downsampled to ~50k-100k points for NDT efficiency.

---

## 7. Error Handling and Recovery

### 7.1 Localization Failure Recovery

```cpp
bool NavigationManager::waitForLocalization(double timeout) {
  auto start_time = node_->now();

  while ((node_->now() - start_time).seconds() < timeout) {
    // Check NDT fitness score
    double fitness_score = ndt_localization_->getFitnessScore();

    if (fitness_score < ndt_fitness_threshold_) {
      RCLCPP_INFO(logger_, "Localization converged: fitness=%.3f", fitness_score);
      return true;
    }

    // Check if robot is stationary (required for good NDT convergence)
    if (!isRobotStationary()) {
      RCLCPP_WARN(logger_, "Robot moving during localization - waiting for stillness");
    }

    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_ERROR(logger_, "Localization timeout after %.1fs", timeout);
  return false;
}
```

### 7.2 Navigation Failure Handling

**Common Failure Modes:**
1. **Timeout:** Nav2 cannot find path or reach goal within time limit
2. **Blocked:** Obstacle permanently blocking path
3. **Stuck:** Robot cannot make progress (wheel slip, etc.)
4. **Lost:** Localization quality degraded

**Recovery Strategy:**
```cpp
void NavigationManager::handleNavigationFailure(
  const Waypoint& wp,
  int error_code)
{
  retry_count_[wp.id]++;

  if (retry_count_[wp.id] >= max_retries_per_waypoint_) {
    RCLCPP_ERROR(logger_, "Max retries exceeded for waypoint %d - skipping", wp.id);
    route_manager_->markWaypointComplete();  // Skip this waypoint
    return;
  }

  switch (error_code) {
    case NavigationResult::ERROR_TIMEOUT:
      RCLCPP_WARN(logger_, "Navigation timeout - retrying with longer timeout");
      // Increase timeout for retry
      break;

    case NavigationResult::ERROR_BLOCKED:
      RCLCPP_WARN(logger_, "Path blocked - waiting 30s for obstacle to clear");
      rclcpp::sleep_for(std::chrono::seconds(30));
      // Retry navigation
      break;

    case NavigationResult::ERROR_STUCK:
      RCLCPP_WARN(logger_, "Robot stuck - executing recovery behavior");
      executeRecoveryBehavior("backup");  // Back up and try again
      break;

    case NavigationResult::ERROR_LOST:
      RCLCPP_ERROR(logger_, "Localization lost - re-localizing");
      if (!waitForLocalization(30.0)) {
        missionFailed("Re-localization failure");
      }
      break;
  }
}
```

---

## 8. Performance Optimization

### 8.1 NDT Performance Tuning

**Computation Budget:**
- NDT alignment: <100ms @ 10 Hz (GMKtec Nucbox K6)
- Target: 10 Hz localization rate for smooth navigation

**Optimizations:**
```cpp
// 1. Voxel downsampling of input scan
pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
voxel_filter.setLeafSize(0.5, 0.5, 0.5);  // 50cm voxels
voxel_filter.setInputCloud(raw_scan);
voxel_filter.filter(*downsampled_scan);

// 2. Multi-threading
ndt_.setNumThreads(4);  // Use 4 CPU cores

// 3. Reduce max iterations if converging fast
if (recent_fitness_scores_good) {
  ndt_.setMaximumIterations(15);  // Reduce from 30
}

// 4. Use previous pose as good initial guess (reduces iterations)
ndt_.align(output_cloud, previous_pose_);
```

---

## 9. Testing Strategy

### 9.1 Unit Tests

**Test: NDT Localization Accuracy**
```cpp
TEST(NDTLocalizationTest, AccuracyOnKnownMap) {
  NDTLocalizationNode ndt_node;

  // Load test map
  ndt_node.loadMapPointCloud("test_data/test_map.pcd");

  // Load test scans with known ground truth poses
  std::vector<TestScan> test_scans = loadTestScans("test_data/scans_with_gt.txt");

  for (const auto& test_scan : test_scans) {
    auto estimated_pose = ndt_node.localize(test_scan.scan);

    // Compare with ground truth
    double position_error = (estimated_pose.position - test_scan.gt_pose.position).norm();
    double orientation_error = computeOrientationError(estimated_pose, test_scan.gt_pose);

    EXPECT_LT(position_error, 0.10);  // <10cm
    EXPECT_LT(orientation_error, 0.05);  // <3°
  }
}
```

**Test: Route Manager**
```cpp
TEST(RouteManagerTest, WaypointSequence) {
  RouteManager route_mgr;

  ASSERT_TRUE(route_mgr.loadRoute("test_data/test_route.yaml"));

  EXPECT_EQ(route_mgr.getNumWaypoints(), 4);
  EXPECT_FALSE(route_mgr.isRouteComplete());

  // Get waypoints in sequence
  auto wp1 = route_mgr.getNextWaypoint();
  EXPECT_EQ(wp1.id, 0);

  route_mgr.markWaypointComplete();

  auto wp2 = route_mgr.getNextWaypoint();
  EXPECT_EQ(wp2.id, 1);

  // ... continue for all waypoints

  route_mgr.markWaypointComplete();
  EXPECT_TRUE(route_mgr.isRouteComplete());
}
```

---

### 9.2 Integration Tests

**Test: Full Mission Execution (Simulation)**
```cpp
TEST_F(NavigationIntegrationTest, CompleteMission) {
  // Spawn robot in Gazebo at known start position
  spawnRobot(Pose(0, 0, 0));

  // Load pre-built map
  loadMap("test_maps/outdoor_test_map");

  // Start mission with 4 waypoints
  auto mission = createTestMission(4);
  navigation_manager_->startMission(mission);

  // Wait for mission completion (timeout: 300s)
  auto result = waitForMissionComplete(300.0);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.waypoints_completed, 4);

  // Verify final position
  auto final_pose = getRobotPose();
  auto expected_pose = mission.waypoints.back().position;

  double position_error = (final_pose.position - expected_pose.position).norm();
  EXPECT_LT(position_error, 0.5);  // <50cm at final waypoint
}
```

---

## 10. Deployment Checklist

**Before deploying autonomous navigation:**

1. ✅ **Map Quality:** Verify saved map has good loop closure (no major drift)
2. ✅ **NDT Tuning:** Test NDT localization accuracy on map (<10cm error)
3. ✅ **Route Validation:** Test all waypoints are reachable and collision-free
4. ✅ **Recovery Behaviors:** Verify robot can recover from stuck/blocked situations
5. ✅ **Safety Integration:** Emergency stop functional during navigation
6. ✅ **Performance:** NDT @ 10 Hz, Nav2 controller @ 20 Hz on target hardware

---

**Document Status:** Draft
**Implementation Status:** Ready for development (ParcelPal proven architecture)
**Approvals Required:** Navigation Lead, Perception Lead, Systems Integration Engineer
