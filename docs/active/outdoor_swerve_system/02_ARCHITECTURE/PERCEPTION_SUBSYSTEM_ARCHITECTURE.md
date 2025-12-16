# Perception Subsystem Architecture

**Document ID:** ARCH-PERC-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft

---

## 1. Overview

The **Perception Subsystem** provides environment sensing for obstacle detection and avoidance. This architecture is **60-70% reused from ParcelPal**, using CPU-only processing with binary obstacle classification (NO deep learning).

**Key Capabilities:**
- 3D LiDAR-based obstacle detection (360° coverage)
- Binary obstacle classification (person = obstacle, car = obstacle, wall = obstacle)
- Ground plane removal and voxel downsampling
- CPU-only processing (PCL - Point Cloud Library)
- IMU-based tilt compensation
- Integration with Nav2 costmap

**Design Philosophy (ParcelPal Proven):**
- **NO object classification:** All non-ground obstacles treated equally
- **NO deep learning:** CPU-only processing on GMKtec Nucbox K6 (AMD Radeon 780M integrated GPU not used)
- **Binary detection:** Free space vs. obstacle (sufficient for navigation)

---

## 2. System Architecture

### 2.1 Component Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                    Perception Subsystem                          │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                  3D LiDAR Processing Pipeline              │ │
│  │                                                            │ │
│  │  /points_raw (10 Hz)                                       │ │
│  │      ↓                                                     │ │
│  │  ┌──────────────┐                                          │ │
│  │  │  Voxel Grid  │  Downsample 0.05m voxels                │ │
│  │  │  Filter      │  (reduce 150k → 30k points)             │ │
│  │  └──────┬───────┘                                          │ │
│  │         ↓                                                   │ │
│  │  ┌──────────────┐                                          │ │
│  │  │ Ground Plane │  RANSAC ground removal                   │ │
│  │  │  Removal     │  (IMU tilt compensation)                │ │
│  │  └──────┬───────┘                                          │ │
│  │         ↓                                                   │ │
│  │  ┌──────────────┐                                          │ │
│  │  │  Height      │  Filter obstacles (0.1m - 2.0m)         │ │
│  │  │  Filter      │  (ignore ground + overhead)             │ │
│  │  └──────┬───────┘                                          │ │
│  │         ↓                                                   │ │
│  │  ┌──────────────┐                                          │ │
│  │  │  Clustering  │  Euclidean clustering (optional)        │ │
│  │  │  (Optional)  │  (group nearby points)                  │ │
│  │  └──────┬───────┘                                          │ │
│  │         ↓                                                   │ │
│  │  /points_filtered (10 Hz)                                  │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────┐         ┌────────────────┐                 │
│  │  IMU           │────────▶│  Tilt          │                 │
│  │  (50 Hz)       │  pitch/ │  Compensation  │                 │
│  │                │  roll   │                │                 │
│  │ - Orientation  │         │ - Adjust ground│                 │
│  │ - Angular vel  │         │   plane normal │                 │
│  └────────────────┘         └────────────────┘                 │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                  Costmap Integration                       │ │
│  │                                                            │ │
│  │  /points_filtered → Nav2 Voxel Layer → Local Costmap      │ │
│  │                                                            │ │
│  │  - 3D → 2D projection (top-down occupancy grid)           │ │
│  │  - Inflation layer (safety buffer)                        │ │
│  │  - Dynamic obstacle tracking                              │ │
│  └────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────┘
```

**NO Object Classification:**
- Camera NOT used for perception (only for ArUco docking)
- NO TensorFlow, PyTorch, or deep learning frameworks
- All obstacles treated as binary (free space vs. occupied)

---

## 3. ROS 2 Node Architecture

### 3.1 Point Cloud Filter Node

**Node Name:** `pcl_filter`
**Language:** C++
**Lifecycle:** Standard node

**Subscribed Topics:**
- `/points_raw` (sensor_msgs/PointCloud2): Raw 3D LiDAR point cloud (10 Hz)
- `/imu/data` (sensor_msgs/Imu): IMU for tilt compensation (50 Hz)

**Published Topics:**
- `/points_filtered` (sensor_msgs/PointCloud2): Filtered point cloud (obstacles only, 10 Hz)
- `/ground_plane` (sensor_msgs/PointCloud2): Detected ground plane (debug, 1 Hz)
- `/pcl_filter/status` (diagnostic_msgs/DiagnosticStatus): Filter performance metrics

**Parameters:**
```yaml
pcl_filter:
  ros__parameters:
    # Input/output frames
    input_frame: "lidar_frame"
    output_frame: "base_link"
    
    # Voxel grid downsampling
    voxel_leaf_size: 0.05         # meters (5cm voxels)
    
    # Ground plane removal (RANSAC)
    ground_removal:
      enabled: true
      distance_threshold: 0.05    # meters (points within 5cm of plane = ground)
      max_iterations: 100         # RANSAC iterations
      use_imu_tilt: true          # IMU-aided ground plane normal
    
    # Height filter
    height_filter:
      min_height: 0.1             # meters (ignore ground)
      max_height: 2.0             # meters (ignore overhead)
    
    # Clustering (optional, for object segmentation)
    clustering:
      enabled: false              # Disabled by default (not needed for binary detection)
      cluster_tolerance: 0.2      # meters
      min_cluster_size: 10        # points
      max_cluster_size: 5000      # points
    
    # Performance
    processing_rate: 10.0         # Hz (match LiDAR rate)
```

---

### 3.2 3D LiDAR Driver Node

**Node Name:** `lidar_driver`
**Language:** C++ (vendor-provided driver)
**Lifecycle:** Standard node

**Published Topics:**
- `/points_raw` (sensor_msgs/PointCloud2): Raw point cloud from LiDAR

**Parameters:**
```yaml
lidar_driver:
  ros__parameters:
    device_ip: "192.168.1.201"    # LiDAR IP address (Ethernet connection)
    frame_id: "lidar_frame"
    scan_rate: 10                 # Hz
    min_range: 0.5                # meters
    max_range: 50.0               # meters (outdoor range)
    
    # Point cloud size
    expected_points: 150000       # Typical outdoor scan (depends on LiDAR model)
```

---

### 3.3 IMU Driver Node

**Node Name:** `imu_driver`
**Language:** C++ (vendor-provided driver)
**Lifecycle:** Standard node

**Published Topics:**
- `/imu/data` (sensor_msgs/Imu): IMU orientation and angular velocity (50 Hz)

**Parameters:**
```yaml
imu_driver:
  ros__parameters:
    port: "/dev/ttyUSB0"          # Serial port
    baud_rate: 115200
    frame_id: "imu_frame"
    rate: 50                      # Hz
    
    # Calibration
    gyro_bias: [0.01, -0.02, 0.005]   # rad/s (from calibration)
    accel_bias: [0.05, -0.03, 0.1]    # m/s² (from calibration)
```

---

## 4. Point Cloud Processing Pipeline

### 4.1 Voxel Grid Downsampling

**Purpose:** Reduce computational load while preserving obstacle features

**Algorithm (PCL VoxelGrid):**
```cpp
void voxelGridFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
    double leaf_size) {

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(input);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*output);
    
    // Typical reduction: 150k points → 30k points (5× speedup)
}
```

**Performance:**
- Input: ~150,000 points @ 10 Hz (outdoor 3D LiDAR)
- Output: ~30,000 points @ 10 Hz (5cm voxels)
- Processing time: <10ms on GMKtec Nucbox K6

---

### 4.2 Ground Plane Removal (RANSAC)

**Purpose:** Remove ground plane to isolate obstacles

**Algorithm:**
```cpp
void removeGroundPlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacles,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& ground,
    const Eigen::Vector3d& imu_tilt) {

    // 1. Construct ground plane model from IMU tilt
    //    Ground normal = [0, 0, 1] rotated by IMU pitch/roll
    Eigen::Vector3d ground_normal = rotateVector(Eigen::Vector3d(0, 0, 1), imu_tilt);
    
    // 2. RANSAC plane fitting
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);  // 5cm tolerance
    seg.setMaxIterations(100);
    seg.setInputCloud(input);
    
    // Optionally seed with IMU-based normal
    if (use_imu_seed) {
        seg.setAxis(ground_normal);
        seg.setEpsAngle(0.1);  // 10° tolerance from IMU
    }
    
    seg.segment(*inliers, *coefficients);
    
    // 3. Extract obstacles (non-ground points)
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.setNegative(true);  // Get non-ground points
    extract.filter(*obstacles);
    
    // 4. Extract ground (for debugging)
    extract.setNegative(false);
    extract.filter(*ground);
}
```

**IMU Tilt Compensation:**
```cpp
Eigen::Vector3d getGroundNormalFromIMU(const sensor_msgs::msg::Imu& imu) {
    // Extract pitch and roll from IMU quaternion
    tf2::Quaternion q(imu.orientation.x, imu.orientation.y, 
                      imu.orientation.z, imu.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    // Rotate z-axis (up vector) by pitch and roll
    Eigen::Vector3d up(0, 0, 1);
    Eigen::AngleAxisd rollRot(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchRot(pitch, Eigen::Vector3d::UnitY());
    
    Eigen::Vector3d ground_normal = pitchRot * rollRot * up;
    return ground_normal;
}
```

**Performance:**
- Input: ~30,000 points (after voxel downsampling)
- Output: ~15,000 points (obstacles only)
- Processing time: <20ms on GMKtec Nucbox K6

---

### 4.3 Height Filter

**Purpose:** Filter obstacles by height (ignore ground debris and overhead structures)

**Algorithm:**
```cpp
void heightFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
    double min_height,
    double max_height) {

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_height, max_height);
    pass.filter(*output);
    
    // Typical: min=0.1m (ignore small ground debris), max=2.0m (ignore overhead)
}
```

**Height Ranges:**
- Ground debris: z < 0.1m (ignored)
- Relevant obstacles: 0.1m ≤ z ≤ 2.0m (people, vehicles, walls, wheelchairs)
- Overhead structures: z > 2.0m (ignored, unless low clearance area)

---

### 4.4 Optional Clustering (NOT Used by Default)

**Purpose:** Group nearby points into discrete obstacles (useful for tracking, but not needed for binary detection)

**Algorithm (Euclidean Clustering):**
```cpp
void euclideanClustering(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters,
    double tolerance,
    int min_size,
    int max_size) {

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(tolerance);  // 20cm
    ec.setMinClusterSize(min_size);     // 10 points
    ec.setMaxClusterSize(max_size);     // 5000 points
    ec.setSearchMethod(tree);
    ec.setInputCloud(input);
    ec.extract(cluster_indices);
    
    // Convert indices to separate point clouds
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*input, indices, *cluster);
        clusters.push_back(cluster);
    }
}
```

**NOT Used by Default:**
- Binary obstacle detection doesn't need clustering
- Clustering adds computational overhead (~10-20ms)
- Nav2 costmap handles obstacle grouping internally

---

## 5. Costmap Integration

### 5.1 Nav2 Voxel Layer

**Configuration (local_costmap.yaml):**
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "inflation_layer"]
      
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.2         # meters (vertical voxel size)
        z_voxels: 16              # 16 × 0.2m = 3.2m vertical range
        max_obstacle_height: 2.0  # meters (from height filter)
        min_obstacle_height: 0.1  # meters (from height filter)
        mark_threshold: 2         # Min points in voxel to mark as obstacle
        
        # Point cloud source
        observation_sources: scan
        scan:
          topic: /points_filtered
          sensor_frame: lidar_frame
          data_type: PointCloud2
          marking: true           # Mark obstacles
          clearing: true          # Clear free space
          min_obstacle_height: 0.1
          max_obstacle_height: 2.0
          obstacle_range: 10.0    # meters (mark obstacles up to 10m)
          raytrace_range: 15.0    # meters (clear free space up to 15m)
```

**How It Works:**
1. Voxel layer receives `/points_filtered` (obstacles only, height 0.1-2.0m)
2. Projects 3D points onto 2D occupancy grid (top-down view)
3. Marks occupied cells in costmap
4. Raytraces from robot to each point to clear free space
5. Inflation layer adds safety buffer around obstacles

---

### 5.2 3D → 2D Projection

**Algorithm (Inside Nav2 Voxel Layer):**
```cpp
void projectPointCloudToCostmap(
    const sensor_msgs::msg::PointCloud2& cloud,
    nav2_costmap_2d::Costmap2D& costmap) {

    // 1. Convert ROS message to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud, *pcl_cloud);
    
    // 2. For each point, mark corresponding costmap cell
    for (const auto& point : pcl_cloud->points) {
        // Transform point to costmap frame (if needed)
        geometry_msgs::msg::PointStamped point_in;
        point_in.point.x = point.x;
        point_in.point.y = point.y;
        point_in.point.z = point.z;
        
        // Project to 2D (ignore z after height filtering)
        unsigned int mx, my;
        if (costmap.worldToMap(point.x, point.y, mx, my)) {
            // Mark cell as occupied (cost = 254)
            costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
    }
    
    // 3. Raytrace for free space clearing (see Nav2 source)
}
```

---

## 6. Sensor Fusion

### 6.1 IMU + LiDAR Fusion

**Purpose:** Compensate for robot tilt (slopes, uneven ground)

**Fusion Strategy:**
- IMU provides ground plane orientation (pitch, roll)
- RANSAC ground removal uses IMU as initial seed
- More robust ground detection on slopes

**Benefits:**
- Accurate ground removal even on 10° slopes
- Prevents ground from being marked as obstacles
- Faster RANSAC convergence (fewer iterations needed)

---

### 6.2 Camera NOT Used for Perception

**Important:** Cameras are ONLY used for ArUco marker detection during docking

**Rationale:**
- No object classification needed (binary obstacle detection sufficient)
- CPU-only processing (no GPU for deep learning)
- 3D LiDAR provides 360° coverage (cameras have limited FOV)
- Cameras sensitive to lighting (outdoor operation)

---

## 7. Performance Optimization

### 7.1 CPU Budget Allocation

**Total CPU Budget:** <70% during normal operation (8 cores, 16 threads)

**Perception CPU Allocation:**
- Voxel downsampling: ~5% (10ms @ 10 Hz)
- Ground removal (RANSAC): ~10% (20ms @ 10 Hz)
- Height filter: ~2% (5ms @ 10 Hz)
- Nav2 voxel layer: ~8% (handled by Nav2)
- **Total Perception:** ~25% CPU

**Remaining Budget:**
- Navigation: ~20%
- Localization (NDT): ~15%
- Control (swerve drive): ~5%
- Other: ~5%
- Reserve: ~30%

---

### 7.2 Processing Pipeline Optimization

**Parallelization:**
```cpp
void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 1. Voxel downsampling (single-threaded, fast)
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled = voxelGridFilter(input_cloud);
    
    // 2. Ground removal (multi-threaded RANSAC available in PCL)
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles = removeGroundPlane(downsampled);
    
    // 3. Height filter (single-threaded, trivial)
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = heightFilter(obstacles);
    
    // 4. Publish filtered cloud
    publishFilteredCloud(filtered);
}
```

**Key Optimizations:**
- Voxel downsampling FIRST (reduces points 5×, speeds up all downstream operations)
- RANSAC with IMU seed (faster convergence, fewer iterations)
- NO clustering (not needed for binary detection)
- Processing rate locked to LiDAR rate (10 Hz, no faster)

---

## 8. Testing Strategy

### 8.1 Unit Tests
- Voxel downsampling (known cloud → expected downsampled cloud)
- Ground plane removal (synthetic ground + obstacles → obstacles only)
- Height filter (known heights → expected filtered output)

### 8.2 Integration Tests
- Full pipeline with recorded LiDAR data
- IMU tilt compensation on slopes
- Costmap generation from filtered point cloud

### 8.3 Field Tests
- Ground removal accuracy (flat ground, 5° slope, 10° slope)
- Obstacle detection range (0.5m to 10m)
- Processing latency (<50ms end-to-end)
- CPU usage (<25% perception subsystem)
- Outdoor robustness (rain, dust, varying lighting - LiDAR unaffected)

---

## 9. Performance Targets

| Metric | Target | Validation |
|--------|--------|------------|
| Processing Latency | <50ms (point cloud → costmap) | Timestamp analysis |
| Processing Rate | 10 Hz (match LiDAR) | Timestamp analysis |
| CPU Usage (Perception) | <25% | htop / system monitor |
| Ground Removal Accuracy | >95% (flat), >90% (10° slope) | Manual labeling |
| Obstacle Detection Range | 0.5m - 10m | Detection tests |
| False Positive Rate | <5% (non-obstacles marked) | Field tests |
| False Negative Rate | <2% (obstacles missed) | Field tests |

---

## 10. Implementation Phases

### Phase 1: Basic Pipeline (Sprints 1-2)
- ✅ 3D LiDAR driver integration
- ✅ Voxel downsampling
- ✅ Height filter
- ✅ Point cloud visualization (RViz)

### Phase 2: Ground Removal (Sprints 3-4)
- ✅ RANSAC ground plane removal
- ✅ IMU integration for tilt compensation
- ✅ Ground removal tuning (flat, slopes)

### Phase 3: Costmap Integration (Sprints 5-6)
- ✅ Nav2 voxel layer configuration
- ✅ 3D → 2D projection
- ✅ Obstacle marking and free space clearing
- ✅ Integration with navigation

### Phase 4: Optimization & Validation (Sprints 7-8)
- ✅ CPU profiling and optimization
- ✅ Outdoor field tests
- ✅ Performance tuning (accuracy vs. speed)
- ✅ Robustness testing (weather, lighting)

---

**Document Status:** Draft
**Implementation Status:** Not Started
**Approvals Required:** Perception Engineer, Navigation Lead, System Architect
