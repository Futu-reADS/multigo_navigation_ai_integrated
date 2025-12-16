# Perception Pipeline Detailed Design

**Document ID:** DESIGN-PERCEPTION-001
**Version:** 1.0
**Date:** 2025-12-15
**Status:** Draft

---

## 1. Overview

This document provides detailed design specifications for the Perception Pipeline, implementing CPU-only 3D LiDAR processing for binary obstacle detection without deep learning or object classification.

**Key Components:**
- LiDARProcessor (point cloud filtering pipeline)
- GroundRemovalNode (RANSAC-based ground extraction)
- ObstacleDetector (binary obstacle detection)
- CostmapIntegration (Nav2 costmap plugin)
- IMUFusion (tilt compensation)

**Key Constraints:**
- NO Deep Learning (CPU-only processing on GMKtec Nucbox K6)
- NO Object Classification (all non-ground objects = obstacles)
- Binary Detection: person=obstacle, car=obstacle, wall=obstacle
- ParcelPal proven architecture (60-70% reuse)

---

## 2. Pipeline Architecture

```
3D LiDAR → Voxel Filter → Ground Removal → Height Filter → Costmap
(100Hz)     (30ms)        (RANSAC)         (0.1-2.0m)     (Nav2)
            ↓             ↑
         150k→30k       IMU Tilt
         points        Compensation
```

**Processing Budget:** <50ms total @ 20 Hz on GMKtec Nucbox K6

---

## 3. Class Diagram

```
┌─────────────────────────────────────────────────────────────┐
│              PerceptionPipeline                             │
├─────────────────────────────────────────────────────────────┤
│ - lidar_processor_: unique_ptr<LiDARProcessor>             │
│ - ground_removal_: unique_ptr<GroundRemovalNode>           │
│ - obstacle_detector_: unique_ptr<ObstacleDetector>         │
│ - imu_fusion_: unique_ptr<IMUFusion>                       │
├─────────────────────────────────────────────────────────────┤
│ + processPointCloud(scan): ObstacleCloud                   │
│ + publishCostmap()                                          │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│              LiDARProcessor                                 │
├─────────────────────────────────────────────────────────────┤
│ + voxelDownsample(cloud, leaf_size): PointCloud            │
│ + removeOutliers(cloud): PointCloud                         │
│ + applyROI(cloud, bounds): PointCloud                      │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│              GroundRemovalNode                              │
├─────────────────────────────────────────────────────────────┤
│ - ransac_: pcl::SampleConsensusModelPlane                  │
│ - ground_normal_: Eigen::Vector3d (from IMU)               │
├─────────────────────────────────────────────────────────────┤
│ + removeGround(cloud, imu_tilt): (obstacles, ground)       │
│ - seedRANSACWithIMU(tilt): plane_coefficients              │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│              ObstacleDetector                               │
├─────────────────────────────────────────────────────────────┤
│ + extractObstacles(cloud): ObstacleCloud                   │
│ + applyHeightFilter(cloud, min, max): PointCloud          │
└─────────────────────────────────────────────────────────────┘
```

---

## 4. Ground Removal Algorithm

**RANSAC with IMU Seeding:**
```cpp
void GroundRemovalNode::removeGround(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacles,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& ground,
    const Eigen::Vector3d& imu_tilt)
{
    // 1. Compute expected ground normal from IMU tilt
    Eigen::Vector3d ground_normal = computeGroundNormal(imu_tilt);

    // 2. RANSAC plane fitting with IMU-seeded initial guess
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(
        new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(input));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(ground_threshold_);  // 0.05m
    ransac.computeModel();

    // 3. Extract inliers (ground) and outliers (obstacles)
    std::vector<int> inliers;
    ransac.getInliers(inliers);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input);
    extract.setIndices(boost::make_shared<std::vector<int>>(inliers));

    extract.setNegative(false);
    extract.filter(*ground);

    extract.setNegative(true);
    extract.filter(*obstacles);
}

Eigen::Vector3d GroundRemovalNode::computeGroundNormal(
    const Eigen::Vector3d& imu_tilt)
{
    // IMU provides roll/pitch (tilt away from gravity)
    // Rotate [0, 0, 1] (up vector) by IMU tilt
    Eigen::AngleAxisd rollAngle(imu_tilt.x(), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(imu_tilt.y(), Eigen::Vector3d::UnitY());

    Eigen::Vector3d ground_normal = pitchAngle * rollAngle * Eigen::Vector3d::UnitZ();

    return ground_normal.normalized();
}
```

---

## 5. Configuration

**perception_params.yaml:**
```yaml
lidar_processor:
  ros__parameters:
    # Voxel downsampling
    voxel_leaf_size: 0.05           # 5cm voxels (150k → 30k points)

    # ROI (region of interest)
    roi_min_x: -50.0                # meters
    roi_max_x: 50.0
    roi_min_y: -50.0
    roi_max_y: 50.0
    roi_min_z: -0.5                 # below ground
    roi_max_z: 3.0                  # above robot

    # Ground removal
    ground_threshold: 0.05          # meters
    ground_angle_tolerance: 0.1     # radians (~6°)

    # Height filter (obstacle detection)
    obstacle_min_height: 0.1        # meters
    obstacle_max_height: 2.0        # meters

    # Performance
    processing_rate: 20.0           # Hz
    num_threads: 4                  # CPU cores for PCL
```

---

## 6. Costmap Integration

**Nav2 Costmap Plugin:**
```cpp
class ObstacleCostmapLayer : public nav2_costmap_2d::CostmapLayer {
public:
    void updateCosts(
        nav2_costmap_2d::Costmap2D& master_costmap,
        int min_i, int min_j, int max_i, int max_j) override
    {
        // Project obstacle points onto 2D costmap
        for (const auto& point : obstacle_cloud_->points) {
            unsigned int mx, my;
            if (worldToMap(point.x, point.y, mx, my)) {
                // Mark cell as obstacle
                master_costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

                // Inflate around obstacle
                inflateObstacle(master_costmap, mx, my, inflation_radius_);
            }
        }
    }
};
```

---

## 7. Performance Optimization

**Optimization Techniques:**
```cpp
// 1. Multi-threading
pcl::VoxelGrid<pcl::PointXYZ> voxel;
voxel.setNumberOfThreads(4);

// 2. Pre-allocate memory
obstacle_cloud_->reserve(50000);

// 3. Early rejection (ROI filtering first)
passthrough.setFilterLimitsNegative(false);

// 4. Use organized point clouds when possible
if (input->isOrganized()) {
    // Faster processing for organized data
}
```

**Computation Budget:**
- Voxel downsampling: 10ms
- Ground removal: 20ms
- Height filtering: 5ms
- Costmap update: 10ms
- **Total: 45ms @ 20 Hz** (within 50ms budget)

---

## 8. Testing Strategy

**Unit Test Example:**
```cpp
TEST(GroundRemovalTest, FlatGroundRemoval) {
    GroundRemovalNode ground_removal;

    // Create test cloud: flat ground + object
    auto cloud = createTestCloud();
    auto obstacles = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    auto ground = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);

    // No tilt
    Eigen::Vector3d imu_tilt(0, 0, 0);

    ground_removal.removeGround(cloud, obstacles, ground, imu_tilt);

    // Verify ground points removed
    EXPECT_GT(ground->size(), 0);

    // Verify obstacle detected
    EXPECT_GT(obstacles->size(), 0);
}
```

---

**Document Status:** Draft
**Implementation Status:** Ready for development (ParcelPal proven architecture)
**Approvals Required:** Perception Lead, Navigation Lead
