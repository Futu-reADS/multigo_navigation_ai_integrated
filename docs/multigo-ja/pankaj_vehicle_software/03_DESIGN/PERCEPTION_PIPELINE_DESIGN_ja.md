# 知覚パイプライン詳細設計

**ドキュメントID:** DESIGN-PERCEPTION-001
**バージョン:** 1.0
**日付:** 2025-12-15
**ステータス:** ドラフト

---

## 1. 概要

本ドキュメントは、ディープラーニングやオブジェクト分類を行わずに、CPU専用の3D LiDAR処理によるバイナリ障害物検出を実装する、知覚パイプラインの詳細設計仕様を提供します。

**主要コンポーネント:**
- LiDARProcessor（点群フィルタリングパイプライン）
- GroundRemovalNode（RANSACベースの地面抽出）
- ObstacleDetector（バイナリ障害物検出）
- CostmapIntegration（Nav2コストマッププラグイン）
- IMUFusion（傾斜補償）

**主要制約:**
- ディープラーニングなし（GMKtec Nucbox K6上でCPU専用処理）
- オブジェクト分類なし（すべての非地面オブジェクト = 障害物）
- バイナリ検出: 人=障害物、車=障害物、壁=障害物
- ParcelPal実証済みアーキテクチャ（60-70%再利用）

---

## 2. パイプラインアーキテクチャ

```
3D LiDAR → Voxelフィルタ → 地面除去 → 高さフィルタ → コストマップ
(100Hz)     (30ms)         (RANSAC)     (0.1-2.0m)    (Nav2)
            ↓              ↑
         150k→30k       IMU傾斜
         点群          補償
```

**処理予算:** GMKtec Nucbox K6上で20 Hzで合計<50ms

---

## 3. クラス図

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

## 4. 地面除去アルゴリズム

**IMUシーディング付きRANSAC:**
```cpp
void GroundRemovalNode::removeGround(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacles,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& ground,
    const Eigen::Vector3d& imu_tilt)
{
    // 1. IMU傾斜から予想される地面法線を計算
    Eigen::Vector3d ground_normal = computeGroundNormal(imu_tilt);

    // 2. IMUシーディング初期推定値を用いたRANSAC平面フィッティング
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(
        new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(input));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(ground_threshold_);  // 0.05m
    ransac.computeModel();

    // 3. インライア（地面）とアウトライア（障害物）を抽出
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
    // IMUはロール/ピッチ（重力からの傾き）を提供
    // IMU傾斜によって[0, 0, 1]（上方向ベクトル）を回転
    Eigen::AngleAxisd rollAngle(imu_tilt.x(), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(imu_tilt.y(), Eigen::Vector3d::UnitY());

    Eigen::Vector3d ground_normal = pitchAngle * rollAngle * Eigen::Vector3d::UnitZ();

    return ground_normal.normalized();
}
```

---

## 5. 構成

**perception_params.yaml:**
```yaml
lidar_processor:
  ros__parameters:
    # Voxelダウンサンプリング
    voxel_leaf_size: 0.05           # 5cm voxels（150k → 30k点）

    # ROI（関心領域）
    roi_min_x: -50.0                # メートル
    roi_max_x: 50.0
    roi_min_y: -50.0
    roi_max_y: 50.0
    roi_min_z: -0.5                 # 地面より下
    roi_max_z: 3.0                  # ロボットより上

    # 地面除去
    ground_threshold: 0.05          # メートル
    ground_angle_tolerance: 0.1     # ラジアン（約6°）

    # 高さフィルタ（障害物検出）
    obstacle_min_height: 0.1        # メートル
    obstacle_max_height: 2.0        # メートル

    # パフォーマンス
    processing_rate: 20.0           # Hz
    num_threads: 4                  # PCL用CPUコア
```

---

## 6. コストマップ統合

**Nav2コストマッププラグイン:**
```cpp
class ObstacleCostmapLayer : public nav2_costmap_2d::CostmapLayer {
public:
    void updateCosts(
        nav2_costmap_2d::Costmap2D& master_costmap,
        int min_i, int min_j, int max_i, int max_j) override
    {
        // 障害物点を2Dコストマップに投影
        for (const auto& point : obstacle_cloud_->points) {
            unsigned int mx, my;
            if (worldToMap(point.x, point.y, mx, my)) {
                // セルを障害物としてマーク
                master_costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

                // 障害物周辺を膨張
                inflateObstacle(master_costmap, mx, my, inflation_radius_);
            }
        }
    }
};
```

---

## 7. パフォーマンス最適化

**最適化技術:**
```cpp
// 1. マルチスレッディング
pcl::VoxelGrid<pcl::PointXYZ> voxel;
voxel.setNumberOfThreads(4);

// 2. メモリの事前割り当て
obstacle_cloud_->reserve(50000);

// 3. 早期リジェクション（最初にROIフィルタリング）
passthrough.setFilterLimitsNegative(false);

// 4. 可能な場合は整列点群を使用
if (input->isOrganized()) {
    // 整列データでの高速処理
}
```

**計算予算:**
- Voxelダウンサンプリング: 10ms
- 地面除去: 20ms
- 高さフィルタリング: 5ms
- コストマップ更新: 10ms
- **合計: 20 Hzで45ms**（50ms予算内）

---

## 8. テスト戦略

**ユニットテスト例:**
```cpp
TEST(GroundRemovalTest, FlatGroundRemoval) {
    GroundRemovalNode ground_removal;

    // テスト点群を作成: 平らな地面 + オブジェクト
    auto cloud = createTestCloud();
    auto obstacles = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    auto ground = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);

    // 傾きなし
    Eigen::Vector3d imu_tilt(0, 0, 0);

    ground_removal.removeGround(cloud, obstacles, ground, imu_tilt);

    // 地面点が除去されたことを確認
    EXPECT_GT(ground->size(), 0);

    // 障害物が検出されたことを確認
    EXPECT_GT(obstacles->size(), 0);
}
```

---

**ドキュメントステータス:** ドラフト
**実装ステータス:** 開発準備完了（ParcelPal実証済みアーキテクチャ）
**承認が必要:** 知覚リード、ナビゲーションリード
