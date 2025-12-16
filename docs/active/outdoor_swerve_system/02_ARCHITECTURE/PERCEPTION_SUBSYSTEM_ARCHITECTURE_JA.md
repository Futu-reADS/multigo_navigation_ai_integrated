# 知覚サブシステムアーキテクチャ

**文書ID:** ARCH-PERC-001
**バージョン:** 1.0
**日付:** 2025年12月15日
**ステータス:** ドラフト

---

## 1. 概要

**知覚サブシステム**は、障害物検出と回避のための環境センシングを提供します。このアーキテクチャは**ParcelPalから60-70%再利用**され、バイナリ障害物分類（深層学習なし）によるCPUのみの処理を使用します。

**主要機能:**
- 3D LiDARベース障害物検出（360°カバレッジ）
- バイナリ障害物分類（人=障害物、車=障害物、壁=障害物）
- 地面平面除去とボクセルダウンサンプリング
- CPUのみの処理（PCL - Point Cloud Library）
- IMUベース傾斜補償
- Nav2コストマップとの統合

**設計哲学（ParcelPal実証済み）:**
- **オブジェクト分類なし:** すべての非地面障害物を同等に扱う
- **深層学習なし:** GMKtec Nucbox K6でのCPUのみ処理（AMD Radeon 780M統合GPUは未使用）
- **バイナリ検出:** 自由空間 vs. 障害物（ナビゲーションに十分）

---

## 2. システムアーキテクチャ

### 2.1 コンポーネント図

```
┌──────────────────────────────────────────────────────────────────┐
│                    知覚サブシステム                               │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                  3D LiDAR処理パイプライン                  │ │
│  │                                                            │ │
│  │  /points_raw (10 Hz)                                       │ │
│  │      ↓                                                     │ │
│  │  ┌──────────────┐                                          │ │
│  │  │ボクセルグリッド│  0.05mボクセルでダウンサンプル          │ │
│  │  │ フィルター   │  (150k → 30k点に削減)                   │ │
│  │  └──────┬───────┘                                          │ │
│  │         ↓                                                   │ │
│  │  ┌──────────────┐                                          │ │
│  │  │ 地面平面     │  RANSAC地面除去                          │ │
│  │  │  除去        │  (IMU傾斜補償)                          │ │
│  │  └──────┬───────┘                                          │ │
│  │         ↓                                                   │ │
│  │  ┌──────────────┐                                          │ │
│  │  │  高さ        │  障害物フィルター (0.1m - 2.0m)         │ │
│  │  │  フィルター  │  (地面+頭上を無視)                      │ │
│  │  └──────┬───────┘                                          │ │
│  │         ↓                                                   │ │
│  │  ┌──────────────┐                                          │ │
│  │  │クラスタリング│  ユークリッドクラスタリング（オプション）│ │
│  │  │（オプション）│  (近傍点をグループ化)                   │ │
│  │  └──────┬───────┘                                          │ │
│  │         ↓                                                   │ │
│  │  /points_filtered (10 Hz)                                  │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────┐         ┌────────────────┐                 │
│  │  IMU           │────────▶│  傾斜          │                 │
│  │  (50 Hz)       │  pitch/ │  補償          │                 │
│  │                │  roll   │                │                 │
│  │ - 向き         │         │ - 地面平面     │                 │
│  │ - 角速度       │         │   法線調整     │                 │
│  └────────────────┘         └────────────────┘                 │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                  コストマップ統合                          │ │
│  │                                                            │ │
│  │  /points_filtered → Nav2ボクセルレイヤー → ローカルコストマップ│ │
│  │                                                            │ │
│  │  - 3D → 2D投影（トップダウン占有グリッド）                │ │
│  │  - インフレーションレイヤー（安全バッファ）                │ │
│  │  - 動的障害物追跡                                          │ │
│  └────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────┘
```

**オブジェクト分類なし:**
- カメラは知覚に使用されない（ArUcoドッキングのみ）
- TensorFlow、PyTorch、または深層学習フレームワークなし
- すべての障害物をバイナリ（自由空間 vs. 占有）として扱う

---

## 3. ROS 2ノードアーキテクチャ

### 3.1 点群フィルターノード

**ノード名:** `pcl_filter`
**言語:** C++
**ライフサイクル:** 標準ノード

**購読トピック:**
- `/points_raw` (sensor_msgs/PointCloud2): 生3D LiDAR点群（10 Hz）
- `/imu/data` (sensor_msgs/Imu): 傾斜補償用IMU（50 Hz）

**配信トピック:**
- `/points_filtered` (sensor_msgs/PointCloud2): フィルター済み点群（障害物のみ、10 Hz）
- `/ground_plane` (sensor_msgs/PointCloud2): 検出された地面平面（デバッグ、1 Hz）
- `/pcl_filter/status` (diagnostic_msgs/DiagnosticStatus): フィルター性能メトリック

**パラメータ:**
```yaml
pcl_filter:
  ros__parameters:
    # 入力/出力フレーム
    input_frame: "lidar_frame"
    output_frame: "base_link"
    
    # ボクセルグリッドダウンサンプリング
    voxel_leaf_size: 0.05         # メートル（5cmボクセル）
    
    # 地面平面除去（RANSAC）
    ground_removal:
      enabled: true
      distance_threshold: 0.05    # メートル（平面から5cm以内の点=地面）
      max_iterations: 100         # RANSAC反復
      use_imu_tilt: true          # IMU支援地面平面法線
    
    # 高さフィルター
    height_filter:
      min_height: 0.1             # メートル（地面を無視）
      max_height: 2.0             # メートル（頭上を無視）
    
    # クラスタリング（オプション、オブジェクトセグメンテーション用）
    clustering:
      enabled: false              # デフォルトで無効（バイナリ検出には不要）
      cluster_tolerance: 0.2      # メートル
      min_cluster_size: 10        # 点
      max_cluster_size: 5000      # 点
    
    # 性能
    processing_rate: 10.0         # Hz（LiDARレートに一致）
```

---

### 3.2 3D LiDARドライバーノード

**ノード名:** `lidar_driver`
**言語:** C++（ベンダー提供ドライバー）
**ライフサイクル:** 標準ノード

**配信トピック:**
- `/points_raw` (sensor_msgs/PointCloud2): LiDARからの生点群

**パラメータ:**
```yaml
lidar_driver:
  ros__parameters:
    device_ip: "192.168.1.201"    # LiDAR IPアドレス（Ethernet接続）
    frame_id: "lidar_frame"
    scan_rate: 10                 # Hz
    min_range: 0.5                # メートル
    max_range: 50.0               # メートル（屋外範囲）
    
    # 点群サイズ
    expected_points: 150000       # 典型的な屋外スキャン（LiDARモデルに依存）
```

---

### 3.3 IMUドライバーノード

**ノード名:** `imu_driver`
**言語:** C++（ベンダー提供ドライバー）
**ライフサイクル:** 標準ノード

**配信トピック:**
- `/imu/data` (sensor_msgs/Imu): IMU向きと角速度（50 Hz）

**パラメータ:**
```yaml
imu_driver:
  ros__parameters:
    port: "/dev/ttyUSB0"          # シリアルポート
    baud_rate: 115200
    frame_id: "imu_frame"
    rate: 50                      # Hz
    
    # キャリブレーション
    gyro_bias: [0.01, -0.02, 0.005]   # rad/s（キャリブレーションから）
    accel_bias: [0.05, -0.03, 0.1]    # m/s²（キャリブレーションから）
```

---

## 4. 点群処理パイプライン

### 4.1 ボクセルグリッドダウンサンプリング

**目的:** 障害物特徴を保持しながら計算負荷を削減

**アルゴリズム（PCL VoxelGrid）:**
```cpp
void voxelGridFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
    double leaf_size) {

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(input);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*output);
    
    // 典型的な削減: 150k点 → 30k点（5×高速化）
}
```

**性能:**
- 入力: 約150,000点 @ 10 Hz（屋外3D LiDAR）
- 出力: 約30,000点 @ 10 Hz（5cmボクセル）
- 処理時間: GMKtec Nucbox K6で<10ms

---

### 4.2 地面平面除去（RANSAC）

**目的:** 障害物を分離するため地面平面を除去

**アルゴリズム:**
```cpp
void removeGroundPlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacles,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& ground,
    const Eigen::Vector3d& imu_tilt) {

    // 1. IMU傾斜から地面平面モデルを構築
    //    地面法線 = IMUピッチ/ロールで回転した[0, 0, 1]
    Eigen::Vector3d ground_normal = rotateVector(Eigen::Vector3d(0, 0, 1), imu_tilt);
    
    // 2. RANSAC平面フィッティング
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);  // 5cm許容範囲
    seg.setMaxIterations(100);
    seg.setInputCloud(input);
    
    // オプションでIMUベース法線でシード
    if (use_imu_seed) {
        seg.setAxis(ground_normal);
        seg.setEpsAngle(0.1);  // IMUから10°許容範囲
    }
    
    seg.segment(*inliers, *coefficients);
    
    // 3. 障害物を抽出（非地面点）
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.setNegative(true);  // 非地面点を取得
    extract.filter(*obstacles);
    
    // 4. 地面を抽出（デバッグ用）
    extract.setNegative(false);
    extract.filter(*ground);
}
```

**IMU傾斜補償:**
```cpp
Eigen::Vector3d getGroundNormalFromIMU(const sensor_msgs::msg::Imu& imu) {
    // IMU四元数からピッチとロールを抽出
    tf2::Quaternion q(imu.orientation.x, imu.orientation.y, 
                      imu.orientation.z, imu.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    // z軸（上向きベクトル）をピッチとロールで回転
    Eigen::Vector3d up(0, 0, 1);
    Eigen::AngleAxisd rollRot(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchRot(pitch, Eigen::Vector3d::UnitY());
    
    Eigen::Vector3d ground_normal = pitchRot * rollRot * up;
    return ground_normal;
}
```

**性能:**
- 入力: 約30,000点（ボクセルダウンサンプリング後）
- 出力: 約15,000点（障害物のみ）
- 処理時間: GMKtec Nucbox K6で<20ms

---

### 4.3 高さフィルター

**目的:** 高さで障害物をフィルター（地面の破片と頭上構造物を無視）

**アルゴリズム:**
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
    
    // 典型: min=0.1m（小さな地面破片を無視）、max=2.0m（頭上を無視）
}
```

**高さ範囲:**
- 地面破片: z < 0.1m（無視）
- 関連障害物: 0.1m ≤ z ≤ 2.0m（人、車両、壁、車椅子）
- 頭上構造物: z > 2.0m（無視、低クリアランスエリア以外）

---

### 4.4 オプションクラスタリング（デフォルトでは未使用）

**目的:** 近傍点を離散障害物にグループ化（追跡に有用、バイナリ検出には不要）

**アルゴリズム（ユークリッドクラスタリング）:**
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
    ec.setMinClusterSize(min_size);     // 10点
    ec.setMaxClusterSize(max_size);     // 5000点
    ec.setSearchMethod(tree);
    ec.setInputCloud(input);
    ec.extract(cluster_indices);
    
    // インデックスを個別点群に変換
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*input, indices, *cluster);
        clusters.push_back(cluster);
    }
}
```

**デフォルトでは未使用:**
- バイナリ障害物検出はクラスタリング不要
- クラスタリングは計算オーバーヘッドを追加（約10-20ms）
- Nav2コストマップが内部で障害物グループ化を処理

---

## 5. コストマップ統合

### 5.1 Nav2ボクセルレイヤー

**設定（local_costmap.yaml）:**
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
        z_resolution: 0.2         # メートル（垂直ボクセルサイズ）
        z_voxels: 16              # 16 × 0.2m = 3.2m垂直範囲
        max_obstacle_height: 2.0  # メートル（高さフィルターから）
        min_obstacle_height: 0.1  # メートル（高さフィルターから）
        mark_threshold: 2         # ボクセル内の最小点数で障害物マーク
        
        # 点群ソース
        observation_sources: scan
        scan:
          topic: /points_filtered
          sensor_frame: lidar_frame
          data_type: PointCloud2
          marking: true           # 障害物をマーク
          clearing: true          # 自由空間をクリア
          min_obstacle_height: 0.1
          max_obstacle_height: 2.0
          obstacle_range: 10.0    # メートル（最大10mまで障害物マーク）
          raytrace_range: 15.0    # メートル（最大15mまで自由空間クリア）
```

**動作方法:**
1. ボクセルレイヤーが`/points_filtered`を受信（障害物のみ、高さ0.1-2.0m）
2. 3D点を2D占有グリッドに投影（トップダウンビュー）
3. コストマップ内の占有セルをマーク
4. ロボットから各点へレイトレースして自由空間をクリア
5. インフレーションレイヤーが障害物周辺に安全バッファを追加

---

### 5.2 3D → 2D投影

**アルゴリズム（Nav2ボクセルレイヤー内）:**
```cpp
void projectPointCloudToCostmap(
    const sensor_msgs::msg::PointCloud2& cloud,
    nav2_costmap_2d::Costmap2D& costmap) {

    // 1. ROSメッセージをPCLに変換
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud, *pcl_cloud);
    
    // 2. 各点について、対応するコストマップセルをマーク
    for (const auto& point : pcl_cloud->points) {
        // 必要に応じて点をコストマップフレームに変換
        geometry_msgs::msg::PointStamped point_in;
        point_in.point.x = point.x;
        point_in.point.y = point.y;
        point_in.point.z = point.z;
        
        // 2Dに投影（高さフィルター後にzを無視）
        unsigned int mx, my;
        if (costmap.worldToMap(point.x, point.y, mx, my)) {
            // セルを占有としてマーク（コスト = 254）
            costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
    }
    
    // 3. 自由空間クリアのためのレイトレース（Nav2ソース参照）
}
```

---

## 6. センサーフュージョン

### 6.1 IMU + LiDARフュージョン

**目的:** ロボット傾斜を補償（斜面、不均一な地面）

**融合戦略:**
- IMUが地面平面の向き（ピッチ、ロール）を提供
- RANSAC地面除去がIMUを初期シードとして使用
- より堅牢な地面検出

**利点:**
- 10°斜面でも正確な地面除去
- 地面が障害物としてマークされるのを防止
- より高速なRANSAC収束（必要反復回数が少ない）

---

### 6.2 カメラは知覚に未使用

**重要:** カメラはドッキング中のArUcoマーカー検出**のみ**に使用

**根拠:**
- オブジェクト分類不要（バイナリ障害物検出で十分）
- CPUのみ処理（深層学習用のGPUなし）
- 3D LiDARが360°カバレッジ提供（カメラは限定FOV）
- カメラは照明に敏感（屋外動作）

---

## 7. 性能最適化

### 7.1 CPU予算配分

**合計CPU予算:** 通常動作中<70%（8コア、16スレッド）

**知覚CPU配分:**
- ボクセルダウンサンプリング: 約5%（10 Hzで10ms）
- 地面除去（RANSAC）: 約10%（10 Hzで20ms）
- 高さフィルター: 約2%（10 Hzで5ms）
- Nav2ボクセルレイヤー: 約8%（Nav2が処理）
- **知覚合計:** 約25% CPU

**残予算:**
- ナビゲーション: 約20%
- ローカライゼーション（NDT）: 約15%
- 制御（スワーブドライブ）: 約5%
- その他: 約5%
- 予備: 約30%

---

### 7.2 処理パイプライン最適化

**並列化:**
```cpp
void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 1. ボクセルダウンサンプリング（シングルスレッド、高速）
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled = voxelGridFilter(input_cloud);
    
    // 2. 地面除去（PCLでマルチスレッドRANSAC利用可能）
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles = removeGroundPlane(downsampled);
    
    // 3. 高さフィルター（シングルスレッド、軽量）
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = heightFilter(obstacles);
    
    // 4. フィルター済み点群を配信
    publishFilteredCloud(filtered);
}
```

**主要最適化:**
- ボクセルダウンサンプリングを**最初に**実行（点を5×削減、すべての下流操作を高速化）
- IMUシードでRANSAC（より高速な収束、反復回数が少ない）
- クラスタリング**なし**（バイナリ検出には不要）
- 処理レートをLiDARレートにロック（10 Hz、それ以上速くしない）

---

## 8. テスト戦略

### 8.1 ユニットテスト
- ボクセルダウンサンプリング（既知点群→期待ダウンサンプル点群）
- 地面平面除去（合成地面+障害物→障害物のみ）
- 高さフィルター（既知高さ→期待フィルター出力）

### 8.2 統合テスト
- 記録LiDARデータでの完全パイプライン
- 斜面でのIMU傾斜補償
- フィルター済み点群からのコストマップ生成

### 8.3 フィールドテスト
- 地面除去精度（平坦地面、5°斜面、10°斜面）
- 障害物検出範囲（0.5m〜10m）
- 処理遅延（エンドツーエンド<50ms）
- CPU使用率（知覚サブシステム<25%）
- 屋外ロバスト性（雨、塵、変動照明 - LiDAR影響なし）

---

## 9. 性能目標

| メトリック | 目標 | 検証 |
|--------|--------|------------|
| 処理遅延 | <50ms（点群→コストマップ） | タイムスタンプ解析 |
| 処理レート | 10 Hz（LiDARに一致） | タイムスタンプ解析 |
| CPU使用率（知覚） | <25% | htop/システムモニター |
| 地面除去精度 | >95%（平坦）、>90%（10°斜面） | 手動ラベリング |
| 障害物検出範囲 | 0.5m - 10m | 検出テスト |
| 誤検出率 | <5%（非障害物がマーク） | フィールドテスト |
| 見逃し率 | <2%（障害物が見逃される） | フィールドテスト |

---

## 10. 実装フェーズ

### フェーズ1: 基本パイプライン（スプリント1-2）
- ✅ 3D LiDARドライバー統合
- ✅ ボクセルダウンサンプリング
- ✅ 高さフィルター
- ✅ 点群可視化（RViz）

### フェーズ2: 地面除去（スプリント3-4）
- ✅ RANSAC地面平面除去
- ✅ 傾斜補償用IMU統合
- ✅ 地面除去チューニング（平坦、斜面）

### フェーズ3: コストマップ統合（スプリント5-6）
- ✅ Nav2ボクセルレイヤー設定
- ✅ 3D → 2D投影
- ✅ 障害物マーキングと自由空間クリア
- ✅ ナビゲーションとの統合

### フェーズ4: 最適化&検証（スプリント7-8）
- ✅ CPUプロファイリングと最適化
- ✅ 屋外フィールドテスト
- ✅ 性能チューニング（精度 vs. 速度）
- ✅ ロバスト性テスト（天候、照明）

---

**文書ステータス:** ドラフト
**実装ステータス:** 未開始
**承認必要:** 知覚エンジニア、ナビゲーションリード、システムアーキテクト
