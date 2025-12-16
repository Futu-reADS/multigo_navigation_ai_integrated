# ナビゲーションコントローラ詳細設計

**ドキュメントID:** DESIGN-NAV-001
**バージョン:** 1.0
**日付:** 2025-12-15
**ステータス:** ドラフト

---

## 1. 概要

本ドキュメントは、実績のあるParcelPal 2フェーズナビゲーション戦略を実装するナビゲーションコントローラの詳細設計仕様を提供します: オフラインSLAMマッピング + オンラインNDT位置推定によるGPSフリー長距離ナビゲーション (500m-1km)。

**主要コンポーネント:**
- NavigationManager (ミッションコーディネーター)
- NDTLocalizationNode (マップベース位置推定)
- RouteManager (複数ウェイポイントルート実行)
- Nav2統合 (グローバル/ローカルプランニング、回復動作)
- MapManager (マップロード、切り替え、検証)

**主要機能:**
- GPS/GNSS不要 (コスト削減: $1,500-3,000)
- 連続SLAM不要 (ドリフト蓄積なし)
- 500m超で±10cm位置推定精度
- 複数ウェイポイントルートサポート (1ルートあたり4-8停止)
- 実績のあるParcelPalアーキテクチャ (60-70%再利用)

---

## 2. アーキテクチャ概要

### 2.1 2フェーズナビゲーション戦略

**フェーズ1: オフラインマッピング (手動、一回限り)**
```
人間オペレーターがジョイスティックでロボットをエリア内で運転
    ↓
SLAMアルゴリズム (Cartographer または SLAM Toolbox)
    ↓
2D占有グリッドマップ + ループクロージャを生成
    ↓
マップをファイルに保存 (map.pgm + map.yaml)
    ↓
完了 - このエリアでは二度とSLAMを実行しない
```

**フェーズ2: オンライン位置推定 (自律)**
```
フェーズ1から保存されたマップをロード
    ↓
NDTスキャンマッチング (現在のLiDARスキャン vs 保存されたマップ)
    ↓
マップフレームでロボット姿勢 (x, y, θ) を推定
    ↓
/odom → /map変換をパブリッシュ
    ↓
Nav2がグローバルプランニングにマップを使用
```

**なぜドリフトなしで機能するか:**
- SLAMはループクロージャ付きで一度だけオフライン実行 (保存されたマップにドリフトなし)
- NDTは固定された保存マップに対して位置推定 (ドリフト蓄積なし)
- 結果: 500m超で±10cm精度 (ParcelPalで実証済み)

---

## 3. クラス図

```
┌─────────────────────────────────────────────────────────────┐
│              NavigationManager                              │
│              (ミッションコーディネーター)                    │
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
│              (複数ウェイポイント管理)                        │
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
│              (マップベース位置推定)                          │
├─────────────────────────────────────────────────────────────┤
│ - ndt_: pcl::NormalDistributionsTransform                  │
│ - target_cloud_: pcl::PointCloud (保存マップ)              │
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
│              Nav2スタック統合                                │
├─────────────────────────────────────────────────────────────┤
│ - BT Navigator (ビヘイビアツリー実行)                       │
│ - Planner Server (グローバルパスプランニング)               │
│   - NavFn / Dijkstraプランナー                             │
│ - Controller Server (ローカル軌道実行)                      │
│   - SwerveDriveController (カスタムプラグイン)             │
│ - Smoother Server (パススムージング)                        │
│ - Recovery Behaviors (スタック回復、スピン、バックアップ)   │
└─────────────────────────────────────────────────────────────┘
```

---

## 4. 詳細コンポーネント仕様

### 4.1 NDTLocalizationNode

**ヘッダー (ndt_localization_node.hpp):**
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
  // NDTアルゴリズムパラメータ
  double resolution;               // メートル (例: 1.0)
  double step_size;                // メートル (例: 0.1)
  double transformation_epsilon;   // (例: 0.01)
  int max_iterations;             // (例: 30)

  // 収束基準
  double fitness_score_threshold;  // (例: 1.0)
  double max_correspondence_distance; // メートル (例: 2.0)

  // パフォーマンス
  int num_threads;                 // (例: 4)

  // 初期姿勢不確実性
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

  // NDTソルバー
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;

  // マップ点群 (NDTの目標)
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;

  // 現在の姿勢推定
  Eigen::Matrix4f current_pose_;
  bool pose_initialized_{false};

  // ROSインターフェース
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // パラメータ
  NDTParameters params_;
};

}  // namespace navigation

#endif  // NDT_LOCALIZATION_NODE_HPP_
```

**主要メソッド実装:**

**performNDTAlignment:**
```cpp
bool NDTLocalizationNode::performNDTAlignment(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
  Eigen::Matrix4f& transformation)
{
  // 入力クラウドを設定 (現在のLiDARスキャン)
  ndt_.setInputSource(input_cloud);

  // 初期推定 (前回の姿勢または初期姿勢を使用)
  Eigen::Matrix4f guess = pose_initialized_ ? current_pose_ : getInitialPoseMatrix();

  // NDTアライメントを実行
  pcl::PointCloud<pcl::PointXYZ> output_cloud;
  ndt_.align(output_cloud, guess);

  // 収束をチェック
  bool converged = ndt_.hasConverged();
  double fitness_score = ndt_.getFitnessScore();

  if (!converged || !checkConvergence(fitness_score)) {
    RCLCPP_WARN(logger_, "NDTが収束しませんでした: fitness=%.3f, converged=%d",
                fitness_score, converged);
    return false;
  }

  // 最終変換を取得
  transformation = ndt_.getFinalTransformation();

  RCLCPP_DEBUG(logger_, "NDTが収束しました: fitness=%.3f, iterations=%d",
               fitness_score, ndt_.getFinalNumIteration());

  return true;
}

bool NDTLocalizationNode::checkConvergence(double fitness_score) {
  // fitnessスコアが低いほど良いアライメント
  // 屋外環境での良好なfitness: <1.0
  return fitness_score < params_.fitness_score_threshold;
}
```

**loadMapPointCloud:**
```cpp
bool NDTLocalizationNode::loadMapPointCloud(const std::string& map_pcd_file) {
  target_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_pcd_file, *target_cloud_) == -1) {
    RCLCPP_ERROR(logger_, "マップ点群のロード失敗: %s", map_pcd_file.c_str());
    return false;
  }

  RCLCPP_INFO(logger_, "マップ点群をロードしました: %zuポイント", target_cloud_->size());

  // NDTの目標クラウドを設定
  ndt_.setInputTarget(target_cloud_);

  // NDTパラメータを設定
  ndt_.setResolution(params_.resolution);
  ndt_.setStepSize(params_.step_size);
  ndt_.setTransformationEpsilon(params_.transformation_epsilon);
  ndt_.setMaximumIterations(params_.max_iterations);

  // パフォーマンスのためにマルチスレッディングを有効化
  ndt_.setNumThreads(params_.num_threads);

  return true;
}
```

---

### 4.2 RouteManager

**ルートファイル形式 (YAML):**
```yaml
route:
  id: "warehouse_to_cafeteria"
  name: "Main Building Route"
  map_id: "main_building_floor_1"

  waypoints:
    - id: 0
      name: "車椅子ピックアップ"
      position:
        x: 10.5
        y: 3.2
        theta: 0.0  # ラジアン
      action: "DOCK_WHEELCHAIR"
      dwell_time: 30.0  # 秒

    - id: 1
      name: "廊下チェックポイント"
      position:
        x: 25.0
        y: 3.5
        theta: 0.0
      action: "PASS_THROUGH"
      dwell_time: 0.0

    - id: 2
      name: "エレベーター待機エリア"
      position:
        x: 42.0
        y: 8.0
        theta: 1.57  # 90° 右
      action: "WAIT_FOR_ELEVATOR"
      dwell_time: 60.0  # 最大待機時間

    - id: 3
      name: "目的地 (カフェテリア)"
      position:
        x: 55.0
        y: 15.0
        theta: 3.14  # 180°
      action: "UNDOCK_WHEELCHAIR"
      dwell_time: 30.0
```

**実装:**
```cpp
bool RouteManager::loadRoute(const std::string& route_file) {
  // YAMLファイルを解析
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

  RCLCPP_INFO(logger_, "ルート '%s' を %zu ウェイポイントでロードしました",
              route_id_.c_str(), waypoints_.size());

  return true;
}

Waypoint RouteManager::getNextWaypoint() {
  if (current_waypoint_idx_ >= waypoints_.size()) {
    throw std::runtime_error("ルートにこれ以上ウェイポイントがありません");
  }

  return waypoints_[current_waypoint_idx_];
}

void RouteManager::markWaypointComplete() {
  if (current_waypoint_idx_ < waypoints_.size()) {
    RCLCPP_INFO(logger_, "ウェイポイント %d '%s' 完了",
                waypoints_[current_waypoint_idx_].id,
                waypoints_[current_waypoint_idx_].name.c_str());

    current_waypoint_idx_++;

    if (isRouteComplete()) {
      route_status_ = RouteStatus::COMPLETED;
      RCLCPP_INFO(logger_, "ルート '%s' 完了!", route_id_.c_str());
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

**ミッション実行状態機械:**
```
┌──────────────────────────────────────────────────────────────┐
│           ミッション実行状態機械                              │
└──────────────────────────────────────────────────────────────┘

    ┌────────┐
    │  IDLE  │
    └───┬────┘
        │ START_MISSION
        ↓
    ┌──────────┐
    │LOAD_ROUTE│ ← ルートYAMLをロード、ウェイポイントを検証
    └────┬─────┘
         │ ROUTE_LOADED
         ↓
    ┌──────────┐
    │LOAD_MAP  │ ← このルート用の保存マップをロード
    └────┬─────┘
         │ MAP_LOADED
         ↓
    ┌──────────┐
    │LOCALIZE  │ ← NDT収束を待つ
    └────┬─────┘
         │ LOCALIZED
         ↓
    ┌──────────┐
    │NAVIGATE  │ ← ウェイポイントを順次実行
    └────┬─────┘
         │ ループ: 各ウェイポイント
         │   ↓
         │  ┌─────────────────┐
         │  │ Nav2 NavigateToPose(waypoint)
         │  └─────────────────┘
         │   ↓
         │  ┌─────────────────┐
         │  │ WAYPOINT_REACHED
         │  │ アクション実行 (ドック、アンドック、待機)
         │  └─────────────────┘
         │   ↓
         │  ┌─────────────────┐
         │  │ 次のウェイポイント
         │  └─────────────────┘
         │
         │ ALL_WAYPOINTS_COMPLETE
         ↓
    ┌──────────┐
    │COMPLETE  │
    └──────────┘

    エラーハンドリング:
    ┌─────────┐
    │ FAILED  │ ← 位置推定失敗 / ナビゲーションタイムアウト / 障害物ブロック
    └──────────┘
```

**実装:**
```cpp
void NavigationManager::executeMission() {
  auto mission = current_mission_;

  // 1. ルートをロード
  if (!route_manager_->loadRoute(mission.route_file)) {
    RCLCPP_ERROR(logger_, "ルートのロード失敗: %s", mission.route_file.c_str());
    missionFailed("ルートロード失敗");
    return;
  }

  // 2. マップをロード
  std::string map_file = getMapPath(route_manager_->getMapId());
  if (!map_manager_->loadMap(map_file)) {
    RCLCPP_ERROR(logger_, "マップのロード失敗: %s", map_file.c_str());
    missionFailed("マップロード失敗");
    return;
  }

  // 3. 位置推定収束を待つ
  if (!waitForLocalization(30.0)) {  // 30秒タイムアウト
    RCLCPP_ERROR(logger_, "位置推定タイムアウト");
    missionFailed("位置推定失敗");
    return;
  }

  // 4. ウェイポイントを実行
  while (!route_manager_->isRouteComplete()) {
    Waypoint wp = route_manager_->getNextWaypoint();

    RCLCPP_INFO(logger_, "ウェイポイント %d へナビゲート中: '%s'", wp.id, wp.name.c_str());

    // Nav2にナビゲーションゴールを送信
    auto goal = createNavigationGoal(wp);
    auto result_future = nav2_client_->async_send_goal(goal);

    // 結果を待つ
    auto result = result_future.get();

    if (result->code != NavigationResult::SUCCESS) {
      RCLCPP_ERROR(logger_, "ウェイポイント %d へのナビゲーション失敗: %d", wp.id, result->code);
      handleNavigationFailure(wp, result->code);
      continue;  // ポリシーに基づいて再試行またはスキップ
    }

    // ウェイポイントアクションを実行
    executeWaypointAction(wp);

    // 滞在時間
    if (wp.dwell_time > 0.0) {
      RCLCPP_INFO(logger_, "ウェイポイント %d で %.1f秒滞在", wp.id, wp.dwell_time);
      rclcpp::sleep_for(std::chrono::duration<double>(wp.dwell_time));
    }

    // 完了をマーク
    route_manager_->markWaypointComplete();
  }

  // ミッション完了
  RCLCPP_INFO(logger_, "ミッション '%s' 完了!", mission.id.c_str());
  publishMissionStatus(MissionStatus::COMPLETED);
}
```

---

## 5. Nav2統合

### 5.1 Nav2設定 (nav2_params.yaml)

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
      tolerance: 0.5                # メートル
      use_astar: false              # Dijkstraは屋外開放空間で十分
      allow_unknown: false          # 未知領域を通るプランニング禁止
      use_final_approach_orientation: true

controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "swerve_drive_controller::SwerveDriveController"
      # (完全な設定はSWERVE_DRIVE_CONTROLLER_DESIGN.mdを参照)

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

## 6. マップ管理

### 6.1 マップファイル構造

```
maps/
├── main_building_floor_1/
│   ├── map.pgm                # 占有グリッド画像
│   ├── map.yaml               # マップメタデータ
│   └── map.pcd                # NDT用点群
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

**map.yaml 例:**
```yaml
image: map.pgm
resolution: 0.05      # メートル/ピクセル
origin: [-50.0, -30.0, 0.0]  # [x, y, theta]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

**map.pcd:** SLAMセッションの点群データから生成、NDT効率のため〜50k-100kポイントにダウンサンプリング。

---

## 7. エラーハンドリングと回復

### 7.1 位置推定失敗回復

```cpp
bool NavigationManager::waitForLocalization(double timeout) {
  auto start_time = node_->now();

  while ((node_->now() - start_time).seconds() < timeout) {
    // NDT fitnessスコアをチェック
    double fitness_score = ndt_localization_->getFitnessScore();

    if (fitness_score < ndt_fitness_threshold_) {
      RCLCPP_INFO(logger_, "位置推定が収束しました: fitness=%.3f", fitness_score);
      return true;
    }

    // ロボットが静止しているかチェック (良好なNDT収束に必要)
    if (!isRobotStationary()) {
      RCLCPP_WARN(logger_, "位置推定中にロボットが移動中 - 静止を待っています");
    }

    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_ERROR(logger_, "%.1f秒後に位置推定タイムアウト", timeout);
  return false;
}
```

### 7.2 ナビゲーション失敗ハンドリング

**一般的な失敗モード:**
1. **タイムアウト:** Nav2が制限時間内にパスを見つけられないまたはゴールに到達できない
2. **ブロック:** 障害物がパスを永続的にブロック
3. **スタック:** ロボットが進行できない (車輪スリップなど)
4. **ロスト:** 位置推定品質が低下

**回復戦略:**
```cpp
void NavigationManager::handleNavigationFailure(
  const Waypoint& wp,
  int error_code)
{
  retry_count_[wp.id]++;

  if (retry_count_[wp.id] >= max_retries_per_waypoint_) {
    RCLCPP_ERROR(logger_, "ウェイポイント %d の最大再試行回数を超えました - スキップ", wp.id);
    route_manager_->markWaypointComplete();  // このウェイポイントをスキップ
    return;
  }

  switch (error_code) {
    case NavigationResult::ERROR_TIMEOUT:
      RCLCPP_WARN(logger_, "ナビゲーションタイムアウト - より長いタイムアウトで再試行");
      // 再試行のためのタイムアウトを増加
      break;

    case NavigationResult::ERROR_BLOCKED:
      RCLCPP_WARN(logger_, "パスがブロックされています - 障害物が除去されるまで30秒待機");
      rclcpp::sleep_for(std::chrono::seconds(30));
      // ナビゲーションを再試行
      break;

    case NavigationResult::ERROR_STUCK:
      RCLCPP_WARN(logger_, "ロボットがスタック - 回復動作を実行");
      executeRecoveryBehavior("backup");  // バックアップして再試行
      break;

    case NavigationResult::ERROR_LOST:
      RCLCPP_ERROR(logger_, "位置推定を喪失 - 再位置推定中");
      if (!waitForLocalization(30.0)) {
        missionFailed("再位置推定失敗");
      }
      break;
  }
}
```

---

## 8. パフォーマンス最適化

### 8.1 NDTパフォーマンスチューニング

**計算予算:**
- NDTアライメント: GMKtec Nucbox K6で10 Hz @ <100ms
- 目標: スムーズなナビゲーションのための10 Hz位置推定レート

**最適化:**
```cpp
// 1. 入力スキャンのボクセルダウンサンプリング
pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
voxel_filter.setLeafSize(0.5, 0.5, 0.5);  // 50cmボクセル
voxel_filter.setInputCloud(raw_scan);
voxel_filter.filter(*downsampled_scan);

// 2. マルチスレッディング
ndt_.setNumThreads(4);  // 4 CPUコアを使用

// 3. 高速収束時は最大反復回数を削減
if (recent_fitness_scores_good) {
  ndt_.setMaximumIterations(15);  // 30から削減
}

// 4. 前回の姿勢を良好な初期推定として使用 (反復回数を削減)
ndt_.align(output_cloud, previous_pose_);
```

---

## 9. テスト戦略

### 9.1 単体テスト

**テスト: NDT位置推定精度**
```cpp
TEST(NDTLocalizationTest, AccuracyOnKnownMap) {
  NDTLocalizationNode ndt_node;

  // テストマップをロード
  ndt_node.loadMapPointCloud("test_data/test_map.pcd");

  // 既知のグラウンドトゥルース姿勢を持つテストスキャンをロード
  std::vector<TestScan> test_scans = loadTestScans("test_data/scans_with_gt.txt");

  for (const auto& test_scan : test_scans) {
    auto estimated_pose = ndt_node.localize(test_scan.scan);

    // グラウンドトゥルースと比較
    double position_error = (estimated_pose.position - test_scan.gt_pose.position).norm();
    double orientation_error = computeOrientationError(estimated_pose, test_scan.gt_pose);

    EXPECT_LT(position_error, 0.10);  // <10cm
    EXPECT_LT(orientation_error, 0.05);  // <3°
  }
}
```

**テスト: ルートマネージャー**
```cpp
TEST(RouteManagerTest, WaypointSequence) {
  RouteManager route_mgr;

  ASSERT_TRUE(route_mgr.loadRoute("test_data/test_route.yaml"));

  EXPECT_EQ(route_mgr.getNumWaypoints(), 4);
  EXPECT_FALSE(route_mgr.isRouteComplete());

  // ウェイポイントを順番に取得
  auto wp1 = route_mgr.getNextWaypoint();
  EXPECT_EQ(wp1.id, 0);

  route_mgr.markWaypointComplete();

  auto wp2 = route_mgr.getNextWaypoint();
  EXPECT_EQ(wp2.id, 1);

  // ... すべてのウェイポイントで続行

  route_mgr.markWaypointComplete();
  EXPECT_TRUE(route_mgr.isRouteComplete());
}
```

---

### 9.2 統合テスト

**テスト: 完全ミッション実行 (シミュレーション)**
```cpp
TEST_F(NavigationIntegrationTest, CompleteMission) {
  // Gazeboで既知の開始位置にロボットを生成
  spawnRobot(Pose(0, 0, 0));

  // 事前構築マップをロード
  loadMap("test_maps/outdoor_test_map");

  // 4ウェイポイントでミッション開始
  auto mission = createTestMission(4);
  navigation_manager_->startMission(mission);

  // ミッション完了を待つ (タイムアウト: 300秒)
  auto result = waitForMissionComplete(300.0);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.waypoints_completed, 4);

  // 最終位置を検証
  auto final_pose = getRobotPose();
  auto expected_pose = mission.waypoints.back().position;

  double position_error = (final_pose.position - expected_pose.position).norm();
  EXPECT_LT(position_error, 0.5);  // 最終ウェイポイントで<50cm
}
```

---

## 10. デプロイメントチェックリスト

**自律ナビゲーションをデプロイする前に:**

1. ✅ **マップ品質:** 保存されたマップが良好なループクロージャを持つか検証 (大きなドリフトなし)
2. ✅ **NDTチューニング:** マップでのNDT位置推定精度をテスト (<10cm誤差)
3. ✅ **ルート検証:** すべてのウェイポイントが到達可能で衝突フリーかテスト
4. ✅ **回復動作:** ロボットがスタック/ブロック状況から回復できるか検証
5. ✅ **安全性統合:** ナビゲーション中に緊急停止が機能するか確認
6. ✅ **パフォーマンス:** ターゲットハードウェアでNDT @ 10 Hz、Nav2コントローラ @ 20 Hz

---

**ドキュメントステータス:** ドラフト
**実装ステータス:** 開発準備完了 (ParcelPal実証済みアーキテクチャ)
**必要な承認:** ナビゲーションリード、パーセプションリード、システム統合エンジニア
