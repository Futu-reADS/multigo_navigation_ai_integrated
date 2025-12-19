# ナビゲーションサブシステムアーキテクチャ

**文書ID:** ARCH-NAV-001
**バージョン:** 1.0
**日付:** 2025年12月15日
**ステータス:** ドラフト

---

## 1. 概要

**ナビゲーションサブシステム**は、GPSなしで500m〜1kmのルートに対する自律ナビゲーション機能を提供します。このアーキテクチャは、**実証済みのParcelPalパターンに従います**（コード再利用0%、アーキテクチャアプローチ70%）、スワーブドライブと車椅子搬送要件に適応しています。

**主要機能:**
- 2フェーズナビゲーション（オフラインマッピング + オンラインローカライゼーション）
- GPS不要のローカライゼーションのためのNDTスキャンマッチング（500mで±10cm）
- マルチウェイポイントルート実行（ルートあたり4〜8停止）
- 全方向経路計画（スワーブドライブ機能）
- 動的障害物回避
- 故障処理のためのリカバリービヘイビア

**ローカライゼーション戦略（ParcelPal実証済み）:**
- **フェーズ1（オフライン）:** ループクロージャを使用した手動SLAMマッピング1回 → 静的マップを保存
- **フェーズ2（オンライン）:** 保存されたマップ上でのNDTローカライゼーション → ドリフト累積なし！

---

## 2. システムアーキテクチャ

### 2.1 コンポーネント図

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

## 3. ROS 2ノードアーキテクチャ

### 3.1 Nav2スタックノード

**標準Nav2ノード（ros-humble-navigation2から）:**

1. **BT Navigator** (`bt_navigator`)
   - ビヘイビアツリー実行を管理
   - プランナー、コントローラー、リカバリーサーバーを調整
   - ウェイポイントフォロイングを処理

2. **Planner Server** (`planner_server`)
   - スタートからゴールまでのグローバルパスを生成
   - プラグインアーキテクチャ: NavFn、Smac Planner Hybrid-A*
   - パス可視化を公開

3. **Controller Server** (`controller_server`)
   - ローカル軌道最適化でグローバルパスに従う
   - プラグインアーキテクチャ: DWB、TEB、**Swerve Controller（カスタム）**
   - `/cmd_vel`コマンドを公開

4. **Recovery Server** (`recoveries_server`)
   - 失敗時にリカバリービヘイビアを実行
   - ビヘイビア: スピン、バックアップ、待機

5. **Waypoint Follower** (`waypoint_follower`)
   - ウェイポイントのリストをナビゲート
   - アクションインターフェース: `nav2_msgs/action/FollowWaypoints`

6. **Lifecycle Manager** (`lifecycle_manager`)
   - Nav2ノードのライフサイクルを管理
   - 起動、シャットダウン、リカバリー

---

### 3.2 ローカライゼーションノード（NDT - 標準ROS 2）

**ノード名:** `ndt_localization`
**パッケージ:** `ndt_omp`または`pcl::NormalDistributionsTransform`を使用したカスタム実装（Autowareではない）
**言語:** C++
**注記:** 新しいコードでParcelPalの実証済み2フェーズナビゲーションアプローチを実装

**サブスクライブトピック:**
- `/points`（sensor_msgs/PointCloud2）: 3D LiDARポイントクラウド（10 Hz）
- `/imu/data`（sensor_msgs/Imu）: 方向用IMU（50 Hz）
- `/odom`（nav_msgs/Odometry）: ホイールオドメトリ（50 Hz）

**パブリッシュトピック:**
- `/tf`（tf2_msgs/TFMessage）: map → odom変換
- `/ndt_pose`（geometry_msgs/PoseStamped）: 現在の姿勢推定
- `/ndt_score`（std_msgs/Float32）: NDTアライメントスコア（収束品質）

**サービス:**
- `/ndt/set_initial_pose`（geometry_msgs/PoseWithCovarianceStamped）: 手動初期化

**パラメーター:**
```yaml
# NDT設定（Autowareデフォルト）
ndt:
  trans_epsilon: 0.01             # 収束閾値（並進）
  step_size: 0.1                  # ニュートン最適化ステップサイズ
  resolution: 1.0                 # ボクセルグリッド解像度（メートル）
  max_iterations: 30              # 最大最適化反復

  # パフォーマンス
  num_threads: 4                  # 並列実行

  # 初期化
  initial_pose_topic: "/initialpose"  # RVizまたはUIから
  use_imu: true                   # IMU支援初期化
  use_odom: true                  # オドメトリ支援予測

# マップ
map_topic: "/map"                 # 事前構築された静的マップ
map_frame: "map"
base_frame: "base_link"
odom_frame: "odom"
```

---

### 3.3 ルートマネージャーノード（新規）

**ノード名:** `route_manager`
**言語:** C++またはPython
**ライフサイクル:** 標準ノード

**サブスクライブトピック:**
- `/ndt_pose`（geometry_msgs/PoseStamped）: 現在のロボット姿勢
- `/waypoint_follower/feedback`（nav2_msgs/action/FollowWaypoints/Feedback）: ナビゲーション進捗

**パブリッシュトピック:**
- `/route_status`（custom_msgs/RouteStatus）: ルート進捗（完了したウェイポイント、ETA）
- `/mission_markers`（visualization_msgs/MarkerArray）: ウェイポイント可視化

**アクションサーバー:**
- `/execute_route`（custom_msgs/action/ExecuteRoute）: マルチストップルートの実行

**アクションクライアント:**
- `/follow_waypoints`（nav2_msgs/action/FollowWaypoints）: Nav2ウェイポイントフォロワー

**パラメーター:**
```yaml
# ルート設定
max_waypoints_per_route: 8        # ミッションあたりの最大停止数
waypoint_tolerance: 0.3           # メートル（到達閾値）
reroute_on_failure: true          # 失敗時に代替パスを試行
max_reroute_attempts: 3           # N回失敗後に諦める

# ウェイポイントデータベース
waypoint_file: "config/waypoints.yaml"  # 事前定義されたウェイポイント位置
```

---

## 4. 2フェーズナビゲーション戦略

### 4.1 フェーズ1: オフラインマッピング（手動、1回のみ）

**目標:** ループクロージャを使用した高品質静的マップの作成

**手順:**
```bash
# 1. SLAM Toolboxをマッピングモードで起動
ros2 launch slam_toolbox online_async_launch.py

# 2. ロボットを運用エリア全体（500m〜1km）を手動で走行させる
#    - 高品質スキャンのためにゆっくり走行（0.5 m/s）
#    - ループクロージャのために開始位置を再訪
#    - すべてのパス、ドッキングステーション、ウェイポイントをカバー

# 3. ループクロージャ収束後にマップを保存
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: {data: 'campus_map'}"

# 4. 生成されたマップファイル:
#    - campus_map.yaml（マップメタデータ）
#    - campus_map.pgm（占有グリッド画像）
```

**マップ品質検証:**
- ループクロージャエラー: <10cm（SLAM Toolboxログを確認）
- ゴーストウォールや重複フィーチャーなし
- すべてのドッキングステーションがマップ上に表示
- ウェイポイントが既知の位置に対応

---

### 4.2 フェーズ2: オンラインローカライゼーション（運用モード）

**目標:** 保存されたマップ上で<±10cm精度でロボットをローカライズ

**手順:**
```bash
# 1. 事前構築マップをロード
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=campus_map.yaml

# 2. NDTローカライゼーションを起動
ros2 launch autoware_localization ndt_localization.launch.py

# 3. ロボット姿勢を初期化（次のいずれかの方法）:
#    a) RVizの「2D Pose Estimate」ツールで手動初期化
#    b) 最後に既知の姿勢からの自動初期化（config/last_pose.yaml）
#    c) 非常に不確実な場合のマルチ仮説パーティクルフィルター初期化

# 4. NDTが正確な姿勢に収束（通常<5秒）
#    - /ndt_scoreトピックを監視（良好なアライメントには>0.5が必要）
#    - RVizで姿勢を確認（LiDARスキャンがマップにオーバーレイ）
```

**ローカライゼーション監視:**
```cpp
void monitorLocalization() {
    // /ndt_scoreをサブスクライブ
    double ndt_score = getCurrentNDTScore();

    if (ndt_score < 0.3) {
        // アライメント不良 - ローカライゼーションが失われた可能性
        triggerRelocalization();
    }

    // 姿勢共分散も監視
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    double position_std = sqrt(pose.pose.covariance[0]);  // x分散

    if (position_std > 0.5) {
        // 高不確実性 - ローカライゼーション劣化
        reduceSpeed();  // 安全対策
    }
}
```

---

## 5. 経路計画

### 5.1 グローバルプランナー（NavFn / Smac Hybrid-A*）

**アルゴリズム選択:**
- **NavFn（Dijkstra）:** 高速、シンプル、ホロノミック仮定（スワーブドライブに適している）
- **Smac Hybrid-A*:** 制約された空間に適している、ロボットフットプリントを考慮

**設定（nav2_params.yaml）:**
```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5                # ゴール許容範囲（メートル）
      use_astar: false              # Dijkstra（屋外オープンスペースで高速）
      allow_unknown: true           # 未知空間を通過計画（屋外拡張）
```

**代替（複雑な屋内/屋外遷移用）:**
```yaml
    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      tolerance: 0.5
      downsample_costmap: false     # 完全解像度を使用
      allow_unknown: true
      max_planning_time: 5.0        # 秒
```

---

### 5.2 ローカルコントローラー（DWB / Swerve Controller）

**DWBコントローラー（標準Nav2）:**
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0    # Hz
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      # 速度制限（全方向！）
      max_vel_x: 1.5
      max_vel_y: 1.5              # スワーブドライブは横移動可能
      max_vel_theta: 2.0
      min_vel_x: -1.5
      min_vel_y: -1.5

      # 軌道生成
      sim_time: 1.7               # 先読みシミュレーション秒数
      vx_samples: 20
      vy_samples: 20              # 横速度をサンプリング
      vtheta_samples: 40

      # 軌道スコアリング
      path_distance_bias: 32.0    # グローバルパスに留まることを優先
      goal_distance_bias: 24.0    # ゴールに向かうことを優先
      occdist_scale: 0.01         # 障害物を回避
```

**カスタムSwerve Controller（SWERVE_DRIVE_ARCHITECTURE.mdから）:**
```yaml
    FollowPath:
      plugin: "swerve_drive_controller::SwerveDriveController"
      # 完全な設定についてはSWERVE_DRIVE_ARCHITECTURE.mdを参照
```

---

## 6. マルチウェイポイントナビゲーション

### 6.1 ウェイポイント定義

**ウェイポイントファイル（config/waypoints.yaml）:**
```yaml
waypoints:
  - id: "building_a_entrance"
    pose:
      position: {x: 10.5, y: 20.3, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}  # 東向き
    type: "pickup"

  - id: "building_b_entrance"
    pose:
      position: {x: 150.2, y: 35.8, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}  # 北向き
    type: "dropoff"

  - id: "building_c_entrance"
    pose:
      position: {x: 280.7, y: -12.4, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: -0.707, w: 0.707}  # 西向き
    type: "dropoff"
```

---

### 6.2 ルート実行

**アクションインターフェース:**
```cpp
// custom_msgs/action/ExecuteRoute.action
# Goal
string[] waypoint_ids     # 訪問するウェイポイントIDの順序付きリスト
---
# Result
bool success
string message
int32 waypoints_completed
---
# Feedback
int32 current_waypoint_index
string current_waypoint_id
float32 distance_remaining  # 現在のウェイポイントまでのメートル
float32 eta_seconds
```

**ルートマネージャー実装:**
```cpp
void executeRoute(const ExecuteRoute::Goal& goal) {
    // 1. 設定からウェイポイントをロード
    std::vector<geometry_msgs::msg::PoseStamped> waypoint_poses;
    for (const auto& waypoint_id : goal.waypoint_ids) {
        waypoint_poses.push_back(loadWaypoint(waypoint_id));
    }

    // 2. Nav2ウェイポイントフォロワーを呼び出し
    nav2_msgs::action::FollowWaypoints::Goal nav2_goal;
    nav2_goal.poses = waypoint_poses;

    auto nav2_result = waypoint_follower_client_->sendGoal(nav2_goal);

    // 3. 進捗を監視してフィードバックを公開
    while (!nav2_result->is_finished()) {
        ExecuteRoute::Feedback feedback;
        feedback.current_waypoint_index = getCurrentWaypointIndex();
        feedback.distance_remaining = computeDistanceToWaypoint();
        feedback.eta_seconds = estimateETA();
        sendFeedback(feedback);
    }

    // 4. 結果を処理
    if (nav2_result->success) {
        ExecuteRoute::Result result;
        result.success = true;
        result.waypoints_completed = goal.waypoint_ids.size();
        sendResult(result);
    } else {
        // 再ルーティングまたは失敗処理
        handleNavigationFailure(nav2_result->message);
    }
}
```

---

## 7. コストマップ設定

### 7.1 グローバルコストマップ（静的マップ）

**目的:** 事前構築マップでの長距離経路計画

**設定:**
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0       # Hz（静的マップ、まれな更新）
      publish_frequency: 1.0
      robot_base_frame: base_link
      global_frame: map

      # マッププラグイン
      plugins: ["static_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: /map

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.5     # メートル（安全バッファ）
        cost_scaling_factor: 3.0
```

---

### 7.2 ローカルコストマップ（動的障害物）

**目的:** LiDARからのリアルタイム障害物回避

**設定:**
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0       # Hz
      publish_frequency: 2.0
      robot_base_frame: base_link
      global_frame: odom

      width: 5                    # メートル（ローカルウィンドウ）
      height: 5
      resolution: 0.05            # メートル/ピクセル（5cm）

      # プラグイン
      plugins: ["voxel_layer", "inflation_layer"]

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.2         # メートル（垂直解像度）
        z_voxels: 16              # 3.2m垂直範囲
        max_obstacle_height: 2.0  # メートル（頭上障害物を無視）
        min_obstacle_height: 0.1  # メートル（地面を無視）
        mark_threshold: 2         # 障害物をマークする最小ポイント数

        # ポイントクラウドソース
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
        inflation_radius: 0.3     # メートル（屋内は狭く、屋外は広く）
        cost_scaling_factor: 3.0
```

---

## 8. リカバリービヘイビア

### 8.1 故障シナリオ

**一般的な故障モード:**
1. **パスブロック:** パス上の一時的な障害物
2. **ローカライゼーション喪失:** NDTスコア低下、高共分散
3. **ゴール到達不能:** ゴールへの有効なパスなし
4. **ロボットスタック:** ホイールスリップ、進捗なし

---

### 8.2 リカバリーアクション

**設定:**
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
      wait_duration: 5.0          # 秒
```

**ビヘイビアツリー統合:**
```xml
<!-- Nav2デフォルトビヘイビアツリー（抜粋） -->
<BehaviorTree>
  <PipelineSequence name="NavigateWithReplanning">
    <RateController hz="1.0">
      <RecoveryNode number_of_retries="6" name="NavigateRecovery">
        <PipelineSequence name="NavigateWithReplanning">
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
          <FollowPath path="{path}" controller_id="FollowPath"/>
        </PipelineSequence>
        <ReactiveFallback name="RecoveryFallback">
          <Spin spin_dist="1.57"/>        <!-- 90°スピン -->
          <Wait wait_duration="5.0"/>
          <BackUp backup_dist="0.3" backup_speed="0.1"/>
        </ReactiveFallback>
      </RecoveryNode>
    </RateController>
  </PipelineSequence>
</BehaviorTree>
```

---

## 9. 他サブシステムとの統合

### 9.1 ローカライゼーション統合

**データフロー:**
- NDTローカライゼーションが`map → odom` TF変換を公開
- Nav2がTFルックアップ（`map → base_link`）経由でロボット姿勢を受信
- スワーブドライブからのオドメトリが`odom → base_link` TFを公開

**TFツリー:**
```
map（NDTから、静的マップフレーム）
 └─ odom（NDTから、NDTで補正）
     └─ base_link（オドメトリから、ホイールエンコーダー）
         └─ lidar_frame（静的、URDFから）
         └─ camera_1_frame（静的、URDFから）
```

---

### 9.2 パーセプション統合

**障害物検出:**
- パーセプションサブシステムが`/points`（フィルタリングされた3D LiDAR）を公開
- ローカルコストマップの`voxel_layer`が`/points`を消費
- 障害物がコストマップにマーク → コントローラーが回避

**地面除去:**
- パーセプションが地面平面除去を実行（PERCEPTION_SUBSYSTEM_ARCHITECTURE.mdを参照）
- 障害物のみ（高さ0.1m〜2.0m）がコストマップに到達

---

### 9.3 スワーブドライブ統合

**コマンドインターフェース:**
- コントローラーサーバーが`/cmd_vel`（geometry_msgs/Twist）を公開
- スワーブドライブコントローラーが`/cmd_vel`をサブスクライブ
- スワーブドライブが全方向運動（vx、vy、ω）を実行

**利点:**
- 横移動（vy）により回転なしでタイトコーナリングが可能
- DWBコントローラーがvy軌道をサンプリング可能

---

## 10. テスト戦略

### 10.1 ユニットテスト
- 経路計画の正確性（既知のマップ → 期待されるパス）
- NDTローカライゼーション精度（シミュレートされたスキャン → 既知の姿勢）
- ウェイポイントのロードと解析

### 10.2 統合テスト
- Nav2スタック統合（全ノード実行中）
- ローカライゼーション + ナビゲーション（シミュレートされたマップ + LiDAR）
- マルチウェイポイント実行（3〜5ウェイポイント）

### 10.3 フィールドテスト
- マッピング品質（ループクロージャエラー <10cm）
- 距離に対するローカライゼーション精度（500mで±10cm）
- パスフォロイング精度（クロストラックエラー <20cm）
- 障害物回避（動的障害物）
- マルチウェイポイントルート（4〜8停止、合計500m〜1km）
- リカバリービヘイビア有効性（ブロックされたパス、スタックロボット）

---

## 11. パフォーマンス目標

| 指標 | 目標 | 検証方法 |
|--------|--------|------------|
| ローカライゼーション精度 | 500mで±10cm | GPSグラウンドトゥルースまたは測量されたウェイポイント |
| パスフォロイング精度 | <20cmクロストラックエラー | ログされたパス vs. 計画されたパス |
| 計画時間（グローバル） | 500mパスで<1秒 | タイムスタンプ解析 |
| 制御ループレート | 20 Hz | タイムスタンプ解析 |
| 障害物反応時間 | <500ms | 動的障害物テスト |
| ウェイポイント到達閾値 | ±30cm | フィールドテスト |
| マルチウェイポイント成功率 | >90%（500m〜1kmルート） | フィールドテスト（50ルート） |

---

## 12. 実装フェーズ

### フェーズ1: ローカライゼーション（スプリント1-2）
- ✅ NDTローカライゼーション統合（Autoware）
- ✅ 手動SLAMマッピング手順
- ✅ TFツリー設定
- ✅ ローカライゼーション監視

### フェーズ2: Nav2統合（スプリント3-4）
- ✅ Nav2スタック設定
- ✅ グローバル/ローカルコストマップセットアップ
- ✅ 経路計画（NavFn）
- ✅ ローカル制御（DWBまたはSwerve）

### フェーズ3: マルチウェイポイントナビゲーション（スプリント5-6）
- ✅ ルートマネージャーノード
- ✅ ウェイポイントデータベース
- ✅ Nav2ウェイポイントフォロワー統合
- ✅ 失敗時の再ルーティング

### フェーズ4: フィールド検証（スプリント7-8）
- ✅ キャンパスマッピング（500m〜1km）
- ✅ ローカライゼーション精度テスト
- ✅ マルチウェイポイントルート実行
- ✅ リカバリービヘイビアチューニング

---

**文書ステータス:** ドラフト
**実装ステータス:** 未開始
**必要な承認:** ナビゲーションリード、ローカライゼーションエンジニア、ルート計画エンジニア、システムアーキテクト
