# 実装ガイド - フェーズごとの詳細

**ドキュメントバージョン:** 1.0
**最終更新:** 2025-12-02
**総作業時間:** 616時間 (開発者2名で16週間)

---

## 📋 目次

1. [概要](#概要)
2. [フェーズ1: クリティカルバグと安全性 (第1-4週)](#フェーズ1-クリティカルバグと安全性)
3. [フェーズ2: テストインフラストラクチャ (第5-8週)](#フェーズ2-テストインフラストラクチャ)
4. [フェーズ3: ROS 2ベストプラクティス (第9-12週)](#フェーズ3-ros-2ベストプラクティス)
5. [フェーズ4: デプロイメントと運用 (第13-16週)](#フェーズ4-デプロイメントと運用)
6. [週次チェックリスト](#週次チェックリスト)
7. [進捗トラッキング](#進捗トラッキング)

---

## 概要

### タイムライン概要

```
第1-4週:   フェーズ1  (クリティカルバグと安全性)        → 安全なシステム
第5-8週:   フェーズ2  (テストインフラストラクチャ)      → テスト済みシステム
第9-12週:  フェーズ3  (ROS 2ベストプラクティス)        → 堅牢なシステム
第13-16週: フェーズ4  (デプロイメントと運用)            → デプロイ可能なシステム
```

### 作業時間配分

| フェーズ | 時間 | 割合 | 優先度 |
|-------|-------|------------|----------|
| フェーズ1 | 144 | 23% | 🔴 クリティカル |
| フェーズ2 | 96 | 16% | 🔴 クリティカル |
| フェーズ3 | 104 | 17% | 🟡 高 |
| フェーズ4 | 104 | 17% | 🟡 高 |
| **フェーズ5-6** (将来) | 168 | 27% | 🟢 オプション |
| **合計** | **616** | **100%** | |

### 最小実行可能タイムライン

**より早くデプロイしたい場合:**

- **監視付きテストのみ:** フェーズ1 (4週間) - 安全だが検証されていない
- **本番デプロイメント:** フェーズ1-2 (8週間) - 安全 + テスト済み
- **完全本番:** フェーズ1-4 (16週間) - 安全 + テスト済み + 洗練

---

## フェーズ1: クリティカルバグと安全性

**期間:** 4週間
**作業時間:** 144時間
**優先度:** 🔴 クリティカル - これなしではデプロイ不可
**チーム:** 開発者2名

### 目標

✅ すべてのクリティカルコードバグを修正
✅ 安全性スーパーバイザーレイヤーを実装
✅ nav_masterにステートマシンを追加
✅ 緊急停止を実装
✅ ドッキング中のLiDAR監視を追加
✅ 基本的なジオフェンシング

**成果物:** テスト準備が整った安全なシステム

---

### 第1週: クリティカルバグ修正

**作業時間:** 16時間

#### タスク 1.1: PID積分バグを修正 (4時間)

**問題:** CRIT-01 - PID積分が累積されない
**ファイル:** `src/nav_docking/nav_docking.cpp`
**優先度:** 🔴 クリティカル

**現在の実装 (誤り):**
```cpp
double integral = error * callback_duration;  // 上書きしてしまう!
```

**修正:**
```cpp
// クラス宣言内:
double integral_dist_ = 0.0;
double integral_y_ = 0.0;
double integral_yaw_ = 0.0;

// 制御ループ内:
integral_dist_ += error * callback_duration;  // 累積する!
double vel_x = Kp * error + Ki * integral_dist_ + Kd * derivative;
```

**手順:**
1. `nav_docking.cpp`を開く
2. 積分項のメンバー変数を追加
3. コンストラクタで0.0に初期化
4. `integral = ...`を`integral += ...`に変更
5. 3軸すべて(X, Y, Yaw)に適用
6. シングルマーカーとデュアルマーカー両方の関数に適用
7. ビルドとテスト

**検証:**
- ドッキングがよりスムーズになる
- ロボットが正確なターゲットに到達する(定常偏差なし)
- 10回のドッキング試行でテスト

**参照:** [ISSUES-AND-FIXES.md](./ISSUES-AND-FIXES.md#crit-01-pid-integral-bug)

---

#### タスク 1.2: デュアルマーカー距離計算を修正 (1時間)

**問題:** CRIT-02 - 誤った平均計算
**ファイル:** `src/nav_docking/nav_docking.cpp`
**優先度:** 🔴 クリティカル

**現在の実装 (誤り):**
```cpp
double distance = (left_marker_x) + (right_marker_x) / 2;  // = left + right/2
```

**修正:**
```cpp
double distance = (left_marker_x + right_marker_x) / 2;  // = (left + right)/2
```

**手順:**
1. `left_marker_x) + (right_marker_x) / 2`を検索
2. 合計の周りに括弧を追加
3. 同様の計算すべて(X, Y, Yaw)に適用
4. ビルドとテスト

**検証:**
- ロボットがマーカー間の中央に正しく配置される
- 最終位置を測定: 中央から±1mm以内であるべき

---

#### タスク 1.3: 変数を初期化 (1時間)

**問題:** HIGH-01 - 未初期化変数
**ファイル:** `src/nav_docking/nav_docking.cpp`
**優先度:** 🟡 高

**修正:** コンストラクタ初期化リストに追加:
```cpp
NavDockingNode::NavDockingNode() : Node("nav_docking_node"),
    first_confirmation_received_(false),
    second_confirmation_received_(false),
    integral_dist_(0.0),
    integral_y_(0.0),
    integral_yaw_(0.0),
    prev_error_dist_(0.0),
    prev_error_y_(0.0),
    prev_error_yaw_(0.0)
{
    // ... コンストラクタの残り
}
```

---

#### タスク 1.4: すべての修正をテスト (10時間)

**テストプロトコル:**
1. システムをビルド: `colcon build`
2. シミュレーションを実行: `ros2 launch boot simulation.launch.py`
3. 20回のドッキング試行をテスト
4. 測定:
   - 成功率(100%であるべき)
   - 最終位置精度(±1-2mm以内であるべき)
   - ドッキング時間(25-30秒であるべき)
5. 結果を記録

---

### 第2週: 安全性アーキテクチャ設計

**作業時間:** 40時間

#### タスク 2.1: 安全性スーパーバイザーを設計 (8時間)

**成果物:** 安全性アーキテクチャドキュメント

**定義すべきトピック:**
1. 安全性状態(SAFE, CAUTION, UNSAFE, EMERGENCY)
2. 安全性違反(障害物が近すぎる、崖、マーカー喪失など)
3. オーバーライド機構(すべての動作を停止する方法)
4. 統合ポイント(どのノードが安全性シグナルをサブスクライブするか)

**出力:** 以下を含む設計ドキュメント:
- 状態図
- 安全性違反リスト
- トピック/サービスインターフェース
- 統合計画

**参照:** [SYSTEM-ARCHITECTURE.md](./SYSTEM-ARCHITECTURE.md#safety-supervisor-layer)

---

#### タスク 2.2: 緊急停止を実装 (12時間)

**問題:** CRIT-05 - ソフトウェア緊急停止がない
**優先度:** 🔴 クリティカル

**実装:**

**1. 緊急停止トピックを作成:**
```bash
トピック: /multigo/safety/emergency_stop
タイプ: std_msgs/Bool
QoS: RELIABLE, TRANSIENT_LOCAL, KEEP_ALL
```

**2. すべてのモーションノードを更新** (nav_docking, nav_control, mecanum_wheels):
```cpp
// e-stopをサブスクライブ
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
bool emergency_stop_active_ = false;

void estopCallback(std_msgs::msg::Bool::SharedPtr msg) {
    emergency_stop_active_ = msg->data;
    if (emergency_stop_active_) {
        RCLCPP_ERROR(get_logger(), "緊急停止!");
        publishZeroVelocity();
        cancelAllActions();
    }
}

void publishVelocity(Twist cmd) {
    if (emergency_stop_active_) return;  // e-stop中は動かない
    cmd_vel_pub_->publish(cmd);
}
```

**3. テスト:**
```bash
# e-stopをトリガー
ros2 topic pub /multigo/safety/emergency_stop std_msgs/Bool "data: true"

# 検証: ロボットが即座に停止
# e-stopを解除
ros2 topic pub /multigo/safety/emergency_stop std_msgs/Bool "data: false"
```

---

#### タスク 2.3: ステートマシンを実装 (24時間)

**問題:** 明示的な状態管理がない
**優先度:** 🔴 高

**実装:**

**1. 状態を定義:**
```cpp
enum class MissionState {
    IDLE,
    WAITING_APPROACH_CONFIRMATION,
    NAVIGATING_TO_GOAL,
    WAITING_DOCK_CONFIRMATION,
    DOCKING,
    DOCKED,
    ERROR
};
```

**2. nav_masterにステートマシンを実装:**
```cpp
class MissionStateMachine : public rclcpp::Node {
private:
    MissionState current_state_ = MissionState::IDLE;

public:
    void update() {
        switch (current_state_) {
            case MissionState::IDLE:
                if (approach_requested_) {
                    transitionTo(MissionState::WAITING_APPROACH_CONFIRMATION);
                }
                break;

            case MissionState::WAITING_APPROACH_CONFIRMATION:
                if (user_confirmed_) {
                    transitionTo(MissionState::NAVIGATING_TO_GOAL);
                    sendApproachGoal();
                }
                break;

            // ... さらなる状態
        }
    }

    void transitionTo(MissionState new_state) {
        RCLCPP_INFO(get_logger(), "状態: %s -> %s",
                    stateToString(current_state_),
                    stateToString(new_state));

        current_state_ = new_state;
        publishState(new_state);  // 監視用
    }
};
```

**3. 監視用に状態をパブリッシュ:**
```bash
トピック: /multigo/mission/state
タイプ: std_msgs/String
```

**4. すべての状態遷移をテスト**

---

### 第3週: 安全性機能

**作業時間:** 44時間

#### タスク 3.1: 安全性スーパーバイザーノードを実装 (24時間)

**新しいパッケージを作成:** `safety_supervisor`

**実装:**
```cpp
class SafetySupervisor : public rclcpp::Node {
public:
    enum class SafetyState { SAFE, CAUTION, UNSAFE, EMERGENCY_STOP };

private:
    SafetyState current_state_ = SafetyState::SAFE;

    // モニター
    rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<Bool>::SharedPtr estop_sub_;
    rclcpp::Subscription<PoseArray>::SharedPtr markers_sub_;

    // エンフォーサー
    rclcpp::Publisher<Bool>::SharedPtr safety_stop_pub_;
    rclcpp::Publisher<Float32>::SharedPtr speed_limit_pub_;

public:
    void scanCallback(LaserScan::SharedPtr msg) {
        double min_dist = *std::min_element(msg->ranges.begin(), msg->ranges.end());

        if (min_dist < 0.15) {  // 15cm = 危険
            transitionTo(SafetyState::EMERGENCY_STOP);
        } else if (min_dist < 0.30) {  // 30cm = 注意
            transitionTo(SafetyState::CAUTION);
        }
    }

    void transitionTo(SafetyState new_state) {
        if (new_state == current_state_) return;

        current_state_ = new_state;

        switch (new_state) {
            case SafetyState::SAFE:
                publishSpeedLimit(1.0);  // 100%
                publishSafetyStop(false);
                break;
            case SafetyState::CAUTION:
                publishSpeedLimit(0.5);  // 50%
                RCLCPP_WARN(get_logger(), "注意: 速度を低減");
                break;
            case SafetyState::EMERGENCY_STOP:
                publishSpeedLimit(0.0);
                publishSafetyStop(true);
                RCLCPP_ERROR(get_logger(), "緊急停止!");
                break;
        }
    }
};
```

---

#### タスク 3.2: ドッキング中にLiDARを追加 (20時間)

**問題:** CRIT-03 - ビジョンのみのドッキング(障害物に対して盲目)
**優先度:** 🔴 クリティカル

**nav_dockingでの実装:**

```cpp
// LiDARをサブスクライブ
rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
bool obstacle_in_safety_zone_ = false;

void scanCallback(LaserScan::SharedPtr msg) {
    // ドッキングゾーン(30cm)の障害物をチェック
    for (auto range : msg->ranges) {
        if (range < 0.30) {
            obstacle_in_safety_zone_ = true;
            RCLCPP_WARN(get_logger(), "ドッキング中に障害物を検出!");
            return;
        }
    }
    obstacle_in_safety_zone_ = false;
}

void dualMarkerCmdVelPublisher() {
    // 速度をパブリッシュする前の安全性チェック
    if (obstacle_in_safety_zone_) {
        RCLCPP_WARN(get_logger(), "経路に障害物、ドッキング一時停止");
        publishZeroVelocity();
        return;
    }

    // 通常のドッキング制御
    // ...
}
```

**テスト:** ドッキング中にロボットと車椅子の間に障害物を配置 → 停止するべき

---

### 第4週: ジオフェンシングとテスト

**作業時間:** 44時間

#### タスク 4.1: 基本的なジオフェンシング (16時間)

**問題:** CRIT-08 - 仮想境界がない
**優先度:** 🔴 クリティカル(病院用)

**実装:**

**1. 設定ファイルを作成** `config/safety_zones.yaml`:
```yaml
keep_out_zones:
  - name: "test_zone_1"
    polygon: [[0, 0], [1, 0], [1, 1], [0, 1]]
    reason: "テスト立入禁止ゾーン"
```

**2. ゾーンチェッカーを実装:**
```cpp
class SafetyZoneManager : public rclcpp::Node {
private:
    std::vector<Polygon> keep_out_zones_;

public:
    void loadZones(std::string yaml_file) {
        // YAMLをパースしてポリゴンをロード
    }

    bool isPositionSafe(double x, double y) {
        for (auto& zone : keep_out_zones_) {
            if (pointInPolygon(x, y, zone)) {
                return false;  // 立入禁止ゾーン内
            }
        }
        return true;
    }
};
```

**3. 安全性スーパーバイザーと統合**

---

#### タスク 4.2: フェーズ1統合テスト (20時間)

**テストプロトコル:**

**1. バグ修正の検証 (6時間):**
- [ ] 20回のドッキング試行、100%成功
- [ ] 最終位置精度±1-2mm
- [ ] 振動やオーバーシュートなし

**2. 安全性機能の検証 (8時間):**
- [ ] 緊急停止が機能(100ms未満で即座に停止)
- [ ] ステートマシンが正しく遷移
- [ ] 障害物検出時にLiDARがドッキングを停止
- [ ] 立入禁止ゾーンがナビゲーションを防ぐ

**3. 統合テスト (6時間):**
- [ ] 完全なワークフロー: アプローチ → ドッキング → アンドック
- [ ] エラー復旧(マーカー喪失 → 復旧)
- [ ] 複数のドッキングサイクル(10回)

---

#### タスク 4.3: ドキュメント更新 (8時間)

ドキュメントを更新:
- [ ] フェーズ1のすべての変更を記録
- [ ] アーキテクチャ図を更新
- [ ] オペレーター安全手順を記述
- [ ] フェーズ1完了レポートを作成

---

### フェーズ1チェックリスト

**フェーズ1を完了とマークする前に:**

- [ ] 3つのクリティカルバグすべてを修正(CRIT-01, CRIT-02, HIGH-01)
- [ ] 安全性スーパーバイザーノードが動作
- [ ] 緊急停止がテスト済みで動作
- [ ] ステートマシンがnav_masterに統合
- [ ] ドッキング中のLiDAR監視が動作
- [ ] 基本的なジオフェンシングが実装・テスト済み
- [ ] 20回以上の成功したドッキング試行
- [ ] 最終位置精度±2mm以下
- [ ] ドキュメント更新済み
- [ ] フェーズ1レポート作成済み

**成果物:** フェーズ2(テスト)の準備が整った安全なシステム

---

## フェーズ2: テストインフラストラクチャ

**期間:** 4週間 (第5-8週)
**作業時間:** 96時間
**優先度:** 🔴 クリティカル
**チーム:** 開発者2名

### 目標

✅ 80%の自動テストカバレッジ
✅ CI/CDパイプラインが動作
✅ 40-50のユニットテストを記述
✅ 10-15の統合テストを記述
✅ リグレッション保護

**成果物:** テスト・検証済みシステム

---

### 第5-6週: ユニットテスト

**作業時間:** 60時間

#### タスク 5.1: テストフレームワークのセットアップ (12時間)

**1. テスト依存関係をインストール:**
```bash
sudo apt install ros-humble-ament-cmake-gtest ros-humble-ament-cmake-pytest
pip3 install pytest pytest-cov
```

**2. テスト構造を作成:**
```
multigo_navigation/
├── src/
│   └── nav_docking/
│       ├── src/
│       │   └── nav_docking.cpp
│       └── test/
│           ├── test_pid_control.cpp
│           ├── test_marker_processing.cpp
│           └── test_state_machine.cpp
```

**3. CMakeLists.txtに追加:**
```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_pid_control test/test_pid_control.cpp)
  target_link_libraries(test_pid_control ${PROJECT_NAME})
endif()
```

---

#### タスク 5.2: ユニットテストを記述 (48時間)

**記述すべきクリティカルテスト:**

**1. PID制御テスト (8時間):**
```cpp
TEST(NavDockingTest, PIDIntegralAccumulates) {
    double integral = 0.0;
    double error = 1.0;
    double dt = 0.1;

    // 5回の反復をシミュレート
    for (int i = 0; i < 5; i++) {
        integral += error * dt;
    }

    EXPECT_NEAR(integral, 0.5, 0.001);  // 0.1ではなく0.5であるべき
}

TEST(NavDockingTest, DualMarkerAveraging) {
    double left = 1.0, right = 0.8;
    double center = (left + right) / 2;
    EXPECT_NEAR(center, 0.9, 0.001);  // 1.4ではない!
}
```

**2. 座標変換テスト (8時間):**
```cpp
TEST(ArucoDetectTest, OpenCVToROSConversion) {
    cv::Vec3d opencv_tvec(1.0, 0.0, 2.0);
    auto ros_pose = convertOpenCVToROS(opencv_tvec);

    EXPECT_NEAR(ros_pose.position.x, 2.0, 0.001);  // Z->X
    EXPECT_NEAR(ros_pose.position.y, -1.0, 0.001); // X->-Y
}
```

**3. ステートマシンテスト (8時間):**
```cpp
TEST(StateMachineTest, StateTransitions) {
    StateMachine sm;
    EXPECT_EQ(sm.getState(), State::IDLE);

    sm.handleEvent(Event::APPROACH_REQUESTED);
    EXPECT_EQ(sm.getState(), State::WAITING_CONFIRMATION);

    sm.handleEvent(Event::USER_CONFIRMED);
    EXPECT_EQ(sm.getState(), State::NAVIGATING);
}
```

**4. 安全性テスト (8時間):**
```cpp
TEST(SafetySupervisorTest, EmergencyStopTriggered) {
    SafetySupervisor supervisor;

    // 非常に近い障害物をシミュレート(10cm)
    auto scan = createLaserScan(0.10);
    supervisor.scanCallback(scan);

    EXPECT_EQ(supervisor.getState(), SafetyState::EMERGENCY_STOP);
}
```

**目標: 合計40-50のユニットテスト**

---

### 第7週: 統合テスト

**作業時間:** 20時間

#### タスク 7.1: 統合テスト環境のセットアップ (8時間)

**1. テスト用のシミュレーション起動ファイルを作成:**
```python
# test_simulation.launch.py
def generate_launch_description():
    return LaunchDescription([
        # Gazeboを起動
        IncludeLaunchDescription('simulation.launch.py'),

        # テストノードを起動
        Node(package='nav_docking', executable='nav_docking_node'),

        # 5秒後にテストを実行
        TimerAction(period=5.0, actions=[
            ExecuteProcess(cmd=['pytest', 'test/integration/'])
        ])
    ])
```

---

#### タスク 7.2: 統合テストを記述 (12時間)

**クリティカル統合テスト:**

**1. アプローチワークフロー (4時間):**
```python
def test_approach_success():
    # セットアップ: 原点にロボット、(5, 0)にマーカー
    publish_marker_pose(x=5.0, y=0.0, yaw=math.pi)

    # 実行: アプローチゴールを送信
    result = send_approach_goal()

    # 検証: ロボットがゴールに到達
    assert result.success == True
    final_pose = get_robot_pose()
    assert abs(final_pose.x - 4.695) < 0.05  # 5cm以内
```

**2. ドッキングワークフロー (4時間):**
```python
def test_docking_precision():
    # セットアップ: アプローチ位置にロボットを配置
    position_robot_at_approach()

    # 実行: ドックゴールを送信
    result = send_dock_goal()

    # 検証: 高精度
    assert result.success == True
    assert result.final_distance_error_x < 0.002  # <2mm
    assert result.final_distance_error_y < 0.002
```

**3. 安全性統合 (4時間):**
```python
def test_obstacle_stops_docking():
    # セットアップ: ドッキング開始
    start_docking()

    # シミュレート: 障害物が出現
    spawn_obstacle_in_path()

    # 検証: ロボットが停止
    time.sleep(0.5)
    velocity = get_robot_velocity()
    assert velocity.linear.x < 0.01  # ほぼ停止
```

**目標: 10-15の統合テスト**

---

### 第8週: CI/CDパイプライン

**作業時間:** 16時間

#### タスク 8.1: GitHub Actionsのセットアップ (8時間)

**`.github/workflows/ci.yml`を作成:**
```yaml
name: CI Pipeline

on: [push, pull_request]

jobs:
  build-and-test:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v2

      - name: Setup ROS 2
        uses: ros-tooling/setup-ros@v0.6
        with:
          required-ros-distributions: humble

      - name: Build
        run: |
          source /opt/ros/humble/setup.bash
          colcon build --symlink-install

      - name: Unit Tests
        run: |
          source install/setup.bash
          colcon test --packages-select nav_docking nav_goal
          colcon test-result --verbose

      - name: Coverage
        run: |
          pip install coverage
          coverage report --fail-under=80

      - name: Upload Coverage
        uses: codecov/codecov-action@v2
```

---

#### タスク 8.2: 品質ゲートの設定 (4時間)

**1. カバレッジ要件:**
- 最低80%のライン カバレッジ
- 最低70%のブランチ カバレッジ

**2. マージ前にCIが合格する必要:**
- すべてのテストが合格
- カバレッジ閾値を満たす
- ビルド成功

---

#### タスク 8.3: ドキュメント (4時間)

**テストガイドを記述:**
- ローカルでテストを実行する方法
- 新しいテストを記述する方法
- CI/CDパイプラインの説明
- カバレッジ要件

---

### フェーズ2チェックリスト

**フェーズ2を完了とマークする前に:**

- [ ] 40以上のユニットテストを記述・合格
- [ ] 10以上の統合テストを記述・合格
- [ ] 80%以上のテストカバレッジを達成
- [ ] CI/CDパイプラインが動作
- [ ] CIですべてのテストが合格
- [ ] カバレッジレポートが生成
- [ ] テストドキュメントが完成
- [ ] フェーズ2レポート作成済み

**成果物:** リグレッション保護を備えたテスト・検証済みシステム

---

## フェーズ3: ROS 2ベストプラクティス

**期間:** 4週間 (第9-12週)
**作業時間:** 104時間
**優先度:** 🟡 高
**チーム:** 開発者2名

### 目標

✅ クリティカルコンポーネントのライフサイクルノード
✅ 明示的なQoSポリシー
✅ 拡張されたアクション定義
✅ 標準化されたトピック命名
✅ コマンド調停

**成果物:** 本番品質のROS 2システム

---

### 第9-10週: ライフサイクルノード

**作業時間:** 48時間 (ノードあたり12時間 × 4ノード)

#### 変換するノード

1. **nav_docking** (12時間)
2. **nav_control** (12時間)
3. **mecanum_wheels** (12時間)
4. **aruco_detect** (12時間)

#### 実装パターン

```cpp
#include <rclcpp_lifecycle/lifecycle_node.hpp>

class NavDockingLifecycle : public rclcpp_lifecycle::LifecycleNode {
public:
    NavDockingLifecycle() : LifecycleNode("nav_docking_node") {}

    CallbackReturn on_configure(const State&) override {
        // パラメータをロードして検証
        declare_parameter<double>("Kp_dist", 0.5);
        Kp_dist_ = get_parameter("Kp_dist").as_double();

        if (!validateParameters()) {
            return CallbackReturn::FAILURE;
        }

        // パブリッシャー/サブスクライバーを作成(非アクティブ)
        cmd_vel_pub_ = create_publisher<Twist>("/cmd_vel_final", 10);

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const State&) override {
        // パブリッシャーをアクティブ化
        cmd_vel_pub_->on_activate();

        // タイマーを開始
        timer_ = create_wall_timer(100ms, [this]() { controlLoop(); });

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const State&) override {
        // コマンド送信を停止
        cmd_vel_pub_->on_deactivate();
        timer_->cancel();

        return CallbackReturn::SUCCESS;
    }
};
```

**参照:** [SYSTEM-ARCHITECTURE.md](./SYSTEM-ARCHITECTURE.md#lifecycle-nodes)

---

### 第11週: 通信の改善

**作業時間:** 36時間

#### タスク 11.1: QoSポリシー (8時間)

**各トピックに適切なQoSを実装:**

```cpp
// 安全性クリティカル(e-stop)
auto qos_critical = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
    .history(RMW_QOS_POLICY_HISTORY_KEEP_ALL);

// 制御コマンド
auto qos_control = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1);

// センサーデータ
auto qos_sensor = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
    .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1);
```

---

#### タスク 11.2: 拡張されたアクション (12時間)

**リッチフィードバックを持つアクションを再設計:**

```
# Enhanced Dock.action
geometry_msgs/PoseStamped target_pose
float32 tolerance
---
bool success
string error_code
string detailed_message
geometry_msgs/Pose final_pose
float32 final_distance_error_x
float32 final_distance_error_y
float32 duration_seconds
---
string current_phase
float32 distance_to_target
float32 centering_error
bool markers_visible
```

---

#### タスク 11.3: トピック命名 (16時間)

**すべてのトピックを標準化:**

変更前:
```
/cmd_vel
/cmd_vel_final
/aruco_detect/markers_left
```

変更後:
```
/multigo/navigation/cmd_vel
/multigo/docking/cmd_vel
/multigo/perception/markers/left
```

**すべてのノード + 起動ファイル + ドキュメントを更新**

---

### 第12週: コマンド調停

**作業時間:** 20時間

#### タスク 12.1: アービトレーターを実装 (16時間)

**command_arbitratorパッケージを作成:**

```cpp
class CommandArbitrator : public rclcpp::Node {
    enum class Priority {
        EMERGENCY = 0,
        SAFETY = 1,
        DOCKING = 2,
        NAVIGATION = 3
    };

    void selectCommand() {
        // 最高優先度のアクティブコマンドを見つける
        // モーターにパブリッシュ
        // 調停決定をログ
    }
};
```

**サブスクライブ:**
- `/multigo/safety/cmd_vel` (優先度 1)
- `/multigo/docking/cmd_vel` (優先度 2)
- `/multigo/navigation/cmd_vel` (優先度 3)

**パブリッシュ:**
- `/multigo/motion/cmd_vel` (最終コマンド)

---

### フェーズ3チェックリスト

- [ ] 4つのクリティカルノードがライフサイクルに変換済み
- [ ] すべてのトピックにQoSポリシーを適用
- [ ] 拡張されたアクション定義を実装
- [ ] すべてのトピックを標準規約に改名
- [ ] コマンドアービトレーターが動作
- [ ] すべてのテストが更新され合格
- [ ] ドキュメント更新済み
- [ ] フェーズ3レポート作成済み

**成果物:** 本番品質のROS 2システム

---

## フェーズ4: デプロイメントと運用

**期間:** 4週間 (第13-16週)
**作業時間:** 104時間
**優先度:** 🟡 高
**チーム:** 開発者2名

### 目標

✅ ティーチングモード + ウェイポイントシステム
✅ Dockerデプロイメント
✅ 設定管理
✅ 診断システム
✅ オペレータードキュメント

**成果物:** デプロイ可能、運用可能なシステム

---

### 第13-14週: ティーチングモード

**作業時間:** 40時間

#### タスク 13.1: ウェイポイントシステム (20時間)

**ウェイポイントマネージャーを作成:**

```cpp
class WaypointManager : public rclcpp::Node {
private:
    std::map<std::string, geometry_msgs::msg::Pose> waypoints_;

public:
    void loadWaypoints(std::string yaml_file);
    Pose getWaypoint(std::string id);
    void saveWaypoint(std::string id, Pose pose);
};
```

**ウェイポイント設定:** `config/waypoints.yaml`
```yaml
waypoints:
  - id: "room_205"
    name: "Room 205"
    pose:
      position: [15.3, 8.7, 0.0]
      orientation: [0, 0, 0, 1]
```

---

#### タスク 13.2: ティーチングモード (20時間)

**起動ファイルにモードパラメータを追加:**

```python
DeclareLaunchArgument('mode',
    default_value='autonomous',
    choices=['teaching', 'autonomous'])

if mode == 'teaching':
    # 手動制御を有効化
    # RTAB-Mapがマップを記録
    # ウェイポイントを保存可能
```

**ティーチングワークフロー:**
1. ティーチングモードで起動
2. 環境内でロボットを手動で運転
3. ウェイポイントをマーク: `ros2 service call /waypoint/save "id: 'room_205'"`
4. マップを保存: `ros2 service call /rtabmap/save_map`
5. 保存されたマップで自律モードに切り替え

---

### 第15週: Dockerデプロイメント

**作業時間:** 40時間

#### タスク 15.1: Dockerfileを作成 (24時間)

**docker-compose.yml:**
```yaml
version: '3.8'

services:
  hardware:
    image: multigo/hardware:${VERSION}
    privileged: true
    devices:
      - /dev/video0:/dev/video0
    command: ros2 launch boot boot.launch.py

  navigation:
    image: multigo/navigation:${VERSION}
    depends_on: [hardware]
    command: ros2 launch boot run.launch.py

  safety:
    image: multigo/safety:${VERSION}
    command: ros2 run safety_supervisor safety_supervisor_node
    restart: always
```

---

#### タスク 15.2: 設定管理 (16時間)

**階層的な設定:**

```
config/
├── robot/
│   └── multigo_001.yaml      # ロボット固有
├── environment/
│   └── hospital_floor2.yaml  # 環境固有
├── mission/
│   └── wheelchair_dock.yaml  # ミッションパラメータ
└── defaults/
    └── navigation.yaml       # システムデフォルト
```

**設定ローダーは順番にマージ:**
1. システムデフォルト
2. 環境設定
3. ミッション設定
4. ロボット固有設定

---

### 第16週: 診断とドキュメント

**作業時間:** 24時間

#### タスク 16.1: 診断システム (16時間)

**diagnostic_updaterを使用して実装:**

```cpp
#include <diagnostic_updater/diagnostic_updater.hpp>

class NavDockingNode {
private:
    diagnostic_updater::Updater diagnostics_;

public:
    void setupDiagnostics() {
        diagnostics_.setHardwareID("multigo_001");
        diagnostics_.add("Camera Status", this, &NavDockingNode::checkCamera);
        diagnostics_.add("Marker Detection", this, &NavDockingNode::checkMarkers);
    }

    void checkCamera(diagnostic_updater::DiagnosticStatusWrapper& stat) {
        if ((now() - last_image_time_).seconds() > 1.0) {
            stat.summary(DiagnosticStatus::ERROR, "カメラ画像なし");
        } else {
            stat.summary(DiagnosticStatus::OK, "カメラ正常");
        }
    }
};
```

**監視:**
- カメラフレームレート
- LiDARデータレート
- マーカー検出率
- モーターステータス
- バッテリーレベル
- ローカライゼーション品質

---

#### タスク 16.2: オペレータードキュメント (8時間)

**オペレーターマニュアルを記述:**
- 日々の起動手順
- ティーチングモードの使用法
- トラブルシューティングガイド
- 安全手順
- 緊急時対応

---

### フェーズ4チェックリスト

- [ ] ティーチングモードが動作
- [ ] ウェイポイントシステムが動作
- [ ] Dockerデプロイメントがテスト済み
- [ ] 設定システムが実装済み
- [ ] 診断がパブリッシュ中
- [ ] オペレーターマニュアルが完成
- [ ] フェーズ4レポート作成済み
- [ ] システムがデプロイ準備完了

**成果物:** 本番環境準備完了のデプロイ可能なシステム

---

## 週次チェックリスト

### テンプレート: 週次レビュー

**毎週金曜日午後4時:**

1. **進捗レビュー** (15分)
   - 今週何が完了したか?
   - 何がブロックされているか?
   - 何が繰り越されるか?

2. **来週の計画** (15分)
   - 来週のタスクを割り当て
   - 依存関係を特定
   - 優先順位を設定

3. **課題トラッキング** (10分)
   - 発見された新しい課題は?
   - 優先順位に変更は?
   - GitHubプロジェクトボードを更新

4. **ドキュメント** (10分)
   - このガイドで進捗を更新
   - 逸脱を記録
   - 必要に応じて見積もりを更新

---

## 進捗トラッキング

### GitHubプロジェクトボード構造

**カラム:**
```
To Do → In Progress → In Review → Done
```

**ラベル:**
- `phase-1` / `phase-2` / `phase-3` / `phase-4`
- `priority-critical` / `priority-high` / `priority-medium`
- `bug` / `feature` / `documentation`
- `blocked`

### バーンダウントラッキング

**フェーズごとの残り時間を追跡:**

| 週 | フェーズ | 計画時間 | 実績時間 | 残り |
|------|-------|---------------|--------------|-----------|
| 1 | フェーズ1 | 16 | | 128 |
| 2 | フェーズ1 | 40 | | 88 |
| ... | | | | |

### 課題トラッキング

**すべてのタスクを課題にリンク:**
- 各タスクのGitHub課題を作成
- フェーズと優先度でラベル付け
- 進捗に応じてステータスを更新
- 完了時にクローズ

---

## リスク管理

### 一般的なリスク

**リスク1: タスクが見積もりより長くかかる**
- **軽減策:** 見積もりに20%のバッファ
- **対応:** 毎週タイムラインを調整、厳格に優先順位付け

**リスク2: 新しいクリティカルバグが発見される**
- **軽減策:** フェーズ2で包括的なテスト
- **対応:** バックログに追加、再優先順位付け、タイムライン延長の可能性

**リスク3: チームメンバーが不在**
- **軽減策:** 知識共有、ドキュメント化
- **対応:** 割り当てを調整、必要に応じてタイムライン延長

**リスク4: ハードウェアの問題**
- **軽減策:** ほとんどの開発でシミュレーションを使用
- **対応:** ハードウェアの問題を即座にエスカレーション

---

## 成功基準

### フェーズ1の成功
- ✅ すべてのクリティカルバグを修正
- ✅ 安全性スーパーバイザーが動作
- ✅ 20回以上の成功したドッキング試行
- ✅ 位置精度±2mm

### フェーズ2の成功
- ✅ 80%以上のテストカバレッジ
- ✅ CI/CDパイプラインが合格
- ✅ テスト失敗ゼロ

### フェーズ3の成功
- ✅ すべてのクリティカルノードがライフサイクルを使用
- ✅ QoSポリシーが適用済み
- ✅ コマンド調停が動作

### フェーズ4の成功
- ✅ ティーチングモードが機能
- ✅ Dockerデプロイメントが成功
- ✅ 診断がレポート中

### 全体の成功
- ✅ 全4フェーズ完了
- ✅ 95%以上のドッキング成功率
- ✅ テスト環境でシステムがデプロイ済み
- ✅ オペレーターがトレーニング済み
- ✅ ドキュメント完成

---

## 次のステップ

**今すぐ:**
1. チームでこのガイド全体をレビュー
2. GitHubプロジェクトボードを作成
3. フェーズ1タスクの課題を作成
4. フェーズ1第1週のタスクを割り当て
5. 週次レビュー会議をスケジュール

**今週:**
- バグ修正を開始(CRIT-01, CRIT-02, HIGH-01)
- 安全性アーキテクチャを設計
- 開発環境をセットアップ

**今月:**
- フェーズ1を完了
- 「安全なシステム」マイルストーンを達成
- フェーズ2の準備

---

**最終更新:** 2025-12-02
**ドキュメントバージョン:** 1.0
**質問?** [START-HERE.md](./START-HERE.md)を参照するか、チームミーティングで質問してください
