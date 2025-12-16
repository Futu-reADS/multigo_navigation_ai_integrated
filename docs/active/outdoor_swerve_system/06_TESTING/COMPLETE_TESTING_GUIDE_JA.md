# 完全テストガイド

**ドキュメントID:** TEST-GUIDE-COMPLETE-001
**バージョン:** 1.0
**日付:** 2025-12-15
**ステータス:** 包括的テストリファレンス

---

## フェーズ6ドキュメント完成

本ドキュメントは、屋外車椅子搬送ロボットシステムの完全なテスト戦略を提供します。ユニットテスト、統合テスト、フィールドテスト計画から、受け入れ基準、テスト自動化までをカバーしています。

---

## 目次

1. [ユニットテスト計画](#1-ユニットテスト計画)
2. [統合テスト計画](#2-統合テスト計画)
3. [フィールドテスト計画](#3-フィールドテスト計画)
4. [受け入れ基準](#4-受け入れ基準)
5. [テスト自動化](#5-テスト自動化)

---

## 1. ユニットテスト計画

### 1.1 テストフレームワーク

**C++ (gtest/gmock):**
```cpp
// test/test_swerve_module.cpp
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "swerve_drive_controller/swerve_module.hpp"

using namespace swerve_drive;

class SwerveModuleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    module_ = std::make_unique<SwerveModule>(0);  // FL module
  }

  void TearDown() override
  {
    module_.reset();
  }

  std::unique_ptr<SwerveModule> module_;
};

TEST_F(SwerveModuleTest, InitialStateIsZero)
{
  auto state = module_->getModuleState();
  EXPECT_DOUBLE_EQ(state.drive_velocity, 0.0);
  EXPECT_DOUBLE_EQ(state.steering_angle, 0.0);
}

TEST_F(SwerveModuleTest, SetTargetUpdatesState)
{
  module_->setTarget(1.0, M_PI / 4);
  auto state = module_->getModuleState();

  EXPECT_DOUBLE_EQ(state.target_velocity, 1.0);
  EXPECT_DOUBLE_EQ(state.target_angle, M_PI / 4);
}

TEST_F(SwerveModuleTest, FlipOptimizationReducesRotation)
{
  module_->updateState(0.0, 0.0);  // 現在角度: 0°
  module_->setTarget(1.0, 170.0 * M_PI / 180.0);  // 目標: 170°
  module_->optimizeSteeringAngle();

  auto state = module_->getModuleState();

  // -10°に反転され、駆動が逆転するはず
  EXPECT_NEAR(state.steering_angle, -10.0 * M_PI / 180.0, 0.01);
  EXPECT_DOUBLE_EQ(state.drive_direction, -1.0);
}
```

**Python (pytest):**
```python
# test/test_perception.py
import pytest
import numpy as np
from perception_pipeline.ground_removal import GroundRemovalNode

@pytest.fixture
def ground_removal_node():
    """グラウンド除去ノードのフィクスチャ"""
    node = GroundRemovalNode()
    yield node
    node.destroy_node()

def test_ransac_ground_detection(ground_removal_node):
    """RANSACによる地面検出をテスト"""
    # 合成点群を生成（地面 + 障害物）
    ground_points = np.random.uniform(-5, 5, (1000, 3))
    ground_points[:, 2] = 0.0  # Z=0（地面）

    obstacle_points = np.random.uniform(-2, 2, (100, 3))
    obstacle_points[:, 2] = np.random.uniform(0.5, 2.0, 100)  # Z=0.5-2.0

    all_points = np.vstack([ground_points, obstacle_points])

    # 地面除去を実行
    obstacles, ground = ground_removal_node.remove_ground(all_points)

    # 検証: ほぼすべての地面点が除去されること
    assert len(ground) > 900
    assert len(obstacles) < 200

def test_height_filter(ground_removal_node):
    """高さフィルターをテスト"""
    points = np.array([
        [0, 0, -0.5],  # 地面以下（除外）
        [0, 0, 0.5],   # 有効な障害物
        [0, 0, 1.5],   # 有効な障害物
        [0, 0, 2.5],   # 高すぎる（除外）
    ])

    filtered = ground_removal_node.apply_height_filter(points, 0.1, 2.0)

    assert len(filtered) == 2
    assert np.all((filtered[:, 2] >= 0.1) & (filtered[:, 2] <= 2.0))
```

---

### 1.2 カバレッジ目標

**目標:**
- **行カバレッジ:** 80%以上
- **ブランチカバレッジ:** 70%以上
- **重要コンポーネント:** 90%以上（安全、ドッキング、運動学）

**カバレッジ測定:**
```bash
# C++（gcov/lcov使用）
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="--coverage"
colcon test
lcov --capture --directory build --output-file coverage.info
lcov --remove coverage.info '/usr/*' --output-file coverage_filtered.info
genhtml coverage_filtered.info --output-directory coverage_html

# Python（pytest-cov使用）
pytest --cov=perception_pipeline --cov-report=html
```

---

### 1.3 テストすべきコンポーネント

**スワーブドライブ:**
- ✅ 逆運動学（全移動方向）
- ✅ 順運動学（オドメトリー精度）
- ✅ モジュール最適化（フリップ最適化）
- ✅ 速度制限（最大速度チェック）
- ✅ デッドバンド処理

**ドッキング:**
- ✅ ArUco検出（様々な照明条件）
- ✅ 姿勢推定精度
- ✅ PBVSコントローラー（収束）
- ✅ 状態マシン遷移
- ✅ タイムアウト処理

**ナビゲーション:**
- ✅ NDT自己位置推定（マップマッチング）
- ✅ グローバルプランナー（パス生成）
- ✅ ローカルプランナー（障害物回避）
- ✅ ルート管理（マルチウェイポイント）
- ✅ リカバリー動作

**認識:**
- ✅ 地面除去（RANSAC）
- ✅ 高さフィルター
- ✅ ボクセルダウンサンプリング
- ✅ コストマップ統合
- ✅ 衝突予測

**安全性:**
- ✅ 非常停止検出（<100ms）
- ✅ ウォッチドッグタイマー
- ✅ ヘルスチェック（すべてのノード）
- ✅ 衝突検出
- ✅ 速度制限ロジック

---

## 2. 統合テスト計画

### 2.1 ROS 2起動テスト

**起動ファイルテスト:**
```python
# test/test_system_launch.py
import unittest
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
import launch_testing.markers

@launch_testing.markers.keep_alive
def generate_test_description():
    return LaunchDescription([
        Node(
            package='swerve_drive_controller',
            executable='swerve_drive_controller_node',
            name='swerve_controller'
        ),
        Node(
            package='safety_monitor',
            executable='safety_monitor_node',
            name='safety_monitor'
        ),
        ReadyToTest()
    ])

class TestSystemLaunch(unittest.TestCase):
    def test_nodes_start(self, proc_info):
        """すべてのノードが正常に起動することを検証"""
        proc_info.assertWaitForShutdown(process=None, timeout=5)

    def test_topics_published(self):
        """必須トピックが公開されることを検証"""
        # トピック存在確認のロジック
        pass
```

---

### 2.2 Gazeboシミュレーション

**シミュレーション環境:**
```bash
# Gazeboシミュレーション起動
ros2 launch wheelchair_robot_gazebo simulation.launch.py
```

**シミュレーションテストシナリオ:**

1. **基本移動:**
   - 前進、後退、左右移動
   - 回転（その場）
   - 対角線移動

2. **障害物回避:**
   - 静的障害物
   - 動的障害物（移動する人）
   - 狭い通路通過

3. **ナビゲーション:**
   - 単一ゴールへのナビゲーション
   - マルチウェイポイントルート
   - リプランニング（ブロックされたパス）

4. **ドッキング:**
   - 正面からのアプローチ
   - オフセットからのアプローチ
   - 照明変化下での検出

---

### 2.3 サブシステム統合

**テストペア:**
- **ナビゲーション ↔ スワーブドライブ:** cmd_vel実行精度
- **認識 ↔ ナビゲーション:** 障害物検出 → 回避
- **ドッキング ↔ スワーブドライブ:** 視覚サーボイング制御
- **安全 ↔ すべて:** 非常停止伝播
- **UI ↔ ミッション管理:** コマンド実行

**統合テスト例:**
```cpp
// test/test_nav_swerve_integration.cpp
TEST_F(NavigationIntegrationTest, CmdVelFollowing)
{
  // cmd_velメッセージを公開
  auto cmd_vel = geometry_msgs::msg::Twist();
  cmd_vel.linear.x = 1.0;
  cmd_vel.linear.y = 0.5;
  cmd_vel.angular.z = 0.2;

  cmd_vel_pub_->publish(cmd_vel);

  // スワーブコントローラーが正しく応答することを待つ
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  // ホイールコマンドを確認
  ASSERT_TRUE(wheel_commands_received_);
  EXPECT_NEAR(wheel_commands_->modules[0].drive_velocity, 1.2, 0.1);
}
```

---

## 3. フィールドテスト計画

### 3.1 テスト環境

**場所:**
- 屋外駐車場（平坦、障害物最小限）
- 屋外歩道（傾斜あり、縁石）
- キャンパス道路（複数の交差点）
- 建物入口（ドッキングテスト）

**天候条件:**
- ☀️ 晴天（直射日光、影のコントラスト）
- ☁️ 曇天（安定した照明）
- 🌧️ 小雨（防水テスト、滑りやすい表面）
- 🌙 夕暮れ（低照度）

---

### 3.2 テストシナリオ

**シナリオ1: 基本ナビゲーション（500m）**

**目的:** 長距離でのNDT自己位置推定精度を検証

**手順:**
1. ロボット起動、NDT初期化
2. 500m離れたゴールを設定
3. 自律ナビゲーション実行
4. 位置誤差を連続記録

**成功基準:**
- 累積位置誤差 < 10cm
- ゴール到達 < 5cm誤差
- パス追従誤差 < 15cm

---

**シナリオ2: 障害物回避**

**目的:** 動的障害物検出と回避を検証

**手順:**
1. ロボットをゴールに向けてナビゲート開始
2. テスター（人）が予期せずパスを横断
3. ロボットが停止または回避することを確認
4. 安全距離（>50cm）を維持

**成功基準:**
- 衝突ゼロ
- 停止距離 < 1m（障害物から）
- 回避後にパス再計画

---

**シナリオ3: 車椅子ドッキング（高精度）**

**目的:** 屋外でのドッキング精度を検証

**手順:**
1. ロボットを車椅子の2m以内に配置
2. ドッキングシーケンス開始
3. ArUco検出、視覚サーボイング実行
4. 最終位置誤差を測定

**成功基準:**
- 位置誤差 < ±5mm（屋外）
- 姿勢誤差 < ±2度
- ドッキング時間 < 30秒
- 成功率 > 95%（20回試行）

---

**シナリオ4: マルチウェイポイントミッション**

**目的:** 完全なエンドツーエンドミッションを検証

**手順:**
1. 4ストップのルート読み込み（合計1km）
2. ミッション開始
3. 各ストップで車椅子ピックアップ/ドロップオフをシミュレート
4. ミッション完了

**成功基準:**
- 全ウェイポイント到達
- ミッション時間 < 15分
- 手動介入ゼロ
- バッテリー残量 > 50%

---

**シナリオ5: 非常時安全**

**目的:** 非常停止とフェイルセーフを検証

**手順:**
1. ロボット移動中（1 m/s）
2. 物理的E-Stopボタンを押す
3. 停止時間と距離を測定
4. E-Stopリセット、操作再開

**成功基準:**
- 停止時間 < 100ms（ボタン押下から）
- 停止距離 < 20cm
- 再開成功（E-Stopリセット後）
- すべてのモーターが即座に無効化

---

### 3.3 データ収集

**記録データ:**
```bash
# ROSバッグ記録
ros2 bag record -a -o field_test_scenario_3

# 記録されるトピック:
# - /sensors/lidar/points
# - /sensors/imu
# - /sensors/camera/front/image_raw
# - /sensors/camera/rear/image_raw
# - /current_pose
# - /odom
# - /cmd_vel
# - /swerve/wheel_commands
# - /safety/status
# - /docking/status
```

**メトリクス:**
- 位置精度（真値とNDT比較）
- パス追従誤差
- ドッキング精度（手動測定）
- 障害物検出距離
- バッテリー消費
- CPU/メモリ使用率
- ネットワーク遅延（UI ↔ ロボット）

---

### 3.4 安全プロトコル

**フィールドテスト前:**
- [ ] 物理的E-Stopボタンテスト
- [ ] ソフトウェアE-Stop伝播テスト
- [ ] バッテリー残量 > 80%
- [ ] すべてのセンサー動作確認
- [ ] 安全ベスト着用（テスター）
- [ ] テストエリア立入禁止コーン設置

**テスト中:**
- [ ] 常に2名のオペレーター（1名=ロボット監視、1名=E-Stop保持）
- [ ] 最大速度制限（初期テスト時0.5 m/s）
- [ ] 通行人に対する明確な可視性（LED警告灯）
- [ ] 無線非常停止（予備）

**緊急手順:**
1. E-Stop押下
2. ロボット停止確認
3. 電源オフ（必要に応じて）
4. インシデントログ記録

---

## 4. 受け入れ基準

### 4.1 MVPリリース基準

**機能:**
- ✅ 500mナビゲーション（±10cm精度）
- ✅ 障害物検出・回避（>50cm安全距離）
- ✅ 車椅子ドッキング（±5mm精度、屋外）
- ✅ 非常停止（<100ms応答）
- ✅ マルチウェイポイントミッション（4ストップ）
- ✅ バッテリー動作（4時間、連続動作）
- ✅ タッチスクリーンUI（基本操作）
- ✅ eHMI（LED + 音声フィードバック）

**パフォーマンス:**
- NDT自己位置推定: 10 Hz
- LiDAR処理: 10 Hz
- 制御ループ: 20 Hz
- UI応答性: < 100ms
- CPU使用率: < 70%（連続）
- メモリ使用率: < 16GB

**信頼性:**
- ミッション成功率: > 90%（20試行）
- ドッキング成功率: > 95%（20試行）
- 平均故障間隔（MTBF）: > 8時間
- 手動介入率: < 10%（ミッションあたり）

---

### 4.2 本番リリース基準

**機能（MVPに追加）:**
- ✅ 1km+ ナビゲーション（±10cm精度維持）
- ✅ 屋内ドッキング（±2mm精度）
- ✅ 雨天動作（小雨、IP54）
- ✅ 多言語UI（EN/JP/ZH）
- ✅ フリート管理統合
- ✅ リモート監視
- ✅ 自動充電ドッキング

**パフォーマンス:**
- ミッション成功率: > 95%（100試行）
- ドッキング成功率: > 98%（100試行）
- MTBF: > 40時間
- 手動介入率: < 5%

**安全認証:**
- [ ] ISO 13482（パーソナルケアロボット）準拠
- [ ] リスク評価完了
- [ ] FMEA（故障モード影響解析）
- [ ] 現地規制認証

---

## 5. テスト自動化

### 5.1 CI/CDパイプライン

**GitHub Actionsワークフロー:**
```yaml
# .github/workflows/ros2_ci.yml
name: ROS 2 CI

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main, develop ]

jobs:
  build_and_test:
    runs-on: ubuntu-22.04
    container:
      image: ros:humble

    steps:
      - name: チェックアウト
        uses: actions/checkout@v3

      - name: rosdep依存関係インストール
        run: |
          apt update
          rosdep update
          rosdep install --from-paths src --ignore-src -r -y

      - name: ビルド
        run: |
          . /opt/ros/humble/setup.sh
          colcon build --symlink-install

      - name: テスト実行
        run: |
          . /opt/ros/humble/setup.sh
          . install/setup.sh
          colcon test --return-code-on-test-failure

      - name: テスト結果表示
        run: colcon test-result --verbose

      - name: カバレッジ生成
        run: |
          colcon build --cmake-args -DCMAKE_CXX_FLAGS="--coverage"
          colcon test
          lcov --capture --directory build --output-file coverage.info

      - name: カバレッジアップロード
        uses: codecov/codecov-action@v3
        with:
          files: ./coverage.info
```

---

### 5.2 自動テスト実行

**毎晩のテスト:**
```bash
# cron: 毎日午前2時に実行
0 2 * * * cd ~/wheelchair_robot_ws && \
  source install/setup.bash && \
  colcon test --return-code-on-test-failure && \
  python3 scripts/email_test_report.py
```

**リグレッションテストスイート:**
```bash
# すべてのユニットテスト実行
colcon test --packages-select-regex ".*_test"

# 統合テストのみ
colcon test --packages-select integration_tests

# フィールドテストプレイバック（記録データ使用）
ros2 bag play field_test_scenario_3.db3 &
ros2 launch wheelchair_robot_test regression_test.launch.py
```

---

### 5.3 継続的監視

**実行時メトリクス:**
```python
# scripts/metrics_collector.py
import rclpy
from rclpy.node import Node

class MetricsCollector(Node):
    """パフォーマンスメトリクスを収集してInfluxDBに送信"""

    def __init__(self):
        super().__init__('metrics_collector')

        # メトリクスをサブスクライブ
        self.create_subscription(
            SafetyStatus, '/safety/status', self.safety_callback, 10)
        self.create_subscription(
            RobotStatus, '/robot_status', self.robot_callback, 10)

    def safety_callback(self, msg):
        # 最小障害物距離を記録
        self.influx_client.write_point(
            'obstacle_distance', msg.min_obstacle_distance)

    def robot_callback(self, msg):
        # CPU/メモリ/バッテリーを記録
        self.influx_client.write_points([
            ('cpu_usage', msg.cpu_usage_percent),
            ('memory_usage', msg.memory_usage_percent),
            ('battery_voltage', msg.battery_voltage)
        ])
```

**アラート:**
- CPU使用率 > 80%（5分間）
- メモリ使用率 > 90%
- バッテリー電圧 < 42V
- 自己位置推定品質 < 0.5
- 障害物検出なし（10秒間）

---

## まとめ

完全なテスト戦略は以下を保証します:
1. **コード品質:** 80%+ カバレッジ、自動回帰検出
2. **統合検証:** すべてのサブシステムが正しく相互作用
3. **実世界検証:** 実際の屋外条件下でのパフォーマンス
4. **安全保証:** 厳格な安全プロトコルと認証
5. **継続的改善:** 自動化されたCI/CD、継続的監視

---

**ドキュメントステータス:** 完成
**フェーズ6ステータス:** 100% 完了（統合済み）
**承認要件:** QAリード、安全責任者、システム統合リード
