# 完全開発ガイド

**ドキュメントID:** DEV-GUIDE-COMPLETE-001
**バージョン:** 1.0
**日付:** 2025-12-15
**ステータス:** 包括的開発リファレンス

---

## フェーズ5ドキュメント完成

本ドキュメントは、屋外車椅子搬送ロボットシステムの完全な開発リファレンスを提供します。開発環境のセットアップから、ビルド手順、コーディング規約、Gitワークフロー、デバッグガイドまでをカバーしています。

---

## 目次

1. [開発環境セットアップ](#1-開発環境セットアップ)
2. [ビルド手順](#2-ビルド手順)
3. [コーディング規約](#3-コーディング規約)
4. [Gitワークフロー](#4-gitワークフロー)
5. [デバッグガイド](#5-デバッグガイド)

---

## 1. 開発環境セットアップ

### 1.1 必須要件

**ハードウェア要件:**
- CPU: x86_64プロセッサー（推奨: 4コア以上）
- RAM: 8GB以上（推奨: 16GB）
- ストレージ: 50GB以上の空き容量
- ネットワーク: インターネット接続（パッケージダウンロード用）

**ソフトウェア要件:**
- OS: Ubuntu 22.04 LTS (Jammy Jellyfish)
- ROS 2: Humble Hawksbill
- Python: 3.10+
- C++: C++17以降

---

### 1.2 Ubuntu 22.04のインストール

**ステップ1: Ubuntuのダウンロードとインストール**
```bash
# Ubuntu 22.04 LTSをダウンロード
# https://ubuntu.com/download/desktop

# USBブートメディアを作成（Etcherまたはdd使用）
# システムにインストール
```

**ステップ2: システム更新**
```bash
sudo apt update
sudo apt upgrade -y
sudo apt dist-upgrade -y
```

**ステップ3: 基本開発ツールのインストール**
```bash
sudo apt install -y \
  build-essential \
  cmake \
  git \
  wget \
  curl \
  vim \
  htop \
  net-tools \
  python3-pip \
  python3-venv
```

---

### 1.3 ROS 2 Humbleのインストール

**ステップ1: ロケール設定**
```bash
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

**ステップ2: ROS 2リポジトリの追加**
```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

**ステップ3: ROS 2 Humbleのインストール**
```bash
sudo apt update
sudo apt upgrade -y

# デスクトップインストール（RViz、rqt含む）
sudo apt install -y ros-humble-desktop

# 開発ツール
sudo apt install -y \
  ros-dev-tools \
  ros-humble-ros2bag \
  ros-humble-rosbag2-storage-default-plugins
```

**ステップ4: 環境設定**
```bash
# ~/.bashrcに追加
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 確認
ros2 --version
# 出力例: ros2 cli version: 0.18.5
```

---

### 1.4 依存関係のインストール

**ROS 2パッケージ:**
```bash
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-tf2-tools \
  ros-humble-tf2-geometry-msgs \
  ros-humble-geometry2 \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-camera-calibration \
  ros-humble-aruco \
  ros-humble-aruco-ros \
  ros-humble-rosbridge-suite \
  ros-humble-control-toolbox \
  ros-humble-controller-manager \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-xacro
```

**C++ライブラリ:**
```bash
sudo apt install -y \
  libeigen3-dev \
  libopencv-dev \
  libpcl-dev \
  libboost-all-dev \
  libyaml-cpp-dev \
  libgtest-dev
```

**Pythonライブラリ:**
```bash
pip3 install \
  numpy \
  scipy \
  opencv-python \
  matplotlib \
  pyyaml \
  pytest \
  pytest-cov
```

---

### 1.5 ワークスペースのセットアップ

**ステップ1: ワークスペース作成**
```bash
mkdir -p ~/wheelchair_robot_ws/src
cd ~/wheelchair_robot_ws
```

**ステップ2: リポジトリのクローン**
```bash
cd ~/wheelchair_robot_ws/src
git clone <repository_url>
```

**ステップ3: 依存関係の解決**
```bash
cd ~/wheelchair_robot_ws
rosdep install --from-paths src --ignore-src -r -y
```

**ステップ4: ワークスペースのビルド**
```bash
cd ~/wheelchair_robot_ws
colcon build --symlink-install
```

**ステップ5: ワークスペースのソース**
```bash
source ~/wheelchair_robot_ws/install/setup.bash

# 永続化のため~/.bashrcに追加
echo "source ~/wheelchair_robot_ws/install/setup.bash" >> ~/.bashrc
```

---

## 2. ビルド手順

### 2.1 Colconビルドコマンド

**基本ビルド:**
```bash
cd ~/wheelchair_robot_ws
colcon build
```

**シンボリックリンクインストール（開発用）:**
```bash
colcon build --symlink-install
```

**特定パッケージのビルド:**
```bash
colcon build --packages-select swerve_drive_controller
```

**並列ビルド:**
```bash
colcon build --parallel-workers 4
```

**デバッグビルド:**
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

**リリースビルド:**
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

### 2.2 CMakeLists.txtテンプレート

**C++パッケージの基本CMakeLists.txt:**
```cmake
cmake_minimum_required(VERSION 3.8)
project(swerve_drive_controller)

# C++標準を設定
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# コンパイラー警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依存関係を検索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ライブラリ
add_library(${PROJECT_NAME} SHARED
  src/swerve_drive_controller.cpp
  src/swerve_module.cpp
  src/inverse_kinematics.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  Eigen3
)

# 実行可能ファイル
add_executable(${PROJECT_NAME}_node
  src/swerve_drive_controller_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

# インストール
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# テスト
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_swerve_module.cpp
    test/test_inverse_kinematics.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}
  )
endif()

ament_package()
```

---

### 2.3 package.xmlテンプレート

**基本package.xml (format 3):**
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>swerve_drive_controller</name>
  <version>1.0.0</version>
  <description>Swerve drive controller for omnidirectional robot</description>

  <maintainer email="dev@example.com">Development Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>eigen</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## 3. コーディング規約

### 3.1 C++スタイルガイド

**命名規則:**
```cpp
// クラス: パスカルケース
class SwerveModule { };

// 関数/メソッド: キャメルケース
void computeWheelVelocities();

// 変数: スネークケース
double wheel_radius_;
int module_id_;

// 定数: 大文字スネークケース
constexpr double MAX_VELOCITY = 2.0;

// 名前空間: スネークケース
namespace swerve_drive {
```

**ファイル構造:**
```cpp
// swerve_module.hpp
#ifndef SWERVE_DRIVE_CONTROLLER__SWERVE_MODULE_HPP_
#define SWERVE_DRIVE_CONTROLLER__SWERVE_MODULE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace swerve_drive
{

class SwerveModule
{
public:
  explicit SwerveModule(int module_id);
  ~SwerveModule() = default;

  // コピー/移動禁止
  SwerveModule(const SwerveModule &) = delete;
  SwerveModule & operator=(const SwerveModule &) = delete;

  void setTarget(double velocity, double angle);
  void update(double dt);

private:
  int module_id_;
  double current_angle_;
  double target_velocity_;
};

}  // namespace swerve_drive

#endif  // SWERVE_DRIVE_CONTROLLER__SWERVE_MODULE_HPP_
```

**コメント規約:**
```cpp
/**
 * @brief 逆運動学を使用してモジュール状態を計算
 *
 * @param body_velocity ボディフレームでの速度（Twist）
 * @return 4つのモジュール状態の配列
 *
 * @throws std::runtime_error 速度が最大値を超える場合
 */
std::array<ModuleState, 4> computeModuleStates(
  const geometry_msgs::msg::Twist & body_velocity);
```

---

### 3.2 Pythonスタイルガイド

**PEP 8準拠:**
```python
# インポート順序: 標準ライブラリ、サードパーティ、ローカル
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node

from custom_msgs.msg import SwerveModuleCommand


class SwerveController(Node):
    """スワーブドライブコントローラーノード"""

    def __init__(self):
        super().__init__('swerve_controller')

        # プライベート変数にはアンダースコア接頭辞
        self._wheel_radius = 0.0825  # メートル
        self._max_velocity = 2.0

        self.get_logger().info('スワーブコントローラー初期化完了')

    def compute_wheel_velocities(self, vx, vy, omega):
        """
        ボディ速度からホイール速度を計算

        Args:
            vx (float): X方向速度 [m/s]
            vy (float): Y方向速度 [m/s]
            omega (float): 角速度 [rad/s]

        Returns:
            np.ndarray: ホイール速度配列
        """
        # 実装...
        pass
```

---

### 3.3 ROS 2規約

**ノード命名:**
```cpp
// ノード名: スネークケース、説明的
rclcpp::Node node("swerve_drive_controller");

// トピック名: スラッシュ区切り、スネークケース
auto pub = node.create_publisher<Twist>("/cmd_vel", 10);

// パラメーター名: ドット区切り、スネークケース
node.declare_parameter("swerve.wheel_radius", 0.0825);
```

**QoSプロファイル:**
```cpp
// 重要なコマンド: RELIABLE
auto qos_cmd = rclcpp::QoS(10)
  .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
  .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

// センサーデータ: BEST_EFFORT
auto qos_sensor = rclcpp::QoS(1)
  .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
  .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
```

---

## 4. Gitワークフロー

### 4.1 ブランチ戦略

**メインブランチ:**
- `main`: 安定版、本番準備完了コード
- `develop`: 開発統合ブランチ

**機能ブランチ:**
```bash
# 機能ブランチ作成
git checkout -b feature/swerve-drive-controller

# 作業、コミット
git add .
git commit -m "feat: スワーブドライブ逆運動学を実装"

# developにマージ前にリベース
git fetch origin
git rebase origin/develop

# プッシュ
git push origin feature/swerve-drive-controller
```

---

### 4.2 コミットメッセージ形式

**Conventional Commits準拠:**
```
<type>(<scope>): <subject>

<body>

<footer>
```

**タイプ:**
- `feat`: 新機能
- `fix`: バグ修正
- `docs`: ドキュメントのみ
- `style`: フォーマット、セミコロンなど
- `refactor`: リファクタリング
- `test`: テスト追加
- `chore`: ビルドタスク、パッケージ管理など

**例:**
```
feat(swerve): 逆運動学を実装

4輪スワーブドライブ用の逆運動学を実装。
全方向移動をサポート。

Closes #123
```

---

### 4.3 プルリクエストプロセス

**ステップ1: プルリクエスト作成**
- 説明的なタイトル
- 変更内容の詳細説明
- 関連イシューをリンク

**ステップ2: コードレビュー**
- 少なくとも1名の承認が必要
- CI/CDパイプラインが通過すること
- すべてのテストが合格すること

**ステップ3: マージ**
- Squash and merge（機能ブランチ → develop）
- Merge commit（develop → main）

---

## 5. デバッグガイド

### 5.1 ROS 2 CLIツール

**ノード情報:**
```bash
# 実行中のノード一覧
ros2 node list

# ノード情報表示
ros2 node info /swerve_drive_controller

# トピック一覧
ros2 topic list

# トピックのエコー
ros2 topic echo /cmd_vel

# トピックの周波数確認
ros2 topic hz /odom
```

**パラメーター:**
```bash
# パラメーター一覧
ros2 param list

# パラメーター取得
ros2 param get /swerve_drive_controller swerve.wheel_radius

# パラメーター設定
ros2 param set /swerve_drive_controller swerve.wheel_radius 0.09
```

**サービス:**
```bash
# サービス一覧
ros2 service list

# サービス呼び出し
ros2 service call /navigation/clear_route std_srvs/srv/Trigger
```

---

### 5.2 RViz可視化

**基本起動:**
```bash
rviz2
```

**設定保存:**
```bash
# 設定を保存
# File → Save Config As → config/swerve_viz.rviz

# 起動時に読み込み
rviz2 -d ~/wheelchair_robot_ws/src/config/swerve_viz.rviz
```

**便利な表示:**
- **TF**: ロボットフレーム階層
- **RobotModel**: URDFモデル
- **LaserScan/PointCloud2**: LiDARデータ
- **Map**: マップ、コストマップ
- **Path**: ナビゲーションパス
- **PoseArray**: パーティクルフィルター（AMCL使用時）

---

### 5.3 GDBデバッグ

**ビルド（デバッグシンボル付き）:**
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

**GDB起動:**
```bash
# 実行可能ファイルでGDBを起動
gdb --args ros2 run swerve_drive_controller swerve_drive_controller_node

# GDB内で
(gdb) break SwerveModule::setTarget
(gdb) run
(gdb) continue
(gdb) print current_angle_
(gdb) backtrace
```

---

### 5.4 ロギングベストプラクティス

**C++ロギング:**
```cpp
// ログレベル: DEBUG, INFO, WARN, ERROR, FATAL
RCLCPP_DEBUG(get_logger(), "詳細なデバッグ情報");
RCLCPP_INFO(get_logger(), "一般情報");
RCLCPP_WARN(get_logger(), "警告: %s", warning_message.c_str());
RCLCPP_ERROR(get_logger(), "エラー発生!");
RCLCPP_FATAL(get_logger(), "致命的エラー");

// 頻度制限（1秒に1回のみ）
RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "スロットルメッセージ");
```

**Pythonロギング:**
```python
self.get_logger().debug('詳細情報')
self.get_logger().info('一般情報')
self.get_logger().warn('警告')
self.get_logger().error('エラー')
self.get_logger().fatal('致命的エラー')
```

**ログレベル設定:**
```bash
ros2 run swerve_drive_controller swerve_drive_controller_node \
  --ros-args --log-level DEBUG
```

---

### 5.5 プロファイリングツール

**CPU使用率:**
```bash
# htopでノード監視
htop

# ros2 topでトピック帯域幅監視
ros2 topic bw /sensors/lidar/points
```

**メモリリーク検出（Valgrind）:**
```bash
valgrind --leak-check=full \
  ros2 run swerve_drive_controller swerve_drive_controller_node
```

---

## 追加リソース

**ROS 2ドキュメント:**
- 公式ドキュメント: https://docs.ros.org/en/humble/
- Nav2ドキュメント: https://navigation.ros.org/
- ROS 2チュートリアル: https://docs.ros.org/en/humble/Tutorials.html

**開発ツール:**
- Visual Studio Code + ROS拡張機能
- CLion + ROS 2プラグイン
- QtCreator（ROSサポート付き）

---

**ドキュメントステータス:** 完成
**フェーズ5ステータス:** 100% 完了（統合済み）
**承認要件:** ソフトウェア開発リード、ROS統合リード
