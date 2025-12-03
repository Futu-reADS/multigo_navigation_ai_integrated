# クイックセットアップガイド

**所要時間:** 30分
**前提条件:** Ubuntu 22.04、基本的なROS 2の知識

---

## 1. ROS 2 Humbleのインストール (15分)

```bash
# ROS 2 aptリポジトリの追加
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS 2 Humbleのインストール
sudo apt update
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup

# 依存関係のインストール
sudo apt install python3-colcon-common-extensions python3-rosdep
```

---

## 2. リポジトリのクローン (2分)

```bash
mkdir -p ~/multigo_ws/src
cd ~/multigo_ws/src
git clone <repository_url> multigo_navigation_ai_integrated
cd ..
```

---

## 3. 依存関係のインストール (5分)

```bash
cd ~/multigo_ws

# rosdepの初期化
sudo rosdep init  # 既に実行済みの場合はスキップ
rosdep update

# 依存関係のインストール
rosdep install --from-paths src --ignore-src -r -y
```

---

## 4. ビルド (5分)

```bash
cd ~/multigo_ws

# ROS 2のソース
source /opt/ros/humble/setup.bash

# ビルド
colcon build --symlink-install

# ワークスペースのソース
source install/setup.bash
```

---

## 5. シミュレーションでのテスト (3分)

```bash
# シミュレーションの起動
ros2 launch boot simulation.launch.py

# 別のターミナルで:
source ~/multigo_ws/install/setup.bash

# テストゴールの送信
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 1.0, y: 0.0, z: 0.0}}
}"
```

---

## トラブルシューティング

**ビルドエラー？**
- ROS 2がソースされているか確認: `echo $ROS_DISTRO` (「humble」と表示されるはず)
- 依存関係の不足: `rosdep install --from-paths src --ignore-src -r -y`

**シミュレーションが起動しない？**
- Gazeboのインストール: `sudo apt install ros-humble-gazebo-ros-pkgs`

**テストが失敗する？**
- 予想通りです！現在のカバレッジは0%です。フェーズ2については、IMPLEMENTATION-GUIDEを参照してください。

---

**次のステップ:**
- 読む: [START-HERE.md](./START-HERE.md) / [START-HERE_ja.md](./START-HERE_ja.md)
- バグ修正: [IMPLEMENTATION-GUIDE.md - フェーズ1](./IMPLEMENTATION-GUIDE.md#phase-1-critical-bugs--safety)
- システムを理解: [SYSTEM-ARCHITECTURE.md](./SYSTEM-ARCHITECTURE.md) / [SYSTEM-ARCHITECTURE_ja.md](./SYSTEM-ARCHITECTURE_ja.md)

**最終更新:** 2025-12-02
