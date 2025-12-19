# インターフェース要件

**文書ID:** REQ-INT-001
**バージョン:** 1.0
**日付:** 2025年12月15日
**ステータス:** ドラフト

---

## 1. はじめに

この文書は、ROS 2トピック、サービス、アクション、外部APIを含むすべての**ソフトウェアインターフェース**を規定します。

---

## 2. ROS 2トピック

### 2.1 センサートピック（INT-TOPIC-SENS）

| トピック | タイプ | パブリッシャー | サブスクライバー | レート | QoS |
|-------|------|-----------|------------|------|-----|
| /points | sensor_msgs/PointCloud2 | lidar_driver | perception | 10 Hz | BEST_EFFORT |
| /camera_*/image_raw | sensor_msgs/Image | camera_driver | aruco_detector | 30 Hz | BEST_EFFORT |
| /imu/data | sensor_msgs/Imu | imu_driver | localization | 50 Hz | RELIABLE |
| /odom | nav_msgs/Odometry | wheel_odometry | localization | 50 Hz | RELIABLE |

**合計: 4センサートピック**

### 2.2 ナビゲーショントピック（INT-TOPIC-NAV）

| トピック | タイプ | パブリッシャー | サブスクライバー | レート | QoS |
|-------|------|-----------|------------|------|-----|
| /cmd_vel | geometry_msgs/Twist | controller | swerve_driver | 20 Hz | RELIABLE |
| /pose | geometry_msgs/PoseStamped | localization | planner | 10 Hz | RELIABLE |
| /costmap | nav_msgs/OccupancyGrid | perception | planner | 5 Hz | RELIABLE |

**合計: 3ナビゲーショントピック**

### 2.3 安全トピック（INT-TOPIC-SAFE）

| トピック | タイプ | パブリッシャー | サブスクライバー | レート | QoS |
|-------|------|-----------|------------|------|-----|
| /emergency_stop | std_msgs/Bool | safety_supervisor | all_nodes | イベント | RELIABLE |
| /diagnostics | diagnostic_msgs/DiagnosticArray | all_nodes | monitor | 1 Hz | RELIABLE |

**合計: 2安全トピック**

---

## 3. ROS 2サービス

### 3.1 システムサービス（INT-SRV-SYS）

| サービス | タイプ | サーバー | 目的 |
|---------|------|--------|---------|
| /start_mission | std_srvs/Trigger | mission_manager | ナビゲーションミッションを開始 |
| /cancel_mission | std_srvs/Trigger | mission_manager | 現在のミッションをキャンセル |
| /reset_localization | std_srvs/Trigger | localization | 初期姿勢にリセット |

**合計: 3サービス**

---

## 4. ROS 2アクション

### 4.1 ナビゲーションアクション（INT-ACT-NAV）

| アクション | タイプ | サーバー | 目的 |
|--------|------|--------|---------|
| /navigate_to_pose | nav2_msgs/NavigateToPose | nav2_bt_navigator | 単一のゴールへナビゲート |
| /follow_waypoints | nav2_msgs/FollowWaypoints | nav2_bt_navigator | マルチウェイポイントミッション |
| /dock | custom_msgs/Dock | docking_controller | 精密ドッキング |

**合計: 3アクション**

---

## 5. 外部API

### 5.1 UI REST API（INT-API-UI）

| エンドポイント | メソッド | 目的 |
|----------|--------|---------|
| /api/mission/start | POST | ウェイポイント付きミッションを開始 |
| /api/mission/status | GET | ミッションステータスを取得 |
| /api/robot/state | GET | ロボット状態を取得 |
| /api/emergency_stop | POST | 緊急停止をトリガー |

**合計: 4 APIエンドポイント**

---

## 6. eHMIシリアルプロトコル

### 6.1 コマンド（INT-SERIAL-CMD）

| コマンド | 形式 | 目的 |
|---------|--------|---------|
| STATE | STATE:{num}\n | eHMI状態を設定（0-28） |
| VOLUME | VOLUME:{0-10}\n | 音声ボリュームを設定 |
| LANGUAGE | LANGUAGE:{EN\|JP\|ZH}\n | 言語を設定 |

**合計: 3コマンド**

---

## 7. 要件サマリー

| カテゴリー | 合計 |
|----------|-------|
| **ROS 2トピック** | **9** |
| **ROS 2サービス** | **3** |
| **ROS 2アクション** | **3** |
| **REST API** | **4** |
| **シリアルプロトコル** | **3** |
| **合計** | **22** |

---

**文書ステータス:** ドラフト
**必要な承認:** ソフトウェアリード、統合エンジニア
