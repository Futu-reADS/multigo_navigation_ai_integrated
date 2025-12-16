# 完全デプロイメントガイド

**ドキュメントID:** DEPLOY-GUIDE-COMPLETE-001
**バージョン:** 1.0
**日付:** 2025-12-15
**ステータス:** 包括的デプロイメントリファレンス

---

## フェーズ7ドキュメント完成

本ドキュメントは、屋外車椅子搬送ロボットシステムの完全なデプロイメントリファレンスを提供します。ハードウェア組み立てから、ソフトウェアインストール、キャリブレーション手順、運用マニュアル、保守ガイドまでをカバーしています。

---

## 目次

1. [ハードウェア組み立て](#1-ハードウェア組み立て)
2. [ソフトウェアインストール](#2-ソフトウェアインストール)
3. [キャリブレーション手順](#3-キャリブレーション手順)
4. [運用マニュアル](#4-運用マニュアル)
5. [保守ガイド](#5-保守ガイド)

---

## 1. ハードウェア組み立て

### 1.1 部品表（BOM）

**コンピューティングプラットフォーム:**
| 部品 | 仕様 | 数量 | 単価（概算） | 備考 |
|------|------|------|------------|------|
| GMKtec Nucbox K6 | Ryzen 7 7840HS, 32GB RAM | 1 | $600 | メインコンピューター |
| microSD カード | 128GB, Class 10 | 1 | $15 | OSインストール用 |
| SSD (オプション) | 1TB NVMe | 1 | $80 | ログストレージ拡張 |

**センサー:**
| 部品 | 仕様 | 数量 | 単価（概算） | 備考 |
|------|------|------|------------|------|
| 3D LiDAR | 屋外対応, IP65+, 360° | 1 | $1,200-2,500 | Velodyneプロトコル互換 |
| RGBカメラ | 1080p, 60fps, USB3.0 | 2 | $40 | ArUco検出用（前面+後面） |
| IMU（9軸） | ±250 dps, ±16g | 1 | $25 | 傾斜補正用 |

**アクチュエーター:**
| 部品 | 仕様 | 数量 | 単価（概算） | 備考 |
|------|------|------|------------|------|
| ホバーボード用インホイールモーター | 6.5", 350W, エンコーダー付き | 8 | $60 | 4駆動 + 4操舵 |
| モーターコントローラー | CAN対応, 500W | 8 | $80 | CANopen CiA 402 |

**電源:**
| 部品 | 仕様 | 数量 | 単価（概算） | 備考 |
|------|------|------|------------|------|
| LiFePO4バッテリー | 48V 20Ah (≥1kWh) | 1 | $400 | 4時間以上動作 |
| DC-DCコンバーター | 48V→12V, 10A | 1 | $30 | コンピューター、センサー |
| DC-DCコンバーター | 48V→5V, 5A | 1 | $20 | UI、eHMI |
| バッテリー管理システム（BMS） | 48V, 20A, バランシング | 1 | $60 | バッテリー保護 |

**安全性:**
| 部品 | 仕様 | 数量 | 単価（概算） | 備考 |
|------|------|------|------------|------|
| 非常停止ボタン | 赤色きのこ型、NC接点 | 2 | $15 | 前面+後面 |
| 安全リレー | 強制ガイド接点、<100ms | 1 | $50 | E-Stopループ |
| 警告灯 | LED、赤色回転灯 | 1 | $25 | 移動中表示 |

**UI/eHMI:**
| 部品 | 仕様 | 数量 | 単価（概算） | 備考 |
|------|------|------|------------|------|
| タッチスクリーン | 7-10", IP65, ≥400 nit | 1 | $150 | メインUI |
| ESP32-S3 | DevKitC-1, USB | 1 | $10 | eHMIコントローラー |
| WS2812B LEDストリップ | 60 LED/m, IP65 | 2m | $20 | 周囲照明 |
| HUB75 LEDマトリックス | 64×32, P4 | 1 | $40 | テキスト表示 |
| I2Sオーディオアンプ | MAX98357A, 3W | 1 | $5 | 音声出力 |
| スピーカー | 3W, 8Ω, 防水 | 1 | $10 | 音声フィードバック |

**構造・機械:**
| 部品 | 仕様 | 数量 | 単価（概算） | 備考 |
|------|------|------|------------|------|
| アルミフレーム | 40×40mm, 長さ可変 | 6m | $60 | シャシー構造 |
| スワーブモジュールハウジング | 3Dプリント/CNC | 4 | $30 | モーター取り付け |
| エンクロージャー | IP54, プラスチック | 1 | $80 | エレクトロニクス保護 |
| ベアリング | ラジアル, 操舵軸用 | 8 | $5 | スワーブ回転 |

**配線:**
| 部品 | 仕様 | 数量 | 単価（概算） | 備考 |
|------|------|------|------------|------|
| CANバスケーブル | ツイストペア, シールド | 10m | $20 | モーターコントローラー |
| 電源ケーブル | 10 AWG, 高柔軟性 | 10m | $15 | 48V配線 |
| USB3.0ケーブル | 5m, ロック機構 | 3 | $15 | カメラ、eHMI |
| イーサネットケーブル | Cat6, 屋外対応 | 5m | $10 | LiDAR |
| コネクター各種 | XT60, Anderson, JST | - | $30 | 電源接続 |

**合計概算:** $4,000 - 5,500（数量・サプライヤーにより変動）

---

### 1.2 組み立て手順

**ステップ1: シャシー組み立て**
```
1. アルミフレームを以下の寸法でカット:
   - 長辺: 1200mm × 2本
   - 短辺: 800mm × 2本
   - 垂直支柱: 600mm × 4本

2. フレームを長方形に組み立て（1200×800mm）:
   - L字ブラケットで固定
   - 対角線測定で直角を確認

3. 垂直支柱を4隅に取り付け
```

**ステップ2: スワーブモジュール取り付け**
```
1. 各コーナーにスワーブモジュールを配置:
   - FL（前左）: (600mm, 400mm)
   - FR（前右）: (600mm, -400mm)
   - RL（後左）: (-600mm, 400mm)
   - RR（後右）: (-600mm, -400mm)

2. 各モジュールの組み立て:
   - ベアリングをハウジングに圧入
   - 操舵モーター取り付け（垂直軸）
   - 駆動モーター取り付け（水平、ホイール直結）
   - エンコーダー配線接続

3. シャシーに固定（M8ボルト）
```

**ステップ3: 電源システム**
```
1. バッテリー取り付け:
   - シャシー中央、低重心位置
   - 振動パッド使用
   - ストラップで固定

2. メインパワースイッチ接続:
   - バッテリー+ → スイッチ → 配電

3. DC-DCコンバーター取り付け:
   - 48V→12V: GMKtec、センサー用
   - 48V→5V: UI、eHMI用

4. ヒューズ追加:
   - メイン: 30A（バッテリー出力）
   - 12V線: 15A
   - 5V線: 5A
```

---

### 1.3 配線図

**電源配線:**
```
バッテリー (48V 20Ah)
    │
    ├─→ メインパワースイッチ
    ├─→ 非常停止リレー（NC、<100ms）
    ├─→ DC-DC 48V→12V（コンピューター、センサー）
    ├─→ DC-DC 48V→5V（UI、eHMI）
    └─→ モーターコントローラー（8×）→ スワーブモーター

非常停止ループ:
E-Stop1（前面）─┐
E-Stop2（後面）─┼─→ 安全リレー → モーター電源カット
手動リセット ───┘
```

**CANバス配線:**
```
GMKtec Nucbox K6 (CANハット経由)
    │
    ├─ CAN_H ───┬─ MC1 (FL drive)
    ├─ CAN_L    ├─ MC2 (FL steer)
    │           ├─ MC3 (FR drive)
    │           ├─ MC4 (FR steer)
    │           ├─ MC5 (RL drive)
    │           ├─ MC6 (RL steer)
    │           ├─ MC7 (RR drive)
    │           └─ MC8 (RR steer)
    │
    └─ 終端抵抗 120Ω（両端）
```

**センサー接続:**
```
GMKtec Nucbox K6:
    ├─ USB3.0 ポート1 → 前面カメラ
    ├─ USB3.0 ポート2 → 後面カメラ
    ├─ USB3.0 ポート3 → ESP32-S3 (eHMI)
    ├─ USB2.0 ポート1 → IMU (USB-UART変換)
    └─ イーサネット → LiDAR (192.168.1.201)
```

---

## 2. ソフトウェアインストール

### 2.1 Ubuntu 22.04セットアップ

**ステップ1: OSインストール**
```bash
# Ubuntu 22.04 LTSをmicroSDまたはSSDにインストール
# インストーラーの設定:
# - ユーザー名: wheelchair-robot
# - ホスト名: wheelchair-robot-01
# - 最小インストール（GUI不要）
# - SSH有効化
```

**ステップ2: ネットワーク設定**
```bash
# 静的IPを設定（ロボット運用時）
sudo nano /etc/netplan/01-netcfg.yaml

# 内容:
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: false
      addresses: [192.168.1.100/24]
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]

# 適用
sudo netplan apply
```

**ステップ3: システム更新**
```bash
sudo apt update
sudo apt upgrade -y
sudo apt dist-upgrade -y
sudo reboot
```

---

### 2.2 ROS 2 Humbleインストール

**開発ガイドの手順を参照:** `05_DEVELOPMENT/COMPLETE_DEVELOPMENT_GUIDE_JA.md`

簡潔版:
```bash
# リポジトリ追加
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu jammy main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# インストール
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools

# 環境設定
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 2.3 ワークスペースセットアップ

```bash
# ワークスペース作成
mkdir -p ~/wheelchair_robot_ws/src
cd ~/wheelchair_robot_ws/src

# リポジトリクローン
git clone <repository_url>

# 依存関係インストール
cd ~/wheelchair_robot_ws
rosdep install --from-paths src --ignore-src -r -y

# ビルド
colcon build --symlink-install

# 環境ソース
source ~/wheelchair_robot_ws/install/setup.bash
echo "source ~/wheelchair_robot_ws/install/setup.bash" >> ~/.bashrc
```

---

### 2.4 システムサービス設定

**ROS 2自動起動サービス:**
```bash
# サービスファイル作成
sudo nano /etc/systemd/system/wheelchair-robot.service

# 内容:
[Unit]
Description=Wheelchair Transport Robot ROS 2 Service
After=network.target

[Service]
Type=simple
User=wheelchair-robot
WorkingDirectory=/home/wheelchair-robot/wheelchair_robot_ws
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && \
  source /home/wheelchair-robot/wheelchair_robot_ws/install/setup.bash && \
  ros2 launch wheelchair_robot_bringup robot_bringup.launch.py'
Restart=on-failure
RestartSec=5s

[Install]
WantedBy=multi-user.target

# サービス有効化
sudo systemctl daemon-reload
sudo systemctl enable wheelchair-robot.service
sudo systemctl start wheelchair-robot.service

# ステータス確認
sudo systemctl status wheelchair-robot.service
```

---

## 3. キャリブレーション手順

### 3.1 LiDARキャリブレーション

**目的:** LiDARの base_link 相対位置を決定

**手順:**
```bash
# 1. LiDARをロボットシャシーに取り付け
#    位置: 中央、高さ0.5m（地面から）

# 2. 座標測定:
#    - X: ロボット前方からの距離（例: 0.0m = 中央）
#    - Y: 中心線からの横方向オフセット（例: 0.0m = 中央）
#    - Z: 地面からの高さ（例: 0.5m）

# 3. TFスタティックパブリッシャーを更新
# robot_description/urdf/sensors.urdf.xacro:

<joint name="base_link_to_lidar" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.0 0.0 0.5" rpy="0 0 0"/>
</joint>

# 4. 確認
ros2 run tf2_tools view_frames
# base_link → lidar_link変換を確認
```

---

### 3.2 カメラキャリブレーション

**ステップ1: 内部パラメーターキャリブレーション**
```bash
# チェスボードパターン印刷（8×6、正方形サイズ25mm）

# カメラキャリブレーションノード起動
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.025 \
  image:=/sensors/camera/front/image_raw \
  camera:=/sensors/camera/front

# チェスボードを様々な角度・距離で移動
# "CALIBRATE"ボタンが有効になるまで（X, Y, Size, Skewが緑）
# "CALIBRATE"クリック → "SAVE"クリック

# キャリブレーションファイル保存:
# /tmp/calibrationdata.tar.gz → config/camera_front_calibration.yaml
```

**ステップ2: 外部パラメーターキャリブレーション**
```bash
# カメラの base_link 相対位置を測定

# 前面カメラ（ArUco検出用）:
#   位置: ロボット前面、高さ1.0m、下向き15°
#   TF: base_link → front_camera_link

<joint name="base_link_to_front_camera" type="fixed">
  <parent link="base_link"/>
  <child link="front_camera_link"/>
  <origin xyz="0.6 0.0 1.0" rpy="0 0.26 0"/>  <!-- 15° = 0.26 rad -->
</joint>

# 後面カメラ:
#   位置: ロボット後面、高さ1.0m、下向き15°
#   TF: base_link → rear_camera_link

<joint name="base_link_to_rear_camera" type="fixed">
  <parent link="base_link"/>
  <child link="rear_camera_link"/>
  <origin xyz="-0.6 0.0 1.0" rpy="0 0.26 3.14"/>  <!-- 180° yaw -->
</joint>
```

---

### 3.3 IMUキャリブレーション

**目的:** IMUバイアスと軸アライメントを補正

**手順:**
```bash
# 1. IMUを水平面に静置（ロボット停止、平坦な地面）

# 2. バイアスキャリブレーション実行
ros2 run imu_calibration calibrate_imu

# 3. 100サンプル収集（約10秒）

# 4. キャリブレーションパラメーター保存:
# config/imu_calibration.yaml:

imu:
  gyro_bias:
    x: 0.002
    y: -0.001
    z: 0.003
  accel_bias:
    x: 0.05
    y: -0.03
    z: 9.81  # 重力オフセット
  rotation_matrix:  # base_linkとIMU軸のアライメント
    - [1, 0, 0]
    - [0, 1, 0]
    - [0, 0, 1]
```

---

### 3.4 ホイールエンコーダーキャリブレーション

**目的:** ホイール半径とエンコーダースケールファクター決定

**手順:**
```bash
# 1. ロボットを平坦な地面に配置、マーク付け

# 2. キャリブレーションノード起動
ros2 run swerve_drive_controller wheel_calibration

# 3. ロボットを直進5m移動させる（手動または自動）

# 4. 実際の移動距離を測定（テープメジャー使用）

# 5. エンコーダーカウント数を記録

# 6. ホイール半径計算:
#    wheel_radius = actual_distance / (encoder_counts * 2π)

# 例:
# 実測距離: 5.00m
# エンコーダーカウント: 9720パルス
# ホイール半径 = 5.00 / (9720 * 2π) = 0.0819m ≈ 82mm

# 7. config/swerve_params.yaml を更新:
swerve:
  wheel_radius: 0.0819  # メートル
  encoder_resolution: 2048  # パルス/回転
```

---

## 4. 運用マニュアル

### 4.1 起動手順

**日次起動チェックリスト:**
```
□ 目視検査:
  □ タイヤの空気圧/損傷確認
  □ 配線の緩み/損傷確認
  □ E-Stopボタン動作確認（押して解放）
  □ 警告灯動作確認

□ バッテリー:
  □ 電圧確認（≥48V、推奨≥50V）
  □ BMS正常表示確認
  □ 充電レベル ≥80%（フルミッション用）

□ センサー:
  □ LiDAR回転確認（電源投入時）
  □ カメラ画像確認（障害物なし、レンズ清掃）
  □ IMU接続確認（緑LED）

□ ソフトウェア:
  □ システムサービス起動確認
    sudo systemctl status wheelchair-robot.service
  □ 全ノード起動確認
    ros2 node list
  □ ログエラー確認
    ros2 topic echo /safety/status
```

**システム起動:**
```bash
# 自動起動の場合（システムサービス有効時）:
sudo systemctl start wheelchair-robot.service

# 手動起動の場合:
cd ~/wheelchair_robot_ws
source install/setup.bash
ros2 launch wheelchair_robot_bringup robot_bringup.launch.py
```

**UIアクセス:**
```
# タッチスクリーンから:
# 1. システム起動を待つ（約30秒）
# 2. メイン画面が表示される
# 3. ステータス確認（全システム緑）
```

---

### 4.2 マッピング手順

**新しいエリアの初回マップ作成:**
```bash
# 1. ロボットを開始位置に配置

# 2. マッピングモード起動
ros2 launch wheelchair_robot_mapping slam_mapping.launch.py

# 3. 手動操縦でロボットを移動（ジョイスティック使用）
#    - ゆっくり移動（<0.5 m/s）
#    - 全エリアをカバー
#    - 同じ場所を複数回通過（ループクロージャー）

# 4. RVizでマップ品質確認

# 5. マップ保存
ros2 run nav2_map_server map_saver_cli -f ~/maps/building_floor_1

# 6. 生成ファイル:
#    - building_floor_1.pgm （画像）
#    - building_floor_1.yaml （メタデータ）
```

**マップ品質チェック:**
```
✅ 壁が連続（途切れなし）
✅ 障害物が正確に表現
✅ グリッド解像度適切（推奨: 0.05m）
✅ ループクロージャー成功（開始点=終了点）
```

---

### 4.3 ミッション実行

**ルート設定:**
```yaml
# config/routes/route_building_A.yaml
route_name: "Building A - 4 Stop Route"
waypoints:
  - name: "Entrance"
    pose:
      x: 0.0
      y: 0.0
      yaw: 0.0
    action: "DOCK_WHEELCHAIR"  # 車椅子ピックアップ

  - name: "Elevator Lobby"
    pose:
      x: 50.0
      y: 10.0
      yaw: 1.57
    action: "WAIT"  # 5秒待機

  - name: "Clinic Entrance"
    pose:
      x: 80.0
      y: 25.0
      yaw: 0.0
    action: "UNDOCK_WHEELCHAIR"  # 車椅子ドロップオフ

  - name: "Return to Entrance"
    pose:
      x: 0.0
      y: 0.0
      yaw: 3.14
    action: "IDLE"
```

**ミッション開始:**
```bash
# UIから:
# 1. "Select Mission"をタップ
# 2. "Building A - 4 Stop Route"を選択
# 3. ミッションパラメーター確認
# 4. "Start Mission"をタップ

# CLIから（デバッグ用）:
ros2 action send_goal /execute_mission \
  custom_msgs/action/ExecuteMission \
  "{mission_id: 'route_building_A', route_file: 'config/routes/route_building_A.yaml'}"
```

**ミッション監視:**
```bash
# 現在のミッション状態
ros2 topic echo /robot_status

# ドッキングステータス
ros2 topic echo /docking/status

# 安全ステータス
ros2 topic echo /safety/status

# RVizで可視化
rviz2 -d config/mission_viz.rviz
```

---

### 4.4 緊急手順

**シナリオ1: 非常停止起動**
```
1. E-Stopボタンを押す（赤いきのこ型）
2. ロボットが即座に停止することを確認（<100ms）
3. 状況評価:
   - 障害物を除去
   - 故障確認
   - ログ確認
4. 安全確認後、E-Stopボタンをひねって解放
5. UIから"Reset Emergency Stop"
6. 操作再開またはミッション中止
```

**シナリオ2: 自己位置推定喪失**
```
症状: /current_pose が発行されない、またはlocalization_quality < 0.3

対応:
1. ロボット停止（E-Stop）
2. 既知の位置に手動移動
3. RVizで"2D Pose Estimate"ツール使用
4. ロボットの位置と向きを手動設定
5. NDTが収束するまで待機（品質 > 0.7）
6. ミッション再開
```

**シナリオ3: バッテリー低下**
```
警告: battery_percent < 20%

対応:
1. 現在のミッション中止
2. 充電ステーションへナビゲート
3. バッテリー交換または充電（≥80%）
4. システム再起動
```

**シナリオ4: センサー故障**
```
LiDAR故障:
  → 即座にミッション中止、手動操作に切り替え

カメラ故障:
  → ドッキング機能無効、ナビゲーション継続可能

IMU故障:
  → 地面除去精度低下、平坦地のみ運用
```

---

### 4.5 シャットダウン手順

**通常シャットダウン:**
```bash
# 1. すべてのミッション完了確認

# 2. UIから"Shutdown Robot"

# または CLIから:
sudo systemctl stop wheelchair-robot.service

# 3. システムシャットダウン
sudo shutdown -h now

# 4. メインパワースイッチOFF

# 5. 保管:
#    - 屋内保管（雨風を避ける）
#    - バッテリー保管電圧: 48-50V（長期保管時）
```

---

## 5. 保守ガイド

### 5.1 定期保守スケジュール

**日次（運用前）:**
```
□ 目視検査（タイヤ、配線、ボディ）
□ E-Stop機能テスト
□ バッテリー電圧確認
□ センサーレンズ清掃（カメラ、LiDAR）
□ ログエラー確認
□ ソフトウェアステータス確認
```

**週次:**
```
□ タイヤ空気圧確認/調整
□ ボルト締結トルク確認
□ 配線接続確認
□ バッテリー端子清掃
□ LiDAR校正チェック
□ オドメトリー精度確認（10m直進テスト）
```

**月次:**
```
□ モーターベアリンググリース注入
□ CANバス通信品質テスト
□ カメラ再キャリブレーション
□ IMU再キャリブレーション
□ ソフトウェア更新確認
□ バックアップ実行（マップ、設定ファイル）
```

**四半期:**
```
□ タイヤ交換（摩耗度に応じて）
□ バッテリー容量テスト
□ 全モーターエンコーダーキャリブレーション
□ LiDAR点群品質評価
□ フルシステム負荷テスト
```

**年次:**
```
□ バッテリー交換（容量<80%時）
□ モーターベアリング交換
□ 全電気接続点検
□ ソフトウェア全面監査
□ 安全認証更新（必要に応じて）
```

---

### 5.2 コンポーネント交換手順

**タイヤ交換:**
```
1. ロボットをジャッキアップ
2. モーターコントローラー電源OFF
3. ホイールナット取り外し
4. モーターとホイール分離
5. 新しいホイール取り付け
6. エンコーダーキャリブレーション実行
7. 走行テスト
```

**バッテリー交換:**
```
1. メインパワースイッチOFF
2. バッテリーコネクター取り外し（XT60）
3. 固定ストラップ解除
4. 古いバッテリー取り外し（重量注意: 約8kg）
5. 新しいバッテリー設置
6. BMS接続確認
7. 電圧測定（48-54V）
8. システム起動テスト
```

**カメラ交換:**
```
1. システムシャットダウン
2. USB接続取り外し
3. マウント固定ネジ取り外し
4. 新しいカメラ取り付け
5. 内部/外部パラメーターキャリブレーション
6. ArUco検出テスト
```

---

### 5.3 トラブルシューティングガイド

**問題: ロボットが直進しない（横流れ）**
```
原因: ホイール半径キャリブレーション不正確

解決策:
1. 各ホイール半径を再測定
2. config/swerve_params.yaml 更新
3. 10m直進テストで検証
```

**問題: NDT自己位置推定がドリフト**
```
原因: マップ品質低下、またはセンサー劣化

解決策:
1. LiDAR点群品質確認（ゴースト点、ノイズ）
2. マップ再作成（必要に応じて）
3. NDTパラメーター調整（分解能、イテレーション）
```

**問題: ドッキング精度低下（>5mm誤差）**
```
原因: カメラキャリブレーションドリフト、照明変化

解決策:
1. カメラ内部パラメーター再キャリブレーション
2. ArUcoマーカー印刷品質確認（摩耗、反射）
3. 照明補正パラメーター調整（CLAHE）
```

**問題: CANバス通信エラー**
```
原因: ケーブル損傷、コネクター緩み、終端抵抗不良

解決策:
1. CANバスツールで診断（candump、cansend）
2. 終端抵抗測定（120Ω両端）
3. ケーブル導通テスト
4. コネクター再圧着
```

---

### 5.4 スペアパーツリスト

**推奨在庫（運用サイト）:**
| 部品 | 数量 | 交換頻度 | 重要度 |
|------|------|---------|--------|
| タイヤ（6.5"ホバーボード） | 2 | 6ヶ月 | 高 |
| バッテリー（48V 20Ah） | 1 | 2年 | 高 |
| USBカメラ | 1 | 1年 | 中 |
| E-Stopボタン | 1 | 2年 | 高 |
| CANケーブル（5m） | 2 | 必要時 | 中 |
| ヒューズ（30A, 15A, 5A） | 各5 | 必要時 | 高 |
| XT60コネクター | 5セット | 必要時 | 中 |
| DC-DCコンバーター（48V→12V） | 1 | 3年 | 中 |

---

## まとめ

完全なデプロイメントガイドにより以下が可能になります:
1. **適切な組み立て:** 詳細なBOMと手順
2. **信頼性の高いインストール:** ステップバイステップのソフトウェア設定
3. **正確なキャリブレーション:** すべてのセンサーとアクチュエーター
4. **安全な運用:** 包括的な手順と緊急プロトコル
5. **長期メンテナンス:** 定期スケジュールとトラブルシューティング

---

**ドキュメントステータス:** 完成
**フェーズ7ステータス:** 100% 完了（統合済み）
**承認要件:** ハードウェアエンジニア、運用マネージャー、安全責任者
