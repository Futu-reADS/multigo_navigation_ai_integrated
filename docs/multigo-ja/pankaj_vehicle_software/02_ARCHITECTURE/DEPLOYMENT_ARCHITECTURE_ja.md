# デプロイメントアーキテクチャ（車両ソフトウェア）

**プロジェクト:** 屋外車椅子輸送ロボット
**文書タイプ:** アーキテクチャ仕様
**ステータス:** アクティブ
**バージョン:** 2.0（パイロット向け簡略化）
**最終更新:** 2025年12月19日
**オーナー:** Pankaj（車両ソフトウェア）
**スコープ:** 車両ソフトウェアデプロイメントのみ

---

## 目次

1. [概要](#1-概要)
2. [車両計算プラットフォーム](#2-車両計算プラットフォーム)
3. [ソフトウェアスタック](#3-ソフトウェアスタック)
4. [デプロイメントプロセス](#4-デプロイメントプロセス)
5. [設定管理](#5-設定管理)
6. [更新メカニズム](#6-更新メカニズム)
7. [監視とログ](#7-監視とログ)

---

## 1. 概要

### 1.1 目的

この文書は、車両ソフトウェアがオンボード計算ユニットにどのようにデプロイされるかを定義します。

**スコープ内:**
- 車両ソフトウェアインストール（ROS 2、依存関係、カスタムパッケージ）
- Systemdサービス設定
- 設定ファイルデプロイメント
- 更新メカニズム（パイロットは手動、将来的にOTA）
- ローカルログ設定

**スコープ外:**
- TVMサーバーデプロイメント（Unnoの責任）
- データベースデプロイメント（Unnoの責任）
- クラウドインフラストラクチャ（Unnoの責任）
- ハードウェアインストール（土屋の責任）
- マルチリージョンデプロイメント（パイロットには不要）

### 1.2 デプロイメントトポロジー

```
┌─────────────────────────────────────────────────────────┐
│                    TVMサーバー                           │
│                 (Unnoのデプロイメント)                    │
│         参照: unno_tvm_server/DEPLOYMENT_*.md            │
└──────────────────┬──────────────────────────────────────┘
                   │
                   │ HTTPS/WSS (TVM API)
                   │
┌──────────────────▼──────────────────────────────────────┐
│                車両計算ユニット                            │
│          (この文書のスコープ)                             │
│                                                          │
│  ┌────────────────────────────────────────────────┐    │
│  │  オペレーティングシステム: Ubuntu 22.04 LTS     │    │
│  └────────────────────────────────────────────────┘    │
│                                                          │
│  ┌────────────────────────────────────────────────┐    │
│  │  ミドルウェア: ROS 2 Humble                     │    │
│  └────────────────────────────────────────────────┘    │
│                                                          │
│  ┌────────────────────────────────────────────────┐    │
│  │  車両ソフトウェア（ROS 2ワークスペース）         │    │
│  │  - ナビゲーション（Nav2、NDTローカライゼーション）│    │
│  │  - ドッキング（ArUcoビジュアルサーボイング）     │    │
│  │  - 知覚（LiDAR処理）                           │    │
│  │  - 制御（Swerveドライブコントローラー）         │    │
│  │  - 安全（緊急停止モニター）                     │    │
│  │  - TVMクライアント（REST + WebSocket）        │    │
│  │  - ローカルUI（Reactアプリ）                   │    │
│  └────────────────────────────────────────────────┘    │
│                                                          │
│  ┌────────────────────────────────────────────────┐    │
│  │  Systemdサービス                                │    │
│  │  - multigo-vehicle.service（メイン）            │    │
│  │  - multigo-ui.service（ローカルUI）            │    │
│  └────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────┘
```

---

## 2. 車両計算プラットフォーム

### 2.1 ハードウェア仕様

**計算ユニット:** GMKtec Nucbox K6
- **CPU:** AMD Ryzen 7 7840HS（8コア、16スレッド、3.8-5.1 GHz）
- **RAM:** 32GB DDR5
- **GPU:** AMD Radeon 780M（統合）
- **ストレージ:** 1TB NVMe SSD
- **ポート:** USB 3.2、USB-C、イーサネット、HDMI

**提供元:** 土屋（ハードウェアチーム）
**ソフトウェアチームの責任:** OSとソフトウェアのみインストール

### 2.2 オペレーティングシステム

**OS:** Ubuntu 22.04 LTS（Jammy Jellyfish）
- **カーネル:** 5.15+（デフォルトUbuntu 22.04カーネル）
- **アーキテクチャ:** x86_64
- **インストール方法:** 標準Ubuntu ServerまたはDesktopインストール

**Ubuntu 22.04を選択した理由:**
- ROS 2 HumbleがUbuntu 22.04を公式サポート
- 2027年4月までの長期サポート
- 広範なハードウェアドライバーサポート
- 大規模なコミュニティ

---

## 3. ソフトウェアスタック

### 3.1 システム依存関係

**新しいUbuntu 22.04にインストール:**

```bash
# システム更新
sudo apt update && sudo apt upgrade -y

# ROS 2 Humble依存関係のインストール
sudo apt install -y \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-venv

# ROS 2リポジトリ追加
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS 2 Humbleインストール
sudo apt update
sudo apt install -y ros-humble-desktop  # またはヘッドレス用ros-humble-base

# ROS 2開発ツールのインストール
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization

# rosdep初期化
sudo rosdep init
rosdep update
```

### 3.2 車両ソフトウェア依存関係

**ROS 2パッケージ:**
- `ros-humble-navigation2` - Nav2ナビゲーションスタック
- `ros-humble-nav2-bringup` - Nav2起動ファイル
- `ros-humble-robot-localization` - EKF、UKFローカライゼーション
- `ros-humble-pcl-ros` - 点群処理
- `ros-humble-cv-bridge` - OpenCV-ROSブリッジ
- `ros-humble-image-transport` - カメラ画像転送

**Python依存関係:**
```bash
pip3 install \
    opencv-python \
    numpy \
    scipy \
    requests \
    websockets \
    pyyaml \
    pyjwt
```

**参照:** 完全な依存関係リストは`DEVELOPMENT_SETUP_GUIDE.md`を参照

### 3.3 車両ソフトウェアワークスペース

**ワークスペース構造:**
```
/home/multigo/multigo_ws/
├── src/
│   ├── nav_control/       # ナビゲーションコントローラー
│   ├── nav_docking/       # ドッキングコントローラー
│   ├── nav_goal/          # ゴールマネージャー
│   ├── perception/        # LiDAR処理
│   ├── swerve_drive/      # Swerveドライブコントローラー
│   ├── safety_monitor/    # 安全モニター
│   ├── tvm_client/        # TVM APIクライアント
│   └── local_ui/          # Reactローカル UI
├── build/                 # ビルド成果物（gitで無視）
├── install/               # インストール済みパッケージ
└── log/                   # ビルドログ
```

**ビルドコマンド:**
```bash
cd /home/multigo/multigo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

---

## 4. デプロイメントプロセス

### 4.1 初期デプロイメント（新規インストール）

**ステップ1: Ubuntu 22.04をインストール**
- Ubuntu 22.04 Serverでブータブル USBを作成
- USBから車両計算ユニットをブート
- Ubuntuをインストール（ガイド付きインストール）
- ユーザーを作成: `multigo`（インストール中にパスワード設定）
- インストール中にSSHサーバーを有効化

**ステップ2: システム依存関係をインストール**
```bash
ssh multigo@<vehicle-ip>
sudo apt update && sudo apt upgrade -y
# セクション3.1のシステム依存関係インストールを実行
```

**ステップ3: 車両ソフトウェアリポジトリをクローン**
```bash
mkdir -p /home/multigo/multigo_ws/src
cd /home/multigo/multigo_ws/src
git clone https://github.com/<org>/multigo_navigation.git
```

**ステップ4: ROS依存関係をインストール**
```bash
cd /home/multigo/multigo_ws
rosdep install --from-paths src --ignore-src -r -y
```

**ステップ5: 車両ソフトウェアをビルド**
```bash
cd /home/multigo/multigo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

**ステップ6: 設定ファイルをデプロイ**
```bash
sudo mkdir -p /etc/multigo
sudo cp /home/multigo/multigo_ws/src/multigo_navigation/config/vehicle_config.yaml \
    /etc/multigo/config.yaml
sudo chmod 600 /etc/multigo/config.yaml
sudo chown multigo:multigo /etc/multigo/config.yaml

# 車両固有の値で設定を編集
sudo nano /etc/multigo/config.yaml
# 設定: vehicle_id、tvm_server_url、api_key
```

**ステップ7: Systemdサービスをインストール**
```bash
sudo cp /home/multigo/multigo_ws/src/multigo_navigation/deploy/multigo-vehicle.service \
    /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable multigo-vehicle
sudo systemctl start multigo-vehicle
```

**ステップ8: デプロイメントを検証**
```bash
sudo systemctl status multigo-vehicle
journalctl -u multigo-vehicle -f  # ログをチェック
rostopic list  # ROS トピックが実行中か確認
```

### 4.2 Systemdサービス設定

**ファイル:** `/etc/systemd/system/multigo-vehicle.service`

```ini
[Unit]
Description=Multigo車両ソフトウェア
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=multigo
Group=multigo
WorkingDirectory=/home/multigo/multigo_ws
Environment="ROS_DOMAIN_ID=0"
Environment="RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && \
    source /home/multigo/multigo_ws/install/setup.bash && \
    ros2 launch multigo_bringup vehicle.launch.py"
Restart=always
RestartSec=10s

[Install]
WantedBy=multi-user.target
```

**重要なポイント:**
- `multigo`ユーザーとして実行（rootではない）
- 障害時に自動再起動（10秒遅延）
- ROS 2とワークスペースのセットアップをソース
- メイン車両起動ファイルを開始

---

## 5. 設定管理

### 5.1 設定ファイルの場所

**メイン設定:** `/etc/multigo/config.yaml`

**設定構造例:**
```yaml
vehicle:
  id: "VH-001"  # 一意の車両ID
  model: "MultiGo-Outdoor-v1"

tvm:
  server_url: "https://tvm.example.com"
  api_key_file: "/etc/multigo/api_key.secret"
  websocket_url: "wss://tvm.example.com/ws"
  heartbeat_interval: 1.0  # 秒

navigation:
  max_velocity: 1.5  # m/s
  max_acceleration: 0.5  # m/s²

docking:
  aruco_marker_id: 42
  target_distance: 0.05  # メートル（±5mm精度）

safety:
  estop_timeout: 0.1  # 秒（100ms）
  obstacle_detection_distance: 1.0  # メートル

logging:
  level: "INFO"  # DEBUG、INFO、WARNING、ERROR
  file: "/var/log/multigo/vehicle.log"
  max_size_mb: 100
  rotation_count: 5
```

### 5.2 シークレット管理

**APIキーは別途保存:**
```bash
# シークレットファイルを作成
sudo touch /etc/multigo/api_key.secret
sudo chmod 600 /etc/multigo/api_key.secret
sudo chown multigo:multigo /etc/multigo/api_key.secret
echo "<API_KEY_FROM_TVM>" | sudo tee /etc/multigo/api_key.secret
```

**設定で参照:**
```yaml
tvm:
  api_key_file: "/etc/multigo/api_key.secret"
```

**別ファイルにする理由:**
- 設定ファイルはgitにコミット可能（シークレットなし）
- シークレットファイルはより厳格な権限（600）
- シークレットのローテーションが容易

---

## 6. 更新メカニズム

### 6.1 手動更新（パイロット）

**パイロットでは、更新はSSH経由で手動:**

```bash
# ステップ1: 車両にSSH
ssh multigo@<vehicle-ip>

# ステップ2: サービス停止
sudo systemctl stop multigo-vehicle

# ステップ3: コード更新
cd /home/multigo/multigo_ws/src/multigo_navigation
git pull origin main

# ステップ4: 再ビルド
cd /home/multigo/multigo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# ステップ5: サービス再起動
sudo systemctl start multigo-vehicle

# ステップ6: 検証
sudo systemctl status multigo-vehicle
journalctl -u multigo-vehicle -n 50  # 最近のログをチェック
```

### 6.2 将来: OTA更新（パイロット後）

**生産向け計画（パイロットではない）:**
- TVMサーバーが更新パッケージをプッシュ
- 車両がバックグラウンドで更新をダウンロード
- メンテナンスウィンドウ中に更新を適用
- 更新失敗時のロールバック機能

**パイロットで実装しない理由:**
- 複雑さが増す
- リモートで車両をブリックするリスク
- 1-2台の車両では手動更新が許容可能

---

## 7. 監視とログ

### 7.1 ローカルログ

**システムログ（systemdジャーナル）:**
```bash
# 車両サービスログを表示
journalctl -u multigo-vehicle -f

# 最近のエラーを表示
journalctl -u multigo-vehicle -p err -n 100

# ブート以降のログを表示
journalctl -u multigo-vehicle --boot
```

**アプリケーションログ:**
- **場所:** `/var/log/multigo/`
- **ファイル:**
  - `vehicle.log` - メインアプリケーションログ
  - `navigation.log` - ナビゲーションサブシステム
  - `safety.log` - 安全モニター（緊急停止イベント、衝突）
  - `tvm_client.log` - TVM API通信

**ログローテーション:**
```bash
# /etc/logrotate.d/multigo
/var/log/multigo/*.log {
    daily
    rotate 7
    compress
    delaycompress
    missingok
    notifempty
    create 0644 multigo multigo
}
```

### 7.2 リモート監視（TVM統合）

**車両がTVMにテレメトリを送信:**
- **場所:** REST API経由で送信（`POST /api/v1/telemetry`）
- **頻度:** 1 Hz
- **データ:**
  - 車両ステータス（IDLE、NAVIGATING、DOCKED、ERROR）
  - バッテリーレベル
  - 現在位置（x、y、方位）
  - アクティブエラー
  - CPU/RAM使用率

**TVMに送信されるログ:**
- **クリティカルエラーのみ**（すべてのログではない）
- `POST /api/v1/errors`経由で送信
- TVMは集中データベースに保存

**実装:** `tvm_client`パッケージを参照

---

## 8. デプロイメントチェックリスト

### 8.1 デプロイメント前

- [ ] ハードウェアプラットフォーム組み立て済み（土屋の責任）
- [ ] Ubuntu 22.04ブータブル USB準備済み
- [ ] ネットワーク設定計画済み（WiFi SSID、パスワード）
- [ ] 車両ID割り当て済み（例: VH-001、VH-002）
- [ ] TVMサーバーURLとAPIキー取得済み（Unnoから）

### 8.2 デプロイメント手順

- [ ] 車両計算ユニットにUbuntu 22.04をインストール
- [ ] システム依存関係をインストール（ROS 2、ビルドツール）
- [ ] 車両ソフトウェアリポジトリをクローン
- [ ] 車両ソフトウェアワークスペースをビルド
- [ ] 設定ファイルをデプロイ（`/etc/multigo/config.yaml`）
- [ ] APIキーをデプロイ（`/etc/multigo/api_key.secret`）
- [ ] Systemdサービスをインストール
- [ ] サービスを有効化して開始
- [ ] サービスが実行中か確認（`systemctl status`）
- [ ] ROS トピック確認（`ros2 topic list`）
- [ ] TVM接続確認（認証成功のログをチェック）

### 8.3 デプロイメント後

- [ ] 基本動作テスト（前後に運転）
- [ ] 緊急停止テスト（ハードウェアとソフトウェア）
- [ ] TVMテレメトリテスト（データがTVMダッシュボードに表示されるか確認）
- [ ] ナビゲーションテスト（シンプルなウェイポイントナビゲーション）
- [ ] ドッキングテスト（ドッキングステーション利用可能な場合）
- [ ] 車両固有の設定を文書化
- [ ] 車両インベントリに追加（スプレッドシートまたはTVMデータベース）

---

## 9. トラブルシューティング

### 9.1 一般的な問題

**問題: サービス起動失敗**
```bash
# サービスステータスをチェック
sudo systemctl status multigo-vehicle

# ログをチェック
journalctl -u multigo-vehicle -n 100

# 一般的な原因:
# - 依存関係不足（rosdep install実行）
# - ROSセットアップがソースされていない（サービスファイルのExecStartをチェック）
# - 権限問題（サービスは'multigo'ユーザーとして実行すべき）
```

**問題: TVMサーバーに接続できない**
```bash
# ネットワーク接続性をチェック
ping tvm.example.com

# HTTPS接続をチェック
curl https://tvm.example.com/api/v1/health

# APIキーをチェック
cat /etc/multigo/api_key.secret  # 空でないか確認

# 認証エラーのログをチェック
journalctl -u multigo-vehicle | grep "TVM"
```

**問題: ROSトピックがパブリッシュされない**
```bash
# ROSノードが実行中かチェック
ros2 node list

# 特定のトピックをチェック
ros2 topic echo /sensors/lidar/points

# ROSドメインIDをチェック
echo $ROS_DOMAIN_ID  # 0であるべき

# サービス再起動
sudo systemctl restart multigo-vehicle
```

---

## 10. 比較: エンタープライズ vs パイロット

**エンタープライズバージョンから削除:**

### インフラストラクチャ
- ❌ マルチリージョンAWSデプロイメント（us-east-1、us-west-2）
- ❌ Kubernetesオーケストレーション（EKS）
- ❌ ロードバランシング（ALB、NLB）
- ❌ 自動スケーリンググループ
- ❌ VPC、サブネット、セキュリティグループ
- **理由:** TVMサーバーはUnnoの責任、車両は単一計算ユニット

### CI/CD
- ❌ 自動CI/CDパイプライン（GitHub Actionsでデプロイメント）
- ❌ ブルーグリーンデプロイメント
- ❌ カナリアリリース
- ❌ 自動ロールバック
- **理由:** パイロットは手動デプロイメント、後でCI/CD追加

### 監視
- ❌ Prometheus + Grafana
- ❌ ELKスタック（Elasticsearch、Logstash、Kibana）
- ❌ CloudWatchダッシュボード
- ❌ PagerDutyアラート
- **理由:** パイロットはjournalctl + TVMテレメトリ、パイロット後に高度な監視

### 災害復旧
- ❌ セカンダリリージョンのホットスタンバイデータベース
- ❌ 自動フェイルオーバー
- ❌ RTO/RPO要件
- ❌ バックアップ/リストアプロシージャ
- **理由:** パイロットには不要、サーバーDRはUnnoの責任

### 設定管理
- ❌ AWS Systems Manager Parameter Store
- ❌ HashiCorp Vault
- ❌ 保存時の暗号化設定
- **理由:** パイロットはファイル権限付きシンプルなYAMLファイル

---

## まとめ

**パイロット向け簡略化デプロイメント:**
- ✅ 車両への手動Ubuntuインストール
- ✅ aptパッケージからのROS 2 Humble
- ✅ Git clone + colconビルド
- ✅ 自動起動用Systemdサービス
- ✅ シンプルなYAML設定
- ✅ SSH経由の手動更新
- ✅ ローカルログ + TVMテレメトリ

**合計労力:** 初期デプロイメント1台あたり約4時間

**次のステップ:**
1. 開発環境用に`DEVELOPMENT_SETUP_GUIDE.md`に従う
2. 最初に開発マシンでデプロイメントをテスト
3. ソフトウェアテスト完了後に車両にデプロイ
4. 車両固有の設定を文書化

---

**文書ステータス:** ✅ アクティブ（パイロットレベル）
**次のレビュー:** 最初の車両デプロイメント後
**承認者:** Senior（2025年12月19日）

---

**文書終了**
