# ドッキングサブシステムアーキテクチャ

**文書ID:** ARCH-DOCK-001
**バージョン:** 1.0
**日付:** 2025年12月15日
**ステータス:** ドラフト

---

## 1. 概要

**ドッキングサブシステム**は、ArUcoマーカーを使用したビジュアルサーボによる車椅子ベースへの精密アタッチメントを実現します。これは車椅子搬送ユースケース専用に設計された**新規サブシステム**（ParcelPalには存在しない）です。

**主要機能:**
- 2段階ドッキング（粗ナビゲーション+精密ビジュアルサーボ）
- ArUcoマーカーベース精密測位（屋外±2-5mm）
- デュアルカメラ融合による信頼性
- ステートマシン駆動ドッキングシーケンス
- 安全監視と故障回復
- 屋外動作能力（小雨、変動照明）

**ドッキング目標:** ±2-5mm位置精度、屋外成功率>95%

---

## 2. システムアーキテクチャ

### 2.1 コンポーネント図

```
┌─────────────────────────────────────────────────────────────────┐
│                    ドッキングサブシステム                         │
│                                                                 │
│  ┌──────────────────┐      ┌──────────────────────────────┐   │
│  │  Nav2            │─────▶│  ドッキングコーディネーター    │   │
│  │  (粗アプローチ    │      │  (ステートマシン)             │   │
│  │   ゾーンへ)       │      │                              │   │
│  └──────────────────┘      │  - IDLE → APPROACH → SEARCH  │   │
│                             │  - SERVOING → DOCKED         │   │
│                             │  - 故障処理                  │   │
│                             └──────────┬───────────────────┘   │
│                                        │                        │
│                                        │ マーカー姿勢           │
│                                        ↓                        │
│                      ┌─────────────────────────────┐           │
│                      │   ArUco検出器               │           │
│                      │   (OpenCV cv::aruco)        │           │
│                      │                             │           │
│  ┌─────────────┐    │  - デュアルカメラ融合       │           │
│  │ カメラ1     │───▶│  - 姿勢推定 (PnP)           │           │
│  │ (前方)      │    │  - 外れ値除去               │           │
│  └─────────────┘    └─────────────┬───────────────┘           │
│                                    │                            │
│  ┌─────────────┐                   │ マーカー姿勢               │
│  │ カメラ2     │───────────────────┘                            │
│  │ (前方代替)  │                                                │
│  └─────────────┘                                                │
│                                        │                        │
│                                        │ 目標速度               │
│                                        ↓                        │
│                      ┌─────────────────────────────┐           │
│                      │   ビジュアルサーボ          │           │
│                      │   コントローラー (PBVS)     │           │
│                      │                             │           │
│                      │  - 位置ベースVS             │           │
│                      │  - PID制御 (x,y,θ)          │           │
│                      │  - 速度制限                 │           │
│                      └──────────┬──────────────────┘           │
│                                 │                               │
│                                 │ /cmd_vel                      │
│                                 ↓                               │
│                      ┌─────────────────────────────┐           │
│                      │   スワーブドライブコントローラー│         │
│                      │   (全方向移動)              │           │
│                      └─────────────────────────────┘           │
└─────────────────────────────────────────────────────────────────┘
```

**主要コンポーネント:**
1. **ドッキングコーディネーター**: ドッキングシーケンスを管理するステートマシン
2. **ArUco検出器**: デュアルカメラからのマーカー検出と姿勢推定
3. **ビジュアルサーボコントローラー**: PID制御による位置ベースビジュアルサーボ（PBVS）
4. **スワーブドライブコントローラー**: 精密モーションコマンド実行

---

## 3. ROS 2ノードアーキテクチャ

### 3.1 ドッキングコーディネーターノード

**ノード名:** `docking_coordinator`
**言語:** C++
**ライフサイクル:** マネージドライフサイクルノード

**購読トピック:**
- `/aruco_poses` (geometry_msgs/PoseArray): 検出されたArUcoマーカー姿勢
- `/odom` (nav_msgs/Odometry): 現在のロボットオドメトリ
- `/camera_1/image_raw` (sensor_msgs/Image): 前方カメラ画像
- `/camera_2/image_raw` (sensor_msgs/Image): 前方代替カメラ画像

**配信トピック:**
- `/docking/state` (std_msgs/String): 現在のドッキング状態
- `/docking/status` (custom_msgs/DockingStatus): 詳細ステータスと診断
- `/cmd_vel` (geometry_msgs/Twist): ビジュアルサーボ中の速度コマンド

**アクションサーバー:**
- `/dock` (custom_msgs/action/Dock): ドッキングシーケンス実行
- `/undock` (custom_msgs/action/Undock): アンドッキングシーケンス実行

**アクションクライアント:**
- `/navigate_to_pose` (nav2_msgs/action/NavigateToPose): 粗アプローチナビゲーション

**パラメータ:**
```yaml
# アプローチゾーン設定
approach_distance: 1.5          # ドッキングステーションからのメートル
approach_tolerance: 0.3         # メートル（Nav2 xy_goal_tolerance）

# ビジュアルサーボ設定
servoing_gains:
  kp_x: 0.8                     # x方向比例ゲイン
  kp_y: 0.8                     # y方向比例ゲイン
  kp_theta: 1.2                 # θ方向比例ゲイン
  ki_x: 0.1                     # x方向積分ゲイン
  ki_y: 0.1                     # y方向積分ゲイン

servoing_limits:
  max_linear_vel: 0.2           # サーボ中のm/s
  max_angular_vel: 0.3          # サーボ中のrad/s

# ドッキング成功基準
docking_tolerance:
  position_xy: 0.005            # メートル（±5mm）
  orientation: 0.087            # ラジアン（±5°）
  stable_duration: 2.0          # 秒

# 故障閾値
max_servoing_time: 60.0         # 秒
marker_lost_timeout: 5.0        # 秒
```

---

### 3.2 ArUco検出器ノード

**ノード名:** `aruco_detector`
**言語:** C++
**ライフサイクル:** 標準ノード

**購読トピック:**
- `/camera_1/image_raw` (sensor_msgs/Image): 前方カメラ画像
- `/camera_1/camera_info` (sensor_msgs/CameraInfo): カメラキャリブレーション
- `/camera_2/image_raw` (sensor_msgs/Image): 前方代替カメラ画像
- `/camera_2/camera_info` (sensor_msgs/CameraInfo): カメラキャリブレーション

**配信トピック:**
- `/aruco_poses` (geometry_msgs/PoseArray): ロボットフレームでの検出マーカー姿勢
- `/aruco_debug` (sensor_msgs/Image): マーカーオーバーレイ付きデバッグ画像
- `/aruco_detections` (custom_msgs/ArucoDetectionArray): カメラごとの検出

**パラメータ:**
```yaml
# ArUco設定
dictionary: "DICT_4X4_50"       # ArUco辞書
marker_size: 0.15               # メートル（15cm物理サイズ）
detection_rate: 10              # Hz

# デュアルカメラ融合
fusion_mode: "weighted_average" # または "best_camera"
min_detection_confidence: 0.7   # 低信頼度検出を拒否
max_reprojection_error: 2.0     # ピクセル

# カメラ内部パラメータ（camera_infoから読み込み）
camera_1_frame: "camera_1_optical_frame"
camera_2_frame: "camera_2_optical_frame"
base_frame: "base_link"
```

---

### 3.3 ビジュアルサーボコントローラーノード

**ノード名:** `visual_servoing_controller`
**言語:** C++
**ライフサイクル:** 標準ノード

**購読トピック:**
- `/aruco_poses` (geometry_msgs/PoseArray): 目標マーカー姿勢
- `/docking/state` (std_msgs/String): ドッキング状態（制御有効/無効）

**配信トピック:**
- `/cmd_vel` (geometry_msgs/Twist): スワーブドライブへの速度コマンド

**パラメータ:**
```yaml
# 制御方法
control_method: "PBVS"          # 位置ベースビジュアルサーボ

# PIDゲイン（屋外動作向けにチューニング）
pid_gains:
  kp: [0.8, 0.8, 1.2]           # [x, y, θ]
  ki: [0.1, 0.1, 0.0]           # [x, y, θ]
  kd: [0.05, 0.05, 0.1]         # [x, y, θ]

# 速度制限
max_vel:
  linear_x: 0.2                 # m/s
  linear_y: 0.2                 # m/s
  angular_z: 0.3                # rad/s

# デッドバンド（振動回避）
deadband:
  position_xy: 0.002            # メートル（±2mm）
  orientation: 0.017            # ラジアン（±1°）
```

---

## 4. ドッキングステートマシン

### 4.1 状態遷移図

```
      ┌─────────────┐
      │    IDLE     │
      └──────┬──────┘
             │ /dockアクション呼び出し
             ↓
      ┌─────────────┐
      │  APPROACH   │ ← Nav2ナビゲーションでアプローチゾーンへ（±30cm）
      └──────┬──────┘
             │ アプローチゾーン到達
             ↓
      ┌─────────────┐
      │   SEARCH    │ ← ゆっくり回転、ArUcoマーカーをスキャン
      └──────┬──────┘
             │ マーカー検出
             ↓
      ┌─────────────┐
      │  SERVOING   │ ← 目標姿勢へビジュアルサーボ（±5mm）
      └──────┬──────┘
             │ 位置安定（2秒間±5mm）
             ↓
      ┌─────────────┐
      │   DOCKED    │ ← 車椅子へロックコマンド送信
      └──────┬──────┘
             │ /undockアクション呼び出し
             ↓
      ┌─────────────┐
      │ UNDOCKING   │ ← アンロック、0.5m後退
      └──────┬──────┘
             │ アンドッキング完了
             ↓
      ┌─────────────┐
      │    IDLE     │
      └─────────────┘

故障遷移（任意の状態から）:
- マーカー消失>5秒 → SEARCH
- タイムアウト → FAILED → IDLE
- 緊急停止 → IDLE
```

---

### 4.2 状態実装

**IDLE:**
- アクティブ制御なし
- `/dock`アクションゴール待機
- 車椅子ロック解除

**APPROACH:**
- アプローチ姿勢でNav2 `/navigate_to_pose`アクション呼び出し
- アプローチ姿勢 = ドッキングステーション姿勢 + オフセット（1.5m前方）
- 許容範囲: ±30cm（Nav2 xy_goal_tolerance）
- **遷移:** アプローチゾーン到達 → SEARCH

**SEARCH:**
- ゆっくり回転（0.2 rad/s）してArUcoマーカーをスキャン
- `/aruco_poses`トピック購読
- **遷移:** 3フレーム連続でマーカー検出 → SERVOING
- **タイムアウト:** 30秒 → FAILED

**SERVOING:**
- ビジュアルサーボコントローラー有効化
- マーカー姿勢誤差から目標速度を計算
- 10 Hzで`/cmd_vel`配信
- 位置誤差監視: `error = sqrt((x-x_target)^2 + (y-y_target)^2)`
- **遷移:** 誤差<5mmかつ2秒間安定 → DOCKED
- **マーカー消失:** >5秒 → SEARCH
- **タイムアウト:** 60秒 → FAILED

**DOCKED:**
- すべての動作停止（`/cmd_vel` = 0）
- 車椅子へロックコマンド送信（将来: CAN/シリアルインターフェース）
- `/dock`アクションで成功フィードバック配信
- **遷移:** `/undock`アクション呼び出し → UNDOCKING

**UNDOCKING:**
- 車椅子アンロック（将来: CAN/シリアルインターフェース）
- 0.5m後退（2.5秒間-0.2 m/sのオープンループモーション）
- **遷移:** モーション完了 → IDLE

**FAILED:**
- すべての動作停止
- `/dock`アクションで故障フィードバック配信
- 故障理由をログ記録（マーカー消失、タイムアウト、安全違反）
- **遷移:** 自動 → IDLE

---

## 5. ArUco検出パイプライン

### 5.1 マーカー検出（OpenCV）

**アルゴリズム:**
```cpp
void detectMarkers(const cv::Mat& image) {
    // 1. グレースケールに変換
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // 2. 適応的ヒストグラム均等化（屋外照明ロバスト性）
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(gray, gray);

    // 3. ArUcoマーカー検出
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::Ptr<cv::aruco::Dictionary> dictionary = 
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::DetectorParameters> params = 
        cv::aruco::DetectorParameters::create();
    
    // 屋外検出向けにチューニング
    params->adaptiveThreshWinSizeMin = 3;
    params->adaptiveThreshWinSizeMax = 23;
    params->adaptiveThreshWinSizeStep = 10;
    
    cv::aruco::detectMarkers(gray, dictionary, marker_corners, marker_ids, params);

    // 4. コーナー品質の低い検出を拒否
    filterLowQualityDetections(marker_corners, marker_ids);
}
```

---

### 5.2 姿勢推定（PnP）

**アルゴリズム:**
```cpp
geometry_msgs::msg::Pose estimateMarkerPose(
    const std::vector<cv::Point2f>& corners,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    double marker_size) {

    // 3Dマーカーコーナー（マーカーフレーム、z=0平面）
    std::vector<cv::Point3f> object_points = {
        {-marker_size/2,  marker_size/2, 0},  // 左上
        { marker_size/2,  marker_size/2, 0},  // 右上
        { marker_size/2, -marker_size/2, 0},  // 右下
        {-marker_size/2, -marker_size/2, 0}   // 左下
    };

    // PnP（Perspective-n-Point）解法
    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(
        object_points,
        corners,
        camera_matrix,
        dist_coeffs,
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_IPPE_SQUARE  // 平面マーカーに最適
    );

    if (!success) {
        return {};  // 無効な姿勢
    }

    // 再投影誤差をチェック
    std::vector<cv::Point2f> reprojected;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, reprojected);
    double error = computeReprojectionError(corners, reprojected);
    if (error > max_reprojection_error) {
        return {};  // 姿勢品質不良
    }

    // geometry_msgs::Poseに変換
    return toPose(rvec, tvec);
}
```

---

### 5.3 デュアルカメラ融合

**加重平均融合:**
```cpp
geometry_msgs::msg::Pose fuseCameraPoses(
    const geometry_msgs::msg::Pose& pose_cam1,
    const geometry_msgs::msg::Pose& pose_cam2,
    double confidence_cam1,
    double confidence_cam2) {

    // 信頼度正規化
    double total_confidence = confidence_cam1 + confidence_cam2;
    double w1 = confidence_cam1 / total_confidence;
    double w2 = confidence_cam2 / total_confidence;

    // 位置の加重平均
    geometry_msgs::msg::Pose fused;
    fused.position.x = w1 * pose_cam1.position.x + w2 * pose_cam2.position.x;
    fused.position.y = w1 * pose_cam1.position.y + w2 * pose_cam2.position.y;
    fused.position.z = w1 * pose_cam1.position.z + w2 * pose_cam2.position.z;

    // 向きのSLERP（球面線形補間）
    fused.orientation = slerp(
        pose_cam1.orientation,
        pose_cam2.orientation,
        w2  // 補間係数
    );

    return fused;
}

// 信頼度メトリック: 再投影誤差とマーカーサイズの関数
double computeConfidence(double reprojection_error, double marker_area_pixels) {
    // 誤差低い → 信頼度高い
    double error_factor = 1.0 / (1.0 + reprojection_error);
    
    // マーカー大きい → 信頼度高い（カメラに近い）
    double size_factor = std::clamp(marker_area_pixels / 10000.0, 0.1, 1.0);
    
    return error_factor * size_factor;
}
```

---

## 6. ビジュアルサーボ制御

### 6.1 位置ベースビジュアルサーボ（PBVS）

**制御則:**
```cpp
geometry_msgs::msg::Twist computeServoingCommand(
    const geometry_msgs::msg::Pose& current_pose,
    const geometry_msgs::msg::Pose& target_pose) {

    // 1. ロボットフレームでの姿勢誤差を計算
    Eigen::Vector3d position_error;
    position_error.x() = target_pose.position.x - current_pose.position.x;
    position_error.y() = target_pose.position.y - current_pose.position.y;
    position_error.z() = 0.0;  // zを無視（平面運動を仮定）

    // 2. 向き誤差を計算（四元数間の角度）
    tf2::Quaternion q_current, q_target;
    tf2::fromMsg(current_pose.orientation, q_current);
    tf2::fromMsg(target_pose.orientation, q_target);
    tf2::Quaternion q_error = q_target * q_current.inverse();
    double theta_error = q_error.getAngle();

    // 3. PID制御
    Eigen::Vector3d integral_error;  // 時間経過で累積
    Eigen::Vector3d derivative_error;  // 変化率

    // 比例項
    double vx = kp_x * position_error.x() + ki_x * integral_error.x();
    double vy = kp_y * position_error.y() + ki_y * integral_error.y();
    double omega = kp_theta * theta_error;

    // 4. デッドバンド適用（目標近くでの振動回避）
    if (position_error.norm() < deadband_position) {
        vx = 0.0;
        vy = 0.0;
    }
    if (std::abs(theta_error) < deadband_orientation) {
        omega = 0.0;
    }

    // 5. 速度制限
    double v_mag = std::sqrt(vx*vx + vy*vy);
    if (v_mag > max_linear_vel) {
        vx *= max_linear_vel / v_mag;
        vy *= max_linear_vel / v_mag;
    }
    omega = std::clamp(omega, -max_angular_vel, max_angular_vel);

    // 6. Twistメッセージ構築
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.angular.z = omega;

    return cmd_vel;
}
```

---

### 6.2 安定性検証

**ドッキング完了チェック:**
```cpp
bool isDockingStable(
    const std::vector<geometry_msgs::msg::Pose>& pose_history,
    double duration_threshold,
    double position_tolerance,
    double orientation_tolerance) {

    // 少なくともNサンプル必要（例: 20サンプル @ 10 Hz = 2秒）
    int required_samples = static_cast<int>(duration_threshold * control_rate);
    if (pose_history.size() < required_samples) {
        return false;
    }

    // 最後のNサンプルがすべて許容範囲内かチェック
    geometry_msgs::msg::Pose target = pose_history.back();
    for (int i = pose_history.size() - required_samples; i < pose_history.size(); i++) {
        double position_error = computePositionError(pose_history[i], target);
        double orientation_error = computeOrientationError(pose_history[i], target);

        if (position_error > position_tolerance ||
            orientation_error > orientation_tolerance) {
            return false;  // 不安定
        }
    }

    return true;  // 必要期間安定
}
```

---

## 7. 安全機能

### 7.1 ドッキング中の衝突回避

**アプローチ:**
- APPROACHとSEARCH状態では3D LiDARを使用（通常のNav2コストマップ）
- SERVOING中は障害物回避を**無効化**（ビジュアルサーボパスを信頼）
- 車椅子ベースの存在を監視（ドッキング位置での予想障害物）

**実装:**
```cpp
void updateCostmapConfiguration(DockingState state) {
    if (state == DockingState::SERVOING) {
        // サーボ中はコストマップインフレーション無効化
        costmap_client->setParameter("inflation_radius", 0.0);
    } else {
        // アプローチ/サーチ中は通常のインフレーション
        costmap_client->setParameter("inflation_radius", 0.3);
    }
}
```

---

### 7.2 故障検出と回復

**故障モード:**
1. **サーボ中のマーカー消失:** SEARCH状態に戻る、マーカー再取得
2. **SERVOINGでタイムアウト:** 60秒後、FAILED宣言、IDLEに戻る
3. **不安定マーカー姿勢:** フレーム間で姿勢が>10cmジャンプしたら外れ値を拒否
4. **カメラ故障:** 両カメラ故障時、ドッキング中止
5. **アプローチ中の衝突:** Nav2が障害物検出、ドッキング中止

**回復戦略:**
```cpp
void handleFailure(FailureReason reason) {
    switch (reason) {
        case FailureReason::MARKER_LOST:
            if (marker_lost_duration < marker_lost_timeout) {
                // 短時間消失: 最後の既知姿勢でサーボ継続
                continue_with_last_pose = true;
            } else {
                // 長時間消失: SEARCHに戻る
                transitionTo(DockingState::SEARCH);
            }
            break;

        case FailureReason::TIMEOUT:
            // ドッキング中止、IDLEに戻る
            transitionTo(DockingState::FAILED);
            publishActionResult(false, "ドッキングタイムアウト");
            break;

        case FailureReason::CAMERA_FAILURE:
            // 致命的故障、回復不可
            transitionTo(DockingState::FAILED);
            publishActionResult(false, "カメラ故障");
            break;

        case FailureReason::COLLISION:
            // アプローチ中にNav2が衝突検出
            transitionTo(DockingState::FAILED);
            publishActionResult(false, "アプローチ中の衝突");
            break;
    }
}
```

---

## 8. テスト戦略

### 8.1 ユニットテスト
- ArUco検出精度（既知マーカー→期待姿勢）
- PnP姿勢推定の正確性
- デュアルカメラ融合アルゴリズム
- ビジュアルサーボ制御則（誤差→速度）
- ステートマシン遷移

### 8.2 統合テスト
- 記録カメラ画像でのArUco検出器ノード
- シミュレーションマーカー姿勢でのビジュアルサーボコントローラー
- Gazeboシミュレーションでの完全ドッキングシーケンス
- 故障回復シナリオ

### 8.3 フィールドテスト（屋外）
- ドッキング成功率（目標: >95%）
- ドッキング精度（目標: ±2-5mm）
- 照明へのロバスト性（曇天、晴天、薄暮）
- 小雨へのロバスト性（IP65+カメラ）
- マーカー検出範囲（0.5m〜3m）
- ドッキング時間（目標: アプローチゾーンから<30秒）

---

## 9. 性能目標

| メトリック | 目標 | 検証 |
|--------|--------|------------|
| ドッキング精度（屋外） | ±2-5mm (x, y) | モーションキャプチャまたは手動測定 |
| ドッキング精度（屋内） | ±1mm (x, y) | モーションキャプチャまたは手動測定 |
| ドッキング成功率 | >95% | フィールドテスト（100試行） |
| マーカー検出範囲 | 0.5m - 3m | 検出テスト |
| マーカー検出レート | 10 Hz | タイムスタンプ解析 |
| ビジュアルサーボ制御レート | 10 Hz | タイムスタンプ解析 |
| ドッキング時間（APPROACH→DOCKED） | <30秒 | フィールドテスト |
| マーカー消失回復時間 | <5秒 | 故障回復テスト |

---

## 10. 実装フェーズ

### フェーズ1: ArUco検出（スプリント1-2）
- ✅ シングルカメラArUco検出
- ✅ 姿勢推定（PnP）
- ✅ デュアルカメラ融合
- ✅ ユニットテスト

### フェーズ2: ビジュアルサーボ（スプリント3-4）
- ✅ PBVSコントローラー実装
- ✅ PIDチューニング（シミュレーション）
- ✅ 安定性検証
- ✅ スワーブドライブとの統合

### フェーズ3: ステートマシン（スプリント5-6）
- ✅ ドッキングコーディネーターノード
- ✅ 状態遷移
- ✅ Nav2統合（アプローチフェーズ）
- ✅ 故障処理

### フェーズ4: フィールド検証（スプリント7-8）
- ✅ 屋外ドッキングテスト
- ✅ 精度チューニング
- ✅ ロバスト性テスト（照明、天候）
- ✅ 成功率検証（100試行）

---

**文書ステータス:** ドラフト
**実装ステータス:** 未開始
**承認必要:** ドッキングエンジニア、ビジョンエンジニア、ナビゲーションリード、安全エンジニア
