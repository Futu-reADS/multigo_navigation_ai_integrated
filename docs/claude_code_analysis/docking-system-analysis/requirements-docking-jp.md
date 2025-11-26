# ドッキングシステム - 要求仕様書

**ブランチ:** feature/localization | **アナリスト:** Claude AI | **日付:** Nov 25, 2025

---

## ドキュメントの目的

このドキュメントは、既存のソースコードを分析して導き出されたMulti Go自律ドッキングシステムの**包括的な要求仕様分析**を提供します。各要求仕様には**現在の実装ステータス**が記載され、対処する必要がある**ギャップ**が特定されています。

---

## ステータス凡例

| 記号 | ステータス | 説明 |
|--------|--------|-------------|
| ✅ | **COMPLETE** | 完全に実装され正しく動作している |
| 🟡 | **PARTIAL** | 開始されているが不完全または問題がある |
| ❌ | **NOT IMPLEMENTED** | 必要だが完全に欠落している |
| 🐛 | **BUGGY** | 実装されているが重大なバグがある |
| ❓ | **UNCLEAR** | 要求仕様が満たされているか不明 |

---

## 1. 機能要求仕様

### 1.1 視覚マーカー検出

#### FR-1.1.1: ArUcoマーカー認識
**ステータス:** ✅ **COMPLETE**

**要求仕様:** システムは、キャリブレーション済みカメラを使用してDICT_6X6_250辞書からArUcoマーカーを検出する必要があります。

**実装:**
- **ファイル:** `aruco_detect.cpp:106-243`
- **メソッド:** OpenCV `cv::aruco::detectMarkers()` および `estimatePoseSingleMarkers()`
- **辞書:** `cv::aruco::DICT_6X6_250`
- **カメラ:** キャリブレーション済みの内部パラメータと歪み係数

**エビデンス:**
```cpp
cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
cv::aruco::estimatePoseSingleMarkers(markerCorners, marker_width,
                                     camera_matrix, dist_coeffs, rvecs, tvecs);
```

**検証:** ✓ テスト済み、動作中

---

#### FR-1.1.2: デュアルカメラサポート
**ステータス:** ✅ **COMPLETE**

**要求仕様:** システムは、左右のカメラからの同時検出をサポートする必要があります。

**実装:**
- **ファイル:** `nav_docking.cpp:89-93`
- 2つの個別のArUco検出器インスタンス
- トピック: `aruco_detect/markers_left`, `aruco_detect/markers_right`

**エビデンス:**
```cpp
pose_array_left_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
    marker_topic_left, 10, ...);
pose_array_right_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
    marker_topic_right, 10, ...);
```

**検証:** ✓ 実装済み

---

#### FR-1.1.3: マーカー姿勢推定
**ステータス:** ✅ **COMPLETE**

**要求仕様:** システムは、カメラフレームに対する検出されたマーカーの6自由度姿勢（位置+方向）を推定する必要があります。

**実装:**
- **ファイル:** `aruco_detect.cpp:146-184`
- 位置: `tvecs[i]` をROS座標系に変換
- 方向: ロドリゲス回転ベクトル → 回転行列 → クォータニオン

**エビデンス:**
```cpp
pose.position.x = tvecs[i][2];
pose.position.y = -tvecs[i][0];
pose.position.z = tvecs[i][1];
// Quaternion conversion with coordinate transform
```

**検証:** ✓ 正しく動作中

---

#### FR-1.1.4: マーカーIDフィルタリング
**ステータス:** ✅ **COMPLETE**

**要求仕様:** システムは、検出されたマーカーを目的のIDでフィルタリングし、それ以外を無視する必要があります。

**実装:**
- **ファイル:** `aruco_detect.cpp:139-143`
- パラメータ: `desired_aruco_marker_id`
- 一致するマーカーのみを公開

**エビデンス:**
```cpp
if (markerIds[i] == desired_aruco_marker_id)
{
    // Process marker
}
```

**検証:** ✓ テスト済み

---

### 1.2 座標変換

#### FR-1.2.1: カメラからベースリンクへの変換
**ステータス:** ✅ **COMPLETE**

**要求仕様:** システムは、TF2を使用してマーカー姿勢をカメラフレームからロボットbase_linkフレームに変換する必要があります。

**実装:**
- **ファイル:** `nav_docking.cpp:248, 307`
- TF2ルックアップ、2秒タイムアウト
- 適切なエラーハンドリング

**エビデンス:**
```cpp
cameraToBase_link = tf_buffer->lookupTransform(base_frame, camera_left_frame,
                                               tf2::TimePointZero, tf2::durationFromSec(2));
```

**検証:** ✓ 動作中

---

#### FR-1.2.2: カメラからマップへの変換
**ステータス:** ✅ **COMPLETE**

**要求仕様:** システムは、ナビゲーションゴールのためにマーカー姿勢をカメラフレームからマップフレームに変換する必要があります。

**実装:**
- **ファイル:** `nav_goal.cpp:172, 257`
- 接近フェーズのゴール生成に使用

**エビデンス:**
```cpp
cameraToMap = tf_buffer->lookupTransform(map_frame, camera_front_left_frame,
                                        tf2::TimePointZero, tf2::durationFromSec(2));
```

**検証:** ✓ 動作中

---

#### FR-1.2.3: 変換例外ハンドリング
**ステータス:** ✅ **COMPLETE**

**要求仕様:** システムは、クラッシュすることなくTF2変換例外を適切に処理する必要があります。

**実装:**
- **ファイル:** `nav_docking.cpp:290-296`, `nav_goal.cpp:238-242`
- Try-catchブロックとログ記録

**エビデンス:**
```cpp
catch (tf2::TransformException &ex)
{
    RCLCPP_WARN(this->get_logger(), "Could not transform...: %s", ex.what());
    return;
}
```

**検証:** ✓ 実装済み

---

### 1.3 ドッキング状態マシン

#### FR-1.3.1: マルチステージドッキングプロセス
**ステータス:** 🟡 **PARTIAL** (正式な状態マシンなし)

**要求仕様:** システムは、ドッキングを段階的に実行する必要があります: 接近 → 整列 → 最終ドッキング → 完了。

**現在の実装:**
- **ステージ3:** `nav_goal` が処理（約5cmまで接近）
- **ステージ4:** `nav_docking.frontMarkerCmdVelPublisher()` が処理（約1cmまで整列）
- **ステージ5:** `nav_docking.dualMarkerCmdVelPublisher()` が処理（約1mmまで精密ドック）

**問題点:**
- ❌ 正式な状態マシンクラスなし
- ❌ 状態遷移はブール値フラグで管理
- ❌ 状態図のドキュメントなし
- ❌ `stage_6_docking_status` は宣言されているが使用されていない（`nav_docking.h:73`）

**ギャップ:** enum状態と明確な遷移ロジックを持つ適切なFSM実装が必要。

**推奨事項:** 状態マシンパターンを実装:
```cpp
enum class DockingState {
    IDLE,
    APPROACHING,     // Stage 3
    ALIGNING,        // Stage 4
    DOCKING,         // Stage 5
    COMPLETE,
    FAILED
};
```

---

#### FR-1.3.2: ステージ遷移条件
**ステータス:** 🟡 **PARTIAL**

**要求仕様:** システムは、明確で測定可能な条件に基づいてステージ間を遷移する必要があります。

**現在の遷移:**
1. **ステージ3 → ステージ4:** `marker_tx < 0.05m` (`nav_goal.cpp:190`)
2. **ステージ4 → ステージ5:** すべてのエラー < `min_error` (0.01m) (`nav_docking.cpp:452-460`)
3. **ステージ5 → 完了:** すべてのエラー < `min_docking_error` (0.001m) 確認あり (`nav_docking.cpp:540-544`)

**問題点:**
- ✅ 遷移は動作中
- ❌ ステージ間の振動を防ぐヒステリシスなし
- ❌ ステージ4はマーカー喪失 > 3秒で自身にリセット可能（`nav_docking.cpp:493`）
- ❌ 遷移タイムアウトなし（無限ループの可能性）

**ギャップ:** ヒステリシスとタイムアウトを追加

---

#### FR-1.3.3: エラー回復とリトライロジック
**ステータス:** ❌ **NOT IMPLEMENTED**

**要求仕様:** システムは、最大リトライ制限と障害報告を含むエラー回復を実装する必要があります。

**現在の状態:**
- ⚠️ ステージ4はマーカー喪失時に静かにリセット
- ⚠️ リトライカウンターなし
- ⚠️ 最大試行回数制限なし
- ⚠️ アクションクライアントへの障害報告なし

**ギャップ:** 包括的なエラー回復が必要:
- 最大制限付きリトライカウンター（例：3回試行）
- ステージごとのタイムアウト（例：60秒）
- エラーコード付きの適切な障害処理
- バックアウトとリトライのオプション

**推奨事項:**
```cpp
int retry_count = 0;
const int MAX_RETRIES = 3;
rclcpp::Time stage_start_time;
const double STAGE_TIMEOUT = 60.0;  // seconds
```

---

### 1.4 マーカー検出戦略

#### FR-1.4.1: デュアルマーカーモード
**ステータス:** 🐛 **BUGGY** (計算エラー)

**要求仕様:** 両方のマーカーが見える場合、システムは精度向上のため両マーカーからの平均位置を使用する必要があります。

**実装:**
- **ファイル:** `nav_docking.cpp:379-396`, `476-514`
- 条件: 両マーカーの遅延 < 0.2秒

**発見されたバグ:**
```cpp
// 現在 (間違い):
double distance = (left_marker_x) + (right_marker_x) / 2;  // ← 括弧が欠落!
// 正しくは:
double distance = (left_marker_x + right_marker_x) / 2;
```

**影響:** 🔴 重大 - 距離計算が不正確

**例:**
```
left_x = 1.0m, right_x = 2.0m
現在: 1.0 + (2.0/2) = 2.0m  ← 間違い!
正しい: (1.0 + 2.0) / 2 = 1.5m
```

**また:**
```cpp
double center = (left_marker_y - -right_marker_y);  // 二重否定で混乱
// 正しくは:
double center = (left_marker_y + right_marker_y) / 2;
```

**必要なアクション:** ⚠️ デプロイ前に即座に修正

---

#### FR-1.4.2: シングルマーカーフォールバック
**ステータス:** ✅ **COMPLETE**

**要求仕様:** 1つのマーカーが失われたか遅延した場合、システムはシングルマーカーモードにフォールバックする必要があります。

**実装:**
- **ファイル:** `nav_docking.cpp:397-425`
- フォールバック条件: `callback_duration >= marker_delay_threshold_sec` (0.2秒)
- 最短遅延のマーカーを選択
- オフセンターマーカーを補償するために横方向オフセット（±0.17m）を適用

**エビデンス:**
```cpp
if (callback_duration_left < callback_duration_right)
    // Use left marker with offset
else
    // Use right marker with offset
```

**検証:** ✓ 正しく動作中

---

#### FR-1.4.3: マーカー遅延しきい値
**ステータス:** ✅ **COMPLETE**

**要求仕様:** マーカーの更新がしきい値より古い場合、システムはシングルマーカーモードに切り替える必要があります。

**実装:**
- **ファイル:** `nav_docking.cpp:69, 427`
- しきい値: 0.2秒
- 最後のマーカー更新からの時間を計算
- 古いデータの使用を防止

**検証:** ✓ 実装済み

---

### 1.5 制御システム

#### FR-1.5.1: X、Y、およびYaw用のPID制御
**ステータス:** 🐛 **BUGGY** (積分項が壊れている)

**要求仕様:** システムは、X（前進）、Y（横方向）、Yaw（回転）軸でのスムーズな速度コマンドにPID制御を使用する必要があります。

**実装:**
- **ファイル:** `nav_docking.cpp:187-219`
- 関数: `calculate(error, prev_error, kp, ki, kd, ...)`
- 各軸に対して個別のPID

**重大なバグ:**
```cpp
// Line 197:
double integral = error * callback_duration;  // ← これは積分ではない!
```

**問題:** これは `error * dt` を計算しており、単にスケーリングされたエラーであり、積分ではありません。
**正しい実装:**
```cpp
// 蓄積が必要:
integral_sum += error * callback_duration;
```

**影響:** 🔴 重大
- Kiゲインが設計通りに機能しない
- 積分ワインドアップ保護なし
- 累積エラー補正なし

**また欠落:**
- ❌ 積分状態変数（呼び出し間で保持されない）
- ❌ アンチワインドアップメカニズム

**必要なアクション:** ⚠️ 積分計算を正しく再実装

**推奨事項:**
```cpp
// In header:
double integral_x = 0.0;
double integral_y = 0.0;
double integral_yaw = 0.0;
const double INTEGRAL_MAX = 1.0;  // Anti-windup limit

// In calculate function:
integral_sum += error * dt;
// Clamp integral
if (integral_sum > INTEGRAL_MAX) integral_sum = INTEGRAL_MAX;
if (integral_sum < -INTEGRAL_MAX) integral_sum = -INTEGRAL_MAX;
double output = kp * error + ki * integral_sum + kd * derivative;
```

---

#### FR-1.5.2: 速度クランピング
**ステータス:** 🟡 **PARTIAL** (問題あり)

**要求仕様:** システムは、出力速度を安全な制限（最大および最小）にクランプする必要があります。

**実装:**
- **ファイル:** `nav_docking.cpp:206-216`
- 最大速度: 0.1 m/s
- 最小速度: 0.005 m/s

**問題:**
```cpp
// Enforce minimum output magnitude
if (std::abs(output) < min_output) {
    output = (output > 0) ? min_output : -min_output;
}
```

**問題:** PIDが0.002 m/s（デッドゾーン内）を計算した場合、0.005 m/sに引き上げられ、min_errorチェックに違反します。これにより、ターゲット付近で振動が発生する可能性があります。

**ギャップ:** 最小出力は error > min_error の場合にのみ適用すべき

**推奨事項:**
```cpp
if (std::abs(error) > min_error && std::abs(output) < min_output) {
    output = (output > 0) ? min_output : -min_output;
}
```

---

#### FR-1.5.3: デッドゾーンハンドリング
**ステータス:** ✅ **COMPLETE**

**要求仕様:** エラーが許容しきい値（デッドゾーン）内にある場合、システムは動作をコマンドしてはいけません。

**実装:**
- **ファイル:** `nav_docking.cpp:192-194`
- エラーを `min_error` しきい値に対してチェック
- デッドゾーン内の場合は0を返す

**エビデンス:**
```cpp
if (std::abs(error) <= min_error) {
    return 0.0;
}
```

**検証:** ✓ 動作中

---

#### FR-1.5.4: Y軸整列優先度
**ステータス:** ✅ **COMPLETE**

**要求仕様:** ステージ4では、システムは前進（X）の前に横方向（Y）整列を優先する必要があります。

**実装:**
- **ファイル:** `nav_docking.cpp:430-446`
- `|error_y| >= min_y_error` の場合、`linear.x = 0` に設定
- 整列時のみ前進

**エビデンス:**
```cpp
if (fabs(error_y) < min_y_error)
{
    // Three-axis control
    twist_msg.linear.x = calculate(...);
    twist_msg.linear.y = calculate(...);
    twist_msg.angular.z = calculate(...);
}
else  // align robot first
{
    twist_msg.linear.x = 0;  // No forward motion
    twist_msg.linear.y = calculate(...);
    twist_msg.angular.z = calculate(...);
}
```

**検証:** ✓ 正しく実装されている

**理由:** 角度をつけた接近を防ぎ、まっすぐな接近を確保。

---

### 1.6 アクションサーバーインターフェース

#### FR-1.6.1: ドックアクションサーバー
**ステータス:** ✅ **COMPLETE**

**要求仕様:** システムは、ドッキングリクエスト用のROS2アクションサーバーを提供する必要があります。

**実装:**
- **ファイル:** `nav_docking.cpp:9-14`
- アクションタイプ: `nav_interface::action::Dock`
- アクション名: `dock`
- ゴール: `bool dock_request`
- 結果: `bool success`
- フィードバック: `float64 distance`

**エビデンス:**
```cpp
action_server_ = rclcpp_action::create_server<Dock>(
    this, "dock",
    std::bind(&Nav_docking::handle_goal, ...),
    std::bind(&Nav_docking::handle_cancel, ...),
    std::bind(&Nav_docking::handle_accepted, ...));
```

**検証:** ✓ 標準ROS2アクションパターン

---

#### FR-1.6.2: ゴール検証
**ステータス:** 🟡 **PARTIAL** (些細な検証)

**要求仕様:** システムは、受け入れる前にドッキングゴールを検証する必要があります。

**実装:**
- **ファイル:** `nav_docking.cpp:119-128`
- 現在は `dock_request == true` のみチェック

**問題:** 検証が些細で、次をチェックしない:
- ❌ ロボット位置（範囲内か？）
- ❌ マーカーが見えるか
- ❌ システム準備状態
- ❌ バッテリーレベル
- ❌ 既存のアクティブゴールなし

**ギャップ:** 検証を強化

**推奨事項:**
```cpp
// Check preconditions
if (!markers_recently_seen()) return REJECT;
if (battery_level < MINIMUM_BATTERY) return REJECT;
if (is_goal_active()) return REJECT;
```

---

#### FR-1.6.3: フィードバック公開
**ステータス:** ✅ **COMPLETE**

**要求仕様:** システムは、ドッキング中にターゲットまでの現在の距離を含むフィードバックを公開する必要があります。

**実装:**
- **ファイル:** `nav_docking.cpp:169-172`
- フィードバックで `error_x`（距離）を公開
- レート: 1 Hzログにスロットルされているが、毎ループ公開

**エビデンス:**
```cpp
feedback->distance = static_cast<double>(feedback_distance);
goal_handle->publish_feedback(feedback);
```

**検証:** ✓ 動作中

**注:** フィードバックはX軸エラーであり、ユークリッド距離ではありません。総エラーの公開を検討:
```cpp
feedback->distance = std::sqrt(error_x*error_x + error_y*error_y + error_yaw*error_yaw);
```

---

#### FR-1.6.4: キャンセルサポート
**ステータス:** ✅ **COMPLETE**

**要求仕様:** システムは、ゴールのキャンセルをサポートし、即座に動作を停止する必要があります。

**実装:**
- **ファイル:** `nav_docking.cpp:131-136, 161-167`
- キャンセルリクエストを受け入れる
- 制御ループを停止するために `enable_callback = false` を設定
- キャンセルされた結果を公開

**エビデンス:**
```cpp
if (goal_handle->is_canceling())
{
    RCLCPP_INFO(this->get_logger(), "Goal canceled.");
    goal_handle->canceled(result);
    Nav_docking::enable_callback = false;
    return;
}
```

**検証:** ✓ 実装済み

**問題:** ⚠️ 停止前にゼロ速度を公開しない。ロボットがドリフトする可能性。

**推奨事項:** コールバックを無効化する前にゼロツイストを公開。

---

### 1.7 安全性と堅牢性

#### FR-1.7.1: マーカータイムアウト検出
**ステータス:** ✅ **COMPLETE**

**要求仕様:** タイムアウト期間内にマーカーが検出されない場合、システムは動作を停止する必要があります。

**実装:**
- **ファイル:** `nav_docking.cpp:427-469, 516`
- タイムアウト: 0.2秒
- タイムアウト超過時にゼロ速度を公開

**エビデンス:**
```cpp
if ((callback_duration < marker_delay_threshold_sec))
{
    // Control active
    cmd_vel_pub->publish(twist_msg);
}
else
{
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;
    cmd_vel_pub->publish(twist_msg);
}
```

**検証:** ✓ 動作中

---

#### FR-1.7.2: 変換タイムアウト
**ステータス:** ✅ **COMPLETE**

**要求仕様:** システムは、ブロッキングを防ぐためにTFルックアップでタイムアウトする必要があります。

**実装:**
- **ファイル:** `nav_docking.cpp:248, 307`
- タイムアウト: `lookupTransform()` で2秒

**エビデンス:**
```cpp
tf_buffer->lookupTransform(base_frame, camera_left_frame,
                          tf2::TimePointZero, tf2::durationFromSec(2));
```

**検証:** ✓ 動作中

---

#### FR-1.7.3: 完了前の確認
**ステータス:** ✅ **COMPLETE**

**要求仕様:** システムは、ドッキング完了とマークする前に2回連続した成功読み取りを必要とする必要があります。

**実装:**
- **ファイル:** `nav_docking.cpp:540-552`
- ブール値フラグ: `confirmed_docking_status`
- チェック間の0.5秒遅延

**エビデンス:**
```cpp
if (confirmed_docking_status==true)
{
    stage_5_docking_status = true;  // Complete
}
else
{
    confirmed_docking_status=true;
    rclcpp::Rate rate(2);
    rate.sleep();  // 0.5 seconds
    return;
}
```

**検証:** ✓ ノイズによる誤完了を防止

---

#### FR-1.7.4: 速度ランピング
**ステータス:** ❌ **NOT IMPLEMENTED**

**要求仕様:** システムは、急激な加速を防ぐために速度をスムーズにランプする必要があります。

**現在の状態:**
- コマンドは瞬時に最大からゼロにジャンプ可能
- 加速度制限なし

**影響:**
- ⚠️ モーターへの機械的ストレス
- ⚠️ 急停止時のトラクション損失
- ⚠️ 乗客の快適性低下

**ギャップ:** 速度ランピングが必要

**推奨事項:**
```cpp
double ramp_velocity(double current, double target, double max_accel, double dt)
{
    double max_change = max_accel * dt;
    double change = target - current;
    if (std::abs(change) > max_change)
        return current + std::copysign(max_change, change);
    return target;
}
```

---

#### FR-1.7.5: ドッキング中の衝突検出
**ステータス:** ❌ **NOT IMPLEMENTED**

**要求仕様:** システムは、ドッキング中に障害物を監視し、衝突が差し迫っている場合は中止する必要があります。

**現在の状態:**
- ビジョン（ArUcoマーカー）のみ使用
- ドッキング中のLiDAR/ソナー統合なし
- 安全スキャナー監視なし

**影響:** ⚠️ 高 - カメラビューにない障害物と衝突する可能性

**ギャップ:** 安全層が必要

**推奨事項:**
- 安全スキャナートピックをサブスクライブ
- 接近経路の障害物を監視
- 障害物検出時に中止してバックアウト

---

#### FR-1.7.6: アクション実行タイムアウト
**ステータス:** ❌ **NOT IMPLEMENTED**

**要求仕様:** ドッキングが最大許容時間より長くかかる場合、システムはタイムアウトする必要があります。

**現在の状態:**
- **ファイル:** `nav_docking.cpp:159-173`
- `while (stage_5_docking_status == false)` - タイムアウトなし!

**問題:** マーカーが失われたりドッキングが失敗した場合、無期限に実行可能

**ギャップ:** 最大実行時間を追加

**推奨事項:**
```cpp
auto start_time = std::chrono::steady_clock::now();
const double MAX_DOCKING_TIME = 60.0;  // seconds

while (stage_5_docking_status == false) {
    auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time).count();

    if (elapsed > MAX_DOCKING_TIME) {
        result->success = false;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Docking timeout");
        return;
    }
    // ... rest of logic
}
```

---

### 1.8 構成とパラメータ

#### FR-1.8.1: PIDパラメータ構成
**ステータス:** ✅ **COMPLETE**

**要求仕様:** システムは、構成ファイルからPIDゲインをロードする必要があります。

**実装:**
- **ファイル:** `nav_docking.cpp:53-74`
- 構成: `src/nav_docking/config/docking_pid_params.yaml`
- パラメータ: X、Y、Z軸のkp、ki、kd

**エビデンス:**
```cpp
this->declare_parameter("pid_parameters.kp_x", 0.00);
this->get_parameter("pid_parameters.kp_x", kp_x);
```

**検証:** ✓ 動作中

---

#### FR-1.8.2: 動的パラメータ再構成
**ステータス:** ❌ **NOT IMPLEMENTED**

**要求仕様:** システムは、ノードを再起動せずにランタイムパラメータ更新をサポートする必要があります。

**現在の状態:**
- パラメータは起動時のみロード
- PIDゲインを変更するにはノードを再起動する必要がある
- フィールドでのチューニングが困難

**ギャップ:** パラメータコールバックを追加

**推奨事項:**
```cpp
// In constructor:
param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&Nav_docking::parametersCallback, this, std::placeholders::_1));

// Callback:
rcl_interfaces::msg::SetParametersResult Nav_docking::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    for (const auto &param : parameters) {
        if (param.get_name() == "pid_parameters.kp_x")
            kp_x = param.as_double();
        // ... other parameters
    }
    return result;
}
```

---

#### FR-1.8.3: マーカーオフセットキャリブレーション
**ステータス:** 🟡 **PARTIAL**

**要求仕様:** システムは、マーカー位置オフセットの簡単なキャリブレーションをサポートする必要があります。

**実装:**
- **ファイル:** `nav_docking.cpp:25-29`
- オフセット用のパラメータが存在
- 値はlaunchファイルで変更可能

**問題点:**
- ❌ キャリブレーション手順がドキュメント化されていない
- ❌ 現在のオフセットを測定するツールなし
- ❌ コメントアウトされたキャリブレーションログ（`nav_docking.cpp:393-395, 408-410, etc.`）

**ギャップ:** キャリブレーションツールとドキュメントが必要

**推奨事項:**
1. 現在のマーカー位置をログに記録するキャリブレーションモードを作成
2. キャリブレーション手順をドキュメント化
3. 最適なオフセットを計算するスクリプトを提供

---

### 1.9 診断と監視

#### FR-1.9.1: システムヘルス診断
**ステータス:** ❌ **NOT IMPLEMENTED**

**要求仕様:** システムは、ヘルス監視のための診断メッセージを公開する必要があります。

**現在の状態:**
- ROS2診断が公開されていない
- 基本的なログ記録のみ

**ギャップ:** 診断パブリッシャーを追加

**推奨事項:**
```cpp
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10);

// Publish status:
// - Marker detection rate
// - Control loop frequency
// - Transform availability
// - Current stage
// - Error magnitudes
```

---

#### FR-1.9.2: パフォーマンスメトリクスログ記録
**ステータス:** ❌ **NOT IMPLEMENTED**

**要求仕様:** システムは、分析のためにドッキング試行統計をログに記録する必要があります。

**現在の状態:**
- 構造化されたログ記録なし
- 成功/失敗追跡なし
- タイミングデータが保存されていない

**ギャップ:** メトリクス収集を追加

**推奨事項:**
```cpp
struct DockingMetrics {
    rclcpp::Time start_time;
    rclcpp::Time end_time;
    bool success;
    int retry_count;
    double final_error_x;
    double final_error_y;
    double final_error_yaw;
};

// Log to file for post-analysis
```

---

#### FR-1.9.3: デバッグ可視化
**ステータス:** 🟡 **PARTIAL**

**要求仕様:** システムは、RVizでのデバッグのために可視化マーカーを公開する必要があります。

**現在の状態:**
- ArUcoマーカーはTFフレームとしてブロードキャスト（✓）
- 速度コマンド可視化なし
- エラーベクトル可視化なし
- ドッキングパス可視化なし

**ギャップ:** RVizマーカーを追加

**推奨事項:**
- 以下のために `visualization_msgs::Marker` を公開:
  - 目的位置（ターゲット）
  - 現在のエラーベクトル
  - 計画された軌道
  - ステージインジケーター

---

### 1.10 アンドッキング

#### FR-1.10.1: アンドッキングアクション
**ステータス:** ❌ **NOT IMPLEMENTED**

**要求仕様:** システムは、ロボットをドックから離すためのアンドッキングアクションを提供する必要があります。

**現在の状態:**
- アンドッキングアクションが定義されていない
- 逆手順なし
- 手動介入が必要

**影響:** 不完全なシステム、手動操作が必要

**ギャップ:** アンドッキングを実装

**推奨事項:**
```cpp
// Add to nav_interface:
# Undock.action
bool undock_request
---
bool success
---
float64 distance

// Implement reverse procedure:
// 1. Move back X meters
// 2. Rotate to desired heading
// 3. Return success
```

---

### 1.11 スレッド安全性

#### FR-1.11.1: 共有状態保護
**ステータス:** 🐛 **BUGGY** (スレッドセーフでない)

**要求仕様:** システムは、マルチスレッド環境で共有状態をミューテックスで保護する必要があります。

**現在の状態:**
- **ファイル:** `nav_docking.cpp:140-142`
- 別スレッドでアクション実行
- コールバックはROSスピナースレッドで実行
- ロックなしでアクセスされる共有変数:
  - `stage_4_docking_status`
  - `stage_5_docking_status`
  - `enable_callback`
  - `error_x`, `error_y`, `error_yaw`
  - タイマーコールバック

**問題:** 競合状態が可能

**影響:** 🔴 重大 - 未定義動作

**エビデンス:**
```cpp
// nav_docking.cpp:140-142
void Nav_docking::handle_accepted(const std::shared_ptr<GoalHandleDock> goal_handle)
{
    // Spawns new thread - no synchronization!
    std::thread{std::bind(&Nav_docking::execute, this, goal_handle)}.detach();
}
```

**ギャップ:** ミューテックス保護を追加

**推奨事項:**
```cpp
#include <mutex>

private:
    std::mutex state_mutex_;

// In functions accessing shared state:
void frontMarkerCmdVelPublisher()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    // ... access shared variables
}
```

---

#### FR-1.11.2: デュアルタイマー競合状態
**ステータス:** 🐛 **BUGGY** (競合状態)

**要求仕様:** システムは、複数の制御ループからの同時公開を防ぐ必要があります。

**現在の状態:**
- **ファイル:** `nav_docking.cpp:100-106`
- 2つのタイマー: `front_timer_` および `dual_timer_`
- 両方とも同じトピック `cmd_vel_final` に公開
- 同期なし

**問題:**
```cpp
front_timer_ = this->create_wall_timer(period,
    std::bind(&Nav_docking::frontMarkerCmdVelPublisher, this));
dual_timer_ = this->create_wall_timer(period,
    std::bind(&Nav_docking::dualMarkerCmdVelPublisher, this));
```

両方のタイマーが同時に発火し、競合するコマンドを公開する可能性があります。

**影響:** 🔴 重大 - 予測不可能な動作

**ギャップ:** 単一タイマーを使用するかミューテックスを追加

**推奨事項:**
```cpp
// Option 1: Single timer
control_timer_ = this->create_wall_timer(period,
    std::bind(&Nav_docking::controlLoop, this));

void controlLoop() {
    if (stage_4_docking_status == false)
        frontMarkerCmdVelPublisher();
    else
        dualMarkerCmdVelPublisher();
}

// Option 2: Mutex
std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
cmd_vel_pub->publish(twist_msg);
```

---

## 2. 非機能要求仕様

### 2.1 パフォーマンス

#### NFR-2.1.1: 制御ループ周波数
**ステータス:** ✅ **COMPLETE**

**要求仕様:** システムは、最低20 Hzで制御ループを実行する必要があります。

**実装:**
- **ファイル:** `nav_docking.h:68`
- `publish_rate = 30` Hz

**検証:** ✓ 要求仕様を満たす（30 > 20）

---

#### NFR-2.1.2: マーカー検出レイテンシ
**ステータス:** ❓ **UNCLEAR**

**要求仕様:** システムは、100ms以内にマーカー姿勢を検出および処理する必要があります。

**現在の状態:**
- 検出はコールバックで実行（画像到着時）
- タイミング測定なし
- レイテンシログなし

**ギャップ:** パフォーマンスメトリクスを追加

---

#### NFR-2.1.3: 変換ルックアップパフォーマンス
**ステータス:** ✅ **COMPLETE**

**要求仕様:** システムは、制御ループをブロックせずに変換を取得する必要があります。

**実装:**
- TFルックアップで2秒タイムアウト
- 例外ハンドリングがブロッキングを防止

**検証:** ✓ 動作中

---

### 2.2 精度

#### NFR-2.2.1: 最終ドッキング精度
**ステータス:** ✅ **COMPLETE** (バグが修正されれば)

**要求仕様:** システムは、最終ドッキングで±1mm位置精度を達成する必要があります。

**実装:**
- **ファイル:** `nav_docking.cpp:131`
- `min_docking_error = 0.001` m (1mm)
- ステージ5はすべてのエラー < 1mmを要求

**検証:** ✓ 要求仕様が正しくエンコード

**注:** 実際の精度は以下に依存:
- カメラキャリブレーションの品質
- マーカーサイズと品質
- 照明条件
- 修正されたバグ（特にデュアルマーカー計算）

---

#### NFR-2.2.2: 接近精度
**ステータス:** ✅ **COMPLETE**

**要求仕様:** システムは、ステージ4整列で±1cm精度を達成する必要があります。

**実装:**
- **ファイル:** `nav_docking.h:129`
- `min_error = 0.01` m (1cm)

**検証:** ✓ 要求仕様を満たす

---

### 2.3 信頼性

#### NFR-2.3.1: ドッキング成功率
**ステータス:** ❓ **UNCLEAR** (データなし)

**要求仕様:** システムは、通常条件下で95%以上の成功率を達成する必要があります。

**現在の状態:**
- 成功/失敗追跡なし
- 統計収集なし
- ベースラインが確立されていない

**ギャップ:** メトリクス収集とテストを実装

---

#### NFR-2.3.2: 耐障害性
**ステータス:** 🟡 **PARTIAL**

**要求仕様:** システムは、単一点障害を適切に処理する必要があります。

**現在の処理:**
- ✅ 1つのマーカー喪失: シングルマーカーにフォールバック
- ✅ TFタイムアウト: エラーをログに記録してサイクルをスキップ
- ❌ 両方のマーカー喪失: 停止するがアクションを中止しない
- ❌ カメラ故障: 検出なし
- ❌ モーター故障: 検出なし

**ギャップ:** 障害検出と報告を改善

---

### 2.4 安全性

#### NFR-2.4.1: 緊急停止サポート
**ステータス:** ❓ **UNCLEAR**

**要求仕様:** システムは、100ms以内に緊急停止信号に応答する必要があります。

**現在の状態:**
- 緊急停止サブスクリプションなし
- アクションキャンセルは利用可能だがクライアントリクエストが必要
- ハードウェアe-stop統合なし

**ギャップ:** 緊急停止ハンドリングを追加

---

#### NFR-2.4.2: 衝突回避
**ステータス:** ❌ **NOT IMPLEMENTED**

**要求仕様:** ドッキング中に0.5m以内に障害物が検出された場合、システムは停止する必要があります。

**現在の状態:**
- ドッキング中の障害物検出なし
- ビジョンのみ（非視覚障害物に対して盲目）

**ギャップ:** 安全スキャナーを統合

---

### 2.5 保守性

#### NFR-2.5.1: コードドキュメンテーション
**ステータス:** 🟡 **PARTIAL**

**要求仕様:** すべてのpublic関数にはドキュメンテーションコメントが必要です。

**現在の状態:**
- Doxygenコメントなし
- 最小限のインラインコメント
- 関数レベルのドキュメントなし
- いくつかの高レベルコメントが存在

**ギャップ:** 包括的なドキュメントを追加

---

#### NFR-2.5.2: 構成管理
**ステータス:** 🟡 **PARTIAL**

**要求仕様:** システム構成は、YAMLファイルで外部化する必要があります。

**現在の状態:**
- ✅ YAMLのPIDゲイン
- ❌ しきい値がハードコード
- ❌ launchファイルのマーカーID
- ❌ 構成検証なし

**ギャップ:** すべてのマジックナンバーを構成に移動

---

### 2.6 テスト

#### NFR-2.6.1: ユニットテストカバレッジ
**ステータス:** ❌ **NOT IMPLEMENTED**

**要求仕様:** システムは、80%以上のユニットテストカバレッジを持つ必要があります。

**現在の状態:**
- ドッキングコンポーネントの**ユニットテストなし**
- mecanum_wheelsパッケージのlintingテストのみ

**ギャップ:** 包括的なユニットテストを作成

**推奨事項:**
```cpp
// Tests needed:
// - PID calculation correctness
// - Dual/single marker switching logic
// - State transitions
// - Error calculations
// - Transform utilities
// - Parameter validation
```

---

#### NFR-2.6.2: 統合テスト
**ステータス:** ❌ **NOT IMPLEMENTED**

**要求仕様:** システムは、完全なドッキングシーケンスの統合テストを持つ必要があります。

**現在の状態:**
- 自動統合テストなし
- 手動フィールドテストのみ

**ギャップ:** 統合テストスイートを作成

---

#### NFR-2.6.3: シミュレーションテスト
**ステータス:** ❓ **UNCLEAR**

**要求仕様:** システムは、Gazeboシミュレーションでテスト可能である必要があります。

**現在の状態:**
- launchファイルでシミュレーションが言及されている
- シミュレーションでドッキングがテストされているか不明
- ドキュメント化されたシミュレーションテスト手順なし

**ギャップ:** シミュレーションテスト環境を作成

---

## 3. ギャップの要約

### 🔴 重大（修正必須）

| ID | 要求仕様 | ギャップ | 影響 |
|----|-------------|-----|--------|
| FR-1.5.1 | PID制御 | 積分項が壊れている | Kiゲインが無効 |
| FR-1.4.1 | デュアルマーカー | 距離計算バグ | 不正確な位置決め |
| FR-1.11.1 | スレッド安全性 | ミューテックス保護なし | 競合状態 |
| FR-1.11.2 | タイマー同期 | デュアルタイマーの競合 | 予測不可能な動作 |
| FR-1.2.1 | バグ修正 | 間違ったパラメータ名 (nav_goal) | 間違ったマーカーID |

### 🟡 高優先度

| ID | 要求仕様 | ギャップ | 影響 |
|----|-------------|-----|--------|
| FR-1.3.1 | 状態マシン | 正式なFSMなし | 保守が困難 |
| FR-1.3.3 | エラー回復 | リトライロジックなし | 無限ループの可能性 |
| FR-1.6.2 | 検証 | 些細なゴールチェック | 悪いゴールを受け入れる |
| FR-1.7.4 | 速度ランピング | 加速度制限なし | ぎくしゃくした動き |
| FR-1.7.6 | タイムアウト | アクションタイムアウトなし | 無限実行の可能性 |
| FR-1.10.1 | アンドッキング | 実装されていない | 不完全なシステム |

### 🟢 中優先度

| ID | 要求仕様 | ギャップ | 影響 |
|----|-------------|-----|--------|
| FR-1.5.2 | 速度クランプ | 最小速度ロジックの問題 | ターゲット付近で振動 |
| FR-1.7.5 | 衝突 | 障害物検出なし | 安全リスク |
| FR-1.8.2 | 再構成 | 動的パラメータなし | チューニングが困難 |
| FR-1.9.1 | 診断 | 実装されていない | 監視が困難 |
| FR-2.6.1 | ユニットテスト | なし | 信頼性が低い |

---

## 4. 推奨実装ロードマップ

### フェーズ1: 重大バグ修正（第1週）
1. ✅ PID積分計算を修正
2. ✅ デュアルマーカー距離式を修正
3. ✅ パラメータ割り当てバグを修正
4. ✅ ミューテックス保護を追加
5. ✅ デュアルタイマーを統合

### フェーズ2: アーキテクチャ改善（第2-3週）
1. ✅ 適切な状態マシンを実装
2. ✅ エラー回復とリトライロジックを追加
3. ✅ アクション実行タイムアウトを追加
4. ✅ ゴール検証を改善

### フェーズ3: 安全性と堅牢性（第4-5週）
1. ✅ 速度ランピングを追加
2. ✅ 衝突検出を統合
3. ✅ 緊急停止ハンドラーを追加
4. ✅ 包括的な入力検証を実装

### フェーズ4: 機能とテスト（第6-8週）
1. ✅ アンドッキングを実装
2. ✅ 診断公開を追加
3. ✅ ユニットテストスイートを作成
4. ✅ 統合テストを作成
5. ✅ パフォーマンス監視を追加

### フェーズ5: 仕上げとドキュメンテーション（第9-10週）
1. ✅ 動的再構成を追加
2. ✅ コードドキュメントを改善
3. ✅ キャリブレーションツールを作成
4. ✅ ユーザーマニュアルを作成
5. ✅ フィールド検証を実施

---

## 5. テストチェックリスト

### 必要なユニットテスト
- [ ] PIDコントローラー計算
- [ ] デュアル/シングルマーカー切り替え
- [ ] エラー計算精度
- [ ] 状態遷移ロジック
- [ ] 変換ヘルパー
- [ ] パラメータ検証
- [ ] 速度クランピング

### 必要な統合テスト
- [ ] 完全なドッキングシーケンス（ステージ3→4→5）
- [ ] ドッキング中のシングルマーカーフォールバック
- [ ] マーカー喪失からの回復
- [ ] 各ステージでのアクションキャンセル
- [ ] 連続した複数のドッキング試行

### 必要なシミュレーションテスト
- [ ] 完璧なマーカーでのドッキング
- [ ] 1つのマーカーが遮蔽されたドッキング
- [ ] 検出にノイズがあるドッキング
- [ ] 障害回復シナリオ

### 必要なフィールドテスト
- [ ] さまざまな照明条件
- [ ] 異なる接近角度
- [ ] 異なる速度
- [ ] さまざまな距離のマーカー
- [ ] 長期信頼性（100回以上の試行）

---

## 6. コンプライアンスマトリックス

| 要求仕様カテゴリ | 完成 | 部分的 | 未実装 | バグあり | 合計 |
|---------------------|----------|---------|-----------|-------|-------|
| 視覚検出 | 4 | 0 | 0 | 0 | 4 |
| 変換 | 3 | 0 | 0 | 0 | 3 |
| 状態マシン | 0 | 2 | 1 | 0 | 3 |
| 検出戦略 | 2 | 0 | 0 | 1 | 3 |
| 制御システム | 2 | 1 | 0 | 2 | 5 |
| アクションインターフェース | 3 | 1 | 0 | 0 | 4 |
| 安全性 | 3 | 0 | 3 | 0 | 6 |
| 構成 | 1 | 1 | 1 | 0 | 3 |
| 診断 | 0 | 1 | 2 | 0 | 3 |
| アンドッキング | 0 | 0 | 1 | 0 | 1 |
| スレッド安全性 | 0 | 0 | 0 | 2 | 2 |
| **合計** | **18** | **6** | **8** | **5** | **37** |

**完成率:** 49% (18/37) ✅ 完成
**対応が必要:** 51% (19/37) - 6部分的、8欠落、5バグあり

---

*この要求仕様書は、実装が進むにつれて更新する必要があります。次のステップ: 重大なバグに対処し、その後欠落している安全機能を実装します。*
