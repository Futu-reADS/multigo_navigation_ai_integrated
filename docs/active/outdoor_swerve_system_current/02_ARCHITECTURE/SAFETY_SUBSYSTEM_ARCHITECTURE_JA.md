# 安全サブシステムアーキテクチャ

**文書ID:** ARCH-SAFE-001
**バージョン:** 1.0
**日付:** 2025年12月15日
**ステータス:** ドラフト

---

## 1. 概要

**安全サブシステム**は**最優先サブシステム**であり、乗客と傍観者の安全な運用を保証します。4つの独立した安全レイヤーで**多層防御**戦略を実装します。

**主要機能:**
- 緊急停止（ハードウェアレベル、<100ms応答）
- 乗客安全監視（安全なアタッチメント、速度制限、滑らかな動き）
- 運用安全（障害物回避、斜面制限、天候制限）
- システムヘルス監視（ウォッチドッグ、診断、フェールセーフ）
- ISO 13482準拠（パーソナルケアロボット安全）

**安全哲学（多層防御）:**
```
レイヤー1: 緊急停止（ハードウェア）     ← <100ms、独立回路
レイヤー2: システム監視（ウォッチドッグ） ← ソフトウェアウォッチドッグ、ヘルスチェック
レイヤー3: 衝突回避（知覚+計画）        ← 予測的安全
レイヤー4: 運用安全（制約）            ← 速度制限、斜面制限
```

---

## 2. システムアーキテクチャ

### 2.1 コンポーネント図

```
┌──────────────────────────────────────────────────────────────────┐
│                    安全サブシステム                               │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │          レイヤー1: 緊急停止（ハードウェア）                │ │
│  │                                                            │ │
│  │  物理E-Stopボタン ──┐                                      │ │
│  │  UI E-Stopボタン ───┼──▶ リレー ──▶ モーター電源OFF       │ │
│  │  安全モニタートリガー─┘      （ハードウェア回路）          │ │
│  │                                  <100ms応答                │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │        レイヤー2: システム監視（ソフトウェア）              │ │
│  │                                                            │ │
│  │  ┌──────────────────┐                                      │ │
│  │  │ 安全モニター     │  ウォッチドッグ、ヘルスチェック      │ │
│  │  │   (スーパーバイザー)│  全サブシステムがステータス報告   │ │
│  │  │                  │  故障時にE-Stopトリガー              │ │
│  │  └────────┬─────────┘                                      │ │
│  │           │                                                 │ │
│  │           │ ヘルスステータス                                │ │
│  │           ↓                                                 │ │
│  │  ┌────────────────────────────────────────┐               │ │
│  │  │ サブシステムヘルスモニター:             │               │ │
│  │  │ - ナビゲーションヘルス                  │               │ │
│  │  │ - ローカライゼーションヘルス（NDTスコア）│               │ │
│  │  │ - 知覚ヘルス（LiDAR）                  │               │ │
│  │  │ - スワーブドライブヘルス（ホイールエンコーダー）│        │ │
│  │  │ - ドッキングヘルス（ArUco検出）        │               │ │
│  │  │ - バッテリーヘルス（電圧、電流）        │               │ │
│  │  └────────────────────────────────────────┘               │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │   レイヤー3: 衝突回避（知覚+計画）                          │ │
│  │                                                            │ │
│  │  3D LiDAR → 知覚 → コストマップ → Nav2 → 障害物回避       │ │
│  │  最小距離: 障害物から0.5m（インフレーション）             │ │
│  │  動的障害物追跡と予測                                      │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │        レイヤー4: 運用安全（制約）                          │ │
│  │                                                            │ │
│  │  - 速度制限: 1.0 m/s（乗客）、1.5 m/s（空）               │ │
│  │  - 加速度制限: 0.5 m/s²                                   │ │
│  │  - 斜面制限: 最大10°                                      │ │
│  │  - 天候制限: 大雨なし（IP54+）                            │ │
│  │  - 運用時間: 日中のみ（自律夜間なし）                      │ │
│  └────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────┘
```

---

## 3. ROS 2ノードアーキテクチャ

### 3.1 安全モニターノード

**ノード名:** `safety_monitor`
**言語:** C++
**ライフサイクル:** マネージドライフサイクルノード（最優先）

**購読トピック:**
- `/odom` (nav_msgs/Odometry): ロボット速度（速度制限）
- `/ndt_score` (std_msgs/Float32): ローカライゼーション品質
- `/points_filtered` (sensor_msgs/PointCloud2): 障害物検出
- `/battery_status` (sensor_msgs/BatteryState): バッテリーヘルス
- `/diagnostics` (diagnostic_msgs/DiagnosticArray): サブシステムヘルス
- `/passenger_attached` (std_msgs/Bool): 車椅子ドッキングステータス

**配信トピック:**
- `/safety_status` (custom_msgs/SafetyStatus): 全体安全状態
- `/emergency_stop` (std_msgs/Bool): 緊急停止トリガー
- `/cmd_vel_limited` (geometry_msgs/Twist): 安全制限後の速度コマンド

**購読サービス（ヘルスチェック）:**
- `/navigation/health` (custom_srvs/GetHealth)
- `/perception/health` (custom_srvs/GetHealth)
- `/swerve_drive/health` (custom_srvs/GetHealth)

**配信アクション:**
- `/safe_stop` (custom_msgs/action/SafeStop): 停止への制御減速

**パラメータ:**
```yaml
safety_monitor:
  ros__parameters:
    # ウォッチドッグタイムアウト
    watchdog:
      navigation_timeout: 1.0     # 秒（更新なし→E-Stop）
      localization_timeout: 1.0   # 秒
      perception_timeout: 0.5     # 秒（クリティカル！）
      control_timeout: 0.2        # 秒（非常にクリティカル！）
    
    # ヘルスチェック閾値
    health:
      min_ndt_score: 0.3          # ローカライゼーション品質
      min_battery_voltage: 42.0   # ボルト（48V公称の80%）
      max_cpu_usage: 90.0         # パーセント
      max_temperature: 80.0       # 摂氏（CPU/バッテリー）
    
    # 運用制限
    limits:
      max_speed_passenger: 1.0    # m/s
      max_speed_empty: 1.5        # m/s
      max_acceleration: 0.5       # m/s²
      max_slope: 10.0             # 度（17.6%勾配）
      min_obstacle_distance: 0.5  # メートル
    
    # 緊急停止
    estop:
      hardware_relay_topic: "/estop_relay"  # GPIO出力
      deceleration_rate: 1.0      # m/s²（制御停止）
```

---

### 3.2 緊急停止コントローラーノード

**ノード名:** `estop_controller`
**言語:** C++
**ライフサイクル:** 標準ノード

**購読トピック:**
- `/estop_button_physical` (std_msgs/Bool): 物理E-Stopボタン（GPIO入力）
- `/estop_button_ui` (std_msgs/Bool): UI E-Stopボタン
- `/emergency_stop` (std_msgs/Bool): ソフトウェアE-Stopトリガー（safety_monitorから）

**配信トピック:**
- `/estop_relay` (std_msgs/Bool): リレー制御（GPIO出力、モーター電源切断）
- `/estop_status` (custom_msgs/EStopStatus): E-Stop状態と理由

**ハードウェアインターフェース:**
- GPIO入力: 物理E-Stopボタン（アクティブロー、プルアップ抵抗）
- GPIO出力: リレー制御（NOリレー、モーターコントローラーへの48V切断）

**設定:**
```yaml
estop_controller:
  ros__parameters:
    # GPIOピン
    gpio:
      estop_button_pin: 17        # BCMピン（Raspberry Pi/類似）
      relay_pin: 27               # BCMピン
    
    # デバウンス
    debounce_ms: 50               # 50msバウンス無視
    
    # 自動リセット
    auto_reset_enabled: false     # 手動リセット必要
    auto_reset_delay: 5.0         # 秒（有効な場合）
```

---

### 3.3 速度ガバナーノード

**ノード名:** `velocity_governor`
**言語:** C++
**ライフサイクル:** 標準ノード

**購読トピック:**
- `/cmd_vel` (geometry_msgs/Twist): 生速度コマンド（Nav2から）
- `/passenger_attached` (std_msgs/Bool): 乗客ステータス
- `/safety_status` (custom_msgs/SafetyStatus): 現在の安全状態

**配信トピック:**
- `/cmd_vel_limited` (geometry_msgs/Twist): 制限速度コマンド（スワーブドライブへ）

**アルゴリズム:**
```cpp
geometry_msgs::msg::Twist limitVelocity(
    const geometry_msgs::msg::Twist& cmd_vel,
    bool passenger_onboard,
    const SafetyStatus& safety_status) {

    geometry_msgs::msg::Twist limited = cmd_vel;

    // 1. 速度制限
    double max_linear = passenger_onboard ? max_speed_passenger : max_speed_empty;
    double linear_mag = std::sqrt(cmd_vel.linear.x * cmd_vel.linear.x +
                                   cmd_vel.linear.y * cmd_vel.linear.y);
    if (linear_mag > max_linear) {
        double scale = max_linear / linear_mag;
        limited.linear.x *= scale;
        limited.linear.y *= scale;
    }

    // 2. 角速度制限
    limited.angular.z = std::clamp(cmd_vel.angular.z, -max_angular_vel, max_angular_vel);

    // 3. 加速度制限（前回コマンドと比較）
    double dt = 0.05;  // 20 Hzコントローラー
    double accel_x = (limited.linear.x - prev_cmd_.linear.x) / dt;
    double accel_y = (limited.linear.y - prev_cmd_.linear.y) / dt;
    
    if (std::abs(accel_x) > max_acceleration) {
        limited.linear.x = prev_cmd_.linear.x + std::copysign(max_acceleration * dt, accel_x);
    }
    if (std::abs(accel_y) > max_acceleration) {
        limited.linear.y = prev_cmd_.linear.y + std::copysign(max_acceleration * dt, accel_y);
    }

    // 4. 安全オーバーライド
    if (safety_status.emergency_stop) {
        // 即座に停止
        limited.linear.x = 0.0;
        limited.linear.y = 0.0;
        limited.angular.z = 0.0;
    } else if (safety_status.degraded_mode) {
        // 劣化モードで速度低減（例: 低バッテリー、低品質ローカライゼーション）
        limited.linear.x *= 0.5;
        limited.linear.y *= 0.5;
        limited.angular.z *= 0.5;
    }

    prev_cmd_ = limited;
    return limited;
}
```

---

## 4. 安全レイヤー（多層防御）

### 4.1 レイヤー1: 緊急停止（ハードウェア）

**目的:** モーターへの即座の電源切断（<100ms応答）

**実装:**
```
物理E-Stopボタン（NC、常時閉）
    │
    ├──▶ GPIO入力（estop_controller）
    │
    └──▶ ハードウェアリレー（NO、常時開）
            │
            └──▶ モーターコントローラーへの48V電源
```

**ハードウェア回路:**
- 物理E-Stopボタンがリレーコイルと直列配線
- ボタン押下 → リレー開 → モーター電源切断
- ソフトウェアから独立（フェールセーフ）

**ソフトウェアインターフェース:**
```cpp
void handlePhysicalEStop(bool button_pressed) {
    if (button_pressed) {
        // 1. リレー開（モーター電源切断）
        gpio_->setPin(relay_pin, false);
        
        // 2. E-Stopステータス配信
        estop_status.active = true;
        estop_status.reason = "物理ボタン押下";
        estop_publisher_->publish(estop_status);
        
        // 3. イベントログ
        RCLCPP_ERROR(get_logger(), "緊急停止作動: 物理ボタン");
    }
}

void resetEStop() {
    // 手動リセット必要
    // 1. 安全条件確認
    if (!verifySafeToReset()) {
        RCLCPP_WARN(get_logger(), "E-Stopリセット不可: 危険条件");
        return;
    }
    
    // 2. リレー閉（モーター電源復元）
    gpio_->setPin(relay_pin, true);
    
    // 3. リセットステータス配信
    estop_status.active = false;
    estop_status.reason = "リセット";
    estop_publisher_->publish(estop_status);
    
    RCLCPP_INFO(get_logger(), "E-Stopリセット");
}
```

---

### 4.2 レイヤー2: システム監視（ウォッチドッグ）

**目的:** ソフトウェア故障を検出し安全シャットダウンをトリガー

**ウォッチドッグアルゴリズム:**
```cpp
class SafetyMonitor {
private:
    struct SubsystemHealth {
        std::string name;
        rclcpp::Time last_update;
        double timeout;
        bool healthy;
    };
    
    std::map<std::string, SubsystemHealth> subsystems_;

public:
    void checkWatchdog() {
        rclcpp::Time now = this->now();
        
        for (auto& [name, health] : subsystems_) {
            double elapsed = (now - health.last_update).seconds();
            
            if (elapsed > health.timeout) {
                RCLCPP_ERROR(get_logger(), "ウォッチドッグタイムアウト: %s (%.2fs)", 
                             name.c_str(), elapsed);
                health.healthy = false;
                
                // 安全停止トリガー（緊急停止ではなく、制御減速）
                triggerSafeStop(name + " ウォッチドッグタイムアウト");
            }
        }
    }
    
    void updateSubsystemHealth(const std::string& name, bool healthy) {
        if (subsystems_.count(name)) {
            subsystems_[name].last_update = this->now();
            subsystems_[name].healthy = healthy;
        }
    }
    
    void triggerSafeStop(const std::string& reason) {
        // 停止への制御減速（緊急電源切断ではない）
        auto goal = custom_msgs::action::SafeStop::Goal();
        goal.reason = reason;
        goal.deceleration_rate = deceleration_rate_;
        
        safe_stop_client_->async_send_goal(goal);
        
        // 劣化モード配信
        safety_status_.emergency_stop = false;
        safety_status_.degraded_mode = true;
        safety_status_.reason = reason;
        status_publisher_->publish(safety_status_);
    }
};
```

**ヘルスチェック:**
```cpp
void performHealthChecks() {
    // 1. ローカライゼーションヘルス（NDTスコア）
    if (ndt_score_ < min_ndt_score) {
        updateSubsystemHealth("localization", false);
        RCLCPP_WARN(get_logger(), "ローカライゼーション不良: NDTスコア %.2f < %.2f", 
                    ndt_score_, min_ndt_score);
    }
    
    // 2. バッテリーヘルス
    if (battery_voltage_ < min_battery_voltage) {
        updateSubsystemHealth("battery", false);
        RCLCPP_ERROR(get_logger(), "低バッテリー: %.1fV < %.1fV", 
                     battery_voltage_, min_battery_voltage);
        triggerSafeStop("低バッテリー");
    }
    
    // 3. CPU温度
    double cpu_temp = getCPUTemperature();
    if (cpu_temp > max_temperature) {
        RCLCPP_ERROR(get_logger(), "CPU過熱: %.1f°C", cpu_temp);
        triggerSafeStop("CPU過熱");
    }
    
    // 4. 知覚ヘルス（LiDARデータレート）
    double lidar_rate = getLidarDataRate();
    if (lidar_rate < 5.0) {  // 10 Hz期待、5 Hzまで許容
        updateSubsystemHealth("perception", false);
        RCLCPP_WARN(get_logger(), "LiDARデータレート低下: %.1f Hz", lidar_rate);
    }
}
```

---

### 4.3 レイヤー3: 衝突回避

**目的:** 予測的障害物回避（衝突が起こる前に防止）

**統合:**
- 知覚サブシステムが障害物検出（3D LiDAR → フィルター済み点群）
- Nav2コストマップがインフレーション付き障害物マーク（0.5mバッファ）
- Nav2コントローラーがパスフォロー中に障害物回避

**最小距離強制:**
```yaml
# local_costmap.yamlで
inflation_layer:
  inflation_radius: 0.5           # メートル（安全バッファ）
  cost_scaling_factor: 3.0        # 障害物近くで積極的なコスト増加
```

**動的障害物処理:**
- ボクセルレイヤーが5 Hzで障害物をクリアしマーク（リアルタイム追跡）
- DWBコントローラーが障害物回避軌道をサンプリング
- パスブロック時の回復動作（スピン、バックアップ、待機）

---

### 4.4 レイヤー4: 運用安全（制約）

**目的:** 危険条件を防ぐため運用制限を強制

**速度制限:**
- **乗客搭乗:** 1.0 m/s（3.6 km/h、歩行ペース）
- **空:** 1.5 m/s（5.4 km/h、速歩）
- **加速度:** 0.5 m/s²（緩やか、乗客不快感回避）

**斜面制限:**
- **最大斜面:** 10°（17.6%勾配、約100kg荷重容量）
- **監視:** IMUピッチ/ロール
- **アクション:** 過度な斜面でのナビゲーションゴール拒否

**天候制限:**
- **IP54+等級:** 小雨動作
- **大雨なし:** 浸水リスク
- **夜間動作なし:** 自律ナビゲーションは日中のみ（屋内は手動オーバーライド）

**運用時間:**
```yaml
operational_hours:
  enabled: true
  daylight_only: true             # 夜間自律動作なし
  override_indoor: true           # 屋内動作はいつでも許可
  sunrise_offset: 30              # 分（日の出30分後まで待機）
  sunset_offset: -30              # 分（日没30分前に停止）
```

---

## 5. 乗客安全機能

### 5.1 安全なアタッチメント検証

**ドッキング確認:**
```cpp
bool verifySecureAttachment() {
    // 1. ドッキング精度チェック（±5mm以内必須）
    double docking_error = computeDockingError();
    if (docking_error > 0.005) {  // 5mm
        RCLCPP_ERROR(get_logger(), "ドッキング誤差大きすぎ: %.1fmm", docking_error * 1000);
        return false;
    }
    
    // 2. 車椅子ロックステータスチェック（将来: CAN/シリアルインターフェース）
    bool locked = checkWheelchairLock();
    if (!locked) {
        RCLCPP_ERROR(get_logger(), "車椅子ロックされていない");
        return false;
    }
    
    // 3. IMU安定性検証（乗客落ち着いた、過度な動きなし）
    double imu_accel_mag = getIMUAccelMagnitude();
    if (imu_accel_mag > 0.5) {  // m/s²（重力除く）
        RCLCPP_WARN(get_logger(), "過度な動き検出、乗客が落ち着くまで待機");
        return false;
    }
    
    return true;
}
```

---

### 5.2 滑らかな動き制御

**ジャーク制限:**
```cpp
// 加速度変化率制限（ジャーク）
double max_jerk = 1.0;  // m/s³

double jerk_x = (current_accel_x - prev_accel_x) / dt;
if (std::abs(jerk_x) > max_jerk) {
    current_accel_x = prev_accel_x + std::copysign(max_jerk * dt, jerk_x);
}
```

**滑らかなスタートとストップ:**
- 2秒かけて0から目標まで加速をランプ
- 安全停止のための制御減速（1.0 m/s²）
- 突然の方向変化なし（角加速度制限）

---

## 6. 故障検出と回復

### 6.1 故障カテゴリ

| 故障タイプ | 例 | 応答 | 回復 |
|------------|---------|----------|----------|
| クリティカル | E-Stopボタン、衝突 | 緊急停止 | 手動リセット |
| 重大 | ローカライゼーション消失、LiDAR故障 | 安全停止 | 自動再試行または手動 |
| 軽微 | 低バッテリー警告、高CPU | 劣化モード | 制限付き継続 |
| 警告 | 低WiFi、高温 | ログのみ | 監視 |

---

### 6.2 回復アクション

**安全停止:**
```cpp
void executeSafeStop(double deceleration_rate) {
    // 1. 制御減速
    double current_speed = getCurrentSpeed();
    double target_speed = 0.0;
    
    rclcpp::Rate rate(20);  // 20 Hz
    while (current_speed > 0.01) {
        double delta_v = deceleration_rate * 0.05;  // dt = 50ms
        target_speed = std::max(0.0, current_speed - delta_v);
        
        publishVelocityCommand(target_speed);
        rate.sleep();
        current_speed = getCurrentSpeed();
    }
    
    // 2. ブレーキ作動（利用可能な場合）
    engageBrakes();
    
    // 3. 停止ステータス配信
    RCLCPP_INFO(get_logger(), "安全停止完了");
}
```

**安全状態への復帰:**
- 最寄りウェイポイントへナビゲート（ローカライゼーションOKの場合）
- 充電ステーションへ戻る（バッテリー低下の場合）
- その場待機し手動介入要求（重大故障の場合）

---

## 7. テスト戦略

### 7.1 ユニットテスト
- 緊急停止応答時間（<100ms）
- 速度制限の正確性
- ヘルスチェック閾値
- ウォッチドッグタイムアウト検出

### 7.2 統合テスト
- 全サブシステムでの完全安全スタック
- 各ソースからのE-Stopトリガー（物理、UI、ソフトウェア）
- 様々な速度からの安全停止
- 劣化モードからの回復

### 7.3 フィールドテスト（安全クリティカル）
- 移動中の緊急停止（様々な速度）
- 障害物回避（静的および動的障害物）
- 斜面制限強制（10°斜面テスト）
- 乗客安全（滑らかな動き、安全なアタッチメント）
- バッテリー枯渇処理（低バッテリー安全帰還）
- ローカライゼーション消失回復

---

## 8. 性能目標

| メトリック | 目標 | 検証 |
|--------|--------|------------|
| 緊急停止応答時間 | <100ms | ハードウェアテスト |
| ウォッチドッグ検出時間 | <1秒 | 故障注入 |
| 速度制限遅延 | <20ms | タイムスタンプ解析 |
| ヘルスチェックレート | 10 Hz | タイムスタンプ解析 |
| 安全停止減速度 | 1.0 m/s² | 加速度計 |
| 衝突回避距離 | >0.5m | 障害物テスト |

---

## 9. 実装フェーズ

### フェーズ1: 緊急停止（スプリント1-2）
- ✅ ハードウェアリレー回路
- ✅ 物理E-Stopボタン統合
- ✅ UI E-Stopボタン
- ✅ E-Stopコントローラーノード

### フェーズ2: システム監視（スプリント3-4）
- ✅ 安全モニターノード
- ✅ ウォッチドッグ実装
- ✅ ヘルスチェック（全サブシステム）
- ✅ 診断統合

### フェーズ3: 運用安全（スプリント5-6）
- ✅ 速度ガバナー
- ✅ 加速度制限
- ✅ 斜面監視（IMU）
- ✅ 運用制約

### フェーズ4: 乗客安全（スプリント7-8）
- ✅ 安全なアタッチメント検証
- ✅ 滑らかな動き制御（ジャーク制限）
- ✅ 乗客搭乗検出
- ✅ 安全検証テスト

---

**文書ステータス:** ドラフト
**実装ステータス:** 未開始
**承認必要:** 安全エンジニア、システムアーキテクト、規制コンプライアンスリード
