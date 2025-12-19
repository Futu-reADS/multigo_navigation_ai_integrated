# 電気回路図リファレンス

**チーム:** Tsuchiya（ハードウェア）
**日付:** 2025-12-16
**バージョン:** 1.0

## 回路図ドキュメント

詳細な電気回路図は別のCADファイルで管理されています:

1. **メイン配電** (`schematics/power_distribution.pdf`)
   - 48Vバッテリー接続
   - DC-DCコンバーター回路
   - ヒューズとリレー配置
   - 非常停止回路

2. **モーターコントローラー配線** (`schematics/motor_controllers.pdf`)
   - CANバストポロジー
   - モーター相接続（U、V、W）
   - ホールセンサー配線
   - 電源接続

3. **コンピュートユニット接続** (`schematics/compute_unit.pdf`)
   - Jetson Orin Nanoピン配置
   - センサー接続（UART、I2C、SPI）
   - USBハブ配線
   - Ethernet接続

4. **安全コントローラーPCB** (`schematics/safety_controller.pdf`)
   - STM32F4マイクロコントローラー回路
   - 非常停止入力回路
   - リレードライバー回路
   - ウォッチドッグタイマー回路

5. **充電ポートインターフェース** (`schematics/charging_port.pdf`)
   - ポゴピン接続
   - 充電リレー回路
   - CANバスインターフェース
   - ステータスLED回路

## ワイヤーハーネス仕様

参照: `documentation/wiring_harness_spec.xlsx`

- コネクタータイプとピンアサインメント
- ワイヤーゲージとカラーコーディング
- ケーブル長と配線経路
- 圧着仕様

---
**参照:** ELECTRICAL_SYSTEM_ARCHITECTURE.md、ELECTRICAL_REQUIREMENTS.md（61要件）
