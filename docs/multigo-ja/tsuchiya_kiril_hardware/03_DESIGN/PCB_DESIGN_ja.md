# PCB設計リファレンス

**チーム:** Tsuchiya（ハードウェア）
**日付:** 2025-12-16
**バージョン:** 1.0

## カスタムPCB

### 安全コントローラPCB

**目的:** 非常停止管理、ウォッチドッグタイマー

**コンポーネント:**
- STM32F4マイクロコントローラ（車載グレード）
- デュアル電源（冗長性のための24V + 12V）
- 非常停止入力回路（絶縁型）
- リレードライバ（60Aコイル、48Vメイン電源を制御）
- ウォッチドッグタイマー（500msタイムアウト）
- ステータスLED（電源、非常停止アクティブ、故障）

**ファイル:**
- 回路図: `pcb/safety_controller/safety_controller_sch.pdf`
- レイアウト: `pcb/safety_controller/safety_controller_layout.pdf`
- ガーバーファイル: `pcb/safety_controller/gerbers/`

### 電力配分PCB

**目的:** 電圧監視、電流検出、ヒューズホルダ

**コンポーネント:**
- 電圧監視用ADC（48V、24V、12V、5Vレール）
- ホール効果電流センサー（50A範囲）
- ヒューズホルダ（車載用ブレードヒューズ）
- レールごとのステータスLEDインジケータ

**ファイル:**
- 回路図: `pcb/power_distribution/power_dist_sch.pdf`
- レイアウト: `pcb/power_distribution/power_dist_layout.pdf`

---
**参考文献:** ELECTRICAL_SYSTEM_ARCHITECTURE.md
