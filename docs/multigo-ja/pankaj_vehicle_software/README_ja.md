# 車両ソフトウェア（ROS 2） - Pankajの範囲

**チームリーダー:** Pankaj
**範囲:** ROS 2車両ソフトウェアのみ
**タイムライン:** 18週間の開発+テスト
**ステータス:** 実装準備完了

---

## 🎯 あなたが構築するもの

**自律ナビゲーション用ROS 2ソフトウェアスタック:**
- ✅ **ナビゲーションシステム** - Nav2 + NDT位置推定（GPSなし）
- ✅ **ドッキングコントローラー** - ArUcoビジュアルサーボイング（±2-5mm精度）
- ✅ **スワーブドライブコントローラー** - 逆運動学、全方向移動
- ✅ **知覚パイプライン** - LiDAR処理、障害物検出（CPUのみ）
- ✅ **安全監視** - ソフトウェアウォッチドッグ、緊急停止ロジック
- ✅ **ローカルUI** - Reactタッチスクリーンインターフェース
- ✅ **eHMI制御** - ESP32通信（シリアル）
- ✅ **TVMクライアント** - サーバーへのREST + WebSocket API呼び出し

---

## ❌ あなたが構築しないもの

- ❌ **ハードウェアプラットフォーム**（TsuchiyaとKirilの責任範囲）
  - 機械設計、電気配線、センサー取り付けは含まれません
  - モーターコントローラー、バッテリー管理、物理システムは含まれません

- ❌ **TVMサーバー**（Unnoの責任範囲）
  - バックエンドサーバー、データベース、フリートダッシュボードは含まれません
  - ユーザー管理、予約システムは含まれません

**あなたの焦点:** 車両上で動作するソフトウェア（ROS 2ノード、コントローラー、アルゴリズム）

---

## 🔗 あなたのインターフェース

### ハードウェアへのインターフェース（Tsuchiya + Kiril）

**ドキュメント:** `04_INTERFACES/HARDWARE_SOFTWARE_INTERFACE.md`

**ハードウェアがあなたに提供するもの:**
- センサーからのROS 2トピック（LiDAR、カメラ、IMU、エンコーダー、バッテリー）
- モーターコントローラーからのCANバス応答
- eHMI（ESP32）からのシリアルメッセージ

**あなたがハードウェアに提供するもの:**
- ROS 2コマンドトピック（速度コマンド、ステアリング角度）
- CANバスモーター制御コマンド
- eHMIへのシリアルコマンド（LEDパターン、音、ステータス）

### TVMサーバーへのインターフェース（Unno）

**ドキュメント:**
- `04_INTERFACES/TVM_API_SPECIFICATION.md`
- `04_INTERFACES/TVM_DATA_MODELS.md`

**あなたがサーバーに送信するもの:**
- 車両テレメトリー（位置、ステータス、バッテリー、エラー）REST API経由
- 1秒ごとのハートビート

**サーバーがあなたに送信するもの:**
- ミッションコマンド（派遣、キャンセル）WebSocket経由
- 緊急停止コマンド
- 設定更新

---

## 🛠️ 技術スタック

**オペレーティングシステム:**
- Ubuntu 22.04 LTS

**ミドルウェア:**
- ROS 2 Humble（C++およびPython）

**主要ライブラリ:**
- **ナビゲーション:** Nav2、NDTスキャンマッチャー
- **コンピュータービジョン:** OpenCV 4.x（ArUco検出）
- **ポイントクラウド:** PCL（LiDAR処理）
- **UI:** React 18 + Next.js 14 + TypeScript
- **TVMクライアント:** Python requests + websockets

**GPU不要:** すべての処理はCPUで実行（AMD Ryzen 7 7840HS内蔵グラフィックス）

---

## 📁 フォルダ構造

```
pankaj_vehicle_software/
├── 01_REQUIREMENTS/          ← ソフトウェアが満たすべき要件
│   ├── NAVIGATION_REQUIREMENTS.md
│   ├── DOCKING_SYSTEM_REQUIREMENTS.md
│   ├── SWERVE_DRIVE_REQUIREMENTS.md（ソフトウェア制御のみ）
│   ├── PERCEPTION_REQUIREMENTS.md
│   ├── SAFETY_REQUIREMENTS.md
│   ├── PERFORMANCE_REQUIREMENTS.md
│   ├── UI_EHMI_REQUIREMENTS.md
│   ├── SOFTWARE_REQUIREMENTS.md
│   ├── VEHICLE_TVM_CLIENT_REQUIREMENTS.md
│   ├── ERROR_HANDLING_REQUIREMENTS.md
│   ├── PRIVACY_REQUIREMENTS.md
│   └── SECURITY_REQUIREMENTS.md
│
├── 02_ARCHITECTURE/          ← システムの構成
│   ├── NAVIGATION_SUBSYSTEM_ARCHITECTURE.md
│   ├── DOCKING_SUBSYSTEM_ARCHITECTURE.md
│   ├── SWERVE_DRIVE_ARCHITECTURE.md
│   ├── PERCEPTION_SUBSYSTEM_ARCHITECTURE.md
│   ├── SAFETY_SUBSYSTEM_ARCHITECTURE.md
│   ├── UI_ARCHITECTURE.md
│   ├── EHMI_ARCHITECTURE.md
│   ├── VEHICLE_OVERALL_ARCHITECTURE.md
│   ├── DEPLOYMENT_ARCHITECTURE.md（ソフトウェアデプロイメント）
│   └── SYSTEM_INTEGRATION_ARCHITECTURE.md
│
├── 03_DESIGN/                ← 詳細設計
│   ├── NAVIGATION_CONTROLLER_DESIGN.md
│   ├── DOCKING_CONTROLLER_DESIGN.md
│   ├── SWERVE_DRIVE_CONTROLLER_DESIGN.md
│   ├── PERCEPTION_PIPELINE_DESIGN.md
│   ├── SAFETY_MONITOR_DESIGN.md
│   ├── UI_COMPONENTS_DESIGN.md
│   └── EHMI_FIRMWARE_DESIGN.md
│
├── 04_INTERFACES/            ← 他チームへのインターフェース
│   ├── TVM_API_SPECIFICATION.md ← Unnoへ（重要！）
│   ├── TVM_DATA_MODELS.md ← Unnoへ
│   ├── HARDWARE_SOFTWARE_INTERFACE.md ← Tsuchiyaへ（重要！）
│   └── ROS2_TOPICS.md ← 内部
│
├── 05_DEVELOPMENT/           ← 開発方法
│   ├── VEHICLE_SOFTWARE_DEVELOPMENT_GUIDE.md
│   ├── CODE_STANDARDS_AND_PRACTICES.md
│   └── DEVELOPMENT_SETUP_GUIDE.md
│
├── 06_TESTING/               ← テスト方法
│   └── TESTING_REQUIREMENTS.md
│
└── README.md                 ← このファイル
```

---

## 🚀 クイックスタート

### 1. まずインターフェースドキュメントを読む

**重要 - コーディング前に必読:**
1. `04_INTERFACES/TVM_API_SPECIFICATION.md` - Unnoのサーバーとの通信方法
2. `04_INTERFACES/HARDWARE_SOFTWARE_INTERFACE.md` - ハードウェアとの通信方法
3. `04_INTERFACES/ROS2_TOPICS.md` - 内部ROSトピック構造

### 2. 要件を確認

何を構築するかを理解するために、以下から始めてください:
1. `01_REQUIREMENTS/NAVIGATION_REQUIREMENTS.md`
2. `01_REQUIREMENTS/DOCKING_SYSTEM_REQUIREMENTS.md`
3. `01_REQUIREMENTS/SWERVE_DRIVE_REQUIREMENTS.md`

### 3. 開発環境をセットアップ

以下に従ってください: `05_DEVELOPMENT/DEVELOPMENT_SETUP_GUIDE.md`

### 4. 実装を開始

推奨される順序:
1. **第1-2週:** スワーブドライブコントローラー（基本動作）
2. **第3-4週:** ナビゲーションスタック（Nav2 + NDT）
3. **第5-6週:** 知覚パイプライン（LiDAR処理）
4. **第7-8週:** ドッキングコントローラー（ArUco検出）
5. **第9-10週:** 安全監視
6. **第11-12週:** ローカルUI
7. **第13-14週:** eHMI統合
8. **第15-16週:** TVMクライアント
9. **第17-18週:** 統合テスト

---

## 🔑 主要設計原則

### 1. ソフトウェアのみ
- ハードウェア仕様（モーター、バッテリー、センサー）を気にする必要はありません
- Tsuchiyaがインターフェース仕様に従ってハードウェアを提供します
- あなたはROSトピックを消費し、コマンドを送信するだけです

### 2. TVMクライアント、サーバーではない
- APIを呼び出し、実装はしません
- Unnoがサーバーを構築します
- あなたの仕事: テレメトリーを送信し、コマンドを受信する

### 3. パイロットレベルのシンプルさ
- 基本認証（JWT）
- 基本セキュリティ（HTTPS、APIキー）
- エンタープライズ機能なし（災害復旧、高度な監視）
- 機能性に焦点を当てる

### 4. CPUのみの処理
- GPU不要
- ディープラーニングなし
- 古典的なアルゴリズム（RANSAC、OpenCV、PCL）
- 内蔵グラフィックスで動作

---

## 📋 依存関係

### Tsuchiya + Kiril（ハードウェアチーム）から

**彼らから必要なもの:**
- [ ] 組み立てられ電源が入ったハードウェアプラットフォーム
- [ ] センサートピックを公開するROS 2ドライバー
- [ ] CANコマンドに応答するモーターコントローラー
- [ ] シリアルコマンドに応答するeHMI ESP32

**調整:**
- `shared/INTERFACES/HARDWARE_SOFTWARE_INTERFACE.md`を一緒に確認
- 各インターフェースを段階的にテスト
- 早期ソフトウェアテスト用にハードウェアをモック

### Unno（TVMサーバーチーム）から

**彼から必要なもの:**
- [ ] ネットワーク経由でアクセス可能な稼働中のTVMサーバー
- [ ] 実装されたREST APIエンドポイント
- [ ] 接続を受け入れるWebSocketサーバー
- [ ] 動作する認証（JWT）

**調整:**
- `shared/INTERFACES/TVM_API_SPECIFICATION.md`を一緒に確認
- 早期クライアントテスト用にモックサーバーを使用
- 第15週に統合テスト

---

## ⚠️ 重要な注意事項

### 範囲の境界

**以下を行っている場合、範囲外です:**
- ❌ 機械部品の設計（CADモデル）
- ❌ 電気回路図の作成
- ❌ モーターやバッテリーの選定
- ❌ TVM用データベーススキーマの構築
- ❌ フリート管理ダッシュボードの作成

**あなたの範囲のみ:**
- ✅ ROS 2ノードの記述（C++およびPython）
- ✅ 制御アルゴリズムの実装
- ✅ ローカル車両UIの作成（React）
- ✅ TVM APIクライアントコードの記述

### プライバシーとセキュリティ

**法的レビュー必須:**
- `01_REQUIREMENTS/PRIVACY_REQUIREMENTS.md`はLi Sanによるレビューが必要です
- Li Sanが承認するまでプライバシー機能を実装しないでください
- セキュリティ要件を確定する前に上級者と調整してください

### テスト哲学

- 各ROS 2ノードを個別に単体テスト
- サブシステム（ナビゲーション、ドッキングなど）の統合テスト
- ハードウェアが準備できていない場合はモックハードウェアを使用
- TVMサーバーが準備できていない場合はモックTVMサーバーを使用
- 完全なシステムテストはすべてのチームが統合した後のみ

---

## 📞 連絡先

**あなたのチーム:** Pankaj（単独 + AI支援）

**ハードウェアチーム:** Tsuchiya + Kiril
- **インターフェース:** HARDWARE_SOFTWARE_INTERFACE.md
- **調整:** 週次統合会議

**サーバーチーム:** Unno
- **インターフェース:** TVM_API_SPECIFICATION.md
- **調整:** APIレビュー会議

**法務:** Li San
- **レビュー:** プライバシー要件
- **ステータス:** レビュー保留中

**上級者:** [名前]
- **承認:** 最終アーキテクチャと範囲

---

## ✅ 成功基準

### 技術指標

- [ ] ナビゲーション精度: 100m走行で±10cm
- [ ] ドッキング精度: ±5mm
- [ ] ドッキング成功率: ≥90%
- [ ] ミッション成功率: ≥90%
- [ ] 障害物検出: 精度>95%、再現率>90%
- [ ] バッテリー寿命: 乗客を乗せて≥4時間
- [ ] 緊急停止応答: <100ms

### コード品質

- [ ] すべてのROS 2ノードが単体テストに合格
- [ ] コードカバレッジ≥80%
- [ ] コードがCODE_STANDARDS_AND_PRACTICES.mdの基準に従っている
- [ ] すべてのインターフェースが正しく実装されている
- [ ] ドキュメントが完成している

### 統合

- [ ] ハードウェア-ソフトウェアインターフェースが動作している
- [ ] TVM APIクライアントが動作している
- [ ] すべてのサブシステムが統合されている
- [ ] 安全システムが検証されている

---

**ドキュメントバージョン:** 1.0
**最終更新日:** 2025年12月19日
**ステータス:** 実装準備完了
**次のアクション:** フェーズ3を開始（このフォルダへドキュメントを移動）

---

**ハッピーコーディング！ 🚀**
