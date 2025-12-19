# マルチチーム車椅子輸送ロボットドキュメント

**プロジェクト:** フリート管理機能付き屋外優先車椅子輸送ロボット
**バージョン:** 1.0
**日付:** 2025年12月20日
**ステータス:** ドキュメント完成、シニア承認待ち

---

## 🚀 クイックスタート

### プロジェクト関係者向け
- **シニアマネジメント:** `FINAL_SUMMARY_FOR_SENIOR_APPROVAL_ja.md`を読む
- **法務チーム（李さん）:** `PRIVACY_LEGAL_REVIEW_PACKAGE_FOR_LI_SAN_ja.md`を読む

### 開発チーム向け
- **Pankajチーム（車両ソフトウェア）:** `pankaj_vehicle_software/README_ja.md`から開始
- **Unnoチーム（TVMサーバー）:** `unno_tvm_server/README_ja.md`から開始
- **Tsuchiya/Kirilチーム（ハードウェア）:** `tsuchiya_kiril_hardware/README_ja.md`から開始
- **全チーム:** `shared/TEAM_SCOPE_DEFINITION_ja.md`でチーム境界を確認

### クリティカルインターフェース契約（パイロット用に凍結）
1. `shared/INTERFACES/TVM_API_SPECIFICATION_ja.md` - REST + WebSocket API（Pankaj ↔ Unno）
2. `shared/INTERFACES/HARDWARE_SOFTWARE_INTERFACES_ja.md` - ROSトピック + CAN + シリアル（Pankaj ↔ Tsuchiya/Kiril）
3. `shared/INTERFACES/TVM_DATA_MODELS_ja.md` - データベーススキーマ + データ構造（Pankaj ↔ Unno）

---

## 📁 フォルダ構造

```
multigo-ja/
│
├── README_ja.md                            ← 現在地
├── FINAL_SUMMARY_FOR_SENIOR_APPROVAL_ja.md ← 完全プロジェクトサマリー
├── PRIVACY_LEGAL_REVIEW_PACKAGE_FOR_LI_SAN_ja.md
├── DOCS_FOLDER_ANALYSIS_AND_CLEANUP_RECOMMENDATIONS_ja.md
│
├── pankaj_vehicle_software/                (69ファイル - 車両ソフトウェア)
│   ├── README_ja.md                        ← 完全ガイド（507行）
│   ├── 01_REQUIREMENTS/
│   ├── 02_ARCHITECTURE/
│   ├── 03_DESIGN/
│   ├── 04_INTERFACES/
│   ├── 05_DEVELOPMENT/
│   └── 06_TESTING/
│
├── unno_tvm_server/                        (19ファイル - TVMサーバー)
│   ├── README_ja.md                        ← 完全ガイド（410行）
│   ├── TVM_API_IMPLEMENTATION_GUIDE_FOR_UNNO_ja.md  ← ステップバイステップ + コード（742行）
│   ├── 01_REQUIREMENTS/
│   ├── 02_ARCHITECTURE/
│   ├── 03_DESIGN/
│   ├── 04_INTERFACES/
│   └── 05_DEVELOPMENT/
│
├── tsuchiya_kiril_hardware/                (15ファイル - ハードウェア)
│   ├── README_ja.md                        ← 完全ガイド（507行）
│   ├── HARDWARE_REQUIREMENTS_FOR_TSUCHIYA_ja.md  ← 仕様 + ROS例（853行）
│   ├── 01_REQUIREMENTS/
│   ├── 02_ARCHITECTURE/
│   ├── 03_DESIGN/
│   ├── 04_INTERFACES/
│   └── 05_DEVELOPMENT/
│
└── shared/                                 (7ファイル - チーム間共有リソース)
    ├── README_ja.md                        ← 共有ドキュメントの使用方法（380行）
    ├── TEAM_SCOPE_DEFINITION_ja.md         ← 誰が何をするか（470+行）**要読**
    └── INTERFACES/                         ← **凍結契約**
        ├── TVM_API_SPECIFICATION_ja.md     (1,425行)
        ├── TVM_DATA_MODELS_ja.md           (1,361行)
        ├── HARDWARE_SOFTWARE_INTERFACES_ja.md (1,010行)
        ├── ROS2_TOPICS_ja.md
        └── INTERFACE_SPECIFICATIONS_COMPLETE_ja.md
```

**合計:** 113 markdownファイル（クリーン、整理済み、重複なし）

---

## 👥 3チーム構造

### チーム1: Pankaj（車両ソフトウェア）
- **技術:** ROS 2 Humble、Ubuntu 22.04、C++/Python
- **責任:** 車両上で実行されるすべて
  - 自律ナビゲーション（NDT自己位置推定、Nav2）
  - 車椅子ドッキング（ArUco検出、ビジュアルサーボ）
  - スワーブドライブ制御
  - 知覚（LiDAR処理）
  - 安全システム（緊急停止、バンパー、ウォッチドッグ）
  - ローカルUI（タッチスクリーン）
  - TVMクライアントインターフェース
- **タイムライン:** 18週間
- **フォルダ:** `pankaj_vehicle_software/`

### チーム2: Unno（TVMサーバー）
- **技術:** バックエンド（Node.js/Python/Java）、PostgreSQL、React
- **責任:** フリート管理サーバー＆ダッシュボード
  - TVMサーバーバックエンド（REST API + WebSocket）
  - データベース（PostgreSQL）
  - フリートダッシュボード（Web UI）
  - ユーザー認証（JWT）
  - ミッション配信、テレメトリ取り込み
- **タイムライン:** 20週間（15週MVP + 5週高度機能）
- **フォルダ:** `unno_tvm_server/`

### チーム3: Tsuchiya + Kiril（ハードウェア）
- **Tsuchiya:** 機械、電気、センサー、コンピュート
- **Kiril:** 外観デザイン、eHMI（ESP32ファームウェア）
- **責任:** 物理ロボットプラットフォーム
  - シャーシ、モーター、電源、センサー
  - コンピュート設置（GMKtec Nucbox K6）
  - ROS 2センサードライバー（LiDAR、IMU、カメラ、バンパー）
  - CANバス配線
  - ESP32-S3ファームウェア（LED/オーディオ制御）
- **タイムライン:** 16週間（Tsuchiya）、10週間（Kiril）
- **フォルダ:** `tsuchiya_kiril_hardware/`

---

## 🎯 主要技術決定事項

- **GPSなし:** 屋内/屋外施設 - 代わりにNDT自己位置推定を使用
- **CPUのみ:** GMKtec Nucbox K6（ディスクリートGPUなし） - 最適化された知覚
- **スワーブドライブ:** 全方向移動（4独立モジュール）
- **ArUcoドッキング:** デュアルカメラビジュアルサーボ（±2-5mm精度）
- **簡素化されたセキュリティ:** 30要件（パイロット適切、エンタープライズではない）
- **手動デプロイメント:** Ubuntu 22.04 + systemd（Kubernetes/Docker複雑性なし）
- **小雨対応:** IP54防水、小雨で動作（<2.5mm/時）

---

## ⚙️ 技術スタック

**車両ソフトウェア（Pankaj）:**
- ROS 2 Humble on Ubuntu 22.04
- C++17 & Python 3.10
- Nav2、NDT自己位置推定、OpenCV
- GMKtec Nucbox K6（AMD Ryzen 7 7840HS、32GB RAM）

**フリート管理（Unno）:**
- バックエンド: Node.js（Express）またはPython（FastAPI）またはJava（Spring Boot）
- データベース: PostgreSQL 15
- フロントエンド: React 18 + TypeScript
- 認証: JWT（1時間有効期限）
- リアルタイム: WebSocket（Socket.ioまたは同等）

**ハードウェア（Tsuchiya + Kiril）:**
- 4× スワーブドライブモジュール（Vex Robotics）
- 2× 2D LiDAR（前面/後面、270°）
- 3× バンパースイッチ（前面/左/右）
- 2× カメラ（ドッキング用ステレオ）
- 1× IMU（9自由度）
- ESP32-S3（eHMIファームウェア）
- BMS（48Vバッテリー管理）

---

## 📅 開発タイムライン

- **第1週:** チームへの通知、李さんへ法務パッケージ送信
- **第2-3週:** 開発環境セットアップ
- **第4週以降:** 並列独立開発
- **第6週:** 統合チェックポイント（Pankaj + Unno - TVM API）
- **第10週:** 統合チェックポイント（Pankaj + Tsuchiya - ハードウェア引き渡し）
- **第12週:** 統合チェックポイント（Pankaj + Kiril - eHMI）
- **第15週:** 完全システム統合
- **第18週:** パイロット展開

---

## 🚨 重要事項

### インターフェース変更プロトコル
- `shared/INTERFACES/`のすべてのインターフェース契約は**パイロットプロジェクト用に凍結**
- 変更には両方の影響を受けるチーム + シニアマネジメントの承認が必要
- 詳細は`shared/TEAM_SCOPE_DEFINITION_ja.md`セクション3を参照

### パイロット用簡素化スコープ
- **セキュリティ:** 30要件（エンタープライズレベル152ではない）
- **プライバシー:** 李さんによる法的レビュー待ち（2-3週間）
- **デプロイメント:** 手動systemd（Kubernetes/AWSなし）
- **焦点:** 概念実証、本番環境対応ではない

### 独立開発
- チームはモッククライアントで独立開発
- インターフェース契約により並列作業が可能
- 統合は定義されたチェックポイントで実施

---

## 📖 ナビゲーションガイド

### ソフトウェア開発者（Pankajチーム）
1. 読む: `pankaj_vehicle_software/README_ja.md`（完全ガイド）
2. 読む: `shared/INTERFACES/TVM_API_SPECIFICATION_ja.md`（呼び出すAPI）
3. 読む: `shared/INTERFACES/HARDWARE_SOFTWARE_INTERFACES_ja.md`（使用するROSトピック）
4. 確認: `pankaj_vehicle_software/01_REQUIREMENTS/`詳細要件

### フリート管理開発者（Unnoチーム）
1. 読む: `unno_tvm_server/README_ja.md`（完全ガイド）
2. 読む: `unno_tvm_server/TVM_API_IMPLEMENTATION_GUIDE_FOR_UNNO_ja.md`（ステップバイステップ + コード）
3. 読む: `shared/INTERFACES/TVM_API_SPECIFICATION_ja.md`（実装するAPI仕様）
4. 読む: `shared/INTERFACES/TVM_DATA_MODELS_ja.md`（データベーススキーマ）

### ハードウェアエンジニア（Tsuchiya/Kirilチーム）
1. 読む: `tsuchiya_kiril_hardware/README_ja.md`（完全ガイド）
2. 読む: `tsuchiya_kiril_hardware/HARDWARE_REQUIREMENTS_FOR_TSUCHIYA_ja.md`（仕様 + ROS例）
3. 読む: `shared/INTERFACES/HARDWARE_SOFTWARE_INTERFACES_ja.md`（公開するROSトピック）
4. 確認: `tsuchiya_kiril_hardware/01_REQUIREMENTS/`詳細要件

### プロジェクトマネージャー
1. 読む: `FINAL_SUMMARY_FOR_SENIOR_APPROVAL_ja.md`（完全プロジェクトサマリー）
2. 読む: `shared/TEAM_SCOPE_DEFINITION_ja.md`（チーム境界と決定ツリー）
3. 確認: チームREADMEでタイムラインと成果物

---

## 🔗 関連ドキュメント

- **AI アシスタントコンテキスト:** `../CLAUDE.md`（包括的プロジェクトコンテキスト）
- **メインREADME:** `../README.md`（バイリンガルドキュメント構造）
- **英語原本:** `../multigo/`（100%翻訳済み - 87/87ファイル）

---

**最終更新:** 2025年12月20日
**ステータス:** ドキュメント完成、実装フェーズのシニア承認待ち
**管理者:** プロジェクトチームリード

---

**質問や clarification については、チーム別READMEファイルを参照するか、チームリードに直接連絡してください。**
