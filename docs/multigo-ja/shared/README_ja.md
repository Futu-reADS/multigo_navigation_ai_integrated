# 共有ドキュメント - 屋外スワーブシステム

**使用目的:** 3チーム共通の参照ドキュメント

---

## 📁 フォルダ内容

```
shared/
├── 00_OVERVIEW/          ← プロジェクト概要とステータス
│   ├── PROJECT_STATUS.md
│   ├── VEHICLE_SYSTEM_OVERVIEW.md
│   ├── TEAM_RESPONSIBILITIES.md
│   └── DETAILED_IMPLEMENTATION_PLAN_2026.md
│
└── INTERFACES/           ← マスターインターフェース仕様（契約）
    ├── TVM_API_SPECIFICATION.md ✅ クリティカル
    ├── TVM_DATA_MODELS.md ✅ クリティカル
    ├── HARDWARE_SOFTWARE_INTERFACE.md ✅ クリティカル
    ├── ROS2_TOPICS.md
    └── INTERFACE_SPECIFICATIONS_COMPLETE.md
```

---

## 🎯 このフォルダに含まれるもの

### 1. 概要ドキュメント（`00_OVERVIEW/`）

**目的:** 全チーム向けのハイレベルプロジェクト情報

- **PROJECT_STATUS.md** - 現在のプロジェクトステータス、マイルストーン、進捗追跡
- **VEHICLE_SYSTEM_OVERVIEW.md** - システムアーキテクチャ概要、コンポーネント相互作用
- **TEAM_RESPONSIBILITIES.md** - 各チームの範囲の明確な定義
- **DETAILED_IMPLEMENTATION_PLAN_2026.md** - 完全な実装タイムラインとフェーズ

**使用者:**
- プロジェクトコンテキストのための全3チーム
- ステータスレビューのためのシニア
- オンボーディングのための新チームメンバー

---

### 2. インターフェースドキュメント（`INTERFACES/`）

**目的:** チーム間のインターフェース契約のマスターコピー

**⚠️ クリティカル - これらは契約です:**
- チーム間の調整なしに変更しないでください
- 変更には影響を受けるすべてのチームの承認が必要
- バージョン管理が必須

#### インターフェース: Pankaj ↔ Unno（TVM API）

**ドキュメント:**
- `TVM_API_SPECIFICATION.md`（1,425行） - 完全なREST + WebSocket API
- `TVM_DATA_MODELS.md`（1,361行） - JSONスキーマと検証ルール

**定義内容:**
- テレメトリーアップロード用のRESTエンドポイント（車両 → サーバー）
- ミッション配信用のWebSocketコマンド（サーバー → 車両）
- 認証プロトコル（JWT）
- エラーハンドリングとリトライロジック

**影響を受けるチーム:**
- **Pankaj:** TVMクライアントを実装（これらのAPIを呼び出す）
- **Unno:** TVMサーバーを実装（これらのAPIを提供する）

**使用方法:**
1. Pankajは何を呼び出すかを理解するために読む
2. Unnoは何を実装するかを理解するために読む
3. 両方がこれらの仕様に対してテスト
4. 統合テストが準拠を検証

---

#### インターフェース: Pankaj ↔ Tsuchiya+Kiril（ハードウェア-ソフトウェア）

**ドキュメント:**
- `HARDWARE_SOFTWARE_INTERFACE.md`（1,010行） - 完全なROS 2 + シリアル + CAN仕様

**定義内容:**
- ハードウェアが公開するROS 2トピック（センサー: LiDAR、カメラ、IMU、バッテリー、バンパー）
- ソフトウェアが公開するROS 2トピック（コマンド: 速度、操舵角）
- モーターコントローラー用のCANバスプロトコル
- eHMI用のシリアルプロトコル（ESP32通信）

**影響を受けるチーム:**
- **Pankaj:** センサートピックをサブスクライブ、コマンドトピックをパブリッシュ
- **Tsuchiya+Kiril:** センサートピックをパブリッシュ、コマンドトピックをサブスクライブ

**使用方法:**
1. Tsuchiyaは正しいトピックをパブリッシュするようにハードウェアをセットアップ
2. Pankajはトピックを消費/生成するソフトウェアを書く
3. 両方がメッセージフォーマットが仕様と一致することを検証
4. 統合テストがデータフローを検証

---

#### インターフェース: 内部ROS 2（Pankajのみ）

**ドキュメント:**
- `ROS2_TOPICS.md` - 車両ソフトウェア内の内部ROS 2ノード通信

**定義内容:**
- ナビゲーション、ドッキング、知覚ノード間のトピック
- メッセージタイプと周波数
- 命名規則

**影響を受けるチーム:**
- **Pankajのみ**（車両ソフトウェア内部）

**使用方法:**
- ROS 2ノードを設計する際に参照
- 一貫したトピック命名を確保
- メッセージタイプが一致することを検証

---

## 🔒 インターフェース変更プロトコル

**インターフェースを変更する必要がある場合:**

1. **影響を受けるチームを特定:**
   - TVM API変更: Pankaj + Unno
   - ハードウェア-ソフトウェア変更: Pankaj + Tsuchiya+Kiril

2. **変更を提案:**
   - 変更を文書化（旧 → 新）
   - なぜ必要かを説明
   - 後方互換性を検討

3. **承認を調整:**
   - 両チームが同意する必要がある
   - 大きな変更にはシニア承認

4. **マスタードキュメントを更新:**
   - `shared/INTERFACES/`（マスターコピー）で更新
   - 必要に応じてセマンティックバージョニングを使用（v1 → v2）
   - 移行パスを文書化

5. **すべてのチームに通知:**
   - 変更サマリーを投稿
   - チーム固有のコピーを更新
   - 統合テストをスケジュール

**例:**
```
変更: テレメトリーJSONに新しいフィールドを追加
影響: TVM_DATA_MODELS.md
チーム: Pankaj（車両が送信）+ Unno（サーバーが受信）
プロセス:
1. Pankajが提案: テレメトリーに"tire_pressure"フィールドを追加
2. Unnoがレビューして承認
3. shared/INTERFACES/TVM_DATA_MODELS.mdを更新
4. Pankajが車両クライアントコードを更新
5. Unnoがサーバーデータベーススキーマを更新
6. 新しいフィールドで統合テスト
```

---

## 📋 チームがこのフォルダを使用する方法

### Pankaj（車両ソフトウェア）

**最初に読む:**
1. `INTERFACES/TVM_API_SPECIFICATION.md` - Unnoのサーバーを呼び出す方法
2. `INTERFACES/HARDWARE_SOFTWARE_INTERFACE.md` - ハードウェアと通信する方法
3. `00_OVERVIEW/TEAM_RESPONSIBILITIES.md` - 自分の範囲を確認

**開発中に参照:**
- API呼び出しを実装する際にインターフェース仕様を確認
- トピック名が仕様と一致することを検証
- タイムラインについてプロジェクトステータスを確認

**自分のフォルダにコピー:**
- すべてのインターフェースドキュメントは`pankaj_vehicle_software/04_INTERFACES/`で参照

---

### Unno（TVMサーバー）

**最初に読む:**
1. `INTERFACES/TVM_API_SPECIFICATION.md` - 実装するAPI
2. `INTERFACES/TVM_DATA_MODELS.md` - 検証するJSONスキーマ
3. `00_OVERVIEW/TEAM_RESPONSIBILITIES.md` - 自分の範囲を確認

**開発中に参照:**
- 仕様通りにRESTエンドポイントを実装
- 仕様通りにWebSocketコマンドを実装
- 検証にJSONスキーマを使用
- 統合テストタイムラインについてプロジェクトステータスを確認

**自分のフォルダにコピー:**
- TVM APIとデータモデルは`unno_tvm_server/04_INTERFACES/`で参照

---

### Tsuchiya + Kiril（ハードウェア）

**最初に読む:**
1. `INTERFACES/HARDWARE_SOFTWARE_INTERFACE.md` - パブリッシュ/サブスクライブするROSトピック
2. `00_OVERVIEW/TEAM_RESPONSIBILITIES.md` - 自分の範囲を確認

**開発中に参照:**
- センサートピックが仕様と一致することを検証
- メッセージ周波数を確認（LiDAR 10Hz、IMU 50Hzなど）
- eHMIのシリアルプロトコルを検証
- 統合テストタイムラインについてプロジェクトステータスを確認

**自分のフォルダにコピー:**
- ハードウェア-ソフトウェアインターフェースは`tsuchiya_kiril_hardware/04_INTERFACES/`で参照

---

## ⚠️ 重要なルール

### 1. インターフェースドキュメントは凍結されています

**調整なしにこれらを変更しないでください:**
- ROS トピック名
- メッセージタイプ
- APIエンドポイント
- JSONスキーマ
- シリアルプロトコル

**理由:** 変更はチーム間の統合を壊します

---

### 2. マスターコピーは`shared/INTERFACES/`にあります

**チームフォルダには参照があり、コピーではありません:**
- `pankaj_vehicle_software/04_INTERFACES/` → 共有ドキュメントを参照
- `unno_tvm_server/04_INTERFACES/` → 共有ドキュメントを参照
- `tsuchiya_kiril_hardware/04_INTERFACES/` → 共有ドキュメントを参照

**不一致を見つけた場合:**
- `shared/INTERFACES/`バージョンが真実のソース
- 即座に不一致を報告
- チームフォルダを一致するように更新

---

### 3. 概要ドキュメントはコンテキストのみ

**`00_OVERVIEW/`のファイルは情報提供です:**
- プロジェクトステータス更新
- タイムライン参照
- チーム責任定義

**拘束力のある仕様ではありません:**
- 技術契約にはインターフェースドキュメントを使用
- プロジェクトコンテキストには概要ドキュメントを使用

---

## 📞 連絡先

**プロジェクトリード:** Pankaj（車両ソフトウェア）
**TVMサーバーリード:** Unno
**ハードウェアリード:** Tsuchiya（機械/電気）、Kiril（外装/eHMI）
**法務レビュー:** Li San（プライバシー要件）
**シニア:** [Name]（最終承認）

---

## 🔄 ドキュメント更新プロセス

### 概要ドキュメントの更新

**更新できる人:**
- Pankaj（プロジェクトステータス、タイムライン更新）
- 任意のチームリード（通知付き）

**プロセス:**
1. `shared/00_OVERVIEW/`に変更を加える
2. 他のチームに通知
3. 承認不要（情報提供のみ）

**例:**
- PROJECT_STATUS.mdを進捗とともに毎週更新

---

### インターフェースドキュメントの更新

**更新できる人:**
- 影響を受けるチーム間の調整後のみ

**プロセス:**
1. 変更を提案（正当化付き）
2. 影響を受けるすべてのチームから承認を得る
3. 大きな変更の場合はシニア承認を得る
4. `shared/INTERFACES/`（マスター）を更新
5. すべてのチームに通知
6. チーム固有の参照を更新
7. 統合テストをスケジュール

**例:**
- 新しいROSトピックの追加にはPankaj + Tsuchiya承認が必要

---

## ✅ クイックリファレンス

### どのインターフェースが必要ですか？

**Pankajの場合:**
- Unnoのサーバーを呼び出す必要がある？ → `TVM_API_SPECIFICATION.md`
- センサーを読む必要がある？ → `HARDWARE_SOFTWARE_INTERFACE.md`
- 内部ROSトピックが必要？ → `ROS2_TOPICS.md`

**Unnoの場合:**
- サーバーAPIを実装する必要がある？ → `TVM_API_SPECIFICATION.md`
- JSONスキーマが必要？ → `TVM_DATA_MODELS.md`

**TsuchiyaまたはKirilの場合:**
- センサーデータをパブリッシュする必要がある？ → `HARDWARE_SOFTWARE_INTERFACE.md`
- モーターコマンドを受信する必要がある？ → `HARDWARE_SOFTWARE_INTERFACE.md`

---

## 📊 インターフェースステータス

| インターフェースドキュメント | ステータス | 行数 | チーム | 最終更新 |
|-------------------|--------|-------|-------|--------------|
| TVM_API_SPECIFICATION.md | ✅ 本番準備完了 | 1,425 | Pankaj ↔ Unno | 2025年12月19日 |
| TVM_DATA_MODELS.md | ✅ 本番準備完了 | 1,361 | Pankaj ↔ Unno | 2025年12月19日 |
| HARDWARE_SOFTWARE_INTERFACE.md | ✅ 本番準備完了 | 1,010 | Pankaj ↔ Tsuchiya+Kiril | 2025年12月19日 |
| ROS2_TOPICS.md | ✅ 完了 | ~500 | Pankaj（内部） | 2025年12月19日 |

**すべてのインターフェースは凍結され、実装準備完了です。**

---

## 🚀 始め方

### 新しいチームメンバーの場合:

1. **プロジェクト概要を読む:**
   - `00_OVERVIEW/VEHICLE_SYSTEM_OVERVIEW.md` - システムを理解
   - `00_OVERVIEW/TEAM_RESPONSIBILITIES.md` - チームの範囲を理解

2. **チームのインターフェースを読む:**
   - Pankaj: TVM APIとハードウェア-ソフトウェアインターフェースの両方を読む
   - Unno: TVM APIインターフェースを読む
   - Tsuchiya+Kiril: ハードウェア-ソフトウェアインターフェースを読む

3. **チームフォルダに移動:**
   - `pankaj_vehicle_software/README.md` - ソフトウェアチームの場合はここから開始
   - `unno_tvm_server/README.md` - サーバーチームの場合はここから開始
   - `tsuchiya_kiril_hardware/README.md` - ハードウェアチームの場合はここから開始

---

## 📝 バージョン履歴

**バージョン 1.0**（2025年12月19日）
- 初期共有フォルダ構造
- 3つのインターフェースドキュメントが確定
- 概要ドキュメントが統合

---

**ドキュメントステータス:** ✅ 完了
**管理者:** Pankaj（プロジェクトリード）
**最終更新:** 2025年12月19日

---

**質問がありますか？**
- インターフェース変更: 影響を受けるチームと調整
- プロジェクトステータス: `00_OVERVIEW/PROJECT_STATUS.md`を確認
- 範囲の質問: `00_OVERVIEW/TEAM_RESPONSIBILITIES.md`を確認

---

**共有ドキュメントREADME終了**
