# 機械CAD設計リファレンス

**チーム:** Tsuchiya（ハードウェア）
**日付:** 2025-12-16
**バージョン:** 1.0

## CADファイル

機械設計はSolidWorks/Fusion 360で管理されています:

1. **メインアセンブリ** (`cad/robot_assembly.sldasm`)
   - 完全ロボットアセンブリ
   - すべてのサブアセンブリ統合
   - 部品表（BOM）

2. **シャーシフレーム** (`cad/chassis_frame.sldprt`)
   - 40mm × 40mmアルミニウム押出フレーム
   - 取り付け穴とブラケット
   - 材料: 6061-T6アルミニウム

3. **スワーブドライブモジュール** (`cad/swerve_module.sldasm`)
   - ドライブモーターアセンブリ
   - ステアモーターとベルトドライブ
   - スイベルベアリングハウジング
   - ホイールとタイヤ

4. **ドッキング機構** (`cad/docking_mechanism.sldasm`)
   - 折り畳みアーム（サーボ駆動）
   - 係合ピン
   - アライメントガイド
   - センサーマウント

5. **エンクロージャー** (`cad/enclosures/`)
   - バッテリーエンクロージャー（IP65）
   - コンピュートユニットエンクロージャー（IP54）
   - ディスプレイエンクロージャー
   - すべての取り付けハードウェア

## 製造図面

加工用技術図面:
- `drawings/chassis_frame.pdf` - フレーム加工図面
- `drawings/mounting_brackets.pdf` - ブラケット設計
- `drawings/enclosure_panels.pdf` - パネル切断図面

---
**参照:** MECHANICAL_SYSTEM_ARCHITECTURE.md、MECHANICAL_REQUIREMENTS.md（99要件）
