# ハードウェア開発ガイド

**チーム:** 土屋（ハードウェア）
**日付:** 2025-12-16
**バージョン:** 1.0

## ワークフロー

1. **要件レビュー** - 電気/機械要件を理解
2. **設計** - 回路図（KiCAD）またはCADモデル（SolidWorks）を作成
3. **レビュー** - チームリードによるピアレビュー
4. **プロトタイプ** - PCB、機械部品を発注
5. **テスト** - 仕様が満たされていることを確認
6. **ドキュメント** - 回路図、BOMを更新

## ファイル構成

```
hardware/
├── schematics/
│   ├── safety_controller/
│   ├── power_distribution/
│   └── ...
├── cad/
│   ├── chassis_frame.sldprt
│   ├── swerve_module.sldasm
│   └── ...
├── bom/
│   └── complete_bom.xlsx
└── documentation/
    ├── wiring_harness_spec.xlsx
    └── assembly_instructions.pdf
```

## バージョン管理

- 回路図: KiCADファイル用Git LFS
- CAD: STEPファイルをGitにエクスポート
- ソースファイル: チーム共有ドライブに保管

---
**参照:** 全ハードウェアアーキテクチャと設計文書
