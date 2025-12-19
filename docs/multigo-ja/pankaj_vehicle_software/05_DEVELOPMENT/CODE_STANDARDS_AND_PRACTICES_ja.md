# コード標準とプラクティス

**全チーム**
**日付:** 2025-12-16
**バージョン:** 1.0

## 一般原則

1. **テストを最初に書く**（適用可能な場合はTDD）
2. **コードレビューが必要**（マージ前に≥1名の承認者）
3. **関数を小さく保つ**（<50行）
4. **意味のある名前**（`temp`、`data`、`foo`は不可）
5. **複雑なロジックを文書化**（コメントはWHYを説明、WHATではない）

## Gitコミットメッセージ

Conventional Commitsを使用:
```
feat: add ArUco docking controller
fix: correct CAN bus termination issue
docs: update API specification
test: add integration tests for docking
refactor: simplify swerve drive kinematics
```

## コードレビューチェックリスト

- [ ] テストが通る（`colcon test`または`npm test`）
- [ ] コードカバレッジ ≥80%
- [ ] lintエラーなし
- [ ] ドキュメントが更新されている
- [ ] セキュリティ脆弱性なし（Dependabot、Snyk）

## テスト要件

- **ユニットテスト:** すべての外部依存関係をモック
- **統合テスト:** コンポーネント間の相互作用をテスト
- **E2Eテスト:** フルワークフローをテスト（CI/CDで使用）

---
**参照:** .claude/WORKFLOW.md, TESTING_REQUIREMENTS.md
