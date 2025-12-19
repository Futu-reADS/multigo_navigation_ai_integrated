# 開発セットアップガイド

**全チーム**
**日付:** 2025-12-16
**バージョン:** 1.0

## 前提条件

### 全チーム
- Git 2.40+
- Docker 24+ および Docker Compose
- VS Code または好みの IDE

### 車両ソフトウェア（Pankaj）
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.10+
- C++コンパイラ（GCC 11+）

### フリート管理（Unno）
- Node.js 20 LTS
- PostgreSQL 15
- Redis 7.x

### ハードウェア（Tsuchiya）
- KiCAD 7+（PCB設計）
- SolidWorks または Fusion 360（CAD）

## リポジトリ構造

```
multigo_navigation_ai_integrated/
├── src/                  # ROS 2 パッケージ（車両ソフトウェア）
├── fleet_management/     # TVM サーバー + フリート UI
├── hardware/            # 回路図、CAD ファイル
└── docs/                # このドキュメント
```

## 開発環境セットアップ

### 1. リポジトリのクローン
```bash
git clone https://github.com/organization/multigo_navigation_ai_integrated.git
cd multigo_navigation_ai_integrated
```

### 2. チーム別セットアップ

**車両ソフトウェア:**
```bash
cd src/
rosdep install --from-paths . --ignore-src -r -y
colcon build
source install/setup.bash
```

**フリート管理:**
```bash
cd fleet_management/tvm-server/
npm install
cp .env.example .env  # 環境変数を設定
docker-compose up -d  # PostgreSQL + Redis を起動
npm run dev           # 開発サーバーを起動
```

## 開発ワークフロー

1. フィーチャーブランチを作成: `git checkout -b feature/issue-123-description`
2. 変更を加える
3. テストを実行: `colcon test`（車両）または `npm test`（フリート）
4. Conventional Commits でコミット: `feat: add new feature`
5. プッシュしてプルリクエストを作成
6. CI/CD チェックが通過するまで待つ
7. チームリードにレビューを依頼

---
**参照:** .claude/WORKFLOW.md, CLAUDE.md
