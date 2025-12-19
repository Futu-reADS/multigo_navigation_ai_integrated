# 車両ソフトウェア開発ガイド

**チーム:** パンカジ（車両ソフトウェア）
**日付:** 2025-12-16
**バージョン:** 1.0

## ROS 2パッケージ構造

```
src/
├── aruco_detect/        # ArUcoマーカー検出
├── nav_control/         # ナビゲーションコントローラー
├── nav_docking/         # ドッキングコントローラー
├── nav_goal/            # ゴール管理
├── swerve_drive/        # スワーブドライブコントローラー
└── tvm_client/          # TVMフリートクライアント
```

## ビルドとテスト

```bash
# 全パッケージをビルド
colcon build

# 特定パッケージをビルド
colcon build --packages-select nav_control

# テストを実行
colcon test

# 特定テストを実行
colcon test --packages-select nav_control
```

## コード標準

- **C++:** Google C++スタイルガイド
- **Python:** PEP 8
- **ROS 2:** ROS 2ベストプラクティスに従う
- **テスト:** ≥80%コードカバレッジ

## 一般的な開発タスク

### 新しいROS 2ノードの作成
```bash
cd src/
ros2 pkg create --build-type ament_cmake my_package
```

### 依存関係の追加
`package.xml`と`CMakeLists.txt`を編集後:
```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

---
**参照:** 全車両要件と設計文書
