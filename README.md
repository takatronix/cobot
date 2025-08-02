# myCobot 280 ROS2 Control System

**完全統合制御システム - 安全性と使いやすさを重視**

## 🚀 概要

myCobot 280 Pi用の包括的なROS2制御システムです。安全性、使いやすさ、拡張性を重視して設計されており、手動制御からAI統合まで幅広い用途に対応します。

### ✨ 主要機能

**🔧 制御機能**
- **6軸アーム制御**: 精密な関節制御と座標制御
- **フレキシブルグリッパー制御**: 把持力調整と状態監視
- **位置管理システム**: 保存・読み込み・リスト・削除
- **安全システム**: 関節制限チェックと自動修正

**🎮 操作モード**
- **Manual** 🟡: 手動ティーチング・操作
- **Auto** 🔵: 自動実行モード  
- **AI** 🟣: AI統合制御
- **Calibration** 🩷: キャリブレーション
- **Emergency** 🔴: 緊急停止

**🛡️ 安全機能**
- **リアルタイム安全チェック**: 関節制限監視
- **自動角度修正**: 限界超過時の自動クリッピング
- **緊急停止システム**: 即座の動作停止
- **LED視覚フィードバック**: モード別色分け表示

## 📁 プロジェクト構造

```
cobot/
├── ros2_ws/src/cobot/        # ROS2パッケージ
│   ├── cobot/                # メインノード
│   │   ├── cobot_node.py     # 統合制御ノード
│   │   ├── gripper_control.py # グリッパー制御
│   │   └── ...
│   ├── msg/                  # カスタムメッセージ
│   ├── srv/                  # カスタムサービス
│   └── urdf/                 # ロボットモデル
├── scripts/                  # 制御スクリプト
│   ├── home                  # ホーム移動
│   ├── manual/auto/ai        # モード切り替え
│   ├── save/goto/list        # 位置管理
│   └── status/stop           # 状態確認・停止
├── positions/                # サンプル位置データ
└── docs/                     # ドキュメント
```

## 🚀 クイックスタート

### 1. インストール

```bash
# リポジトリクローン
git clone https://github.com/takatronix/cobot.git
cd cobot

# ROS2ワークスペースにコピー
cp -r ros2_ws /home/ros2/
cd /home/ros2/ros2_ws

# ビルド
colcon build --packages-select cobot
source install/setup.bash
```

### 2. 制御スクリプトセットアップ

```bash
# スクリプトをホームディレクトリにコピー
cp -r cobot/scripts ~/cobot
chmod +x ~/cobot/*

# 位置データディレクトリ作成
mkdir -p ~/cobot/positions
cp cobot/positions/* ~/cobot/positions/
```

### 3. 起動・基本操作

```bash
# ノード起動
ros2 run cobot cobot_node

# 基本操作（別端末で）
cd ~/cobot
./status        # 状態確認
./manual        # マニュアルモード
./home          # ホーム移動
./save my_pose  # 現在位置を保存
./goto my_pose  # 保存位置に移動
./list          # 保存位置一覧
```

## 🎮 制御スクリプト

| スクリプト | 機能 | 使用例 |
|------------|------|--------|
| `./status` | ロボット状態確認 | `./status` |
| `./manual` | マニュアルモード | `./manual` |
| `./auto` | 自動モード | `./auto` |
| `./ai` | AIモード | `./ai` |
| `./home` | ホーム移動 | `./home` |
| `./stop` | 緊急停止 | `./stop` |
| `./save` | 位置保存 | `./save work_position` |
| `./goto` | 位置移動 | `./goto work_position` |
| `./list` | 位置一覧 | `./list` |
| `./delete` | 位置削除 | `./delete old_position` |

## 🔧 ROS2インターフェース

**📡 主要トピック**
- `/cobot/status` - リアルタイム状態情報（10Hz）
- `/cobot/save_position` - 位置保存指令

**🔧 主要サービス**  
- `/cobot/manual|auto|ai` - モード切り替え
- `/cobot/home` - ホーム移動（全モード対応）
- `/cobot/save|goto_*|list|delete` - 位置管理

**📖 詳細仕様**: [ROS2 API仕様書](docs/ROS2_API.md) を参照

## 🛡️ 安全システム

### 関節制限
```
J1: ±180°  J2: ±135°  J3: ±135°
J4: ±150°  J5: ±145°  J6: ±165°
```

### 安全モード
- **strict**: 制限超過時は保存ブロック
- **loose**: 制限超過時は自動修正して保存（デフォルト）
- **off**: 安全チェック無効

## 🎯 LED状態表示

| モード | 色 | 説明 |
|--------|----|----- |
| Manual | 🟡 黄色 | 手動操作可能 |
| Auto | 🔵 青色 | 自動実行中 |
| AI | 🟣 紫色 | AI制御中 |
| Calibration | 🩷 ピンク | キャリブレーション |
| Emergency | 🔴 赤色 | 緊急停止 |
| Init | 💙 シアン | 初期化中 |

## 📍 位置管理

### 基本位置
- `home` - ホームポジション（安全制限内に修正済み）
- `safe` - 安全ポジション  
- `ready` - 準備ポジション
- `open` - グリッパー開放位置
- `close` - グリッパー閉じ位置

### 位置保存形式
```json
{
  "name": "position_name",
  "angles": [-48.77, 135, -135, -12.39, 61.87, -62.75],
  "description": "",
  "gripper_value": 38,
  "created_at": "Sat Aug  2 16:59:53 2025"
}
```

## 🤖 AI統合対応

このシステムはVision-Language-Action（VLA）モデルとの統合を想定して設計されています：

- **純粋制御インターフェース**: AIシステムが直接制御可能
- **状態フィードバック**: リアルタイム状態監視
- **安全層**: AI出力の安全性検証
- **位置記憶**: AI学習用の位置データ管理

## 🔧 開発・カスタマイズ

### 新しい位置の追加
```bash
# 手動でロボットを目標位置に移動
./manual
# 位置を保存
./save new_position_name
```

### カスタムモードの追加
`ros2_ws/src/cobot/cobot/cobot_node.py` の `RobotMode` enum と対応する処理を追加

### 安全制限の調整
`SafetyChecker` クラスの `joint_limits` を修正

## 📞 サポート・貢献

- **開発者**: Takashi Otsuka (takatronix@gmail.com)
- **リポジトリ**: [https://github.com/takatronix/cobot](https://github.com/takatronix/cobot)
- **Issues**: GitHubのIssuesで報告

## 📄 ライセンス

MIT License - 詳細は `LICENSE` ファイルを参照

---

**⚡ 高速・安全・直感的な myCobot 制御システム ⚡**