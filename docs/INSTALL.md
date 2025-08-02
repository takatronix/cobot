# myCobot 280 ROS2 インストール・セットアップガイド

## 📋 前提条件

### ハードウェア要件
- **myCobot 280 Pi** (Raspberry Pi版)
- **フレキシブルグリッパー** (推奨)
- **Raspberry Pi 4** (4GB以上推奨)
- **Ubuntu 22.04** + **ROS2 Humble**

### ソフトウェア要件
- ROS2 Humble Hawksbill
- Python 3.10+
- PyMyCobot ライブラリ
- Git

## 🚀 詳細インストール手順

### 1. システム準備

```bash
# システム更新
sudo apt update && sudo apt upgrade -y

# 必要パッケージインストール
sudo apt install -y git python3-pip python3-colcon-common-extensions

# ROS2 Humble（未インストールの場合）
# https://docs.ros.org/en/humble/Installation.html
```

### 2. PyMyCobot ライブラリインストール

```bash
# PyMyCobot インストール
pip3 install pymycobot

# シリアル権限設定
sudo usermod -a -G dialout $USER
# 再ログインまたは再起動が必要
```

### 3. cobotシステム インストール

```bash
# 作業ディレクトリへ移動
cd /home/ros2  # または適切なワークスペース

# リポジトリクローン
git clone https://github.com/takatronix/cobot.git

# ROS2ワークスペースセットアップ
mkdir -p ros2_ws/src
cp -r cobot/ros2_ws/src/cobot ros2_ws/src/

# ビルド
cd ros2_ws
colcon build --packages-select cobot

# 環境設定
source install/setup.bash
echo "source /home/ros2/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 4. 制御スクリプトセットアップ

```bash
# ホームディレクトリにcobotフォルダ作成
mkdir -p ~/cobot

# スクリプトコピー
cp /home/ros2/cobot/scripts/* ~/cobot/
chmod +x ~/cobot/*

# 位置データディレクトリ準備
mkdir -p ~/cobot/positions
cp /home/ros2/cobot/positions/* ~/cobot/positions/
```

### 5. ロボット接続設定

```bash
# シリアルポート確認
ls /dev/ttyAMA* /dev/ttyUSB*

# 通常は /dev/ttyAMA0 (Pi版の場合)
# 必要に応じて cobot_node.py の設定を調整
```

## 🔧 初期設定・テスト

### 1. ロボット接続テスト

```bash
# 新しい端末でノード起動
cd ~/ros2_ws
source install/setup.bash
ros2 run cobot cobot_node
```

### 2. 基本動作確認

```bash
# 別の端末で
cd ~/cobot

# 状態確認
./status

# マニュアルモード
./manual

# ホーム移動（全モードで動作）
./home
```

### 3. 位置保存・移動テスト

```bash
# 現在位置を保存
./save test_position

# 保存位置に移動
./goto test_position

# 位置一覧確認
./list
```

## ⚙️ 設定ファイル

### cobot_node.py 主要設定

```python
# シリアルポート設定
PORT = "/dev/ttyAMA0"
BAUDRATE = 1000000

# 更新頻度
PUBLISH_RATE = 10.0  # Hz

# 安全モード
save_safety_mode = "loose"  # strict/loose/off
```

### 関節制限値

```python
joint_limits = [
    (-180, 180),  # J1: ±180°
    (-135, 135),  # J2: ±135° 
    (-135, 135),  # J3: ±135°
    (-150, 150),  # J4: ±150°
    (-145, 145),  # J5: ±145°
    (-165, 165)   # J6: ±165°
]
```

## 🛠️ トラブルシューティング

### よくある問題

**1. シリアル接続エラー**
```bash
# 権限確認
groups $USER
# dialoutグループが含まれていることを確認

# ポート確認
ls -la /dev/ttyAMA0
sudo chmod 666 /dev/ttyAMA0  # 一時的解決法
```

**2. ノードが起動しない**
```bash
# 依存関係確認
pip3 list | grep pymycobot

# ROS2環境確認
echo $ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
```

**3. ロボットが応答しない**
```bash
# ロボット電源・接続確認
# ATOMの画面表示を確認
# 緊急停止ボタンの状態確認

# ノード再起動
pkill -f cobot_node
ros2 run cobot cobot_node
```

**4. 位置保存ができない**
```bash
# ディレクトリ権限確認
ls -la ~/cobot/positions/
chmod 755 ~/cobot/positions/

# 安全チェック設定確認（strict→looseに変更）
```

### ログ確認

```bash
# ROS2ログ
ros2 node list
ros2 topic list
ros2 service list

# cobot状態確認
ros2 topic echo /cobot/status --once
```

## 🔄 システム更新

```bash
# リポジトリ更新
cd /home/ros2/cobot
git pull origin main

# パッケージ再ビルド
cd /home/ros2/ros2_ws
colcon build --packages-select cobot

# スクリプト更新
cp /home/ros2/cobot/scripts/* ~/cobot/
chmod +x ~/cobot/*
```

## 🎯 パフォーマンス最適化

### 1. 起動高速化
```bash
# setup.bashの読み込みを避ける（スクリプトで既に設定済み）
# 不要なノードの停止
```

### 2. 応答性向上
```bash
# 更新頻度調整（cobot_node.py）
PUBLISH_RATE = 20.0  # より高頻度に

# CPUリソース確認
htop
```

## 📞 サポート

問題が解決しない場合：

1. **GitHub Issues**: [https://github.com/takatronix/cobot/issues](https://github.com/takatronix/cobot/issues)
2. **ログ情報**: エラーメッセージと実行環境の詳細を記載
3. **連絡先**: takatronix@gmail.com

---

**💡 ヒント**: 初回セットアップ後は `./status` で正常性を確認し、`./manual`→`./home` の基本動作テストを推奨します。