# cobot280 - myCobot280 Enhanced Control Package

## 📋 概要
myCobot280の高機能制御パッケージ - シンプルな名前で強力な機能

## 🗂️ パッケージ構成

```
cobot280/
├── README.md                # このファイル
├── package.xml              # パッケージ情報
├── setup.py                 # Pythonパッケージ設定
├── cobot280/               # Pythonソースコード
│   ├── __init__.py
│   ├── cobot280_node.py        # メインノード
│   ├── gripper_control.py      # グリッパー制御
│   └── safety_manager.py       # 安全機能
├── launch/                  # 起動ファイル
├── config/                  # 設定ファイル
├── urdf/                    # URDFファイル
├── srv/                     # サービス定義
├── msg/                     # メッセージ定義
└── doc/                     # ドキュメント
    ├── gripper_urdf_spec.md    # グリッパーURDF仕様
    └── urdf_development/       # URDF開発資料
```

## 🎯 主要機能

### **基本制御**
- 6軸アーム制御（角度・座標・速度）
- フレキシブルグリッパー制御
- 同期・非同期動作制御

### **安全機能**
- **フリーモード**: ティーチング用の持続的脱力モード
- **サーボリリース**: 緊急時の一時的脱力
- 緊急停止機能

### **ROS2統合**
- TF配信（リンク・ジョイント情報）
- joint_state配信（RViz2対応）
- サービス・アクション対応

### **ティーチング機能**
- ドラッグ&ティーチモード
- 位置記録・再生
- 軌道教示

## 🚀 クイックスタート

### **1. ビルド**
```bash
cd /home/ros2/ros2_ws
colcon build --packages-select cobot280
source install/setup.bash
```

### **2. 起動**
```bash
# 基本制御ノード起動
ros2 launch cobot280 cobot280.launch.py

# RViz2込みで起動
ros2 launch cobot280 cobot280_rviz.launch.py
```

### **3. 基本操作**
```bash
# フリーモード開始（ティーチング）
ros2 service call /cobot280/set_free_mode cobot280_interfaces/srv/SetFreeMode "{free_mode: true}"

# 現在角度取得
ros2 topic echo /cobot280/joint_states

# グリッパー制御
ros2 service call /cobot280/set_gripper_value cobot280_interfaces/srv/SetGripperValue "{value: 50}"
```

## 📖 詳細仕様

- [グリッパーURDF仕様](doc/gripper_urdf_spec.md)
- [URDF開発資料](doc/urdf_development/)

## 🔧 開発状況

- ✅ ROS2パッケージ作成
- ✅ ディレクトリ構成
- ✅ ドキュメント整理
- 🔄 メインノード実装中
- ⏳ URDF完成
- ⏳ 安全機能実装
- ⏳ テスト・検証

## 📝 ライセンス
Apache-2.0

## 👥 開発者
- Takashi Otsuka (takatronix@gmail.com)
