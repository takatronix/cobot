# フレキシブルグリッパーURDF完成仕様書

## 🎯 目標
myCobot280用フレキシブルグリッパーの完全なURDFモデルを作成する

## 📋 現状と計画

### ✅ 完了済み
- Adaptive Gripperの3Dモデルファイルをコピー完了
- 場所: `/home/ros2/ros2_ws/src/mycobot_ros2/mycobot_description/urdf/flexible_gripper_models/`

### 📁 利用可能な3Dモデルファイル (.dae)
```
flexible_gripper_models/
├── gripper_base.dae      (2.3MB) ← グリッパーベース部分
├── gripper_left1.dae     (340KB) ← 左指 第1セクション  
├── gripper_left2.dae     (421KB) ← 左指 第2セクション
├── gripper_left3.dae     (853KB) ← 左指 第3セクション
├── gripper_right1.dae    (340KB) ← 右指 第1セクション
├── gripper_right2.dae    (421KB) ← 右指 第2セクション
└── gripper_right3.dae    (452KB) ← 右指 第3セクション
```

## 🛠️ .daeファイルについて

### **.daeファイルとは？**
- **COLLADA** (COLLAborative Design Activity) フォーマット
- XML形式の3Dモデルファイル
- ジオメトリ、材質、テクスチャ、アニメーション情報を含む
- ROS/URDFでサポートされている標準形式

### **作成方法**
1. **既存ファイルをコピー** ← 今回はこれ ✅
2. **3D CADソフトで作成:**
   - Blender (無料) → .daeエクスポート
   - Fusion 360 → .daeエクスポート
   - SolidWorks → .daeエクスポート
3. **メッシュ変換:**
   - .stl → .dae変換
   - .obj → .dae変換

## 🔧 フリーモード・サーボリリース仕様

### **フリーモード制御**
```python
# ティーチングモード (持続的)
set_free_mode(1)    # フリーモード開始
is_free_mode()      # 状態確認 (0/1)
set_free_mode(0)    # フリーモード終了
```
**用途**: ドラッグ&ティーチ、手動位置教示

### **サーボリリース制御**  
```python
# 緊急脱力 (一時的)
release_all_servos()        # 全関節脱力 (デフォルト: ダンピングあり)
release_all_servos(1)       # 全関節脱力 (ダンピングなし)
release_servo(joint_id)     # 単一関節脱力
```
**用途**: 緊急停止、安全確保、メンテナンス

### **ROS2サービス仕様**
```bash
# 既存実装済み (enhanced_listen_real_service.py)
/set_free_mode     # SetFreeMode.srv
/get_free_mode     # GetFreeMode.srv

# 追加予定
/release_servos    # ReleaseServos.srv (新規作成)
/emergency_stop    # EmergencyStop.srv (新規作成)
```

## 🎯 次のステップ

### Phase 1: URDF改良
1. 既存の`mycobot_280_pi_with_flexible_gripper.urdf`を修正
2. 簡易box形状 → 3D .daeモデルに変更
3. 適切なリンク・ジョイント構成

### Phase 2: ROS2ノード拡張
1. フリーモード制御の完全実装
2. サーボリリース機能追加
3. 緊急停止機能実装

### Phase 3: テスト・検証
1. RViz2での表示確認
2. 実機での動作テスト
3. MoveIt2統合テスト

## 📝 技術仕様

### **myCobot280 Enhanced Control Node**
```yaml
パッケージ名: cobot280
主要機能:
  - 6軸アーム制御 (角度・座標・速度)
  - フレキシブルグリッパー制御
  - フリーモード・ティーチング機能
  - サーボリリース・緊急停止
  - TF配信・joint_state配信
  - 診断・ログ機能
```

### **URDFモデル仕様**
```yaml
ベースモデル: mycobot_280_pi.urdf
グリッパー: flexible_gripper (7パーツ構成)
TF構成: base_link → link1~6 → gripper_base → finger_links
関節数: 6軸 + グリッパー関節
```