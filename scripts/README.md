# myCobot280 Enhanced Control System 🤖

## 📋 概要
myCobot280の高機能制御システムの開発資料とAPI仕様書

## 🗂️ ファイル構成

### URDFモデル
```
ros2_ws/src/mycobot_ros2/mycobot_description/urdf/mycobot_280_pi/
├── mycobot_280_pi.urdf                    # 基本モデル（6軸のみ）
├── mycobot_280_pi_with_pump.urdf          # ポンプ付きモデル（完成）
├── mycobot_280_pi_with_camera_flange.urdf # カメラフランジ付き（完成）
└── mycobot_280_pi_with_flexible_gripper.urdf # フレキシブルグリッパー付き（完成）
```

**重要**: グリッパー込みURDFは**完成済み**です！
- `mycobot_280_pi_with_flexible_gripper.urdf` にフレキシブルグリッパーの完全定義あり
- 2本指グリッパー、リンク・ジョイント・慣性パラメータ全て定義済み
- `mimic joint`でグリッパー連動制御も実装済み

### 既存ROS2ノード
```
ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280pi/mycobot_280pi/
├── enhanced_listen_real_service.py  # 拡張サービスノード（フリーモード対応済み）
├── follow_display.py               # 関節状態パブリッシャー
└── slider_control.py              # スライダー制御
```

## 🛠️ PyMyCobot API 機能一覧

### 🎯 基本制御機能

#### 関節制御
```python
# 基本動作
get_angles()              # 現在の関節角度取得
send_angles(angles, speed) # 関節角度設定（非同期）
sync_send_angles(angles, speed) # 関節角度設定（同期）

# 座標制御  
get_coords()              # 現在のエンドエフェクタ座標取得
send_coords(coords, speed) # 座標指定移動（非同期）
sync_send_coords(coords, speed) # 座標指定移動（同期）

# 速度制御
get_speed()               # 現在速度取得
set_speed(speed)          # 速度設定

# JOG制御
jog_angle(joint_id, direction, speed) # 関節単体JOG
jog_coord(axis, direction, speed)     # 座標軸JOG
jog_stop()                           # JOG停止
```

#### 状態監視
```python
# 動作状態
is_moving()               # 動作中かチェック
is_in_position(coords, threshold) # 位置到達チェック
is_paused()               # 一時停止状態チェック

# システム状態
version()                 # ファームウェアバージョン
get_error_information()   # エラー情報取得
clear_error_information() # エラー情報クリア
```

### 🔧 電源・サーボ制御

#### 電源管理
```python
power_on()                # 電源ON
power_off()               # 電源OFF  
is_power_on()             # 電源状態確認
```

#### サーボ制御
```python
# 全体制御
release_all_servos()      # 全サーボリリース（脱力状態）
focus_all_servos()        # 全サーボロック（保持状態）

# 個別制御  
release_servo(servo_id)   # 個別サーボリリース
focus_servo(servo_id)     # 個別サーボロック
is_servo_enable(servo_id) # サーボ有効状態確認

# サーボ診断
get_servo_speeds()        # 全サーボ速度取得
get_servo_voltages()      # 全サーボ電圧取得  
get_servo_temps()         # 全サーボ温度取得
get_servo_status()        # 全サーボ状態取得
```

**🚨 緊急停止 vs サーボリリースの違い**
- **緊急停止(`jog_stop()`)**: 動作を即座に停止、サーボは保持状態維持
- **サーボリリース(`release_all_servos()`)**: サーボを脱力状態にして手動操作可能にする
- **用途**: 緊急停止は安全停止、サーボリリースはティーチング時に使用

### 🤏 グリッパー制御

#### 基本グリッパー制御
```python
# 状態制御
set_gripper_state(flag, speed)  # グリッパー開閉（0=閉じる, 1=開く）
set_gripper_value(value, speed) # グリッパー開度設定（0-100）
get_gripper_value()            # 現在のグリッパー開度取得
is_gripper_moving()            # グリッパー動作中チェック

# 校正・設定
set_gripper_calibration()      # グリッパー校正
set_gripper_protect_current(current) # 保護電流設定
```

#### 高度なグリッパー制御（一部モデル）
```python
# HTS グリッパー（高トルクサーボ）
set_HTS_gripper_torque(torque) # トルク設定
get_HTS_gripper_torque()       # トルク取得
init_gripper()                 # グリッパー初期化
```

### 🎓 ティーチング機能

#### フリーモード
```python
set_free_mode(flag)           # フリーモード切替（True=手動操作可能）
is_free_mode()                # フリーモード状態確認
```

#### ドラッグティーチ（一部モデル）
```python
drag_teach_save()             # ドラッグティーチ記録開始
drag_teach_execute()          # 記録した動作実行
drag_teach_pause()            # 記録一時停止
drag_teach_clean()            # 記録データクリア
```

### 📊 I/O制御

#### デジタルI/O
```python
set_digital_output(pin, state) # デジタル出力設定
get_digital_input(pin)         # デジタル入力読取

# 基本I/O
set_basic_output(pin, state)   # 基本出力設定
get_basic_input(pin)           # 基本入力読取
```

#### アナログ・PWM
```python
set_pwm_output(pin, value)     # PWM出力設定
set_color(r, g, b)            # LED色設定（Atom搭載モデル）
```

### 🔧 高度な制御機能

#### エンコーダー制御
```python
get_encoder(servo_id)          # エンコーダー値取得
set_encoder(servo_id, value, speed) # エンコーダー値設定
get_encoders()                 # 全エンコーダー値取得
set_encoders(encoders, speed)  # 全エンコーダー値設定
```

#### 制御パラメータ
```python
# 関節限界
get_joint_min_angle(joint_id)  # 関節最小角度取得
get_joint_max_angle(joint_id)  # 関節最大角度取得

# サーボパラメータ
set_servo_data(servo_id, data_id, value) # サーボパラメータ設定
get_servo_data(servo_id, data_id)        # サーボパラメータ取得
set_servo_calibration(servo_id)          # サーボ校正
```

## 🏗️ 推奨ノード設計

### パッケージ名
```
mycobot280_enhanced_control
```

### ノード構成
```
mycobot280_enhanced_controller  # メインコントローラーノード
├── joint_state_publisher       # 関節状態パブリッシャー  
├── gripper_controller          # グリッパー制御
├── teaching_mode_manager       # ティーチングモード管理
└── safety_monitor             # 安全監視機能
```

### ROS2インターフェース
```yaml
Topics:
  Publishers:
    /joint_states: sensor_msgs/JointState
    /gripper_state: custom_msgs/GripperState  
    /robot_diagnostics: diagnostic_msgs/DiagnosticArray
    
  Subscribers:
    /joint_trajectory: trajectory_msgs/JointTrajectory
    /gripper_command: custom_msgs/GripperCommand

Services:
  /set_free_mode: std_srvs/SetBool
  /emergency_stop: std_srvs/Trigger
  /set_joint_angles: custom_msgs/SetJointAngles
  /calibrate_gripper: std_srvs/Trigger

Actions:
  /follow_joint_trajectory: control_msgs/FollowJointTrajectory
  /move_to_pose: custom_msgs/MoveToPose
```

## 🎯 MoveIt2統合について

**MoveIt2は別ノード**として実装するのが正解です：

### 理由
1. **責任分離**: 低レベル制御とモーションプランニングを分離
2. **再利用性**: 他のプランナーとも連携可能
3. **安定性**: MoveIt2の問題が基本制御に影響しない

### 推奨構成
```
mycobot280_enhanced_control   # 基本制御（このプロジェクト）
├── ハードウェア制御
├── 安全機能
└── 基本I/O

mycobot280_moveit2_config     # MoveIt2設定（別パッケージ）
├── planning_context.launch.py
├── move_group.launch.py  
└── config/
    ├── joint_limits.yaml
    ├── kinematics.yaml
    └── ompl_planning.yaml
```

### 連携方法
- **基本制御ノード** → `/joint_states`をパブリッシュ
- **MoveIt2** → `/follow_joint_trajectory`アクションで制御指令
- **URDFモデル** → `mycobot_280_pi_with_flexible_gripper.urdf`を使用

## 🚀 実装優先順位

### Phase 1: 基本機能
1. ✅ 関節状態パブリッシャー
2. ✅ 基本関節制御
3. ✅ グリッパー制御
4. ✅ フリーモード

### Phase 2: 安全・診断機能  
1. 緊急停止実装
2. サーボ診断機能
3. エラーハンドリング強化

### Phase 3: 高度な機能
1. ドラッグティーチ
2. I/O制御
3. MoveIt2統合

## 🔍 参考リソース

### 公式ドキュメント
- [Elephant Robotics Gitbook](https://docs.elephantrobotics.com/)
- [PyMyCobot Documentation](https://github.com/elephantrobotics/pymycobot)

### 既存実装例
- [yamachaso/mycobot_ros](https://github.com/yamachaso/mycobot_ros) - 非公式ROS実装
- [thulsonASU drag_n_teach](https://github.com/thulsonASU/Drag_n_teach-ROS-Pkg-for-mycobot_280_M5) - ティーチング機能
- [mertcookimg/mycobot_controller](https://github.com/mertcookimg/mycobot_controller) - MoveIt統合例

---
**最終更新**: 2025年1月25日  
**作成者**: AI Assistant + User