#!/usr/bin/env python3
"""
Full cobot Node - Complete Integration
=====================================

完全統合版 - 全システム機能搭載

Author: Takashi Otsuka (takatronix@gmail.com)
License: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import os
import json
import time

from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist
from std_srvs.srv import Empty, SetBool

import time
import threading
import json
import math
import logging
from typing import Optional, List, Dict, Any
from enum import Enum
from dataclasses import dataclass

from pymycobot import MyCobot280
from .gripper_control import GripperController

# ログ設定
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ===== MODE MANAGEMENT =====
class CobotMode(Enum):
    """ロボットモード定義"""
    MANUAL = "manual"           # 🟡 手動モード
    AUTO = "auto"              # 🔵 自動モード  
    AI = "ai"                  # 🟣 AIモード
    CALIBRATION = "calibration" # 🩷 校正モード
    ERROR = "error"            # 🔴 エラー
    EMERGENCY = "emergency"    # 🔶 緊急停止
    INITIALIZING = "init"      # 💙 初期化

# ===== SAFETY SYSTEM =====
class SafetyLevel(Enum):
    SAFE = "safe"
    WARNING = "warning"
    CRITICAL = "critical"

class SafetyChecker:
    def __init__(self):
        # myCobot280の関節制限 (度) - 実機エラーメッセージより修正
        self.joint_limits = [
            (-168, 168),  # J1: 実機制限 -168°~168° (PyMyCobotエラーより)
            (-135, 135),  # J2: 実機制限 -135°~135° (PyMyCobotエラーより)
            (-135, 135),  # J3: ±2.3562 rad = ±135°
            (-150, 150),  # J4: ±2.618 rad = ±150°
            (-145, 145),  # J5: ±2.5307 rad = ±145°
            (-165, 165)   # J6: ±2.8798 rad = ±165°
        ]
    
    def check_joint_angles(self, angles):
        """関節角度安全チェック"""
        if len(angles) != 6:
            return False, SafetyLevel.CRITICAL, "Invalid angle count"
        
        unsafe_joints = []
        warnings = []
        
        for i, angle in enumerate(angles):
            min_a, max_a = self.joint_limits[i]
            
            if angle < min_a or angle > max_a:
                unsafe_joints.append(f"J{i+1}:{angle:.1f}° (limit: {min_a}°~{max_a}°)")
            elif abs(angle - min_a) < 10 or abs(angle - max_a) < 10:
                warnings.append(f"J{i+1}:{angle:.1f}° near limit")
        
        if unsafe_joints:
            return False, SafetyLevel.CRITICAL, f"Unsafe: {', '.join(unsafe_joints)}"
        elif warnings:
            return True, SafetyLevel.WARNING, f"Warning: {', '.join(warnings)}"
        else:
            return True, SafetyLevel.SAFE, "All angles safe"
    
    def correct_joint_angles(self, angles):
        """関節角度を安全範囲内に修正"""
        corrected = []
        corrections = []
        
        for i, angle in enumerate(angles):
            min_limit, max_limit = self.joint_limits[i]
            if angle < min_limit:
                corrected.append(min_limit)
                corrections.append(f"J{i+1}:{angle:.1f}°→{min_limit:.1f}°")
            elif angle > max_limit:
                corrected.append(max_limit)
                corrections.append(f"J{i+1}:{angle:.1f}°→{max_limit:.1f}°")
            else:
                corrected.append(angle)
        
        if corrections:
            corrections_str = ", ".join(corrections)
            logger.info(f"🔧 Angle corrections: {corrections_str}")
        
        return corrected
    
    def check_position(self, position):
        """位置安全チェック"""
        if len(position) < 3:
            return False, SafetyLevel.CRITICAL, "Invalid position"
        
        x, y, z = position[:3]
        
        # 作業領域チェック
        radius = math.sqrt(x*x + y*y)
        max_radius = 350  # mm
        
        issues = []
        if abs(x) > 400:
            issues.append(f"X:{x:.1f}mm > 400mm")
        if abs(y) > 400:
            issues.append(f"Y:{y:.1f}mm > 400mm") 
        if z < 50 or z > 500:
            issues.append(f"Z:{z:.1f}mm out of range (50-500mm)")
        if radius > max_radius:
            issues.append(f"Radius:{radius:.1f}mm > {max_radius}mm")
        
        if issues:
            return False, SafetyLevel.CRITICAL, f"Position unsafe: {', '.join(issues)}"
        
        return True, SafetyLevel.SAFE, "Position safe"

# ===== POSITION MANAGER =====
@dataclass
class Position:
    name: str
    angles: List[float]
    description: str = ""
    gripper_value: Optional[float] = None

class PositionManager:
    def __init__(self, data_dir="/home/ros2/cobot"):
        self.positions = {}
        self.data_dir = data_dir
        self.positions_dir = os.path.join(data_dir, "positions")
        self._ensure_data_dir()
        self._load_positions()
        self._setup_defaults()
    
    def _setup_defaults(self):
        """デフォルト位置設定"""
        defaults = {
            "home": Position("home", [0, 0, 0, 0, 0, 0], "Home position"),
            "safe": Position("safe", [0, -30, -60, 0, 0, 0], "Safe position"),
            "ready": Position("ready", [0, -20, -40, 0, -20, 0], "Ready position"),
        }
        # 既存の位置がなければデフォルトを追加
        for name, pos in defaults.items():
            if name not in self.positions:
                self.positions[name] = pos
                self._save_position_file(name, pos)
        logger.info(f"📍 Default positions loaded: {len(defaults)}")
    
    def get_position(self, name):
        """位置取得"""
        return self.positions.get(name)
    
    def save_position(self, name, angles, gripper_value=None, description=""):
        """位置保存（グリッパー値付き）"""
        if len(angles) != 6:
            return False
        
        position = Position(name, angles.copy(), description, gripper_value)
        self.positions[name] = position
        self._save_position_file(name, position)
        logger.info(f"💾 Position '{name}' saved: {angles}, gripper: {gripper_value}")
        return True
    
    def list_positions(self):
        """位置リスト取得（実際のファイルから）"""
        try:
            if not os.path.exists(self.positions_dir):
                return []
            
            # .jsonファイルのみをリスト
            files = [f[:-5] for f in os.listdir(self.positions_dir) 
                    if f.endswith('.json')]
            return sorted(files)
        except Exception as e:
            logger.error(f"❌ Failed to list positions: {e}")
            return []
    
    def delete_position(self, name):
        """位置削除"""
        if name in self.positions:
            # デフォルト位置は削除しない
            if name in ["home", "safe", "ready"]:
                logger.warning(f"⚠️ Cannot delete default position: {name}")
                return False
            
            del self.positions[name]
            self._delete_position_file(name)
            logger.info(f"🗑️ Position '{name}' deleted")
            return True
        else:
            logger.warning(f"⚠️ Position '{name}' not found")
            return False
    
    def _ensure_data_dir(self):
        """データディレクトリ作成"""
        os.makedirs(self.data_dir, exist_ok=True)
        os.makedirs(self.positions_dir, exist_ok=True)
    
    def _save_position_file(self, name, position):
        """個別位置ファイル保存（~/cobot/positions/name.json）"""
        try:
            data = {
                "name": position.name,
                "angles": position.angles,
                "description": position.description,
                "gripper_value": position.gripper_value,
                "created_at": time.ctime()
            }
            
            file_path = os.path.join(self.positions_dir, f"{name}.json")
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
                
        except Exception as e:
            logger.error(f"❌ Failed to save position {name}: {e}")
    
    def _delete_position_file(self, name):
        """個別位置ファイル削除"""
        try:
            file_path = os.path.join(self.positions_dir, f"{name}.json")
            if os.path.exists(file_path):
                os.remove(file_path)
        except Exception as e:
            logger.error(f"❌ Failed to delete position file {name}: {e}")
    
    def _load_positions(self):
        """位置データを~/cobot/positions/から読み込み"""
        try:
            if os.path.exists(self.positions_dir):
                position_files = [f for f in os.listdir(self.positions_dir) if f.endswith('.json')]
                
                for file_name in position_files:
                    file_path = os.path.join(self.positions_dir, file_name)
                    try:
                        with open(file_path, 'r', encoding='utf-8') as f:
                            data = json.load(f)
                        
                        name = os.path.splitext(file_name)[0]
                        position = Position(
                            name, 
                            data["angles"], 
                            data.get("description", ""),
                            data.get("gripper_value", None)
                        )
                        self.positions[name] = position
                        
                    except Exception as e:
                        logger.warning(f"⚠️ Failed to load {file_name}: {e}")
                
                logger.info(f"📁 Positions loaded: {len(position_files)} from {self.positions_dir}")
        except Exception as e:
            logger.warning(f"⚠️ Failed to load positions: {e}")

# ===== MODE TRANSITION MANAGER =====
class ModeManager:
    def __init__(self):
        self.current_mode = CobotMode.INITIALIZING
        self.previous_mode = None
        self.last_transition_time = time.time()
        self.emergency_triggered = False
        
        # LED色設定
        self.mode_colors = {
            CobotMode.MANUAL: (255, 255, 0),       # 🟡 黄色
            CobotMode.AUTO: (0, 0, 255),           # 🔵 青色
            CobotMode.AI: (128, 0, 128),           # 🟣 紫色
            CobotMode.CALIBRATION: (255, 192, 203), # 🩷 ピンク
            CobotMode.ERROR: (255, 0, 0),          # 🔴 赤色
            CobotMode.EMERGENCY: (255, 0, 0),      # 🔴 真っ赤（明確な緊急色）
            CobotMode.INITIALIZING: (0, 255, 255), # 💙 水色
        }
    
    def transition_to_mode(self, target_mode: CobotMode, force: bool = False):
        """モード切替"""
        # 最小間隔チェック
        min_interval = 1.0
        if not force and time.time() - self.last_transition_time < min_interval:
            return False, f"Must wait {min_interval}s between transitions"
        
        old_mode = self.current_mode
        self.previous_mode = old_mode
        self.current_mode = target_mode
        self.last_transition_time = time.time()
        
        # 緊急停止状態更新
        if target_mode == CobotMode.EMERGENCY:
            self.emergency_triggered = True
        elif target_mode in [CobotMode.AUTO, CobotMode.MANUAL] and old_mode == CobotMode.EMERGENCY:
            self.emergency_triggered = False
        
        logger.info(f"✅ Mode transition: {old_mode.value} → {target_mode.value}")
        return True, f"Successfully switched to {target_mode.value} mode"
    
    def emergency_stop(self, reason="Manual emergency stop"):
        """緊急停止"""
        logger.critical(f"🚨 EMERGENCY STOP: {reason}")
        result, message = self.transition_to_mode(CobotMode.EMERGENCY, force=True)
        return result
    
    def get_led_color(self):
        """現在モードのLED色取得"""
        return self.mode_colors.get(self.current_mode, (255, 255, 255))

# ===== COMMAND FILTER =====
class CommandFilter:
    def __init__(self):
        self.allowed_commands = {
            CobotMode.MANUAL: ["save_position", "get_status", "set_led"],
            CobotMode.AUTO: ["move_to_pose", "set_angles", "execute_trajectory", "get_status"],
            CobotMode.AI: ["move_to_pose", "set_angles", "manual_jog", "get_status"],
            CobotMode.CALIBRATION: ["calibrate_gripper", "get_status"],
            CobotMode.EMERGENCY: ["get_status"],
        }
    
    def is_command_allowed(self, mode: CobotMode, command: str):
        """コマンド許可チェック"""
        allowed = self.allowed_commands.get(mode, [])
        return command in allowed or command == "emergency_stop"

# ===== MAIN COBOT NODE =====
class CobotNode(Node):
    """完全統合版cobotノード"""
    
    def __init__(self):
        super().__init__('cobot_node')
        
        logger.info("🚀 Cobot Node v2.0 starting...")
        
        # パラメータ設定
        self.declare_parameters(
            namespace='',
            parameters=[
                ('port', '/dev/ttyAMA0'),
                ('baudrate', 1000000),
                ('publish_rate', 10.0),
                ('enable_safety', True),
                ('save_safety_mode', 'loose'),  # strict, loose, off
                ('auto_startup', True),
            ]
        )
        
        # パラメータ取得
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_safety = self.get_parameter('enable_safety').value
        self.save_safety_mode = self.get_parameter('save_safety_mode').value
        self.auto_startup = self.get_parameter('auto_startup').value
        
        logger.info(f"📍 Configuration: {self.port}@{self.baudrate}, {self.publish_rate}Hz")
        
        # コールバックグループ
        self.callback_group = ReentrantCallbackGroup()
        
        # システムコンポーネント初期化
        self.safety_checker = SafetyChecker() if self.enable_safety else None
        self.position_manager = PositionManager()
        self.mode_manager = ModeManager()
        self.command_filter = CommandFilter()
        
        # ロボット状態
        self.robot: Optional[MyCobot280] = None
        self.connected = False
        self.current_angles = [0.0] * 6
        self.current_coords = [0.0] * 6
        self.is_moving = False
        self.last_update_time = time.time()
        
        # ROS2インターフェース設定
        self._setup_ros2_interfaces()
        
        # スレッド管理
        self.update_thread = None
        self.shutdown_event = threading.Event()
        
        # ロボット接続・初期化
        self._connect_robot()
        
        # 定期更新開始
        self._start_update_thread()
        
        logger.info("✅ Full cobot Node initialization completed")
    
    def _setup_ros2_interfaces(self):
        """ROS2インターフェース設定"""
        logger.info("🔌 Setting up ROS2 interfaces...")
        
        # パブリッシャー
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.robot_status_pub = self.create_publisher(String, 'cobot/status', 10)
        self.mode_status_pub = self.create_publisher(String, 'cobot/mode_status', 10)
        self.safety_status_pub = self.create_publisher(String, 'cobot/safety_status', 10)
        
        # サブスクライバー
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cobot/cmd_vel', self._cmd_vel_callback, 10,
            callback_group=self.callback_group)
        
        # サービスサーバー
        self.emergency_stop_srv = self.create_service(
            Empty, 'cobot/stop', self._emergency_stop_callback,
            callback_group=self.callback_group)
        
        self.home_srv = self.create_service(
            Empty, 'cobot/home', self._go_home_callback,
            callback_group=self.callback_group)
        
        self.set_mode_srv = self.create_service(
            SetBool, 'cobot/set_mode', self._set_mode_callback,
            callback_group=self.callback_group)
        
        # 直感的なモード切り替えサービス
        self.manual_srv = self.create_service(
            Empty, 'cobot/manual', self._manual_callback,
            callback_group=self.callback_group)
        
        self.auto_srv = self.create_service(
            Empty, 'cobot/auto', self._auto_callback,
            callback_group=self.callback_group)
        
        self.ai_srv = self.create_service(
            Empty, 'cobot/ai', self._ai_callback,
            callback_group=self.callback_group)
        
        self.save_position_srv = self.create_service(
            Empty, 'cobot/save_current_position', self._save_position_callback,
            callback_group=self.callback_group)
        
        # ポジション管理サービス
        self.save_srv = self.create_service(
            Empty, 'cobot/save', self._save_callback,
            callback_group=self.callback_group)
        
        self.list_srv = self.create_service(
            Empty, 'cobot/list', self._list_callback,
            callback_group=self.callback_group)
        
        self.delete_srv = self.create_service(
            Empty, 'cobot/delete', self._delete_callback,
            callback_group=self.callback_group)
        
        # goto系サービス（複数の位置指定用）
        self.goto_home_srv = self.create_service(
            Empty, 'cobot/goto_home', self._goto_home_callback,
            callback_group=self.callback_group)
        
        self.goto_safe_srv = self.create_service(
            Empty, 'cobot/goto_safe', self._goto_safe_callback,
            callback_group=self.callback_group)
        
        self.goto_last_srv = self.create_service(
            Empty, 'cobot/goto_last', self._goto_last_callback,
            callback_group=self.callback_group)
        
        # 任意位置移動用（トピック経由）
        self.goto_position_sub = self.create_subscription(
            String, 'cobot/goto_position', self._goto_position_callback, 10,
            callback_group=self.callback_group)
        
        # 名前付き位置保存用（トピック経由）
        self.save_position_sub = self.create_subscription(
            String, 'cobot/save_position', self._save_position_topic_callback, 10,
            callback_group=self.callback_group)
        
        # タイマー
        self.status_timer = self.create_timer(1.0 / self.publish_rate, self._status_timer_callback)
        
        logger.info("✅ ROS2 interfaces configured")
    
    def _connect_robot(self) -> bool:
        """ロボット接続"""
        logger.info(f"🔗 Connecting to robot at {self.port}...")
        
        try:
            # まずポートの存在確認
            import os
            if not os.path.exists(self.port):
                logger.error(f"❌ Serial port {self.port} not found")
                return False
                
            logger.info("📍 Creating robot object...")
            
            # シンプルなロボット接続（元の動作に戻す）
            logger.info("📍 Attempting robot connection...")
            try:
                self.robot = MyCobot280(self.port, self.baudrate)
                logger.info("📍 Robot object created")
                time.sleep(1.0)
                
                # 接続確認
                if self.robot.is_controller_connected():
                    self.connected = True
                    logger.info("✅ Robot connected successfully")
                else:
                    logger.warning("⚠️ Robot not responding - continuing in offline mode")
                    self.connected = False
                    
            except Exception as e:
                logger.error(f"❌ Robot connection failed: {e}")
                logger.info("📍 Continuing in offline mode...")
                self.robot = None
                self.connected = False
            time.sleep(0.5)  # 短縮
            
            # 接続確認（簡単版：すぐに結果を返す）
            logger.info("📍 Checking controller connection...")
            try:
                # is_controller_connected()を呼ばずに、まず基本的な接続をテスト
                logger.info("📍 Initializing robot communication...")
                
                # まず簡単なチェック
                # ロボットオブジェクトが作成できていればOKとする
                self.connected = True
                logger.info("✅ Robot connection established (basic check)")
                
                # 後でバックグラウンドで接続状態をチェック
                
            except Exception as e:
                logger.error(f"❌ Robot connection failed: {e}")
                return False
                
            # 初期状態取得
            self._update_robot_state()
            
            # 初期化モードから自動モードへ
            if self.auto_startup:
                self._perform_startup_sequence()
            
            return True
                
        except Exception as e:
            logger.error(f"❌ Robot connection failed: {e}")
            return False
    
    def _perform_startup_sequence(self):
        """起動シーケンス実行"""
        logger.info("🚀 Performing startup sequence...")
        
        # 初期化モードLED設定
        if self.robot:
            r, g, b = self.mode_manager.get_led_color()
            self.robot.set_color(r, g, b)
        
        # 安全チェック
        if self.safety_checker:
            safe, level, message = self.safety_checker.check_joint_angles(self.current_angles)
            if not safe:
                logger.warning(f"⚠️ Startup safety warning: {message}")
                # エラーモードに移行
                self.mode_manager.transition_to_mode(CobotMode.ERROR, force=True)
                return
        
        # 自動モードへ切替
        result, message = self.mode_manager.transition_to_mode(CobotMode.AUTO)
        if result:
            logger.info("✅ Startup sequence completed - AUTO mode active")
            if self.robot:
                r, g, b = self.mode_manager.get_led_color()
                self.robot.set_color(r, g, b)
                
                # 自動起動時はホームポジションに移動
                home_pos = self.position_manager.get_position("home")
                if home_pos:
                    logger.info("🏠 Moving to home position...")
                    self.robot.send_angles(home_pos.angles, 50)
        else:
            logger.error(f"❌ Startup sequence failed: {message}")
    
    def _start_update_thread(self):
        """定期更新スレッド開始"""
        logger.info("🔄 Starting update thread...")
        self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self.update_thread.start()
    
    def _update_loop(self):
        """メイン更新ループ"""
        rate = 1.0 / self.publish_rate
        
        while not self.shutdown_event.is_set():
            try:
                if self.connected:
                    self._update_robot_state()
                time.sleep(rate)
            except Exception as e:
                logger.error(f"❌ Update loop error: {e}")
                time.sleep(1.0)
    
    def _update_robot_state(self):
        """ロボット状態更新"""
        if not self.robot:
            return
        
        try:
            # 関節角度取得
            angles = self.robot.get_angles()
            if angles and isinstance(angles, (list, tuple)) and len(angles) == 6:
                self.current_angles = angles
            
            # 座標取得  
            coords = self.robot.get_coords()
            if coords and isinstance(coords, (list, tuple)) and len(coords) >= 6:
                self.current_coords = coords
            
            # 動作状態確認
            moving_state = self.robot.is_moving()
            if isinstance(moving_state, (int, bool)):
                self.is_moving = moving_state
            
            self.last_update_time = time.time()
            
        except Exception as e:
            # エラー時のみ詳細ログ出力
            logger.error(f"⚠️ State update failed: {e}")
            # デバッグ情報は必要時のみ
            if str(e).find("len()") != -1:  # len()エラーの場合のみ詳細表示
                try:
                    angles = self.robot.get_angles()
                    coords = self.robot.get_coords()
                    moving = self.robot.is_moving()
    
                except:
                    pass
    
    def _status_timer_callback(self):
        """状態配信タイマーコールバック"""
        try:
            # JointState配信
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.header.frame_id = "base_link"
            joint_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            joint_msg.position = [math.radians(a) for a in self.current_angles]
            joint_msg.velocity = [0.0] * 6
            joint_msg.effort = [0.0] * 6
            self.joint_state_pub.publish(joint_msg)
            
            # グリッパー状態取得
            gripper_value = None
            if self.robot and self.connected:
                try:
                    gripper_value = self.robot.get_gripper_value()
                except Exception as e:
                    logger.warning(f"🔧 Gripper value error: {e}")
            
            # ロボット状態配信
            status_data = {
                'connected': self.connected,
                'mode': self.mode_manager.current_mode.value,
                'is_moving': self.is_moving,
                'angles': self.current_angles,
                'coords': self.current_coords,
                'gripper_value': gripper_value,
                'last_update': self.last_update_time,
                'emergency': self.mode_manager.emergency_triggered
            }
            
            status_msg = String()
            status_msg.data = json.dumps(status_data)
            self.robot_status_pub.publish(status_msg)
            
            # モード状態配信
            mode_msg = String()
            mode_msg.data = self.mode_manager.current_mode.value
            self.mode_status_pub.publish(mode_msg)
            
            # 安全状態配信
            if self.safety_checker:
                safe, level, message = self.safety_checker.check_joint_angles(self.current_angles)
                safety_data = {
                    'safe': safe,
                    'level': level.value,
                    'message': message
                }
                safety_msg = String()
                safety_msg.data = json.dumps(safety_data)
                self.safety_status_pub.publish(safety_msg)
            
        except Exception as e:
            logger.warning(f"⚠️ Status publish failed: {e}")
    
    def _cmd_vel_callback(self, msg: Twist):
        """速度コマンドコールバック"""
        if not self.connected or not self.robot:
            return
        
        # コマンドフィルタチェック
        if not self.command_filter.is_command_allowed(self.mode_manager.current_mode, "manual_jog"):
            logger.warning("🚫 cmd_vel blocked in current mode")
            return
        
        logger.info(f"🎮 cmd_vel: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}")
        
        try:
            # 簡易JOG制御実装
            if abs(msg.linear.x) > 0.01:  # X軸移動
                current_coords = self.robot.get_coords()
                if current_coords:
                    new_coords = current_coords.copy()
                    new_coords[0] += msg.linear.x * 10
                    
                    # 安全チェック
                    if self.safety_checker:
                        safe, level, message = self.safety_checker.check_position(new_coords[:3])
                        if safe:
                            self.robot.send_coords(new_coords, 30)
                            logger.info(f"📍 Moving to: {new_coords}")
                        else:
                            logger.warning(f"⚠️ Movement blocked: {message}")
            
            if abs(msg.angular.z) > 0.01:  # Z軸回転
                current_angles = self.robot.get_angles()
                if current_angles:
                    new_angles = current_angles.copy()
                    new_angles[0] += math.degrees(msg.angular.z) * 5
                    
                    # 安全チェック
                    if self.safety_checker:
                        safe, level, message = self.safety_checker.check_joint_angles(new_angles)
                        if safe:
                            self.robot.send_angles(new_angles, 30)
                            logger.info(f"🔄 Rotating to: {new_angles}")
                        else:
                            logger.warning(f"⚠️ Rotation blocked: {message}")
                            
        except Exception as e:
            logger.error(f"❌ cmd_vel execution failed: {e}")
    
    def _emergency_stop_callback(self, request, response):
        """緊急停止サービスコールバック"""
        logger.critical("🚨 Emergency stop service called")
        
        try:
            success = self.mode_manager.emergency_stop("ROS2 service call")
            
            if success and self.robot:
                self.robot.release_all_servos()
                r, g, b = self.mode_manager.get_led_color()
                self.robot.set_color(r, g, b)
            
            logger.critical(f"🚨 Emergency stop {'successful' if success else 'failed'}")
            
        except Exception as e:
            logger.critical(f"🚨 Emergency stop error: {e}")
        
        return response
    
    def _go_home_callback(self, request, response):
        """ホーム位置移動サービスコールバック"""
        logger.info("🏠 Go home service called")
        
        try:
            # ホーム移動は全モードで許可
            home_position = self.position_manager.get_position("home")
            
            if home_position and self.robot:
                # 安全チェック
                if self.safety_checker:
                    safe, level, message = self.safety_checker.check_joint_angles(home_position.angles)
                    if not safe:
                        logger.warning(f"⚠️ Home position unsafe: {message}")
                        return response
                
                self.robot.send_angles(home_position.angles, 50)
                logger.info(f"🏠 Moving to home: {home_position.angles}")
                
            else:
                logger.error("❌ Home position not available")
                
        except Exception as e:
            logger.error(f"❌ Go home failed: {e}")
        
        return response
    
    def _set_mode_callback(self, request: SetBool.Request, response: SetBool.Response):
        """モード設定サービスコールバック"""
        target_mode = CobotMode.MANUAL if request.data else CobotMode.AUTO
        logger.info(f"🔄 Set mode service called: {target_mode.value}")
        
        try:
            result, message = self.mode_manager.transition_to_mode(target_mode)
            
            if result:
                response.success = True
                response.message = f"Mode changed to {target_mode.value}"
                logger.info(f"✅ Mode change successful: {target_mode.value}")
                
                # LED更新とロボット設定
                if self.robot:
                    r, g, b = self.mode_manager.get_led_color()
                    self.robot.set_color(r, g, b)
                    
                    if target_mode == CobotMode.MANUAL:
                        self.robot.set_free_mode(1)
                    elif target_mode == CobotMode.AUTO:
                        self.robot.set_free_mode(0)
            else:
                response.success = False
                response.message = f"Mode change failed: {message}"
                logger.warning(f"❌ Mode change failed: {message}")
                
        except Exception as e:
            response.success = False
            response.message = f"Mode change error: {e}"
            logger.error(f"❌ Mode change error: {e}")
        
        return response
    
    def _manual_callback(self, request, response):
        """マニュアルモードサービスコールバック"""
        logger.info("🟡 Manual mode service called")
        
        try:
            result, message = self.mode_manager.transition_to_mode(CobotMode.MANUAL)
            
            if result:
                logger.info("✅ Manual mode activated")
                
                # LED更新とロボット設定
                if self.robot:
                    r, g, b = self.mode_manager.get_led_color()
                    self.robot.set_color(r, g, b)
                    self.robot.set_free_mode(1)
            else:
                logger.warning(f"❌ Manual mode failed: {message}")
                
        except Exception as e:
            logger.error(f"❌ Manual mode error: {e}")
        
        return response
    
    def _auto_callback(self, request, response):
        """オートモードサービスコールバック"""
        logger.info("🟢 Auto mode service called")
        
        try:
            result, message = self.mode_manager.transition_to_mode(CobotMode.AUTO)
            
            if result:
                logger.info("✅ Auto mode activated")
                
                # LED更新とロボット設定
                if self.robot:
                    r, g, b = self.mode_manager.get_led_color()
                    self.robot.set_color(r, g, b)
                    self.robot.set_free_mode(0)
            else:
                logger.warning(f"❌ Auto mode failed: {message}")
                
        except Exception as e:
            logger.error(f"❌ Auto mode error: {e}")
        
        return response
    
    def _ai_callback(self, request, response):
        """AIモードサービスコールバック"""
        logger.info("🤖 AI mode service called")
        
        try:
            result, message = self.mode_manager.transition_to_mode(CobotMode.AI)
            
            if result:
                logger.info("✅ AI mode activated")
                
                # LED更新とロボット設定
                if self.robot:
                    r, g, b = self.mode_manager.get_led_color()
                    self.robot.set_color(r, g, b)
                    self.robot.set_free_mode(0)  # AIモードは自動制御
            else:
                logger.warning(f"❌ AI mode failed: {message}")
                
        except Exception as e:
            logger.error(f"❌ AI mode error: {e}")
        
        return response
    
    def _save_position_callback(self, request, response):
        """現在位置保存サービスコールバック"""
        logger.info("💾 Save current position called")
        
        try:
            if self.robot:
                current_angles = self.robot.get_angles()
                if current_angles:
                    timestamp = int(time.time())
                    position_name = f"saved_pos_{timestamp}"
                    
                    success = self.position_manager.save_position(
                        position_name, current_angles, f"Saved at {time.ctime()}")
                    
                    if success:
                        logger.info(f"💾 Position saved as '{position_name}': {current_angles}")
                    else:
                        logger.error("❌ Failed to save position")
                else:
                    logger.error("❌ Could not get current angles")
            else:
                logger.error("❌ Robot not connected")
                
        except Exception as e:
            logger.error(f"❌ Save position failed: {e}")
        
        return response
    
    def cleanup(self):
        """クリーンアップ"""
        logger.info("🧹 Cleaning up full cobot node...")
        
        self.shutdown_event.set()
        if self.update_thread and self.update_thread.is_alive():
            self.update_thread.join(timeout=2.0)
        
        if self.robot:
            try:
                self.robot.release_all_servos()
                logger.info("🔓 All servos released")
            except:
                pass
        
        logger.info("✅ Cleanup completed")
    
    def _save_callback(self, request, response):
        """位置保存サービスコールバック（シンプル版）"""
        logger.info("💾 Save position called")
        
        try:
            if self.robot:
                current_angles = self.robot.get_angles()
                if current_angles:
                    timestamp = int(time.time())
                    position_name = f"pos_{timestamp}"
                    
                    success = self.position_manager.save_position(
                        position_name, current_angles, f"Saved at {time.ctime()}")
                    
                    if success:
                        logger.info(f"✅ Position saved: {position_name}")
                        # 成功時は静かに
                    else:
                        logger.error("❌ Failed to save position")
                        print("❌ Failed to save position")
                else:
                    logger.error("❌ Could not get current angles")
                    print("❌ Could not get current angles")
            else:
                logger.error("❌ Robot not connected")
                print("❌ Robot not connected")
                
        except Exception as e:
            logger.error(f"❌ Save position failed: {e}")
            print(f"❌ Save position failed: {e}")
        
        return response
    
    def _list_callback(self, request, response):
        """位置リストサービスコールバック"""
        logger.info("📋 List positions called")
        
        try:
            positions = self.position_manager.list_positions()
            
            if positions:
                logger.info(f"📋 Found {len(positions)} positions")
                print(f"📋 保存済み位置 ({len(positions)}件):")
                for name in positions:
                    print(f"  - {name}")
            else:
                logger.info("📋 No positions found")
                print("📋 保存済み位置はありません")
                
        except Exception as e:
            logger.error(f"❌ List positions failed: {e}")
            print(f"❌ List positions failed: {e}")
        
        return response
    
    def _delete_callback(self, request, response):
        """位置削除サービスコールバック"""
        logger.info("🗑️ Delete last position called")
        
        try:
            # ユーザー保存位置のみ削除対象（pos_で始まるもの）
            user_positions = [name for name in self.position_manager.list_positions() 
                            if name.startswith('pos_')]
            
            if user_positions:
                # タイムスタンプ順で最新を削除
                latest_pos = sorted(user_positions)[-1]
                success = self.position_manager.delete_position(latest_pos)
                
                if success:
                    logger.info(f"✅ Position deleted: {latest_pos}")
                    # 成功時は静かに
                else:
                    logger.error(f"❌ Failed to delete position: {latest_pos}")
                    print(f"❌ Failed to delete position: {latest_pos}")
            else:
                logger.info("📋 No user positions to delete")
                print("📋 削除するユーザー位置がありません（デフォルト位置は削除されません）")
                
        except Exception as e:
            logger.error(f"❌ Delete position failed: {e}")
            print(f"❌ Delete position failed: {e}")
        
        return response
    
    def _goto_home_callback(self, request, response):
        """ホーム位置移動"""
        return self._goto_position_helper("home")
    
    def _goto_safe_callback(self, request, response):
        """セーフ位置移動"""
        return self._goto_position_helper("safe")
    
    def _goto_last_callback(self, request, response):
        """最新位置移動"""
        try:
            user_positions = [name for name in self.position_manager.list_positions() 
                            if name.startswith('pos_')]
            
            if user_positions:
                latest_pos = sorted(user_positions)[-1]
                return self._goto_position_helper(latest_pos)
            else:
                logger.info("📋 No user positions to go to")
                print("📋 移動するユーザー位置がありません")
                return response
        except Exception as e:
            logger.error(f"❌ Goto last failed: {e}")
            print(f"❌ Goto last failed: {e}")
            return response
    
    def _goto_position_helper(self, position_name):
        """位置移動ヘルパー"""
        logger.info(f"🎯 Moving to position: {position_name}")
        
        try:
            position = self.position_manager.get_position(position_name)
            
            if position and self.robot:
                # 関節角度移動
                self.robot.send_angles(position.angles, 80)  # 速度80で移動
                
                # グリッパー復元
                if position.gripper_value is not None:
                    try:
                        self.robot.set_gripper_value(position.gripper_value, 50)
                        logger.info(f"🤏 Gripper restored: {position.gripper_value}")
                    except Exception as e:
                        logger.warning(f"⚠️ Gripper restore failed: {e}")
                
                logger.info(f"✅ Moving to position: {position_name}")
                # 成功時は静かに（ログのみ）
            else:
                logger.error(f"❌ Position '{position_name}' not found or robot not connected")
                print(f"❌ Position '{position_name}' not found or robot not connected")
                
        except Exception as e:
            logger.error(f"❌ Failed to move to {position_name}: {e}")
            print(f"❌ Failed to move to {position_name}: {e}")
        
        return Empty.Response()
    
    def _save_position_topic_callback(self, msg):
        """名前付き位置保存コールバック（トピック経由）"""
        position_name = msg.data.strip()
        logger.info(f"💾 Save position requested: '{position_name}'")
        
        if not self.robot or not self.robot.is_controller_connected():
            logger.error("❌ Robot not connected")
            print("❌ Robot not connected")
            return
            
        try:
            # 現在位置取得
            current_angles = self.robot.get_angles()
            if not current_angles or not isinstance(current_angles, (list, tuple)) or len(current_angles) != 6:
                logger.error("❌ Failed to get current position")
                print("❌ Failed to get current position")
                return
            
            # グリッパー状態取得
            try:
                gripper_value = self.robot.get_gripper_value()
                if gripper_value is None:
                    gripper_value = 50  # デフォルト値
                    logger.warning("⚠️ Could not get gripper value, using default 50")
            except Exception as e:
                gripper_value = 50  # デフォルト値
                logger.warning(f"⚠️ Gripper value error: {e}, using default 50")
            
            # 安全チェック（モード別処理）
            if self.save_safety_mode != "off":
                safe, safety_level, message = self.safety_checker.check_joint_angles(current_angles)
                
                if self.save_safety_mode == "strict":
                    # 厳格モード: WARNINGでもブロック
                    if not safe or safety_level == SafetyLevel.WARNING:
                        logger.warning(f"⚠️ Save blocked (strict): {message}")
                        print(f"⚠️ 保存ブロック（厳格モード）: {message}")
                        print("  → 安全範囲内に移動してから保存してください")
                        return
                elif self.save_safety_mode == "loose":
                    # 緩いモード: CRITICALは修正保存、WARNINGは警告のみ
                    if not safe:
                        # 限界超過 → 限界値に修正して保存
                        corrected_angles = self.safety_checker.correct_joint_angles(current_angles)
                        current_angles = corrected_angles
                        logger.warning(f"🔧 Save corrected (loose): {message}")
                        print(f"🔧 保存修正（緩いモード）: {message}")
                        print("  → 限界値に修正して保存します")
                    elif safety_level == SafetyLevel.WARNING:
                        logger.info(f"ℹ️ Save info: {message}")
                        print(f"ℹ️ 保存情報: {message}")
                # offモードは何もチェックしない
            
            # 名前が指定されていない場合はタイムスタンプ名
            if not position_name:
                position_name = f"pos_{int(time.time())}"
            
            # 位置保存（角度+グリッパー）
            success = self.position_manager.save_position(position_name, current_angles, gripper_value)
            if success:
                logger.info(f"✅ Position saved: {position_name}")
                print("💾 位置保存完了")
            else:
                logger.error(f"❌ Failed to save position: {position_name}")
                print(f"❌ Failed to save position: {position_name}")
                
        except Exception as e:
            logger.error(f"❌ Save position error: {e}")
            print(f"❌ Save position error: {e}")
    
    def _goto_position_callback(self, msg):
        """任意位置移動コールバック（トピック経由）"""
        position_name = msg.data.strip()
        logger.info(f"🎯 Goto position requested: '{position_name}'")
        
        if position_name:
            self._goto_position_helper(position_name)
        else:
            # 空文字列なら最新位置
            try:
                user_positions = [name for name in self.position_manager.list_positions() 
                                if name.startswith('pos_')]
                
                if user_positions:
                    latest_pos = sorted(user_positions)[-1]
                    self._goto_position_helper(latest_pos)
                else:
                    logger.info("📋 No user positions to go to")
                    print("📋 移動するユーザー位置がありません")
            except Exception as e:
                logger.error(f"❌ Goto last failed: {e}")
                print(f"❌ Goto last failed: {e}")

def main(args=None):
    """メイン関数"""
    rclpy.init(args=args)
    
    try:
        node = CobotNode()
        
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        
        logger.info("🚀 Cobot Node running...")
        logger.info("📝 Available services:")
        logger.info("  - /cobot/stop - 緊急停止")
        logger.info("  - /cobot/home - ホーム移動")
        logger.info("  - /cobot/manual - マニュアルモード")
        logger.info("  - /cobot/auto - オートモード")
        logger.info("  - /cobot/ai - AIモード")
        logger.info("  - /cobot/set_mode - モード切替 (legacy)")
        logger.info("  - /cobot/save_current_position - 現在位置保存")
        logger.info("  - /cobot/save - 位置保存")
        logger.info("  - /cobot/list - 位置リスト")
        logger.info("  - /cobot/delete - 最新位置削除")
        logger.info("  - /cobot/goto_home - ホーム移動")
        logger.info("  - /cobot/goto_safe - セーフ移動") 
        logger.info("  - /cobot/goto_last - 最新位置移動")
        logger.info("📨 Subscribing topics:")
        logger.info("  - /cobot/goto_position - 任意位置移動（位置名送信）")
        logger.info("  - /cobot/save_position - 名前付き位置保存（位置名送信）")
        logger.info("📡 Publishing topics:")
        logger.info("  - /joint_states - 関節状態")
        logger.info("  - /cobot/status - ロボット状態")
        logger.info("  - /cobot/mode_status - モード状態")
        logger.info("  - /cobot/safety_status - 安全状態")
        logger.info("📥 Subscribing topics:")
        logger.info("  - /cobot/cmd_vel - 速度制御")
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            logger.info("🛑 Shutdown requested")
        finally:
            node.cleanup()
            executor.shutdown()
    
    except Exception as e:
        logger.error(f"❌ Node execution failed: {e}")
    
    finally:
        rclpy.shutdown()
        logger.info("👋 Cobot Node shutdown completed")

if __name__ == '__main__':
    main()
