#!/usr/bin/env python3
"""
Mode Transition System
======================

モード切替システム - 安全チェック・状態遷移管理

Author: Takashi Otsuka (takatronix@gmail.com)
License: Apache-2.0
"""

from enum import Enum
import time
import logging
from typing import Tuple, Optional, Dict, Any
from dataclasses import dataclass

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class CobotMode(Enum):
    """ロボットモード定義"""
    MANUAL = "manual"           # 🟡 手動モード
    AUTO = "auto"              # 🔵 自動モード  
    AI = "ai"                  # 🟣 AIモード
    CALIBRATION = "calibration" # 🩷 校正モード
    ERROR = "error"            # 🔴 エラー
    EMERGENCY = "emergency"    # �� 緊急停止
    INITIALIZING = "init"      # 💙 初期化

class TransitionResult(Enum):
    """切替結果"""
    SUCCESS = "success"
    BLOCKED = "blocked"
    UNSAFE = "unsafe"
    ERROR = "error"

@dataclass
class ModeState:
    """モード状態"""
    current_mode: CobotMode
    previous_mode: Optional[CobotMode]
    transition_time: float
    is_stable: bool
    error_message: str = ""

class ModeTransitionManager:
    """モード切替管理クラス"""
    
    def __init__(self):
        """初期化"""
        self.current_mode = CobotMode.INITIALIZING
        self.previous_mode = None
        self.transition_history = []
        self.is_motion_active = False
        self.last_transition_time = time.time()
        self.emergency_triggered = False
        
        # LED色設定
        self.mode_colors = {
            CobotMode.MANUAL: (255, 255, 0),       # 🟡 黄色
            CobotMode.AUTO: (0, 0, 255),           # 🔵 青色
            CobotMode.AI: (128, 0, 128),           # 🟣 紫色
            CobotMode.CALIBRATION: (255, 192, 203), # 🩷 ピンク
            CobotMode.ERROR: (255, 0, 0),          # 🔴 赤色
            CobotMode.EMERGENCY: (255, 165, 0),    # 🔶 オレンジ
            CobotMode.INITIALIZING: (0, 255, 255), # 💙 水色
        }
        
        # 切替許可マトリックス
        self._setup_transition_matrix()
    
    def _setup_transition_matrix(self):
        """切替許可マトリックス設定"""
        self.allowed_transitions = {
            CobotMode.INITIALIZING: [
                CobotMode.AUTO,         # 通常起動
                CobotMode.MANUAL,       # メンテナンス起動
                CobotMode.ERROR,        # 起動失敗
                CobotMode.EMERGENCY,    # 緊急時
            ],
            
            CobotMode.AUTO: [
                CobotMode.MANUAL,       # ティーチング切替
                CobotMode.AI,           # AI連携切替
                CobotMode.CALIBRATION,  # 校正切替
                CobotMode.ERROR,        # エラー発生
                CobotMode.EMERGENCY,    # 緊急停止
            ],
            
            CobotMode.MANUAL: [
                CobotMode.AUTO,         # 自動復帰
                CobotMode.CALIBRATION,  # 校正モード
                CobotMode.ERROR,        # エラー発生
                CobotMode.EMERGENCY,    # 緊急停止
            ],
            
            CobotMode.AI: [
                CobotMode.AUTO,         # 自動復帰
                CobotMode.MANUAL,       # 手動介入
                CobotMode.ERROR,        # エラー発生
                CobotMode.EMERGENCY,    # 緊急停止
            ],
            
            CobotMode.CALIBRATION: [
                CobotMode.AUTO,         # 校正完了→自動
                CobotMode.MANUAL,       # 手動確認
                CobotMode.ERROR,        # 校正失敗
                CobotMode.EMERGENCY,    # 緊急停止
            ],
            
            CobotMode.ERROR: [
                CobotMode.AUTO,         # エラー回復
                CobotMode.MANUAL,       # 手動復旧
                CobotMode.EMERGENCY,    # 緊急停止
                CobotMode.INITIALIZING, # 再起動
            ],
            
            CobotMode.EMERGENCY: [
                CobotMode.MANUAL,       # 手動復旧のみ
                CobotMode.INITIALIZING, # システム再起動
            ],
        }
    
    def is_transition_allowed(self, target_mode: CobotMode) -> Tuple[bool, str]:
        """
        切替許可チェック
        
        Args:
            target_mode: 目標モード
            
        Returns:
            Tuple[bool, str]: (許可/拒否, 理由)
        """
        # 同一モードチェック
        if target_mode == self.current_mode:
            return False, f"Already in {target_mode.value} mode"
        
        # 許可マトリックスチェック
        allowed = self.allowed_transitions.get(self.current_mode, [])
        if target_mode not in allowed:
            return False, f"Transition from {self.current_mode.value} to {target_mode.value} not allowed"
        
        # 動作中チェック
        if self.is_motion_active and target_mode not in [CobotMode.EMERGENCY, CobotMode.ERROR]:
            return False, "Cannot switch mode while motion is active"
        
        # 最小間隔チェック
        min_interval = 1.0  # 秒
        if time.time() - self.last_transition_time < min_interval:
            return False, f"Must wait {min_interval}s between transitions"
        
        return True, "Transition allowed"
    
    def perform_safety_check(self, target_mode: CobotMode) -> Tuple[bool, str]:
        """
        安全チェック実行
        
        Args:
            target_mode: 目標モード
            
        Returns:
            Tuple[bool, str]: (安全/危険, 理由)
        """
        # 緊急停止状態チェック
        if self.emergency_triggered and target_mode not in [CobotMode.MANUAL, CobotMode.INITIALIZING]:
            return False, "Emergency state active, only manual or init allowed"
        
        # モード別安全チェック
        if target_mode == CobotMode.MANUAL:
            # 手動モード: 安全位置必要
            return True, "Manual mode - safe position recommended"
        
        elif target_mode == CobotMode.AUTO:
            # 自動モード: 初期化完了必要
            if self.current_mode == CobotMode.INITIALIZING:
                return False, "Initialization not complete"
            return True, "Auto mode safe"
        
        elif target_mode == CobotMode.AI:
            # AIモード: カメラ・センサー確認必要
            return True, "AI mode - sensors should be checked"
        
        elif target_mode == CobotMode.CALIBRATION:
            # 校正モード: 安全位置必要
            return True, "Calibration mode - safe position required"
        
        elif target_mode == CobotMode.EMERGENCY:
            # 緊急停止: 常に許可
            return True, "Emergency stop always allowed"
        
        else:
            return True, "Safety check passed"
    
    def transition_to_mode(self, target_mode: CobotMode, force: bool = False) -> Tuple[TransitionResult, str]:
        """
        モード切替実行
        
        Args:
            target_mode: 目標モード
            force: 強制切替フラグ
            
        Returns:
            Tuple[TransitionResult, str]: (結果, メッセージ)
        """
        logger.info(f"🔄 Mode transition requested: {self.current_mode.value} → {target_mode.value}")
        
        # 強制切替でない場合の許可チェック
        if not force:
            allowed, reason = self.is_transition_allowed(target_mode)
            if not allowed:
                logger.warning(f"🚫 Transition blocked: {reason}")
                return TransitionResult.BLOCKED, reason
            
            # 安全チェック
            safe, safety_reason = self.perform_safety_check(target_mode)
            if not safe:
                logger.warning(f"⚠️ Transition unsafe: {safety_reason}")
                return TransitionResult.UNSAFE, safety_reason
        
        # 切替前処理
        pre_result = self._pre_transition_actions(target_mode)
        if not pre_result[0]:
            logger.error(f"❌ Pre-transition failed: {pre_result[1]}")
            return TransitionResult.ERROR, pre_result[1]
        
        # モード変更実行
        old_mode = self.current_mode
        self.previous_mode = old_mode
        self.current_mode = target_mode
        self.last_transition_time = time.time()
        
        # 切替後処理
        post_result = self._post_transition_actions(old_mode, target_mode)
        if not post_result[0]:
            logger.error(f"❌ Post-transition failed: {post_result[1]}")
            # ロールバック
            self.current_mode = old_mode
            return TransitionResult.ERROR, post_result[1]
        
        # 履歴記録
        self.transition_history.append({
            "from": old_mode.value,
            "to": target_mode.value,
            "timestamp": self.last_transition_time,
            "forced": force
        })
        
        # 緊急停止状態更新
        if target_mode == CobotMode.EMERGENCY:
            self.emergency_triggered = True
        elif target_mode in [CobotMode.AUTO, CobotMode.MANUAL] and old_mode == CobotMode.EMERGENCY:
            self.emergency_triggered = False
        
        logger.info(f"✅ Mode transition successful: {old_mode.value} → {target_mode.value}")
        return TransitionResult.SUCCESS, f"Successfully switched to {target_mode.value} mode"
    
    def _pre_transition_actions(self, target_mode: CobotMode) -> Tuple[bool, str]:
        """切替前処理"""
        
        # 動作停止
        if self.is_motion_active and target_mode != CobotMode.EMERGENCY:
            logger.info("⏹️ Stopping motion before transition...")
            # 実際のロボットでは robot.stop() を呼び出し
            self.is_motion_active = False
        
        # モード別前処理
        if target_mode == CobotMode.MANUAL:
            logger.info("🔧 Preparing for manual mode...")
            # フリーモード設定準備
            
        elif target_mode == CobotMode.AUTO:
            logger.info("🤖 Preparing for auto mode...")
            # サーボON、安全位置確認
            
        elif target_mode == CobotMode.AI:
            logger.info("🧠 Preparing for AI mode...")
            # カメラ・センサー初期化
            
        elif target_mode == CobotMode.CALIBRATION:
            logger.info("🎯 Preparing for calibration mode...")
            # 校正環境設定
            
        elif target_mode == CobotMode.EMERGENCY:
            logger.critical("🚨 Emergency stop preparation...")
            # 即座にサーボ解除
        
        return True, "Pre-transition completed"
    
    def _post_transition_actions(self, old_mode: CobotMode, new_mode: CobotMode) -> Tuple[bool, str]:
        """切替後処理"""
        
        # LED色変更
        if new_mode in self.mode_colors:
            r, g, b = self.mode_colors[new_mode]
            logger.info(f"💡 Setting LED to {new_mode.value} color: RGB({r}, {g}, {b})")
            # 実際のロボットでは robot.set_color(r, g, b) を呼び出し
        
        # モード別後処理
        if new_mode == CobotMode.MANUAL:
            logger.info("🟡 Manual mode activated - teaching enabled")
            # robot.set_free_mode(1)
            
        elif new_mode == CobotMode.AUTO:
            logger.info("🔵 Auto mode activated - ready for commands")
            # robot.set_free_mode(0)
            
        elif new_mode == CobotMode.AI:
            logger.info("🟣 AI mode activated - waiting for vision input")
            
        elif new_mode == CobotMode.CALIBRATION:
            logger.info("🩷 Calibration mode activated - precision required")
            
        elif new_mode == CobotMode.EMERGENCY:
            logger.critical("🔶 Emergency mode activated - all servos released")
            # robot.release_all_servos()
            
        elif new_mode == CobotMode.ERROR:
            logger.error("🔴 Error mode activated - diagnostics required")
        
        return True, "Post-transition completed"
    
    def emergency_stop(self, reason: str = "Manual emergency stop") -> bool:
        """緊急停止実行"""
        logger.critical(f"🚨 EMERGENCY STOP TRIGGERED: {reason}")
        
        result, message = self.transition_to_mode(CobotMode.EMERGENCY, force=True)
        
        if result == TransitionResult.SUCCESS:
            logger.critical("🚨 Emergency stop successful")
            return True
        else:
            logger.critical(f"🚨 Emergency stop failed: {message}")
            return False
    
    def recover_from_emergency(self, target_mode: CobotMode = CobotMode.MANUAL) -> Tuple[bool, str]:
        """緊急停止からの復旧"""
        if not self.emergency_triggered:
            return False, "No emergency state to recover from"
        
        logger.info(f"🔧 Recovering from emergency to {target_mode.value} mode...")
        
        result, message = self.transition_to_mode(target_mode)
        
        if result == TransitionResult.SUCCESS:
            logger.info("✅ Emergency recovery successful")
            return True, message
        else:
            logger.error(f"❌ Emergency recovery failed: {message}")
            return False, message
    
    def get_current_state(self) -> ModeState:
        """現在の状態取得"""
        return ModeState(
            current_mode=self.current_mode,
            previous_mode=self.previous_mode,
            transition_time=self.last_transition_time,
            is_stable=time.time() - self.last_transition_time > 2.0,
            error_message=""
        )
    
    def get_allowed_transitions(self) -> list:
        """現在のモードから可能な切替先一覧"""
        return [mode.value for mode in self.allowed_transitions.get(self.current_mode, [])]
    
    def get_transition_history(self, limit: int = 10) -> list:
        """切替履歴取得"""
        return self.transition_history[-limit:]
    
    def set_motion_active(self, active: bool):
        """動作状態設定"""
        self.is_motion_active = active
        logger.debug(f"🤖 Motion active: {active}")

def main():
    """テスト用メイン関数"""
    logger.info("🔄 Mode Transition System Test Starting...")
    
    manager = ModeTransitionManager()
    
    # 初期状態表示
    state = manager.get_current_state()
    logger.info(f"📊 Initial state: {state.current_mode.value}")
    
    # 切替テストシナリオ
    test_scenarios = [
        # 正常な切替シーケンス
        (CobotMode.AUTO, False, "Normal startup"),
        (CobotMode.MANUAL, False, "Switch to teaching"),
        (CobotMode.AUTO, False, "Return to auto"),
        (CobotMode.AI, False, "Switch to AI"),
        (CobotMode.CALIBRATION, False, "Switch to calibration"),
        
        # 異常ケース
        (CobotMode.INITIALIZING, False, "Invalid backward transition"),
        (CobotMode.ERROR, False, "Force error state"),
        (CobotMode.MANUAL, False, "Recover from error"),
        
        # 緊急停止テスト
        ("EMERGENCY", False, "Emergency stop test"),
        (CobotMode.MANUAL, False, "Emergency recovery"),
    ]
    
    for i, (target, force, description) in enumerate(test_scenarios):
        print(f"\n--- Test {i+1}: {description} ---")
        
        if target == "EMERGENCY":
            # 緊急停止テスト
            success = manager.emergency_stop("Test emergency")
            print(f"🚨 Emergency stop: {'✅ Success' if success else '❌ Failed'}")
        else:
            # 通常の切替テスト
            result, message = manager.transition_to_mode(target, force)
            
            status_icons = {
                TransitionResult.SUCCESS: "✅",
                TransitionResult.BLOCKED: "🚫", 
                TransitionResult.UNSAFE: "⚠️",
                TransitionResult.ERROR: "❌"
            }
            icon = status_icons.get(result, "?")
            
            print(f"{icon} {result.value.upper()}: {message}")
        
        # 現在状態表示
        current_state = manager.get_current_state()
        allowed = manager.get_allowed_transitions()
        print(f"📍 Current: {current_state.current_mode.value}")
        print(f"🔄 Allowed: {allowed}")
        
        time.sleep(0.5)  # テスト間隔
    
    # 最終状態・履歴表示
    print(f"\n📊 Final State:")
    final_state = manager.get_current_state()
    print(f"  Mode: {final_state.current_mode.value}")
    print(f"  Previous: {final_state.previous_mode.value if final_state.previous_mode else 'None'}")
    print(f"  Stable: {final_state.is_stable}")
    
    print(f"\n📜 Transition History:")
    history = manager.get_transition_history()
    for h in history:
        print(f"  {h['from']} → {h['to']} ({'forced' if h['forced'] else 'normal'})")
    
    logger.info("✅ Mode transition test completed")

if __name__ == "__main__":
    main()
