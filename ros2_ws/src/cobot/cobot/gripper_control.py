#!/usr/bin/env python3
"""
Gripper Control System - Simple but Powerful
==========================================
"""

from pymycobot import MyCobot280
import time
import logging
from enum import Enum
from typing import Optional, Tuple

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class GripperState(Enum):
    UNKNOWN = "unknown"
    OPEN = "open"
    CLOSED = "closed"
    MOVING = "moving"
    ERROR = "error"

class GripperController:
    def __init__(self, port="/dev/ttyAMA0", baudrate=1000000):
        self.robot = MyCobot280(port, baudrate)
        self.current_value = 50
        self.target_value = 50
        self.is_calibrated = False
        self.open_value = 100
        self.closed_value = 0
        self.protection_current = 300  # mA
        
    def connect(self) -> bool:
        """グリッパー接続・初期化"""
        try:
            time.sleep(0.5)
            if self.robot.is_controller_connected():
                logger.info("✅ Gripper connected")
                self.init_gripper()
                return True
            else:
                logger.error("❌ Gripper connection failed")
                return False
        except Exception as e:
            logger.error(f"❌ Connection error: {e}")
            return False
    
    def init_gripper(self):
        """グリッパー初期化"""
        try:
            self.robot.init_gripper()
            logger.info("🔧 Gripper initialized")
            
            # 保護電流設定
            self.robot.set_gripper_protect_current(self.protection_current)
            logger.info(f"🛡️ Protection current set: {self.protection_current}mA")
            
            # 現在値取得
            self.update_current_value()
            
        except Exception as e:
            logger.error(f"❌ Init failed: {e}")
    
    def update_current_value(self) -> Optional[int]:
        """現在値更新"""
        try:
            value = self.robot.get_gripper_value()
            if value is not None:
                self.current_value = value
                logger.debug(f"📊 Current value: {value}")
                return value
        except Exception as e:
            logger.error(f"❌ Failed to get value: {e}")
        return None
    
    def set_value(self, value: int, speed: int = 50) -> bool:
        """グリッパー値設定"""
        if not (0 <= value <= 100):
            logger.error(f"❌ Invalid value: {value} (must be 0-100)")
            return False
        
        if not (1 <= speed <= 100):
            logger.error(f"❌ Invalid speed: {speed} (must be 1-100)")
            return False
        
        try:
            self.robot.set_gripper_value(value, speed)
            self.target_value = value
            logger.info(f"🎯 Gripper set to {value} (speed: {speed})")
            return True
        except Exception as e:
            logger.error(f"❌ Set failed: {e}")
            return False
    
    def open_gripper(self, speed: int = 50) -> bool:
        """グリッパーを開く"""
        logger.info("🔓 Opening gripper...")
        return self.set_value(self.open_value, speed)
    
    def close_gripper(self, speed: int = 50) -> bool:
        """グリッパーを閉じる"""
        logger.info("🤏 Closing gripper...")
        return self.set_value(self.closed_value, speed)
    
    def set_position(self, position: str, speed: int = 50) -> bool:
        """名前付き位置設定"""
        positions = {
            "open": self.open_value,
            "closed": self.closed_value,
            "half": 50,
            "quarter": 25,
            "three_quarter": 75
        }
        
        if position not in positions:
            logger.error(f"❌ Unknown position: {position}")
            return False
        
        value = positions[position]
        logger.info(f"📍 Moving to position '{position}' (value: {value})")
        return self.set_value(value, speed)
    
    def calibrate(self, open_value: int = 100, closed_value: int = 0) -> bool:
        """グリッパー校正"""
        logger.info(f"🎯 Calibrating gripper: open={open_value}, closed={closed_value}")
        
        try:
            # 校正実行
            self.robot.set_gripper_calibration()
            
            # 設定値更新
            self.open_value = open_value
            self.closed_value = closed_value
            self.is_calibrated = True
            
            logger.info("✅ Calibration completed")
            return True
        except Exception as e:
            logger.error(f"❌ Calibration failed: {e}")
            return False
    
    def stop(self) -> bool:
        """グリッパー停止"""
        try:
            self.robot.gripper_stop()
            logger.info("⏹️ Gripper stopped")
            return True
        except Exception as e:
            logger.error(f"❌ Stop failed: {e}")
            return False
    
    def is_moving(self) -> bool:
        """動作中確認"""
        try:
            return self.robot.is_gripper_moving()
        except Exception as e:
            logger.error(f"❌ Movement check failed: {e}")
            return False
    
    def get_state(self) -> GripperState:
        """グリッパー状態取得"""
        try:
            if self.is_moving():
                return GripperState.MOVING
            
            self.update_current_value()
            
            if self.current_value is None:
                return GripperState.ERROR
            
            if self.current_value >= self.open_value - 5:
                return GripperState.OPEN
            elif self.current_value <= self.closed_value + 5:
                return GripperState.CLOSED
            else:
                return GripperState.UNKNOWN
                
        except Exception as e:
            logger.error(f"❌ State check failed: {e}")
            return GripperState.ERROR
    
    def wait_for_completion(self, timeout: float = 5.0) -> bool:
        """動作完了待機"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if not self.is_moving():
                logger.info("✅ Movement completed")
                return True
            time.sleep(0.1)
        
        logger.warning("⚠️ Movement timeout")
        return False
    
    def get_status(self) -> dict:
        """状態情報取得"""
        self.update_current_value()
        
        return {
            "current_value": self.current_value,
            "target_value": self.target_value,
            "state": self.get_state().value,
            "is_moving": self.is_moving(),
            "is_calibrated": self.is_calibrated,
            "open_value": self.open_value,
            "closed_value": self.closed_value,
            "protection_current": self.protection_current
        }
    
    def test_sequence(self):
        """テストシーケンス実行"""
        logger.info("🧪 Starting gripper test sequence...")
        
        sequences = [
            ("Open", "open", 2),
            ("Close", "closed", 2),
            ("Half open", "half", 2),
            ("Three quarter", "three_quarter", 2),
            ("Quarter", "quarter", 2),
            ("Full open", "open", 1),
        ]
        
        for name, position, wait_time in sequences:
            logger.info(f"📍 {name}...")
            if self.set_position(position, 60):
                time.sleep(wait_time)
                status = self.get_status()
                logger.info(f"📊 Current: {status['current_value']}, State: {status['state']}")
            else:
                logger.error(f"❌ Failed: {name}")
                break
        
        logger.info("✅ Test sequence completed")

def main():
    logger.info("🦾 Gripper Control Test Starting...")
    
    gripper = GripperController()
    
    if not gripper.connect():
        logger.error("❌ Failed to connect gripper")
        return
    
    # 初期状態表示
    status = gripper.get_status()
    logger.info(f"📊 Initial status: {status}")
    
    # 基本動作テスト
    logger.info("🧪 Basic operation test...")
    
    gripper.open_gripper(50)
    time.sleep(2)
    
    gripper.close_gripper(50)
    time.sleep(2)
    
    gripper.set_value(65, 50)
    time.sleep(2)
    
    # 名前付き位置テスト
    logger.info("🧪 Named position test...")
    gripper.set_position("half", 60)
    time.sleep(2)
    
    # テストシーケンス実行
    gripper.test_sequence()
    
    # 最終状態表示
    final_status = gripper.get_status()
    logger.info(f"📊 Final status: {final_status}")
    
    logger.info("✅ Gripper test completed")

if __name__ == "__main__":
    main()
