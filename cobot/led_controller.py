#!/usr/bin/env python3
"""
cobot280 LED Controller - Basic Test Version
===========================================

myCobot280のATOM LED制御テスト

Author: Takashi Otsuka (takatronix@gmail.com)
"""

from pymycobot import MyCobot280
import time
import logging

# ログ設定
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_led_basic():
    """基本的なLED制御テスト"""
    logger.info("🤖 Basic LED Test Starting...")
    
    try:
        # ロボット接続
        robot = MyCobot280("/dev/ttyAMA0", 1000000)
        time.sleep(0.5)
        
        if not robot.is_connected():
            logger.error("❌ Robot connection failed")
            return
        
        logger.info("✅ Robot connected")
        
        # 色テスト
        colors = [
            ("🟡 Manual (Yellow)", 255, 255, 0),
            ("🔵 Auto Working (Blue)", 0, 0, 255),
            ("🟢 Auto Ready (Green)", 0, 255, 0),
            ("🟣 AI Thinking (Purple)", 128, 0, 128),
            ("⚪ AI Waiting (White)", 255, 255, 255),
            ("🩷 Calibration (Pink)", 255, 192, 203),
            ("🔴 Error (Red)", 255, 0, 0),
            ("🔶 Emergency (Orange)", 255, 165, 0),
            ("💙 Initializing (Cyan)", 0, 255, 255),
        ]
        
        for name, r, g, b in colors:
            logger.info(f"{name}: RGB({r}, {g}, {b})")
            robot.set_color(r, g, b)
            time.sleep(2)
        
        logger.info("✅ LED test completed")
        
        # 手動モードテスト
        logger.info("🟡 Testing manual mode (free mode)...")
        robot.set_free_mode(1)
        time.sleep(3)
        
        # 緊急停止テスト
        logger.info("🔶 Testing emergency stop...")
        robot.release_all_servos()
        robot.set_color(255, 165, 0)  # オレンジ
        time.sleep(2)
        
        # 復帰
        robot.set_free_mode(0)
        robot.set_color(0, 255, 0)  # 緑
        logger.info("✅ All basic tests completed")
        
    except Exception as e:
        logger.error(f"❌ Test failed: {e}")
    
    finally:
        try:
            robot.set_color(0, 0, 0)  # LED OFF
        except:
            pass

if __name__ == "__main__":
    test_led_basic()
