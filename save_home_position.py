#!/usr/bin/env python3
"""
Save Current Position as Home
============================
"""

from pymycobot import MyCobot280
import time
import json
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def save_home_position():
    try:
        # ロボット接続
        robot = MyCobot280('/dev/ttyAMA0', 1000000)
        time.sleep(1.0)
        
        if not robot.is_controller_connected():
            logger.error("❌ Robot not connected")
            return
        
        # 現在角度取得
        current_angles = robot.get_angles()
        if not current_angles or len(current_angles) != 6:
            logger.error("❌ Could not get current angles")
            return
        
        # 現在座標取得
        current_coords = robot.get_coords()
        
        logger.info(f"📍 Current angles: {current_angles}")
        if current_coords:
            logger.info(f"📍 Current coords: {current_coords}")
        
        # ホームポジション定義
        home_position = {
            "name": "home",
            "angles": current_angles,
            "coords": current_coords if current_coords else None,
            "description": f"Home position saved at {time.ctime()}",
            "timestamp": time.time()
        }
        
        # 位置設定ファイルに保存
        config_file = "positions.json"
        try:
            with open(config_file, 'r') as f:
                positions = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            positions = {}
        
        # ホーム位置更新
        positions["home"] = home_position
        
        # ファイルに保存
        with open(config_file, 'w') as f:
            json.dump(positions, f, indent=2, ensure_ascii=False)
        
        logger.info("✅ Home position saved successfully!")
        logger.info(f"💾 Saved to: {config_file}")
        logger.info(f"🏠 New home angles: {current_angles}")
        
        # 確認のため青色点灯
        robot.set_color(0, 255, 0)  # 緑色で保存成功
        time.sleep(1)
        robot.set_color(255, 255, 0)  # 黄色に戻す（手動モード）
        
    except Exception as e:
        logger.error(f"❌ Failed to save home position: {e}")

if __name__ == "__main__":
    logger.info("🏠 Saving current position as home...")
    save_home_position()
    logger.info("✅ Complete!")
