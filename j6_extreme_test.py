#!/usr/bin/env python3
"""
J6 Extreme Range Test
====================
J6の端から端移動の安全性テスト
"""

from pymycobot import MyCobot280
import time
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def normalize_angle(angle):
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle

def safe_j6_move(robot, target_j6, speed=20):
    """J6のみ安全移動"""
    current = robot.get_angles()
    if not current: return False
    
    current_j6 = current[5]
    diff = normalize_angle(target_j6 - current_j6)
    
    logger.info(f'🎯 J6移動: {current_j6:.1f}° → {target_j6:.1f}° (差分: {diff:.1f}°)')
    
    # 段階移動
    max_steps = 6
    step_size = 50.0
    
    for step in range(max_steps):
        current = robot.get_angles()
        if not current: return False
        
        current_j6 = current[5]
        remaining = normalize_angle(target_j6 - current_j6)
        
        if abs(remaining) <= 3.0:
            logger.info(f'✅ J6移動完了: {current_j6:.1f}°')
            return True
        
        # 次のステップ
        if abs(remaining) <= step_size:
            next_j6 = target_j6
        else:
            step_angle = step_size if remaining > 0 else -step_size
            next_j6 = current_j6 + step_angle
        
        logger.info(f'📍 Step {step+1}: {current_j6:.1f}° → {next_j6:.1f}°')
        
        # J6のみ変更
        next_angles = current.copy()
        next_angles[5] = next_j6
        robot.send_angles(next_angles, speed)
        time.sleep(2.5)
    
    logger.warning('⚠️ 移動未完了')
    return False

def main():
    logger.info('🧪 J6極限テスト開始')
    
    robot = MyCobot280('/dev/ttyAMA0', 1000000)
    time.sleep(1)
    
    if not robot.is_controller_connected():
        logger.error('❌ Robot not connected')
        return
    
    # 現在位置
    start_pos = robot.get_angles()
    logger.info(f'📍 開始位置: J6 = {start_pos[5]:.1f}°')
    
    # 極限テストシーケンス
    extreme_tests = [
        ('🔄 左極限', -170),
        ('🔄 右極限', +170), 
        ('🔄 左極限再び', -170),
        ('🏠 ホーム復帰', 0),
    ]
    
    print()
    print('=== J6極限可動テスト ===')
    print('⚠️  危険: 340度の大回転!')
    print('🛡️  安全制御で段階移動')
    print()
    
    for test_name, target in extreme_tests:
        print(f'{test_name} ({target}°)')
        print('実行しますか？ (y/N): ', end='')
        
        response = input()
        if response.lower() != 'y':
            print('スキップ')
            continue
        
        start_time = time.time()
        success = safe_j6_move(robot, target, 20)
        duration = time.time() - start_time
        
        if success:
            result = robot.get_angles()
            actual = result[5] if result else 0
            error = abs(actual - target)
            print(f'✅ 成功: {actual:.1f}° (誤差: {error:.1f}°, 時間: {duration:.1f}s)')
        else:
            print('❌ 失敗')
            break
        
        print()
    
    print('🎉 J6極限テスト完了')
    
    # 最終位置確認
    final_pos = robot.get_angles()
    if final_pos:
        logger.info(f'📍 最終位置: J6 = {final_pos[5]:.1f}°')

if __name__ == '__main__':
    main()
