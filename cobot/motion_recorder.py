#!/usr/bin/env python3
"""
Motion Recorder for myCobot280 - デバッグ版
ロボットの動作を記録・再生するためのクラス（大量のデバッグログ付き）
"""

import os
import json
import time
import threading
from dataclasses import dataclass, asdict
from typing import List, Optional, Dict, Any
from datetime import datetime

@dataclass
class MotionPoint:
    """単一のモーションポイント"""
    time: float
    angles: List[float]
    gripper_value: Optional[float] = None

@dataclass 
class Motion:
    """モーション全体のデータ"""
    name: str
    duration: float
    sampling_rate: float
    max_duration: float
    description: str
    created_at: str
    points: List[MotionPoint]

class MotionRecorder:
    """モーション記録・再生クラス（デバッグ版）"""
    
    def __init__(self, data_dir="/home/ros2/cobot"):
        """初期化"""
        print(f"🔧 [DEBUG] MotionRecorder.__init__ called with data_dir={data_dir}")
        
        self.data_dir = data_dir
        self.motions_dir = os.path.join(data_dir, "motions")
        
        # 記録設定
        self.default_sampling_rate = 0.1   # 10Hz (安定性と滑らかさのバランス)
        self.max_duration = 30.0  # 30秒
        
        # 記録状態
        self.is_recording = False
        self.current_motion_name = None
        self.recording_thread = None
        self.start_time = None
        self.motion_points = []
        
        # 再生状態
        self.is_playing = False
        self.current_playing_motion = None
        self.playback_thread = None
        self.should_stop_playback = False
        
        # ロボット参照
        self.robot = None
        self.cobot_node = None  # cobot_nodeインスタンス
        
        # ディレクトリ作成
        os.makedirs(self.motions_dir, exist_ok=True)
        print(f"🔧 [DEBUG] MotionRecorder initialized. motions_dir={self.motions_dir}")
        
    def set_robot(self, robot):
        """ロボットインスタンス設定"""
        print(f"🔧 [DEBUG] set_robot called. robot={type(robot) if robot else None}")
        self.robot = robot
        print(f"🔧 [DEBUG] Robot set successfully")
        
    def set_cobot_node(self, cobot_node):
        """cobot_nodeインスタンス設定"""
        print(f"🔧 [DEBUG] set_cobot_node called. cobot_node={type(cobot_node) if cobot_node else None}")
        self.cobot_node = cobot_node
        print(f"🔧 [DEBUG] Cobot node set successfully")
        
    def start_recording(self, motion_name: str, sampling_rate: float = 0.0) -> tuple[bool, str]:
        """記録開始"""
        print(f"🔧 [DEBUG] start_recording called: motion_name='{motion_name}', sampling_rate={sampling_rate}")
        
        try:
            if self.is_recording:
                print("🔧 [DEBUG] Already recording, returning False")
                return False, "既に記録中です"
                
            if not self.robot:
                print("🔧 [DEBUG] Robot not set, returning False")
                return False, "ロボット未接続"
                
            # サンプリング周波数設定
            if sampling_rate <= 0:
                sampling_rate = self.default_sampling_rate
                
            print(f"🔧 [DEBUG] Starting recording with sampling_rate={sampling_rate}")
            
            # 記録開始
            self.is_recording = True
            self.current_motion_name = motion_name
            self.start_time = time.time()
            self.motion_points = []
            
            print(f"🔧 [DEBUG] Recording state set. Creating thread...")
            
            # 記録スレッド開始
            self.recording_thread = threading.Thread(
                target=self._record_loop,
                args=(sampling_rate,),
                daemon=True
            )
            print(f"🔧 [DEBUG] Thread created, starting...")
            self.recording_thread.start()
            print(f"🔧 [DEBUG] Thread started successfully")
            
            return True, f"記録開始: {motion_name}"
            
        except Exception as e:
            print(f"🔧 [DEBUG] Exception in start_recording: {e}")
            self.is_recording = False
            return False, f"記録開始エラー: {e}"
    
    def _record_loop(self, sampling_rate: float):
        """記録ループ（別スレッド）"""
        print(f"🔧 [DEBUG] _record_loop started. sampling_rate={sampling_rate}")
        
        try:
            loop_count = 0
            while self.is_recording:
                loop_count += 1
                # デバッグログを100イテレーション毎に減らす
                if loop_count % 100 == 1:
                    print(f"🔧 [DEBUG] Record loop iteration {loop_count}")
                
                current_time = time.time()
                elapsed = current_time - self.start_time
                
                # 最大時間チェック
                if elapsed >= self.max_duration:
                    print(f"⏰ 最大記録時間({self.max_duration}秒)に達しました")
                    break
                
                # 現在の状態取得（cobot_nodeから）
                try:
                    # **重要** - cobot_nodeの current_angles を使用
                    if self.cobot_node and hasattr(self.cobot_node, 'current_angles'):
                        angles = self.cobot_node.current_angles.copy()  # コピーして安全に使用
                    else:
                        # フォールバック（通常は発生しない）
                        if loop_count % 100 == 1:
                            print("⚠️ cobot_node.current_angles not available, skipping")
                        continue
                    
                    if angles is None:
                        if loop_count % 100 == 1:
                            print("⚠️ 角度データがNone")
                        continue
                    
                    # キャッシュされたグリッパー値を使用（シリアル通信回避）
                    gripper_value = self.cobot_node.current_gripper_value if self.cobot_node else None
                    
                    # データ有効性チェック
                    if not isinstance(angles, (list, tuple)):
                        if loop_count % 100 == 1:
                            print(f"⚠️ 角度データが配列ではありません: {angles} (type: {type(angles)})")
                        continue
                        
                    if len(angles) != 6:
                        if loop_count % 100 == 1:
                            print(f"⚠️ 無効な角度データ: {angles}")
                        continue
                    
                    point = MotionPoint(
                        time=elapsed,
                        angles=angles,
                        gripper_value=gripper_value
                    )
                    self.motion_points.append(point)
                    
                    # ポイント追加のログも減らす
                    if loop_count % 100 == 1:
                        print(f"🔧 [DEBUG] Point added. Total points: {len(self.motion_points)}")
                    
                except Exception as e:
                    print(f"❌ データ取得エラー: {e}")
                    print(f"🔧 [DEBUG] Exception details: {type(e).__name__}: {e}")
                    # エラーが続く場合は記録停止
                    if len(self.motion_points) == 0 and elapsed > 5.0:
                        print("❌ 連続エラーのため記録停止")
                        break
                
                # サンプリング間隔待機
                print(f"🔧 [DEBUG] Sleeping for {sampling_rate}s...")
                time.sleep(sampling_rate)
                
        except Exception as e:
            print(f"❌ 記録ループエラー: {e}")
            print(f"🔧 [DEBUG] Record loop exception: {type(e).__name__}: {e}")
        finally:
            print(f"🔧 [DEBUG] Record loop ending, calling finalize...")
            # 記録終了処理
            if self.is_recording:
                self._finalize_recording(sampling_rate)
    
    def stop_recording(self) -> tuple[bool, str]:
        """記録停止（非ブロッキング）"""
        print(f"🔧 [DEBUG] stop_recording called")
        
        try:
            if not self.is_recording:
                print("🔧 [DEBUG] Not recording, returning False")
                return False, "記録中ではありません"
                
            print("⏹️ 記録停止要求受信")
            self.is_recording = False
            print("✅ 記録停止フラグ設定完了（スレッドは自然終了）")
            
            # 少し待ってからスレッド終了確認
            if self.recording_thread and self.recording_thread.is_alive():
                print("🔧 [DEBUG] Waiting for recording thread to finish...")
                self.recording_thread.join(timeout=2.0)
                if self.recording_thread.is_alive():
                    print("⚠️ Recording thread still alive after 2s timeout")
                else:
                    print("✅ Recording thread finished")
            
            return True, "記録停止完了"
            
        except Exception as e:
            print(f"❌ 記録停止例外: {e}")
            print(f"🔧 [DEBUG] Stop recording exception: {type(e).__name__}: {e}")
            return False, f"記録停止エラー: {e}"
    
    def _finalize_recording(self, sampling_rate: float):
        """記録終了処理"""
        print(f"🔧 [DEBUG] _finalize_recording called with {len(self.motion_points)} points")
        
        try:
            self.is_recording = False
            
            if not self.motion_points:
                print("❌ 記録データがありません")
                return
                
            # モーション作成
            duration = self.motion_points[-1].time if self.motion_points else 0.0
            motion = Motion(
                name=self.current_motion_name,
                duration=duration,
                sampling_rate=sampling_rate,
                max_duration=self.max_duration,
                description=f"記録時間: {duration:.1f}秒, ポイント数: {len(self.motion_points)}",
                created_at=datetime.now().strftime("%a %b %d %H:%M:%S %Y"),
                points=self.motion_points
            )
            
            print(f"🔧 [DEBUG] Motion object created: {motion.name}, duration={motion.duration}")
            
            # ファイル保存
            print(f"🔧 [DEBUG] Calling _save_motion...")
            success, message = self._save_motion(motion)
            if success:
                print(f"✅ モーション保存: {self.current_motion_name} ({len(self.motion_points)}ポイント)")
            else:
                print(f"❌ 保存失敗: {message}")
                
        except Exception as e:
            print(f"❌ 記録終了処理エラー: {e}")
            print(f"🔧 [DEBUG] Finalize exception: {type(e).__name__}: {e}")
    
    def _save_motion(self, motion: Motion) -> tuple[bool, str]:
        """モーション保存"""
        print(f"🔧 [DEBUG] _save_motion called for motion: {motion.name}")
        
        try:
            file_path = os.path.join(self.motions_dir, f"{motion.name}.json")
            print(f"🔧 [DEBUG] Saving to file: {file_path}")
            
            # データクラスをdict変換
            motion_dict = {
                "name": motion.name,
                "duration": motion.duration,
                "sampling_rate": motion.sampling_rate,
                "max_duration": motion.max_duration,
                "description": motion.description,
                "created_at": motion.created_at,
                "points": [asdict(point) for point in motion.points]
            }
            
            print(f"🔧 [DEBUG] Motion dict created with {len(motion_dict['points'])} points")
            
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(motion_dict, f, indent=2, ensure_ascii=False)
                
            print(f"🔧 [DEBUG] File written successfully to {file_path}")
            return True, f"保存完了: {file_path}"
            
        except Exception as e:
            print(f"🔧 [DEBUG] Save exception: {type(e).__name__}: {e}")
            return False, f"保存エラー: {e}"

    def play_motion(self, motion_name: str, speed: float = 1.0) -> tuple[bool, str]:
        """モーション再生"""
        print(f"🔧 [DEBUG] play_motion called: {motion_name}, speed={speed}")
        
        try:
            if self.is_playing:
                return False, f"既に再生中です: {self.current_playing_motion}"
            
            # モーションファイル読み込み
            motion_file = os.path.join(self.motions_dir, f"{motion_name}.json")
            if not os.path.exists(motion_file):
                return False, f"モーションファイルが見つかりません: {motion_name}"
            
            with open(motion_file, 'r', encoding='utf-8') as f:
                motion_data = json.load(f)
            
            # Motion オブジェクト作成
            motion = Motion(
                name=motion_data['name'],
                duration=motion_data['duration'],
                sampling_rate=motion_data['sampling_rate'],
                max_duration=motion_data['max_duration'],
                description=motion_data['description'],
                created_at=motion_data['created_at'],
                points=[MotionPoint(**point) for point in motion_data['points']]
            )
            
            print(f"🔧 [DEBUG] Motion loaded: {len(motion.points)} points")
            
            # 既存スレッドの終了を待機
            if self.playback_thread and self.playback_thread.is_alive():
                print("🕰️ 既存スレッド終了待機中...")
                self.should_stop_playback = True
                self.playback_thread.join(timeout=2.0)
            
            # フラグリセット
            self.should_stop_playback = False
            
            # 再生スレッド開始
            self.is_playing = True
            self.current_playing_motion = motion_name
            
            print(f"🎦 再生スレッド開始: {motion_name}")
            self.playback_thread = threading.Thread(
                target=self._playback_worker,
                args=(motion, speed),
                daemon=True
            )
            self.playback_thread.start()
            
            return True, f"再生開始: {motion_name} (速度: {speed}x)"
            
        except Exception as e:
            print(f"❗ 再生エラー: {e}")
            return False, f"再生エラー: {e}"
    
    def _playback_worker(self, motion: Motion, speed: float):
        """再生ワーカー（別スレッド）"""
        start_time = time.time()
        print(f"▶️ 再生開始: {motion.name} (速度: {speed}x, 非同期)")
        print(f"🕰️ 記録時間: {motion.duration:.1f}秒, ポイント数: {len(motion.points)}")
        print(f"🔧 [DEBUG] ロボット接続状態: {self.robot is not None}")
        
        try:
            for i, point in enumerate(motion.points):
                if self.should_stop_playback:
                    print("⏹️ 再生停止要求")
                    break
                
                # 関節角度設定（MyCobot280が内部でスレッドセーフ）
                try:
                    self.robot.send_angles(point.angles, 100)  # 最高速度で移動
                    # デバッグログを100ポイント毎に減らす
                    if i % 100 == 0:
                        print(f"🔧 [DEBUG] Point {i}: angles={point.angles}")
                except Exception as e:
                    print(f"⚠️ 移動エラー (ポイント{i}): {e}")
                
                # グリッパー設定（値の範囲チェック付き）
                if point.gripper_value is not None:
                    try:
                        # グリッパー値を0-100の範囲にクランプ
                        gripper_value = max(0, min(100, point.gripper_value))
                        
                        # 正しい理解：0=完全に閉じる、100=完全に開く
                        # 古いデータの値をそのまま使用（変換不要）
                        # 値の範囲チェックのみ行う
                        if gripper_value < 0:
                            gripper_value = 0
                            print(f"🤏 グリッパー修正: 範囲外 → 0 (完全に閉じる)")
                        elif gripper_value > 100:
                            gripper_value = 100
                            print(f"🤏 グリッパー修正: 範囲外 → 100 (完全に開く)")
                        
                        if gripper_value != point.gripper_value:
                            print(f"⚠️ グリッパー値を修正: {point.gripper_value} → {gripper_value}")
                        
                        self.robot.set_gripper_value(gripper_value, 100)  # 最高速度で動作
                        # デバッグログを100ポイント毎に減らす
                        if i % 100 == 0:
                            print(f"🤏 [DEBUG] Point {i}: gripper={gripper_value}")
                        # set_gripper_value()は同期関数なので待機不要
                        
                    except Exception as e:
                        print(f"⚠️ グリッパーエラー (ポイント{i}): {e}")
                
                # 実行時間を考慮した待機時間調整
                if i < len(motion.points) - 1:
                    # 次のポイントまでの理想的な時間間隔
                    next_time = motion.points[i + 1].time
                    target_interval = (next_time - point.time) / speed
                    
                    # 現在の経過時間を計算
                    current_elapsed = time.time() - start_time
                    expected_elapsed = point.time / speed
                    
                    # 累積遅延を考慮した待機時間
                    delay_compensation = expected_elapsed - current_elapsed
                    adjusted_wait = target_interval + delay_compensation
                    
                    # 最小待機時間を保証（負の値を防ぐ）
                    final_wait = max(0.001, adjusted_wait)  # 1ms最小
                    
                    # デバッグ情報（100ポイント毎）
                    if i % 100 == 0:
                        print(f"🕰️ [DEBUG] Point {i}: target={target_interval:.3f}s, delay={delay_compensation:+.3f}s, wait={final_wait:.3f}s")
                    
                    time.sleep(final_wait)
            
            end_time = time.time()
            actual_duration = end_time - start_time
            expected_duration = motion.duration / speed
            print(f"▶️ 再生完了: {motion.name} (速度: {speed}x)")
            print(f"🕰️ 実際再生時間: {actual_duration:.1f}秒 (期待: {expected_duration:.1f}秒)")
            if abs(actual_duration - expected_duration) > 1.0:
                print(f"⚠️ 時間誤差: {actual_duration - expected_duration:+.1f}秒")
            
        except Exception as e:
            print(f"❗ 再生処理エラー: {e}")
        finally:
            self.is_playing = False
            self.current_playing_motion = None
            self.should_stop_playback = False
    
    def stop_playback(self):
        """再生停止"""
        if self.is_playing:
            self.should_stop_playback = True
            print("⏹️ 再生停止要求")
            
            # スレッドの終了を待機
            if self.playback_thread and self.playback_thread.is_alive():
                print("🕰️ 再生スレッド終了待機中...")
                self.playback_thread.join(timeout=3.0)  # 3秒タイムアウト
                if self.playback_thread.is_alive():
                    print("⚠️ 再生スレッドが強制終了されました")
                else:
                    print("✅ 再生スレッドが正常終了")
    
    def list_motions(self) -> List[str]:
        try:
            if not os.path.exists(self.motions_dir):
                return []
            files = [f[:-5] for f in os.listdir(self.motions_dir) 
                    if f.endswith('.json')]
            return sorted(files)
        except Exception as e:
            print(f"❌ リスト取得エラー: {e}")
            return []