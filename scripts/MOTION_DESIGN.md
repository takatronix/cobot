# モーション保存システム設計

## 📝 基本機能

### 1. 記録モード
```bash
./motion_save demo_motion [sampling_hz]
# → フリーモードに切り替え
# → 角度記録開始（デフォルト：20Hz = 0.05秒間隔）
# → 手動でロボット操作
# → 30秒で自動終了 or ./motion_stop で手動終了
```

### 2. 再生モード  
```bash
./motion_play demo_motion [speed]
# → 記録した軌道を再生
# → speed: 0.5〜2.0（デフォルト：1.0）
# → 例: ./motion_play demo_motion 0.8 （0.8倍速）
```

### 3. 管理機能
```bash
./motion_list    # モーション一覧
./motion_delete  # モーション削除
```

## 🗂️ データ構造

### motions/[name].json
```json
{
  "name": "demo_motion",
  "duration": 12.5,
  "sampling_rate": 0.05,
  "max_duration": 30.0,
  "points": [
    {
      "time": 0.0,
      "angles": [0, 0, 0, 0, 0, 0],
      "gripper": 50
    },
    {
      "time": 0.1, 
      "angles": [1.2, -2.3, 1.5, 0.8, -0.5, 2.1],
      "gripper": 45
    }
  ],
  "created_at": "2025-08-02T17:30:00"
}
```

## 🔧 実装詳細

### ROS2インターフェース
- `/cobot/motion_record` (Service) - 記録開始/停止
- `/cobot/motion_play` (Service) - 再生
- `/cobot/motion_status` (Topic) - 記録状態

### シェルスクリプト
- `motion_save` - 記録開始スクリプト
- `motion_play` - 再生スクリプト  
- `motion_list` - 一覧表示
- `motion_delete` - 削除

### 安全機能
- 記録中の異常角度チェック
- 再生時の安全制限確認
- 緊急停止対応

## 🎯 実装順序

1. **モーション記録クラス** (MotionRecorder)
2. **ROS2サービス追加** (cobot_node.py)
3. **シェルスクリプト作成**
4. **テスト・デバッグ**