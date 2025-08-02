# myCobot280 LED・ボタン制御仕様

## 🎯 概要
myCobot280のATOM LED制御と画面タッチボタン操作の詳細仕様

## 💡 ATOM LED制御

### 基本LED制御
```python
# PyMyCobot APIより
set_color(r, g, b)  # RGB: 0-255
```

### 状態表示LED（公式仕様+実機テスト結果）
| 状態 | 色 | RGB値 | 説明 |
|------|-----|-------|------|
| **フリーモード** | 🟡 **黄色** | **(255, 255, 0)** | **公式仕様：ボタン押下でティーチング可能** |
| 正常動作 | 🔵 青 | (0, 0, 255) | ロボット正常稼働中 |
| エラー | 🔴 赤 | (255, 0, 0) | エラー発生・通信異常 |
| 動作中 | 🔵 青 | (0, 0, 255) | 軌道実行中 |
| 待機 | ⚪ 白 | (255, 255, 255) | 待機状態 |
| 停止 | ⚫ 消灯 | (0, 0, 0) | 電源OFF/スリープ |

### ⚠️ LED自動変化の複雑な挙動（実機テスト結果）
**予測困難な動作が確認された:**
- **1回目**: 🔵青設定 → 動作中🟢緑 → 🟢緑維持
- **2回目**: 🔵青設定 → 動作中🔵青 → 🔵青維持（同条件で結果が異なる）
- **赤色**: 🔴赤設定 → 動作中🔴赤 → 🔴赤維持（一貫して維持）

**ROS2実装指針**: 青色の自動変化は不安定のため、明示的な色設定を常に行う。

## 🎮 ATOM画面タッチボタン

### 物理ボタン操作
- **デバイス**: ATOM画面タッチ操作
- **操作**: 画面を押す（物理的なタッチ）
- **確認**: 公式ドキュメントより確認済み

### フリーモード操作（重要）
```
**「when the Atom light turns yellow, you need to press and hold the Atom button to move the joint.」**
```

**操作手順:**
1. フリーモードに入る → **黄色LED点灯**
2. **ATOMボタンを押し続ける**
3. 押し続けながら関節を手で動かす
4. ボタンを離すと動作停止

### モード切替制限
- **物理ボタンでの自動切替**: ❌ **不可能**
- **制御方法**: ソフトウェア制御のみ
- **切替**: ROS2サービス経由で制御

## 🔧 ROS2インターフェース

### LED制御サービス
```bash
/cobot/set_led
  srv: cobot_msgs/srv/SetLed
  req: uint8 red, uint8 green, uint8 blue, bool blink
  res: bool success, string message

/cobot/status_led  
  srv: cobot_msgs/srv/StatusLed
  req: string status  # "normal", "error", "free_mode", "moving", "idle", "off"
  res: bool success, string message
```

### GPIO制御サービス
```bash
/cobot/set_gpio
  srv: cobot_msgs/srv/SetGpio
  req: uint8 pin, bool value
  res: bool success, bool value

/cobot/get_gpio
  srv: cobot_msgs/srv/GetGpio  
  req: uint8 pin
  res: bool success, bool value
```

## 📋 実装例

### ステータスLED制御
```python
# フリーモード開始時
self.set_color(255, 255, 0)  # 黄色点灯

# エラー発生時  
self.set_color(255, 0, 0)    # 赤色点灯

# 正常動作時
self.set_color(0, 255, 0)    # 緑色点灯
```

### 安全な起動シーケンス
```python
def safe_startup():
    # 1. 赤色点灯（初期化中）
    set_color(255, 0, 0)
    
    # 2. システム初期化
    initialize_robot()
    
    # 3. 緑色点灯（準備完了）
    set_color(0, 255, 0)
    
    # 注意: フリーモードでの起動は危険！
    # デフォルト: サーボON状態で起動
```

## ⚠️ 安全上の注意

### フリーモード使用時
1. **LED確認**: 黄色点灯を必ず確認
2. **ボタン操作**: ATOMボタン押し続けが必須
3. **安全範囲**: 動作範囲内での操作のみ
4. **緊急停止**: 電源断での即座停止準備

### デフォルト設定
- **起動時**: サーボON状態（安全）
- **フリーモード**: 手動サービス呼び出しのみ
- **自動復帰**: エラー時は安全状態へ復帰

## 📚 参考情報
- 公式ドキュメント: Elephant Robotics myCobot280
- API仕様: PyMyCobot library
- ハードウェア: M5Stack ATOM Matrix