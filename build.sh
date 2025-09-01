#!/bin/bash
# cobot パッケージビルド・インストールスクリプト
# 使用方法: ./build

echo "🔨 cobotパッケージビルド開始..."

# ワークスペースディレクトリに移動
cd /home/ros2/ros2_ws

# 既存のビルドファイルをクリーンアップ（オプション）
if [ "$1" = "clean" ]; then
    echo "🧹 クリーンビルド実行中..."
    rm -rf build/cobot install/cobot log/cobot
fi

# cobotパッケージのみビルド
echo "🔧 colcon build実行中..."
colcon build --packages-select cobot

# ビルド結果チェック
if [ $? -eq 0 ]; then
    echo "✅ ビルド完了"
    
    # インストール環境を更新
    echo "📦 インストール環境更新中..."
    source install/setup.bash
    
    echo "✅ cobotパッケージのビルド・インストール完了！"
    echo ""
    echo "🚀 使用方法:"
    echo "  ./cobot_start  - cobotノード起動"
    echo "  ./status       - ステータス確認"
    echo "  ./calibrate    - キャリブレーション実行"
    echo ""
else
    echo "❌ ビルド失敗"
    exit 1
fi
