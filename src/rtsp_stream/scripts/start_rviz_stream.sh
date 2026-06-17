#!/bin/bash
# 启动RViz窗口捕获和RTSP流

echo "=========================================="
echo "RViz建图视频流启动脚本"
echo "=========================================="
echo ""

# 检查依赖
echo "检查依赖..."

if ! command -v ffmpeg &> /dev/null; then
    echo "✗ FFmpeg未安装"
    echo "  安装: sudo apt-get install ffmpeg"
    exit 1
fi
echo "✓ FFmpeg已安装"

if ! python3 -c "import mss" 2>/dev/null; then
    echo "✗ mss库未安装"
    echo "  安装: pip3 install mss pillow"
    exit 1
fi
echo "✓ mss库已安装"

if ! command -v xdotool &> /dev/null; then
    echo "⚠ xdotool未安装（可选，用于查找RViz窗口）"
    echo "  安装: sudo apt-get install xdotool"
fi

echo ""
echo "=========================================="
echo "使用说明:"
echo "=========================================="
echo "1. 确保RViz正在运行并显示建图画面"
echo "2. 此脚本将捕获屏幕并转换为RTSP流"
echo "3. 在其他设备上使用VLC播放"
echo ""
read -p "按Enter键继续，或Ctrl+C取消..."

# 获取脚本目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 启动RViz捕获
echo ""
echo "启动RViz窗口捕获..."
python3 "$SCRIPT_DIR/rviz_to_rtsp.py"
