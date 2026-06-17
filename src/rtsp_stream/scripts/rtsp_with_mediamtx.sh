#!/bin/bash
# 使用mediamtx作为RTSP服务器

DISPLAY_NUM="${DISPLAY:-:0}"
RTSP_PORT="${1:-8554}"
STREAM_NAME="${2:-mapping}"
FPS="${3:-15}"
RESOLUTION="${4:-1920x1080}"

echo "=========================================="
echo "RTSP视频流 (使用mediamtx)"
echo "=========================================="
echo "显示器: $DISPLAY_NUM"
echo "RTSP端口: $RTSP_PORT"
echo "流名称: $STREAM_NAME"
echo "帧率: $FPS FPS"
echo "分辨率: $RESOLUTION"
echo "=========================================="
echo ""

# 检查mediamtx
if ! command -v mediamtx &> /dev/null; then
    echo "错误: mediamtx未安装"
    echo ""
    echo "安装方法:"
    echo "  bash install_mediamtx.sh"
    echo ""
    echo "或手动安装:"
    echo "  wget https://github.com/bluenviron/mediamtx/releases/download/v1.5.0/mediamtx_v1.5.0_linux_arm64v8.tar.gz"
    echo "  tar -xzf mediamtx_v1.5.0_linux_arm64v8.tar.gz"
    echo "  sudo mv mediamtx /usr/local/bin/"
    exit 1
fi

# 检查FFmpeg
if ! command -v ffmpeg &> /dev/null; then
    echo "错误: FFmpeg未安装"
    echo "安装: sudo apt-get install ffmpeg"
    exit 1
fi

# 获取本机IP
LOCAL_IP=$(hostname -I | awk '{print $1}')

echo "RTSP流地址:"
echo "  本地: rtsp://localhost:$RTSP_PORT/$STREAM_NAME"
echo "  远程: rtsp://$LOCAL_IP:$RTSP_PORT/$STREAM_NAME"
echo ""
echo "播放命令:"
echo "  vlc rtsp://$LOCAL_IP:$RTSP_PORT/$STREAM_NAME"
echo ""
echo "按Ctrl+C停止"
echo "=========================================="
echo ""

# 清理函数
cleanup() {
    echo ""
    echo "正在停止..."
    kill $FFMPEG_PID 2>/dev/null
    kill $MEDIAMTX_PID 2>/dev/null
    rm -f /tmp/mediamtx.yml
    exit 0
}

trap cleanup SIGINT SIGTERM

# 创建mediamtx配置
cat > /tmp/mediamtx.yml << EOF
# mediamtx配置
rtspAddress: :$RTSP_PORT
rtmpAddress: :1935
hlsAddress: :8888
webrtcAddress: :8889

paths:
  $STREAM_NAME:
    runOnInit: echo "Stream started"
    runOnDemand: echo "Stream requested"
EOF

# 启动mediamtx
echo "启动mediamtx服务器..."
mediamtx /tmp/mediamtx.yml &
MEDIAMTX_PID=$!

# 等待mediamtx启动
sleep 3

# 使用FFmpeg捕获屏幕并推送到mediamtx
echo "启动屏幕捕获..."
# ffmpeg \
#     -f x11grab \
#     -video_size $RESOLUTION \
#     -framerate $FPS \
#     -i $DISPLAY_NUM \
#     -c:v libx264 \
#     -preset ultrafast \
#     -tune zerolatency \
#     -b:v 2M \
#     -maxrate 2M \
#     -bufsize 4M \
#     -pix_fmt yuv420p \
#     -g $(($FPS * 2)) \
#     -f rtsp \
#     -rtsp_transport tcp \
#     rtsp://localhost:$RTSP_PORT/$STREAM_NAME &

FFMPEG_PID=$!

echo ""
echo "服务器已启动！"
echo "mediamtx PID: $MEDIAMTX_PID"
echo "FFmpeg PID: $FFMPEG_PID"
echo ""
echo "等待几秒后可以开始播放..."

# 保持运行
wait
