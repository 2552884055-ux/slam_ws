#!/bin/bash
# 启动mediamtx服务器

CONFIG_FILE="$(rospack find rtsp_stream)/web/mediamtx.yml"

if [ ! -f "$CONFIG_FILE" ]; then
    echo "错误: 配置文件不存在: $CONFIG_FILE"
    exit 1
fi

if ! command -v mediamtx &> /dev/null; then
    echo "错误: mediamtx未安装"
    echo "请运行安装脚本: ./install_dependencies.sh"
    exit 1
fi

echo "启动mediamtx服务器..."
echo "配置文件: $CONFIG_FILE"
echo ""
echo "WebRTC端口: 8889"
echo "RTSP端口: 8554"
echo ""

mediamtx "$CONFIG_FILE"
