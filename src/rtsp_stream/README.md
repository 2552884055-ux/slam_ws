# RTSP视频流传输包

将ROS图像话题转换为RTSP视频流，用于远程查看建图过程。

## 功能特性

- 订阅ROS图像话题并转换为RTSP流
- 支持FFmpeg和GStreamer两种实现方式
- 低延迟视频传输
- 可配置分辨率、帧率和码率
- 支持多客户端同时观看

## 依赖安装

### FFmpeg方式（推荐）
```bash
sudo apt-get update
sudo apt-get install ffmpeg
```

### GStreamer方式
```bash
sudo apt-get install gstreamer1.0-tools gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly gstreamer1.0-rtsp \
    python3-gi gstreamer1.0-libav
```

### Python依赖
```bash
pip3 install opencv-python
```

## 编译

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 使用方法

### 1. 启动建图系统

首先启动livox_ros_driver2和fast_lio_global节点：

```bash
# 终端1: 启动Livox驱动
roslaunch livox_ros_driver2 msg_MID360.launch

# 终端2: 启动FAST-LIO建图
roslaunch fast_lio_global mapping_mid360.launch
```

### 2. 启动RViz可视化

```bash
# 终端3: 启动RViz
rviz -d src/FAST_LIO_GLOBAL/rviz_cfg/mapping.rviz
```

### 3. 使用image_view发布RViz截图

由于RViz本身不直接发布图像话题，需要使用截图工具：

```bash
# 方法1: 使用rqt_image_view的截图功能
rosrun rqt_image_view rqt_image_view

# 方法2: 使用rviz的截图插件并发布
# 或者使用专门的RViz图像发布节点
```

### 4. 启动RTSP流服务器

```bash
# 使用FFmpeg（推荐）
roslaunch rtsp_stream rtsp_stream.launch image_topic:=/camera/image_raw

# 使用GStreamer
roslaunch rtsp_stream rtsp_stream.launch image_topic:=/camera/image_raw use_gstreamer:=true

# 自定义参数
roslaunch rtsp_stream rtsp_stream.launch \
    image_topic:=/camera/image_raw \
    rtsp_port:=8554 \
    stream_name:=mapping \
    width:=1920 \
    height:=1080 \
    fps:=30 \
    bitrate:=4M
```

## 前端播放

### VLC播放器
```bash
vlc rtsp://YOUR_IP:8554/mapping
```

### FFplay
```bash
ffplay -rtsp_transport tcp rtsp://YOUR_IP:8554/mapping
```

### 网页播放（使用video.js）
```html
<!DOCTYPE html>
<html>
<head>
    <link href="https://vjs.zencdn.net/7.20.3/video-js.css" rel="stylesheet" />
</head>
<body>
    <video id="my-video" class="video-js" controls preload="auto" width="1280" height="720">
        <source src="rtsp://YOUR_IP:8554/mapping" type="rtsp/h264">
    </video>
    <script src="https://vjs.zencdn.net/7.20.3/video.min.js"></script>
</body>
</html>
```

### 使用WebRTC转换（推荐用于浏览器）

由于浏览器不直接支持RTSP，建议使用mediamtx进行转换：

```bash
# 安装mediamtx
wget https://github.com/bluenviron/mediamtx/releases/download/v1.0.0/mediamtx_v1.0.0_linux_amd64.tar.gz
tar -xzf mediamtx_v1.0.0_linux_amd64.tar.gz
./mediamtx

# 然后在浏览器访问
# http://YOUR_IP:8889/mapping
```

## 参数说明

- `image_topic`: 输入的ROS图像话题
- `rtsp_port`: RTSP服务器端口（默认8554）
- `stream_name`: 流名称（默认mapping）
- `fps`: 视频帧率（默认30）
- `width`: 视频宽度（默认1280）
- `height`: 视频高度（默认720）
- `bitrate`: 视频码率（默认2M）

## 故障排除

### 1. 没有图像输出
- 检查image_topic是否正确：`rostopic list`
- 检查图像是否发布：`rostopic hz /camera/image_raw`

### 2. FFmpeg启动失败
- 确认FFmpeg已安装：`ffmpeg -version`
- 检查端口是否被占用：`netstat -tuln | grep 8554`

### 3. 延迟过高
- 降低分辨率和码率
- 使用有线网络连接
- 调整编码预设为ultrafast

### 4. 连接被拒绝
- 检查防火墙设置
- 确认IP地址和端口正确
- 使用TCP传输：`-rtsp_transport tcp`

## 性能优化

1. **降低延迟**：使用ultrafast预设和zerolatency调优
2. **提高质量**：增加码率，使用slower预设
3. **节省带宽**：降低分辨率和帧率
4. **多客户端**：使用shared模式

## 许可证

MIT License
