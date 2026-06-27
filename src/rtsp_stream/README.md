# rtsp_stream —— 建图画面 RTSP / WebRTC 远程监看

把 **RViz/建图画面（屏幕抓取）** 或 **ROS 图像话题** 经 FFmpeg 推到本地 [mediamtx](https://github.com/bluenviron/mediamtx) 流媒体网关，供远程通过 RTSP / WebRTC / HLS 观看建图过程。

> ⚠️ 推流后端是 **FFmpeg + mediamtx**。本包并**不包含可用的 GStreamer 实现**，README 历史版本里的 GStreamer 说法已废弃。

---

## 1. 两条推流路径

| 路径 | 实现 | 输入 | 适用 |
|------|------|------|------|
| **屏幕抓取** | C++ `rtsp_streamer`（FFmpeg `x11grab`） | X11 桌面/RViz 窗口 | 直接把整屏/RViz 画面推出去，不依赖 ROS 图像话题 |
| **ROS 图像** | Python `rtsp_server.py`（cv_bridge + FFmpeg 子进程） | `~image_topic` 图像话题 | 已有图像话题（含 `rviz_capture.py` 抓屏发布的 `/rviz/image`） |

两条路径都把流推给 mediamtx；浏览器端通过 `web/index.html`（WebRTC）或 VLC/ffplay（RTSP）观看。

```
 [X11 桌面] ──x11grab──► rtsp_streamer ─┐
                                        ├─► mediamtx ──► RTSP / WebRTC / HLS ──► VLC / 浏览器
 ROS 图像 ──► rtsp_server.py ──ffmpeg──┘
 (rviz_capture.py 用 mss 抓 RViz 窗口 → 发布 /rviz/image)
```

---

## 2. 组成

### 可执行 / 脚本
| 文件 | 类型 | 作用 |
|------|------|------|
| `src/rtsp_streamer.cpp` → `rtsp_streamer` | C++ | FFmpeg `x11grab` 抓屏推 RTSP |
| `scripts/rtsp_server.py` | Python | 订阅 `~image_topic`，转 OpenCV → FFmpeg 推 RTSP |
| `scripts/rviz_capture.py` | Python | 用 `mss` 抓 RViz 窗口，发布为 ROS 图像（默认 `/rviz/image`） |
| `scripts/start_mediamtx.sh` | Shell | 启动 mediamtx（读 `web/mediamtx.yml`） |
| `scripts/rtsp_with_mediamtx.sh` | Shell | 一键拉起 mediamtx + 推流 |
| `scripts/start_rviz_stream.sh` | Shell | 一键：抓 RViz → 推流 |

### 配置 / 前端
| 文件 | 作用 |
|------|------|
| `config/rtsp_config.yaml` | `image_topic` / `rtsp_port` / `stream_name` / `fps` / `width` / `height` / `bitrate` / 编码 `preset`、`tune` |
| `web/mediamtx.yml` | mediamtx 服务配置（RTSP/HLS/WebRTC 端口与路径） |
| `web/index.html` | 浏览器 WebRTC 播放页 |

> ⚠️ **launch 文件目前不完整**：
> - `launch/rtsp_stream.launch` 引用了 `rtsp_server_simple.py`（仓库中无此文件）；
> - `launch/mapping_with_rtsp.launch` 引用了 `info_display.py`（仓库中无此文件）。
>
> 直接 `roslaunch` 会因找不到脚本而失败。在补齐这两个脚本前，请按下文用 `scripts/` 下的脚本或 `rosrun` 单独运行。

---

## 3. 依赖

```bash
sudo apt-get update && sudo apt-get install ffmpeg
pip3 install opencv-python mss          # rtsp_server.py / rviz_capture.py
# 另需 mediamtx 二进制（流媒体网关）：
#   https://github.com/bluenviron/mediamtx/releases
```

编译：
```bash
cd ~/slam_ws && catkin_make && source devel/setup.bash
```

---

## 4. 使用

### 4.1 ROS 图像 → RTSP（`rtsp_server.py`）
```bash
# 1) 启动 mediamtx 网关
rosrun rtsp_stream start_mediamtx.sh        # 或 ./scripts/start_mediamtx.sh

# 2) 若画面来自 RViz，先抓屏发布为 /rviz/image
rosrun rtsp_stream rviz_capture.py

# 3) 推流（订阅图像话题，推到 mediamtx）
rosrun rtsp_stream rtsp_server.py _image_topic:=/rviz/image
```

### 4.2 屏幕抓取 → RTSP（C++ `rtsp_streamer`）
```bash
rosrun rtsp_stream start_mediamtx.sh
rosrun rtsp_stream rtsp_streamer            # FFmpeg x11grab 抓屏推流
```

### 4.3 观看
```bash
# VLC / ffplay（RTSP）
ffplay -rtsp_transport tcp rtsp://<IP>:8554/mapping

# 浏览器（WebRTC，推荐）——打开 web/index.html，或访问 mediamtx 的 WebRTC 端口
#   http://<IP>:8889/mapping
```

---

## 5. 参数（`config/rtsp_config.yaml` / 节点私有参数）

| 参数 | 默认 | 说明 |
|------|------|------|
| `image_topic` | `/camera/image_raw` | 输入图像话题（`rtsp_server.py`） |
| `rtsp_port` | `8554` | RTSP 端口 |
| `stream_name` | `mapping` | 流名（路径 `rtsp://IP:8554/<stream_name>`） |
| `fps` | `30` | 帧率 |
| `width` / `height` | `1280` / `720` | 分辨率 |
| `bitrate` | `2M` | 码率 |
| `encoder.preset` / `encoder.tune` | `ultrafast` / `zerolatency` | FFmpeg 编码预设/调优 |

> 注意：部分参数（如 `image_topic`）只对 Python `rtsp_server.py` 生效；C++ 屏幕抓取路径不订阅图像话题。

---

## 6. 故障排查

| 现象 | 排查 |
|------|------|
| 没有画面 | `rostopic hz <image_topic>` 确认有图像；屏幕抓取确认 `DISPLAY` 可用 |
| FFmpeg 起不来 | `ffmpeg -version`；`netstat -tuln | grep 8554` 看端口占用 |
| 浏览器放不出来 | 浏览器不支持原生 RTSP，需走 mediamtx 的 WebRTC（`web/index.html`） |
| 延迟高 | 降分辨率/码率，编码用 `ultrafast` + `zerolatency`，用有线网络 |
| `roslaunch` 报找不到脚本 | 见上文：launch 引用的 `rtsp_server_simple.py` / `info_display.py` 尚未提供，改用 `rosrun` |

---

## 许可证

MIT License
