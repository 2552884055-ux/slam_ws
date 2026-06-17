# 单个雷达目标(一个检测点 / Object)
float32 azimuth     # 方位角 (rad), 雷达系水平面内, 正前方为0
float32 elevation   # 俯仰角 (rad), 2D雷达填0
float32 range       # 距离 (m)
float32 doppler     # 径向速度 (m/s), 目标相对雷达, 远离为正(由驱动定义,见 doppler_sign)
float32 snr         # 信噪比/强度 (可选, 用于加权或滤波)
uint32  id          # 目标跟踪ID (可选)
