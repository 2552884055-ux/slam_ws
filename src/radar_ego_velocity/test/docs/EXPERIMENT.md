# 无真雷达的「雷达抗退化」消融验证

目标：在**没有毫米波雷达硬件**的情况下，量化"融合雷达自速度"对 LiDAR 退化场景的改善。

## 原理（为什么这套验证成立）

融合端（`laserMapping.cpp::fuse_radar_velocity`）只消费一个话题 `/radar_velocity`
（`geometry_msgs/TwistStamped`，雷达系自速度），并不关心它从哪来。所以我们：

1. **全视场**跑一遍 FAST-LIO（几何良好 → 轨迹准）当作**参考真值**；
2. 用一个节点把参考轨迹的速度按外参投到雷达系、加噪声，**合成** `/radar_velocity`；
3. 人工把 LiDAR 裁成"管廊"（去掉前后端特征 → 轴向不可观），制造**真实退化**；
4. 对比"退化纯 LiDAR" vs "退化 + 合成雷达"相对参考真值的误差。

合成速度来自全视场那次，而退化 run 里这部分信息对被裁剪的 LiDAR 是**真实缺失**的，
所以雷达确实在补 LiDAR 拿不到的约束——这正是要验证的机制。

> 诚实说明：合成雷达不是真正独立的传感器，它验证的是**融合机制能否修复退化方向的漂移**，
> 不能替代真实雷达的噪声特性。论文里若要更强证据，再叠加路线 B（公开雷达数据集）。

## 一次性准备

```bash
cd ~/haiyang/bishe/slam_ws/slam_ws
catkin_make            # 或 catkin build
source devel/setup.bash
pip3 install evo --user   # 轨迹评估工具
```

设 `BAG=/path/to/你的.bag`（含 `/livox/lidar` + `/livox/imu`，无需雷达）。

---

## Run 1 — 参考真值（全视场，不开雷达）

```bash
# 终端A
roslaunch radar_ego_velocity ablation.launch degrade:=false radar_en:=false rviz:=false
# 终端B：录参考里程计（既当真值，又给Run3当合成源）
rosbag record -O ref_odom.bag /Odometry
# 终端C
rosbag play $BAG
# 播完后 Ctrl-C 终端A、B
```

转成 TUM 真值轨迹：

```bash
evo_traj bag ref_odom.bag /Odometry --save_as_tum
mv Odometry.tum ref.tum
```

## Run 2 — 退化基线（人工退化，纯 LiDAR）

```bash
# 终端A
roslaunch radar_ego_velocity ablation.launch degrade:=true radar_en:=false \
    degrade_mode:=corridor halfwidth:=30 rviz:=true
# 终端B
rosbag record -O deg_odom.bag /Odometry
# 终端C
rosbag play $BAG
```

rviz 里应能看到轨迹**沿某个方向被拉伸/漂移**。导出：

```bash
evo_traj bag deg_odom.bag /Odometry --save_as_tum && mv Odometry.tum deg.tum
```

## Run 3 — 退化 + 合成雷达

```bash
# 终端A
roslaunch radar_ego_velocity ablation.launch degrade:=true radar_en:=true \
    degrade_mode:=corridor halfwidth:=30 rviz:=true
# 终端B
rosbag record -O fix_odom.bag /Odometry
# 终端C：同时播原始bag + 参考里程计(重映射成 /ref_odom 喂给合成节点)
rosbag play $BAG ref_odom.bag /Odometry:=/ref_odom
```

检查合成话题确实在发：`rostopic hz /radar_velocity`。导出：

```bash
evo_traj bag fix_odom.bag /Odometry --save_as_tum && mv Odometry.tum fix.tum
```

---

## 评估（APE / RPE）

三次 run 跑的是同一个 bag，`/Odometry` 时间戳一致，evo 用 `-a` 时间对齐即可：

```bash
# 退化纯LiDAR 的误差
evo_ape tum ref.tum deg.tum -va --plot --save_results deg.zip
# 退化+雷达 的误差
evo_ape tum ref.tum fix.tum -va --plot --save_results fix.zip
# 并排对比
evo_res deg.zip fix.zip --use_filenames --plot
```

**预期结论**：`fix`（退化+雷达）的 APE RMSE 显著低于 `deg`（退化纯 LiDAR），
轨迹图上 `fix` 贴回参考真值、`deg` 沿退化方向飘走。RPE 同理。

也可以叠画三条轨迹直观看：

```bash
evo_traj tum deg.tum fix.tum --ref ref.tum -va --plot
```

---

## 调参与扩展

| 参数 | 位置 | 作用 |
|------|------|------|
| `degrade_mode` | launch | `corridor`(轴向退化) / `fov`(缩视场) / `passthrough`(不裁,核管线) |
| `halfwidth` | launch | 保留扇区半宽，越小退化越狠（漂得越凶） |
| `vel_cov` | launch / mid360.yaml | 合成噪声 & 融合信任度，越小越信雷达 |
| `noise_std` | radar_vel_synth | 单独控制合成噪声标准差(m/s) |

进一步实验思路：
- **退化强度扫描**：固定其他量，扫 `halfwidth = 45/30/20/15`，画"退化程度 vs APE(开/关雷达)"曲线，证明退化越狠雷达收益越大。
- **噪声鲁棒性**：扫 `noise_std`，看雷达在多大测量噪声下仍有正收益。
- **核对管线**：`degrade_mode:=passthrough` 应与 Run1 几乎一致（验证退化节点本身不引入误差）。
