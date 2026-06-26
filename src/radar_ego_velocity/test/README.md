# test/ —— 雷达抗退化 仿真测试

按类型分类，新增仿真测试时请沿用此结构：

```
test/
├── scripts/     # 仿真数据生成器 & ROS 节点 (python)
│   ├── sim_corridor.py     # 管廊仿真 rosbag 生成 (合成几何 或 --pcd 真实点云)
│   ├── radar_vel_synth.py  # 由真值合成 /radar_velocity (无需真雷达)
│   └── lidar_degrade.py    # 人工裁剪点云制造退化
├── launch/      # 启动文件
│   └── ablation.launch     # 一键开关消融 (degrade / radar_en / vel_cov ...)
├── docs/        # 文档
│   ├── EXPERIMENT.md       # 验证方法指南 (路线 A/B/C)
│   └── SIM_RESULTS.md      # 仿真结果与结论
└── results/     # 结果图: 每个仿真测试单独建子文件夹
    ├── synthetic_corridor/ # 合成纯退化场景
    │   ├── corridor_result.png # off vs on
    │   └── corridor_viz.png    # 管廊点云可视化
    └── real_tunnel/        # 真实 scans.pcd 隧道场景
        ├── real_corridor.png   # 点云结构分析
        ├── real_sim_viz.png    # 仿真渲染核对
        └── real_result.png     # 三场景总结对比
```

> 约定：`results/` 下**每次仿真测试单独建一个子文件夹**（按场景命名，如
> `synthetic_corridor/`、`real_tunnel/`），不要把图平铺在 `results/` 根目录。

## 快速开始

```bash
catkin_make && source devel/setup.bash
# 1) 合成纯退化管廊
/usr/bin/python3 src/radar_ego_velocity/test/scripts/sim_corridor.py -o /tmp/corridor.bag
# 2) 真实隧道几何 (用 FAST-LIO 地图作几何)
/usr/bin/python3 src/radar_ego_velocity/test/scripts/sim_corridor.py \
    --pcd src/FAST_LIO_GLOBAL/PCD/scans.pcd --x_start 2 --x_end 26 -o /tmp/corridor_real.bag
# 3) 消融 (节点经 catkin 安装, roslaunch 按包名递归找 launch)
roslaunch radar_ego_velocity ablation.launch radar_en:=false rviz:=false &
rosbag play -r 0.4 /tmp/corridor_real.bag
```

详见 [docs/SIM_RESULTS.md](docs/SIM_RESULTS.md) 与 [docs/EXPERIMENT.md](docs/EXPERIMENT.md)。
