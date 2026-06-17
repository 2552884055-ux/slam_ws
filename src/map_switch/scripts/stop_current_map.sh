#!/bin/bash

nodes=(
    "tf_robot2map"
    "laserMapping"
    "map_publishe"
    "map_server"
    "global_localization"
    "transform_fusion"
)

echo "正在尝试一次性杀死节点: ${nodes[*]}"
rosnode kill "${nodes[@]}"
if [ $? -eq 0 ]; then
    echo "所有指定节点已成功杀死。"
else
    echo "杀死部分或全部节点失败。"
fi
    