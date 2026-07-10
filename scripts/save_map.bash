#!/bin/bash

###############################
# 保存 SLAM Toolbox 的 2D 地图
###############################

# 地图保存目录
MAP_DIR="${MAP_DIR:-/car_ws/.ros/maps}"

# 地图话题
MAP_TOPIC="${MAP_TOPIC:-/map}"

# 地图名称（默认 map）
MAP_NAME=${1:-map}

echo "===> 保存二维地图：${MAP_NAME}"

# 创建目录
mkdir -p "${MAP_DIR}"

# 保存地图
ros2 run nav2_map_server map_saver_cli \
    -f "${MAP_DIR}/${MAP_NAME}" \
    --ros-args -r map:=${MAP_TOPIC}

if [ $? -eq 0 ]; then
    echo ""
    echo "=============================="
    echo "二维地图保存成功！"
    echo "PGM : ${MAP_DIR}/${MAP_NAME}.pgm"
    echo "YAML: ${MAP_DIR}/${MAP_NAME}.yaml"
    echo "=============================="
else
    echo ""
    echo "二维地图保存失败！"
    echo "请检查："
    echo "  1. slam_toolbox 是否正在发布 ${MAP_TOPIC}"
    echo "  2. nav2_map_server 是否已安装"
    exit 1
fi