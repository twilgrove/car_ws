#!/bin/bash
set -e

source /car_ws/scripts/env.sh

# 日期日志文件夹
export ROS_LOG_DIR=/car_ws/.ros/log/$(date +%Y%m%d-%H%M%S)
mkdir -p "$ROS_LOG_DIR"

exec ros2 launch chassis_driver joy_car.launch.py
#exec sleep infinity
