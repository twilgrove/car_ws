#!/bin/bash

### ====== 可根据自己情况修改的配置 ======
# fastlio2 /map_save 生成的固定文件
PCD_SRC="/home/wheeltec/test.pcd"

# 3D 点云最终保存目录（你自己定义）
PCD_DIR="/home/wheeltec/pcd_maps"

# 2D 地图保存目录（你自己定义）
MAP_DIR="/home/wheeltec/maps"

# octomap_server 输出的 2D 栅格话题名
MAP_TOPIC="/projected_map"

# 等待 test.pcd 生成的最大时间（秒）
WAIT_TIMEOUT=60

# 轮询间隔（秒）
WAIT_INTERVAL=1
### ======================================

# 地图名字：第一个参数；如果没传，就用 'map'
MAP_NAME=${1:-map}

echo "===> 地图名称：${MAP_NAME}"
echo "===> 3D 源文件：${PCD_SRC}"
echo "===> 3D 目标目录：${PCD_DIR}"
echo "===> 2D 目标目录：${MAP_DIR}"

# 确保目录存在
mkdir -p "${PCD_DIR}"
mkdir -p "${MAP_DIR}"

echo "===> 1) 调用 /map_save 保存 3D 点云..."
ros2 service call /map_save std_srvs/srv/Trigger "{}"

echo "===> 2) 等待 Fast-LIO2 生成 ${PCD_SRC} ..."

elapsed=0
while [ ! -f "${PCD_SRC}" ] && [ ${elapsed} -lt ${WAIT_TIMEOUT} ]; do
  sleep ${WAIT_INTERVAL}
  elapsed=$((elapsed + WAIT_INTERVAL))
  echo "    已等待 ${elapsed} 秒..."
done

if [ ! -f "${PCD_SRC}" ]; then
  echo "!!! 超时 ${WAIT_TIMEOUT} 秒仍未找到 ${PCD_SRC}，请检查 fastlio2 的输出路径和 /map_save 服务。"
  exit 1
fi

echo "    找到文件：${PCD_SRC}"

TARGET_PCD="${PCD_DIR}/${MAP_NAME}.pcd"
cp "${PCD_SRC}" "${TARGET_PCD}"
rm -f "${PCD_SRC}"

echo "    已复制为：${TARGET_PCD}"
echo "    已删除原始：${PCD_SRC}"

echo "===> 3) 使用 map_saver_cli 保存 2D 栅格地图..."
ros2 run nav2_map_server map_saver_cli -f "${MAP_DIR}/${MAP_NAME}" \
  --ros-args -r map:=${MAP_TOPIC}

if [ $? -eq 0 ]; then
  echo "===> 完成："
  echo "    3D: ${TARGET_PCD}"
  echo "    2D: ${MAP_DIR}/${MAP_NAME}.pgm  +  ${MAP_DIR}/${MAP_NAME}.yaml"
else
  echo "!!! 保存 2D 地图失败，请检查 map_saver_cli 是否存在、话题 ${MAP_TOPIC} 是否在发布。"
fi
