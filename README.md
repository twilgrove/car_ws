# car_ws Docker

```bash
# 构建镜像
docker compose build

# 运行容器
docker compose up -d

# 编译工作区
colcon build --symlink-install --parallel-workers 4 --cmake-args -DCMAKE_BUILD_TYPE=Release

ros2 launch car_bringup mapping.launch.py
ros2 launch car_bringup navigation.launch.py
ros2 launch chassis_driver joy_car.launch.py

#保存slam_toolbox定位信息
ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph "{filename: /car_ws/.ros/maps/slam_map}"

#保存地图
./save_2d_map.sh map

# 其他
docker exec -it car_ws-car-1 bash     # 进入容器
docker restart car_ws-car-1           # 重启容器
docker buildx prune -a                # 清除缓存
```
