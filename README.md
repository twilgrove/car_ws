# 项目文件架构
```text
.
├── map_tool.html          # 地图可视化/编辑工具（html）
├── save_map.bash          # 一键保存地图脚本
└── src/                   # 所有ROS2功能包源码目录
    ├── car_bringup/       # 【启动总入口】机器人启动、配置、launch文件统一管理
    ├── car_map/           # 【感知与建图模块】激光雷达、SLAM建图相关功能包集合
    │   ├── chassis_driver/          # 底盘驱动、遥控器（joy）控制，发布里程计状态
    │   ├── fast_lio/                # 激光SLAM建图核心（Fast-LIO算法），实现边走边建图
    │   ├── icp_registration/        # 点云配准、位姿优化，提升定位精度
    │   ├── linefit_ground_segementation_ros2  # 地面点云分割，滤除地面干扰点
    │   └── livox_ros_driver2/       # Livox/MID360激光雷达官方驱动，读取雷达点云数据
    └── car_nav/           # 【导航与路径规划模块】基于Nav2框架的相关功能包
        ├── costmap_converter/       # 代价地图转几何障碍物（TEB局部规划器必需依赖）
        ├── nav2_patrol_pkg/         # 自主巡检核心插件，实现巡检路径与任务管理
        └── teb_local_planner/       # TEB局部路径规划器，提供平滑轨迹与动态避障
```