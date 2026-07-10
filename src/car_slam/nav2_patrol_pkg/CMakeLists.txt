cmake_minimum_required(VERSION 3.8)
project(nav2_patrol_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# 2. 编译插件库 (生成 libnav2_patrol_plugin.so)
add_library(nav2_patrol_plugin SHARED
  src/ArrivalAction.cpp
)
ament_target_dependencies(nav2_patrol_plugin
  rclcpp
  rclcpp_action
  nav2_core
  nav2_msgs
  geometry_msgs
  pluginlib
)
# 注册插件
pluginlib_export_plugin_description_file(nav2_core plugin_description.xml)

# 3. 编译发送节点 (生成可执行文件 patrol_sender)
add_executable(patrol_sender src/patrol_sender.cpp)
ament_target_dependencies(patrol_sender
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
)

# 4. 安装规则
install(TARGETS nav2_patrol_plugin patrol_sender
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
