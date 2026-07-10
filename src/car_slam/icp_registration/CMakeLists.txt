cmake_minimum_required(VERSION 3.8)
project(icp_registration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 查找依赖
find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED) 

# 自动查找 package.xml 里声明的所有依赖 (包括 std_msgs, sensor_msgs, tf2 等)
ament_auto_find_build_dependencies()

# PCL 配置 (PCL 不是标准的 ROS 包，所以要单独处理)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 2. 创建库
ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

# 添加头文件路径
target_include_directories(${PROJECT_NAME} PUBLIC include)

# 手动链接 PCL (ament_auto 不管这个)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

# 3. 注册组件节点 (生成可执行文件)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN icp::IcpNode
  EXECUTABLE ${PROJECT_NAME}_node
)

# =========================================================
#  安装规则
# =========================================================

# A. 安装可执行文件和库
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# B. 安装 Config 和 Launch 文件夹
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# C. 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()