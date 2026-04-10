cmake_minimum_required(VERSION 3.8)
project(chassis_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external/serial/include
)

set(SERIAL_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/external/serial/src/serial.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/external/serial/src/impl/unix.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/external/serial/src/impl/win.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/external/serial/src/impl/list_ports/list_ports_linux.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/external/serial/src/impl/list_ports/list_ports_osx.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/external/serial/src/impl/list_ports/list_ports_win.cc
)


add_executable(chassis_node
  src/chassis_node.cpp ${SERIAL_SOURCES}
)

ament_target_dependencies(chassis_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)


install(TARGETS
  chassis_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
