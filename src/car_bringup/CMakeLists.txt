cmake_minimum_required(VERSION 3.10)
project(car_bringup)

find_package(ament_cmake_auto REQUIRED)


ament_auto_package(
  INSTALL_TO_SHARE
  urdf
  meshes
  launch
  config
  rviz
)
