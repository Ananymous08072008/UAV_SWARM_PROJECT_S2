cmake_minimum_required(VERSION 3.8)
project(uavx_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config maps models worlds
  DESTINATION share/${PROJECT_NAME})

# Lets Gazebo resolve model://uavx_quad and package://uavx_gazebo/... URIs.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
