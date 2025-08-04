cmake_minimum_required(VERSION 3.8)
project(arlo_mejorado)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)

install(
  DIRECTORY 
    launch
    worlds
    models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
