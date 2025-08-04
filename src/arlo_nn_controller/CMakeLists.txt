cmake_minimum_required(VERSION 3.8)
project(arlo_nn_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_srvs REQUIRED)

#find_package(arlo_gp_controller REQUIRED)
find_package(arlo_interfaces REQUIRED)

set(msg_files
  "msg/Num.msg"
)

set(srv_files
  "srv/EvaluateDriver.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)

include_directories(
  include
)

add_executable(arlo_nn_controller_node
  src/startServerNode.cpp
  src/SimulationController.cpp
  src/SimulationState.cpp
)


ament_target_dependencies(arlo_nn_controller_node 
  rclcpp 
  std_msgs 
  nav_msgs
  sensor_msgs
  geometry_msgs
  rosgraph_msgs
  std_srvs
  #arlo_gp_controller
  arlo_interfaces
)

rosidl_target_interfaces(arlo_nn_controller_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  arlo_nn_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.h"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
