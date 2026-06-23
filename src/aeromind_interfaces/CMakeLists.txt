cmake_minimum_required(VERSION 3.8)
project(aeromind_interfaces)

# 默认使用 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 消息定义
set(msg_files
  msg/Detection.msg
  msg/ObstacleMap.msg
  msg/Path.msg
  msg/DroneState.msg
)

# 服务定义
set(srv_files
  srv/PlanPath.srv
  srv/Takeoff.srv
  srv/Land.srv
  srv/ArmDrone.srv
  srv/ExecuteTask.srv
)

# 生成消息和服务代码
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
