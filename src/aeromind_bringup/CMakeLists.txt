cmake_minimum_required(VERSION 3.8)
project(aeromind_bringup)

find_package(ament_cmake REQUIRED)

# 将 launch 文件安装到 share 目录
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
