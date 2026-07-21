cmake_minimum_required(VERSION 3.8)
project(aeromind_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(aeromind_bringup_tools)

# 将 launch 文件安装到 share 目录
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/aeromind_demo_check
  scripts/aeromind_mission_metrics
  scripts/aeromind_slam_check
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(contest_tools
    COMMAND /usr/bin/python3 -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/test -p test_*.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
endif()

ament_package()
