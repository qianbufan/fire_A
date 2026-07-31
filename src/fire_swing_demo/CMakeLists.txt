cmake_minimum_required(VERSION 3.8)
project(fire_swing_demo)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/horizontal_disk_sweep.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
