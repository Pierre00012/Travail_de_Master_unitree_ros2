cmake_minimum_required(VERSION 3.8)
project(unitree_ros2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/obstacle_avoidance.py
  scripts/move_forward.py
  scripts/diag_lidar.py
  scripts/diag_z.py
  scripts/mission_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
