cmake_minimum_required(VERSION 3.5)
project(rokae_xMateSR4_moveit_config VERSION 0.0.3)
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

