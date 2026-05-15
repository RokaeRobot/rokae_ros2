cmake_minimum_required(VERSION 3.5)
project(rokae_xMatePro7_moveit_config VERSION 0.0.4)
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

