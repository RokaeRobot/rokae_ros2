cmake_minimum_required(VERSION 3.22)
project(rokae_xMateAR5L_moveit_config VERSION 0.0.4)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})
# ament_python_install_module(${PROJECT_NAME}/launch_common.py)

ament_package()