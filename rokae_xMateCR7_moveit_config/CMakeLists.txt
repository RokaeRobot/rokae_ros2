cmake_minimum_required(VERSION 3.5)
project(rokae_xMateCR7_moveit_config VERSION 1.7.0)
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

