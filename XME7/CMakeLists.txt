cmake_minimum_required(VERSION 3.5)

project(XME7)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # 如果你用 Python 编写 launch 文件


#find_package(roslaunch)

install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}/
)


ament_export_dependencies(rosidl_default_runtime)

ament_package()