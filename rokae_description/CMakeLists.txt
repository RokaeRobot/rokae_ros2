cmake_minimum_required(VERSION 3.5)
project(rokae_description VERSION 0.0.3)
find_package(ament_cmake REQUIRED)

# 安装文件夹：launch meshes urdf config
install(
  DIRECTORY launch meshes urdf config
  DESTINATION share/${PROJECT_NAME}/
)

# ✅ 安装 Python 可执行脚本（你要加的内容）
install(PROGRAMS
  scripts/gui_to_joint_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

# 这个包是纯描述包（无代码），不需要导出依赖
ament_package()



#shangjialu 03/19