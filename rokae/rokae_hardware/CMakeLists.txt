cmake_minimum_required(VERSION 3.5)
project(rokae_hardware)

# 使用 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------------
# 1. 查找依赖项（用ROS 2方式）
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rokae_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(controller_manager_msgs REQUIRED)



set(SDK_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/sdk/lib")

# 2.添加硬件接口库
add_library(rokae_hardware_interface SHARED
  src/rokae_hardware_interface.cpp
)

#3.添加头文件路径
target_include_directories(rokae_hardware_interface PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk/include
  #${CMAKE_CURRENT_SOURCE_DIR}/sdk/include/rokae/new_sdk
  ${EIGEN3_INCLUDE_DIR}
)

# 4. 添加静态库（ROKAE SDK）：将SDK 路径导入静态库
add_library(xCoreSDK STATIC IMPORTED)
set_target_properties(xCoreSDK PROPERTIES
    #IMPORTED_LOCATION "${SDK_LIB_DIR}/libxCoreSDK.a"
    IMPORTED_LOCATION "${SDK_LIB_DIR}/libxCoreSDK_static.a"
)

add_library(xMateModel STATIC IMPORTED)
set_target_properties(xMateModel PROPERTIES
    IMPORTED_LOCATION "${SDK_LIB_DIR}/libxMateModel.a"  POSITION_INDEPENDENT_CODE ON
)

add_library(xMateSDK_so STATIC IMPORTED)
set_target_properties(xMateSDK_so PROPERTIES
    # IMPORTED_LOCATION "${SDK_LIB_DIR}/libxCoreSDK.so.0.3.3"  POSITION_INDEPENDENT_CODE ON
    IMPORTED_LOCATION "${SDK_LIB_DIR}/libxCoreSDK.so.0.6.0"  POSITION_INDEPENDENT_CODE ON
)

target_link_libraries(rokae_hardware_interface
  xCoreSDK
  #xMateModel  # 如果需要
  xMateSDK_so
)


# #添加头文件路径
# target_include_directories(rokae_hardware PRIVATE
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
#   ${CMAKE_CURRENT_SOURCE_DIR}/sdk/include
#   #${EIGEN3_INCLUDE_DIR}
# )

# # 5. 链接依赖库
# target_link_libraries(rokae_hardware
#   rokae_hardware_interface
#   xCoreSDK
#   xMateModel
# )

#5.对rokae_hardware_interface工程声明依赖
ament_target_dependencies(rokae_hardware_interface
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  rokae_msgs
  controller_manager
  hardware_interface
  pluginlib
  orocos_kdl
  realtime_tools
  Eigen3
)

# 6. 安装节点与头文件
install(TARGETS
  rokae_hardware_interface
  DESTINATION lib/${PROJECT_NAME}
  # rokae_hardware
  # ARCHIVE DESTINATION lib
  # LIBRARY DESTINATION lib
  # RUNTIME DESTINATION bin
)

add_executable(connect src/connect.cpp)

ament_target_dependencies(connect 
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  rokae_msgs
  controller_manager
  hardware_interface
  pluginlib
  realtime_tools
  Eigen3
)

#3.添加头文件路径
target_include_directories(connect PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk/include
  #${CMAKE_CURRENT_SOURCE_DIR}/sdk/include/rokae/new_sdk
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(connect 
  xCoreSDK
  xMateModel  # 如果需要
  xMateSDK_so
)

install(TARGETS connect 
  DESTINATION lib/${PROJECT_NAME}
  # ARCHIVE DESTINATION lib
  # LIBRARY DESTINATION lib
  # RUNTIME DESTINATION bin
)


add_executable(thread_test src/thread_test.cpp src/rokae_hardware_interface.cpp)   #需要包含src/rokae_hardware_interface.cpp否则无法调用里面接口

ament_target_dependencies(thread_test 
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  rokae_msgs
  controller_manager
  hardware_interface
  pluginlib
  realtime_tools
  Eigen3
)

#3.添加头文件路径
target_include_directories(thread_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk/include
  #${CMAKE_CURRENT_SOURCE_DIR}/sdk/include/rokae/new_sdk
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(thread_test 
  xCoreSDK
  xMateModel  # 如果需要
  xMateSDK_so
)

install(TARGETS thread_test 
  DESTINATION lib/${PROJECT_NAME}
  # ARCHIVE DESTINATION lib
  # LIBRARY DESTINATION lib
  # RUNTIME DESTINATION bin
)


install(DIRECTORY include/
  DESTINATION include/
)

# add_executable(movej src/movej.cpp)
# ament_target_dependencies(movej
#   rclcpp
#   moveit_ros_planning_interface
#   controller_manager_msgs
# )

# install(TARGETS movej
#   DESTINATION lib/${PROJECT_NAME}
# )

# 7.插件导出（用于 controller manager 动态加载）  
# 将 hardware_interface 插件描述文件 xml 注册到插件系统供 controller manager 使用 pluginlib 动态加载实现的接口类
pluginlib_export_plugin_description_file(hardware_interface rokae_hardware_interface.xml)
install(FILES rokae_hardware_interface.xml 
DESTINATION share/${PROJECT_NAME})

#8.
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  rokae_msgs
  controller_manager
  hardware_interface
  pluginlib
  realtime_tools
  Eigen3
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# 9.生成 ament package
ament_package()