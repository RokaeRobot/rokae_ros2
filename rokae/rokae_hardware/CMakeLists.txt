cmake_minimum_required(VERSION 3.8)
project(rokae_hardware)

# 设置构建类型（可选）
set(CMAKE_BUILD_TYPE Debug)

# 设置编译器警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 使用 C++17
set(CMAKE_CXX_STANDARD 17)

# -----------------------------------------------------------------------------
# 1. 查找依赖项（ROS 2 包）
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 可选：ROKAE 自定义消息包（如果存在）
find_package(rokae_msgs REQUIRED)

# 可选：MoveIt 相关包
find_package(moveit_ros_planning_interface REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

add_definitions(-DXMATEMODEL_LIB_SUPPORTED)
# -----------------------------------------------------------------------------
# 2. 设置 SDK 路径
# -----------------------------------------------------------------------------
# ROKAE SDK 目录（假设 SDK 在当前项目的 sdk 目录下）
set(SDK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sdk/include)
set(SDK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sdk/lib)

# -----------------------------------------------------------------------------
# 3. 声明可执行文件
# -----------------------------------------------------------------------------

# (3.1) 硬件接口库（如果需要，否则可以注释掉）
add_library(rokae_hardware_interface SHARED
  src/rokae_hardware_interface.cpp
)

# (3.2) 连接测试节点
add_executable(connect src/connect.cpp)

# (3.3) MoveJ 测试节点
add_executable(movej src/movej.cpp)

# (3.4) 轨迹监控节点
add_executable(trajectory_monitor_node src/trajectory_monitor.cpp)

# (3.5) 新增：rokae_driver 节点（这是你要添加的）
add_executable(rokae_driver src/rokae_driver.cpp)

# -----------------------------------------------------------------------------
# 4. 为每个节点链接依赖
# -----------------------------------------------------------------------------

# (4.1) 硬件接口库的依赖
ament_target_dependencies(rokae_hardware_interface
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  sensor_msgs
  rokae_msgs
  controller_manager
  hardware_interface
  pluginlib
  realtime_tools
  Eigen3
  orocos_kdl
)

# (4.2) connect 节点的依赖
ament_target_dependencies(connect
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  sensor_msgs
  rokae_msgs
  controller_manager
  hardware_interface
  pluginlib
  realtime_tools
  Eigen3
)

# (4.3) movej 节点的依赖
ament_target_dependencies(movej
  rclcpp
  moveit_ros_planning_interface
  controller_manager_msgs
  realtime_tools
  std_msgs
)

# (4.4) trajectory_monitor_node 的依赖
ament_target_dependencies(trajectory_monitor_node
  rclcpp
  moveit_msgs
  std_msgs
)

# (4.5) rokae_driver_node 的依赖
ament_target_dependencies(rokae_driver
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  # 如果使用 rokae_msgs，则添加
  rokae_msgs
  tf2_geometry_msgs
)

# -----------------------------------------------------------------------------
# 5. 包含目录配置
# -----------------------------------------------------------------------------

# 公共包含目录
set(COMMON_INCLUDES
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${SDK_INCLUDE_DIR}
  ${Eigen3_INCLUDE_DIRS}
)

# 为每个目标添加包含目录
foreach(target 
    rokae_hardware_interface
    connect
    movej
    trajectory_monitor_node
    rokae_driver
)
  target_include_directories(${target}
    PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
      ${SDK_INCLUDE_DIR}
      ${Eigen3_INCLUDE_DIRS}
      ${orocos_kdl_INCLUDE_DIRS}
  )
endforeach()

# -----------------------------------------------------------------------------
# 6. SDK 库配置
# -----------------------------------------------------------------------------

# (6.1) 添加 SDK 库目录到链接路径
link_directories(${SDK_LIB_DIR})

# (6.2) ROKAE SDK 静态库
# 注意：ROKAE SDK 可能有多个库文件
add_library(xCoreSDK STATIC IMPORTED)
set_target_properties(xCoreSDK PROPERTIES
    IMPORTED_LOCATION "${SDK_LIB_DIR}/libxCoreSDK.a"
)

add_library(xMateModel STATIC IMPORTED)
set_target_properties(xMateModel PROPERTIES
    IMPORTED_LOCATION "${SDK_LIB_DIR}/libxMateModel.a"
    POSITION_INDEPENDENT_CODE ON
)

# 如果是动态库，可以这样处理
# 假设 ROKAE 也有一个主要的 .so 文件
if(EXISTS "${SDK_LIB_DIR}/libxCoreSDK.so")
  add_library(xCoreSDK_so SHARED IMPORTED)
  set_target_properties(xCoreSDK_so PROPERTIES
    IMPORTED_LOCATION "${SDK_LIB_DIR}/libxCoreSDK.so"
  )
endif()

# -----------------------------------------------------------------------------
# 7. 链接库配置（为每个节点链接 SDK 库）
# -----------------------------------------------------------------------------

# 公共的 SDK 库列表
set(COMMON_SDK_LIBS
  xCoreSDK
  xMateModel
  # 如果有动态库版本
  # xCoreSDK_so
  orocos-kdl
  ${Eigen3_LIBRARIES}
)

# 为每个目标链接库
target_link_libraries(rokae_hardware_interface ${COMMON_SDK_LIBS})
target_link_libraries(connect ${COMMON_SDK_LIBS})
target_link_libraries(movej ${COMMON_SDK_LIBS})
target_link_libraries(rokae_driver ${COMMON_SDK_LIBS})

# trajectory_monitor_node 可能不需要 SDK 库
# target_link_libraries(trajectory_monitor_node ...)

# -----------------------------------------------------------------------------
# 8. 安装配置
# -----------------------------------------------------------------------------

# 安装可执行文件
install(TARGETS
  rokae_hardware_interface
  connect
  movej
  trajectory_monitor_node
  rokae_driver
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件（如果有）
install(DIRECTORY include/
  DESTINATION include
)

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装插件描述文件（硬件接口需要）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rokae_hardware_interface.xml")
  pluginlib_export_plugin_description_file(hardware_interface rokae_hardware_interface.xml)
  install(FILES rokae_hardware_interface.xml 
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# -----------------------------------------------------------------------------
# 9. 生成 ament 包
# -----------------------------------------------------------------------------
ament_package()