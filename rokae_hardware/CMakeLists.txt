cmake_minimum_required(VERSION 3.8)
project(rokae_hardware VERSION 1.7.0)
set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# -----------------------------------------------------------------------------
# 1. 查找依赖项
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
find_package(rokae_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(controller_manager_msgs REQUIRED)

add_definitions(-DXMATEMODEL_LIB_SUPPORTED)

# -----------------------------------------------------------------------------
# 2. SDK 路径
# -----------------------------------------------------------------------------
set(SDK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sdk/include)
set(SDK_LIB_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/sdk/lib)

# -----------------------------------------------------------------------------
# 3. 声明目标
# -----------------------------------------------------------------------------

# 硬件接口插件库
add_library(rokae_hardware_interface SHARED
  src/rokae_hardware_interface.cpp
)

# 连接测试
add_executable(connect_test src/connect_test.cpp)

# MoveIt 关节空间运动测试
add_executable(movej src/movej_moveit_test.cpp)

# 停止运动示范
add_executable(stop_motion_demo src/stop_motion_demo.cpp)

# 驱动节点
add_executable(rokae_driver  src/rokae_driver.cpp)
add_executable(rokae_driver7 src/rokae_driver7.cpp)
add_executable(movej_client  src/movej_client.cpp)

# -----------------------------------------------------------------------------
# 4. ament 依赖
# -----------------------------------------------------------------------------
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

ament_target_dependencies(connect_test
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

ament_target_dependencies(movej
  rclcpp
  moveit_ros_planning_interface
  controller_manager_msgs
  realtime_tools
  std_msgs
)

ament_target_dependencies(stop_motion_demo
  rclcpp
  std_msgs
)

ament_target_dependencies(rokae_driver
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  rokae_msgs
  tf2_geometry_msgs
)

ament_target_dependencies(rokae_driver7
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  rokae_msgs
  tf2_geometry_msgs
)

ament_target_dependencies(movej_client
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  rokae_msgs
  tf2_geometry_msgs
)

# -----------------------------------------------------------------------------
# 5. 包含目录
# -----------------------------------------------------------------------------
foreach(target
    rokae_hardware_interface
    connect_test
    movej
    stop_motion_demo
    rokae_driver
    rokae_driver7
    movej_client
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
# 6. SDK 静态库
# -----------------------------------------------------------------------------
link_directories(${SDK_LIB_DIR})

add_library(xCoreSDK STATIC IMPORTED)
set_target_properties(xCoreSDK PROPERTIES
  IMPORTED_LOCATION "${SDK_LIB_DIR}/libxCoreSDK.a"
)

add_library(xMateModel STATIC IMPORTED)
set_target_properties(xMateModel PROPERTIES
  IMPORTED_LOCATION "${SDK_LIB_DIR}/libxMateModel.a"
  POSITION_INDEPENDENT_CODE ON
)

if(EXISTS "${SDK_LIB_DIR}/libxCoreSDK.so")
  add_library(xCoreSDK_so SHARED IMPORTED)
  set_target_properties(xCoreSDK_so PROPERTIES
    IMPORTED_LOCATION "${SDK_LIB_DIR}/libxCoreSDK.so"
  )
endif()

set(COMMON_SDK_LIBS
  xCoreSDK
  xMateModel
  orocos-kdl
  ${Eigen3_LIBRARIES}
)

# -----------------------------------------------------------------------------
# 7. 链接 SDK 库
# -----------------------------------------------------------------------------
target_link_libraries(rokae_hardware_interface ${COMMON_SDK_LIBS})
target_link_libraries(connect_test             ${COMMON_SDK_LIBS})
target_link_libraries(movej                    ${COMMON_SDK_LIBS})
target_link_libraries(stop_motion_demo         ${COMMON_SDK_LIBS})
target_link_libraries(rokae_driver             ${COMMON_SDK_LIBS})
target_link_libraries(rokae_driver7            ${COMMON_SDK_LIBS})

# -----------------------------------------------------------------------------
# 8. 安装
# -----------------------------------------------------------------------------
install(TARGETS
  rokae_hardware_interface
  connect_test
  movej
  stop_motion_demo
  rokae_driver
  rokae_driver7
  movej_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rokae_hardware_interface.xml")
  pluginlib_export_plugin_description_file(hardware_interface rokae_hardware_interface.xml)
  install(FILES rokae_hardware_interface.xml
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# -----------------------------------------------------------------------------
# 9. ament 包
# -----------------------------------------------------------------------------
ament_package()
