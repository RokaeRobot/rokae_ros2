cmake_minimum_required(VERSION 3.8)
project(rokae_hardware VERSION 0.0.3)

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# -----------------------------------------------------------------------------
# 1. 查找依赖项（ROS 2 包）
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
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
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

find_package(rokae_msgs REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

add_definitions(-DXMATEMODEL_LIB_SUPPORTED)

# -----------------------------------------------------------------------------
# 生成服务接口（步骤5 核心）
# -----------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  srv/SceneService.srv
  DEPENDENCIES std_msgs
)

# -----------------------------------------------------------------------------
# 2. 设置 SDK 路径
# -----------------------------------------------------------------------------
set(SDK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sdk/include)
set(SDK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sdk/lib)

# -----------------------------------------------------------------------------
# 3. 声明可执行文件
# -----------------------------------------------------------------------------

add_library(rokae_hardware_interface SHARED
  src/rokae_hardware_interface.cpp
)

add_executable(connect_test src/connect_test.cpp)

add_executable(movej src/movej_moveit_test.cpp)

add_executable(rokae_driver src/rokae_driver.cpp)

add_executable(movej_client src/movej_client.cpp)
add_executable(rokae_driver7 src/rokae_driver7.cpp)

add_executable(stop_motion_demo src/stop_motion_demo.cpp)

add_executable(scene_service src/scene_service.cpp)

# -----------------------------------------------------------------------------
# 4. 为每个节点链接依赖
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

ament_target_dependencies(rokae_driver
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

ament_target_dependencies(rokae_driver7
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  rokae_msgs
  tf2_geometry_msgs
)

ament_target_dependencies(stop_motion_demo
  rclcpp
  std_msgs
)

ament_target_dependencies(
  scene_service
  rclcpp
  std_msgs
  ament_index_cpp
  rosidl_default_runtime
)

# -----------------------------------------------------------------------------
# 5. 包含目录配置
# -----------------------------------------------------------------------------

foreach(target
    rokae_hardware_interface
    connect_test
    movej
    rokae_driver
    rokae_driver7
    movej_client
    stop_motion_demo
    scene_service
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

# -----------------------------------------------------------------------------
# 7. 链接库配置
# -----------------------------------------------------------------------------
set(COMMON_SDK_LIBS
  xCoreSDK
  xMateModel
  orocos-kdl
  ${Eigen3_LIBRARIES}
)

target_link_libraries(rokae_hardware_interface ${COMMON_SDK_LIBS})
target_link_libraries(connect_test ${COMMON_SDK_LIBS})
target_link_libraries(movej ${COMMON_SDK_LIBS})
target_link_libraries(rokae_driver ${COMMON_SDK_LIBS})
target_link_libraries(rokae_driver7 ${COMMON_SDK_LIBS})
target_link_libraries(stop_motion_demo ${COMMON_SDK_LIBS})

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(scene_service ${cpp_typesupport_target})

# -----------------------------------------------------------------------------
# 8. 安装配置
# -----------------------------------------------------------------------------
install(TARGETS
  rokae_hardware_interface
  connect_test
  movej
  rokae_driver
  rokae_driver7
  movej_client
  stop_motion_demo
  scene_service
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

install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/record_sim.sh
  scripts/replay_sim.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rokae_hardware_interface.xml")
  pluginlib_export_plugin_description_file(hardware_interface rokae_hardware_interface.xml)
  install(FILES rokae_hardware_interface.xml
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
