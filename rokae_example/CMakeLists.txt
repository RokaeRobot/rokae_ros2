cmake_minimum_required(VERSION 3.8)
project(rokae_example VERSION 0.0.3)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# 1. 查找依赖项
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

add_definitions(-DXMATEMODEL_LIB_SUPPORTED)

# -----------------------------------------------------------------------------
# 2. Rokae SDK 路径（复用 rokae_hardware 中的 SDK，不重复拷贝）
# -----------------------------------------------------------------------------
set(SDK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../rokae_hardware/sdk/include)
set(SDK_LIB_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/../rokae_hardware/sdk/lib)

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

set(COMMON_SDK_LIBS
  xCoreSDK
  xMateModel
  orocos-kdl
  ${Eigen3_LIBRARIES}
)

# -----------------------------------------------------------------------------
# 3. 可执行目标
# -----------------------------------------------------------------------------

# --- cartesian_s_line：MoveIt 笛卡尔空间直线轨迹 ---
add_executable(cartesian_s_line src/cartesian_s_line.cpp)
ament_target_dependencies(cartesian_s_line
  rclcpp
  moveit_ros_planning_interface
  controller_manager_msgs
  moveit_msgs
  realtime_tools
  std_msgs
)

# --- follow_joint_position：MoveIt + FollowJointTrajectory 往复轨迹 ---
add_executable(follow_joint_position src/follow_joint_position.cpp)
ament_target_dependencies(follow_joint_position
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  control_msgs
  controller_manager_msgs
  trajectory_msgs
  realtime_tools
  std_msgs
)

# --- follow_cart_position：MoveIt 笛卡尔动态正弦跟随 ---
add_executable(follow_cart_position src/follow_cart_position.cpp)
ament_target_dependencies(follow_cart_position
  rclcpp
  moveit_ros_planning_interface
  controller_manager_msgs
  moveit_msgs
  geometry_msgs
  realtime_tools
  std_msgs
)

# --- joint_position_control：MoveIt + FollowJointTrajectory 余弦轨迹 ---
add_executable(joint_position_control src/joint_position_control.cpp)
ament_target_dependencies(joint_position_control
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  control_msgs
  controller_manager_msgs
  trajectory_msgs
  realtime_tools
  std_msgs
)

# --- joint_s_line：MoveIt 关节空间顺序运动 ---
add_executable(joint_s_line src/joint_s_line.cpp)
ament_target_dependencies(joint_s_line
  rclcpp
  moveit_ros_planning_interface
  controller_manager_msgs
  realtime_tools
  std_msgs
)

# --- servoj_demo：直接调用 Rokae SDK 的实时 ServoJ 示范 ---
add_executable(servoj_demo src/servoj_demo.cpp)
ament_target_dependencies(servoj_demo
  rclcpp
  std_srvs
)
target_include_directories(servoj_demo PRIVATE
  ${SDK_INCLUDE_DIR}
  ${Eigen3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)
target_link_libraries(servoj_demo ${COMMON_SDK_LIBS})

# -----------------------------------------------------------------------------
# 4. 安装
# -----------------------------------------------------------------------------
install(TARGETS
  cartesian_s_line
  follow_joint_position
  follow_cart_position
  joint_position_control
  joint_s_line
  servoj_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
