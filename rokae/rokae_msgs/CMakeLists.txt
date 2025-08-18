cmake_minimum_required(VERSION 3.5)
project(rokae_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(message_generation REQUIRED)
find_package(builtin_interfaces REQUIRED)


set(msg_files
  msg/ExternalForce.msg
  msg/JointPosVel.msg
  msg/RobotMode.msg
  msg/RobotState.msg
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    #message_generation
    builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()