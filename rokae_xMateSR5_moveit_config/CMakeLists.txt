cmake_minimum_required(VERSION 3.5)
project(rokae_xMateSR5_moveit_config VERSION 1.7.0)
# find_package(ament_cmake REQUIRED)

# ament_package()

# if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
#   install(
#     DIRECTORY launch
#     DESTINATION share/${PROJECT_NAME}
#     PATTERN "setup_assistant.launch" EXCLUDE)
# endif()

# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
