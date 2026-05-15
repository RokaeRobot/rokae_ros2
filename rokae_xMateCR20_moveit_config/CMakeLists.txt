cmake_minimum_required(VERSION 3.22)
project(rokae_xMateCR20_moveit_config VERSION 0.0.4)
find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}/
)
