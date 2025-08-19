cmake_minimum_required(VERSION 3.8)
project(aist_robotiq_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*)
file(GLOB srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*)
file(GLOB action_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} action/*)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    ${action_files}
  DEPENDENCIES
    ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
)

ament_auto_package()
