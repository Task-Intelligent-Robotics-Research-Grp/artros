cmake_minimum_required(VERSION 3.8)
project(aist_aruco_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*)
rosidl_generate_interfaces(
  ${PROJECT_NAME}
    ${msg_files}
  DEPENDENCIES
    ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
)

ament_auto_package()
