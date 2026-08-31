cmake_minimum_required(VERSION 3.20)
project(aist_visualization)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##################
#  dependencies  #
##################
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Qt6 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

#########################################
#  component: libaist_visualization.so  #
#########################################
ament_auto_add_library(
  ${PROJECT_NAME} SHARED
    src/mesh_generator.cpp
    src/robot_description_provider.cpp
)

rclcpp_components_register_nodes(
  ${PROJECT_NAME}
    ${PROJECT_NAME}::MeshGenerator
    ${PROJECT_NAME}::RobotDescriptionProvider
)

ament_auto_add_library(
  ${PROJECT_NAME}_plugins SHARED
    src/textured_mesh_display.cpp
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

##########
#  test  #
##########
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch config urdf)
