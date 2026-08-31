cmake_minimum_required(VERSION 3.8)
project(aist_depth_filter)

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
#  Dependencies  #
##################
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

###########
#  Build  #
###########
ament_auto_add_library(
  ${PROJECT_NAME} SHARED
    src/DepthFilter.cpp
)

target_include_directories(
  ${PROJECT_NAME} PRIVATE
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  ${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
)

rclcpp_components_register_nodes(
  ${PROJECT_NAME}
    ${PROJECT_NAME}::DepthFilter
)

##########
#  Test  #
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

ament_auto_package(INSTALL_TO_SHARE launch config)
