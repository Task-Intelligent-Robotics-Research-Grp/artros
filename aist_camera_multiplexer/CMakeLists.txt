cmake_minimum_required(VERSION 3.20)
project(aist_camera_multiplexer)

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

##############################################
#  component: libaist_camera_multiplexer.so  #
##############################################
ament_auto_add_library(
  ${PROJECT_NAME} SHARED
    src/Multiplexer.cpp
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
    PLUGIN     ${PROJECT_NAME}::Multiplexer
    EXECUTABLE ${PROJECT_NAME}_node
)

############################################
#  install python library and executables  #
############################################
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/select_camera.py
    scripts/select_realsense.py
  DESTINATION
    lib/${PROJECT_NAME}
)

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

ament_auto_package(INSTALL_TO_SHARE launch config)
