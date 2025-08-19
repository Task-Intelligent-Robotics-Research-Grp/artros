cmake_minimum_required(VERSION 3.8)
project(aist_precision_gripper)

## Compile as C++11, supported in ROS Kinetic and newer
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

###########
#  Build  #
###########
ament_auto_add_library(
  ${PROJECT_NAME} SHARED
    src/precision_gripper_controller.cpp
)

rclcpp_components_register_nodes(
  ${PROJECT_NAME}
    PLUGIN     ${PROJECT_NAME}::PrecisionGripperController
    EXECUTABLE ${PROJECT_NAME}_node
)

########################
#  Python executables  #
########################
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/test_client.py
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch config urdf meshes)
