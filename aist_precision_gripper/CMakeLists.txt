##########################################################################
# Set minimum required version of cmake, project name and compile options
##########################################################################
cmake_minimum_required(VERSION 3.5.0)
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

# find dependencies
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

#############
#  Install  #
#############
install(
  TARGETS  ${PROJECT_NAME}
#  EXPORT   export_${PROJECT_NAME}
  ARCHIVE  DESTINATION lib
  LIBRARY  DESTINATION lib
  RUNTIME  DESTINATION bin
)

install(
  DIRECTORY launch config urdf meshes
  DESTINATION share/${PROJECT_NAME}/
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

ament_auto_package()
