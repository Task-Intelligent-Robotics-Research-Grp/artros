cmake_minimum_required(VERSION 3.8)
project(aist_utility)

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

find_package(PkgConfig)
pkg_check_modules(LIBTIFF REQUIRED libtiff-4)
find_package(OpenCV REQUIRED)

include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

########################################
#  shared library: libaist_utility.so  #
########################################
ament_auto_add_library(
  ${PROJECT_NAME} SHARED
    src/tiff.cpp
    src/ply.cpp
    src/oply/OrderedPly.cpp
    src/rply/rply.c
)

ament_target_dependencies(
  ${PROJECT_NAME}
    LIBTIFF
    OpenCV
)

#############
#  rotconv  #
#############
ament_auto_add_executable(
  rotconv
    src/rotconv.cpp
)

###########################
#  joint_state_extractor  #
###########################
ament_auto_add_library(
  joint_state_extractor SHARED
    src/joint_state_extractor.cpp
)

rclcpp_components_register_node(
  joint_state_extractor
    PLUGIN "${PROJECT_NAME}::JointStateExtractor"
    EXECUTABLE joint_state_extractor_node
)

#################
#  capture_pcd  #
#################
ament_auto_add_library(
  capture_pcd SHARED
    src/capture_pcd.cpp
)

rclcpp_components_register_node(
  capture_pcd
    PLUGIN "${PROJECT_NAME}::PCDCapturer"
    EXECUTABLE capture_pcd_node
)

##########################
#  butterworth_lpf_test  #
##########################
ament_auto_add_library(
  butterworth_lpf_test SHARED
    src/butterworth_lpf_test.cpp
)

rclcpp_components_register_node(
  butterworth_lpf_test
    PLUGIN "${PROJECT_NAME}::ButterworthLPFTest"
    EXECUTABLE butterworth_lpf_test_node
)

##############################
#  spline_extrapolator_test  #
##############################
ament_auto_add_library(
  spline_extrapolator_test SHARED
    src/spline_extrapolator_test.cpp
)

rclcpp_components_register_node(
  spline_extrapolator_test
    PLUGIN "${PROJECT_NAME}::SplineExtrapolatorTest"
    EXECUTABLE spline_extrapolator_test_node
)

#############
#  Install  #
#############
install(
  PROGRAMS    scripts/gensig.py
  DESTINATION lib/${PROJECT_NAME}
)

#############
#  Exports  #
#############
ament_export_dependencies(OpenCV)

############
##  test  ##
############
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

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE launch
)
