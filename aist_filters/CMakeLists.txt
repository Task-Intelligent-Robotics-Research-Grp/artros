cmake_minimum_required(VERSION 3.14.4)
project(aist_filters)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#######################
#  Find dependencies  #
#######################
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
#  Build  #
###########
ament_auto_add_library(
  butterworth SHARED
    src/butterworth.cpp)

ament_target_dependencies(
  butterworth
    filters
    pluginlib
    aist_utility
)

##########
#  Test  #
##########
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    butterworth_test
      test/test_butterworth.cpp
  )

  target_link_libraries(
    butterworth_test
      butterworth
  )
endif()

#############
#  Install  #
#############
pluginlib_export_plugin_description_file(filters plugins_description.xml)

ament_export_dependencies(
  filters
  pluginlib
  rclcpp
  aist_utility
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
