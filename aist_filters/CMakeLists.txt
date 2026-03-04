cmake_minimum_required(VERSION 3.14.4)
project(aist_filters)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()

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
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(filters REQUIRED)
find_package(aist_utility REQUIRED)
#ament_auto_find_build_dependencies()

include_directories(
)

###########
#  Build  #
###########
add_library(
  butterworth SHARED
    src/butterworth.cpp)

target_include_directories(
  butterworth PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(
  butterworth PUBLIC
    ${CONTROL_FILTERS_INCLUDE_TARGETS}
)

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
pluginlib_export_plugin_description_file(filters ${PROJECT_NAME}.xml)

install(
  DIRECTORY   include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS butterworth
  EXPORT  export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)
  
ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)
  
ament_export_dependencies(
  filters
  pluginlib
  rclcpp
  aist_utility
)

ament_package()
