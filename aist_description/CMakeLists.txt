cmake_minimum_required(VERSION 3.8)
project(aist_description)

find_package(ament_cmake_core REQUIRED)

#############
## Install ##
#############
install(
  PROGRAMS
    scripts/convert_stl_to_ply.py
    scripts/generate_urdf_from_meshes.py
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch devices objects parts scenes wrc
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
