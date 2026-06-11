cmake_minimum_required(VERSION 3.8)
project(descripe)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes models textures urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS close_loops.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
