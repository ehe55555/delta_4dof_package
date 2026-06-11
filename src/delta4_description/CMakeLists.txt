cmake_minimum_required(VERSION 3.8)
project(delta4_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes textures urdf config models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
