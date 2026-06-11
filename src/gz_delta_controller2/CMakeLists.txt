cmake_minimum_required(VERSION 3.8)
project(gz_delta_controller2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(gz_delta_controller2 SHARED
  src/DeltaMotorController2.cc
)

target_include_directories(gz_delta_controller2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(gz_delta_controller2
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS gz_delta_controller2
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

ament_package()