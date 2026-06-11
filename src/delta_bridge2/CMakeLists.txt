cmake_minimum_required(VERSION 3.8)
project(delta_bridge2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# =====================================================
# Main bridge:
# ROS joint command -> Gazebo joint command
# =====================================================
add_executable(delta_bridge2_node
  src/delta_bridge2_node.cpp
)

ament_target_dependencies(delta_bridge2_node
  rclcpp
  std_msgs
)

target_link_libraries(delta_bridge2_node
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

# =====================================================
# Feedback bridge:
# Gazebo feedback Double topics -> ROS Float64MultiArray
# =====================================================
add_executable(feedback_bridge_node
  src/feedback_bridge_node.cpp
)

ament_target_dependencies(feedback_bridge_node
  rclcpp
  std_msgs
)

target_link_libraries(feedback_bridge_node
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

# =====================================================
# Install executables
# =====================================================
install(TARGETS
  delta_bridge2_node
  feedback_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()