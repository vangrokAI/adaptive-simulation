cmake_minimum_required(VERSION 3.8)
project(adaptive_simulation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Create executable for contact simulation node
add_executable(surface_contact_simulator_node 
  src/surface_contact_simulator_node.cpp
)

ament_target_dependencies(surface_contact_simulator_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  trajectory_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Create executable for obstacle manager node
add_executable(obstacle_manager_node 
  src/obstacle_manager_node.cpp
)

ament_target_dependencies(obstacle_manager_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Create executable for visualization node
add_executable(simulation_visualization_node 
  src/simulation_visualization_node.cpp
)

ament_target_dependencies(simulation_visualization_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install executables
install(TARGETS
  surface_contact_simulator_node
  obstacle_manager_node
  simulation_visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
