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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(lbr_description REQUIRED)
find_package(lbr_hardware REQUIRED)
find_package(lbr_controllers REQUIRED)
find_package(lbr_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# Include directories
include_directories(
  include
)

# C++ Libraries
add_library(
  ft_sensor_interface
  src/ft_twincat_bridge/ft_sensor_interface.cpp
)
ament_target_dependencies(
  ft_sensor_interface
  rclcpp
  geometry_msgs
)

add_library(
  adaptive_controller
  src/adaptive_control/adaptive_controller.cpp
  src/adaptive_control/adaptive_strategy.cpp
)
ament_target_dependencies(
  adaptive_controller
  rclcpp
  controller_interface
  hardware_interface
  geometry_msgs
  lbr_controllers
)

# Plugin libraries
pluginlib_export_plugin_description_file(controller_interface
  config/controller_plugins.xml)

# C++ Executables
add_executable(
  ft_twincat_bridge_node
  src/ft_twincat_bridge/ft_twincat_bridge_node.cpp
  src/ft_twincat_bridge/ads_data_provider.cpp
  src/ft_twincat_bridge/sim_data_provider.cpp
)
ament_target_dependencies(
  ft_twincat_bridge_node
  rclcpp
  geometry_msgs
  std_msgs
  tf2_ros
)
target_link_libraries(ft_twincat_bridge_node
  ft_sensor_interface
)

add_executable(
  surface_contact_test
  src/adaptive_control/surface_contact_test.cpp
)
ament_target_dependencies(
  surface_contact_test
  rclcpp
  geometry_msgs
  lbr_msgs
)
target_link_libraries(surface_contact_test
  adaptive_controller
)

# Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/calibrate_ft_sensor.py
  scripts/visualize_ft_data.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ targets
install(
  TARGETS
    ft_sensor_interface
    adaptive_controller
    ft_twincat_bridge_node
    surface_contact_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(
  DIRECTORY
    config
    include/
    launch
    meshes
    models
    urdf
    worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install resource files and documentation
install(
  FILES
    package.xml
    README.md
    CONTRIBUTING.md
    LICENSE
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # C++ unit tests
  ament_add_gtest(
    test_ft_sensor_interface
    test/unit/test_ft_sensor_interface.cpp
  )
  target_link_libraries(test_ft_sensor_interface
    ft_sensor_interface
  )

  ament_add_gtest(
    test_adaptive_controller
    test/unit/test_adaptive_controller.cpp
  )
  target_link_libraries(test_adaptive_controller
    adaptive_controller
  )

  # Python tests
  ament_add_pytest_test(
    test_ft_data_processing
    test/unit/test_ft_data_processing.py
  )
endif()

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(
  ft_sensor_interface
  adaptive_controller
)
ament_export_dependencies(
  rclcpp
  rclpy
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  controller_interface
  hardware_interface
  pluginlib
  lbr_description
  lbr_hardware
  lbr_controllers
  lbr_msgs
  yaml_cpp_vendor
)

ament_package()
