cmake_minimum_required(VERSION 3.8)
project(hardware)

# --- Find dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# --- Include directories ---
include_directories(
  include
)

# --- Create library ---
add_library(ros2_control_ers6 SHARED
  src/ers6_system.cpp
  src/comms.cpp
  src/actuator.cpp
  src/hardware_components.cpp
#  src/mock_hardware_interface.cpp
)

ament_target_dependencies(ros2_control_ers6
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# --- Plugin export ---
pluginlib_export_plugin_description_file(hardware_interface hardware/ers6_system_plugin.xml)

# --- Install library ---
install(TARGETS ros2_control_ers6
  EXPORT export_ros2_control_ers6
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --- Install headers (if any) ---
install(DIRECTORY include/
  DESTINATION include/
)

# --- Install plugin xml ---
install(FILES hardware/ers6_system_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
