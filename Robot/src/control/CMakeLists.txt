cmake_minimum_required(VERSION 3.8)
project(control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config control
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
