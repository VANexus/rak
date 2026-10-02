cmake_minimum_required(VERSION 3.8)
project(boot)

find_package(ament_cmake REQUIRED)

# This package is launch files and config only -- nothing to build.
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
