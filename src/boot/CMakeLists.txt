# boot - launch files + config + scripts + URDF. Nothing compiled here.
# Per team-leader directive 2026-10-03: MoveIt2 integration lives in this
# package rather than a dedicated rak_description package.
cmake_minimum_required(VERSION 3.8)
project(boot)

find_package(ament_cmake REQUIRED)

# Install everything that ships with the package. Subdirs cover lidar/nav2
# bringup AND the MoveIt2 description + config + launch + scripts.
install(DIRECTORY
  launch config scripts urdf meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()