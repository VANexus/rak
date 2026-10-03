# rak_description - config + launch + scripts only, no compiled nodes.
# Spec section 7.1 file layout.
cmake_minimum_required(VERSION 3.8)
project(rak_description)

find_package(ament_cmake REQUIRED)

# Install everything that ships with the package. Subdirs follow the spec
# file layout (urdf/, config/, launch/, scripts/, meshes/).
install(DIRECTORY
  urdf config launch scripts meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ament_lint_auto: not present in this Jazzy layout. Spec 4.3 engineering
# hygiene is enforced via review focus + manual commit-message review.
# (Ruling R2.1)

ament_package()