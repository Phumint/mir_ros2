cmake_minimum_required(VERSION 3.8)
project(mir_description)

find_package(ament_cmake REQUIRED)
# find_package(catkin REQUIRED COMPONENTS
#   roslaunch
# )

find_package(urdf REQUIRED)
###################################
## catkin specific configuration ##
###################################
# catkin_package()

#############
## Install ##
#############

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  config
  launch
  meshes
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

# roslaunch_add_file_check(launch)
ament_package()
