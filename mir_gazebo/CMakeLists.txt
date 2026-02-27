cmake_minimum_required(VERSION 3.8)
project(mir_gazebo)

find_package(ament_cmake REQUIRED)
# find_package(catkin REQUIRED COMPONENTS
#   roslaunch
# )

###################################
## catkin specific configuration ##
# ###################################
# catkin_package()

#############
## Install ##
#############

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  config
  launch
  maps
  sdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

ament_package()