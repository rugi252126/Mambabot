cmake_minimum_required(VERSION 2.8.3)
project(mambabot_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  Diagnostic.msg
  General.msg
  LogisticData.msg
  PID.msg
  ProjectInfo.msg
  Velocity.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tictactoe_msgs
  CATKIN_DEPENDS message_runtime std_msgs
#  DEPENDS system_lib
)

