cmake_minimum_required(VERSION 2.8.3)
project(asr_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  audio_common_msgs
  std_msgs
  hirop_msgs
  roscpp
)

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

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
  INCLUDE_DIRS include
#  LIBRARIES asr_bridge
  CATKIN_DEPENDS audio_common_msgs roscpp std_msgs hirop_msgs
)

LINK_DIRECTORIES(/home/fshs/work/hirop/build/install/lib)

include_directories(
  include
  /home/fshs/work/hirop/include
  /home/fshs/work/Linux_awaken_exp/include
  ${catkin_INCLUDE_DIRS}
)


add_executable(asr_bridge src/main.cpp)

target_link_libraries(asr_bridge
  hirop_asr
  asound
  ${catkin_LIBRARIES}
)

