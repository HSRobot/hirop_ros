cmake_minimum_required(VERSION 2.8.3)
project(asr_bridge)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  hirop_msgs
  roscpp
)

#find_package(hirop_asr REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs hirop_msgs
)

include_directories(
  include
#  ${hirop_asr_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#add_executable(asr_bridge src/main.cpp)

#add_dependencies(asr_bridge ${hirop_msgs_EXPORTED_TARGETS})

#target_link_libraries(asr_bridge
#  asound
#  ${hirop_asr_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

