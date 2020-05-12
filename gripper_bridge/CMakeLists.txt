cmake_minimum_required(VERSION 2.8.3)
project(gripper_bridge)

add_compile_options(-std=c++11)

add_definitions(-D__LINUX__)
add_definitions(-D_COUT_)

file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  hirop_msgs
  geometry_msgs
  message_generation
)

find_package(hirop_gripper REQUIRED)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${hirop_gripper_INCLUDE_DIRS}
  include
)

#link_directories(/usr/lib/x86_64-linux-gnu/)

add_executable(gripper_bridge
  src/main.cpp
  src/ros_gripper.cpp
)

add_dependencies(gripper_bridge ${hirop_msgs_EXPORTED_TARGETS}　${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(gripper_bridge
  ${hirop_gripper_LIBRARIES}
  ${catkin_LIBRARIES}
  boost_thread
  yaml-cpp
)
