cmake_minimum_required(VERSION 2.8.3)
project(gripper_bridge)

add_compile_options(-std=c++11)

add_definitions(-D__LINUX__)

file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
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
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${hirop_gripper_INCLUDE_DIRS}
  include
)

add_executable(gripper_bridge
  src/main.cpp
  src/ros_gripper.cpp
)

add_dependencies(gripper_bridge ${hirop_msgs_EXPORTED_TARGETS}　${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(gripper_bridge
  ${hirop_gripper_LIBRARIES}
  ${catkin_LIBRARIES}
)
