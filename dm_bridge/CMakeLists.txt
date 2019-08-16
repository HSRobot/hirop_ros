cmake_minimum_required(VERSION 2.8.3)
project(dm_bridge)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  hirop_msgs
  roscpp
)

find_package(hirop_datamanager REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs hirop_msgs
)

include_directories(
  include
  ${hirop_datamanager_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(dm_bridge src/main.cpp)

add_executable(dm_bridge_test src/test.cpp)

add_dependencies(dm_bridge ${hirop_msgs_EXPORTED_TARGETS})

add_dependencies(dm_bridge_test ${hirop_msgs_EXPORTED_TARGETS})

target_link_libraries(dm_bridge
  ${hirop_datamanager_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(dm_bridge_test
  ${hirop_datamanager_LIBRARIES}
  ${catkin_LIBRARIES}
)
