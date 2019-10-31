cmake_minimum_required(VERSION 2.8.3)
project(vision_bridge)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  hirop_msgs
  tf
  image_transport
)

#find_package(hirop_vision REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp cv_bridge tf
)

add_definitions(-D__LINUX__)

include_directories(
  include
#  ${hirop_vision_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#add_executable(vision_bridge src/main.cpp src/ros_detector.cpp src/ros_trainer.cpp)

## 当工程以来与工程内自身的消息或服务时，需添加以下内容
#add_dependencies(vision_bridge  ${hirop_msgs_EXPORTED_TARGETS})

#target_link_libraries(vision_bridge
#   ${hirop_vision_LIBRARIES}
#   ${catkin_LIBRARIES}
# )
