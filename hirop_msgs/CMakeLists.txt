cmake_minimum_required(VERSION 2.8.3)
project(hirop_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  ObjectArray.msg
  ObjectInfo.msg
)

add_service_files(
  FILES
  Look.srv
  SavePCL.srv
  LoadPCL.srv
  CleanPCL.srv
  StartListen.srv
  StopListen.srv
  GetData.srv
  SaveData.srv
  detection.srv
  listDetector.srv
  listObject.srv
  train.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS  std_msgs
)
