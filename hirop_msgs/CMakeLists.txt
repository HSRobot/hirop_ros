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
  GeneralPos.msg
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
  listGenerator.srv
  MoveToName.srv
  MoveToPos.srv
  Pick.srv
  PickPlaceStop.srv
  Place.srv
  RemoveObject.srv
  ShowObject.srv 
  listActuator.srv
  SetGenActuator.srv
  moveTo.srv
  setOpMode.srv
  setVord.srv
  startJog.srv
  stopJog.srv
  setIODout.srv
  setWorkFrame.srv
  connectGripper.srv
  disConnectGripper.srv
  openGripper.srv
  closeGripper.srv
  listGripper.srv
  SetGripper.srv
  StopGripper.srv
  UploadScene.srv
  robotConn.srv
  robotError.srv
  getForce.srv
  moveSeqIndex.srv
  
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS  std_msgs
)
