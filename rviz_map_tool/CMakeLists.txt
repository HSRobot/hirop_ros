cmake_minimum_required(VERSION 2.8.3)
project(rviz_map_tool)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
)

#find_package(hirop_datamanager REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rviz
)

## 开启QT的自动MOC功能，开启后CMake会自动MOC于源文件同级目录下的头文件
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

FILE(GLOB_RECURSE UiFiles "src/*.ui")
add_custom_target(uifiles SOURCES ${UiFiles})

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

include_directories(
  ${PROJECT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
#  ${hirop_datamanager_INCLUDE_DIRS}
)

## moc生成的相关文件也会直接包含进这里
set(SRC_FILES
  src/tool.cpp
  src/panel.cpp
)

set(CMAKE_BUILD_TYPE Debug)

#add_library(${PROJECT_NAME}
#  ${SRC_FILES}
#)

#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
#
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#target_link_libraries(${PROJECT_NAME}
#  ${QT_LIBRARIES}
#  ${catkin_LIBRARIES}
#  ${hirop_datamanager_LIBRARIES}
#)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)
