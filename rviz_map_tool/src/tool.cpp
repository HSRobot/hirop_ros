#include "tool.h"
#include "panel.h"

using namespace hirop_ros;
using namespace rviz;


MapFlagTool* MapFlagTool::_instance = NULL;


MapFlagTool::MapFlagTool(){
    _instance = this;
}

MapFlagTool::~MapFlagTool(){

}

void MapFlagTool::onInitialize(){

    PoseTool::onInitialize();
    setName( "Map Flag" );

    std::cout << "in tool init" << std::endl;

}

void MapFlagTool::onPoseSet(double x, double y, double theta){

    MapFlagManagerPanel *panel = MapFlagManagerPanel::getInstance();

    if(panel){
        panel->getFlag(x, y, theta, scene_manager_);
    }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hirop_ros::MapFlagTool,rviz::Tool)
