#include <iostream>

#include <ros/ros.h>

#include "hirop_msgs/listGripper.h"
#include "hirop_msgs/SetGripper.h"
#include "hirop_msgs/connectGripper.h"
#include "hirop_msgs/disConnectGripper.h"
#include "hirop_msgs/openGripper.h"
#include "hirop_msgs/closeGripper.h"
#include "hirop_msgs/setIODout.h"
#include "hirop_msgs/StopGripper.h"

#include "hirop/gripper/execute.h"

#define SETGRIPPER  "setGripper"
#define LISTGRIPPER "listGripper"
#define CONNECTGRIPPER "connectGripper"
#define DISCONNECTGRIPPER "disConnectGripper"
#define OPENGRIPPER "openGripper"
#define CLOSEGRIPPER "closeGripper"
#define STOPGRIPPER "stopGripper"

#define SETIODOUT "hsc3SetIODout"

using namespace hirop_gripper;

class GripperService{
    
public:
    GripperService(ros::NodeHandle n);

    ~GripperService();
    
    int start();

    bool listGripperCB(hirop_msgs::listGripper::Request& req, hirop_msgs::listGripper::Response& res);
    bool setGripperCB(hirop_msgs::SetGripper::Request& req, hirop_msgs::SetGripper::Response& res);
    bool connectCB(hirop_msgs::connectGripper::Request& req, hirop_msgs::connectGripper::Response& res);
    bool disconnectCB(hirop_msgs::disConnectGripper::Request& req, hirop_msgs::disConnectGripper::Response& res);
    bool openCB(hirop_msgs::openGripper::Request& req, hirop_msgs::openGripper::Response& res);
    bool closeCB(hirop_msgs::closeGripper::Request& req, hirop_msgs::closeGripper::Response& res);
    bool stopCB(hirop_msgs::StopGripper::Request& req, hirop_msgs::StopGripper::Response& res);

private:
    bool isGripperPtr();

private:
    ros::NodeHandle n_gripper;

    Gripper *gripperPtr;

    ros::ServiceServer set_gripper;
    ros::ServiceServer list_gripper;
    ros::ServiceServer connect;
    ros::ServiceServer disconnect;
    ros::ServiceServer open;
    ros::ServiceServer close;
    ros::ServiceServer stop;

    ros::ServiceClient setIO;

    std::string gripper_config_path_;
};
