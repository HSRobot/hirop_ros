#include <iostream>
#include <stdio.h>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "hirop_msgs/listGripper.h"
#include "hirop_msgs/SetGripper.h"
#include "hirop_msgs/connectGripper.h"
#include "hirop_msgs/disConnectGripper.h"
#include "hirop_msgs/openGripper.h"
#include "hirop_msgs/closeGripper.h"
#include "hirop_msgs/setIODout.h"
#include "hirop_msgs/StopGripper.h"
#include "hirop_msgs/moveSeqIndex.h"
#include "hirop_msgs/getForce.h"
#include "hirop/gripper/execute.h"

#define SETGRIPPER  "setGripper"
#define LISTGRIPPER "listGripper"
#define CONNECTGRIPPER "connectGripper"
#define DISCONNECTGRIPPER "disConnectGripper"
#define OPENGRIPPER "openGripper"
#define CLOSEGRIPPER "closeGripper"
#define STOPGRIPPER "stopGripper"

#define GETFORCEIPPER "getForce"
#define MOVESEQGRIPPER "moveSeq"

#define ACTIONSERVER "gripper_controller/follow_joint_trajectory"
#define JOINTSTATESNAME "joint_states"
#define PUBFRAMEID "From real-time state data"

#define SETIODOUT "hsc3SetIODout"

using namespace hirop_gripper;

class GripperService{

    typedef struct Parameters{
        std::string cmdStr;
    }Parameters;
    
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
    bool getForceCB(hirop_msgs::getForce::Request& req, hirop_msgs::getForce::Response& res);
    bool setMoveSeqIndex(hirop_msgs::moveSeqIndex::Request& req, hirop_msgs::moveSeqIndex::Response& res);
private:
    void parseConfig(std::string gripperName);

    bool isGripperPtr();

    bool executeCMD(const char *cmd, char *result);

    void executeAction();

    void exit();

private:
    ros::NodeHandle n_gripper;

    Gripper* gripperPtr;

    boost::thread* acThrdPtr;

    ros::ServiceServer set_gripper;
    ros::ServiceServer list_gripper;
    ros::ServiceServer connect;
    ros::ServiceServer disconnect;
    ros::ServiceServer open;
    ros::ServiceServer close;
    ros::ServiceServer stop;

    ros::ServiceServer getForce;
    ros::ServiceServer moveSeq;

    ros::ServiceClient setIO;

    std::string gripper_config_path_;
    std::string action_config_path_;

    Parameters m_parm;

};
