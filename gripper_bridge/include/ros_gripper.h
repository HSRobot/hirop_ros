#include <iostream>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/FibonacciAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

#include <yaml-cpp/yaml.h>

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

#define ACTIONSERVER "gripper_controller/follow_joint_trajectory"
#define PUBFRAMEID "From real-time state data"

#define SETIODOUT "hsc3SetIODout"

using namespace hirop_gripper;

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

class GripperService{

    typedef struct Parameters{

        std::vector<std::string> joint_names;
        std::vector<double> joint_states;
        uint action_points;
        uint action_positions;

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

    void onGoal(const control_msgs::FollowJointTrajectoryGoalPtr& jointState);

    void publishJointState();

private:
    bool isGripperPtr();

    void parseConfig(std::string gripperName);

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

    Server* actionServer;

    ros::Publisher pub;

    control_msgs::FollowJointTrajectoryGoalPtr jointState;

    std::string gripper_config_path_;
    std::string action_config_path_;

    boost::thread* pubThrd;

    std::vector<double> joint_states;
    bool open_gripper;
    bool pubstate;

    Parameters m_parm;

};
