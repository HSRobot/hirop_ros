#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <ros/ros.h>
#include <iostream>

#include "hirop_msgs/moveTo.h"
#include "hirop_msgs/setOpMode.h"
#include "hirop_msgs/setVord.h"
#include "hirop_msgs/startJog.h"
#include "hirop_msgs/stopJog.h"
#include "hirop_msgs/setWorkFrame.h"
#include "hirop_msgs/setIODout.h"

#include "CommApi.h"
#include "Hsc3Def.h"
#include "proxy/ProxyMotion.h"
#include "proxy/ProxyIO.h"

using namespace std;
using namespace Hsc3::Comm;
using namespace Hsc3::Proxy;

class Hsc3ApiRos{
    
public:
    Hsc3ApiRos(ros::NodeHandle n);

    ~Hsc3ApiRos();
    
    int start();

    bool moveToCB(hirop_msgs::moveTo::Request& req, hirop_msgs::moveTo::Response& res);
    bool setOpModeCB(hirop_msgs::setOpMode::Request& req, hirop_msgs::setOpMode::Response& res);
    bool setVordCB(hirop_msgs::setVord::Request& req, hirop_msgs::setVord::Response& res);
    bool startJogCB(hirop_msgs::startJog::Request& req, hirop_msgs::startJog::Response& res);
    bool stopJogCB(hirop_msgs::stopJog::Request &req, hirop_msgs::stopJog::Response& res);
    bool setWorkFrameCB(hirop_msgs::setWorkFrame::Request& req, hirop_msgs::setWorkFrame::Response& res);
    bool setIODoutCB(hirop_msgs::setIODout::Request& req, hirop_msgs::setIODout::Response& res);

private:
    CommApi *commapi;
    ProxyMotion *proMo;
    ProxyIO *proIO;

    ros::NodeHandle n_hsc3;

    ros::ServiceServer move_to;
    ros::ServiceServer set_opmode;
    ros::ServiceServer set_vord;
    ros::ServiceServer start_jog;
    ros::ServiceServer stop_jog;
    ros::ServiceServer set_workfarm;
    ros::ServiceServer set_iodout;

    std::string robotIp_;
    int robotPort_;

    const static int8_t gpId = 0;
    HMCErrCode ret;
};
