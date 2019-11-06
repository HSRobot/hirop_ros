#include "ros_gripper.h"

GripperService::GripperService(ros::NodeHandle n)
{
    this->n_gripper = n;
    gripperPtr = new Gripper();
}

GripperService::~GripperService()
{
    gripperPtr = NULL;
    delete gripperPtr;
}

int GripperService::start()
{

    n_gripper.param("/gripper_bridge/generator_config_path",  gripper_config_path_,
                 std::string("/home/ros/work/hirop/config/GripperConfig.yaml"));

    ROS_INFO("generator_config_path_:%s", gripper_config_path_.c_str());

    set_gripper = n_gripper.advertiseService(SETGRIPPER, &GripperService::setGripperCB, this);
    list_gripper = n_gripper.advertiseService(LISTGRIPPER, &GripperService::listGripperCB, this);
    connect = n_gripper.advertiseService(CONNECTGRIPPER, &GripperService::connectCB, this);
    disconnect = n_gripper.advertiseService(DISCONNECTGRIPPER, &GripperService::disconnectCB, this);
    open = n_gripper.advertiseService(OPENGRIPPER, &GripperService::openCB, this);
    close = n_gripper.advertiseService(CLOSEGRIPPER, &GripperService::closeCB, this);
    stop = n_gripper.advertiseService(STOPGRIPPER, &GripperService::stopCB, this);

    return 0;
}

bool GripperService::isGripperPtr()
{
    if(gripperPtr == NULL){
        ROS_ERROR("gripperPtr is NULL");
        return  false;
    }
    return true;
}

bool GripperService::setGripperCB(hirop_msgs::SetGripper::Request &req, hirop_msgs::SetGripper::Response &res)
{
    std::string str("Airpowere");
    std::string gripperName = req.gripperName;
    const char* showStr;

    if(!isGripperPtr()){
        res.isSucceeful = false;
        return false;
    }

    showStr = strstr(gripperName.c_str(),str.c_str());//返回指向第一次出现r位置的指针，如果没找到则返回NULL
    if(showStr != NULL){
        setIO = n_gripper.serviceClient<hirop_msgs::setIODout>(SETIODOUT);
        if(!setIO.waitForExistence(ros::Duration(5))){
            res.isSucceeful = false;
            return false;
        }
    }

    if(gripperPtr->setGripper(gripperName, "0" , gripper_config_path_) != 0){
        res.isSucceeful = false;
        return false;
    }

    res.isSucceeful = true;
    return true;
}

bool GripperService::listGripperCB(hirop_msgs::listGripper::Request &req, hirop_msgs::listGripper::Response &res)
{
    std::vector<std::string> glist;

    if(!isGripperPtr()){
        return false;
    }

    if(gripperPtr->getGripperList(glist) != 0){
        return false;
    }

    res.gripperList = glist;

    return true;
}

bool GripperService::connectCB(hirop_msgs::connectGripper::Request &req, hirop_msgs::connectGripper::Response &res)
{
    if(!isGripperPtr()){
        res.isConnected = false;
        return false;
    }

    if(gripperPtr->connectGripper() != 0){
        res.isConnected = false;
        return false;
    }

    res.isConnected = true;
    return true;
}

bool GripperService::disconnectCB(hirop_msgs::disConnectGripper::Request &req, hirop_msgs::disConnectGripper::Response &res)
{
    if(!isGripperPtr()){
        res.isDisConnected = false;
        return false;
    }

    if(gripperPtr->disConnectGripper() != 0){
        res.isDisConnected = false;
        return false;
    }

    res.isDisConnected = true;
    return true;
}

bool GripperService::openCB(hirop_msgs::openGripper::Request &req, hirop_msgs::openGripper::Response &res)
{
    if(!isGripperPtr()){
        res.isOpen = false;
        return false;
    }

    if(gripperPtr->openGripper() != 0){
        res.isOpen = false;
        return false;
    }

    res.isOpen = true;
    return true;
}

bool GripperService::closeCB(hirop_msgs::closeGripper::Request &req, hirop_msgs::closeGripper::Response &res)
{
    if(!isGripperPtr()){
        res.isClose = false;
        return false;
    }

    if(gripperPtr->closeGripper() != 0){
        res.isClose = false;
        return false;
    }

    res.isClose = true;
    return true;
}

bool GripperService::stopCB(hirop_msgs::StopGripper::Request &req, hirop_msgs::StopGripper::Response &res)
{
    if(!isGripperPtr()){
        res.reuslt = false;
        return false;
    }

    if(gripperPtr->stopGripprt() != 0){
        res.reuslt = false;
        return false;
    }

    res.reuslt = true;
    return true;

}
