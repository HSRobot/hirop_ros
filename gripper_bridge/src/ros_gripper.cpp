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
    n_gripper.param("/gripper_bridge/gripper_config_path",  gripper_config_path_,
                 std::string("/home/ros/work/hirop/config/GripperConfig.yaml"));
    n_gripper.param("/gripper_bridge/action_config_path",  action_config_path_,
                    std::string("/home/ros/catkin_ws/src/hirop_ros/gripper_bridge"
                                "/config/GripperConfig.yaml"));

    ROS_INFO("gripper_config_path_:%s", gripper_config_path_.c_str());
    ROS_INFO("action_config_path_:%s", action_config_path_.c_str());

    set_gripper = n_gripper.advertiseService(SETGRIPPER, &GripperService::setGripperCB, this);
    list_gripper = n_gripper.advertiseService(LISTGRIPPER, &GripperService::listGripperCB, this);
    connect = n_gripper.advertiseService(CONNECTGRIPPER, &GripperService::connectCB, this);
    disconnect = n_gripper.advertiseService(DISCONNECTGRIPPER, &GripperService::disconnectCB, this);
    open = n_gripper.advertiseService(OPENGRIPPER, &GripperService::openCB, this);
    close = n_gripper.advertiseService(CLOSEGRIPPER, &GripperService::closeCB, this);
    stop = n_gripper.advertiseService(STOPGRIPPER, &GripperService::stopCB, this);

    pub = n_gripper.advertise<control_msgs::FollowJointTrajectoryGoal>("joint_state", 5);

    actionServer = new Server(n_gripper, ACTIONSERVER, boost::bind(&GripperService::onGoal, this, jointState), false);

    return 0;
}

void GripperService::parseConfig(std::string gripperName)
{
    YAML::Node yamlNode = YAML::LoadFile(action_config_path_);

    YAML::Node node = yamlNode["parameters"];
    m_parm.joint_names = node[gripperName]["joint_names"].as<std::vector<std::string> >();
    m_parm.joint_states = node[gripperName]["joint_state"].as<std::vector<double> >();
    m_parm.action_points = node[gripperName]["action_points"].as<uint>();
    m_parm.action_positions = node[gripperName]["action_positions"].as<uint>();

    joint_states = m_parm.joint_states;

#ifdef _COUT_
    for(int i = 0; i < m_parm.joint_names.size(); i++){
        std::cout<<"m_parm.joint_names["<<i<<"]:"<< m_parm.joint_names[i]<<std::endl
                  <<"m_parm.joint_states["<<i<<"]:"<<m_parm.joint_states[i]<<std::endl;
    }

    std::cout<<"m_parm.action_points:"<<m_parm.action_points<<std::endl
              <<"m_parm.action_positions:"<<m_parm.action_positions<<std::endl;
#endif
    return;
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

    pubstate = false;
    actionServer->shutdown();

    if(!isGripperPtr()){
        res.isSucceeful = false;
        return false;
    }

    parseConfig(gripperName);

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

    pubstate = true;
    usleep(500000);
    boost::function0<void > f = boost::bind(&GripperService::publishJointState, this);
    pubThrd = new boost::thread(f);

    actionServer->start();

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

void GripperService::onGoal(const control_msgs::FollowJointTrajectoryGoalPtr &jointState)
{
    open_gripper = false;

    joint_states = jointState->trajectory.points[m_parm.action_points].positions;

    if(jointState->trajectory.points[m_parm.action_points].positions[m_parm.action_positions] == 0.0){
        open_gripper = true;
    }

    if(!isGripperPtr()){
        return;
    }

    if(open_gripper){
        if(gripperPtr->openGripper() != 0){
            return;
        }
    }
    else {
        if(gripperPtr->closeGripper() != 0){
            return;
        }
    }

    sleep(500);

    return;
}

void GripperService::publishJointState()
{
    control_msgs::FollowJointTrajectoryGoal msg;
    ros::Rate loop(10);
    while(pubstate){

        msg.trajectory.header.stamp = ros::Time::now();
        msg.trajectory.header.frame_id = PUBFRAMEID;
        msg.trajectory.joint_names = m_parm.joint_names;
        msg.trajectory.points[m_parm.action_points].positions = joint_states;

        pub.publish(msg);
        loop.sleep();
    }
    return;
}
