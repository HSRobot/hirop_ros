#include "ros_gripper.h"

GripperService::GripperService(ros::NodeHandle n)
{
    this->n_gripper = n;
    gripperPtr = new Gripper();
}

GripperService::~GripperService()
{
    exit();
}

int GripperService::start()
{
    n_gripper.param("/gripper_bridge/gripper_config_path",  gripper_config_path_,
                 std::string("/home/ros/work/hirop/config/GripperConfig.yaml"));
    //n_gripper.param("/gripper_bridge/action_config_path",  action_config_path_,
    //             std::string("/home/ros/work/hirop/config/ActionConfig.yaml"));

    ROS_INFO("gripper_config_path_:%s", gripper_config_path_.c_str());
    //ROS_INFO("action_config_path_:%s", action_config_path_.c_str());

    set_gripper = n_gripper.advertiseService(SETGRIPPER, &GripperService::setGripperCB, this);
    list_gripper = n_gripper.advertiseService(LISTGRIPPER, &GripperService::listGripperCB, this);
    connect = n_gripper.advertiseService(CONNECTGRIPPER, &GripperService::connectCB, this);
    disconnect = n_gripper.advertiseService(DISCONNECTGRIPPER, &GripperService::disconnectCB, this);
    open = n_gripper.advertiseService(OPENGRIPPER, &GripperService::openCB, this);
    close = n_gripper.advertiseService(CLOSEGRIPPER, &GripperService::closeCB, this);
    stop = n_gripper.advertiseService(STOPGRIPPER, &GripperService::stopCB, this);
    return 0;
}

void GripperService::parseConfig(string gripperName)
{
    YAML::Node yamlNode = YAML::LoadFile(action_config_path_);
    if(!yamlNode["parameters"]){
        IErrorPrint("无参数设置");
        return;
    }
    YAML::Node node = yamlNode["parameters"];
    m_parm.cmdStr = node[gripperName]["cmd"].as<std::string>();

#ifdef _COUT_
    std::cout<<"m_parm.cmdStr:"<<m_parm.cmdStr<<std::endl;
#endif

    return ;
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
    try{
        if(!isGripperPtr()){
            res.isSucceeful = false;
            return false;
        }

        //parseConfig(gripperName);

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

        //boost::function0<void > f = boost::bind(&GripperService::executeAction, this);
        //acThrdPtr = new boost::thread(f);

    }catch(ros::Exception& e){
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

void GripperService::exit()
{
    gripperPtr = NULL;
    delete gripperPtr;
}

void GripperService::executeAction()
{
    char* reCmdStr;

    if(!executeCMD(m_parm.cmdStr.data(), reCmdStr)){
        ROS_ERROR("start gripper action failed!!!");
        ROS_INFO("failed message: %s!!",reCmdStr);
    }
    ROS_INFO("start gripper action succeeful!!");
}

bool GripperService::executeCMD(const char *cmd, char *result)
{
    char buf_ps[1024];
    char ps[1024]={0};
    FILE *ptr;
    strcpy(ps, cmd);
    if((ptr=popen(ps, "r"))!=NULL)
    {

        while(fgets(buf_ps, 1024, ptr)!=NULL)
        {
            // 可以通过这行来获取shell命令行中的每一行的输出
            std::cout<<"buf_ps:"<<buf_ps<<std::endl;
            strcat(result, buf_ps);
            if(strlen(result)>1024)
               break;
        }
        pclose(ptr);
        ptr = NULL;
    }
    else
    {
        std::cout<<2222222<<std::endl;
        printf("popen %s error\n", ps);
        return false;
    }
    return true;
}
