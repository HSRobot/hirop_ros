#include "ros_hsc3api.h"

Hsc3ApiRos::Hsc3ApiRos(ros::NodeHandle n)
{
    n_hsc3 = n;
    commapi = new CommApi();
    proMo = new ProxyMotion(commapi);
    proIO = new ProxyIO(commapi);
    commapi->setAutoConn(false);//关闭自动重连功能，否则连接失败
}

Hsc3ApiRos::~Hsc3ApiRos()
{
    proIO = NULL;
    proMo = NULL;
    commapi = NULL;
    delete commapi;
    delete proMo;
    delete proIO;
}

int Hsc3ApiRos::start()
{
    ret = -1;

    n_hsc3.param("/hsc3api_bridge/hsc3_robotIP", robotIp_, std::string("10.10.56.214"));
    n_hsc3.param("/hsc3api_bridge/hsc3_robotPort", robotPort_, int(23234));

    ROS_INFO("robotIp_: %s",robotIp_.c_str());
    ROS_INFO("robotPort_: %d", robotPort_);

    ret = commapi->connect(robotIp_, (uint16_t)robotPort_);
    if(ret != 0){
        ROS_ERROR("connect hsr_robot faile,return value is %d !",ret);
        return -1;
    }

    start_jog = n_hsc3.advertiseService("hsc3StartJog", &Hsc3ApiRos::startJogCB, this);
    stop_jog = n_hsc3.advertiseService("hsc3StopJog", &Hsc3ApiRos::stopJogCB, this);
    set_vord = n_hsc3.advertiseService("hsc3SetVord", &Hsc3ApiRos::setVordCB, this);
    set_opmode = n_hsc3.advertiseService("hsc3SetOpMode", &Hsc3ApiRos::setOpModeCB, this);
    move_to = n_hsc3.advertiseService("hsc3MoveTo", &Hsc3ApiRos::moveToCB, this);
    set_workfarm = n_hsc3.advertiseService("hsc3SetWorkFrame", &Hsc3ApiRos::setWorkFrameCB, this);
    set_iodout = n_hsc3.advertiseService("hsc3SetIODout", &Hsc3ApiRos::setIODoutCB, this);

    return 0;
}

bool Hsc3ApiRos::hsc3ReConnect()
{
    if(!(commapi->isConnected())){
        ret = commapi->connect(robotIp_, (uint16_t)robotPort_);
        if(ret != 0){
            ROS_ERROR("connect hsr_robot faile,return value is %d !",ret);
        }
        return ret == 0 ? true : false;
    }
    return true;
}


bool Hsc3ApiRos::startJogCB(hirop_msgs::startJog::Request &req,hirop_msgs::startJog::Response &res)
{
    int8_t axid = req.axId;
    DirectType direct;
    if(req.direc)
        direct = POSITIVE;
    else
        direct = NEGATIVE;

    if(!hsc3ReConnect()){
        res.ret = ret;
        return false;
    }

    ret = proMo->startJog(gpId,axid, direct);
    res.ret = ret;
    return ret == 0 ? true : false;
}


bool Hsc3ApiRos::stopJogCB(hirop_msgs::stopJog::Request &req,hirop_msgs::stopJog::Response &res)
{
    if(!hsc3ReConnect()){
        res.ret = ret;
        return false;
    }
    ret = proMo->stopJog(gpId);
    res.ret = ret;
    return ret == 0 ? true : false;
}

bool Hsc3ApiRos::setVordCB(hirop_msgs::setVord::Request &req,hirop_msgs::setVord::Response &res)
{
    int32_t vord = req.vord;
    ret = -1;

    if(!hsc3ReConnect()){
        res.ret = ret;
        return false;
    }

    ret = proMo->setJogVord(vord);
    ret = proMo->setVord(vord);
    res.ret = ret;
    return ret == 0 ? true : false;
}

bool Hsc3ApiRos::setOpModeCB(hirop_msgs::setOpMode::Request &req,hirop_msgs::setOpMode::Response &res)
{
     OpMode mode;

     if(!hsc3ReConnect()){
         res.ret = ret;
         return false;
     }

     switch (req.mode) {
     case 1:
         mode = OP_T1;
         break;
     case 2:
         mode = OP_T2;
         break;
     case 3:
         mode = OP_AUT;
         break;
     case 4:
         mode = OP_EXT;
         break;
     default:
         mode = OP_T1;
         break;
     }
     ret = proMo->setOpMode(mode);
     res.ret = ret;
     return ret == 0 ? true : false;
}

bool Hsc3ApiRos::moveToCB(hirop_msgs::moveTo::Request &req,hirop_msgs::moveTo::Response &res)
{
    GeneralPos gpos;
    bool islinear;
    int32_t conifg;
    ret = -1;

    if(!hsc3ReConnect()){
        res.ret = ret;
        return false;
    }

    ret = proMo->getConfig(gpId,conifg);

    gpos.ufNum = req.gpos.ufNum;
    gpos.utNum = req.gpos.utNum;
    gpos.config = conifg;
    gpos.isJoint = req.gpos.isjoint;
    islinear = req.isLinear;

    gpos.vecPos = req.gpos.verpos.data;

    ret = proMo->moveTo(gpId, gpos, islinear);
    res.ret = ret;
    return ret == 0 ? true : false;
}

bool Hsc3ApiRos::setWorkFrameCB(hirop_msgs::setWorkFrame::Request &req, hirop_msgs::setWorkFrame::Response &res)
{
    FrameType frame;

    if(!hsc3ReConnect()){
        res.ret = ret;
        return false;
    }

    switch (req.frame) {
    case 1:
        frame = FRAME_JOINT;
        break;
    case 2:
        frame = FRAME_WORLD;
        break;
    case 3:
        frame = FRAME_TOOL;
        break;
    case 4:
        frame = FRAME_BASE;
        break;
    default:
        frame = FRAME_JOINT;
        break;
    }
    ret = proMo->setWorkFrame(gpId, frame);
    res.ret = ret;
    return ret == 0 ? true : false;
}

bool Hsc3ApiRos::setIODoutCB(hirop_msgs::setIODout::Request &req, hirop_msgs::setIODout::Response &res)
{
    int32_t protIo = req.portIndex;
    bool vlaue = req.value;

    if(!hsc3ReConnect()){
        res.ret = ret;
        return false;
    }

    ret = proIO->setDout(protIo, vlaue);
    res.ret = ret;
    return  ret == 0 ? true : false;
}
