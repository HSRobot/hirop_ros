#include "ros_trainer.h"

TrainService::TrainService(ros::NodeHandle n){
    mNodeHandle = n;
    mTrainer = new Trainer();
}

int TrainService::start(){
    trainServer = mNodeHandle.advertiseService(TRAIN_SERVER_NAME, &TrainService::trainCallback, this);
    ROS_INFO("train service start finish");
}

int TrainService::stop(){

}

bool TrainService::trainCallback(hirop_msgs::train::Request &req, hirop_msgs::train::Response &res){

    if(req.configPath == ""){
        ROS_INFO("the train config file path error");
        return false;
    }

    if(mTrainer->setTrainConfig(req.configPath)){
        ROS_INFO("When setting trainer, something was wrong");
        return false;
    }

    if(!mTrainer->train()){
        return true;
    }

    ROS_INFO("When train object, something was wrong");
    return false;
}
