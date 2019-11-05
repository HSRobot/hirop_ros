#ifndef __ROS_TRAINER_H__
#define __ROS_TRAINER_H__

#include <hirop/vision/trainer.h>
#include <ros/ros.h>
#include <hirop_msgs/train.h>

#define TRAIN_SERVER_NAME "trian"

using namespace hirop_vision;

class TrainService{

public:
    TrainService(ros::NodeHandle n);

    /**
     * @brief start     启动检测服务
     * @return  0 成功 -1 失败
     */
    int start();

    /**
     * @brief stop      停止检测服务
     * @return      0 成功 -1 失败
     */
    int stop();

private:
    bool trainCallback(hirop_msgs::train::Request &req, hirop_msgs::train::Response &res);

private:
    ros::NodeHandle mNodeHandle;

    ros::ServiceServer trainServer;

    Trainer *mTrainer;
};

#endif
