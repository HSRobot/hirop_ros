#include <ros/ros.h>
#include "ros_pickplace.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pickplace_bridge");
    ros::NodeHandle n;
    ros::AsyncSpinner as(4);
    as.start();

    PickPlaceService s(n);
    s.start();
    ros::waitForShutdown();
    return 0;
}
