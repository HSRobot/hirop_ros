#include <ros/ros.h>
#include "ros_hsc3api.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hsc3api_bridge");
    ros::NodeHandle n;
    ros::AsyncSpinner as(2);
    as.start();

    Hsc3ApiRos s(n);
    s.start();
    ros::waitForShutdown();
    return 0;
}
