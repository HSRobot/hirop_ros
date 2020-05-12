#include <ros/ros.h>
#include "ros_gripper.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pickplace_bridge");
    ros::NodeHandle n;
    ros::AsyncSpinner as(4);
    as.start();

    GripperService s(n);
    s.start();
    ros::waitForShutdown();
    return 0;
}
