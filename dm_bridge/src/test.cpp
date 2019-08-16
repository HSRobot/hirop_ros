#include <hirop/datamanager/jointdata.h>
#include <vector>
#include <hirop/datamanager/file_datawriter.h>

#include <ros/ros.h>
#include <hirop_msgs/GetData.h>
#include <hirop_msgs/SaveData.h>

using namespace hirop::data_manager;

int main(int argc, char **argv){

    ros::init(argc, argv, "dm_bridge_test");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<hirop_msgs::GetData>("get_data");
    ros::ServiceClient wclient = n.serviceClient<hirop_msgs::SaveData>("save_data");

    hirop_msgs::SaveData srv1;
    srv1.request.uri = "TEST";
    srv1.request.name = "joint1";

    JointData data;
    std::vector<float> joints;
    joints.push_back(0.233);
    joints.push_back(0.234);
    joints.push_back(0.235);
    joints.push_back(0.236);
    data.joints = joints;

    std::string rawData;
    rawData = data.toBuffer();

    hirop_msgs::GetData srv;
    srv.request.uri = "TEST";
    srv.request.name = "joint1";

    int mode;

    while(1){

        std::cout << "1，测试写 2，测试读 3，退出" << std::endl;
        std::cin >> mode;

        switch(mode){
        case 2:

            if (client.call(srv)){

                ROS_INFO("call service success");

                std::string tmp;
                tmp = srv.response.data;
                JointData* data = (JointData*) HData::fromBuffer(tmp);
                std::cout << "data.joints[2] = " << data->joints[2] << std::endl;

            }else{

                ROS_ERROR("call service error");
                return 1;

            }
            break;

        case 1:
            srv1.request.data = rawData;

            if (wclient.call(srv1)){
                ROS_INFO("call write service success");
            }else{
                ROS_ERROR("call service error");
                return 1;
            }
            break;
        case 3:
            return 0;
        }

    }
    return 0;

}
