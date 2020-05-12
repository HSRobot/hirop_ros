#include <hirop/datamanager/jointdata.h>
#include <vector>
#include <hirop/datamanager/file_datawriter.h>

#include <ros/ros.h>
#include <hirop_msgs/GetData.h>
#include <hirop_msgs/SaveData.h>

using namespace hirop::data_manager;

class DMBridge{

public:
    DMBridge(){
        getDataSrv = _n.advertiseService("get_data", &DMBridge::getCallback, this);
        saveDataSrv = _n.advertiseService("save_data", &DMBridge::saveCallback, this);
    }

    bool getCallback(hirop_msgs::GetData::Request &req, hirop_msgs::GetData::Response &res){

        DataUri uri(req.name);

        uri.setUriFromStr(req.uri);

        std::string rawData = writer.loadRawData(uri);
        res.data =  rawData;

        return true;
    }

    bool saveCallback(hirop_msgs::SaveData::Request &req, hirop_msgs::SaveData::Response &res){

        DataUri uri(req.name);

        uri.setUriFromStr(req.uri);

        std::string rawData = req.data;

        writer.saveRawData(rawData, uri);

        return true;
    }

private:
    ros::ServiceServer getDataSrv;
    ros::ServiceServer saveDataSrv;
    ros::NodeHandle _n;
    FileDataWriter writer;

};

int main(int argc, char** argv){

    ros::init(argc, argv, "dm_bridge");

    DMBridge bridge;

    ros::spin();

}
