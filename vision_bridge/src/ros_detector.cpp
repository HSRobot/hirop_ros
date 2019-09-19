#include "ros_detctor.h"
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

using namespace ros;

DetectorService::DetectorService(NodeHandle n){
    mNodeHandle = n;
    mDetectorPtr = new Detector();
    mDetectorPtr->setOnStateChangeCallback(this);
}

int DetectorService::start(){

    /**
     *  获取相关参数
     */
    mNodeHandle.param("/vision_bridge/use_depth", _useDepth, false);
    mNodeHandle.param("/vision_bridge/use_color", _useColor, true);
    mNodeHandle.param("/vision_bridge/islazy", _isLazy, false);
    mNodeHandle.param("/vision_bridge/publish_tf", _publish_tf, true);
    mNodeHandle.param("/vision_bridge/publish_tf_rate", _publish_tf_rate, 10);
    mNodeHandle.param("/vision_bridge/rgb_topic", _rgbTopicName, std::string("/kinect2/qhd/image_color"));
    mNodeHandle.param("/vision_bridge/depth_topic", _depthTopicName, std::string("/kinect2/qhd/image_depth_rect"));
    mNodeHandle.param("/vision_bridge/camera_frame", _cameraFrame, std::string("kinect2_rgb_optical_frame"));

    /**
     *  发布服务
     */
    detectionServer = mNodeHandle.advertiseService(SERVER_NAME, &DetectorService::detectionCallback, this);
    listDetectorServer = mNodeHandle.advertiseService(LIST_DETECTOR_SERVER_NAME, \
                                                      &DetectorService::listDetectorCallBack, this);
    listObjectServer = mNodeHandle.advertiseService(LIST_OBJECT_SERVER_NAME, \
                                                    &DetectorService::listObjectCallBack, this);

    /**
     *  订阅深度图
     */
    if(_useDepth)
        depthImgSub = mNodeHandle.subscribe(_depthTopicName, 1, &DetectorService::depthImgCB, this);

    /**
     *  订阅彩色图
     */
    if(_useColor)
        colorImgSub = mNodeHandle.subscribe(_rgbTopicName, 1, &DetectorService::colorImgCB, this);

    if(_publish_tf){
        boost::function0<void> f =  boost::bind(&DetectorService::publishObjectTf,this);
        publishTfThread = new boost::thread(f);
        publishTfThread->timed_join(boost::posix_time::microseconds(1));
    }

    posePub = mNodeHandle.advertise<hirop_msgs::ObjectArray>("object_array", 1);

    image_transport::ImageTransport it(mNodeHandle);
    imgPub = it.advertise("preview_image", 1);

    ROS_INFO("detector service start finish \n");
}


bool DetectorService::detectionCallback(hirop_msgs::detection::Request &req, hirop_msgs::detection::Response &res){

    if(_isLazy){
        color_ptr = NULL;
        depth_ptr = NULL;
    }

    /**
     *  订阅深度图
     */
    if(_useDepth && _isLazy)
        depthImgSub = mNodeHandle.subscribe(_depthTopicName, 1, &DetectorService::depthImgCB, this);

    if(_useColor && _isLazy)
        colorImgSub = mNodeHandle.subscribe(_rgbTopicName, 1, &DetectorService::colorImgCB, this);

    /**
     *  wait for image
     */
    for(int i = 0; i < 20; i ++){
        if( ( _useColor && color_ptr == NULL) || ( _useDepth && depth_ptr == NULL) ){
            ros::Duration(1.0).sleep();
            continue;
        }
        break;
    }

    /**
     *  当20秒后还是没有接受到图像，则认为超时了
     */
    if( ( _useColor && color_ptr == NULL) || ( _useDepth && depth_ptr == NULL) ){
        res.result = -1;
        std::cout << "can't no got img" << std::endl;
        return false;
    }

    ENTITY_TYPE type;
    if(req.detectorType == 1)
        type = PYTHON;
    else
        type = CPP;

    mDetectorPtr->setDetector(req.detectorName, req.objectName, type, req.detectorConfig);

    cv::Mat depth;
    cv::Mat color;

    if( _useDepth )
        depth = depth_ptr->image;

    if( _useColor )
        color = color_ptr->image;

    mDetectorPtr->detectionOnce(depth, color);

    return true;
}

void DetectorService::onDetectDone(std::string detector, int ret, std::vector<pose> p, cv::Mat preImg){

    ROS_INFO("detector was %s", detector.c_str());
    ROS_INFO("ret was %d", ret);
    if(p.empty())
        return ;
    ROS_INFO("x = %lf, y = %lf, z = %lf", p[0].position.x, p[0].position.y, p[0].position.z);

    /**
     *  先清空之前的识别结果
     */
    tfLock.lock();
    msg.objects.clear();
    msg.header.frame_id = _cameraFrame;

    /**
     *  将时间戳设置为0,防止在分布式通讯时出现TF过期的问题
     */
    msg.header.stamp.nsec = ros::Time(0).nsec;
    msg.header.stamp.sec = ros::Time(0).sec;

    for(int i =  0; i < p.size(); i ++){
        hirop_msgs::ObjectInfo objectTmp;

        objectTmp.name = p[i].objectName.c_str();
        objectTmp.detector = detector.c_str();

        objectTmp.pose.header = msg.header;

        objectTmp.pose.pose.orientation.x = p[i].quaternion.x;
        objectTmp.pose.pose.orientation.y = p[i].quaternion.y;
        objectTmp.pose.pose.orientation.z = p[i].quaternion.z;
        objectTmp.pose.pose.orientation.w = p[i].quaternion.w;

        objectTmp.pose.pose.position.x = p[i].position.x;
        objectTmp.pose.pose.position.y = p[i].position.y;
        objectTmp.pose.pose.position.z = p[i].position.z;

        msg.objects.push_back(objectTmp);
    }
    tfLock.unlock();
    posePub.publish(msg);

    if(preImg.size <= 0)
        return;

    sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", preImg).toImageMsg();
    imgPub.publish(imgMsg);

}

void DetectorService::depthImgCB(const sensor_msgs::ImageConstPtr& msg){

    std::cout << "Got depth img" << std::endl;

    try
    {
        depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(_isLazy)
        depthImgSub.shutdown();

}

void DetectorService::colorImgCB(const sensor_msgs::ImageConstPtr& msg){

    std::cout << "Got color img" << std::endl;

    try
    {
        color_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(_isLazy)
        colorImgSub.shutdown();
}

bool DetectorService::listDetectorCallBack(hirop_msgs::listDetector::Request &req,
                                           hirop_msgs::listDetector::Response &res){


    std::vector<std::string> list;
    mDetectorPtr->getDetectorList(list);

    res.detectorList = list;

    return true;
}

bool DetectorService::listObjectCallBack(hirop_msgs::listObject::Request &req,
                                         hirop_msgs::listObject::Response &res){

    std::vector<std::string> list;
    mDetectorPtr->getObjectList(req.detectorName, list);
    res.objectList = list;
    return true;

}

void DetectorService::publishObjectTf(){

    ROS_INFO("start publish object tf thread");

    tf::TransformBroadcaster br;
    tf::Transform transform;

    while(ros::ok()){
        ros::Rate rate(_publish_tf_rate);

        tfLock.lock();

        for(int i = 0; i < msg.objects.size(); i++){
            transform.setOrigin( tf::Vector3(msg.objects[i].pose.pose.position.x, msg.objects[i].pose.pose.position.y, \
                                             msg.objects[i].pose.pose.position.z) );
            transform.setRotation(tf::Quaternion(msg.objects[i].pose.pose.orientation.x, msg.objects[i].pose.pose.orientation.y,\
                                                 msg.objects[i].pose.pose.orientation.z, msg.objects[i].pose.pose.orientation.w));
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _cameraFrame, "object" + i));
        }

        tfLock.unlock();

        rate.sleep();
    }
}
