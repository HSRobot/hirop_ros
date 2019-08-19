#include "panel.h"

#include <QHBoxLayout>
#include <QLabel>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include "tool.h"

using namespace hirop_ros;
using namespace rviz;

MapFlagManagerPanel* MapFlagManagerPanel::_instace = NULL;

MapFlagManagerPanel::MapFlagManagerPanel(QWidget* parent) :
    rviz::Panel( parent )
{

    ui.setupUi(this);
    _instace = this;

    _writer = new FileDataWriter();

}

MapFlagManagerPanel::~MapFlagManagerPanel(){
    delete _writer;
}

void MapFlagManagerPanel::getFlag(double x, double y, double theta, Ogre::SceneManager *scene_manager_){

    ui.xEdt->setText(QString::number(x,10,5));
    ui.yEdt->setText(QString::number(y,10,5));
    ui.thetaEdt->setText(QString::number(theta,10,5));

    this->scene_manager_ = scene_manager_;
}

void hirop_ros::MapFlagManagerPanel::on_loadBtn_clicked()
{
    std::string mapName = ui.mapNameEdt->text().toStdString();
    currentMap = mapName;

    /**
     *  加载该地图下的所有flag
     */
    if(mapName != "")
        loadFlags(currentMap);
}

void hirop_ros::MapFlagManagerPanel::on_addBtn_clicked()
{

    std::string name = ui.flagNameEdt->text().toStdString();
    std::string map = ui.mapNameEdt->text().toStdString();

    double x, y, theta;

    x = ui.xEdt->text().toDouble();
    y = ui.yEdt->text().toDouble();
    theta = ui.thetaEdt->text().toDouble();

    if(name != "" && map != ""){
        makeFlagInRviz(name, x, y);
        saveFlag(map, name, x, y, theta);
        ui.flagsComboBox->addItem(QString::fromStdString(name));
    }
}

void MapFlagManagerPanel::makeFlagInRviz(std::string name, double x, double y){

    if(scene_manager_ == NULL){
        MapFlagTool *tool = MapFlagTool::getInstance();
        if(tool == NULL)
            return;
        scene_manager_ = tool->getSceneManager();
    }

    Ogre::Vector3 position;
    position.x = x;
    position.y = y;
    position.z = 0;

    Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity = scene_manager_->createEntity\
            ("package://rviz_plugin_tutorials/media/flag.dae");

    node->attachObject( entity );
    node->setVisible( true );
    node->setPosition( position );

    /**
     *  如果存在重复的flag，则不会进行添加
     */
    nodes.insert(std::make_pair(name, node));
}

void MapFlagManagerPanel::cleanFlags(){

    if(nodes.size() == 0)
        return ;

    std::map<std::string, Ogre::SceneNode*>::iterator it;

    for(it = nodes.begin(); it != nodes.end(); /*it++*/){
        scene_manager_->destroySceneNode(it->second);
        nodes.erase(it++);
    }

}

void MapFlagManagerPanel::saveFlag(std::string map, std::string name, \
                                   double x, double y, double theat){

    /**
     * @brief uri   创建URI 固定的uri MAP/mapname/flags/flagname
     */
    DataUri uri = generateUri(name);

    /**
     *  计算四元数
     */
    Ogre::Quaternion quaternion(Ogre::Radian(theat), Ogre::Vector3::UNIT_Z);

    /**
     * 将ogre的四元数转化为hirop的pose
     */
    hirop::PoseStamped pose;

    pose.frame_id = "map";
    pose.pose.orientation.x = quaternion.x;
    pose.pose.orientation.y = quaternion.y;
    pose.pose.orientation.z = quaternion.z;
    pose.pose.orientation.w = quaternion.w;

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0;

    PoseData data;
    data.pose = pose;

    /**
     *  写入数据
     */
    _writer->saveData(&data, uri);
}

void MapFlagManagerPanel::loadFlags(std::string map){

    DataUri uri;

    uri.addFlag("MAP");
    uri.addFlag(map);
    uri.addFlag("flags");

    std::vector<DataUri> uris;
    _writer->listUri(uri, uris);

    cleanFlags();

    /**
     *  先清空所有flag
     */
    ui.flagsComboBox->clear();


    for(int i = 0; i < uris.size(); i++){
        HData *data = _writer->loadData(uris[i]);

        PoseData *pData = (PoseData*) data;

        if(pData == NULL)
            continue;

        /**
         * @brief makeFlagInRviz    将获取到的flag显示在地图上
         */
        makeFlagInRviz(uris[i].getName(), \
                       pData->pose.pose.position.x, pData->pose.pose.position.y);

        /**
         *  将flag的名称显示在下拉框中
         */
        ui.flagsComboBox->addItem(QString::fromStdString(uris[i].getName()));

        delete data;
    }

}

void hirop_ros::MapFlagManagerPanel::on_flagsComboBox_currentIndexChanged(const QString &text)
{

    DataUri uri = generateUri(text.toStdString());

    PoseData *data = (PoseData*)_writer->loadData(uri);

    if(data ==  NULL)
        return;

    /**
     * 将值显示在控件上
     * @todo    当前返回的theta是不对的
     */
    getFlag(data->pose.pose.position.x, data->pose.pose.position.y, 0.0, scene_manager_);
}

DataUri MapFlagManagerPanel::generateUri(std::string name){

    DataUri uri(name);

    uri.addFlag("MAP");
    uri.addFlag(currentMap);
    uri.addFlag("flags");

    return uri;
}

void hirop_ros::MapFlagManagerPanel::on_delFlagBtn_clicked()
{
    std::string flagName = ui.flagsComboBox->currentText().toStdString();

    DataUri uri = generateUri(flagName);

    _writer->deleteData(uri);

    loadFlags(currentMap);
}

void MapFlagManagerPanel::save(rviz::Config config) const{
    rviz::Panel::save(config);
}

void MapFlagManagerPanel::load(const rviz::Config& config){
    rviz::Panel::load(config);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hirop_ros::MapFlagManagerPanel, rviz::Panel)
