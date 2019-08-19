#pragma once

#include <iostream>
#include <map>

#include <rviz/panel.h>
#include <QPushButton>
#include <QLineEdit>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <hirop/datamanager/file_datawriter.h>
#include <hirop/datamanager/posedata.h>

#include "ui_my_panel.h"

using namespace hirop::data_manager;

namespace hirop_ros{

/**
 * 地图标签管理类
 */
class MapFlagManagerPanel : public rviz::Panel{

    Q_OBJECT
public:

    /**
     * @brief MapFlagManagerPanel   构造函数
     * @param parent                由调用者传递给插件的当前窗口的父窗口
     */
    MapFlagManagerPanel(QWidget* parent = NULL);

    ~MapFlagManagerPanel();

    /**
     * @brief load      当rviz加载参数的时候会调用该函数
     * @param config    rviz的配置参数
     */
    virtual void load( const rviz::Config& config );

    /**
     * @brief save      当rviz保存参数的时候会调用该函数
     * @param config    rviz的配置参数
     */
    virtual void save( rviz::Config config ) const;

    /**
     * @brief getFlag   当获取到flag时候的回调函数
     * @param x
     * @param y
     * @param theta
     */
    void getFlag(double x, double y, double theta, Ogre::SceneManager *scene_manager_);

    /**
     * @brief getInstance   获取系统中该panel的实例
     * @return              实例
     */
    static MapFlagManagerPanel* getInstance() {return _instace;}

private:

    /**
     * @brief makeFlagInRviz   在RVIZ的场景上创建一个flag
     */
    void makeFlagInRviz(std::string name, double x, double y);

    /**
     * @brief cleanFlags        清除RVIZ上的所有flag
     */
    void cleanFlags();

    /**
     * @brief saveFlag      通过HIROP DM模块将flag保存起来
     * @param name          Flag的名称
     * @param x
     * @param y
     * @param theat
     */
    void saveFlag(std::string map, std::string name, double x, double y, double theat);

    /**
     * @brief loadFlags     加载指定地图的所有flag
     * @param map           map名称
     */
    void loadFlags(std::string map);

    /**
     * @brief generateUri   生成URI
     * @param name          uri的名称
     * @return              生成的URI
     */
    DataUri generateUri(std::string name);

private Q_SLOTS:
    /**
     * @brief on_loadBtn_clicked    加载地图时候的槽函数
     */
    void on_loadBtn_clicked();

    /**
     * @brief on_addBtn_clicked     添加flag时候的槽函数
     */
    void on_addBtn_clicked();

    /**
     * @brief on_flagsComboBox_currentIndexChanged  下拉框选择的内容发送变化时调用
     * @param text
     */
    void on_flagsComboBox_currentIndexChanged(const QString &text);

    /**
     * @brief on_delFlagBtn_clicked 删除flag的回调函数
     */
    void on_delFlagBtn_clicked();

private:
    /**
     * @brief instace      实例
     */
    static MapFlagManagerPanel* _instace;

    /**
     * @brief ui        UI文件的实例
     */
    Ui::MyPanelWidget ui;

    /**
     * @brief scene_manager_    rviz中的ogre的场景管理器对象
     */
    Ogre::SceneManager *scene_manager_;

    /**
     * @brief nodes             用于保存所有的flag
     */
    std::map<std::string, Ogre::SceneNode*> nodes;

    /**
     * @brief currentMap        当前选中的地图
     */
    std::string currentMap;

    /**
     * @brief writer        HIROP数据管理模块的读写者对象
     */
    FileDataWriter* _writer;
};

}
