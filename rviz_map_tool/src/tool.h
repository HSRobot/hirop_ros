#pragma once

#include <rviz/default_plugin/tools/pose_tool.h>

namespace hirop_ros{

/**
 * @brief The MapFlagTool class 用于在rviz上对地图进行标记的tool
 */
class MapFlagTool : public rviz::PoseTool{

Q_OBJECT
public:

    /**
     * @brief MapFlagTool   构造函数
     */
    MapFlagTool();

    /**
     * @brief ~MapFlagTool  析构函数
     */
    ~MapFlagTool();

    /**
     * @brief onInitialize  当插件被加载完成后被执行的函数
     */
    virtual void onInitialize();

    /**
     * @brief getInstance   获取实例
     * @return              实例的指针
     */
    static MapFlagTool* getInstance(){
        return _instance;
    }

    /**
     * @brief getSceneManager   获取场景的指针
     * @return
     */
    Ogre::SceneManager* getSceneManager(){return scene_manager_;}

protected:

    /**
     * @brief onPoseSet     当标记工具完成标记后回调函数
     * @param x             坐标x方向的值
     * @param y             坐标y方向的值
     * @param theta         偏航角
     */
    virtual void onPoseSet(double x, double y, double theta);

private:
    /**
     * @brief _instance 实例
     */
    static MapFlagTool* _instance;

};

}
