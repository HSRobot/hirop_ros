/**
*   Copyright (C) 2018 华数机器人
*
*   @file       Hsc3Def.h
*   @brief      华数III型控制器业务接口相关定义
*   @details    
*
*   @author     
*   @date       2018/06/16
*   @version    
*   
*/

#pragma once

#include <stdint.h>
#include <list>
#include <vector>
#include <string>


/********************************* 枚举定义 *********************************/

/**
*   @brief      操作模式
*/
enum OpMode
{
    OP_NONE = 0,
    OP_T1,      ///< T1示教模式
    OP_T2,      ///< T2示教模式
    OP_AUT,     ///< 自动操作模式
    OP_EXT,     ///< 外部操作模式
    OP_DRAG,    ///< 拖动示教模式
    OP_MAX,
};

/**
*   @brief      工作坐标系类型
*/
enum FrameType
{
    FRAME_NONE = -1,
    FRAME_JOINT,    ///< 关节坐标系
    FRAME_WORLD,    ///< 世界坐标系
    FRAME_TOOL,     ///< 工具坐标系
    FRAME_BASE,     ///< 用户坐标系
    FRAME_MAX,
};

/**
*   @brief      单轴运动方向
*/
enum DirectType
{
    POSITIVE = 0,   ///< 正向
    NEGATIVE        ///< 反向
};

/**
*   @brief      手动运行模式
*/
enum ManualMode
{
    MANUAL_CONTINUE = 0,    ///< 持续模式
    MANUAL_INCREMENT,       ///< 增量模式
};

/**
*   @brief      手动运行状态
*/
enum ManState
{
    MAN_STATE_WAIT = 0,     ///< <em>0</em> - 等待信号
    MAN_STATE_GET_PARA,     ///< <em>1</em> - 获取参数
    MAN_STATE_JOG,          ///< <em>2</em> - 点动运行
    MAN_STATE_INCH,         ///< <em>3</em> - 寸动运行
    MAN_STATE_LOCATION,     ///< <em>4</em> - 定位运行
    MAN_STATE_ERROR,        ///< <em>5</em> - 错误状态
    MAN_STATE_MAX,
};

/**
*   @brief      自动运行模式
*/
enum AutoMode
{
    AUTO_MODE_CONTINUE = 0,     ///< <em>0</em> - 持续模式
    AUTO_MODE_STEP,             ///< <em>1</em> - 单步模式
    AUTO_MODE_DEBUG,            ///< <em>2</em> - 调试模式
    AUTO_MODE_MAX,
};

/**
*   @brief      自动运行状态
*/
enum AutoState
{
    AUTO_STATE_NONE = 0,
    AUTO_STATE_UNLOAD,      ///< <em>1</em> - 空闲
    AUTO_STATE_READY,       ///< <em>2</em> - 就绪
    AUTO_STATE_RUNNING,     ///< <em>3</em> - 运行
    AUTO_STATE_PAUSED,      ///< <em>4</em> - 暂停
    AUTO_STATE_STOPPED,     ///< <em>5</em> - 停止
    AUTO_STATE_DONE,        ///< <em>6</em> - 完成
    AUTO_STATE_FAULT,       ///< <em>7</em> - 出错
    AUTO_STATE_WAIT,        ///< <em>8</em> - 等待
    AUTO_STATE_MAX,
};

/**
*   @brief      报警等级
*/
enum ErrLevel
{
    ERR_LEVEL_UNKNOWN = 0,
    ERR_LEVEL_MIN = 1,
    ERR_LEVEL_INFO = 1,     ///< <em>1</em> - 信息
    ERR_LEVEL_NOTE,         ///< <em>2</em> - 提示
    ERR_LEVEL_WARNING,      ///< <em>3</em> - 警告
    ERR_LEVEL_ERROR,        ///< <em>4</em> - 错误
    ERR_LEVEL_FATAL,        ///< <em>5</em> - 严重错误
    ERR_LEVEL_MAX,
};

/**
*   @brief      区域类型
*/
enum AreaType
{
    AREA_TYPE_MIN = -1,
    NORMAL_AREA,        ///< <em>0</em> - 普通区
    BLOCK_AREA,         ///< <em>1</em> - 干涉区
    WORK_AREA,          ///< <em>2</em> - 安全区
    SHARE_AREA,         ///< <em>3</em> - 共享区
    AREA_TYPE_MAX,
};

/**
*   @brief      区域形状类型
*/
enum AreaShapeType
{
    AREA_SHAPE_TYPE_MIN = 0,
    AREA_SHAPE_TYPE_BOX,        ///< <em>1</em> - 盒状
    AREA_SHAPE_TYPE_CYLINDER,   ///< <em>2</em> - 柱状
    AREA_SHAPE_TYPE_MAX,
};

/**
*   @brief      共享区处理模式
*/
enum ShareAreaMode
{
    SHARE_AREA_MODE_MIN = -1,
    SHARE_AREA_MODE_IGNORE = 0,     ///< <em>0</em> - 忽略
    SHARE_AREA_MODE_ERR_STOP,       ///< <em>1</em> - 报错停机
    SHARE_AREA_MODE_WRN_STOP,       ///< <em>2</em> - 警告停机
    SHARE_AREA_MODE_MAX,
};


/********************************* 数据类型定义 *********************************/

/**
*   @brief      关节点数据
*   @details    注：仅包含点数据信息
*/
typedef std::vector<double> JntData;

/**
*   @brief      笛卡尔点数据
*   @details    注：仅包含点数据信息
*/
typedef std::vector<double> LocData;

/**
*   @brief      附加轴坐标点数据
*   @details    注：仅包含点数据信息
*/
typedef std::vector<double> AuxData;


/********************************* 结构体定义 *********************************/

/**
*   @brief      点数据最大数量
*/
const int MAX_PNT_CNT = 9;

/**
*   @struct     JntPos
*   @brief      关节点
*/
struct JntPos
{
    std::vector<double> vecPos; ///< 点数据
};

/**
*   @struct     LocPos
*   @brief      笛卡尔点
*/
struct LocPos
{
    int8_t ufNum;       ///< 工件号 <em>-1</em> - 默认；<em>0..15</em>可用工件
    int8_t utNum;       ///< 工具号 <em>-1</em> - 默认；<em>0..15</em>可用工具
    int32_t config;     ///< config 【注意】参考机器人程序点位定义中的CONFIG定义
    std::vector<double> vecPos; ///< 点数据
};

/**
*   @struct     GeneralPos
*   @brief      通用点
*   @details    可表示关节点/笛卡尔点
*/
struct GeneralPos
{
    bool isJoint;       ///< 是否关节点
    int8_t ufNum;       ///< 工件号（1..n）
    int8_t utNum;       ///< 工具号（1..n）
    int32_t config;     ///< config 【注意】参考机器人程序点位定义中的CONFIG定义
    std::vector<double> vecPos; ///< 点数据
};

/**
*   @struct     GroupData
*   @brief      轴组数据
*   @details    包含状态和位置等信息
*/
struct GroupData
{
    bool en;            ///< 使能状态
    OpMode mode;        ///< 操作模式
    FrameType frame;    ///< 坐标系类型
    int32_t vordJog;    ///< 手动运行倍率
    int32_t vordAut;    ///< 自动运行倍率
    double inchLen;     ///< 手动运行增量距离
    JntData dataJnt;    ///< 关节坐标点数据
    AuxData dataAux;    ///< 附加轴坐标点数据
    int8_t ufNum;       ///< 工件号
    int8_t utNum;       ///< 工具号
    int32_t config;     ///< 形态 【注意】参考机器人程序点位定义中的CONFIG定义
    LocData dataLoc;    ///< 笛卡尔坐标点数据
};

/**
*   @struct     GroupConfig
*   @brief      轴组配置
*   @details    包含配置信息
*/
struct GroupConfig
{
    int32_t typeRob;            ///< 当前机器人类型
    std::string strTypeName;    ///< 当前机器人类型名字
    int32_t cntRob;             ///< 内部轴数
    int32_t cntAux;             ///< 附加轴数
    int32_t beginAux;           ///< 附加轴起始轴号
};

/**
*   @struct     ProgInfo
*   @brief      程序信息
*/
struct ProgInfo
{
    std::string cur_prog;   ///< 当前程序名
    int32_t line_no;        ///< 行号
    AutoState state;        ///< 自动运行状态
};

/**
*   @struct     AreaConfigData
*   @brief      区域配置数据
*/
struct AreaConfigData
{
    int32_t ufnum;                  ///< 原点工件号
    std::vector<double> origin;     ///< 原点数据
    AreaShapeType shapeType;        ///< 形状类型
    std::vector<double> shapeSize;  ///< 形状数据
    double offset;                  ///< 伸缩值
    AreaType type;                  ///< 区域类型
    bool outReverse;                ///< 输出反转
    ShareAreaMode mode;             ///< 共享区处理模式
};
