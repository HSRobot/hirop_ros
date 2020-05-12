/**
*   Copyright (C) 2018 华数机器人
*
*   @file       CommDef.h
*   @brief      华数III型控制器通信相关定义
*   @details    
*
*   @author     
*   @date       2018/06/15
*   @version    
*   
*/

#pragma once

#include <stdint.h>
#include <string>
#include <map>
#include <vector>

namespace Hsc3 {
namespace Comm {

/********************************* 枚举定义 *********************************/

/**
*   @brief      命令优先级枚举
*/
enum CmdPriority
{
    PRIORITY_MIN = 0,
    PRIORITY_HIGHEST,   ///< 极高
    PRIORITY_HIGH,      ///< 高
    PRIORITY_NORMAL,    ///< 普通
    PRIORITY_LOW,       ///< 低
    PRIORITY_LOWEST,    ///< 极低
    PRIORITY_MAX,
};

/********************************* 数据类型定义 *********************************/

/**
*   @brief      命令列表
*/
typedef std::vector<std::string> HMCCmd;

/**
*   @brief      错误码（参考ErrDef.h的定义）
*   @see        ErrDef.h
*/
typedef uint64_t HMCErrCode;

/**
*   @brief      byte
*/
typedef unsigned char byte;

/**
*   @brief      执行命令返回值（包括错误码和返回信息）
*/
typedef std::pair<HMCErrCode, std::string> HMCResultUnit;

/**
*   @brief      执行命令返回值列表
*/
typedef std::vector<HMCResultUnit> HMCResult;


/********************************* 结构体定义 *********************************/

/**
*   @brief      文件信息
*/
struct MFileInfo
{
    std::string strName;        ///< 文件名称
    unsigned long ulSize;       ///< 大小
    unsigned char ucType;       ///< 类型，<em>0</em> - 文件，<em>1</em> - 目录
    std::string strModifyTime;  ///< 修改时间
};

/**
*   @brief      控制器信息
*/
struct Hsc3Info
{
    std::string strIp;      ///< IP
    std::string strName;    ///< 名字
    std::string strSn;      ///< SN（唯一识别码）
};

}
}