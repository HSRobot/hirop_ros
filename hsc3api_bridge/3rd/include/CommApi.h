/**
*   Copyright (C) 2018 华数机器人
*
*   @file       CommApi.h
*   @brief      华数III型二次开发接口 - 通信客户端接口
*   @details    提供了良好封装的CommApi类，适用于华数III型控制器网络通信连接；
*               另外还提供了华数III型控制器扫描用接口。
*
*   @author     
*   @date       2018/06/15
*   @version    
*   
*/

#pragma once
/**
*   @skip DLL_EXPORT
*/
#define _LINUX_
#ifdef _LINUX_
#define DLL_EXPORT __attribute__((visibility("default")))
#else
#define DLL_EXPORT _declspec(dllexport) 
#endif

#include "CommDef.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace Hsc3 {
namespace Comm {

class CommApp;
class FtpApi;

/**
*   @class      CommApi
*   @brief      通信客户端
*   @details    提供接口包含：通信连接配置、连接控制、数据操作等。
*   @date       2018/06/15
*/
class DLL_EXPORT CommApi
{
public:
    /**
     * @brief   构造函数
     * @param   strLogPath  通信日志文件路径（例："comm_log"，"./log/comm_log"）。注：若输入空串，则不记录通信日志
     */
    CommApi(const std::string & strLogPath = "");

    ~CommApi(void);

    /** 
    *   @brief  获取版本信息
    *   @return 版本信息字符串
    */
    std::string getVersionStr();

    /** 
    *   @brief  配置自动重连功能
    *   @details    自动重连模式：使用NetConnect接口触发自动重连，客户端在通信断开后可再次连接，直到使用NetExit接口停止自动重连。
    *               非自动重连模式：使用NetConnect接口进行连接，客户端在通信断开后不会重连，除非再次调用NetConnect接口。
    *               注：CommApi构造时默认为自动重连模式。建议在调用NetConnect接口前进行配置。
    *   @param  en  是否启用
    */
    void setAutoConn(bool en);

    /** 
    *   @brief  配置UDP功能
    *   @details    若启用UDP功能，在TCP通信已连接后，将尝试UDP通信连接。
    *               注：CommApi构造时默认不启用UDP功能。建议在调用NetConnect接口前进行配置。
    *   @param  en  是否启用
    */
    void setUseUdp(bool en);

    /** 
    *   @brief  查询是否已连接
    *   @details    注：返回值仅表示TCP通信连接的状态，与UDP/FTP无关。
    *   @return 返回
    *           <em>true</em>  - 已连接
    *           <em>false</em> - 未连接
    */
    bool isConnected();

    /** 
    *   @brief  连接
    *   @details    自动重连模式：使用该接口返回0x0表示成功触发自动重连功能（不能多次触发），不代表通信已连接。
    *               非自动重连模式：使用该接口会阻塞调用线程，直至连接成功/超时（约3s）失败/其它原因导致的失败。
    *   @param  strIp    IP
    *   @param  uPort    端口
    *   @return 错误码（0x0成功）
    *   @see        setAutoConn
    */
    HMCErrCode connect(const std::string & strIp, uint16_t uPort);

    /** 
    *   @brief  断开连接
    *   @details    注：自动重连模式下使用该接口，还会停止自动重连功能。
    *   @return 错误码（0x0成功）
    *   @see        setAutoConn
    */
    HMCErrCode disconnect();

    /** 
    *   @brief  执行命令
    *   @details    注：使用该接口会阻塞调用线程，直至成功/超时（约8s）失败/其它原因导致的失败。
    *   @param  strCmd      命令内容（参考华数III型控制器业务层接口协议文档填写命令）
    *   @param  priority    优先级（值越小，优先级越高；建议使用CmdPriority枚举类型）
    *   @param[out] strRet  返回信息
    *   @return 错误码（0x0成功）
    */
    HMCErrCode execCmd(const std::string & strCmd, std::string & strRet, int priority);

    /** 
    *   @brief  执行命令
    *   @details    注：使用该接口会阻塞调用线程，直至成功/超时（约8~10s）失败/其它原因导致的失败。
    *   @param  hmcCmd      命令内容（字符串队列，参考华数III型控制器业务层接口协议文档填写命令）
    *   @param  priority    优先级（值越小，优先级越高；建议使用CmdPriority枚举类型）
    *   @param[out] hmcResult   返回信息（HMCResultUnit队列）
    *   @return 错误码（0x0成功）
    */
    HMCErrCode execListCmd(const HMCCmd & hmcCmd, HMCResult & hmcResult, int priority);

    /** 
    *   @brief  获取异步信息
    *   @details    异步信息是控制器主动发送到客户端的信息。注：获取到信息后，客户端内部不会再保留该信息。
    *   @param[out] strMsg  返回信息（无信息时返回空串）
    *   @return 错误码（0x0成功，总是返回0x0）
    */
    HMCErrCode getAsyncMsg(std::string & strMsg);

    /** 
    *   @brief  清空异步信息
    *   @details    清空客户端内部所有的异步信息。
    *   @details    注：获取到信息后，客户端内部不会再保留该信息。
    *   @return 错误码（0x0成功，总是返回0x0）
    */
    HMCErrCode clearAsyncMsg();


    /********************************* UDP相关 *********************************/

    /** 
    *   @brief  获取UDP信息
    *   @details    UDP信息是控制器通过UDP通信主动发送到客户端的信息。注：获取到信息后，客户端内部不会再保留该信息。
    *   @param[out] strMsg  返回信息（无信息时返回空串）
    *   @return 错误码（0x0成功，总是返回0x0）
    *   @see        setUseUdp
    */
    HMCErrCode getUdpMsg(std::string & strMsg);

    /** 
    *   @brief  获取UDP对话ID
    *   @details    注：配置启动UDP功能，TCP通信连接约3s后，待UDP通信已连接，方可获取到正常ID。
    *   @param[out] id  对话ID（正常为大于0的整数，-1表示不存在UDP通信）
    *   @return 错误码（0x0成功）
    *   @see        setUseUdp
    */
    HMCErrCode getUdpSessionId(int & id);

    /** 
    *   @brief  清空缓存的UDP信息
    *   @details    清空客户端内部所有的UDP信息。
    *   @return 错误码（0x0成功，总是返回0x0）
    *   @see        setUseUdp
    */
    HMCErrCode clearUdpMsg();


    /********************************* FTP相关 *********************************/

    /** 
    *   @brief  下载文件
    *   @param  strLocalName    本地文件路径名字
    *   @param  strSrcName      源（服务器）文件路径名字
    *   @return 错误码（0x0成功）
    */
    HMCErrCode getFile(const std::string & strLocalName, const std::string & strSrcName);

    /** 
    *   @brief  上传文件
    *   @param  strDstName      目标（服务器）文件路径名字
    *   @param  strLocalName    本地文件路径名字
    *   @return 错误码（0x0成功）
    */
    HMCErrCode putFile(const std::string & strDstName, const std::string & strLocalName);

    /** 
    *   @brief  删除文件
    *   @param  strPath     （服务器）文件路径名字
    *   @return 错误码（0x0成功）
    */
    HMCErrCode delFile(const std::string & strPath);

    /** 
    *   @brief  删除目录
    *   @details    注：若目录非空，删除操作将失败，可使用fileListInfo接口检查目录列表信息。
    *   @param  strPath     （服务器）目录名字
    *   @return 错误码（0x0成功）
    *   @see        fileListInfo
    */
    HMCErrCode delDir(const std::string & strPath);

    /** 
    *   @brief  创建目录
    *   @param  strPath     （服务器）目录名字
    *   @return 错误码（0x0成功）
    */
    HMCErrCode mkDir(const std::string & strPath);

    /** 
    *   @brief  获取目录列表信息
    *   @param  strDirPath  （服务器）目录名字
    *   @param[out] pFileInfo   目录列表信息
    *   @return 错误码（0x0成功）
    */
    HMCErrCode getFileListInfo(const std::string & strDirPath, std::vector<MFileInfo>& pFileInfo);

private:
    CommApi(CommApi & other);
    CommApi & operator = (CommApi & other);

private:
    CommApp * m_pNet;
    FtpApi * m_pFtp;
};


/** 
*   @brief  扫描华数III型控制器设备
*   @details接口非阻塞，调用后立即返回。使用getHsc3DevicesInfo接口获取扫描结果。注：在保持侦听时间内再次调用会返回失败。
*   @param  ulKeepTimeMs    保持侦听的时间（单位：ms），≥1000
*   @return 错误码（0x0成功）
*   @see        getHsc3DevicesInfo
*/
DLL_EXPORT HMCErrCode scanHsc3Devices(uint64_t ulKeepTimeMs);

/** 
*   @brief  获取华数III型控制器设备信息
*   @details调用scanHsc3Devices接口后，调用本接口获取信息。注：不排除有重复的信息；获取到信息后，客户端内部不会再保留该信息。
*   @param[out] listHsc3Info    网络中华数III型控制器的信息
*   @see        scanHsc3Devices
*/
DLL_EXPORT void getHsc3DevicesInfo(std::vector<Hsc3Info> & listHsc3Info);

}
}
