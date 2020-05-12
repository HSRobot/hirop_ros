/**
*   Copyright (C) 2018 华数机器人
*
*   @file       ProxyVm.h
*   @brief      华数III型二次开发接口 - 业务接口 - 程序运行代理
*   @details    提供了III型控制器程序运行相关业务接口。
*
*   @author     
*   @date       2018/06/16
*   @version    
*   
*/

#pragma once
/**
*   @skip DLL_EXPORT
*/
#ifdef _LINUX_
#define DLL_EXPORT __attribute__((visibility("default")))
#else
#define DLL_EXPORT _declspec(dllexport) 
#endif

#include "CommDef.h"
#include "Hsc3Def.h"
#include <stdint.h>
#include <vector>

namespace Hsc3 {

    namespace Comm {
        class CommApi;
    }

    namespace Proxy {
        /**
        *   @class      ProxyVm
        *   @brief      业务接口 - 程序运行代理
        *   @details    提供接口包含：程序运行控制和状态获取。
        *   @date       2018/06/16
        */
        class DLL_EXPORT ProxyVm
        {
        public:
            /**
            * @brief   构造函数
            * @details 注：确保传入已构造的通信客户端。
            * @param   pNet    通信客户端
            */
            ProxyVm(Hsc3::Comm::CommApi * pNet);

            ~ProxyVm();

            /**
            * @brief   加载程序
            * @param   path        路径（不包含文件名）
            * @param   fileName    主程序文件名
            */
	        Hsc3::Comm::HMCErrCode load(const std::string & path, const std::string & fileName);
	
            /**
            * @brief   启动运行
            * @param   fileName    主程序文件名
            */
	        Hsc3::Comm::HMCErrCode start(const std::string & fileName);
	
	        /**
            * @brief   暂停运行
            * @param   fileName    主程序文件名
            */
	        Hsc3::Comm::HMCErrCode pause(const std::string & fileName);
	
	        /**
            * @brief   停止运行
            * @param   fileName    主程序文件名
            */
	        Hsc3::Comm::HMCErrCode stop(const std::string & fileName);
	
	        /**
            * @brief   单步运行
            * @param   fileName    主程序文件名
            */
	        Hsc3::Comm::HMCErrCode step(const std::string & fileName);
	
	        /**
            * @brief   卸载程序
            * @param   fileName    主程序文件名
            */
	        Hsc3::Comm::HMCErrCode unload(const std::string & fileName);
	
            /**
            * @brief   获取自动运行状态
            * @param   fileName    主程序文件名
            * @param[out]  state   状态
            */
	        Hsc3::Comm::HMCErrCode progState(const std::string & fileName, AutoState & state);

            /**
            * @brief   设置单步模式
            * @param   fileName    主程序文件名
            * @param   en          是否单步
            */
	        Hsc3::Comm::HMCErrCode setStepMode(const std::string & fileName, bool en);
	
            /**
            * @brief   获取运行行号
            * @param   fileName    主程序文件名
            * @param[out]  row     行号
            */
	        Hsc3::Comm::HMCErrCode getCurLineNo(const std::string & fileName, int32_t & row);
	
            /**
            * @brief   获取当前运行程序名
            * @param   fileName    主程序文件名
            * @param[out]  progName    程序名
            */
	        Hsc3::Comm::HMCErrCode getCurProgName(const std::string & fileName, std::string & progName);
	
            /**
            * @brief   获取已加载的主程序名列表
            * @param[out]  strProgNames    主程序名列表（格式：中括号扩起、半角逗号分隔的字符串队列，例“[ abc.prg, def.prg ]”）
            */
            Hsc3::Comm::HMCErrCode mainProgNames(std::string & strProgNames);

            /**
            * @brief   获取已加载的主程序名列表
            * @param[out]  progList    主程序名列表
            */
	        Hsc3::Comm::HMCErrCode mainProgNames(std::vector<std::string> & progList);

            /**
            * @brief   获取运行程序信息
            * @param   fileName    主程序文件名
            * @param[out]  info    运行程序信息
            */
	        Hsc3::Comm::HMCErrCode getProgInfo(const std::string & fileName, ProgInfo & info);

        private:
            Hsc3::Comm::CommApi * m_pNet;
        };
    }

}