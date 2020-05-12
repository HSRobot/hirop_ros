/**
*   Copyright (C) 2018 华数机器人
*
*   @file       ProxySys.h
*   @brief      华数III型二次开发接口 - 业务接口 - 系统功能代理
*   @details    提供了III型控制器系统相关业务接口。
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

namespace Hsc3 {

    namespace Comm {
        class CommApi;
    }

    namespace Proxy {
        /**
        *   @class      ProxySys
        *   @brief      业务接口 - 系统功能代理
        *   @details    提供接口包含：控制器系统操作接口。
        *   @date       2018/07/03
        */
        class DLL_EXPORT ProxySys
        {
        public:
            /**
             * @brief   构造函数
             * @details 注：确保传入已构造的通信客户端。
             * @param   pNet    通信客户端
             */
            ProxySys(Hsc3::Comm::CommApi * pNet);

            ~ProxySys();

            /**
             * @brief   系统复位
             */
            Hsc3::Comm::HMCErrCode reset();

            /**
             * @brief   查询报警详情
             * @param   errCode     错误码
             * @param[out]  strReason   错误原因
             * @param[out]  strElim     排除措施
             */
	        Hsc3::Comm::HMCErrCode queryError(uint64_t errCode, std::string & strReason, std::string & strElim);

            /**
             * @brief   系统是否就绪
             * @param[out]  ok      已/未就绪
             */
	        Hsc3::Comm::HMCErrCode isReady(bool & ok);

            /**
             * @brief   获取系统报警
             * @details 使用该接口会阻塞调用线程，直至获取到信息/等待超时。
             * @param[out]  level   报警级别
             * @param[out]  code    错误码
             * @param[out]  strMsg  信息内容
             * @param   ulWaitTime  等待时间（单位：ms）
             * @return  <em>0x0</em> - 获取到合法信息（但不排除strMsg为空）；
                        <em>KM_ERR_INVALID_MESSAGE</em> - 获取到信息但信息无效；
                        <em>KM_ERR_NO_MESSAGE</em> - 未收到信息（等待超时）
             */
            Hsc3::Comm::HMCErrCode getMessage(ErrLevel & level, uint64_t & code, std::string & strMsg, uint32_t ulWaitTime);

            /**
             * @brief   获取版本信息
             * @param   key     键
                                <em>"Release"</em> - 控制器发布版本；
                                <em>"CL"</em> - 控制器业务层；
                                <em>"ML"</em> - 控制器运动层；
                                <em>"Api"</em> - 二次开发接口
             * @param[out]  value   版本信息
             */
            Hsc3::Comm::HMCErrCode getVersion(const std::string & key, std::string & value);

            /**
             * @brief   系统关机
             */
	        Hsc3::Comm::HMCErrCode shutdown();
	
	        /**
             * @brief   系统重启
             */
	        Hsc3::Comm::HMCErrCode reboot();

            /**
             * @brief   设置语言
	         * @param   strLang     语言名
             */
	        Hsc3::Comm::HMCErrCode setLang(const std::string & strLang);

        private:
            Hsc3::Comm::CommApi * m_pNet;
        };
    }

}