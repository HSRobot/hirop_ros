/**
*   Copyright (C) 2018 华数机器人
*
*   @file       ProxyCollect.h
*   @brief      华数III型二次开发接口 - 业务接口 - 采集功能代理
*   @details    提供了III型控制器采集相关业务接口。
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

#include "Hsc3Def.h"
#include "CommDef.h"
#include <stdint.h>

namespace Hsc3 {

    namespace Comm {
        class CommApi;
    }

    namespace Proxy {
        /**
        *   @class      ProxyCollect
        *   @brief      业务接口 - 采集代理
        *   @details    提供接口包含：采集操作接口。
        *   @date       2018/06/16
        */
        class DLL_EXPORT ProxyCollect
        {
        public:
            /**
             * @brief   构造函数
             * @details 注：确保传入已构造的通信客户端。
             * @param   pNet    通信客户端
             */
            ProxyCollect(Hsc3::Comm::CommApi * pNet);

            ~ProxyCollect();

            /**
             * @brief   添加采集任务
             * @details 一个采集任务包含若干采集变量（参考华数III型控制器可采集变量列表），采样周期为k*T，其中k为倍数，T为基础周期4ms。
                        注：不能重复添加同一ID的采集任务。
             * @param   tid         采集任务ID
             * @param   iSampleKT   采样周期倍数K（≥1）
             * @param   strVarList  采集变量名列表（格式：半角逗号分隔的名字列表，例“var.axis[0].pfb,var.axis[1].pfb,”）
             * @see     delTask
             */
            Hsc3::Comm::HMCErrCode addTask(int32_t tid, int32_t iSampleKT, std::string strVarList);

            /**
             * @brief   删除采集任务
             * @param   tid         采集任务ID
             * @see     addTask
             */
            Hsc3::Comm::HMCErrCode delTask(int32_t tid);

            /**
             * @brief   启动所有采集任务
             * @details 采集数据通过UDP返回（参考采集数据封装协议）。
             * @see     CommApi::getUdpMsg stop
             */
            Hsc3::Comm::HMCErrCode start();

            /**
             * @brief   启动采集任务
             * @details 采集数据通过UDP返回（参考采集数据封装协议）。
             * @param   tid         采集任务ID
             * @see     getUdpMsg stop
             */
            Hsc3::Comm::HMCErrCode start(int32_t tid);

            /**
             * @brief   停止所有采集任务
             * @see     start
             */
            Hsc3::Comm::HMCErrCode stop();

            /**
             * @brief   停止采集任务
             * @param   tid         采集任务ID
             * @see     start
             */
            Hsc3::Comm::HMCErrCode stop(int32_t tid);

            /**
             * @brief   获取存在的采集任务列表
             * @param[out]  listTask        采集任务ID列表
             */
            Hsc3::Comm::HMCErrCode getExistTasks(std::vector<int32_t> & listTask);
	
            /**
             * @brief   获取正在运行的采集任务列表
             * @param[out]  listTask        采集任务ID列表
             */
            Hsc3::Comm::HMCErrCode getRunningTasks(std::vector<int32_t> & listTask);

            /**
             * @brief   获取控制系统当前时间戳
             * @details 建议启动采集任务前先获取控制系统的当前时间戳，通过采集数据中的时间戳与之相减的差值，可换算出距离采集启动的时间。
             * @param[out]  timestamp       时间戳
             */
            Hsc3::Comm::HMCErrCode getCurTimestamp(int32_t & timestamp);

        private:
            Hsc3::Comm::CommApi * m_pNet;
        };
    }

}