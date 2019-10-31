/**
*   Copyright (C) 2018 华数机器人
*
*   @file       ProxyIO.h
*   @brief      华数III型二次开发接口 - 业务接口 - IO操作代理
*   @details    提供了III型控制器IO相关业务接口。
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
        *   @class      ProxyIO
        *   @brief      业务接口 - IO操作代理
        *   @details    提供接口包含：IO操作接口。
        *   @date       2018/06/16
        */
        class DLL_EXPORT ProxyIO
        {
        public:
            /**
             * @brief   构造函数
             * @details 注：确保传入已构造的通信客户端。
             * @param   pNet    通信客户端
             */
            ProxyIO(Hsc3::Comm::CommApi * pNet);

            ~ProxyIO();

            /**
             * @brief   获取数字输入端口数量
             * @param[out]   num     数量
             */
	        Hsc3::Comm::HMCErrCode getMaxDinNum(int32_t & num);

            /**
             * @brief   获取数字输出端口数量
             * @param[out]   num     数量
             */
	        Hsc3::Comm::HMCErrCode getMaxDoutNum(int32_t & num);

            /**
             * @brief   获取数字输入分组数量
             * @details 32个输入为一组（组0包括输入0..31、组1包括输入32..63、...）。
             * @param[out]   num     数量
             */
	        Hsc3::Comm::HMCErrCode getMaxDinGrp(int32_t & num);

            /**
             * @brief   获取数字输出分组数量
             * @details 32个输出为一组（组0包括输出0..31、组1包括输出32..63、...）。
             * @param[out]   num     数量
             */
	        Hsc3::Comm::HMCErrCode getMaxDoutGrp(int32_t & num);

            /**
             * @brief   获取数字输入分组值
             * @param   grpIndex    分组号（0..n-1）
             * @param[out]  num     分组值（32位无符号整数，最低位代表分组中数字输入0的值，最高位代表数字输入31的值）
             */
	        Hsc3::Comm::HMCErrCode getDinGrp(int32_t grpIndex, uint32_t & num);

             /**
             * @brief   获取数字输出分组值
             * @param   grpIndex    分组号（0..n-1）
             * @param[out]  num     分组值（32位无符号整数，最低位代表分组中数字输出0的值，最高位代表数字输出31的值）
             */
	        Hsc3::Comm::HMCErrCode getDoutGrp(int32_t grpIndex, uint32_t & num);

            /**
             * @brief   获取数字输入分组状态
             * @param   grpIndex    分组号（0..n-1）
             * @param[out]  num     分组状态（32位无符号整数，最低位代表分组中数字输入0的状态，最高位代表数字输入31的状态，状态：<em>true</em> - 虚拟；<em>false</em> - 真实）
             * @see     setDinMaskBit
             */
	        Hsc3::Comm::HMCErrCode getDinMaskGrp(int32_t grpIndex, uint32_t & num);

            /**
             * @brief   获取数字输出分组状态
             * @param   grpIndex    分组号（0..n-1）
             * @param[out]  num     分组状态（32位无符号整数，最低位代表分组中数字输出0的状态，最高位代表数字输出31的状态，状态：<em>true</em> - 虚拟；<em>false</em> - 真实）
             * @see     setDoutMaskBit
             */
	        Hsc3::Comm::HMCErrCode getDoutMaskGrp(int32_t grpIndex, uint32_t & num);

            /**
             * @brief   设置数字输入端口值
             * @param   portIndex   端口号（0..n-1）
             * @param   value       值
             * @see     getDinGrp
             */
	        Hsc3::Comm::HMCErrCode setDin(int32_t portIndex, bool value);

            /**
             * @brief   设置数字输出端口值
             * @param   portIndex   端口号（0..n-1）
             * @param   value       值
             * @see     getDoutGrp
             */
	        Hsc3::Comm::HMCErrCode setDout(int32_t portIndex, bool value);

            /**
             * @brief   获取数字输入端口值
             * @param   portIndex   端口号（0..n-1）
             * @param[out]   value  值
             * @see     getDinGrp
             */
	        Hsc3::Comm::HMCErrCode getDin(int32_t portIndex, bool & value);

            /**
             * @brief   获取数字输出端口值
             * @param   portIndex   端口号（0..n-1）
             * @param[out]   value  值
             * @see     getDoutGrp
             */
	        Hsc3::Comm::HMCErrCode getDout(int32_t portIndex, bool & value);

            /**
             * @brief   设置数字输入状态值
             * @details 数字输入状态为“虚拟”时，可通过setDin设置值。
                        数字输入状态为“真实”时，值从IO模块中读取，通过setDin设置值是无效的。
             * @param   portIndex   端口号（0..n-1）
             * @param   stat        状态：<em>true</em> - 虚拟；<em>false</em> - 真实
             * @see     getDinMaskGrp setDin
             */
	        Hsc3::Comm::HMCErrCode setDinMaskBit(int32_t portIndex, bool stat);

            /**
             * @brief   设置数字输出状态值
             * @details 数字输入状态为“虚拟”时，可通过setDout设置值，但不能触发IO模块输出。
                        数字输入状态为“真实”时，可通过setDout设置值，并可触发IO模块输出。
             * @param   portIndex   端口号（0..n-1）
             * @param   stat        状态：<em>true</em> - 虚拟；<em>false</em> - 真实
             * @see     getDoutMaskGrp setDout
             */
	        Hsc3::Comm::HMCErrCode setDoutMaskBit(int32_t portIndex, bool stat);
		
            /**
             * @brief   钩模块功能文字式交互
             * @details 注：该接口基于复杂的协议，不建议直接使用。
             * @param   strIntent   交互意图
             * @param   strMsg      发送信息
             * @param[out]  strResponse     控制器回复
             */
            Hsc3::Comm::HMCErrCode contactOnHook(const std::string & strIntent, const std::string & strMsg, std::string & strResponse);

        private:
            inline Hsc3::Comm::HMCErrCode getGlobalCommand(std::string cmd, std::string & ret);
            inline Hsc3::Comm::HMCErrCode getGroupCommand(std::string cmd, int8_t gpId, std::string & ret);

        private:
            Hsc3::Comm::CommApi * m_pNet;
        };
    }

}