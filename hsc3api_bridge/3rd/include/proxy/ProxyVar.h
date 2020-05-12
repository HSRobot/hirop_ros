/**
*   Copyright (C) 2018 华数机器人
*
*   @file       ProxyVar.h
*   @brief      华数III型二次开发接口 - 业务接口 - 变量操作代理
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

#include "Hsc3Def.h"
#include "CommDef.h"
#include <stdint.h>

namespace Hsc3 {

    namespace Comm {
        class CommApi;
    }

    namespace Proxy {
        /**
        *   @class      ProxyVar
        *   @brief      业务接口 - 变量操作代理
        *   @details    提供接口包含：变量操作接口。
        *   @date       2018/06/16
        */
        class DLL_EXPORT ProxyVar
        {
        public:
            /**
             * @brief   构造函数
             * @details 注：确保传入已构造的通信客户端。
             * @param   pNet    通信客户端
             */
            ProxyVar(Hsc3::Comm::CommApi * pNet);

            ~ProxyVar();

            /**
             * @brief   保存轴数据
             * @param   axId    轴号（0..n-1）
             * @param   file    数据文件名，选项：
                                <em>"para"</em> - 参数
                                <em>"home"</em> - 原点相关参数
                                <em>"limit"</em> - 软限位相关参数
             */
	        Hsc3::Comm::HMCErrCode saveAx(int8_t axId, const std::string & file);
	
            /**
             * @brief   保存组数据
             * @param   gpId    组号（0..n-1）
             * @param   file    数据文件名，选项：
                                <em>"para"</em> - 参数；
                                <em>"JR"</em> - JR（关节坐标）寄存器值；
                                <em>"LR"</em> - LR（笛卡尔坐标）寄存器值；
                                <em>"UT"</em> - UT寄存器值；
                                <em>"UF"</em> - UF寄存器值；
                                <em>"area"</em> - 区域相关参数
                                <em>"track"</em> - 跟踪相关参数
             */
	        Hsc3::Comm::HMCErrCode saveGp(int8_t gpId, const std::string & file);

            /**
             * @brief   保存公共数据
             * @param   file    数据文件名，选项：
                                <em>"R"</em> - R（实数）寄存器值
             */
	        Hsc3::Comm::HMCErrCode saveCommon(const std::string & file);

            /**
             * @brief   设置R（实数）寄存器值
             * @param   index   索引
             * @param   value   值
             */
	        Hsc3::Comm::HMCErrCode setR(int32_t index, double value);

            /**
             * @brief   获取R（实数）寄存器值
             * @param   index   索引
             * @param[out]   value   值
             */
	        Hsc3::Comm::HMCErrCode getR(int32_t index, double & value);

            /**
             * @brief   获取R（实数）寄存器值
             * @param   index   索引
             * @param   len     数量
             * @param[out]   value   值的列表
             */
            Hsc3::Comm::HMCErrCode getR(int32_t index, int32_t len, std::vector<double> & value);

            /**
             * @brief   设置JR（关节坐标）寄存器值
             * @param   gpId    组号（0..n-1）
             * @param   index   索引
             * @param   data    值
             */
	        Hsc3::Comm::HMCErrCode setJR(int8_t gpId, int32_t index, const JntPos & data);

            /**
             * @brief   获取JR（关节坐标）寄存器值
             * @param   gpId    组号（0..n-1）
             * @param   index   索引
             * @param[out]   data    值
             */
	        Hsc3::Comm::HMCErrCode getJR(int8_t gpId, int32_t index, JntPos & data);

            /**
             * @brief   获取JR（关节坐标）寄存器值
             * @param   gpId    组号（0..n-1）
             * @param   index   索引
             * @param   len     数量
             * @param[out]   listData    值的列表
             */
            Hsc3::Comm::HMCErrCode getJR(int8_t gpId, int32_t index, int32_t len, std::vector<JntPos> & listData);

            /**
             * @brief   设置LR（笛卡尔坐标）寄存器值
             * @param   gpId    组号（0..n-1）
             * @param   index   索引
             * @param   data    值
             */
	        Hsc3::Comm::HMCErrCode setLR(int8_t gpId, int32_t index, const LocPos & data);

            /**
             * @brief   获取LR（笛卡尔坐标）寄存器值
             * @param   gpId    组号（0..n-1）
             * @param   index   索引
             * @param[out]   data    值
             */
	        Hsc3::Comm::HMCErrCode getLR(int8_t gpId, int32_t index, LocPos & data);

            /**
             * @brief   获取LR（笛卡尔坐标）寄存器值
             * @param   gpId    组号（0..n-1）
             * @param   index   索引
             * @param   len     数量
             * @param[out]   listData    值的列表
             */
            Hsc3::Comm::HMCErrCode getLR(int8_t gpId, int32_t index, int32_t len, std::vector<LocPos> & listData);

            /**
             * @brief   配置区域
             * @param   gpId    组号（0..n-1）
             * @param   index   索引
             * @param   data    值
             */
	        Hsc3::Comm::HMCErrCode configArea(int8_t gpId, int32_t index, const AreaConfigData & data);

            /**
             * @brief   获取区域类型
             * @param   gpId    组号（0..n-1）
             * @param   index   索引
             * @param[out]   type      区域类型
             */
            Hsc3::Comm::HMCErrCode getAreaType(int8_t gpId, int8_t index, AreaType & type);

            /**
             * @brief   获取区域类型
             * @param   gpId    组号（0..n-1）
             * @param[out]   listType      区域类型
             */
            Hsc3::Comm::HMCErrCode getAreaType(int8_t gpId, std::vector<AreaType> & listType);

            /**
             * @brief   使能共享区
             * @param   gpId    组号（0..n-1）
             * @param   index   索引
	         * @param   en      是否使能
             */
            Hsc3::Comm::HMCErrCode setShareAreaEn(int8_t gpId, int8_t index, bool en);
	
	        /**
             * @brief   获取共享区使能
             * @param   gpId    组号（0..n-1）
             * @param   index   索引
             * @param[out]   en      是否使能
             */
            Hsc3::Comm::HMCErrCode getShareAreaEn(int8_t gpId, int8_t index, bool & en);

            /**
             * @brief   获取共享区使能
             * @param   gpId    组号（0..n-1）
             * @param[out]   listEn      是否使能
             */
            Hsc3::Comm::HMCErrCode getShareAreaEn(int8_t gpId, std::vector<bool> & listEn);

            /**
             * @brief   获取区域输出
             * @param   gpId    组号（0..n-1）
             * @param   index   索引
             * @param[out]   en      是否输出
             */
            Hsc3::Comm::HMCErrCode getAreaOut(int8_t gpId, int8_t index, bool & en);

            /**
             * @brief   获取区域输出
             * @param   gpId    组号（0..n-1）
             * @param[out]   listEn      是否使能
             */
            Hsc3::Comm::HMCErrCode getAreaOut(int8_t gpId, std::vector<bool> & listEn);

            /**
             * @brief   获取区域配置
             * @param   gpId    组号（0..n-1）
             * @param   index   索引
             * @param[out]   data    值
             */
	        Hsc3::Comm::HMCErrCode getAreaConfig(int8_t gpId, int32_t index, AreaConfigData & data);

        private:
            Hsc3::Comm::CommApi * m_pNet;
        };
    }

}