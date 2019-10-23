/**
*   Copyright (C) 2018 华数机器人
*
*   @file       Entry.h
*   @brief      华数III型二次开发接口 - 业务接口 - 入口
*   @details    提供了III型控制器业务接口的入口。
*
*   @author     
*   @date       2018/10/26
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

class CommApi;


namespace Hsc3 {

    namespace Comm {
        class CommApi;
    }

    namespace Proxy {

        struct EntryPrivate;
        class ProxySys;
        class ProxyMotion;
        class ProxyVm;
        class ProxyVar;
        class ProxyIO;

        /**
        *   @class      Entry
        *   @brief      业务接口 - 入口
        *   @details    提供各具体功能操作接口。
        *   @date       2018/10/26
        */
        class DLL_EXPORT Entry
        {
        public:
            /**
             * @brief   构造函数
             * @details 注：确保传入已构造的通信客户端。
             * @param   pNet    通信客户端
             */
            Entry(Hsc3::Comm::CommApi * pNet);

            ~Entry();

            /**
             * @brief   获取系统功能代理
             * @see     getProxySys
             */
            ProxySys * getProxySys();

            /**
             * @brief   获取运动功能代理
             * @see     ProxyMotion
             */
            ProxyMotion * getProxyMotion();

            /**
             * @brief   获取程序运行代理
             * @see     ProxyVm
             */
            ProxyVm * getProxyVm();

            /**
             * @brief   获取变量操作代理
             * @see     ProxyVar
             */
            ProxyVar * getProxyVar();

            /**
             * @brief   获取IO操作代理
             * @see     ProxyIO
             */
            ProxyIO * getProxyIO();

        private:
            EntryPrivate * m_p;
        };

    }

}

