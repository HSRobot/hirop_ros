/**
*   Copyright (C) 2018 华数机器人
*
*   @file       ErrDef.h
*   @brief      华数III型控制器二次开发接口错误码定义
*   @details    
*
*   @author     
*   @date       2018/06/15
*   @version    
*/

#ifndef ERRDEF_H
#define ERRDEF_H

/**
*   @brief      错误码制作宏
*   @details    二次开发接口公用错误码（32位）构成：
*               8位 - 预留，
*               8位 - 模块识别ID，
*               16位 - 细节错误码。
*   @param  prod    模块识别ID
*   @param  num     细节错误码
*/
#define MAKE_ERR(prod, num)             ((((unsigned long)(prod))<<16)|(num))


/********************************* 模块识别ID定义 *********************************/

// 库级模块
#define PROD_LOG    (0x70)  ///< 日志库ID
#define PROD_COMM   (0x80)  ///< 通信库ID
#define PROD_HSC3   (0xA0)  ///< 接口库ID

// 普通模块
#define PROD_CLIENT (PROD_COMM | 0x1)   ///< TCP/UDP客户端ID
#define PROD_FTP    (PROD_COMM | 0x4)   ///< FTP客户端ID


/********************************* 错误码定义 *********************************/

#define KM_ERR_OK                       0x000000    ///< 正常
#define KM_ERR_BAD                      0xFFFFFF    ///< 错误（通用）

#define KM_ERR_TIME_OUT                 MAKE_ERR(PROD_CLIENT,   0x0001)     ///< 发送消息成功，但超时未返回
#define KM_ERR_INVALID_UUID             MAKE_ERR(PROD_CLIENT,   0x0002)     ///< 无效帧
#define KM_ERR_FAIL_CONN                MAKE_ERR(PROD_CLIENT,   0x0011)     ///< 连接失败
#define KM_ERR_BAD_IP                   MAKE_ERR(PROD_CLIENT,   0x0012)     ///< 错误的IP输入
#define KM_ERR_FAIL_SEND_MSG            MAKE_ERR(PROD_CLIENT,   0x0020)     ///< 发送消息失败
#define KM_ERR_FAIL_LOSE_MSG            MAKE_ERR(PROD_CLIENT,   0x0021)     ///< 发送时丢失消息
#define KM_ERR_NET_NOT_READY            MAKE_ERR(PROD_CLIENT,   0x0030)     ///< 网络未连接
#define KM_ERR_FAIL_PROTOCOL            MAKE_ERR(PROD_CLIENT,   0x0101)     ///< 返回消息的帧格式有问题
#define KM_ERR_FAIL_DECODE              MAKE_ERR(PROD_CLIENT,   0x0201)     ///< 返回消息的封装格式有问题
#define KM_ERR_MISMATCH_MSG_CNT         MAKE_ERR(PROD_CLIENT,   0x0202)     ///< 返回消息数量不正确
#define KM_ERR_FAIL_BROADCAST           MAKE_ERR(PROD_CLIENT,   0x0301)     ///< 广播失败


#define KM_ERR_FTP_APP_TIME_OUT         MAKE_ERR(PROD_FTP,   0x0001)    ///< FTP通信发送超时
#define KM_ERR_FTP_APP_REPLY_TIME_OUT   MAKE_ERR(PROD_FTP,   0x0002)    ///< FTP通信接收超时
#define KM_ERR_FTP_CREATE_DATA          MAKE_ERR(PROD_FTP,   0x0031)    ///< 创建FTP数据链路失败
#define KM_ERR_FTP_FILE_SERVE           MAKE_ERR(PROD_FTP,   0x0031)    ///< FTP服务端文件存在错误
#define KM_ERR_FTP_FILE_LOCAL           MAKE_ERR(PROD_FTP,   0x0032)    ///< 本地文件存在错误
#define KM_ERR_FTP_DATA_OVERFLOW        MAKE_ERR(PROD_FTP,   0x0071)    ///< FTP接收数据过量
#define KM_ERR_FTP_APP_CONN             MAKE_ERR(PROD_FTP,   0x0072)    ///< 连接失败
#define KM_ERR_FTP_APP_SEND_ABORT       MAKE_ERR(PROD_FTP,   0x0073)    ///< 上传文件中止
#define KM_ERR_FTP_APP_WRTE_ABORT       MAKE_ERR(PROD_FTP,   0x0074)    ///< 下载文件写入本地中止
#define KM_ERR_FTP_APP_PASV             MAKE_ERR(PROD_FTP,   0x0075)    ///< 设置PASV模式失败
#define KM_ERR_FTP_APP_CHECK_DATA       MAKE_ERR(PROD_FTP,   0x0076)    ///< 检查返回值失败

#define KM_ERR_FTP_CMD_PASS             MAKE_ERR(PROD_FTP,   0x0061)    ///< PASS命令失败
#define KM_ERR_FTP_CMD_USER             MAKE_ERR(PROD_FTP,   0x0062)    ///< USER命令失败
#define KM_ERR_FTP_CMD_PWD              MAKE_ERR(PROD_FTP,   0x0063)    ///< PWD命令失败
#define KM_ERR_FTP_CMD_CWD              MAKE_ERR(PROD_FTP,   0x0064)    ///< CWD命令失败
#define KM_ERR_FTP_CMD_TRANS            MAKE_ERR(PROD_FTP,   0x0065)    ///< TRANS命令失败
#define KM_ERR_FTP_CMD_FILE_SIZE        MAKE_ERR(PROD_FTP,   0x0066)    ///< FILE_SIZE命令失败
#define KM_ERR_FTP_CMD_DELE             MAKE_ERR(PROD_FTP,   0x0067)    ///< DELE命令失败
#define KM_ERR_FTP_CMD_RMD              MAKE_ERR(PROD_FTP,   0x0068)    ///< RMD命令失败
#define KM_ERR_FTP_CMD_MKD              MAKE_ERR(PROD_FTP,   0x0069)    ///< MKD命令失败

#define KM_ERR_FTP_SOCKET_ADDR          MAKE_ERR(PROD_FTP,   0x0050)    ///< 错误的IP输入
#define KM_ERR_FTP_SOCKET_INVALID       MAKE_ERR(PROD_FTP,   0x0051)    ///< 无效的socket
#define KM_ERR_FTP_SOCKET_CONN          MAKE_ERR(PROD_FTP,   0x0052)    ///< socket连接失败
#define KM_ERR_FTP_SOCKET_SEND          MAKE_ERR(PROD_FTP,   0x0053)    ///< socket发送失败
#define KM_ERR_FTP_SOCKET_RECV          MAKE_ERR(PROD_FTP,   0x0054)    ///< socket接收失败
#define KM_ERR_FTP_SOCKET_SET           MAKE_ERR(PROD_FTP,   0x0055)    ///< socket配置失败

#define KM_ERR_FTP_FAIL_CONNECT         MAKE_ERR(PROD_FTP,   0x0100)    ///< FTP连接失败
#define KM_ERR_FTP_FAIL_LOGIN           MAKE_ERR(PROD_FTP,   0x0200)    ///< FTP登录失败
#define KM_ERR_FTP_FAIL_GET             MAKE_ERR(PROD_FTP,   0x0300)    ///< FTP下载文件失败
#define KM_ERR_FTP_FAIL_PUT             MAKE_ERR(PROD_FTP,   0x0400)    ///< FTP上传文件失败
#define KM_ERR_FTP_FAIL_DEL             MAKE_ERR(PROD_FTP,   0x0500)    ///< FTP删除文件失败
#define KM_ERR_FTP_FAIL_RMD             MAKE_ERR(PROD_FTP,   0x0A00)    ///< FTP删除目录失败
#define KM_ERR_FTP_FAIL_CH_FOLDER       MAKE_ERR(PROD_FTP,   0x0600)    ///< FTP切换目录失败
#define KM_ERR_FTP_FAIL_MK_FOLDER       MAKE_ERR(PROD_FTP,   0x0800)    ///< FTP新建目录失败
#define KM_ERR_FTP_FAIL_FILE_LIST       MAKE_ERR(PROD_FTP,   0x0900)    ///< FTP获取目录列表信息失败

#define KM_ERR_INVALID_PARAM			MAKE_ERR(PROD_HSC3,   0x0001)	///< 无效参数
#define KM_ERR_FAIL_CONVERSION			MAKE_ERR(PROD_HSC3,   0x0002)	///< 数据转换失败
#define KM_ERR_INVALID_MESSAGE          MAKE_ERR(PROD_HSC3,   0x0008)   ///< 返回消息解析失败
#define KM_ERR_NO_MESSAGE               MAKE_ERR(PROD_HSC3,   0x000C)   ///< 未收到消息
#define KM_ERR_RETURN_FALSE             MAKE_ERR(PROD_HSC3,   0x0010)   ///< 不期望的返回消息内容


#endif // ERRDEF_H
