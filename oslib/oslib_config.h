/**
 * @file oslib_config.h
 * @author KYzhang
 * @brief OSLIB配置
 * @version 0.2
 * @date 2021-01-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __OSLIB_CONFIG_H
#define __OSLIB_CONFIG_H

#include "main.h"

/* Reserved Zone Start 1 */
#define OFF         0
#define USER        1
#define FATAL       1
#define ERROR       2
#define WARN        3
#define INFO        4
#define DEBUG       5
#define ALL         5
/* Reserved Zone End 1*/
/*--------------------------------------------------*/

/* OSLIB模块选择 */
#define OSLIB_DEBUG_MODULE_ENABLED  // 调试输出功能
#define OSLIB_UART_MODULE_ENABLED   // 串口功能
#define OSLIB_CAN_MODULE_ENABLED    // CAN功能



/* 选择使用的外设, 需要和HAL库的配置匹配 */
#define UART1_ENABLED
 #define UART2_ENABLED
// #define UART3_ENABLED
//#define UART4_ENABLED
// #define UART5_ENABLED
// #define UART6_ENABLED

#define CAN1_ENABLED
// #define CAN2_ENABLED



/* 调试输出等级 */
#define SELECT_DEBUG_LEVEL      DEBUG    // 调试输出等级, 共有7级, 允许取值
                                        // [OFF/FATAL/ERROR/WARN/INFO/DEBUG/ALL]

/* 串口命令行 */
#define USE_OSLIB_UART_CLI      1       // 是否使用串口命令行, 允许取值[1-Yes/0-No]
#define SELECT_MAJOR_UART       1       // 选择主串口, 即指定一个使用串口命令行的串口, 允许取值[1-UART1/.../6-UART6]

/* 串口接收方式 */
#define SELECT_UART1_RXMODE     0     // 选择串口1接收模式, 允许取值[0-DMA/1-IT]
#define SELECT_UART2_RXMODE     0     // 选择串口2接收模式, 允许取值[0-DMA/1-IT]
#define SELECT_UART3_RXMODE     0     // 选择串口3接收模式, 允许取值[0-DMA/1-IT]
#define SELECT_UART4_RXMODE     0     // 选择串口4接收模式, 允许取值[0-DMA/1-IT]
#define SELECT_UART5_RXMODE     0     // 选择串口5接收模式, 允许取值[0-DMA/1-IT]
#define SELECT_UART6_RXMODE     0     // 选择串口6接收模式, 允许取值[0-DMA/1-IT]

/* CAN范例任务 */
#define USE_OSLIB_CAN_EXAMPLE   0       // 是否使用CAN范例任务处理CAN报文, 该任务会将报文ID通过主串口打印出来, 允许取值[1-Yes/0-No]
/* CAN1回调方式处理CAN报文 */
#define USE_OSLIB_CAN1_CALLBACK 1       // 是否允许使用回调方式处理CAN报文, 允许取值[1-Yes/0-No]
/* CAN1Vesc特殊处理 */
#define USE_OSLIB_CAN1_VESC     0       // 是否启用对VESC报文的特殊处理
/* CAN2回调方式处理CAN报文 */
#define USE_OSLIB_CAN2_CALLBACK 1       // 是否允许使用回调方式处理CAN报文, 允许取值[1-Yes/0-No]
/* CAN2Vesc特殊处理 */
#define USE_OSLIB_CAN2_VESC     0       // 是否启用对VESC报文的特殊处理

/*--------------------------------------------------*/

/* Reserved Zone Start 2 */
#ifndef HAL_UART_MODULE_ENABLED
#undef OSLIB_UART_MODULE_ENABLED
#endif

#ifndef HAL_CAN_MODULE_ENABLED
#undef OSLIB_CAN_MODULE_ENABLED
#endif
/* Reserved Zone End 2*/
#endif