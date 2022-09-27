/**
 * @file oslib_uart.h
 * @author KYzhang
 * @brief OSLIB-UART发送接收管理
 * @version 0.2
 * @date 2021-01-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __OSLIB_UART_H
#define __OSLIB_UART_H

#include "cmsis_os2.h"
#include "oslib_config.h"
#include "usart.h"

#ifdef OSLIB_UART_MODULE_ENABLED

/*-宏----------------------------------------*/

#define TxBufferSize 256 // 发送缓冲区大小 //TODO: 想办法降低发送缓冲区大小
#define RxBufferSize 128  // 接收缓冲区大小

/*-接口函数-----------------------------------*/

extern void OSLIB_Uart_Printf(UART_HandleTypeDef *huart, const char *fmt, ...);
extern void OSLIB_Uart_SendData(UART_HandleTypeDef *huart, const uint8_t *msg, uint32_t size);
extern void OSLIB_Uart_Init(void);
extern void OSLIB_UART_RxIdleCallback(UART_HandleTypeDef *huart);

/*-接口信号量/缓冲区---------------------------------*/

#if defined(UART1_ENABLED)
extern osSemaphoreId_t UART1_RxSema;
extern char UART1_TaskBuffer[RxBufferSize];
extern int UART1_TaskBufferSize;
#endif
#if defined(UART2_ENABLED)
extern osSemaphoreId_t UART2_RxSema;
extern char UART2_TaskBuffer[RxBufferSize];
extern int UART2_TaskBufferSize;
#endif
#if defined(UART3_ENABLED)
extern osSemaphoreId_t UART3_RxSema;
extern char UART3_TaskBuffer[RxBufferSize];
extern int UART3_TaskBufferSize;
#endif
#if defined(UART4_ENABLED)
extern osSemaphoreId_t UART4_RxSema;
extern char UART4_TaskBuffer[RxBufferSize];
extern int UART4_TaskBufferSize;
#endif
#if defined(UART5_ENABLED)
extern osSemaphoreId_t UART5_RxSema;
extern char UART5_TaskBuffer[RxBufferSize];
extern int UART5_TaskBufferSize;
#endif
#if defined(UART6_ENABLED)
extern osSemaphoreId_t UART6_RxSema;
extern char UART6_TaskBuffer[RxBufferSize];
extern int UART6_TaskBufferSize;
#endif

/*-主串口--------------------------------------------*/

#if SELECT_MAJOR_UART == 1 && defined(UART1_ENABLED)
#define huart_major huart1
#elif SELECT_MAJOR_UART == 2 && defined(UART2_ENABLED)
#define huart_major huart2
#elif SELECT_MAJOR_UART == 3 && defined(UART3_ENABLED)
#define huart_major huart3
#elif SELECT_MAJOR_UART == 4 && defined(UART4_ENABLED)
#define huart_major huart4
#elif SELECT_MAJOR_UART == 5 && defined(UART5_ENABLED)
#define huart_major huart5
#elif SELECT_MAJOR_UART == 6 && defined(UART6_ENABLED)
#define huart_major huart6
#else
#error "[OSLIB] Invalid UART"#SELECT_MAJOR_UART"for CLI"
#endif

/*-兼容定义-----------------------------------*/

#define uprintf(fmt, args...) OSLIB_Uart_Printf(&huart_major, fmt, ##args)
#define uprintf_to(huart, fmt, args...) OSLIB_Uart_Printf(huart, fmt, ##args)

#endif

#endif