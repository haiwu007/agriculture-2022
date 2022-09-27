/**
 * @file oslib_uart_cmd.h
 * @author KYzhang
 * @brief OSLIB-UART命令行
 * @version 0.1
 * @date 2021-01-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __OSLIB_UART_CMD_H
#define __OSLIB_UART_CMD_H

#include "cmsis_os2.h"
#include "oslib_config.h"

#ifdef OSLIB_UART_MODULE_ENABLED

/*-指令哈希表定义---------------------------------*/

typedef void (*CommandLineCallback)(int argc, char *argv[]);

typedef struct
{
	char *command;			            /* 命令 */
	char *helpString;		            /* 使用帮助 */
	CommandLineCallback callback;	    /* 回调函数 */
	// int8_t expectNumberofParam;		/* 预期的参数个数 */
} CommandHandle_t;



/*-接口函数--------------------------------------*/
#if USE_OSLIB_UART_CLI
    extern void OSLIB_UartCmd_Init();
    extern void OSLIB_UartCmd_AddCommand(char *command, char *helpString, CommandLineCallback callback);
    extern osThreadId_t UartCommandTaskHandle;
#endif

#endif

#endif