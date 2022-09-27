/**
 * @file oslib_uart_cmd.c
 * @author KYzhang
 * @brief OSLIB-UART命令行
 * @version 0.2
 * @date 2021-01-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "oslib_uart.h"
#include "oslib_uart_cmd.h"
#include "oslib_hash.h"
#include "FreeRTOS.h"
#include <string.h>
#include "cmd_func.h"

#ifdef OSLIB_UART_MODULE_ENABLED
#if USE_OSLIB_UART_CLI

#define COMMANDLINE_MAX_COUNT 5 /* 一次可接收的命令行数 */
#define PARAMETER_MAX_COUNT 15  /* 一条命令中可容纳的参数数目 */

const char *WelcomeMessage = "OSLIB CommandLine\r\n"; /* 欢迎信息 */
static HashTable CommandHashTable;                    /* 命令哈希表 */

/*-句柄定义----------------------------------------------------*/
/* 串口命令行任务句柄 */
osThreadId_t UartCommandTaskHandle;
const osThreadAttr_t UartCmdTask_Attributes = {
    .name = "UartCmdTask",
    .priority = (osPriority_t)osPriorityRealtime,
    .stack_size = 512 * 4};

/*-串口命令行处理函数-----------------------------------------------*/
/**
 * @brief 命令分割函数, 将接收到的多条命令按';'分隔开
 * @param buffer        接收缓冲区
 * @param count         命令的行数
 * @param commandLine   分割出的多条命令
 */
static void OSLIB_UartCmd_CommandSplit(char *buffer, int *count, char *commandLine[])
{
    char *delim = ";\r\n";
    char *token = strtok(buffer, delim);
    uint32_t index = 0;

    while (token && index < COMMANDLINE_MAX_COUNT)
    {
        commandLine[index++] = token;
        token = strtok(NULL, delim);
    }
    *count = index;
}

/**
 * @brief 命令处理函数, 完成命令解析和回调
 * @param commandLine 命令行
 * @return int        0-成功, 1-失败
 */
static int OSLIB_UartCmd_ProcessCommand(char *commandLine)
{
    char *delim = " ";
    char *token = strtok(commandLine, delim);
    int argc = 0;
    char *argv[PARAMETER_MAX_COUNT];

    while (token && argc < PARAMETER_MAX_COUNT)
    {
        argv[argc++] = token;
        token = strtok(NULL, delim);
    }

    if (argc < 1)
    {
        OSLIB_Uart_Printf(&huart_major, "No Input!\r\n");
        return 1;
    }
    CommandHandle_t *cmd;
    if (NULL == (cmd = (CommandHandle_t *)HashTable_get(CommandHashTable, argv[0])))
    {
        OSLIB_Uart_Printf(&huart_major, "Command Not Found!\r\n");
        return 1;
    }
    cmd->callback(argc, argv);
    return 0;
}

/*-任务函数定义--------------------------------------------------*/
#define UARTn_Task(no)                                                     \
    do                                                                     \
    {                                                                      \
        osSemaphoreAcquire(UART##no##_RxSema, osWaitForever);              \
        int cliCount = 0;                                                  \
        char *cli[COMMANDLINE_MAX_COUNT];                                  \
        OSLIB_UartCmd_CommandSplit(UART##no##_TaskBuffer, &cliCount, cli); \
        for (uint8_t i = 0; i < cliCount; i++)                             \
        {                                                                  \
            OSLIB_Uart_Printf(&huart_major, ">>> %s\r\n", cli[i]);         \
            OSLIB_UartCmd_ProcessCommand(cli[i]);                          \
        }                                                                  \
    } while (0)

/**
 * @brief 串口命令行执行任务
 * @param argument 参数未使用
 */
void UartCommandTask(void *argument)
{
    UNUSED(argument);
    OSLIB_Uart_Printf(&huart_major, WelcomeMessage); // 打印欢迎消息
    for (;;)
    {
#if SELECT_MAJOR_UART == 1
        UARTn_Task(1);
#elif SELECT_MAJOR_UART == 2
        UARTn_Task(2);
#elif SELECT_MAJOR_UART == 3
        UARTn_Task(3);
#elif SELECT_MAJOR_UART == 4
        UARTn_Task(4);
#elif SELECT_MAJOR_UART == 5
        UARTn_Task(5);
#elif SELECT_MAJOR_UART == 6
        UARTn_Task(6);
#else
#error "[OSLIB] Invalid UART"#SELECT_MAJOR_UART"for CLI"
#endif
    }
}
/*-帮助命令函数--------------------------------------------------*/

static void _Command_Help(const void *key, void **value, void *c1)
{
    UNUSED(c1);
    char *help = ((CommandHandle_t *)(*value))->helpString;
    OSLIB_Uart_Printf(&huart_major, "%s: %s\r\n", key, help);
}

static void Command_Help(int argc, char *argv[])
{
    HashTable_map(CommandHashTable, _Command_Help, NULL);
}

/*-接口函数----------------------------------------------------*/
/**
 * @brief 串口命令行初始化
 * @note  对指令的添加需要在该函数调用之后
 */
void OSLIB_UartCmd_Init()
{
    /* 命令哈希表 */
    CommandHashTable = HashTable_create(cmpStr, hashStr, NULL);
    OSLIB_UartCmd_AddCommand("help", "Show this page", Command_Help);

    /* 命令初始化 */
    Command_Init();

    /* 命令处理任务 */
    UartCommandTaskHandle = osThreadNew(UartCommandTask, NULL, &UartCmdTask_Attributes);
}

/**
 * @brief 串口命令行添加命令
 * @param command       命令
 * @param helpString    帮助信息
 * @param callback      回调函数
 */
void OSLIB_UartCmd_AddCommand(char *command, char *helpString, CommandLineCallback callback)
{
    CommandHandle_t *newCmd = (CommandHandle_t *)pvPortMalloc(sizeof(CommandHandle_t));
    newCmd->command = (char *)pvPortMalloc(strlen(command) + 1);
    strcpy(newCmd->command, command);
    newCmd->helpString = (char *)pvPortMalloc(strlen(helpString) + 1);
    strcpy(newCmd->helpString, helpString);
    newCmd->callback = callback;
    HashTable_insert(CommandHashTable, newCmd->command, newCmd);
}

#endif

#endif