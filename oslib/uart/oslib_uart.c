/**
 * @file oslib_uart.c
 * @author KYzhang
 * @brief OSLIB-UART发送接收管理
 * @version 0.2
 * @date 2021-01-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "oslib_uart.h"
#include "oslib_uart_cmd.h"
#include "string.h"
#include "stdarg.h"
#include "stdio.h"

#include "oslib_debug.h"
//TODO: 兼容f103

#ifdef OSLIB_UART_MODULE_ENABLED
//TODO: 为了便于调试和阅读, 可能需要将宏函数展开
#define UARTn_TXRES(no)                                     \
    static char UART##no##_TxBuffer[TxBufferSize];          \
    static osSemaphoreId_t UART##no##_TxSema;               \
    const osSemaphoreAttr_t Uart##no##TxSema_Attributes = { \
    .name = "Uart" #no "TxSema"};

#define UARTn_RXRES_DMA(no)                                 \
    static char UART##no##_DMABuffer[RxBufferSize];         \
    char UART##no##_TaskBuffer[RxBufferSize];               \
    int UART##no##_TaskBufferSize;                          \
    osSemaphoreId_t UART##no##_RxSema;                      \
    const osSemaphoreAttr_t Uart##no##RxSema_Attributes = { \
        .name = "Uart" #no "RxSema"};

#define UARTn_RXRES_IT(no)                                      \
    static char UART##no##_RxChar;                              \
    osMessageQueueId_t UART##no##_RxQueue;                       \
    const osMessageQueueAttr_t Uart##no##RxQueue_Attributes = {\
        .name = "Uart" #no "RxQueue"};

/*-缓冲区/信号量定义---------------------------------------------------*/
#if defined(UART1_ENABLED)
UARTn_TXRES(1)
#if (SELECT_UART1_RXMODE == 0)
UARTn_RXRES_DMA(1)
#else
UARTn_RXRES_IT(1)
#endif
#endif
#if defined(UART2_ENABLED)
UARTn_TXRES(2)
#if (SELECT_UART2_RXMODE == 0)
UARTn_RXRES_DMA(2)
#else
UARTn_RXRES_IT(2)
#endif
#endif
#if defined(UART3_ENABLED)
UARTn_TXRES(3)
#if (SELECT_UART3_RXMODE == 0)
UARTn_RXRES_DMA(3)
#else
UARTn_RXRES_IT(3)
#endif
#endif
#if defined(UART4_ENABLED)
UARTn_TXRES(4)
#if (SELECT_UART4_RXMODE == 0)
UARTn_RXRES_DMA(4)
#else
UARTn_RXRES_IT(4)
#endif
#endif
#if defined(UART5_ENABLED)
UARTn_TXRES(5)
#if (SELECT_UART5_RXMODE == 0)
UARTn_RXRES_DMA(5)
#else
UARTn_RXRES_IT(5)
#endif
#endif
#if defined(UART6_ENABLED)
UARTn_TXRES(6)
#if (SELECT_UART6_RXMODE == 0)
UARTn_RXRES_DMA(6)
#else
UARTn_RXRES_IT(6)
#endif
#endif

/*-HAL库中断回调函数----------------------------------------------*/

#define UARTn_TxCplt(no)                       \
    do                                         \
    {                                          \
        Debug("UART" #no ": Tx");              \
        osSemaphoreRelease(UART##no##_TxSema); \
    } while (0)

/**
 * @brief 串口发送中断回调函数
 * @param huart 串口句柄指针
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
#if defined(UART1_ENABLED)
    if (huart->Instance == USART1)
        UARTn_TxCplt(1);
    else
#endif
#if defined(UART2_ENABLED)
        if (huart->Instance == USART2)
        UARTn_TxCplt(2);
    else
#endif
#if defined(UART3_ENABLED)
        if (huart->Instance == USART3)
        UARTn_TxCplt(3);
    else
#endif
#if defined(UART4_ENABLED)
        if (huart->Instance == UART4)
        UARTn_TxCplt(4);
    else
#endif
#if defined(UART5_ENABLED)
        if (huart->Instance == UART5)
        UARTn_TxCplt(5);
    else
#endif
#if defined(UART6_ENABLED)
        if (huart->Instance == USART6)
        UARTn_TxCplt(6);
    else
#endif
        Error("Unknown UART");
}

#define UARTn_RxCplt(no)                                                    \
    do                                                                      \
    {                                                                       \
        Debug("UART" #no ": Rx");                                           \
        osMessageQueuePut(UART##no##_RxQueue, &UART##no##_RxChar, NULL, 0); \
        HAL_UART_Receive_IT(huart, (uint8_t *)&UART##no##_RxChar, 1);       \
    } while (0)

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if defined(UART1_ENABLED) && (SELECT_UART1_RXMODE == 1)
    if (huart->Instance == USART1)
        UARTn_RxCplt(1);
    else
#endif
#if defined(UART2_ENABLED) && (SELECT_UART2_RXMODE == 1)
        if (huart->Instance == USART2)
        UARTn_RxCplt(2);
    else
#endif
#if defined(UART3_ENABLED) && (SELECT_UART3_RXMODE == 1)
        if (huart->Instance == USART3)
        {
            //HAL_UART_Receive_IT(&huart3,(uint8_t*)&Uart1_RxBuff[++Uart1_Rx_Cnt], 1);
            //IMU_RxBuff[++IMU_Rx_Cnt]=UART3_RxChar;//UART3_RxChar    UART##no##_TaskBufferSize
            
            //IMU_Rec_OK=1;
           
            UARTn_RxCplt(3);
        }
        
    else
#endif
#if defined(UART4_ENABLED) && (SELECT_UART4_RXMODE == 1)
        if (huart->Instance == UART4)
        {
            
            UARTn_RxCplt(4);
        }
        
    else
#endif
#if defined(UART5_ENABLED) && (SELECT_UART5_RXMODE == 1)
        if (huart->Instance == UART5)
        UARTn_RxCplt(5);
    else
#endif
#if defined(UART6_ENABLED) && (SELECT_UART6_RXMODE == 1)
        if (huart->Instance == USART6)
        UARTn_RxCplt(6);
    else
#endif
        Error("Unknown UART");
}


#define UARTn_Error(n)                        \
    do                                        \
    {                                         \
        no = #n;                              \
        osSemaphoreRelease(UART##n##_TxSema); \
    } while (0)

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    static char *no;
#if defined(UART1_ENABLED)
    if (huart->Instance == USART1)
        UARTn_Error(1);
    else
#endif
#if defined(UART2_ENABLED)
        if (huart->Instance == USART2)
        UARTn_Error(2);
    else
#endif
#if defined(UART3_ENABLED)
        if (huart->Instance == USART3)
        UARTn_Error(3);
    else
#endif
#if defined(UART4_ENABLED)
        if (huart->Instance == UART4)
        UARTn_Error(4);
    else
#endif
#if defined(UART5_ENABLED)
        if (huart->Instance == UART5)
        UARTn_Error(5);
    else
#endif
#if defined(UART6_ENABLED)
        if (huart->Instance == USART6)
        UARTn_Error(6);
    else
#endif
        no = "?";

    if (huart->ErrorCode & HAL_UART_ERROR_PE)
        Warn("UART%s: Parity Error", no);
    if (huart->ErrorCode & HAL_UART_ERROR_NE)
        Warn("UART%s: Noise Error", no);
    if (huart->ErrorCode & HAL_UART_ERROR_FE)
        Warn("UART%s: Frame Error", no);
    if (huart->ErrorCode & HAL_UART_ERROR_ORE)
        Warn("UART%s: Overrun Error", no);
    if (huart->ErrorCode & HAL_UART_ERROR_DMA)
        Warn("UART%s: DMA transfer Error", no);
}

#define UARTn_RxIdle(no)                                                                    \
    do                                                                                      \
    {                                                                                       \
        Debug("UART" #no ": Rx");                                                           \
        __HAL_UART_CLEAR_IDLEFLAG(huart);                                                   \
        /*HAL_UART_DMAStop(huart);*/                                                        \
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);                                    \
        HAL_DMA_Abort(huart->hdmarx);                                                       \
        CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));               \
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);                                     \
        huart->RxState = HAL_UART_STATE_READY;                                              \
        uint32_t count = RxBufferSize - 1 - __HAL_DMA_GET_COUNTER(huart->hdmarx);           \
        if (count > 0)                                                                      \
        {                                                                                   \
            memcpy(UART##no##_TaskBuffer, UART##no##_DMABuffer, count);                     \
            UART##no##_TaskBufferSize = count;                                              \
            UART##no##_TaskBuffer[count] = '\0';                                            \
            HAL_UART_Receive_DMA(huart, (uint8_t *)UART##no##_DMABuffer, RxBufferSize - 1); \
            osSemaphoreRelease(UART##no##_RxSema);                                          \
        }                                                                                   \
        else                                                                                \
            HAL_UART_Receive_DMA(huart, (uint8_t *)UART##no##_DMABuffer, RxBufferSize - 1); \
    } while (0)

/**
 * @brief 串口空闲中断回调函数
 * @param huart 串口句柄指针
 */
void OSLIB_UART_RxIdleCallback(UART_HandleTypeDef *huart)
{
#if defined(UART1_ENABLED) && (SELECT_UART1_RXMODE == 0)
    if (huart->Instance == USART1)
        UARTn_RxIdle(1);
    else
#endif
#if defined(UART2_ENABLED) && (SELECT_UART2_RXMODE == 0)
        if (huart->Instance == USART2)
        UARTn_RxIdle(2);
    else
#endif
#if defined(UART3_ENABLED) && (SELECT_UART3_RXMODE == 0)
        if (huart->Instance == USART3)
        UARTn_RxIdle(3);
    else
#endif
#if defined(UART4_ENABLED) && (SELECT_UART4_RXMODE == 0)
        if (huart->Instance == UART4)
        UARTn_RxIdle(4);
    else
#endif
#if defined(UART5_ENABLED) && (SELECT_UART5_RXMODE == 0)
        if (huart->Instance == UART5)
        UARTn_RxIdle(5);
    else
#endif
#if defined(UART6_ENABLED) && (SELECT_UART6_RXMODE == 0)
        if (huart->Instance == USART6)
        UARTn_RxIdle(6);
    else
#endif
        Error("Unknown UART");
}

/*-接口函数----------------------------------------------------*/

#define UARTn_TxInit(no)                                                                     \
    do                                                                                       \
    {                                                                                        \
        UART##no##_TxSema = osSemaphoreNew(1, 1, &Uart##no##TxSema_Attributes);              \
    } while (0)

#define UARTn_RxInit_DMA(no)                                                                 \
    do                                                                                       \
    {                                                                                        \
        UART##no##_RxSema = osSemaphoreNew(1, 0, &Uart##no##RxSema_Attributes);              \
        __HAL_UART_ENABLE_IT(&huart##no, UART_IT_IDLE);                                      \
        HAL_UART_Receive_DMA(&huart##no, (uint8_t *)UART##no##_DMABuffer, RxBufferSize - 1); \
    } while (0)

#define UARTn_RxInit_IT(no)                                                                         \
    do                                                                                              \
    {                                                                                               \
        UART##no##_RxQueue = osMessageQueueNew(32, sizeof(uint8_t), &Uart##no##RxQueue_Attributes); \
        HAL_UART_Receive_IT(&huart##no, (uint8_t *)&UART##no##_RxChar, 1);                          \
    } while (0)

/**
 * @brief OSLIB-串口相关组件初始化
 */
void OSLIB_Uart_Init()
{
/* 信号量初始化 */
#if defined(UART1_ENABLED)
    UARTn_TxInit(1);
#if SELECT_UART1_RXMODE == 0
    UARTn_RxInit_DMA(1);
#else
    UARTn_RxInit_IT(1);
#endif
#endif
#if defined(UART2_ENABLED)
    UARTn_TxInit(2);
#if SELECT_UART2_RXMODE == 0
    UARTn_RxInit_DMA(2);
#else
    UARTn_RxInit_IT(2);
#endif
#endif
#if defined(UART3_ENABLED)
    UARTn_TxInit(3);
#if SELECT_UART3_RXMODE == 0
    UARTn_RxInit_DMA(3);
#else
    UARTn_RxInit_IT(3);
#endif
#endif
#if defined(UART4_ENABLED)
    UARTn_TxInit(4);
#if SELECT_UART4_RXMODE == 0
    UARTn_RxInit_DMA(4);
#else
    UARTn_RxInit_IT(4);
#endif
#endif
#if defined(UART5_ENABLED)
    UARTn_TxInit(5);
#if SELECT_UART5_RXMODE == 0
    UARTn_RxInit_DMA(5);
#else
    UARTn_RxInit_IT(5);
#endif
#endif
#if defined(UART6_ENABLED)
    UARTn_TxInit(6);
#if SELECT_UART6_RXMODE == 0
    UARTn_RxInit_DMA(6);
#else
    UARTn_RxInit_IT(6);
#endif
#endif

    /* 任务初始化 */
#if USE_OSLIB_UART_CLI
    OSLIB_UartCmd_Init();
#endif
}

#define UARTn_Printf(no)                                                     \
    do                                                                       \
    {                                                                        \
        while (osSemaphoreAcquire(UART##no##_TxSema, osWaitForever) != osOK) \
            ;                                                                \
        while (HAL_DMA_GetState(huart->hdmatx) == HAL_DMA_STATE_BUSY)        \
            osDelay(1);                                                      \
        int size = 0;                                                        \
        va_list arg_ptr;                                                     \
        va_start(arg_ptr, fmt);                                              \
        size = vsnprintf(UART##no##_TxBuffer, TxBufferSize, fmt, arg_ptr);   \
        if (size >= TxBufferSize)                                            \
        {                                                                    \
            Warn("UART: Tx Too long:%d", size);                              \
            size = TxBufferSize - 1;                                         \
        }                                                                    \
        va_end(arg_ptr);                                                     \
        HAL_UART_Transmit_DMA(huart, (uint8_t *)UART##no##_TxBuffer, size);  \
    } while (0)

/**
 * @brief 串口发送格式化字符串 注意: 请不要在中断中调用该函数!! 如果需要输出调试信息, 请使用LOG宏
 * @param huart     串口句柄
 * @param fmt       待发送字符串
 * @param ...       字符串参数
 */
void OSLIB_Uart_Printf(UART_HandleTypeDef *huart, const char *fmt, ...)
{
#if defined(UART1_ENABLED)
    if (huart->Instance == USART1)
        UARTn_Printf(1);
    else
#endif
#if defined(UART2_ENABLED)
        if (huart->Instance == USART2)
        UARTn_Printf(2);
    else
#endif
#if defined(UART3_ENABLED)
        if (huart->Instance == USART3)
        UARTn_Printf(3);
    else
#endif
#if defined(UART4_ENABLED)
        if (huart->Instance == UART4)
        UARTn_Printf(4);
    else
#endif
#if defined(UART5_ENABLED)
        if (huart->Instance == UART5)
        UARTn_Printf(5);
    else
#endif
#if defined(UART6_ENABLED)
        if (huart->Instance == USART6)
        UARTn_Printf(6);
    else
#endif
        Error("Unknown UART");
}

#define UARTn_SendData(no)                                                   \
    do                                                                       \
    {                                                                        \
        while (osSemaphoreAcquire(UART##no##_TxSema, osWaitForever) != osOK) \
            ;                                                                \
        while (HAL_DMA_GetState(huart->hdmatx) == HAL_DMA_STATE_BUSY)        \
            osDelay(1);                                                      \
        if (size >= TxBufferSize)                                            \
        {                                                                    \
            Warn("UART: Tx Too long:%d", size);                              \
            size = TxBufferSize - 1;                                         \
        }                                                                    \
        memcpy(UART##no##_TxBuffer, msg, size);                              \
        HAL_UART_Transmit_DMA(huart, (uint8_t *)UART##no##_TxBuffer, size);  \
    } while (0)

/**
 * @brief 串口发送数据
 * @param huart 串口句柄
 * @param msg   待发送数据
 * @param size  待发送数据的字节数
 */
void OSLIB_Uart_SendData(UART_HandleTypeDef *huart, const uint8_t *msg, uint32_t size)
{
    #if defined(UART1_ENABLED)
        if (huart->Instance == USART1)
            UARTn_SendData(1);
        else
    #endif
    #if defined(UART2_ENABLED)
            if (huart->Instance == USART2)
            UARTn_SendData(2);
        else
    #endif
    #if defined(UART3_ENABLED)
            if (huart->Instance == USART3)
            UARTn_SendData(3);
        else
    #endif
    #if defined(UART4_ENABLED)
            if (huart->Instance == UART4)
            UARTn_SendData(4);
        else
    #endif
    #if defined(UART5_ENABLED)
            if (huart->Instance == UART5)
            UARTn_SendData(5);
        else
    #endif
    #if defined(UART6_ENABLED)
            if (huart->Instance == USART6)
            UARTn_SendData(6);
        else
    #endif
        Error("Unknown UART");
}

#endif