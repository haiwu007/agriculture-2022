/**
 * @file oslib_can.c
 * @author KYzhang
 * @brief OSLIB-CAN发送接收管理
 * @version 0.2
 * @date 2021-01-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "oslib_can.h"
#include "oslib_can_dispatch.h"
#include "can_func.h"
#if USE_OSLIB_CAN_EXAMPLE
#include "oslib_can_example.h"
#endif

#include "oslib_debug.h"

//TODO: 优化对f103的兼容

#ifdef OSLIB_CAN_MODULE_ENABLED

/*-信号量定义---------------------------------------------------*/
/* CAN1发送互斥信号量 */
#if defined(CAN1_ENABLED)
static osSemaphoreId_t CAN1_TxSema;
const osSemaphoreAttr_t Can1TxSema_Attributes = {
    .name = "Can1TxSema"};
#endif
#if defined(CAN2_ENABLED)
static osSemaphoreId_t CAN2_TxSema;
const osSemaphoreAttr_t Can2TxSema_Attributes = {
    .name = "Can2TxSema"};
#endif
/*-HAL库中断回调函数----------------------------------------------*/
/**
 * @brief 发送邮箱0发送完成中断
 * @param hcan CAN句柄
 */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
#if defined(CAN1_ENABLED)
    if (hcan->Instance == CAN1)
    {
        Debug("CAN1: Tx Mailbox0");
        osSemaphoreRelease(CAN1_TxSema);
    }
    else
#endif
#if defined(CAN2_ENABLED)
        if (hcan->Instance == CAN2)
    {
        Debug("CAN2: Tx Mailbox0");
        osSemaphoreRelease(CAN2_TxSema);
    }
    else
#endif
        Warn("Unknown CAN");
}
/**
 * @brief 发送邮箱1发送完成中断
 * @param hcan CAN句柄
 */
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
#if defined(CAN1_ENABLED)
    if (hcan->Instance == CAN1)
    {
        Debug("CAN1: Tx Mailbox1");
        osSemaphoreRelease(CAN1_TxSema);
    }
    else
#endif
#if defined(CAN2_ENABLED)
        if (hcan->Instance == CAN2)
    {
        Debug("CAN2: Tx Mailbox1");
        osSemaphoreRelease(CAN2_TxSema);
    }
    else
#endif
        Error("Unknown CAN");
}
/**
 * @brief 发送邮箱2发送完成中断
 * @param hcan CAN句柄
 */
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
#if defined(CAN1_ENABLED)
    if (hcan->Instance == CAN1)
    {
        Debug("CAN1: Tx Mailbox2");
        osSemaphoreRelease(CAN1_TxSema);
    }
    else
#endif
#if defined(CAN2_ENABLED)
        if (hcan->Instance == CAN2)
    {
        Debug("CAN2: Tx Mailbox2");
        osSemaphoreRelease(CAN2_TxSema);
    }
    else
#endif
        Error("Unknown CAN");
}

/**
 * @brief 接收FIFO0接收完成中断
 * @param hcan CAN句柄
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
#if defined(CAN1_ENABLED)
    if (hcan->Instance == CAN1)
    {
        static CAN_RxHeaderTypeDef hdr;
        static CAN_ConnMessage msg;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, msg.payload.ui8);
        msg.id = (hdr.IDE == CAN_ID_STD) ? hdr.StdId : hdr.ExtId;
        Debug("CAN1: RxFIFO0[0x%x]", msg.id);
        static osSemaphoreId_t *queue;
        if (NULL != (queue = (osSemaphoreId_t *)HashTable_get(CAN1_QueueHashTable, (void const *)msg.id)))
        { //FIXME: 哈希表是否速度较慢影响中断?
            osMessageQueuePut(*queue, &msg, NULL, 0);
        }
    }
    else
#endif
#if defined(CAN2_ENABLED)
        if (hcan->Instance == CAN2)
    {
        static CAN_RxHeaderTypeDef hdr;
        static CAN_ConnMessage msg;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, msg.payload.ui8);
        msg.id = (hdr.IDE == CAN_ID_STD) ? hdr.StdId : hdr.ExtId;
        Debug("CAN2: RxFIFO0[0x%x]", msg.id);
        static osSemaphoreId_t *queue;
        if (NULL != (queue = (osSemaphoreId_t *)HashTable_get(CAN2_QueueHashTable, (void const *)msg.id)))
        { //FIXME: 哈希表是否速度较慢影响中断?
            osMessageQueuePut(*queue, &msg, NULL, 0);
        }
    }
    else
#endif
        Error("Unknown CAN");
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
}

#define VescMask (~(0x1FUL << 8))
/**
 * @brief 接收FIFO1完成中断
 * @param hcan CAN句柄
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
#if defined(CAN1_ENABLED)
    if (hcan->Instance == CAN1)
    {
        static CAN_RxHeaderTypeDef hdr;
        static CAN_ConnMessage msg;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &hdr, msg.payload.ui8);
        msg.id = (hdr.IDE == CAN_ID_STD) ? hdr.StdId : hdr.ExtId;
        Debug("CAN1: RxFIFO1[0x%x]", msg.id);
        static osSemaphoreId_t *queue;
        if (NULL != (queue = (osSemaphoreId_t *)HashTable_get(CAN1_QueueHashTable, (void const *)(msg.id & VescMask))))
        { //FIXME: 哈希表是否速度较慢影响中断?
            osMessageQueuePut(*queue, &msg, NULL, 0);
        }
    }
    else
#endif
#if defined(CAN2_ENABLED)
        if (hcan->Instance == CAN2)
    {
        static CAN_RxHeaderTypeDef hdr;
        static CAN_ConnMessage msg;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &hdr, msg.payload.ui8);
        msg.id = (hdr.IDE == CAN_ID_STD) ? hdr.StdId : hdr.ExtId;
        Debug("CAN2: RxFIFO1[0x%x]", msg.id);
        static osSemaphoreId_t *queue;
        if (NULL != (queue = (osSemaphoreId_t *)HashTable_get(CAN2_QueueHashTable, (void const *)(msg.id & VescMask))))
        { //FIXME: 哈希表是否速度较慢影响中断?
            osMessageQueuePut(*queue, &msg, NULL, 0);
        }
    }
    else
#endif
        Error("Unknown CAN");
}

/**
 * @brief 接收FIFO1满中断
 * @param hcan CAN句柄
 */
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan)
{
}

/**
 * @brief 错误中断, 使用调试宏发送错误信息
 * @param hcan CAN句柄
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
#if defined(CAN1_ENABLED)
    if (hcan->Instance == CAN1)
    {
        if (hcan->ErrorCode & (HAL_CAN_ERROR_TX_ALST0 | HAL_CAN_ERROR_TX_ALST1 | HAL_CAN_ERROR_TX_ALST2)) // 仲裁丢失
            Warn("CAN1: Tx Arbitration lost");
        else if (hcan->ErrorCode & (HAL_CAN_ERROR_TX_TERR0 | HAL_CAN_ERROR_TX_TERR1 | HAL_CAN_ERROR_TX_TERR2)) // 传输错误
            Warn("CAN1: Tx Transmission error");
        else
            Warn("CAN1: Unknown error");
        osSemaphoreRelease(CAN1_TxSema);
    }
    else
#endif
#if defined(CAN2_ENABLED)
        if (hcan->Instance == CAN2)
    {
        if (hcan->ErrorCode & (HAL_CAN_ERROR_TX_ALST0 | HAL_CAN_ERROR_TX_ALST1 | HAL_CAN_ERROR_TX_ALST2)) // 仲裁丢失
            Warn("CAN2: Tx Arbitration lost");
        else if (hcan->ErrorCode & (HAL_CAN_ERROR_TX_TERR0 | HAL_CAN_ERROR_TX_TERR1 | HAL_CAN_ERROR_TX_TERR2)) // 传输错误
            Warn("CAN2: Tx Transmission error");
        else
            Warn("CAN2: Unknown error");
        osSemaphoreRelease(CAN2_TxSema);
    }
    else
#endif
        Error("Unknown CAN");
}

/*-接口函数----------------------------------------------------*/
/**
 * @brief OSLIB-CAN总线相关组件初始化
 */
void OSLIB_CAN_Init()
{
#if defined(CAN1_ENABLED)
    /* 信号量初始化 */
    CAN1_TxSema = osSemaphoreNew(3, 3, &Can1TxSema_Attributes);

    /* 派发器初始化 */
    OSLIB_CAN_Dispatch_Init(&hcan1);

    /* 启动CAN */
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
        Error_Handler();

    /* 使能发送中断 */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
        Error_Handler();

    /* 使能接收中断 */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        Error_Handler();
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_FULL) != HAL_OK)
        Error_Handler();
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
        Error_Handler();
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_FULL) != HAL_OK)
        Error_Handler();
#endif

#if defined(CAN2_ENABLED)
    /* 信号量初始化 */
    CAN2_TxSema = osSemaphoreNew(3, 3, &Can2TxSema_Attributes);

    /* 派发器初始化 */
    OSLIB_CAN_Dispatch_Init(&hcan2);

    /* 启动CAN */
    if (HAL_CAN_Start(&hcan2) != HAL_OK)
        Error_Handler();

    /* 使能发送中断 */
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
        Error_Handler();

    /* 使能接收中断 */
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        Error_Handler();
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_FULL) != HAL_OK)
        Error_Handler();
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
        Error_Handler();
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_FULL) != HAL_OK)
        Error_Handler();
#endif
#if USE_OSLIB_CAN_EXAMPLE
    CAN_Example_Init();
#endif
}

/**
 * @brief CAN消息发送函数, 可发送拓展ID数据帧和标准ID数据帧, 如果ID过长, 会自动认为是拓展帧
 * @param hcan CAN句柄
 * @param idtype CANID类型 标准帧:CAN_ID_STD 拓展帧:CAN_ID_EXT
 * @param id CAN报文ID
 * @param msg CAN报文地址
 */
void OSLIB_CAN_SendMessage(CAN_HandleTypeDef *hcan, uint32_t idtype, uint32_t id, CAN_Message *msg)
{
#if defined(CAN1_ENABLED)
    if (hcan->Instance == CAN1)
    {
        while (osSemaphoreAcquire(CAN1_TxSema, osWaitForever) != osOK)
            ;
    }
    else
#endif
#if defined(CAN2_ENABLED)
        if (hcan->Instance == CAN2)
    {
        while (osSemaphoreAcquire(CAN2_TxSema, osWaitForever) != osOK)
            ;
    }
    else
#endif
    {
        Error("Unknown CAN");
        return;
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
        osDelay(1);

    assert_param(IS_CAN_IDTYPE(idtype));
    if (!IS_CAN_STDID(id))
        idtype = CAN_ID_EXT;

    CAN_TxHeaderTypeDef hdr = {id, id, idtype, CAN_RTR_DATA, 8, DISABLE};

    uint32_t mailbox;

    HAL_CAN_AddTxMessage(hcan, &hdr, msg->ui8, &mailbox);
}

#endif