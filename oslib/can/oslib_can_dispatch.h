/**
 * @file oslib_can_dispatch.h
 * @author KYzhang
 * @brief OSLIB-CAN接收分配模块
 * @version 0.2
 * @date 2021-01-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __OSLIB_CAN_DISPATCH_H
#define __OSLIB_CAN_DISPATCH_H

#include "oslib_can.h"
#include "cmsis_os2.h"
#include "oslib_hash.h"

#ifdef OSLIB_CAN_MODULE_ENABLED

/*-数据结构-----------------------------------*/
typedef void (*CAN_Callback)(CAN_ConnMessage *msg);
typedef enum
{
    CAN_IDTYPE_STD = CAN_ID_STD,
    CAN_IDTYPE_EXT = CAN_ID_EXT
} CAN_IDTYPE;

typedef struct
{
    uint32_t id;               /* CAN ID */
    CAN_IDTYPE idtype;         /* TYPE: CAN_ID_STD/CAN_ID_EXT */
    osMessageQueueId_t *queue; /* 任务对应的消息队列 */
    CAN_Callback callback;     /* 回调函数 */
} CAN_IDRecord_t;

/*-接口---------------------------------------*/
#if defined(CAN1_ENABLED)
extern HashTable CAN1_QueueHashTable;
#endif
#if defined(CAN2_ENABLED)
extern HashTable CAN2_QueueHashTable;
#endif

extern void OSLIB_CAN_Dispatch_Init(CAN_HandleTypeDef *hcan);

#endif

#endif