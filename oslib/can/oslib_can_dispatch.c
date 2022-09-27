/**
 * @file oslib_can_dispatch.c
 * @author KYzhang
 * @brief OSLIB-CAN接收分配模块
 * @version 0.2
 * @date 2021-01-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "oslib_can.h"
#include "oslib_can_dispatch.h"
#include "oslib_hash.h"
#include "can_func.h"
#include "oslib_debug.h"

#ifdef OSLIB_CAN_MODULE_ENABLED

/*-队列哈希表--------------------------------------------------*/

#if defined(CAN1_ENABLED)
HashTable CAN1_QueueHashTable;
#endif
#if defined(CAN2_ENABLED)
HashTable CAN2_QueueHashTable;
#endif

/*--------------------------------------------------------------------*/

#if defined(CAN1_ENABLED) && USE_OSLIB_CAN1_CALLBACK
static HashTable CAN1_CallbackHashTable; // ID-回调函数哈希表
/*-消息队列定义--------------------------------------------------*/
/* CAN1接收回调队列 */
osMessageQueueId_t CAN1_RxCbQueue;
const osMessageQueueAttr_t Can1RxCbQueue_Attributes = {
    .name = "Can1RxCbQueue"};

/*-任务定义----------------------------------------------------*/
/* CAN1接收回调处理任务 */
static osThreadId_t CAN1_CallbackTaskHandle;
const osThreadAttr_t Can1CallbackTaskHandle_Attributes = {
    .name = "Can1CbTask",
    .priority = (osPriority_t)osPriorityHigh7,
    .stack_size = 128 * 4};

/*-CAN回调任务函数-----------------------------------------------*/
void CAN1_CallbackTask(void *argument)
{
    for (;;)
    {
        static CAN_ConnMessage msg;
        osMessageQueueGet(CAN1_RxCbQueue, &msg, NULL, osWaitForever);
        Debug("CAN1: Callback [%x]", msg.id);
        CAN_Callback cb;
        if (NULL != (cb = (CAN_Callback)HashTable_get(CAN1_CallbackHashTable, (void const *)msg.id)))
            cb(&msg);
    }
}
#endif
#if defined(CAN2_ENABLED) && USE_OSLIB_CAN2_CALLBACK
static HashTable CAN2_CallbackHashTable; // ID-回调函数哈希表
/*-消息队列定义--------------------------------------------------*/
/* CAN2接收回调队列 */
osMessageQueueId_t CAN2_RxCbQueue;
const osMessageQueueAttr_t Can2RxCbQueue_Attributes = {
    .name = "Can2RxCbQueue"};

/*-任务定义----------------------------------------------------*/
/* CAN2接收回调处理任务 */
static osThreadId_t CAN2_CallbackTaskHandle;
const osThreadAttr_t Can2CallbackTaskHandle_Attributes = {
    .name = "Can2CbTask",
    .priority = (osPriority_t)osPriorityHigh7,
    .stack_size = 128 * 4};

/*-CAN回调任务函数-----------------------------------------------*/
void CAN2_CallbackTask(void *argument)
{
    for (;;)
    {
        static CAN_ConnMessage msg;
        osMessageQueueGet(CAN2_RxCbQueue, &msg, NULL, osWaitForever);
        Debug("CAN2: Callback [%x]", msg.id);
        CAN_Callback cb;
        if (NULL != (cb = (CAN_Callback)HashTable_get(CAN2_CallbackHashTable, (void const *)msg.id)))
            cb(&msg);
    }
}
#endif

/*-过滤器配置函数-----------------------------------------*/
#define FilterID(id, idtype) ((idtype == CAN_ID_STD) ? ((id << (18 + 3)) | CAN_RTR_DATA | CAN_ID_STD) : ((id << 3) | CAN_RTR_DATA | CAN_ID_EXT))
#define VescFilterMask (~(0x1F << (8 + 3))) & 0xFFFFFFFE // 0x1F代表不参与过滤的5位
//TODO: 针对标准帧ID采用16位列表方式过滤
/**
 * @brief 以32bit列表方式配置筛选器组
 * 
 * @param hcan 待配置CAN的句柄指针
 * @param fifo CAN_FilterFIFO0/CAN_FilterFIFO1
 * @param bank 筛选器组编号, 范围为0~13
 * @param fid1 筛选器组接受第一个ID
 * @param fid2 筛选器组接受第二个ID
 */
static void OSLIB_CAN_ListFilterConfig(CAN_HandleTypeDef *hcan, uint8_t fifo, uint32_t bank, uint32_t fid1, uint32_t fid2)
{
#if defined(CAN2_ENABLED)
    if (hcan->Instance == CAN2)
        bank += 14;
#endif
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    sFilterConfig.FilterBank = bank;
    sFilterConfig.FilterFIFOAssignment = fifo;
    sFilterConfig.FilterIdHigh = (uint16_t)((fid1 >> 16) & 0xFFFF);
    sFilterConfig.FilterIdLow = (uint16_t)((fid1)&0xFFFF);
    sFilterConfig.FilterMaskIdHigh = (uint16_t)((fid2 >> 16) & 0xFFFF);
    sFilterConfig.FilterMaskIdLow = (uint16_t)((fid2)&0xFFFF);

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
        Error_Handler();
}

/**
 * @brief 以32bit掩码方式配置筛选器组
 * 
 * @param hcan 待配置CAN的句柄指针
 * @param fifo CAN_FilterFIFO0/CAN_FilterFIFO1
 * @param bank 筛选器组编号, 范围为0~13
 * @param id 筛选器组中的参考ID
 * @param mask 筛选器组中的掩码
 */
static void OSLIB_CAN_MaskFilterConfig(CAN_HandleTypeDef *hcan, uint8_t fifo, uint32_t bank, uint32_t id, uint32_t mask)
{
#if defined(CAN2_ENABLED)
    if (hcan->Instance == CAN2)
        bank += 14;
#endif
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    sFilterConfig.FilterBank = bank;
    sFilterConfig.FilterFIFOAssignment = fifo;
    sFilterConfig.FilterIdHigh = (uint16_t)((id >> 16) & 0xFFFF);
    sFilterConfig.FilterIdLow = (uint16_t)((id)&0xFFFF);
    sFilterConfig.FilterMaskIdHigh = (uint16_t)((mask >> 16) & 0xFFFF);
    sFilterConfig.FilterMaskIdLow = (uint16_t)((mask)&0xFFFF);

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
        Error_Handler();
}

/**
 * @brief 为某个CAN配置分发器
 * 
 * @param hcan 待配置CAN的句柄指针
 */
void OSLIB_CAN_Dispatch_Init(CAN_HandleTypeDef *hcan)
{
#if defined(CAN1_ENABLED)
    if (hcan->Instance == CAN1)
    {
        CAN1_QueueHashTable = HashTable_create(NULL, NULL, NULL);
#if USE_OSLIB_CAN1_CALLBACK
        CAN1_CallbackHashTable = HashTable_create(NULL, NULL, NULL);
#endif 
        int bank = 0;
        CAN_IDRecord_t *store0 = NULL;
        CAN_IDRecord_t *store1 = NULL;
#if USE_OSLIB_CAN1_VESC
        for (int i = 0; i < CAN1_VescSize; i++)
        {
            CAN_IDRecord_t *item = &CAN1_VescList[i];
            if (item->queue == NULL || item->idtype == CAN_IDTYPE_STD)
            {
                Error("Wrong Format in CAN1_VescList[%d]", i);
                continue;
            }
            HashTable_insert(CAN1_QueueHashTable, (void const *)item->id, item->queue);
            OSLIB_CAN_MaskFilterConfig(hcan, CAN_FilterFIFO1, bank, FilterID(item->id, item->idtype), VescFilterMask);
            Debug("CAN1: FIFO1 [V%x]", item->id);
            bank += 1;
        }
#endif
        for (int i = 0; i < CAN1_NormSize; i++)
        {
            CAN_IDRecord_t *item = &CAN1_NormList[i];
#if USE_OSLIB_CAN1_CALLBACK
            if (item->queue == NULL)
                item->queue = &CAN1_RxCbQueue;
            if (item->queue == &CAN1_RxCbQueue && item->callback == NULL)
            {
                Error("Wrong Format in CAN1_NormList[%d]", i);
                continue;
            }
#endif
            if (item->queue != NULL) 
            {
                HashTable_insert(CAN1_QueueHashTable, (void const *)item->id, item->queue);

#if USE_OSLIB_CAN1_CALLBACK
                if (item->queue == &CAN1_RxCbQueue)
                    HashTable_insert(CAN1_CallbackHashTable, (void const *)item->id, (void *)item->callback);
#endif
                if (item->idtype == CAN_IDTYPE_STD) // FIFO0
                {
                    if (store0 != NULL)
                    {
                        uint32_t listid1 = FilterID(store0->id, store0->idtype);
                        uint32_t listid2 = FilterID(item->id, item->idtype);
                        OSLIB_CAN_ListFilterConfig(hcan, CAN_FilterFIFO0, bank, listid1, listid2);
                        Debug("CAN1: FIFO0 [%x] [%x]", store0->id, item->id);
                        store0 = NULL;
                        bank += 1;
                    }
                    else
                    {
                        store0 = item;
                    }
                }
                else // FIFO1
                {
                    if (store1 != NULL)
                    {
                        uint32_t listid1 = FilterID(store1->id, store1->idtype);
                        uint32_t listid2 = FilterID(item->id, item->idtype);
                        OSLIB_CAN_ListFilterConfig(hcan, CAN_FilterFIFO1, bank, listid1, listid2);
                        Debug("CAN1: FIFO1 [%x] [%x]", store1->id, item->id);
                        store1 = NULL;
                        bank += 1;
                    }
                    else
                    {
                        store1 = item;
                    }
                }
            }
        }
        if (store0 != NULL)
        {
            uint32_t listid = FilterID(store0->id, store0->idtype);
            OSLIB_CAN_ListFilterConfig(hcan, CAN_FilterFIFO0, bank, listid, listid);
            Debug("CAN1: FIFO0 [%x]", store0->id);
            store0 = NULL;
            bank += 1;
        }
        if (store1 != NULL)
        {
            uint32_t listid = FilterID(store1->id, store1->idtype);
            OSLIB_CAN_ListFilterConfig(hcan, CAN_FilterFIFO1, bank, listid, listid);
            Debug("CAN1: FIFO1 [%x]", store1->id);
            store1 = NULL;
            bank += 1;
        }
#if USE_OSLIB_CAN1_CALLBACK
        CAN1_RxCbQueue = osMessageQueueNew(16, sizeof(CAN_ConnMessage), &Can1RxCbQueue_Attributes);
        CAN1_CallbackTaskHandle = osThreadNew(CAN1_CallbackTask, NULL, &Can1CallbackTaskHandle_Attributes);
        UNUSED(CAN1_CallbackTaskHandle);
#endif
    }
    else
#endif
#if defined(CAN2_ENABLED)
        if (hcan->Instance == CAN2)
    {
        CAN2_QueueHashTable = HashTable_create(NULL, NULL, NULL);
#if USE_OSLIB_CAN2_CALLBACK
        CAN2_CallbackHashTable = HashTable_create(NULL, NULL, NULL);
#endif
        int bank = 0;
        CAN_IDRecord_t *store0 = NULL;
        CAN_IDRecord_t *store1 = NULL;
#if USE_OSLIB_CAN2_VESC
        for (int i = 0; i < CAN2_VescSize; i++)
        {
            CAN_IDRecord_t *item = &CAN2_VescList[i];
            if (item->queue == NULL || item->idtype == CAN_IDTYPE_STD)
            {
                Error("Wrong Format in CAN2_VescList[%d]", i);
                continue;
            }
            HashTable_insert(CAN2_QueueHashTable, (void const *)item->id, item->queue);
            OSLIB_CAN_MaskFilterConfig(hcan, CAN_FilterFIFO1, bank, FilterID(item->id, item->idtype), VescFilterMask);
            Debug("CAN2: FIFO1 [V%x]", item->id);
            bank += 1;
        }
#endif
        for (int i = 0; i < CAN2_NormSize; i++)
        {
            CAN_IDRecord_t *item = &CAN2_NormList[i];
#if USE_OSLIB_CAN2_CALLBACK
            if (item->queue == NULL)
                item->queue = &CAN2_RxCbQueue;
            if (item->queue == &CAN2_RxCbQueue && item->callback == NULL)
            {
                Error("Wrong Format in CAN2_NormList[%d]", i);
                continue;
            }
#endif
            if (item->queue != NULL) 
            {
                HashTable_insert(CAN2_QueueHashTable, (void const *)item->id, item->queue);
#if USE_OSLIB_CAN2_CALLBACK
                if (item->queue == &CAN2_RxCbQueue)
                    HashTable_insert(CAN2_CallbackHashTable, (void const *)item->id, (void *)item->callback);
#endif
                if (item->idtype == CAN_IDTYPE_STD) // FIFO0
                {
                    if (store0 != NULL)
                    {
                        uint32_t listid1 = FilterID(store0->id, store0->idtype);
                        uint32_t listid2 = FilterID(item->id, item->idtype);
                        OSLIB_CAN_ListFilterConfig(hcan, CAN_FilterFIFO0, bank, listid1, listid2);
                        Debug("CAN2: FIFO0 [%x] [%x]", store0->id, item->id);
                        store0 = NULL;
                        bank += 1;
                    }
                    else
                    {
                        store0 = item;
                    }
                }
                else // FIFO1
                {
                    if (store1 != NULL)
                    {
                        uint32_t listid1 = FilterID(store1->id, store1->idtype);
                        uint32_t listid2 = FilterID(item->id, item->idtype);
                        OSLIB_CAN_ListFilterConfig(hcan, CAN_FilterFIFO1, bank, listid1, listid2);
                        Debug("CAN2: FIFO1 [%x] [%x]", store1->id, item->id);
                        store1 = NULL;
                        bank += 1;
                    }
                    else
                    {
                        store1 = item;
                    }
                }
            }
        }

        if (store0 != NULL)
        {
            uint32_t listid = FilterID(store0->id, store0->idtype);
            OSLIB_CAN_ListFilterConfig(hcan, CAN_FilterFIFO0, bank, listid, listid);
            Debug("CAN2: FIFO0 [%x]", store0->id);
            store0 = NULL;
            bank += 1;
        }
        if (store1 != NULL)
        {
            uint32_t listid = FilterID(store1->id, store1->idtype);
            OSLIB_CAN_ListFilterConfig(hcan, CAN_FilterFIFO1, bank, listid, listid);
            Debug("CAN2: FIFO1 [%x]", store1->id);
            store1 = NULL;
            bank += 1;
        }

#if USE_OSLIB_CAN2_CALLBACK
        CAN2_RxCbQueue = osMessageQueueNew(16, sizeof(CAN_ConnMessage), &Can2RxCbQueue_Attributes);
        CAN2_CallbackTaskHandle = osThreadNew(CAN2_CallbackTask, NULL, &Can2CallbackTaskHandle_Attributes);
        UNUSED(CAN2_CallbackTaskHandle);
#endif
    }
    else
#endif
        Error("Unknown CAN");
}
#endif