
#include "oslib_config.h"
#include "oslib_can.h"
#include "can_func.h"
#ifdef OSLIB_UART_MODULE_ENABLED
#include "oslib_uart.h"
#include "base_chassis.h"
#include "handle.h"
#include "dji_boardv2_can.h"
#endif

#if defined(OSLIB_CAN_MODULE_ENABLED)

/*-Declaration--------------------------------------------*/

/* 在这里添加CAN回调函数的定义或外部声明 */
#ifdef OSLIB_UART_MODULE_ENABLED
static void CanCallback_EchoID(CAN_ConnMessage *msg)
{
    uprintf("CAN: Callback[%x]\r\n", msg->id);
}
#endif


/* 在这里添加任务对应消息队列的外部声明 */
#if USE_OSLIB_CAN_EXAMPLE
extern osMessageQueueId_t ExampleCanTaskQueue;
#endif

/*-IDList--------------------------------------------------*/

/* 
 * 以下IDList数组每一项格式为 {<ID>, <ID类型>, <消息队列地址>, <回调函数>}
 * 注意1: <消息队列地址> 和 <回调函数> 两域有且仅有一个有值, 另一个写为NULL
 * 注意2: VescList的ID类型只能为CAN_IDTYPE_EXT, 
 * 注意3: VescList目前只支持任务方式处理, 即 <回调函数> 为NULL, <消息队列地址> 有值
 * 注意4: 两个List中的所有ID不能有冲突, 包括VESC扩展出去的那些ID. 
 *      (比如VescList中有ID[0x58], NormList中有ID[0x158], 会产生冲突.)
 * 注意5: 两个数组中的ID数量有限制, 每个CAN大约要求 2*VescSize+NormSize < 28
 */ 

// #if defined(CAN1_ENABLED)

#if USE_OSLIB_CAN1_VESC
/* 在这里添加对CAN1的VESC的ID和对应消息队列的指针 */
CAN_IDRecord_t CAN1_VescList[] = {
#if USE_OSLIB_CAN_EXAMPLE
    {0x58, CAN_IDTYPE_EXT, &ExampleCanTaskQueue, NULL},
#endif
};

uint32_t CAN1_VescSize = sizeof(CAN1_VescList) / sizeof(CAN_IDRecord_t);
#endif

/* 在这里添加对CAN1的其他ID和对应消息队列的指针或回调函数 */
CAN_IDRecord_t CAN1_NormList[] = {
    {0x1, CAN_IDTYPE_STD, NULL, CanCallback_EchoID},
    {0x9, CAN_IDTYPE_STD, NULL, CanCallback_EchoID},
    {0x2, CAN_IDTYPE_EXT, NULL, CanCallback_EchoID},
    {324, CAN_IDTYPE_STD, NULL, CAN_Callback_Handle_Rocker},
    //(324, CAN_IDTYPE_STD, NULL, CanCallback_EchoID),
    {325, CAN_IDTYPE_STD, NULL, CAN_Callback_Handle_Button},
    {204, CAN_IDTYPE_STD, NULL, CAN_Callback_Locator_ReadPos_X},
    {205, CAN_IDTYPE_STD, NULL, CAN_Callback_Locator_ReadPos_Y},
    {206, CAN_IDTYPE_STD, NULL, CAN_Callback_Locator_ReadPos_Yaw},
    {0x285, CAN_IDTYPE_STD, NULL, CAN_Callback_DJIv2_HomingOver},//接受Homing结束消息
    {0x286, CAN_IDTYPE_STD, NULL, CAN_Callback_DJIv2_PosArrival},//接受位置环到达信息
    {0x281, CAN_IDTYPE_STD, NULL, CAN_Callback_DJIv2_ReadInfo},//接受返回的电机信息
#if USE_OSLIB_CAN_EXAMPLE
    {0x120, CAN_IDTYPE_STD, &ExampleCanTaskQueue, NULL},
#endif
};

uint32_t CAN1_NormSize = sizeof(CAN1_NormList) / sizeof(CAN_IDRecord_t);
// #endif

#if defined(CAN2_ENABLED)
#if USE_OSLIB_CAN2_VESC
/* 在这里添加对CAN2的VESC的ID和对应消息队列的指针 */
CAN_IDRecord_t CAN2_VescList[] = {
#if USE_OSLIB_CAN_EXAMPLE
    {0x15, CAN_IDTYPE_EXT, &ExampleCanTaskQueue, NULL},
#endif
};

uint32_t CAN2_VescSize = sizeof(CAN2_VescList) / sizeof(CAN_IDRecord_t);
#endif

/* 在这里添加对CAN2的其他ID和对应消息队列的指针或回调函数 */
CAN_IDRecord_t CAN2_NormList[] = {
    {0x2, CAN_IDTYPE_STD, NULL, CanCallback_EchoID},
    {0xA, CAN_IDTYPE_STD, NULL, CanCallback_EchoID},
    {0x10, CAN_IDTYPE_EXT, NULL, CanCallback_EchoID},
#if USE_OSLIB_CAN_EXAMPLE
    {0x100, CAN_IDTYPE_STD, &ExampleCanTaskQueue, NULL},
#endif
};

uint32_t CAN2_NormSize = sizeof(CAN2_NormList) / sizeof(CAN_IDRecord_t);
#endif

#endif
