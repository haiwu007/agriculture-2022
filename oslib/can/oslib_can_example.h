/**
 * @file oslib_can_example.h
 * @author KYzhang
 * @brief OSLIB-CAN接收处理任务范例
 * @version 0.2
 * @date 2021-01-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef OSLIB_CAN_EXAMPLE_H
#define OSLIB_CAN_EXAMPLE_H

#include "cmsis_os2.h"
#include "oslib_can.h"

#if defined(OSLIB_CAN_MODULE_ENABLED)
extern osMessageQueueId_t ExampleCanTaskQueue;
extern void CAN_Example_Init();
#endif

#endif