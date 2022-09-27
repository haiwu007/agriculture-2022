/**
 * @file oslib_can.h
 * @author KYzhang
 * @brief OSLIB-CAN发送接收管理
 * @version 0.2
 * @date 2021-01-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __OSLIB_CAN_H
#define __OSLIB_CAN_H

#include "cmsis_os2.h"
#include "oslib_config.h"
#include "can.h"

#ifdef OSLIB_CAN_MODULE_ENABLED

/*-对F103简陋的兼容----------------------------*/

#ifdef STM32F103xB
#define hcan1 hcan
#endif

/*-数据结构-----------------------------------*/

typedef union
{
    char ch[8];
    uint8_t ui8[8];
    uint16_t ui16[4];
    int16_t i16[4];
    int in[2];
    float fl[2];
    double df;
} CAN_Message;

typedef CAN_Message can_msg;

typedef struct
{
    uint32_t id;
    CAN_Message payload;
} CAN_ConnMessage;


/*-接口函数-----------------------------------*/

void OSLIB_CAN_Init();
void OSLIB_CAN_SendMessage(CAN_HandleTypeDef *hcan, uint32_t idtype, uint32_t id, CAN_Message *msg);

/*-兼容定义-----------------------------------*/

#define can_send_msg(id, msg) OSLIB_CAN_SendMessage(&hcan1, CAN_ID_STD, id, msg)
#define can_ext_send_msg(id, msg) OSLIB_CAN_SendMessage(&hcan1, CAN_ID_EXT, id, msg)

#endif

#endif