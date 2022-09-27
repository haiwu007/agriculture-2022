/**
 * @file oslib.h
 * @author KYzhang
 * @brief OSLIB各组件初始化
 * @version 0.2
 * @date 2021-01-04
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef __OSLIB_H
#define __OSLIB_H

#include "oslib_config.h"

#ifdef OSLIB_DEBUG_MODULE_ENABLED
#include "oslib_debug.h"
#endif
#ifdef OSLIB_CAN_MODULE_ENABLED
#include "oslib_can.h"
#endif
#ifdef OSLIB_UART_MODULE_ENABLED
#include "oslib_uart.h"
#endif
/*-接口函数--------------------------------*/
#include "kalman.h"
#include "utils.h"
#include "vec.h"
#include "laser.h"
extern void OSLIB_Init();

#endif