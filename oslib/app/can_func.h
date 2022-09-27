#ifndef __CAN_FUNC_H
#define __CAN_FUNC_H

#include "oslib_can_dispatch.h"

#if defined(OSLIB_CAN_MODULE_ENABLED)

#if defined(CAN1_ENABLED)
extern CAN_IDRecord_t CAN1_VescList[];
extern uint32_t CAN1_VescSize;
extern CAN_IDRecord_t CAN1_NormList[];
extern uint32_t CAN1_NormSize;
#endif

#if defined(CAN2_ENABLED)
extern CAN_IDRecord_t CAN2_VescList[];
extern uint32_t CAN2_VescSize;
extern CAN_IDRecord_t CAN2_NormList[];
extern uint32_t CAN2_NormSize;
#endif

#endif

#endif