#ifndef DJI2_0
#define DJI2_0

#include "stm32f4xx.h"
#include "main.h"
#include "can_utils.h"
// ELMO驱动器CAN广播ID号
//  #define ELMO_BROADCAST_ID (0x000)
// ELMO驱动器接收ID基址
//  #define ELMO_DEVICE_BASEID (0x200)
// ELMO驱动器发送ID基址
//  #define SDO_RESPONSE_COB_ID_BASE 0x280
//大疆驱动板接收ID基址
#define DJI_BASE_ID (0x200)
/**
  * @brief  电机种类  3508 or 2006
//   */
typedef enum
{
	RM_3508 = 1,
	M_2006 = 2,
	NONE = 3 //none表示没有接电机
} MotorType_TypeDef;
typedef enum
{
	MotorStatus_OFF = 0,
	MotorStatus_POS,
	MotorStatus_VEL,
	MotorStatus_CUR,
	MotorStatus_HOMING,
	MotorStatus_STOP
} RunningStatus;

/**
  * @brief  Robomaster Motor 3508/2006 type structure definition
  * @note
  */
typedef struct
{
	int32_t pos, posLast;
	MotorType_TypeDef type;
	int32_t vel; //电机返回的速度，单位： rpm
	int16_t cur; //电机返回的电流值
	int8_t temp; //电机的温度
} MotorType;

typedef struct RM_MotorStatus_t
{
	float pos; // rad
	float rpm;
	float current;
	uint8_t Status;
} RM_MotorStatus_t; // 配合东大驱动板使用的状态结构体

extern RM_MotorStatus_t RM_MotorStatus[4];
extern MotorType Motor[8];

/**
 * @brief  CAN消息种类
 */
typedef enum
{
	MOTORON = 1,
	MOTOROFF,
	VELCFG,
	POSCFG,
	CURCFG,
	VELCTRL,
	POSCTRL,
	CURCTRL,
	READINFO,
	VELCTRLALL,
	POSCTRLALL,
	HOMING,
	LIMITVELCFG,
	NOINITPOSCFG,
} CANOPTION;

void dji_MotorOn(uint32_t boradId, uint16_t motorId);

void dji_MotorOff(uint32_t boradId, uint16_t motorId);

void dji_VelCfg(uint32_t boradId, uint16_t motorId);

void dji_LimitVelCfg(uint32_t boradId, uint16_t motorId);

void dji_PosCfg(uint32_t boradId, uint16_t motorId, int maxPosVel);

void dji_NoInitPosCfg(uint32_t boradId, uint16_t motorId, int maxPosVel);

void dji_CurCfg(uint32_t boradId, uint16_t motorId);

void dji_VelCtrl(uint32_t boradId, uint16_t motorId, int32_t vel);

void dji_PosCtrl(uint32_t boradId, uint16_t motorId, int32_t pos);

void dji_CurCtrl(uint32_t boradId, uint16_t motorId, int32_t cur);

void dji_VelCtrlAll(uint32_t boradId, int16_t vel[4]);

void dji_PosCtrlAll(uint32_t boradId, int16_t pos[4]);

void dji_ReadInfo(uint32_t boardId, uint16_t motorId);

void dji_Homing(uint32_t boardId, uint16_t motorId, int16_t vel, int16_t cur);

void CAN_Callback_DJIv2_ReadInfo(CAN_ConnMessage_s *data);

void CAN_Callback_DJIv2_PosArrival(CAN_ConnMessage_s *data);

void CAN_Callback_DJIv2_HomingOver(CAN_ConnMessage_s *data);

#endif
