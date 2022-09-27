/*******************************************************************************
Copyright:      Bupt
File name:      laser.c
Description:    激光
Author:         20th
Version：       1.0
Data:           2019/12/9
*******************************************************************************/
#include "laser.h"
// #include "cmd.h"
// #include "base_chassis.h"
// TODO :区分三个激光分别对应adc的第几个通道
// left对应AD0, right对应AD1, sideLeft对应AD2, sideRight对应AD3
#define ERROR_ON_LEFT 0 //三个激光与实际的偏差
#define ERROR_ON_RIGHT 0
#define ERROR_ON_SIDE 0
#define DIS_BTW_LR 0.45

extern PID_s laser_ypos_pid;
extern PID_s laser_xpos_pid;

float DISTANCE_LASERHL_AND_LASERHR = 0.279; // 单位m

LASER laser_left;
LASER laser_side_left;
LASER laser_right;
LASER laser_side_right;

uint32_t laser_adc_1[2 * AVERAGE_AMOUNT];
uint32_t laser_adc_2[AVERAGE_AMOUNT];
uint32_t laser_adc_3[AVERAGE_AMOUNT];

PID_s laser_ypos_pid = {200, 50, 0, 0, 0, 0, 0, 0.005};
PID_s laser_xpos_pid = {700, 50, 0, 0, 0, 0, 0, 0.005};

Kalman_s kal_distance_L = {1, 0, 0.01, 37.1160, 0, 1};
Kalman_s kal_distance_R = {1, 0, 0.01, 37.1160, 0, 1};
Kalman_s kal_distance_S = {1, 0, 0.01, 37.1160, 0, 1};
Kalman_s kal_distance_SR = {1, 0, 0.01, 37.1160, 0, 1};

Kalman_s kal_adc_L = {1, 0, 0.01, 37.1160, 0, 1};
Kalman_s kal_adc_R = {1, 0, 0.01, 37.1160, 0, 1};
Kalman_s kal_adc_S = {1, 0, 0.01, 37.1160, 0, 1};
Kalman_s kal_adc_SR = {1, 0, 0.01, 37.1160, 0, 1};

Kalman_s kal_chassis_x = {1, 0, 0.01, 37.1160, 0, 1};
Kalman_s kal_chassis_y = {1, 0, 0.01, 37.1160, 0, 1};

/**激光线性方程参数*/
void laser_calculate_kb(LASER *sensor)
{
  sensor->k_param = (sensor->FAR_distance - sensor->NEAR_distance) / (sensor->FAR_voltage - sensor->NEAR_voltage);
  sensor->b_param = sensor->FAR_distance - sensor->k_param * sensor->FAR_voltage;
  // uprintf("k is %f and b is %f\r\n", sensor->k_param, sensor->b_param);
}

/**激光ADC三个通道分别取数*/
// void laser_adc_split(LASER *laser_l, LASER *laser_r, LASER *laser_s)
//{
//
//   for (int i = 0; i < AVERAGE_AMOUNT; i++)
//   {
//     laser_l->ADC_value[i] = laser_adc[i].adc_l;
//     laser_r->ADC_value[i] = laser_adc[i].adc_r;
//     laser_s->ADC_value[i] = laser_adc[i].adc_s;
//   }
// }

void laser_adc_split_1(uint32_t *laser_adc, LASER *laser)
{
  memcpy(laser->ADC_value, laser_adc, AVERAGE_AMOUNT * sizeof(uint32_t));
}

void laser_adc_split_2(uint32_t *laser_adc, LASER *laser_1, LASER *laser_2)
{
  for (uint8_t i = 0; i < AVERAGE_AMOUNT; i++)
  {
    laser_1->ADC_value[i] = laser_adc[i * 2];
    laser_2->ADC_value[i] = laser_adc[i * 2 + 1];
  }
}

/**激光计算距离*/
float laser_calculate_distance(LASER *sensor, Kalman_s *kal_laser_distance, Kalman_s *kal_laser_adc)
{
  float sum_up = 0;
  float distance;
  for (int i = 0; i < AVERAGE_AMOUNT; i++)
  {
    sum_up += sensor->ADC_value[i];
  }
  sensor->ADC_final = (int)sum_up / AVERAGE_AMOUNT; // 均值滤波
  // sensor->ADC_final = (int)Kalman_GetOutput(kal_laser_adc, sensor->ADC_final);
  distance = sensor->ADC_final * sensor->k_param + sensor->b_param;
  // distance = Kalman_GetOutput(kal_laser_distance, distance);
  return distance;
}

//激光计算x  y  angle 的方式
/**激光计算x*/
float laser_calculate_x_left()
{
  return (0.17f - laser_side_left.distance);
}
/**激光计算y*/
float laser_calculate_y_left()
{
  float laser_y = laser_left.distance - 0.49;
  return laser_y;
}
/**激光计算x*/
float laser_calculate_x_right()
{
  return -((0.17f - laser_side_right.distance));
}
/**激光计算y*/
float laser_calculate_y_right()
{
  float laser_y = laser_right.distance - 0.49;
  return laser_y;
}

/**激光计算角度*/
float laser_calculate_angle()
{
  // float laser_offset = laser_right.distance - laser_left.distance;
  // float laser_angle = atan(laser_offset / DISTANCE_LASERHL_AND_LASERHR);
  // return laser_angle;
  // 2020/10/17 根据新的挡板，跑准x和y后偏航角也就跑好了
  return 0;
}

/**激光初始化:计算kb;打开DMA*/
void laser_init()
{

  // TODO 待校准
  // number1
  laser_left.FAR_distance = 4.0;
  laser_left.FAR_voltage = 1885;
  laser_left.NEAR_distance = 0.03;
  laser_left.NEAR_voltage = 470;
  // number 2
  laser_right.NEAR_voltage = 555;
  laser_right.NEAR_distance = 0.1;
  laser_right.FAR_distance = 0.5;
  laser_right.FAR_voltage = 2660;
  // number 3
  laser_side_left.NEAR_voltage = 255;
  laser_side_left.NEAR_distance = 0.03;
  laser_side_left.FAR_distance = 4.0;
  laser_side_left.FAR_voltage = 1850;
  // number 4
  laser_side_right.FAR_distance = 2;
  laser_side_right.FAR_voltage = 3741;
  laser_side_right.NEAR_distance = 0.03;
  laser_side_right.NEAR_voltage = 177;

  // uprintf("Left---");
  laser_calculate_kb(&laser_left);
  // uprintf("Right---");
  laser_calculate_kb(&laser_right);
  // uprintf("Side---");
  laser_calculate_kb(&laser_side_left);

  laser_calculate_kb(&laser_side_right);

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)laser_adc_1, 2 * AVERAGE_AMOUNT) != HAL_OK)
  {
    uprintf("\r\n##Laser DMA Wrong!##\r\n");
    while (1)
    {
    }
  }
  if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)laser_adc_2, AVERAGE_AMOUNT) != HAL_OK)
  {
    uprintf("\r\n##Laser DMA Wrong!##\r\n");
    while (1)
    {
    }
  }
  if (HAL_ADC_Start_DMA(&hadc3, (uint32_t *)laser_adc_3, AVERAGE_AMOUNT) != HAL_OK)
  {
    uprintf("\r\n##Laser DMA Wrong!##\r\n");
    while (1)
    {
    }
  }
  // uprintf("Laser DMA Success!");
}

/**激光执行函数: 获取三个激光的距离;计算x y angle*/
void laser_exe()
{
  laser_adc_split_2(laser_adc_1, &laser_left, &laser_right);
  laser_adc_split_1(laser_adc_2, &laser_side_left);
  laser_adc_split_1(laser_adc_3, &laser_side_right);
  // left和right分别时左后和右后的激光
  laser_left.distance = laser_calculate_distance(&laser_left, &kal_distance_L, &kal_adc_L) + ERROR_ON_LEFT;
  laser_right.distance = laser_calculate_distance(&laser_right, &kal_distance_R, &kal_adc_R) + ERROR_ON_RIGHT;
  // side是右边左边的激光
  laser_side_left.distance = laser_calculate_distance(&laser_side_left, &kal_distance_S, &kal_adc_S) + ERROR_ON_SIDE;
  laser_side_right.distance = laser_calculate_distance(&laser_side_right, &kal_distance_SR, &kal_adc_SR) + ERROR_ON_SIDE;
  // 坐标系定义为水平向左为x轴正方向，垂直向上为y轴正方向
  // BaseChassis.PostureStatus.laser_pos_yaw = laser_calculate_angle(); // 需要首先计算偏航角以供x和y换算，是与x轴的夹角
  // BaseChassis.PostureStatus.laser_pos_x = laser_calculate_x();

  // BaseChassis.PostureStatus.laser_pos_y = laser_calculate_y();
}

/**激光打印三个距离*/
void laser_print_distance()
{
  /// uprintf("right :%6fm\r\n", laser_right.distance);
  // uprintf("left : %6fm\r\n" , laser_left.distance);
  // uprintf("side_left :%6fm\r\n", laser_side_left.distance);
  uprintf("side_right : %6fm\r\n", laser_side_right.distance);
}

int Laser_PrintPos_Flag = 0;
/**激光打印pos*/
// void Laser_PrintPos()
//{
//   if (!Laser_PrintPos_Flag)
//   {
//     return;
//   }
//   // uprintf("--Laser pos:\r\n");
//   uprintf("  Laser_X:%6fm", BaseChassis.PostureStatus.laser_pos_x);
//   uprintf("  Laser_Y:%6fm", BaseChassis.PostureStatus.laser_pos_y);
//   uprintf("  Laser_Angle:%6f\r\n", BaseChassis.PostureStatus.laser_pos_yaw);
// }

int Laser_PrintADCValue_Flag = 0;
void Laser_PrintADCValue()
{
  if (!Laser_PrintADCValue_Flag)
  {
    return;
  }
  uprintf("  left:");
  uprintf("%d ", laser_left.ADC_final);
  // uprintf("\r\n");
  uprintf("  right:");
  uprintf("%d ", laser_right.ADC_final);
  uprintf("  side_right:%d\r\n", laser_side_right.ADC_final);
  uprintf("side_left:%d ", laser_side_left.ADC_final);
  uprintf("\r\n");
  // uprintf("--adc1 value: %d \r\n", HAL_ADC_GetValue(&hadc1));
}
void laser_print_adc()
{
  for (int i = 0; i < 10; i++)
  {
    uprintf("%d %d %d %d\r\n", laser_left.ADC_value[i], laser_right.ADC_value[i], laser_side_left.ADC_value[i], laser_side_right.ADC_value[i]);
  }
}