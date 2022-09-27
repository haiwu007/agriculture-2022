/*
 * @Author       : Zybz
 * @Date         : 2022-07-10 11:19:54
 * @LastEditors  : Zybz
 * @LastEditTime : 2022-07-10 11:47:29
 * @FilePath     : \robocon-r2_oslib\oslib\peripherals\laser.h
 * @Description  :
 *
 * Copyright (c) 2022 by BUPT RobotTeam, All Rights Reserved.
 */
#ifndef __LASER_H
#define __LASER_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "adc.h"
#include "oslib.h"
#include "string.h"
/*Define Area*/

#define AVERAGE_AMOUNT 10

/*Struct Area*/

typedef struct
{
  uint32_t adc_l;
  uint32_t adc_r;
  uint32_t adc_s;
} LASER_ADC;
typedef struct
{
  uint32_t ADC_value[AVERAGE_AMOUNT];
  int ADC_final;
  float distance;
  float FAR_distance;
  float NEAR_distance;
  float FAR_voltage;
  float NEAR_voltage;
  float k_param;
  float b_param;
} LASER;
/*Variable Area*/

extern LASER laser_left;
extern LASER laser_side_left;
extern LASER laser_right;
extern LASER laser_side_right;

extern LASER_ADC laser_adc[AVERAGE_AMOUNT];

// extern PID_s laser_ypos_pid;
// extern PID_s laser_xpos_pid;

extern int Laser_PrintADCValue_Flag;
extern int Laser_PrintPos_Flag;

/*Function Area*/

void laser_calculate_kb(LASER *sensor);
void laser_adc_split(LASER *laser_l, LASER *laser_r, LASER *laser_s);

float laser_calculate_distance(LASER *sensor, Kalman_s *kal_laser_distance, Kalman_s *kal_laser_adc);
float laser_calculate_x_left();
float laser_calculate_y_left();
float laser_calculate_x_right();
float laser_calculate_y_right();
float laser_calculate_angle();

void laser_init();
void laser_exe();
void laser_print_distance();
void Laser_PrintPos();
void Laser_PrintADCValue();

#ifdef __cplusplus
}
#endif
#endif /*__LASER_H */