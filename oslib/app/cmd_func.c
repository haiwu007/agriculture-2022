#include "cmd_func.h"
#include "stdlib.h"
#include "../uart/oslib_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tim.h"
#include "dji_boardv2_can.h"
#include "omni_chassis.h"
#include "laser.h"
#ifdef OSLIB_CAN_MODULE_ENABLED
#include "../can/oslib_can.h"
#endif

#if defined(OSLIB_UART_MODULE_ENABLED) && USE_OSLIB_UART_CLI
/* 在这里添加命令回调函数的定义或外部声明 */
extern int case_id;
static void Command_Hello(int argc, char *argv[])
{
    uprintf("1st: %.2f || 2nd: %.2f || 3rd: %.2f || 4th: %.2f\n\r", laser_left.distance, laser_right.distance, laser_side_left.distance, laser_side_right.distance);
    uprintf("  side_right:%d\r\n", laser_side_right.ADC_final);
}

static void Command_Echo(int argc, char *argv[])
{
    for (int i = 1; i < argc; i++)
        uprintf("Echo: %s\r\n", argv[i]);
}
void CMD_getposturexy(int argc, char *argv[])
{
    uprintf("%f , %f\r\n", BaseChassis.PostureStatus.x, BaseChassis.PostureStatus.y);
}
void CMD_CaseID(int argc, char *argv[])
{
    int a = atoi(argv[1]);
    case_id = a;
    uprintf("case_id=%d!\r\n", case_id);
}
void CMD_Chassis_Go2Point(int argc, char **argv)
{
    Chassis_Go2Point_Reset();
    BaseChassis.ctrl_mode = CTRL_MODE_GO_TO_POINT;
    float x = atof(argv[1]);
    float y = atof(argv[2]);
    float yaw = atof(argv[3]);
    float start_speed = atof(argv[4]);
    float final_speed = atof(argv[5]);
    OmniChassis.base->Go2PointStatus.target_point.x = x;
    OmniChassis.base->Go2PointStatus.target_point.y = y;
    OmniChassis.base->Go2PointStatus.target_yaw = yaw;
    OmniChassis.base->Go2PointStatus.start_speed = start_speed;
    OmniChassis.base->Go2PointStatus.final_speed = final_speed;
    OmniChassis.base->Go2PointStatus.enable = 1;
    OmniChassis.base->Go2PointStatus.start = 1;

    uprintf("CMD|Target point set to (%.2f,%.2f,%.2f,%.2f,%.2f)\r\n", x, y, yaw, start_speed, final_speed);
}
int state2006 = 0;
void CMD_dajiang(int argc, char *argv[])
{
    if (state2006 == 0)
    {
        dji_PosCtrl(2, 1, 8196 * 8.8);
        dji_PosCtrl(2, 2, -8196 * 8.8);
        state2006++;
    }
    else if (state2006 == 1)
    {
        dji_PosCtrl(2, 1, 8196 * 17.6);
        dji_PosCtrl(2, 2, -8196 * 17.6);
        state2006++;
    }
    else if (state2006 == 2)
    {
        dji_PosCtrl(2, 1, 0);
        dji_PosCtrl(2, 2, 0);
        state2006 = 0;
    }
}
#ifdef OSLIB_CAN_MODULE_ENABLED
static void Command_CanSend(int argc, char *argv[])
{
    if (argc >= 3)
    {
        can_msg msg;
        msg.in[0] = 0x40005850;
        msg.in[1] = 0x00000000;
        uint32_t id = strtol(argv[2], NULL, 16);
        if (argv[1][0] == 's')
        {
            can_send_msg(id, &msg);
            uprintf("CanSend:std[%x]\r\n", id);
        }
        else if (argv[1][0] == 'e')
        {
            can_ext_send_msg(id, &msg);
            uprintf("CanSend:ext[%x]\r\n", id);
        }
        else
            uprintf("Param Error!\r\n");
    }
    else
    {
        uprintf("Param Error!\r\n");
    }
}
#endif

static void Command_Task(int argc, char *argv[])
{
    uprintf("Free Heap: %dB\r\n", xPortGetFreeHeapSize());
#if ((configUSE_TRACE_FACILITY == 1) && (configUSE_STATS_FORMATTING_FUNCTIONS > 0) && (configSUPPORT_DYNAMIC_ALLOCATION == 1))
    static char tasklist_buffer[256];
    uprintf("Name          \tState\tPrio\tStack\tRank\r\n");
    vTaskList(tasklist_buffer);
    uprintf(tasklist_buffer);
#endif
    // uprintf("Name          \tCount\tUtilization\r\n");
    // vTaskGetRunTimeStats(tasklist_buffer);
    // uprintf(tasklist_buffer);
}
static void Command_Omni_Chassis_Ctrl(int argc, char *argv[])
{
    if (strcmp(argv[1], "w") == 0) // 前进
    {
        uprintf("go straight\r\n");
        OmniChassis.base->ctrl_mode = CTRL_MODE_CMD;
        CMD_TargetSpeed = 0.2;  // 0.2 -0.2
        CMD_TargetDir = (1.57); // 1.57 0 3.14
        CMD_TargetOmega = 0;    // 0
    }
    else if (strcmp(argv[1], "a") == 0) // 左平移
    {
        uprintf("go left\r\n");
        OmniChassis.base->ctrl_mode = CTRL_MODE_CMD;
        CMD_TargetSpeed = 0.2; // 0.2 -0.2
        CMD_TargetDir = (0);   // 1.57 0 3.14
        CMD_TargetOmega = 0;   // 0
    }
    else if (strcmp(argv[1], "s") == 0) // 后退
    {
        uprintf("go back\r\n");
        OmniChassis.base->ctrl_mode = CTRL_MODE_CMD;
        CMD_TargetSpeed = -0.2; // 0.2 -0.2
        CMD_TargetDir = (1.57); // 1.57 0 3.14
        CMD_TargetOmega = 0;    // 0
    }
    else if (strcmp(argv[1], "d") == 0) // 右平移
    {
        uprintf("go right\r\n");
        OmniChassis.base->ctrl_mode = CTRL_MODE_CMD;
        CMD_TargetSpeed = 0.2;  // 0.2 -0.2
        CMD_TargetDir = (3.14); // 1.57 0 3.14
        CMD_TargetOmega = 0;    // 0
    }
    else if (strcmp(argv[1], "f") == 0) // 停车
    {
        uprintf("stop\r\n");
        OmniChassis.base->ctrl_mode = CTRL_MODE_CMD;
        CMD_TargetSpeed = 0;    // 0.2 -0.2
        CMD_TargetDir = (1.57); // 1.57 0 3.14
        CMD_TargetOmega = 0;    // 0
    }
    else
    {
        OmniChassis.base->ctrl_mode = CTRL_MODE_CMD;
        CMD_TargetSpeed = 0;    // 0.2 -0.2
        CMD_TargetDir = (1.57); // 1.57 0 3.14
        CMD_TargetOmega = 0;    // 0
    }
}

/* 在这里完成命令到回调函数的映射 */
void Command_Init()
{
    OSLIB_UartCmd_AddCommand("hello", "\"Hello SimpleLib!\"", Command_Hello); /* 输出 "Hello SimpleLib!" */ // 测试串口输出和命令
    OSLIB_UartCmd_AddCommand("echo", "Echo message", Command_Echo); /* 回显参数的内容 */                    // 测试命令参数解析
#ifdef OSLIB_CAN_MODULE_ENABLED
    OSLIB_UartCmd_AddCommand("cansend", "CAN send message", Command_CanSend); /* 发送CAN报文, 内容为两个整数 */ // 测试CAN发送
#endif
    OSLIB_UartCmd_AddCommand("tasks", "Show task list", Command_Task);
    OSLIB_UartCmd_AddCommand("caseid", "", CMD_CaseID);
    OSLIB_UartCmd_AddCommand("getposturexy", "", CMD_getposturexy);
    OSLIB_UartCmd_AddCommand("gopoint", "", CMD_Chassis_Go2Point);
    OSLIB_UartCmd_AddCommand("dajiang", "", CMD_dajiang);
    OSLIB_UartCmd_AddCommand("ctrl", "", Command_Omni_Chassis_Ctrl);
}

#endif
