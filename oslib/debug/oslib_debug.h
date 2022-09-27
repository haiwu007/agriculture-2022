/**
 * @file oslib_debug.h
 * @author KYzhang
 * @brief OSLIB-Debug功能, 使用Jlink自带的RTT方式输出
 * @version 0.2
 * @date 2021-01-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "oslib_config.h"

#ifdef OSLIB_DEBUG_MODULE_ENABLED
    #include "SEGGER_RTT.h"
    #define Log(fmt, args...) SEGGER_RTT_printf(0, fmt, ##args)
#else
    #define Log(fmt, args...)    do {} while(0)
#endif


#if SELECT_DEBUG_LEVEL == OFF
    #define UDebug(fmt, args...) do {} while(0)
    #define Fatal(fmt, args...)  do {} while(0)
    #define Error(fmt, args...)  do {} while(0)
    #define Warn(fmt, args...)   do {} while(0)
    #define Info(fmt, args...)   do {} while(0)
    #define Debug(fmt, args...)  do {} while(0)
#elif SELECT_DEBUG_LEVEL == FATAL || SELECT_DEBUG_LEVEL == USER
    #define UDebug(fmt, args...) do {Log("[User] ");Log(fmt, ##args);Log("\n");} while(0)
    #define Fatal(fmt, args...)  do {Log("[Fatal]");Log(fmt, ##args);Log("\n");} while(0)
    #define Error(fmt, args...)  do {} while(0)
    #define Warn(fmt, args...)   do {} while(0)
    #define Info(fmt, args...)   do {} while(0)
    #define Debug(fmt, args...)  do {} while(0)
#elif SELECT_DEBUG_LEVEL == ERROR
    #define UDebug(fmt, args...) do {Log("[User] ");Log(fmt, ##args);Log("\n");} while(0)
    #define Fatal(fmt, args...)  do {Log("[Fatal]");Log(fmt, ##args);Log("\n");} while(0)
    #define Error(fmt, args...)  do {Log("[Error]");Log(fmt, ##args);Log("\n");} while(0)
    #define Warn(fmt, args...)   do {} while(0)
    #define Info(fmt, args...)   do {} while(0)
    #define Debug(fmt, args...)  do {} while(0)
#elif SELECT_DEBUG_LEVEL == WARN
    #define UDebug(fmt, args...) do {Log("[User] ");Log(fmt, ##args);Log("\n");} while(0)
    #define Fatal(fmt, args...)  do {Log("[Fatal]");Log(fmt, ##args);Log("\n");} while(0)
    #define Error(fmt, args...)  do {Log("[Error]");Log(fmt, ##args);Log("\n");} while(0)
    #define Warn(fmt, args...)   do {Log("[Warn] ");Log(fmt, ##args);Log("\n");} while(0)
    #define Info(fmt, args...)   do {} while(0)
    #define Debug(fmt, args...)  do {} while(0)
#elif SELECT_DEBUG_LEVEL == INFO
    #define UDebug(fmt, args...) do {Log("[User] ");Log(fmt, ##args);Log("\n");} while(0)
    #define Fatal(fmt, args...)  do {Log("[Fatal]");Log(fmt, ##args);Log("\n");} while(0)
    #define Error(fmt, args...)  do {Log("[Error]");Log(fmt, ##args);Log("\n");} while(0)
    #define Warn(fmt, args...)   do {Log("[Warn] ");Log(fmt, ##args);Log("\n");} while(0)
    #define Info(fmt, args...)   do {Log("[Info] ");Log(fmt, ##args);Log("\n");} while(0)
    #define Debug(fmt, args...)  do {} while(0)
#elif SELECT_DEBUG_LEVEL == DEBUG || SELECT_DEBUG_LEVEL == ALL
    #define UDebug(fmt, args...) do {Log("[User] ");Log(fmt, ##args);Log("\n");} while(0)
    #define Fatal(fmt, args...)  do {Log("[Fatal]");Log(fmt, ##args);Log("\n");} while(0)
    #define Error(fmt, args...)  do {Log("[Error]");Log(fmt, ##args);Log("\n");} while(0)
    #define Warn(fmt, args...)   do {Log("[Warn] ");Log(fmt, ##args);Log("\n");} while(0)
    #define Info(fmt, args...)   do {Log("[Info] ");Log(fmt, ##args);Log("\n");} while(0)
    #define Debug(fmt, args...)  do {Log("[Debug]");Log(fmt, ##args);Log("\n");} while(0)
#else
    #error "[OSLIB] Debug level is wrong"
#endif