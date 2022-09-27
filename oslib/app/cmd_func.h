#ifndef __CMD_FUNC_H
#define __CMD_FUNC_H

#include "oslib_uart_cmd.h"

#if defined(OSLIB_UART_MODULE_ENABLED) && USE_OSLIB_UART_CLI

extern void Command_Init();

#endif

#endif