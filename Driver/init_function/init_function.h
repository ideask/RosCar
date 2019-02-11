#ifndef __INIT_FUNCTION_H
#define __INIT_FUNCTION_H

#include "stm32f10x.h"
#include "stdio.h"	

//系统时钟初始化配置
void SystemInit (void);
void UartInit(u32 bound);

void IO_PeripheralPowerInit(void);
void IO_InfraredSendingTubeInit(void);
void IO_EXIT_LeftWheelSpeed(void);
void IO_EXIT_RightWheelSpeed(void);


#endif
