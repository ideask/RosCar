#ifndef __INIT_FUNCTION_H
#define __INIT_FUNCTION_H

#include "includes.h"


//系统时钟初始化配置
void SystemInit (void);
void UartInit(u32 bound);

void IO_PeripheralPowerInit(void);
void IO_InfraredSendingTubeInit(void);
void IO_EXIT_LeftWheelSpeedInit(void);
void IO_EXIT_LeftWheelSpeed_DE(u8 enable_or_disable);
void IO_EXIT_RightWheelSpeedInit(void);
void IO_EXIT_RightWheelSpeed_DE(u8 enable_or_disable);
void TIM1_OdomUpdate_Init(void);
void TIM3_SpeedCalculate_Init(void);
void NVIC_Configuration(void);
void USART1_IRQHandler(void);



#endif
