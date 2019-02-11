#ifndef __ENCODER_DATA_H
#define __ENCODER_DATA_H

#include "stm32f10x.h"

void InfraredSendingTubeOnOff(u8 OnOff);
void EXTI15_10_IRQHandler(void);
void EXTI4_IRQHandler(void);



extern u32 LeftWhellCount;
extern u32 RightWhellCount;


#endif
