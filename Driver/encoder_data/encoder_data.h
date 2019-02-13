#ifndef __ENCODER_DATA_H
#define __ENCODER_DATA_H

#include "includes.h"


void InfraredSendingTubeOnOff(u8 OnOff);
void EXTI15_10_IRQHandler(void);
void EXTI4_IRQHandler(void);
s32 readEncoder(u8 LeftOrRight);






#endif
