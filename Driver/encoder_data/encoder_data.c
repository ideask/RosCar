#include "encoder_data.h"
#include "init_function.h"
u8 LeftWheelFirstMes = 1,RightWheelFirstMes = 1;
u32 LeftWhellCount = 0;
u32 RightWhellCount = 0;

void InfraredSendingTubeOnOff(u8 OnOff)
{
	if(OnOff == 1)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_11); 
	}
	else if(OnOff == 0)
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_11); 
	}
}

void EXTI15_10_IRQHandler(void)//左轮计数
{
	if(EXTI_GetITStatus(EXTI_Line10) != RESET)//EXTI10产生了中断
	{
		if(LeftWheelFirstMes == 1)
		{
			LeftWheelFirstMes = 0;
			printf("LeftWheelFirstMes  \r\n");
		}
		else
		{
			LeftWhellCount++;
			printf("LeftWheelCurrentCount ++\r\n");
			printf("LeftWheelCurrentCount %d \r\n",LeftWhellCount);
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line10);//清除中断
}

void EXTI4_IRQHandler(void)//右轮计数
{
	if(RightWheelFirstMes == 1)
	{
		RightWheelFirstMes = 0;
		printf("RightWheelFirstMes  \r\n");
	}
	else
	{
		RightWhellCount++;
		printf("RightWheelCurrentCount ++ \r\n");
		printf("RightWheelCurrentCount %d \r\n",RightWhellCount);
	}
	EXTI_ClearITPendingBit(EXTI_Line4);//清除中断	
}




