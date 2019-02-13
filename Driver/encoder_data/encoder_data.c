#include "encoder_data.h"


u8 LeftWheelFirstMes = 1,RightWheelFirstMes = 1;


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
			//printf("LeftWheelFirstMes  \r\n");
		}
		else
		{
			if(L_ForwardReverse == Forward) //向前
			{
				LeftWhellCount++;
				//printf("LeftWheelCurrentCount ++\r\n");
				//printf("LeftWheelCurrentCount %d \r\n",LeftWhellCount);
			}
			else if(L_ForwardReverse == Reverse) //向后
			{
				LeftWhellCount--;
				//printf("LeftWheelCurrentCount --\r\n");
				//printf("LeftWheelCurrentCount %d \r\n",LeftWhellCount);
			}
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line10);//清除中断
}

void EXTI4_IRQHandler(void)//右轮计数
{
	if(RightWheelFirstMes == 1)
	{
		RightWheelFirstMes = 0;
		//printf("RightWheelFirstMes  \r\n");
	}
	else
	{
		if(R_ForwardReverse == Forward) //向前
		{
			RightWhellCount++;
			//printf("RightWheelCurrentCount ++\r\n");
			//printf("RightWheelCurrentCount %d \r\n",RightWhellCount);
		}
		else if(R_ForwardReverse == Reverse) //向后
		{
			RightWhellCount--;
			//printf("RightWheelCurrentCount --\r\n");
			//printf("RightWheelCurrentCount %d \r\n",RightWhellCount);
		}

	}
	EXTI_ClearITPendingBit(EXTI_Line4);//清除中断	
}


s32 readEncoder(u8 LeftOrRight)
{
	s32 encVal =0;
	if(LeftOrRight == LEFT)
	{
		IO_EXIT_LeftWheelSpeed_DE(ITDISABLE);//关闭中断
		encVal = LeftWhellCount;
		LeftWhellCount = 0;//清除左电机周期计数编码值
		IO_EXIT_LeftWheelSpeed_DE(ITENABLE);//打开中断
	}
	else if(LeftOrRight == RIGHT)
	{
		IO_EXIT_RightWheelSpeed_DE(ITDISABLE);//关闭中断
		encVal = RightWhellCount;
		RightWhellCount = 0;//清除右电机周期计数编码值
		IO_EXIT_RightWheelSpeed_DE(ITENABLE);//打开中断
	}
	return encVal;
}

