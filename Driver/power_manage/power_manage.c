#include "power_manage.h"



void PeripheralPowerOnOff(u8 OnOff)
{
	if(OnOff == 1)
	{
		GPIO_SetBits(GPIOE,GPIO_Pin_6); 
	}
	else if(OnOff == 0)
	{
		GPIO_ResetBits(GPIOE,GPIO_Pin_6); 
	}
}
				


