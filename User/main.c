#include "stm32f10x.h"
#include "init_function.h"
#include "power_manage.h"
#include "encoder_data.h"




int main(void)
{
	//系统时钟初始化配置
	SystemInit();
	UartInit(115200);
	printf("Kenny Ros car!\r\n");
	//外设电源控制引脚初始化
	IO_PeripheralPowerInit();
	//初始化控制红外测速管开关IO
	IO_InfraredSendingTubeInit();
	//打开外设电源开关
	PeripheralPowerOnOff(1);
	//打开红外测速管开关
	InfraredSendingTubeOnOff(1);
	//打开左右轮编码器中断
	IO_EXIT_LeftWheelSpeed();
	IO_EXIT_RightWheelSpeed();
	while(1)
	{

	}
}
