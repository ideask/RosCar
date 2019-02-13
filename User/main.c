#include "includes.h"

unsigned char  DataStr[]=__DATE__;
unsigned char  TimeStr[]=__TIME__;

union recieveData  //接收到的数据
{
	float d;    //左右轮速度
	unsigned char data[4];
}leftdata,rightdata;       //接收的左右轮数据

union odometry  //里程计数据共用体
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}x_data,y_data,theta_data,vel_linear,vel_angular;     //要发布的里程计数据，分别为：X，Y方向移动的距离，当前角度，线速度，角速度


int main(void)
{
	u8 t=0;
	u8 i=0,j=0,m=0;

	//系统时钟初始化配置
	SystemInit();
	UartInit(115200);
	printf("%s_%s_%d_%d \r\n", __DATE__, __TIME__, sizeof(__DATE__),sizeof(__TIME__));
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
	IO_EXIT_LeftWheelSpeedInit();
	IO_EXIT_RightWheelSpeedInit();
	//周期0.05ms   频率20Khz
	MotoControlTimInit(3199,0);
	TIM3_SpeedCalculate_Init();//速度计算定时器初始化
	//TIM1_OdomUpdate_Init();//里程计发布定时器初始化
	//配置全局中断优先级
	NVIC_Configuration();
	
	while(1)
	{
		if(main_sta&0x01)//执行发送里程计数据步骤
		{
			//里程计数据获取
			x_data.odoemtry_float=position_x;//单位mm
			y_data.odoemtry_float=position_y;//单位mm
			theta_data.odoemtry_float=oriention;//单位rad
			vel_linear.odoemtry_float=velocity_linear;//单位mm/s
			vel_angular.odoemtry_float=velocity_angular;//单位rad/s

			//将所有里程计数据存到要发送的数组
			for(j=0;j<4;j++)
			{
			odometry_data[j]=x_data.odometry_char[j];
			odometry_data[j+4]=y_data.odometry_char[j];
			odometry_data[j+8]=theta_data.odometry_char[j];
			odometry_data[j+12]=vel_linear.odometry_char[j];
			odometry_data[j+16]=vel_angular.odometry_char[j];
			}

			odometry_data[20]='\n';//添加结束符

			//发送数据要串口
			for(i=0;i<21;i++)
			{
				USART_ClearFlag(USART1,USART_FLAG_TC);	//在发送第一个数据前加此句，解决第一个数据不能正常发送的问题				
				USART_SendData(USART1,odometry_data[i]);//发送一个字节到串口	
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); //等待发送结束			
			}

			main_sta&=0xFE;//执行计算里程计数据步骤
		}
		if(main_sta&0x02)//执行计算里程计数据步骤
		{
			odometry(Milemeter_R_Motor,Milemeter_L_Motor);//计算里程计

			main_sta&=0xFD;//执行发送里程计数据步骤
		} 
		if(main_sta&0x08)		 //当发送指令没有正确接收时
		{
			USART_ClearFlag(USART1,USART_FLAG_TC);	//在发送第一个数据前加此句，解决第一个数据不能正常发送的问题
			for(m=0;m<3;m++)
			{
				USART_SendData(USART1,0x00);	
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
			}		
			USART_SendData(USART1,'\n');	
			while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); 
			main_sta&=0xF7;
		}
		if(USART_RX_STA&0x8000)  // 串口1接收函数
		{			
		//接收左右轮速度
			for(t=0;t<4;t++)
			{
				rightdata.data[t]=USART_RX_BUF[t];
				leftdata.data[t]=USART_RX_BUF[t+4];
			}

			//储存左右轮速度
			RecRightWheelSpeed=rightdata.d;//单位mm/s
			RecLeftWheelSpeed=leftdata.d;//单位mm/s

			USART_RX_STA=0;//清楚接收标志位
		}

		//car_control(rightdata.d,leftdata.d);	 //将接收到的左右轮速度赋给小车
		car_control(-500,-500);
	}
}
