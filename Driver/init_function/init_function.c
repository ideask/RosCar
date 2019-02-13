#include "init_function.h"



void SystemInit (void)
{
	__IO uint32_t HSIStartUpStatus = 0;
	// 把 RCC 外设初始化成复位状态，这句是必须的
	RCC_DeInit();

	//使能 HSI
	RCC_HSICmd(ENABLE);

	// 等待 HSI 就绪
	HSIStartUpStatus = RCC->CR & RCC_CR_HSIRDY;

	// 只有 HSI 就绪之后则继续往下执行
	if (HSIStartUpStatus == RCC_CR_HSIRDY) 
	{
		//-------------------------------------------------------------//

		// 使能 FLASH 预存取缓冲区
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		// SYSCLK 周期与闪存访问时间的比例设置，这里统一设置成 2
		// 设置成 2 的时候，SYSCLK 低于 48M 也可以工作，如果设置成 0 或者 1 的时候，
		// 如果配置的 SYSCLK 超出了范围的话，则会进入硬件错误，程序就死了
		// 0：0 < SYSCLK <= 24M
		// 1：24< SYSCLK <= 48M
		// 2：48< SYSCLK <= 72M
		FLASH_SetLatency(FLASH_Latency_2);
		//------------------------------------------------------------//

		// AHB 预分频因子设置为 1 分频，HCLK = SYSCLK
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		// APB2 预分频因子设置为 1 分频，PCLK2 = HCLK
		RCC_PCLK2Config(RCC_HCLK_Div1);

		// APB1 预分频因子设置为 1 分频，PCLK1 = HCLK/2
		RCC_PCLK1Config(RCC_HCLK_Div2);

		//-----------设置各种频率主要就是在这里设置-------------------//
		// 设置 PLL 时钟来源为 HSE，设置 PLL 倍频因子
		// PLLCLK = 4MHz * 16
		RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);
		//-- -----------------------------------------------------//

		// 开启 PLL
		RCC_PLLCmd(ENABLE);

		// 等待 PLL 稳定
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}

		// 当 PLL 稳定之后，把 PLL 时钟切换为系统时钟 SYSCLK
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		// 读取时钟切换状态位，确保 PLLCLK 被选为系统时钟
		while (RCC_GetSYSCLKSource() != 0x08) {}
	} 
	else 
	{
		// 如果 HSI 开启失败，那么程序就会来到这里，用户可在这里添加出错的代码处理
		// 当 HSE 开启失败或者故障的时候，单片机会自动把 HSI 设置为系统时钟，
		// HSI 是内部的高速时钟，8MHZ
		while (1) {}
	}
}


//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
	//标准库需要的支持函数                 
	struct __FILE 
	{ 
		int handle; 

	}; 

	FILE __stdout;       
	//定义_sys_exit()以避免使用半主机模式    
	_sys_exit(int x) 
	{ 
		x = x; 
	} 
	//重定义fputc函数 
	int fputc(int ch, FILE *f)
	{      
		while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	    USART1->DR = (u8) ch;      
		return ch;
	}
#endif 

void UartInit(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟

	//USART1_TX   GPIOA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9

	//USART1_RX	  GPIOA.10初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

	//USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(USART1, ENABLE);                    //使能串口1 

}


void IO_PeripheralPowerInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;			
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void IO_InfraredSendingTubeInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;			
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
}

void IO_EXIT_LeftWheelSpeedInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	//1.使能GPIOD和AFIO时钟,值得注意的是，当使用外部中断的时候必须使能AFIO时钟。
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO,ENABLE);
	//2.GPIO初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO速度为50MHz
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	//3.设置EXTI线
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource10);//将EXIT线10连接到PD10
	EXTI_InitStructure.EXTI_Line = EXTI_Line10;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//上升下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能中断线
	EXTI_Init(&EXTI_InitStructure);//初始化中断
#if 0

	//4.中断向量
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
}

void IO_EXIT_LeftWheelSpeed_DE(u8 enable_or_disable)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	if(enable_or_disable == ITENABLE)
	{
		NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	
	}
	else if(enable_or_disable == ITDISABLE)
	{
		NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
		NVIC_Init(&NVIC_InitStructure);			
	}
}
void IO_EXIT_RightWheelSpeedInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	//1.使能GPIOE和AFIO时钟,值得注意的是，当使用外部中断的时候必须使能AFIO时钟。
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO,ENABLE);
	//2.GPIO初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO速度为50MHz
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	//3.设置EXTI线
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource4);//将EXIT线4连接到PE4
	EXTI_InitStructure.EXTI_Line = EXTI_Line4;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//上升下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能中断线
	EXTI_Init(&EXTI_InitStructure);//初始化中断
#if 0
	//4.中断向量
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
}

void IO_EXIT_RightWheelSpeed_DE(u8 enable_or_disable)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	if(enable_or_disable == ITENABLE)
	{
		NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

	}
	else if(enable_or_disable == ITDISABLE)
	{
		NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
		NVIC_Init(&NVIC_InitStructure);

	}	
}

void TIM3_SpeedCalculate_Init(void)//速度计算定时器初始化
{
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE); 
	
   /* 自动重装载寄存器周期的值(计数值) */
   TIM_TimeBaseStructure.TIM_Period=3999;  //0.5ms 2000hz
	
   /* 累计 TIM_Period个频率后产生一个更新或者中断 */
   TIM_TimeBaseStructure.TIM_Prescaler= 7;
	
   /* 对外部时钟进行采样的时钟分频,这里没有用到 */
   TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;	
   TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
   TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
   TIM_ClearFlag(TIM3, TIM_FLAG_Update);
   TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);		
   TIM_Cmd(TIM3, ENABLE);			
}

void TIM1_OdomUpdate_Init(void)//里程计发布定时器初始化
{
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
   TIM_DeInit(TIM1);
	

   /* 自动重装载寄存器周期的值(计数值) */ 
   TIM_TimeBaseStructure.TIM_Period=399;  //20HZ  500*100us=50ms 
   TIM_TimeBaseStructure.TIM_Prescaler=7999; 
	
	 /* 对外部时钟进行采样的时钟分频,这里没有用到 */
   TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;	
   TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
   TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
   TIM_ClearFlag(TIM1, TIM_FLAG_Update);
   TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);		
   TIM_Cmd(TIM1, ENABLE);	
}

void NVIC_Configuration(void)//优先级配置
{
    NVIC_InitTypeDef NVIC_InitStructure;	
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);//将中断矢量放到Flash的0地址
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);//设置优先级配置的模式,第1组:抢占优先级0(0:7),抢占优先级1(0:7),

    /****************************使能串口1中断，并设置优先级***********************/
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			   //USART1全局中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	   //先占优先级 1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	           //从占优先级 2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  		 	   //使能中断
    NVIC_Init(&NVIC_InitStructure);	     	
		  	
	
	NVIC_InitStructure.NVIC_IRQChannel =EXTI4_IRQn;                //(右)码盘外部中断函数
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel =EXTI15_10_IRQn;             //(左)码盘外部中断函数
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
                                                        
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	 
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}	

void USART1_IRQHandler(void)//串口中断函数
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //是否接受到数据
    {
		serial_rec =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
					
		if((USART_RX_STA&0x8000)==0)//接收未完成
        {
            if(USART_RX_STA&0x4000)//接收到了0x0d
            {
                if(serial_rec==0x0a)
                {
                    if((USART_RX_STA&0x3f)==8)
                    {							
                        USART_RX_STA|=0x8000;	//接收完成了 
                        main_sta|=0x04;
                        main_sta&=0xF7;
                    }
                    else
                    {
                        main_sta|=0x08;
                        main_sta&=0xFB;
                        USART_RX_STA=0;//接收错误,重新开始
                    }
                }
                else 
                {
                    main_sta|=0x08;
                    USART_RX_STA=0;//接收错误,重新开始
                }
            }
            else //还没收到0X0D
            {	
                if(serial_rec==0x0d)USART_RX_STA|=0x4000;
                else
                {
                    USART_RX_BUF[USART_RX_STA&0X3FFF]=serial_rec ;
                    USART_RX_STA++;
                    if(USART_RX_STA>(USART_REC_LEN-1))
                    {
                        main_sta|=0x08;
                        USART_RX_STA=0;//接收数据错误,重新开始接收
                    }							
                }		 
            }
        }   		 
    }
}


