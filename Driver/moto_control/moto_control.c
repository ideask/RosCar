#include "moto_control.h"




void MotoControlTimInit(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef	TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	   //使能定时器4的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO, ENABLE);   //使能外设时钟使能 
	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);		 //TIM4选择全复用功能使能 

	//设置引脚为复用输出功能,输出TIM4 CH1、TIM4 CH2、TIM4 CH3、TIM4 CH4的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;		//TIM_CH1、TIM_CH2、TIM_CH3、TIM_CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		 //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	TIM_TimeBaseStructure.TIM_Period = arr;		//设置在下一个更新事件装入活动的自动重装载寄存器周期的值  80K
	TIM_TimeBaseStructure.TIM_Prescaler =psc;		  //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);	  //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位


	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;		//选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	 //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 100; 			//设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	  //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);		  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);	//CH1预装载使能

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;		//选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	 //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 100; 			//设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	  //输出极性:TIM输出比较极性高	
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);		  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);	//CH2预装载使能  

	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;		//选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	 //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 100; 			//设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	  //输出极性:TIM输出比较极性高
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);		  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);	//CH3预装载使能  


	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;		//选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	 //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 100; 			//设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	  //输出极性:TIM输出比较极性高	
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);		  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);	//CH4预装载使能  
	
	TIM_ARRPreloadConfig(TIM4, ENABLE);	   //使能TIMx在ARR上的预装载寄存器
	TIM_Cmd(TIM4, ENABLE); 		  //使能TIM3  

}

void LeftMotorStop(void)
{
	TIM4->CCR3 = 3200;
	TIM4->CCR4 = 3200;
}

void RightMotorStop(void)
{
	TIM4->CCR1 = 3200;
	TIM4->CCR2 = 3200;
}

void ENC_Calc_Average_Speed(void)//计算三次电机的平均编码数
{   
    u32 i;
	signed long long wtempLL=0;
	signed long long wtempRR=0;

    //累加缓存次数内的速度值
	for (i=0;i<SPEED_BUFFER_SIZE;i++)
	{
		wtempLL += hSpeed_BufferL[i];
		wtempRR += hSpeed_BufferR[i];
	}
    
    //取平均
	wtempLL /= (SPEED_BUFFER_SIZE);
	wtempRR /= (SPEED_BUFFER_SIZE);	
    
    //将平均脉冲数单位转为 r/min
	wtempLL = (wtempLL * SPEED_SAMPLING_FREQ)*60/(ENCODER_L_PPR);
	wtempRR = (wtempRR * SPEED_SAMPLING_FREQ)*60/(ENCODER_R_PPR); 

	if(wtempLL < 0){wtempLL = 0 - wtempLL;}//转化为正数
	if(wtempRR < 0){wtempRR = 0 - wtempRR;}//转化为正数
	
	hRot_Speed_L= ((s16)(wtempLL));//平均转速 r/min
	hRot_Speed_R= ((s16)(wtempRR));//平均转速 r/min
	SpeedL=hRot_Speed_L;//平均转速 r/min
	SpeedR=hRot_Speed_R;//平均转速 r/min
#ifdef DEBUG
	printf("ENC_Calc_Average_Speed() SpeedL:%d,SpeedR:%d \r\n",SpeedL,SpeedR);
#endif 

}

float MaxValue=9;//输出最大限幅
float MinValue=-9;//输出最小限幅
float OutputValue;//PID输出暂存变量,用于积分饱和抑制


float PID_calculate(struct PID *Control,float CurrentValue_left )//位置PID计算B
{
	
	float Value_Kp;//比例分量
	float Value_Ki;//积分分量
	float Value_Kd;//微分分量
	
	Control->error_0 = Control->OwenValue - CurrentValue_left + 0*span;//基波分量，Control->OwenValue为想要的速度，CurrentValue_left为电机真实速度
	Value_Kp = Control->Kp * Control->error_0 ;
	Control->Sum_error += Control->error_0;     
	
    /***********************积分饱和抑制********************************************/
    OutputValue = Control->OutputValue;
    if(OutputValue>5 || OutputValue<-5)	
    {
        Control->Ki = 0; 
    }
    /*******************************************************************/
	
	Value_Ki = Control->Ki * Control->Sum_error;
	Value_Kd = Control->Kd * ( Control->error_0 - Control->error_1);
	Control->error_1 = Control->error_0;//保存一次谐波
	Control->OutputValue = Value_Kp  + Value_Ki + Value_Kd;//输出值计算，注意加减
	
    //限幅
	if( Control->OutputValue > MaxValue)
		Control->OutputValue = MaxValue;
	if (Control->OutputValue < MinValue)
		Control->OutputValue = MinValue;
    
	return (Control->OutputValue) ;
}


void Gain_R(void)//设置右电机 PID调节
{
    
	span=1*(SpeedL-SpeedR);//采集回来的左右轮速度差值
	pulse_R= pulse_R + PID_calculate(&Control_right,hRot_Speed_R);//PID调节
    
    //pwm幅度抑制
	if(pulse_R > 3200) pulse_R = 3200;
	if(pulse_R < 0) pulse_R = 0;
}


void Gain_L(void)//设置左电机 PID调节 
{
    float pulse_L_IN1,pulse_L_IN2,pulse_R_IN1,pulse_R_IN2;
	span=1*(SpeedR-SpeedL);//采集回来的左右轮速度差值
	pulse_L= pulse_L + PID_calculate(&Control_left,hRot_Speed_L);//PID调节
    
    ////pwm 幅度抑制
	if(pulse_L > 3200) pulse_L = 3200;
	if(pulse_L < 0) pulse_L = 0;
	
	if(L_ForwardReverse == Forward)
	{
		pulse_L_IN2 = 0;
		pulse_L_IN1 = pulse_L;	
	}
	else if(L_ForwardReverse == Reverse)
	{
		pulse_L_IN2 = pulse_L;
		pulse_L_IN1 = 0;		
	}

	if(R_ForwardReverse == Forward)
	{
		pulse_R_IN1 = 0;
		pulse_R_IN2 = pulse_R;
	}
	else if(R_ForwardReverse == Reverse)
	{
		pulse_R_IN1 = pulse_R;
		pulse_R_IN2 = 0;	
	}

	TIM4->CCR1 = pulse_R_IN2;//右电机赋值PWM
    TIM4->CCR2 = pulse_R_IN1;//右电机赋值PWM
	TIM4->CCR3 = pulse_L_IN1;//左电机赋值PWM
    TIM4->CCR4 = pulse_L_IN2;//左电机赋值PWM    
#ifdef DEBUG
	printf("TIM4->CCR1:%f\r\n",pulse_R_IN2);
	printf("TIM4->CCR2:%f\r\n",pulse_R_IN1);
	printf("TIM4->CCR3:%f\r\n",pulse_L_IN1);
	printf("TIM4->CCR4:%f\r\n",pulse_L_IN2);
#endif

}


void TIM3_IRQHandler(void)//小车速度计算定时器中断函数
{
#ifdef DEBUG
	printf("enter Tim3 IRQ\r\n");
#endif 
	if ( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET ) 
	{						      
        if (hSpeedMeas_Timebase_500us !=0)//电机编码数采集时间间隔未到
        {
            hSpeedMeas_Timebase_500us--;//开始倒数	
        }
        else    //电机编码数采集时间间隔到了
        {
            s32 wtempR,wtempL;
            
            hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;//恢复电机编码数采集时间间隔
            
            /************************ 1 ***************************/
            
            wtempL = readEncoder(LEFT); //获取左轮子的编码数
            wtempR = readEncoder(RIGHT); //获取右轮子的编码数
            
//            //如果为停止指令，即左右轮速度为零，则清除速度存储器防止前后速度差太大造成小车冲转
//            if((wtempR == 0) && (wtempL == 0))
//            {
//                pulse=pulse1=0;
//            }
             
            /************************ 2 ***************************/
            
            //储存编码数（脉冲数），用于里程计计算
            Milemeter_L_Motor= (float)wtempL; //储存脉冲数
            Milemeter_R_Motor= (float)wtempR;
            
            main_sta|=0x02;//执行计算里程计数据步骤

            /************************ 3 ***************************/
            
            //开始缓存左右轮编码数到数组
            hSpeed_BufferL[bSpeed_Buffer_Index] = wtempL;
            hSpeed_BufferR[bSpeed_Buffer_Index] = wtempR;
            bSpeed_Buffer_Index++;//数组移位
            
            //缓存左右轮编码数到数组结束判断
            if(bSpeed_Buffer_Index >=SPEED_BUFFER_SIZE)
            {
                bSpeed_Buffer_Index=0;//缓存左右轮编码数到数组变量清零
            }
            
            /************************ 4 ***************************/
            
            ENC_Calc_Average_Speed();//计算三次电机的平均编码数
            Gain_R(); //电机A转速PID调节控制 右
            Gain_L(); //电机B转速PID调节控制 左
        }
        
		TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);//清除中断标志位    		 
	}		 
}

void LeftMovingSpeedW(unsigned int val)//左轮方向和速度控制函数
{     
    if(val>10000)
    {  
    	/* 左motor  正转*/
		L_ForwardReverse = Forward;
        Control_left.OwenValue=(val-10000);//PID调节的目标编码数			
    }
    else if(val<10000)
    {  
    	/* 左motor  反转*/
		L_ForwardReverse = Reverse;
        Control_left.OwenValue=(10000-val);//PID调节的目标编码数	 
    }	
    else
    {
		 L_ForwardReverse = STOP;
         Control_left.OwenValue=0;//PID调节的目标编码数
    }	
#ifdef 0
		printf("LeftMovingSpeedW() L_ForwardReverse %d \r\n",L_ForwardReverse);
		printf("LeftMovingSpeedW() Control_left.OwenValue %f \r\n",Control_left.OwenValue);
#endif  

}

void RightMovingSpeedW(unsigned int val2)//右轮方向和速度控制函数
{    
    if(val2>10000)
    {  
        /* 右motor  正转*/
		R_ForwardReverse = Forward;
        Control_right.OwenValue=(val2-10000);//PID调节的目标编码数
    }
    else if(val2<10000)
    {  
        /* 右motor  反转*/
		R_ForwardReverse = Reverse;
        Control_right.OwenValue=(10000-val2);//PID调节的目标编码数	       
    }	
    else
    {
        R_ForwardReverse = STOP;
        Control_right.OwenValue=0;//PID调节的目标编码数
    }	
#ifdef 0
	printf("RightMovingSpeedW() R_ForwardReverse %d \r\n",R_ForwardReverse);
	printf("RightMovingSpeedW() Control_right.OwenValue %f \r\n",Control_right.OwenValue);
#endif  
}

void car_control(float rightspeed,float leftspeed)//小车速度转化和控制函数
{
    float k2=14.1947;      //将车轮速度转换成电机转速 mm/s转换成 r/min              	公式=60/pi/轮子直径*减速比
    
    int right_speed=(int)k2*rightspeed;
    int left_speed=(int)k2*leftspeed;
    RightMovingSpeedW(right_speed+10000);//10000的值确定是由机器人最大行驶速度决定，假设最大行驶速度为1000mm/s,那么转速是17179,此处将Int转成unsigned int
    LeftMovingSpeedW(left_speed+10000);
}

