#include "odometry.h"

u8 once=1;

void TIM1_UP_IRQHandler(void)//里程计发布定时器中断函数
{
#ifdef DEBUG
		printf("enter Tim1 IRQ\r\n");
#endif 

	if( TIM_GetITStatus(TIM1 , TIM_IT_Update) != RESET ) 
	{	
		main_sta|=0x01;//执行发送里程计数据步骤
		
		TIM_ClearITPendingBit(TIM1 , TIM_FLAG_Update);//清除中断标志位  		 
	}		 
}

//里程计计算函数
void odometry(float right,float left) //dt时间内的左右轮速度,用于里程计计算
{	
	if(once)  //常数仅计算一次
	{
		const_frame=wheel_diameter*pi/(line_number*deceleration_ratio);//每个脉冲行走的距离(单位mm)
		const_angle=const_frame/wheel_interval;//每个脉冲行走的角度
		once=0;
	}
    
	distance_sum = 0.5f*(right+left);//在很短的时间内，小车行驶的路程为两轮速度和
	distance_diff = right-left;//在很短的时间内，小车行驶的角度为两轮速度差

    //根据左右轮的方向，纠正短时间内，小车行驶的路程和角度量的正负
	if((RecRightWheelSpeed>0)&&(RecLeftWheelSpeed>0))            //左右均正
	{
		delta_distance = distance_sum;
		delta_oriention = distance_diff;
	}
	else if((RecRightWheelSpeed<0)&&(RecLeftWheelSpeed<0))       //左右均负
	{
		delta_distance = -distance_sum;
		delta_oriention = -distance_diff;
	}
	else if((RecRightWheelSpeed<0)&&(RecLeftWheelSpeed>0))       //左正右负
	{
		delta_distance = -distance_diff;
		delta_oriention = -2.0f*distance_sum;		
	}
	else if((RecRightWheelSpeed>0)&&(RecLeftWheelSpeed<0))       //左负右正
	{
		delta_distance = distance_diff;
		delta_oriention = 2.0f*distance_sum;
	}
	else
	{
		delta_distance=0;
		delta_oriention=0;
	}
    
	oriention_interval = delta_oriention * const_angle;//采样时间内走的角度	
	oriention = oriention + oriention_interval;//计算出里程计方向角
	oriention_1 = oriention + 0.5f * oriention_interval;//里程计方向角数据位数变化，用于三角函数计算
	
    sin_ = sin(oriention_1);//计算出采样时间内y坐标
	cos_ = cos(oriention_1);//计算出采样时间内x坐标
	
    position_x = position_x + delta_distance * cos_ * const_frame;//计算出里程计x坐标
	position_y = position_y + delta_distance * sin_ * const_frame;//计算出里程计y坐标
    
	velocity_linear = delta_distance*const_frame / dt;//计算出里程计线速度
	velocity_angular = oriention_interval / dt;//计算出里程计角速度
	
    //方向角角度纠正
	if(oriention > pi)
	{
		oriention -= pi_2_1;
	}
	else
	{
		if(oriention < -pi)
		{
			oriention += pi_2_1;
		}
	}
}
