#ifndef __GOLBAL_VARIABLE_H
#define __GOLBAL_VARIABLE_H


#include "includes.h"


#define Forward 1
#define Reverse 2
#define STOP 0

#define LEFT  1
#define RIGHT 2
#define ITENABLE 1
#define ITDISABLE 2


#define SPEED_SAMPLING_TIME  9    // (9+1)*500usec = 5MS   ,200hz
#define SPEED_BUFFER_SIZE 3       //左右轮速度缓存数组大小

#define SPEED_SAMPLING_FREQ (u16)(2000/(SPEED_SAMPLING_TIME+1))  //200hz，小车速度采样频率
#define ENCODER_L_PPR           (u16)(16)  // 左	电机码盘线数
#define ENCODER_R_PPR           (u16)(16)  // 右电机码盘线数


struct PID
{
    float Kp;
    float Ki;
    float Kd;
    float error_0;//基波分量
    float error_1;//一次谐波分量
    float error_2;//二次谐波分量
    long  Sum_error;
    float OutputValue;//实际输出量
    float OwenValue;//零误差时的标准输出量
};



extern u8 main_sta; //用作处理主函数各种if，去掉多余的flag（1打印里程计）（2调用计算里程计数据函数）（3串口接收成功）（4串口接收失败）


/***********************************************  状态  *****************************************************************/
extern u8 L_ForwardReverse ;// 0静止	1向前	2向后(左电机)
extern u8 R_ForwardReverse ;// 0静止	1向前	2向后(右电机)

/***********************************************  输出  *****************************************************************/

extern float position_x,position_y,oriention,velocity_linear,velocity_angular;
extern char odometry_data[21];   //发送给串口的里程计数据数组

/***********************************************  输入  *****************************************************************/
#define USART_REC_LEN  			200  	//定义最大接收字节数 200

extern u8 USART_RX_BUF[USART_REC_LEN];     //串口接收缓冲,最大USART_REC_LEN个字节.
extern u16 USART_RX_STA;                   //串口接收状态标记	

extern float RecRightWheelSpeed,RecLeftWheelSpeed;//串口得到的左右轮速度

/***********************************************  底盘参数  *****************************************************************/

//轴距校正值=原轴距/0.987
extern float wheel_interval; //校正后轴距值，单位mm       
extern float deceleration_ratio;  //减速比 1:52
extern float wheel_diameter;     //轮子直径，单位mm
extern float pi_1_2;			 //π/2
extern float pi;              //π
extern float pi_3_2;			 //π*3/2
extern float pi_2_1;			 //π*2
extern float dt;                 //采样时间间隔5ms
extern float line_number;       //码盘线数
extern float oriention_interval;  //dt时间内方向变化值

extern float sin_;        //角度计算值
extern float cos_;

extern float delta_distance,delta_oriention;   //采样时间间隔内运动的距离

extern float const_frame,const_angle,distance_sum,distance_diff;

extern float oriention_1;


extern int span;//采集回来的左右轮速度差值
extern s32 hSpeed_BufferL[SPEED_BUFFER_SIZE], hSpeed_BufferR[SPEED_BUFFER_SIZE];//左右轮速度缓存数组
extern  unsigned int hRot_Speed_L;//左电机平均转速缓存
extern  unsigned int hRot_Speed_R;//右电机平均转速缓存
extern unsigned int SpeedL; //左电机平均转速 r/min，PID调节
extern unsigned int SpeedR; //右电机平均转速 r/min，PID调节
extern  unsigned short int hSpeedMeas_Timebase_500us;//电机编码数采集时间间隔
extern float  Milemeter_L_Motor,Milemeter_R_Motor;//dt时间内的左右轮速度,用于里程计计算
extern u8 bSpeed_Buffer_Index;//缓存左右轮编码数到数组变量
extern float pulse_L;//左电机 PID调节后的PWM值缓存
extern float pulse_R;//右电机 PID调节后的PWM值缓存

extern u8 serial_rec;   //接收串口数据变量


extern s32 LeftWhellCount;//电机编码计数
extern s32 RightWhellCount;//电机编码计数
extern struct PID Control_left ;//左轮PID参数，适于新电机4096
extern struct PID Control_right;//右轮PID参数，适于新电机4096



#endif
